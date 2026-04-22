"""
camera_stream.py
----------------
Starts a lightweight MJPEG HTTP stream from Picamera2.

Default endpoints:
- http://<pi-ip>:8000/
- http://<pi-ip>:8000/stream.mjpg
"""

import socket
import threading
from http import server
from socketserver import ThreadingMixIn
from time import sleep

try:
    import cv2
except Exception:
    cv2 = None

try:
    from picamera2 import Picamera2
except Exception:
    Picamera2 = None

STREAM_RES = (640, 480)
STREAM_FPS = 30
STREAM_HOST = "0.0.0.0"
STREAM_PORT = 8000
JPEG_QUALITY = 80

_running = False
_capture_thread = None
_http_thread = None
_picam2 = None
_httpd = None

_frame_lock = threading.Lock()
_latest_jpeg = None


class _ThreadingHTTPServer(ThreadingMixIn, server.HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


class _CameraController:
    """Compatibility object so main.py can call picam2.stop_recording()."""

    def stop_recording(self):
        stop_camera_stream()


class _StreamHandler(server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self._serve_index()
            return
        if self.path == "/stream.mjpg":
            self._serve_stream()
            return

        self.send_response(404)
        self.end_headers()

    def _serve_index(self):
        page = (
            "<html><head><title>Camera Stream</title></head>"
            "<body style='margin:0;background:#111;color:#eee;'>"
            "<img src='/stream.mjpg' style='width:100%;height:auto;display:block;' />"
            "</body></html>"
        ).encode("utf-8")

        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(page)))
        self.end_headers()
        self.wfile.write(page)

    def _serve_stream(self):
        self.send_response(200)
        self.send_header("Age", "0")
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Pragma", "no-cache")
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=FRAME")
        self.end_headers()

        print(f">> Stream client connected from {self.client_address[0]}")
        frame_sent_count = 0

        while _running:
            with _frame_lock:
                frame = _latest_jpeg

            if frame is None:
                sleep(0.03)
                continue

            try:
                self.wfile.write(b"--FRAME\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode())
                self.wfile.write(frame)
                self.wfile.write(b"\r\n")
                frame_sent_count += 1
            except (BrokenPipeError, ConnectionResetError):
                print(f">> Stream client disconnected after {frame_sent_count} frames")
                break
            except Exception as exc:
                print(f">> Stream error: {exc}")
                break

    def log_message(self, fmt, *args):
        # Suppress default per-request logging noise.
        return


def _get_lan_ip():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect(("8.8.8.8", 80))
        return sock.getsockname()[0]
    except Exception:
        return "127.0.0.1"
    finally:
        sock.close()


def _capture_loop(resolution, fps):
    global _running, _picam2, _latest_jpeg

    if Picamera2 is None or cv2 is None:
        print(">> Camera dependencies missing (picamera2 and/or cv2). Stream disabled.")
        _running = False
        return

    try:
        print(f">> Initializing Picamera2 at {resolution} @ {fps} FPS...")
        _picam2 = Picamera2()
        config = _picam2.create_video_configuration(
            main={"size": resolution, "format": "RGB888"},
            controls={"FrameRate": fps},
        )
        _picam2.configure(config)
        _picam2.start()
        print(">> ✓ Picamera2 started successfully. Capturing frames...")

        encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]
        frame_count = 0

        while _running:
            frame = _picam2.capture_array()
            # Rotate frame 180 degrees to correct inverted image
            frame = cv2.rotate(frame, cv2.ROTATE_180)
            ok, jpeg = cv2.imencode(".jpg", frame, encode_params)
            if not ok:
                continue

            with _frame_lock:
                _latest_jpeg = jpeg.tobytes()
            
            frame_count += 1
            if frame_count % 100 == 0:
                print(f">> Camera: {frame_count} frames captured")
    except Exception as exc:
        print(f"❌ Camera capture error: {exc}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            if _picam2 is not None:
                _picam2.stop()
        except Exception:
            pass


def _http_loop(host, port):
    global _httpd

    try:
        print(f">> Starting HTTP server on {host}:{port}...")
        _httpd = _ThreadingHTTPServer((host, port), _StreamHandler)
        print(f">> ✓ HTTP server ready. Listening on :{port}")
        _httpd.serve_forever(poll_interval=0.2)
    except Exception as exc:
        print(f"❌ Camera HTTP server error: {exc}")
        import traceback
        traceback.print_exc()


def start_camera_stream(
    resolution=STREAM_RES,
    fps=STREAM_FPS,
    host=STREAM_HOST,
    port=STREAM_PORT,
):
    """Start non-blocking MJPEG stream and return a controller with stop_recording()."""
    global _running, _capture_thread, _http_thread, _latest_jpeg

    if _running:
        return _CameraController()

    if Picamera2 is None:
        print("❌ ERROR: picamera2 is not installed. Camera stream NOT started.")
        print("   Install with: pip install picamera2")
        return _CameraController()
    if cv2 is None:
        print("❌ ERROR: cv2 (OpenCV) is not installed. Camera stream NOT started.")
        print("   Install with: pip install opencv-python")
        return _CameraController()

    _running = True
    _latest_jpeg = None

    _capture_thread = threading.Thread(
        target=_capture_loop,
        args=(resolution, fps),
        daemon=True,
    )
    _capture_thread.start()

    _http_thread = threading.Thread(
        target=_http_loop,
        args=(host, port),
        daemon=True,
    )
    _http_thread.start()

    lan_ip = _get_lan_ip()
    print(">> Camera stream online:")
    print(f"   http://{lan_ip}:{port}/")
    print(f"   http://{lan_ip}:{port}/stream.mjpg")

    return _CameraController()


def stop_camera_stream():
    """Stop capture and HTTP server."""
    global _running, _httpd

    _running = False

    if _httpd is not None:
        try:
            _httpd.shutdown()
            _httpd.server_close()
        except Exception:
            pass
        _httpd = None

    if _capture_thread is not None:
        _capture_thread.join(timeout=2)
    if _http_thread is not None:
        _http_thread.join(timeout=2)

    print(">> Camera stream stopped.")
