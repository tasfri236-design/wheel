"""
camera_stream.py
----------------
Picamera2 capture loop with optional MJPEG HTTP stream.

Two entry points:
- ``start_camera_stream(...)``  -> capture loop + standalone HTTP server
  (used by ``camera_viewer.py``).
- ``start_capture_only(...)``   -> capture loop only; FastAPI serves the MJPEG.

In both cases the latest frame is exposed via:
- ``get_latest_bgr_frame()`` -> numpy.ndarray (HxWx3, BGR-ordered) or None.
- ``get_latest_jpeg()`` -> bytes or None.

The Picamera2 ``RGB888`` format actually stores pixels in BGR order in the
numpy buffer, which is why no ``cvtColor`` is needed before handing frames to
OpenCV. See follow_ball.py for the same pattern.
"""

import socket
import threading
from http import server
from socketserver import ThreadingMixIn
from time import sleep
from typing import Optional

try:
    import cv2
except Exception:
    cv2 = None

try:
    import numpy as np
except Exception:
    np = None

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
_latest_bgr = None  # raw BGR ndarray, used by in-process consumers (follow loop)


def is_available() -> bool:
    """True when the picamera2/cv2 stack can be imported on this host."""
    return Picamera2 is not None and cv2 is not None


def is_running() -> bool:
    return _running


def get_latest_jpeg() -> Optional[bytes]:
    """Latest captured frame as JPEG bytes (or None if no frames yet)."""
    with _frame_lock:
        return _latest_jpeg


def get_latest_bgr_frame():
    """Latest captured frame as a BGR-ordered numpy array (or None)."""
    with _frame_lock:
        frame = _latest_bgr
    if frame is None:
        return None
    # Return a copy so callers can mutate without racing the capture loop.
    return frame.copy()


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
    global _running, _picam2, _latest_jpeg, _latest_bgr

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
        print(">> Picamera2 started successfully. Capturing frames...")

        encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]
        frame_count = 0

        while _running:
            frame = _picam2.capture_array()
            # Rotate 180 to correct upside-down mount; this matches the
            # orientation that the rest of the codebase has been tuned for.
            frame = cv2.rotate(frame, cv2.ROTATE_180)
            ok, jpeg = cv2.imencode(".jpg", frame, encode_params)
            if not ok:
                continue

            with _frame_lock:
                _latest_bgr = frame
                _latest_jpeg = jpeg.tobytes()

            frame_count += 1
            if frame_count % 100 == 0:
                print(f">> Camera: {frame_count} frames captured")
    except Exception as exc:
        print(f"Camera capture error: {exc}")
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


def _start_capture(resolution, fps) -> bool:
    """Internal: launch the capture thread if not already running. Returns True on success."""
    global _running, _capture_thread, _latest_jpeg, _latest_bgr

    if _running:
        return True

    if Picamera2 is None:
        print("ERROR: picamera2 is not installed. Camera capture NOT started.")
        print("   Install with: sudo apt install -y python3-picamera2")
        return False
    if cv2 is None:
        print("ERROR: cv2 (OpenCV) is not installed. Camera capture NOT started.")
        print("   Install with: pip install opencv-python-headless")
        return False

    _running = True
    with _frame_lock:
        _latest_jpeg = None
        _latest_bgr = None

    _capture_thread = threading.Thread(
        target=_capture_loop,
        args=(resolution, fps),
        daemon=True,
    )
    _capture_thread.start()
    return True


def start_capture_only(resolution=STREAM_RES, fps=STREAM_FPS) -> bool:
    """Start the Picamera2 capture loop without an HTTP server.

    Used when something else (e.g. FastAPI) will serve frames itself via
    ``get_latest_jpeg()`` / ``get_latest_bgr_frame()``. Idempotent.
    """
    return _start_capture(resolution, fps)


def start_camera_stream(
    resolution=STREAM_RES,
    fps=STREAM_FPS,
    host=STREAM_HOST,
    port=STREAM_PORT,
):
    """Start non-blocking MJPEG stream and return a controller with stop_recording()."""
    global _http_thread

    if not _start_capture(resolution, fps):
        return _CameraController()

    if _http_thread is None or not _http_thread.is_alive():
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
    global _running, _httpd, _latest_jpeg, _latest_bgr

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

    with _frame_lock:
        _latest_jpeg = None
        _latest_bgr = None

    print(">> Camera stream stopped.")
