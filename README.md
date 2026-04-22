# Reaction Wheel Telemetry

This repo supports **two runnable paths**. Both are fully functional; pick the one that matches your clone state.

| Path | What you get | Command | Requires |
|------|--------------|---------|----------|
| **A - Legacy CLI** | Terminal-only motor control on real hardware (`100 cw`, `brake`, …). No HTTP. | `python main.py` | `main.py` at repo root. **Works on a bare clone without `app/`.** |
| **B - FastAPI + web dashboard** | REST + WebSocket API and the React UI. | `python -m uvicorn app.main:app ...` | The **`app/`** package must be present in the clone. If it is not, see [If `app/` is missing from your clone](#if-app-is-missing-from-your-clone). |

Path B exposes:

- `GET /api/health` — IMU / mock status  
- `POST /api/command` — motor commands (JSON)  
- `WebSocket /ws/telemetry` — live telemetry (~20 Hz)  
- Static UI from `web/dist/` when present  

Hardware modes for Path B:

- **Raspberry Pi:** real GPIO, encoder, BNO085 (unless `MOCK_TELEMETRY=1`)  
- **Mock:** `MOCK_TELEMETRY=1` — synthetic encoder + IMU for dev without a Pi  

## Repository layout

- `main.py` — **Path A** legacy CLI (stdin: `100 cw`, `brake`, …); standalone, no HTTP  
- `requirements.txt` — Python deps (FastAPI, Uvicorn, Pydantic, Pi hardware libs)  
- `web/` — React + Vite frontend (`npm run build` → `web/dist/`)  
- `app/` — **Path B** FastAPI backend (may be missing on fresh clones of some branches)  
  - `app/main.py` — FastAPI app, routes, WebSocket, static UI mount  
  - `app/motor_loop.py` — motor control loop  
  - `app/hardware.py` — `PiHardware`, `MockHardware`, `create_hardware()`  
  - `app/state.py` — shared state, stabilize tuning env vars  

## Check what you have

From the folder you cloned into:

```bash
cd /path/to/wheel
ls -la
```

You should see `main.py`, `requirements.txt`, and `web/`. If you also see **`app/`**, Path B is available. If not, follow the next section to recover it, or use Path A only.

## If `app/` is missing from your clone

Symptoms:

- `ls app` prints `No such file or directory`.  
- `python -m uvicorn app.main:app ...` fails with `ModuleNotFoundError: No module named 'app'`.

Recovery options (pick one):

1. **Checkout a branch or commit that contains `app/`**:

   ```bash
   cd /path/to/wheel
   git fetch --all
   git branch -a                     # list local + remote branches
   git checkout <branch-with-app>    # e.g. a feature branch that introduced app/
   ls app                            # verify
   ```

2. **Copy `app/` from another machine that has it** (e.g. your dev laptop):

   ```bash
   # on the Pi, from the repo root:
   scp -r user@dev-host:/path/to/wheel/app ./
   ls app
   ```

3. **Push `app/` from the machine that has it** so future clones include it:

   ```bash
   # on the machine that already has app/:
   cd /path/to/wheel
   git add app
   git commit -m "add app package (FastAPI backend)"
   git push origin main
   # then on the Pi:
   cd /path/to/wheel && git pull --ff-only
   ```

Verify after recovery:

```bash
cd /path/to/wheel
ls app main.py requirements.txt
python -m uvicorn app.main:app --host 127.0.0.1 --port 8000
# server should start and log "Application startup complete."
```

If you never plan to run the dashboard, skip this and use **Path A** below.

## Install (Python)

Create and activate a virtual environment first:

```bash
cd /path/to/wheel
python3 -m venv .venv
source .venv/bin/activate          # Windows: .venv\Scripts\activate
```

Then install the deps that match your path.

### Path A (CLI only)

The legacy `main.py` needs only the Raspberry Pi hardware libs from the bottom of [requirements.txt](requirements.txt):

```bash
pip install "Adafruit-Blinka>=8" "adafruit-circuitpython-bno08x>=1.3.0" "gpiozero>=2.0"
```

(FastAPI, Uvicorn, and Pydantic are **not** required for Path A.)

### Path B (full server)

From repo root, with `app/` present:

```bash
pip install -r requirements.txt
```

This installs FastAPI + Uvicorn + Pydantic **and** the Pi hardware libs. On a Mac/PC without a Pi you can still install everything and run in mock mode; `gpiozero` / Blinka are imported lazily only when real hardware is selected.

## Install Node.js and npm (only for Path B web UI)

On **Raspberry Pi OS** (if `npm` is missing):

```bash
curl -fsSL https://deb.nodesource.com/setup_20.x | sudo -E bash -
sudo apt-get install -y nodejs
node -v && npm -v
```

Build the dashboard once (or after UI changes):

```bash
cd /path/to/wheel/web
npm install
npm run build
cd ..
```

API-only: you can skip Node. Opening `http://<host>:8000/` will return JSON with build instructions until `web/dist/` exists.

## Run — Path A: Legacy CLI (hardware, no web)

Works on any clone that has `main.py`, including ones **without `app/`**.

```bash
cd /path/to/wheel
source .venv/bin/activate
python main.py
```

- Drives the motor directly via GPIO using the pins in [main.py](main.py) (same wiring as Path B — see **Raspberry Pi prerequisites**).  
- Prompts on stdin: `100 cw`, `50 ccw`, `brake`, `Ctrl+C` to stop.  
- **Does not** open any HTTP port. No WebSocket, no web UI.  

## Run — Path B: FastAPI on Raspberry Pi (real hardware)

Preconditions:

- `app/` present in repo (see [If `app/` is missing from your clone](#if-app-is-missing-from-your-clone)).
- I2C enabled, user has GPIO/I2C permissions (see **Raspberry Pi prerequisites**).
- Venv activated, `pip install -r requirements.txt` done.

```bash
cd /path/to/wheel
source .venv/bin/activate
python -m uvicorn app.main:app --host 0.0.0.0 --port 8000
```

This is **real hardware mode**: do **not** set `MOCK_TELEMETRY`. The process uses GPIO, the quadrature encoder, and the BNO085 over I2C.

Verify:

```bash
curl http://127.0.0.1:8000/api/health
# expected: {"ok":true,"imu_connected":true,"mock":false}
```

Open **`http://<pi-ip>:8000`** from another machine on the same network.

- Use **`0.0.0.0`** so other devices can reach the Pi. **`127.0.0.1`** only accepts connections on the Pi itself.  
- The motor thread starts with the server; `Ctrl+C` stops both.

If hardware initialization fails at startup, fix wiring/I2C/permissions or temporarily use mock mode below while debugging.

### Path B mock (no Pi hardware)

```bash
cd /path/to/wheel
source .venv/bin/activate
MOCK_TELEMETRY=1 python -m uvicorn app.main:app --host 127.0.0.1 --port 8000
```

Or from `web/` (parent = repo root):

```bash
cd /path/to/wheel/web
npm run api:mock
```

For real hardware launched from `web/` (binds `127.0.0.1` — fine for a browser on the same Pi; for LAN use the full uvicorn line with `--host 0.0.0.0`):

```bash
npm run api
```

## Run — Path B frontend dev (Vite + API)

Requires `app/` and the Path B API running.

**Terminal 1** (API; mock shown — drop `MOCK_TELEMETRY=1` on the Pi with hardware):

```bash
cd /path/to/wheel
source .venv/bin/activate
MOCK_TELEMETRY=1 python -m uvicorn app.main:app --host 127.0.0.1 --port 8000
```

**Terminal 2** (Vite):

```bash
cd /path/to/wheel/web
npm install
npm run dev
```

Open **`http://127.0.0.1:5173`**. The app talks to **`http://127.0.0.1:8000`** in dev (CORS is allowed). To use another API URL, set `VITE_API_ORIGIN` in `web/.env.development`.

Note: `web/` on its own does nothing — it is a UI shell that streams telemetry and sends commands via the Path B API. Without a running FastAPI server, the dashboard will show “Reconnecting…”.

## Camera ball-follow (PID)

Standalone OpenCV script [follow_ball.py](follow_ball.py) that tracks an **optic-yellow tennis ball**, maps pixel error to a PID command, and drives the reaction wheel through the **Path B** FastAPI `POST /api/command` endpoint. No changes to `app/`; the vision loop runs as its own process.

Prereqs:

- Camera plugged in: USB webcam on `/dev/video0` (default) **or** the Pi Camera on CSI with `--csi`.
- Path B server already running (real hardware or `MOCK_TELEMETRY=1`) and reachable at `--api`.
- Extra Python deps installed (already in [requirements.txt](requirements.txt)):

  ```bash
  pip install -r requirements.txt
  # On Pi OS you can replace the opencv wheel with:
  sudo apt install -y python3-opencv
  ```

### The command

From repo root, with venv activated and the API up:

```bash
python follow_ball.py --api http://127.0.0.1:8000
```

Useful variants:

```bash
python follow_ball.py --show           # preview window + mask
python follow_ball.py --calibrate      # live HSV trackbars
python follow_ball.py --dry-run        # log commands, do not POST
python follow_ball.py --invert         # swap cw/ccw if your rig is mirrored
python follow_ball.py --csi            # use Picamera2 instead of USB
python follow_ball.py --src 1          # pick a different USB camera index
```

### What it does

- Resizes each frame to `--width 400` for speed.
- `GaussianBlur (11,11)` → HSV `inRange` with optic-yellow bounds → `erode x2` → `dilate x2`.
- Largest contour → `minEnclosingCircle` → `(cx, cy, radius)`; rejects blobs smaller than `--min-radius 8`.
- Error = `cx - width/2`. If `|error| <= dead-zone * width` (default 5%) or the ball is close (`radius >= --stop-radius`), it sends `{"action":"brake"}`.
- Otherwise runs a PID and POSTs `{"target_rpm": …, "direction": "cw"|"ccw"}` with `target_rpm` clamped to `MAX_ATTAINABLE_RPM` (100). Commands are rate-limited to 20 Hz.
- Loses ball → brake + reset PID; no ball → no motion.

### Tuning crib sheet

| Flag | Default | Meaning |
|------|---------|---------|
| `--hsv-lower "H,S,V"` | `25,80,80` | Optic-yellow lower bound. Indoor warm light: drop H to ~22. |
| `--hsv-upper "H,S,V"` | `45,255,255` | Upper bound. Outdoor sun: drop S/V floors. |
| `--width` | 400 | Frame width after resize. Lower = faster on Pi. |
| `--dead-zone` | 0.05 | Fraction of width treated as "centered" (brake). Raise to 0.07 if motor twitches. |
| `--stop-radius` | 80 | Braking radius. Calibrate by holding the ball where you want it to stop, note the printed radius, set this to that value. Hysteresis re-enables tracking once radius falls below `0.85 × stop-radius`. |
| `--kp / --ki / --kd` | 0.08 / 0.0 / 0.02 | PID gains on pixel error. Intentionally small so commands react gently; raise Kp if it lags behind the ball. |
| `--max-rpm` | 60 | Upper bound on `|target_rpm|` in each command. Lower if the platform oscillates. |
| `--invert` | off | Flip cw/ccw to match your wheel's torque direction. |

### Troubleshooting

| Problem | Fix |
|---------|-----|
| `cv2.error: (-215:Assertion failed) !_src.empty()` | Camera not opened. Check `/dev/video0` (USB) or add `--csi`. Try `--src 1` for a second camera. |
| `can't open camera by index` + `OpenCV should be configured with libavdevice` on a Pi | Pi Camera on CSI isn't exposed as `/dev/video0` to OpenCV. Re-run with `--csi` (requires `sudo apt install -y python3-picamera2`). |
| `camera opened but returned no frames` | Device claimed but produced nothing. Check `ls /dev/video*`, try a different `--src`, or use `--csi` for the CSI cam. |
| `ERROR: cannot reach FastAPI at http://127.0.0.1:8000` | Start the Path B server in a second terminal first (see **Run — Path B**). `curl /api/health` should succeed before you launch the tracker. |
| Ball flickers in and out of the mask | Widen HSV (drop S/V lower bounds), or raise erode/dilate iterations (edit `BallTracker.detect`). Use `--calibrate`. |
| Motor twitches around centered ball | Raise `--dead-zone` to `0.07`, or lower `--kp`. |
| FPS below 10 on the Pi | Drop `--width` to 320, omit `--show`, install OpenCV via `apt`, or keep `cv2.setNumThreads(2)` (already on). |
| Commands arrive but wheel does not yaw toward ball | Add `--invert` (wheel torque sign is rig-dependent). |
| `Connection refused` / timeouts from `/api/command` | Start Path B server first (see **Run — Path B**), or pass `--api http://<pi-ip>:8000`. |

## Raspberry Pi prerequisites (hardware mode, both paths)

1. Raspberry Pi OS, Python 3.10+.  
2. **I2C enabled:** `sudo raspi-config` → Interface Options → I2C → Enable.  
3. User in `gpio` / `i2c` groups or run with permissions that can access GPIO and I2C.  
4. Wiring (same for Path A `main.py` and Path B `app/hardware.py`):

   - Motor PWM: **GPIO 12** (RPWM), **GPIO 13** (LPWM)  
   - Enables: **GPIO 24** (R_EN), **GPIO 25** (L_EN)  
   - Encoder: **GPIO 22** (A), **GPIO 23** (B)  
   - IMU: **BNO085** on I2C  

## API quick reference (Path B)

- `GET /api/health` — `ok`, `imu_connected`, `mock`  
- `POST /api/command` — e.g. `{"action":"brake"}`, `{"action":"stabilize"}`, `{"target_rpm":50,"direction":"cw"}`  
- `WS /ws/telemetry` — JSON fields: `t`, `motor_rpm`, `wheel_rad_s`, `imu_ok`, `duty_cycle`, `target_rpm`, `direction`, `stabilize_active`, `stabilize_pid_u`  

## Stabilize tuning (Path B, optional)

Environment variables (defaults in `app/state.py`):  
`STABILIZE_THRESHOLD_RAD_S`, `STABILIZE_DEBOUNCE_COUNT`, `STABILIZE_KP`, `STABILIZE_KI`, `STABILIZE_KD`, `STABILIZE_TORQUE_SIGN`, `STABILIZE_INTEGRAL_MAX`.

Example:

```bash
STABILIZE_KP=10 STABILIZE_KI=1.5 STABILIZE_KD=0.05 \
  python -m uvicorn app.main:app --host 0.0.0.0 --port 8000
```

## Troubleshooting

- **Clone has no `app/`** → See [If `app/` is missing from your clone](#if-app-is-missing-from-your-clone). Until it exists, only Path A (`python main.py`) will work.

### `ModuleNotFoundError: No module named 'app'` (wrong folder or incomplete clone)

Python only finds the `app` package if your **current working directory** is on `sys.path` and the **`app/` directory** sits next to you as a top-level package.

Typical mistakes:

- Nested path such as `~/Desktop/wheel/wheel` while the real project root is **`~/Desktop/wheel`** (one level up).
- Clone of a branch that never contained `app/`.

**Fix:**

1. Go to the directory that **contains** the `app` folder:

   ```bash
   cd ~/Desktop/wheel          # adjust if your clone lives elsewhere
   ls -la app main.py          # both must exist; if `ls app` fails, see the recovery section above
   ```

2. Run Uvicorn from **that** directory:

   ```bash
   python -m uvicorn app.main:app --host 0.0.0.0 --port 8000
   ```

3. If you must stay inside a subdirectory (e.g. you are in `wheel/wheel` and `app` is in the parent), point Uvicorn at the parent:

   ```bash
   python -m uvicorn app.main:app --app-dir .. --host 127.0.0.1 --port 8000
   ```

4. If `ls app` fails even from what you think is the repo root, your tree is **incomplete**. Use [If `app/` is missing from your clone](#if-app-is-missing-from-your-clone) to recover, or fall back to Path A.

### `Import string "app.main" must be in format "<module>:<attribute>"`

You ran something like `python -m uvicorn app.main --host …` without the **`:<attribute>`** part.

**Fix:** Uvicorn needs **both** the module and the ASGI application object name. This project’s FastAPI instance is the variable **`app`** inside `app/main.py`, so use:

```bash
python -m uvicorn app.main:app --host 0.0.0.0 --port 8000
```

Not `app.main` alone, and not `app.main:main` (there is no `main` ASGI object).

### Quick reference (errors → correct command)

| Error or symptom | Cause | Fix |
|------------------|--------|-----|
| `ModuleNotFoundError: No module named 'app'` | Wrong `cd`, or `app/` not in clone | `cd` to repo root; `ls app`; if truly missing, see the recovery section; or use `--app-dir ..` from a subfolder |
| `Import string "app.main" must be in format "<module>:<attribute>"` | Missing `:app` | Use `app.main:app` |
| Same `No module named 'app'` after using `app.main:main` | Wrong attribute name | Use **`app.main:app`**, not `:main` |

### Other issues

| Problem | What to do |
|---------|------------|
| `npm: command not found` | Install Node.js/npm (see **Install Node.js** above). |
| Browser on another PC cannot open `:8000` | Bind `--host 0.0.0.0`, not only `127.0.0.1`. |
| Root URL shows JSON “Web UI not built” | Run `cd web && npm install && npm run build`. |
| Stabilize fails / health shows no IMU | Check I2C and BNO085 wiring; stabilize needs IMU in hardware mode. |
| Dashboard shows “Reconnecting…” forever | Path B API is not running or not reachable — start Uvicorn and confirm `curl /api/health` works. |
