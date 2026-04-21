# Reaction Wheel Telemetry

This repo has **two ways** to run motor control:

| What you want | Command | Notes |
|----------------|---------|--------|
| **Web UI + REST + WebSocket** (telemetry dashboard) | `python -m uvicorn app.main:app ...` | Uses `app/` (FastAPI). Serves `web/dist` if built. |
| **Terminal-only** (legacy interactive CLI) | `python main.py` | Root `main.py` only; no HTTP server. |

FastAPI exposes:

- `GET /api/health` — IMU / mock status  
- `POST /api/command` — motor commands (JSON)  
- `WebSocket /ws/telemetry` — live telemetry (~20 Hz)  
- Static UI from `web/dist/` when present  

Hardware modes for the **FastAPI** stack:

- **Raspberry Pi:** real GPIO, encoder, BNO085 (unless `MOCK_TELEMETRY=1`)  
- **Mock:** `MOCK_TELEMETRY=1` or no Pi hardware libraries — synthetic encoder + IMU  

## Repository layout

- `app/main.py` — FastAPI app, routes, WebSocket, static UI  
- `app/motor_loop.py` — motor control loop  
- `app/hardware.py` — `PiHardware`, `MockHardware`, `create_hardware()`  
- `app/state.py` — shared state, stabilize tuning env vars  
- `main.py` — **legacy** CLI (stdin: `100 cw`, `brake`, …); not the HTTP server  
- `web/` — React + Vite frontend (`npm run build` → `web/dist/`)  

**You must run Python from the repository root** — the directory that contains both `app/` and `main.py`. If `import app` fails, you are in the wrong folder (e.g. a nested `wheel/wheel/` with only `web/`).

```bash
cd /path/to/wheel    # must list: app  main.py  requirements.txt
ls app main.py
```

## Install (Python)

```bash
cd /path/to/wheel
python3 -m venv .venv
source .venv/bin/activate          # Windows: .venv\Scripts\activate
pip install -r requirements.txt
```

On a Mac or PC without a Pi, use mock mode so GPIO/I2C packages are not required at import time (or install only the first three lines of `requirements.txt` for a minimal API-only dev setup).

## Install Node.js and npm (only to build or dev the frontend)

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

API-only: you can skip Node; open `http://<host>:8000` will show JSON with build instructions until `web/dist` exists.

## Run — Telemetry server (FastAPI) on Raspberry Pi

From **repository root**, with venv activated:

```bash
cd /path/to/wheel
source .venv/bin/activate
python -m uvicorn app.main:app --host 0.0.0.0 --port 8000
```

This is **real hardware mode**: do **not** set `MOCK_TELEMETRY`. The process uses GPIO, the quadrature encoder, and the BNO085 over I2C (see **Raspberry Pi prerequisites**). Live encoder RPM and IMU yaw rate appear on `/ws/telemetry` and in the UI; `GET /api/health` should report `"mock": false` when `PiHardware` is active (and `"imu_connected": true` if the IMU initialized).

If hardware initialization fails at startup, fix wiring/I2C/permissions or temporarily use mock mode below while debugging.

Open **`http://<pi-ip>:8000`** on another machine on the same network.

- Use **`0.0.0.0`** so other devices can reach the Pi. **`127.0.0.1`** only accepts connections on the Pi itself.  
- The motor thread starts with the server.

### Mock / no hardware (laptop or Pi without wiring)

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

For real hardware from `web/` (still binds `127.0.0.1` — fine for browser on the same Pi; for LAN use the full uvicorn line with `--host 0.0.0.0`):

```bash
npm run api
```

## Run — Frontend dev (Vite + API)

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

## Run — Legacy terminal CLI (no web)

From repository root:

```bash
cd /path/to/wheel
source .venv/bin/activate
python main.py
```

Follow the printed prompts (`100 cw`, `brake`, etc.). This is separate from the FastAPI app.

## Raspberry Pi prerequisites (hardware mode)

1. Raspberry Pi OS, Python 3.10+.  
2. **I2C enabled:** `sudo raspi-config` → Interface Options → I2C → Enable.  
3. User in `gpio` / `i2c` groups or run with permissions that can access GPIO and I2C.  
4. Wiring expected by `PiHardware` in `app/hardware.py`:

   - Motor PWM: **GPIO 12** (RPWM), **GPIO 13** (LPWM)  
   - Enables: **GPIO 24** (R_EN), **GPIO 25** (L_EN)  
   - Encoder: **GPIO 22** (A), **GPIO 23** (B)  
   - IMU: **BNO085** on I2C  

## API quick reference

- `GET /api/health` — `ok`, `imu_connected`, `mock`  
- `POST /api/command` — e.g. `{"action":"brake"}`, `{"action":"stabilize"}`, `{"target_rpm":50,"direction":"cw"}`  
- `WS /ws/telemetry` — JSON fields: `t`, `motor_rpm`, `wheel_rad_s`, `imu_ok`, `duty_cycle`, `target_rpm`, `direction`, `stabilize_active`, `stabilize_pid_u`  

## Stabilize tuning (optional)

Environment variables (defaults in `app/state.py`):  
`STABILIZE_THRESHOLD_RAD_S`, `STABILIZE_DEBOUNCE_COUNT`, `STABILIZE_KP`, `STABILIZE_KI`, `STABILIZE_KD`, `STABILIZE_TORQUE_SIGN`, `STABILIZE_INTEGRAL_MAX`.

Example:

```bash
STABILIZE_KP=10 STABILIZE_KI=1.5 STABILIZE_KD=0.05 \
  python -m uvicorn app.main:app --host 0.0.0.0 --port 8000
```

## Troubleshooting

### `ModuleNotFoundError: No module named 'app'` (wrong folder or incomplete clone)

Python only finds the `app` package if your **current working directory** is on `sys.path` and the **`app/` directory** sits next to you as a top-level package.

Typical mistake: shell prompt shows a **nested** path such as `~/Desktop/wheel/wheel` while the real project root is **`~/Desktop/wheel`** (one level up), where `app/` and `main.py` actually live.

**Fix:**

1. Go to the directory that **contains** the `app` folder:

   ```bash
   cd ~/Desktop/wheel          # adjust if your clone lives elsewhere
   ls -la app main.py        # both must exist; if `ls app` fails, see below
   ```

2. Run Uvicorn from **that** directory:

   ```bash
   python -m uvicorn app.main:app --host 0.0.0.0 --port 8000
   ```

3. If you **must** stay inside a subdirectory (e.g. you are in `wheel/wheel` and `app` is in the parent), point Uvicorn at the parent:

   ```bash
   python -m uvicorn app.main:app --app-dir .. --host 127.0.0.1 --port 8000
   ```

4. If `ls app` fails even from what you think is the repo root, your tree is **incomplete** (only `web/` copied, etc.). Clone or copy the full repository so the **`app/`** package exists, then repeat step 2.

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
| `ModuleNotFoundError: No module named 'app'` | Wrong `cd`, or missing `app/` in clone | `cd` to repo root; `ls app`; full clone if needed; or `--app-dir ..` from a subfolder |
| `Import string "app.main" must be in format "<module>:<attribute>"` | Missing `:app` | Use `app.main:app` |
| Same `No module named 'app'` after using `app.main:main` | Wrong attribute name | Use **`app.main:app`**, not `:main` |

### Other issues

| Problem | What to do |
|---------|------------|
| Clone has `web/` but no `app/` | Use a full clone or copy; the FastAPI code lives under `app/`. |
| `npm: command not found` | Install Node.js/npm (see **Install Node.js** above). |
| Browser on another PC cannot open `:8000` | Bind `--host 0.0.0.0`, not only `127.0.0.1`. |
| Root URL shows JSON “Web UI not built” | Run `cd web && npm install && npm run build`. |
| Stabilize fails / health shows no IMU | Check I2C and BNO085 wiring; stabilize needs IMU in hardware mode. |
