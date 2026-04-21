# Reaction Wheel Telemetry

FastAPI serves:
- a live WebSocket telemetry stream (`/ws/telemetry`)
- command endpoints for motor control (`/api/command`)
- the built React dashboard (`web/dist`)

This project can run in two modes:
- **Raspberry Pi hardware mode** (GPIO + encoder + BNO085 IMU)
- **Mock mode** (`MOCK_TELEMETRY=1`) for development without hardware

## Repository layout

- `app/main.py` - FastAPI app, API routes, WebSocket stream, static UI hosting
- `app/motor_loop.py` - motor control loop (inner PD + stabilize outer PID)
- `app/hardware.py` - Raspberry Pi hardware adapters + mock hardware adapter
- `main.py` - convenience entry point to start Uvicorn
- `web/` - React + Vite frontend

## Raspberry Pi prerequisites

1. Raspberry Pi OS with Python 3.10+ and Node 18+ (Node only needed to build UI).
2. I2C enabled (`sudo raspi-config` -> Interface Options -> I2C -> Enable).
3. User has GPIO/I2C permissions (or run with appropriate privileges).
4. Hardware wired to the GPIO pins expected by `PiHardware`:

   - Motor driver PWM:
     - `RPWM`: GPIO 12
     - `LPWM`: GPIO 13
   - Motor driver enables:
     - `R_EN`: GPIO 24
     - `L_EN`: GPIO 25
   - Quadrature encoder:
     - `ENC_A`: GPIO 22
     - `ENC_B`: GPIO 23
   - IMU:
     - BNO085 over I2C

## Install

From repository root:

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

Build frontend once (or whenever UI changes):

```bash
cd web
npm install
npm run build
cd ..
```

## Run on Raspberry Pi (real hardware)

Always start from the repository root (directory containing `app/` and `main.py`):

```bash
cd /path/to/wheel
python main.py
```

Equivalent:

```bash
python -m uvicorn app.main:app --host 0.0.0.0 --port 8000
```

Open:
- `http://<pi-ip>:8000`

The motor loop starts with the API process.

## Run in mock mode (no GPIO/IMU)

Use this on Mac/Linux dev machines or when hardware is disconnected:

```bash
cd /path/to/wheel
python main.py --mock
```

Equivalent:

```bash
MOCK_TELEMETRY=1 python -m uvicorn app.main:app --host 127.0.0.1 --port 8000
```

## Frontend dev workflow

Run backend and frontend in separate terminals.

Terminal 1 (backend):

```bash
cd /path/to/wheel
MOCK_TELEMETRY=1 python -m uvicorn app.main:app --host 127.0.0.1 --port 8000
```

Terminal 2 (frontend):

```bash
cd /path/to/wheel/web
npm run dev
```

Open:
- `http://127.0.0.1:5173`

The frontend calls `http://127.0.0.1:8000` directly in dev mode by default.

## API quick reference

- `GET /api/health`
  - Returns `ok`, `imu_connected`, and `mock`.
- `POST /api/command`
  - Brake: `{"action":"brake"}`
  - Stabilize: `{"action":"stabilize"}`
  - Spin: `{"target_rpm":50,"direction":"cw"}` or `{"target_rpm":50,"direction":"ccw"}`
- `WS /ws/telemetry`
  - Streams `t`, `motor_rpm`, `wheel_rad_s`, `imu_ok`, `duty_cycle`, `target_rpm`, `direction`, `stabilize_active`, `stabilize_pid_u`

## Stabilize mode tuning (optional)

You can tune stabilize behavior with environment variables:

- `STABILIZE_THRESHOLD_RAD_S` (default `0.2`)
- `STABILIZE_DEBOUNCE_COUNT` (default `15`)
- `STABILIZE_KP` (default `12.0`)
- `STABILIZE_KI` (default `2.0`)
- `STABILIZE_KD` (default `0.08`)
- `STABILIZE_TORQUE_SIGN` (default `1`)
- `STABILIZE_INTEGRAL_MAX` (default `5.0`)

Example:

```bash
STABILIZE_KP=10 STABILIZE_KI=1.5 STABILIZE_KD=0.05 python main.py
```

## Troubleshooting

- `ModuleNotFoundError: No module named 'app'`
  - Start Python from repo root, not from `web/`.
- IMU unavailable in `/api/health`
  - Verify I2C is enabled and BNO085 wiring/power are correct.
- UI root shows a JSON message saying UI not built
  - Run `cd web && npm install && npm run build`.

Install node package manager: 
  # install Node.js + npm (Node 20 LTS)
curl -fsSL https://deb.nodesource.com/setup_20.x | sudo -E bash -
sudo apt-get install -y nodejs
# verify
node -v
npm -v
