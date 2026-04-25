import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import {
  VictoryAxis,
  VictoryChart,
  VictoryLine,
  VictoryTheme,
  VictoryTooltip,
  VictoryVoronoiContainer,
} from "victory";
import "./App.css";

const MAX_POINTS = 480;
const WINDOW_SEC = 50;

type FollowConfig = {
  kp: number;
  ki: number;
  kd: number;
  max_rpm: number;
  dead_zone: number;
  stop_radius: number;
  invert: boolean;
  hsv_lower: [number, number, number];
  hsv_upper: [number, number, number];
  track_width: number;
};

type FollowStatus = {
  running: boolean;
  available: boolean;
  error: string | null;
  fps: number;
  detected: boolean;
  cx: number | null;
  cy: number | null;
  radius: number | null;
  error_px: number | null;
  target_rpm: number;
  direction: string;
  config: FollowConfig;
};

type Telemetry = {
  t: number;
  motor_rpm: number;
  wheel_rad_s: number | null;
  imu_ok: boolean;
  duty_cycle: number;
  target_rpm: number;
  direction: string;
  follow?: FollowStatus;
};

const DEFAULT_FOLLOW_CONFIG: FollowConfig = {
  kp: 0.08,
  ki: 0.0,
  kd: 0.02,
  max_rpm: 60,
  dead_zone: 0.05,
  stop_radius: 80,
  invert: false,
  hsv_lower: [25, 80, 80],
  hsv_upper: [45, 255, 255],
  track_width: 400,
};

type Datum = { x: number; y: number };

function radToRpm(rad: number): number {
  return (rad * 60) / (2 * Math.PI);
}

/**
 * In dev, use FastAPI on :8000 directly. Routing WebSockets through Vite’s proxy
 * often produces EPIPE / ECONNRESET (StrictMode double-mount + proxy quirks).
 * Set `VITE_API_ORIGIN` in `.env.development` if the API runs elsewhere.
 */
function apiOrigin(): string {
  const v = import.meta.env.VITE_API_ORIGIN;
  if (typeof v === "string" && v.trim()) return v.trim().replace(/\/$/, "");
  if (import.meta.env.DEV) return "http://127.0.0.1:8000";
  return "";
}

function wsUrl(): string {
  const origin = apiOrigin();
  if (origin) {
    const u = new URL(origin);
    const wsProto = u.protocol === "https:" ? "wss:" : "ws:";
    return `${wsProto}//${u.host}/ws/telemetry`;
  }
  const proto = location.protocol === "https:" ? "wss" : "ws";
  return `${proto}://${location.host}/ws/telemetry`;
}

function useTelemetryStream() {
  const [connected, setConnected] = useState(false);
  const [last, setLast] = useState<Telemetry | null>(null);
  const [follow, setFollow] = useState<FollowStatus | null>(null);
  const [motorSeries, setMotorSeries] = useState<Datum[]>([]);
  const [wheelSeries, setWheelSeries] = useState<Datum[]>([]);
  const t0Ref = useRef<number | null>(null);

  useEffect(() => {
    let stopped = false;
    let socket: WebSocket | null = null;
    let attempt = 0;
    let timer: ReturnType<typeof setTimeout> | undefined;

    const trim = (pts: Datum[]) =>
      pts.length > MAX_POINTS ? pts.slice(-MAX_POINTS) : pts;

    const connect = () => {
      if (stopped) return;
      socket = new WebSocket(wsUrl());
      socket.onopen = () => {
        setConnected(true);
        attempt = 0;
      };
      socket.onclose = () => {
        setConnected(false);
        if (stopped) return;
        const delay = Math.min(4000, 400 + attempt * 350);
        attempt += 1;
        timer = setTimeout(connect, delay);
      };
      socket.onerror = () => {
        socket?.close();
      };
      socket.onmessage = (ev) => {
        const msg = JSON.parse(ev.data) as Telemetry;
        if (t0Ref.current === null) t0Ref.current = msg.t;
        const x = msg.t - (t0Ref.current ?? msg.t);
        setLast(msg);
        if (msg.follow) setFollow(msg.follow);
        setMotorSeries((prev) => trim([...prev, { x, y: msg.motor_rpm }]));
        if (msg.imu_ok && msg.wheel_rad_s != null) {
          setWheelSeries((prev) => trim([...prev, { x, y: msg.wheel_rad_s as number }]));
        }
      };
    };

    connect();
    return () => {
      stopped = true;
      clearTimeout(timer);
      socket?.close();
    };
  }, []);

  const motorDomainX = useMemo(() => {
    if (!motorSeries.length) return [0, 1] as [number, number];
    const xmax = motorSeries[motorSeries.length - 1].x;
    return [Math.max(0, xmax - WINDOW_SEC), xmax + 0.001] as [number, number];
  }, [motorSeries]);

  const wheelDomainX = useMemo(() => {
    if (!wheelSeries.length) return [0, 1] as [number, number];
    const xmax = wheelSeries[wheelSeries.length - 1].x;
    return [Math.max(0, xmax - WINDOW_SEC), xmax + 0.001] as [number, number];
  }, [wheelSeries]);

  return {
    connected,
    last,
    follow,
    setFollow,
    motorSeries,
    wheelSeries,
    motorDomainX,
    wheelDomainX,
  };
}

const chartTheme = {
  ...VictoryTheme.material,
  axis: {
    ...VictoryTheme.material.axis,
    style: {
      ...VictoryTheme.material.axis?.style,
      axis: { stroke: "#334155", strokeWidth: 1 },
      tickLabels: {
        fill: "#94a3b8",
        fontFamily: "ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace",
        fontSize: 10,
        padding: 4,
      },
      grid: { stroke: "rgba(148,163,184,0.12)", strokeDasharray: "4 4" },
    },
  },
};

type Health = {
  ok: boolean;
  imu_connected: boolean;
  mock: boolean;
  camera_available: boolean;
  camera_running: boolean;
};

function useHealth(connected: boolean): Health | null {
  const [health, setHealth] = useState<Health | null>(null);
  useEffect(() => {
    let cancelled = false;
    const fetchHealth = async () => {
      try {
        const res = await fetch(`${apiOrigin()}/api/health`);
        if (!res.ok) return;
        const j = (await res.json()) as Health;
        if (!cancelled) setHealth(j);
      } catch {
        // ignore
      }
    };
    void fetchHealth();
    const id = setInterval(fetchHealth, 5000);
    return () => {
      cancelled = true;
      clearInterval(id);
    };
  }, [connected]);
  return health;
}

function hsvToString(v: [number, number, number]): string {
  return `${v[0]},${v[1]},${v[2]}`;
}

function parseHsv(s: string): [number, number, number] | null {
  const parts = s.split(",").map((p) => p.trim());
  if (parts.length !== 3) return null;
  const nums = parts.map((p) => Number(p));
  if (nums.some((n) => !Number.isFinite(n))) return null;
  return [Math.round(nums[0]), Math.round(nums[1]), Math.round(nums[2])];
}

function CameraFollowPanel(props: {
  follow: FollowStatus | null;
  health: Health | null;
  setFollow: (s: FollowStatus) => void;
}) {
  const { follow, health, setFollow } = props;
  const cameraAvailable = health?.camera_available ?? true;
  const running = follow?.running ?? false;
  const cfg = follow?.config ?? DEFAULT_FOLLOW_CONFIG;

  const [draft, setDraft] = useState<FollowConfig>(DEFAULT_FOLLOW_CONFIG);
  const [hsvLowerStr, setHsvLowerStr] = useState<string>(hsvToString(DEFAULT_FOLLOW_CONFIG.hsv_lower));
  const [hsvUpperStr, setHsvUpperStr] = useState<string>(hsvToString(DEFAULT_FOLLOW_CONFIG.hsv_upper));
  const [pending, setPending] = useState(false);
  const [showStream, setShowStream] = useState(false);
  const [streamKey, setStreamKey] = useState(0);
  const initFromServer = useRef(false);

  useEffect(() => {
    if (!initFromServer.current && follow) {
      setDraft(follow.config);
      setHsvLowerStr(hsvToString(follow.config.hsv_lower));
      setHsvUpperStr(hsvToString(follow.config.hsv_upper));
      initFromServer.current = true;
    }
  }, [follow]);

  const buildBody = useCallback(() => {
    const lower = parseHsv(hsvLowerStr) ?? draft.hsv_lower;
    const upper = parseHsv(hsvUpperStr) ?? draft.hsv_upper;
    return {
      kp: draft.kp,
      ki: draft.ki,
      kd: draft.kd,
      max_rpm: draft.max_rpm,
      dead_zone: draft.dead_zone,
      stop_radius: draft.stop_radius,
      invert: draft.invert,
      hsv_lower: lower,
      hsv_upper: upper,
      track_width: draft.track_width,
    };
  }, [draft, hsvLowerStr, hsvUpperStr]);

  const post = useCallback(
    async (path: string, body?: object) => {
      setPending(true);
      try {
        const res = await fetch(`${apiOrigin()}${path}`, {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: body ? JSON.stringify(body) : undefined,
        });
        const j = (await res.json().catch(() => ({}))) as Partial<FollowStatus> & { ok?: boolean; error?: string };
        if (j && typeof j === "object" && "running" in j) {
          setFollow(j as FollowStatus);
        }
        if (!res.ok) {
          console.warn(`POST ${path} failed`, j);
        }
      } catch (err) {
        console.warn(err);
      } finally {
        setPending(false);
      }
    },
    [setFollow],
  );

  const startFollow = useCallback(() => {
    setShowStream(true);
    setStreamKey((k) => k + 1);
    void post("/api/follow/start", buildBody());
  }, [buildBody, post]);

  const stopFollow = useCallback(() => {
    void post("/api/follow/stop");
  }, [post]);

  const applyConfig = useCallback(() => {
    void post("/api/follow/config", buildBody());
  }, [buildBody, post]);

  const streamSrc = `${apiOrigin()}/api/camera/stream.mjpg?k=${streamKey}`;

  return (
    <section className="card camera-card">
      <header>
        <h2>Camera follow</h2>
        <span>{running ? "Tracking ball" : cameraAvailable ? "Idle" : "Camera unavailable"}</span>
      </header>
      <div className="camera-body">
        <div className="camera-preview">
          {!cameraAvailable ? (
            <div className="camera-placeholder">
              <p>
                The Pi camera stack (<code>picamera2</code>) is not available on this host. Install
                with <code>sudo apt install -y python3-picamera2</code> on the Raspberry Pi.
              </p>
            </div>
          ) : showStream ? (
            <img
              key={streamKey}
              src={streamSrc}
              alt="Live camera feed"
              onError={() => setShowStream(false)}
            />
          ) : (
            <button type="button" className="primary" onClick={() => { setShowStream(true); setStreamKey((k) => k + 1); }}>
              Show live preview
            </button>
          )}
        </div>

        <div className="camera-controls">
          <div className="status-row">
            <div className={`pill ${follow?.detected ? "ok" : ""}`}>
              <span className="dot" /> {follow?.detected ? "Ball detected" : "No ball"}
            </div>
            <div className="pill">FPS: <strong>{(follow?.fps ?? 0).toFixed(1)}</strong></div>
            <div className="pill">
              Pixel error:{" "}
              <strong>{follow?.error_px == null ? "—" : follow.error_px.toFixed(0)}</strong>
            </div>
            <div className="pill">
              Cmd: <strong>{follow?.direction ?? "brake"} @ {(follow?.target_rpm ?? 0).toFixed(0)} RPM</strong>
            </div>
          </div>

          <div className="control-grid">
            <label>
              Kp
              <input
                type="number"
                step={0.01}
                value={draft.kp}
                onChange={(e) => setDraft((d) => ({ ...d, kp: Number(e.target.value) }))}
              />
            </label>
            <label>
              Ki
              <input
                type="number"
                step={0.01}
                value={draft.ki}
                onChange={(e) => setDraft((d) => ({ ...d, ki: Number(e.target.value) }))}
              />
            </label>
            <label>
              Kd
              <input
                type="number"
                step={0.01}
                value={draft.kd}
                onChange={(e) => setDraft((d) => ({ ...d, kd: Number(e.target.value) }))}
              />
            </label>
            <label>
              Max RPM
              <input
                type="number"
                min={0}
                max={100}
                step={1}
                value={draft.max_rpm}
                onChange={(e) => setDraft((d) => ({ ...d, max_rpm: Number(e.target.value) }))}
              />
            </label>
            <label>
              Dead zone (frac)
              <input
                type="number"
                min={0}
                max={0.3}
                step={0.01}
                value={draft.dead_zone}
                onChange={(e) => setDraft((d) => ({ ...d, dead_zone: Number(e.target.value) }))}
              />
            </label>
            <label>
              Stop radius (px)
              <input
                type="number"
                min={0}
                max={400}
                step={1}
                value={draft.stop_radius}
                onChange={(e) => setDraft((d) => ({ ...d, stop_radius: Number(e.target.value) }))}
              />
            </label>
            <label className="hsv">
              HSV lower (H,S,V)
              <input
                type="text"
                value={hsvLowerStr}
                onChange={(e) => setHsvLowerStr(e.target.value)}
                placeholder="25,80,80"
              />
            </label>
            <label className="hsv">
              HSV upper (H,S,V)
              <input
                type="text"
                value={hsvUpperStr}
                onChange={(e) => setHsvUpperStr(e.target.value)}
                placeholder="45,255,255"
              />
            </label>
            <label className="checkbox">
              <input
                type="checkbox"
                checked={draft.invert}
                onChange={(e) => setDraft((d) => ({ ...d, invert: e.target.checked }))}
              />
              Invert spin direction
            </label>
          </div>

          <div className="control-row">
            {!running ? (
              <button
                type="button"
                className="primary"
                disabled={pending || !cameraAvailable}
                onClick={startFollow}
              >
                Start follow
              </button>
            ) : (
              <button
                type="button"
                className="ghost"
                disabled={pending}
                onClick={stopFollow}
              >
                Stop follow
              </button>
            )}
            <button
              type="button"
              className="ghost"
              disabled={pending || !running}
              onClick={applyConfig}
              title="Hot-update gains/HSV without restarting the loop"
            >
              Apply tuning
            </button>
            <span className="hint">
              Defaults match the CLI: Kp 0.08 / Ki 0 / Kd 0.02, max 60 RPM, 5% dead zone.
              Server caps target RPM at {cfg.max_rpm.toFixed(0)} (≤ 100).
            </span>
          </div>

          {follow?.error && (
            <div className="offline-banner">{follow.error}</div>
          )}
        </div>
      </div>
    </section>
  );
}

function TelemetryChart(props: {
  title: string;
  subtitle: string;
  data: Datum[];
  domainX: [number, number];
  yLabel: string;
  formatY: (v: number) => string;
  emptyHint: string;
}) {
  const { title, subtitle, data, domainX, yLabel, formatY, emptyHint } = props;
  const hasData = data.length > 1;

  return (
    <section className="card">
      <header>
        <h2>{title}</h2>
        <span>{subtitle}</span>
      </header>
      <div className="chart-wrap">
        {!hasData ? (
          <p style={{ color: "var(--muted)", padding: "0 0.5rem 1rem", fontSize: "0.9rem" }}>{emptyHint}</p>
        ) : (
          <VictoryChart
            theme={chartTheme}
            height={280}
            padding={{ left: 54, right: 16, top: 12, bottom: 40 }}
            domain={{ x: domainX }}
            domainPadding={{ y: 12 }}
            containerComponent={
              <VictoryVoronoiContainer
                voronoiDimension="x"
                labels={({ datum }) => {
                  const d = datum as Datum;
                  return `${d.x.toFixed(1)}s · ${yLabel} ${formatY(d.y)}`;
                }}
                labelComponent={
                  <VictoryTooltip
                    cornerRadius={6}
                    flyoutPadding={8}
                    flyoutStyle={{
                      fill: "rgba(15,23,42,0.96)",
                      stroke: "rgba(148,163,184,0.35)",
                      strokeWidth: 1,
                    }}
                    style={{
                      fill: "#e2e8f0",
                      fontSize: 11,
                      fontFamily:
                        "ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace",
                    }}
                  />
                }
              />
            }
          >
            <VictoryAxis dependentAxis tickFormat={(t) => formatY(Number(t))} label={yLabel} style={{ axisLabel: { padding: 36, fill: "#94a3b8" } }} />
            <VictoryAxis tickFormat={(t) => `${Number(t).toFixed(0)}s`} />
            <VictoryLine
              interpolation="monotoneX"
              style={{ data: { stroke: "#60a5fa", strokeWidth: 2 } }}
              data={data}
              x="x"
              y="y"
            />
          </VictoryChart>
        )}
      </div>
    </section>
  );
}

export default function App() {
  const { connected, last, follow, setFollow, motorSeries, wheelSeries, motorDomainX, wheelDomainX } = useTelemetryStream();
  const health = useHealth(connected);
  const [rpmInput, setRpmInput] = useState("40");
  const [dir, setDir] = useState<"cw" | "ccw">("cw");
  const [pending, setPending] = useState(false);
  const followRunning = follow?.running ?? false;

  const sendCommand = useCallback(async (body: object) => {
    setPending(true);
    try {
      const res = await fetch(`${apiOrigin()}/api/command`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(body),
      });
      if (!res.ok) {
        const j = await res.json().catch(() => ({}));
        console.warn("command failed", j);
      }
    } catch (e) {
      console.warn(e);
    } finally {
      setPending(false);
    }
  }, []);

  const wheelRpmLive =
    last && last.imu_ok && last.wheel_rad_s != null ? radToRpm(last.wheel_rad_s) : null;

  return (
    <div className="shell">
      <div className="top">
        <div className="title-block">
          <h1>Reaction wheel telemetry</h1>
          <p>Live motor encoder speed and IMU-derived platform rate, streamed over WebSocket.</p>
        </div>
        <div className="status-row">
          <div className={`pill ${connected ? "ok" : "warn"}`}>
            <span className="dot" />
            {connected ? "Live stream" : "Reconnecting…"}
          </div>
          {last && (
            <>
              <div className="pill">
                Target: <strong>{last.target_rpm.toFixed(0)}</strong> RPM · {last.direction}
              </div>
              <div className="pill">
                Duty: <strong>{(last.duty_cycle * 100).toFixed(0)}%</strong>
              </div>
            </>
          )}
        </div>
      </div>

      {!connected && (
        <div className="offline-banner">WebSocket disconnected. Attempting to reconnect…</div>
      )}

      <CameraFollowPanel follow={follow} health={health} setFollow={setFollow} />

      <div className="grid">
        <TelemetryChart
          title="Motor speed (motor encoder)"
          subtitle="Absolute shaft RPM from the quadrature encoder"
          data={motorSeries}
          domainX={motorDomainX}
          yLabel="RPM"
          formatY={(v) => v.toFixed(1)}
          emptyHint="Waiting for motor samples…"
        />
        <TelemetryChart
          title="Reaction wheel speed (IMU)"
          subtitle="Yaw angular rate (rad/s). Equivalent wheel RPM shown in the status strip."
          data={wheelSeries}
          domainX={wheelDomainX}
          yLabel="rad/s"
          formatY={(v) => v.toFixed(3)}
          emptyHint={
            last && !last.imu_ok
              ? "IMU offline or readings unavailable."
              : "Waiting for IMU samples…"
          }
        />
      </div>

      {last && (
        <div className="panel">
          <h3>Live readouts</h3>
          <div className="status-row">
            <div className="pill">
              Motor RPM: <strong className="live">{last.motor_rpm.toFixed(1)}</strong>
            </div>
            <div className="pill">
              IMU yaw rate:{" "}
              <strong className="live">
                {last.imu_ok && last.wheel_rad_s != null ? `${last.wheel_rad_s.toFixed(3)} rad/s` : "—"}
              </strong>
            </div>
            {wheelRpmLive != null && (
              <div className="pill">
                ≈ wheel RPM: <strong className="live">{wheelRpmLive.toFixed(1)}</strong>
              </div>
            )}
          </div>
        </div>
      )}

      <div className="panel">
        <h3>Motor command</h3>
        <div className="control-row">
          <label style={{ color: "var(--muted)", fontSize: "0.85rem" }}>
            RPM
            <input
              type="number"
              min={0}
              max={100}
              step={1}
              value={rpmInput}
              onChange={(e) => setRpmInput(e.target.value)}
              style={{ marginLeft: "0.35rem" }}
              disabled={followRunning}
            />
          </label>
          <select
            value={dir}
            onChange={(e) => setDir(e.target.value as "cw" | "ccw")}
            disabled={followRunning}
          >
            <option value="cw">Clockwise</option>
            <option value="ccw">Counter-clockwise</option>
          </select>
          <button
            type="button"
            className="primary"
            disabled={pending || followRunning}
            onClick={() => {
              const v = Number(rpmInput);
              if (Number.isFinite(v)) void sendCommand({ target_rpm: v, direction: dir });
            }}
          >
            Apply
          </button>
          <button type="button" className="ghost" disabled={pending} onClick={() => void sendCommand({ action: "brake" })}>
            Brake
          </button>
        </div>
        <p className="hint">
          {followRunning
            ? "Camera follow is in charge of the motor. Stop follow above to drive manually."
            : "Same limits as the CLI: 0–100 RPM, directions cw / ccw, or brake."}
        </p>
      </div>
    </div>
  );
}
