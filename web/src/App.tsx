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

type Telemetry = {
  t: number;
  motor_rpm: number;
  wheel_rad_s: number | null;
  imu_ok: boolean;
  duty_cycle: number;
  target_rpm: number;
  direction: string;
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
        setMotorSeries((prev) => trim([...prev, { x, y: msg.motor_rpm }]));
        if (msg.imu_ok && msg.wheel_rad_s != null) {
          setWheelSeries((prev) => trim([...prev, { x, y: msg.wheel_rad_s }]));
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
  const { connected, last, motorSeries, wheelSeries, motorDomainX, wheelDomainX } = useTelemetryStream();
  const [rpmInput, setRpmInput] = useState("40");
  const [dir, setDir] = useState<"cw" | "ccw">("cw");
  const [pending, setPending] = useState(false);

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
            />
          </label>
          <select value={dir} onChange={(e) => setDir(e.target.value as "cw" | "ccw")}>
            <option value="cw">Clockwise</option>
            <option value="ccw">Counter-clockwise</option>
          </select>
          <button
            type="button"
            className="primary"
            disabled={pending}
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
        <p className="hint">Same limits as the CLI: 0–100 RPM, directions cw / ccw, or brake.</p>
      </div>
    </div>
  );
}
