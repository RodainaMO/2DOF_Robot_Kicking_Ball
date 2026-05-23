import cv2
import threading
import time
import requests
import numpy as np
from ultralytics import YOLO
import csv
from datetime import datetime
from enum import Enum
from http.server import HTTPServer, BaseHTTPRequestHandler
import json

ESP_IP = "10.147.188.66"

URLS = [
    f"http://{ESP_IP}:81/stream",
    f"http://{ESP_IP}/stream"
]

# =========================
# STATE MACHINE
# =========================
class RobotState(Enum):
    SEARCHING = 0
    DETECTING = 1
    CONFIRMED = 2
    KICKING   = 3
    COOLDOWN  = 4

state = RobotState.SEARCHING

STATE_COLORS = {
    RobotState.SEARCHING: (0, 0, 255),
    RobotState.DETECTING: (0, 165, 255),
    RobotState.CONFIRMED: (0, 255, 0),
    RobotState.KICKING:   (0, 0, 200),
    RobotState.COOLDOWN:  (255, 0, 255),
}

def transition(new_state, reason=""):
    global state
    if state != new_state:
        msg = f"[STATE] {state.name} → {new_state.name}" + (f"  ({reason})" if reason else "")
        print(msg)
        telemetry["state_log"].append({
            "time": datetime.now().strftime("%H:%M:%S"),
            "from": state.name,
            "to":   new_state.name,
            "reason": reason
        })
        if len(telemetry["state_log"]) > 50:
            telemetry["state_log"].pop(0)
        state = new_state

# =========================
# TELEMETRY (shared with dashboard)
# =========================
telemetry = {
    "state":       "SEARCHING",
    "cx_px":       0,
    "cy_px":       0,
    "ik_x":        0.0,
    "ik_y":        0.0,
    "latency_ms":  0.0,
    "kick_count":  0,
    "state_log":   []
}
telemetry_lock = threading.Lock()

def update_telemetry(**kwargs):
    with telemetry_lock:
        telemetry.update(kwargs)
        telemetry["state"] = state.name

# =========================
# DASHBOARD SERVER
# =========================
DASHBOARD_HTML = """<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta http-equiv="refresh" content="1">
  <title>Robot Dashboard</title>
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body {
      background: #0d0d0d;
      color: #e0e0e0;
      font-family: 'Courier New', monospace;
      padding: 20px;
    }
    h1 {
      text-align: center;
      color: #00e5ff;
      font-size: 1.6em;
      margin-bottom: 20px;
      letter-spacing: 3px;
    }
    .grid {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 16px;
      max-width: 900px;
      margin: 0 auto;
    }
    .card {
      background: #1a1a1a;
      border: 1px solid #2a2a2a;
      border-radius: 10px;
      padding: 16px;
    }
    .card h2 {
      font-size: 0.75em;
      color: #888;
      letter-spacing: 2px;
      margin-bottom: 12px;
      text-transform: uppercase;
    }
    .state-badge {
      display: inline-block;
      padding: 8px 20px;
      border-radius: 20px;
      font-size: 1.2em;
      font-weight: bold;
      letter-spacing: 2px;
    }
    .SEARCHING { background:#1a0000; color:#ff4444; border:1px solid #ff4444; }
    .DETECTING { background:#1a0d00; color:#ffa500; border:1px solid #ffa500; }
    .CONFIRMED { background:#001a00; color:#00ff00; border:1px solid #00ff00; }
    .KICKING   { background:#00001a; color:#4488ff; border:1px solid #4488ff; }
    .COOLDOWN  { background:#1a001a; color:#ff00ff; border:1px solid #ff00ff; }
    .metric-row {
      display: flex;
      justify-content: space-between;
      align-items: center;
      padding: 8px 0;
      border-bottom: 1px solid #2a2a2a;
    }
    .metric-row:last-child { border-bottom: none; }
    .metric-label { color: #888; font-size: 0.85em; }
    .metric-value { color: #00e5ff; font-size: 1.1em; font-weight: bold; }
    .kick-count { font-size: 2.5em; color: #00ff00; font-weight: bold; text-align:center; }
    .log-entry {
      font-size: 0.75em;
      padding: 4px 0;
      border-bottom: 1px solid #1a1a1a;
      color: #aaa;
    }
    .log-entry span { color: #00e5ff; }
    .log-scroll { max-height: 180px; overflow-y: auto; }
    .full-width { grid-column: span 2; }
    .dot {
      display:inline-block;
      width:8px; height:8px;
      background:#00ff00;
      border-radius:50%;
      margin-right:6px;
      animation: blink 1s infinite;
    }
    @keyframes blink { 0%,100%{opacity:1} 50%{opacity:0.2} }
  </style>
</head>
<body>
  <h1>⚽ ROBOT TELEMETRY DASHBOARD</h1>
  <div class="grid" id="grid">Loading...</div>

  <script>
    async function refresh() {
      try {
        const r = await fetch('/api');
        const d = await r.json();

        document.getElementById('grid').innerHTML = `
          <div class="card">
            <h2>State Machine</h2>
            <div style="text-align:center; padding:10px 0">
              <span class="state-badge ${d.state}">${d.state}</span>
            </div>
          </div>

          <div class="card">
            <h2>Kick Count</h2>
            <div class="kick-count">${d.kick_count}</div>
          </div>

          <div class="card">
            <h2>Ball Position (pixels)</h2>
            <div class="metric-row">
              <span class="metric-label">cx</span>
              <span class="metric-value">${d.cx_px} px</span>
            </div>
            <div class="metric-row">
              <span class="metric-label">cy</span>
              <span class="metric-value">${d.cy_px} px</span>
            </div>
          </div>

          <div class="card">
            <h2>IK Target (cm)</h2>
            <div class="metric-row">
              <span class="metric-label">x</span>
              <span class="metric-value">${d.ik_x} cm</span>
            </div>
            <div class="metric-row">
              <span class="metric-label">y</span>
              <span class="metric-value">${d.ik_y} cm</span>
            </div>
          </div>

          <div class="card full-width">
            <h2>Vision Latency</h2>
            <div class="metric-row">
              <span class="metric-label">Current frame</span>
              <span class="metric-value">${d.latency_ms} ms</span>
            </div>
          </div>

          <div class="card full-width">
            <h2><span class="dot"></span>State Transition Log</h2>
            <div class="log-scroll">
              ${d.state_log.slice().reverse().map(e =>
                `<div class="log-entry">
                  [${e.time}] <span>${e.from}</span> → <span>${e.to}</span>
                  &nbsp;&nbsp;${e.reason}
                </div>`
              ).join('') || '<div class="log-entry">No transitions yet</div>'}
            </div>
          </div>
        `;
      } catch(e) {}
    }
    refresh();
    setInterval(refresh, 800);
  </script>
</body>
</html>"""

class DashboardHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/api':
            with telemetry_lock:
                data = json.dumps(telemetry)
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(data.encode())
        else:
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.end_headers()
            self.wfile.write(DASHBOARD_HTML.encode())

    def log_message(self, format, *args):
        pass  # suppress server logs

def start_dashboard(port=8080):
    server = HTTPServer(('0.0.0.0', port), DashboardHandler)
    t = threading.Thread(target=server.serve_forever, daemon=True)
    t.start()
    print(f"[DASHBOARD] Running at http://localhost:{port}")

# =========================
# KICK SETTINGS
# =========================
KICK_COOLDOWN  = 2.0
last_kick_time = 0
lock_time      = 0
kick_count     = 0

def send_kick(ball_x, ball_y):
    try:
        requests.get(f"http://{ESP_IP}/move?x={ball_x:.1f}&y={ball_y:.1f}", timeout=1.0)
        requests.get(f"http://{ESP_IP}/kick", timeout=1.0)
    except:
        pass

def trigger_kick(ball_x, ball_y):
    threading.Thread(target=send_kick, args=(ball_x, ball_y), daemon=True).start()

# =========================
# STREAM
# =========================
def connect():
    for u in URLS:
        cap = cv2.VideoCapture(u)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if cap.read()[0]:
            print(f"Connected: {u}")
            return cap
        cap.release()
    exit("No stream found")

cap  = connect()
yolo = YOLO("yolov8n.pt")

BALL_IDS      = {32, 36, 37, 38}
REAL_D, FOCAL = 6.5, 700

# Temporal consistency
CONFIRM_FRAMES   = 2
consecutive_hits = 0

# Loosened size filter
MIN_RADIUS = 8
MAX_RADIUS = 150

# =========================
# METRICS SETUP
# =========================
METRICS_FILE  = "vision_metrics.csv"
detection_log = []

with open(METRICS_FILE, "w", newline="") as f:
    csv.writer(f).writerow(["timestamp", "cx_px", "cy_px", "latency_ms"])

print(f"[METRICS] Logging detections to: {METRICS_FILE}")

# =========================
# HOUGH REFINEMENT
# =========================
def hough(frame, x1, y1, x2, y2):
    p   = 15
    roi = frame[max(0,y1-p):min(frame.shape[0],y2+p),
                max(0,x1-p):min(frame.shape[1],x2+p)]
    if not roi.size:
        return None
    g  = cv2.GaussianBlur(cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY), (7,7), 2)
    br = min(x2-x1, y2-y1)//2
    c  = cv2.HoughCircles(g, cv2.HOUGH_GRADIENT, 1.2, 30,
                          param1=60, param2=15,
                          minRadius=max(4, int(br*.3)),
                          maxRadius=int(br*1.5))
    if c is None:
        return None
    ox, oy = max(0,x1-p), max(0,y1-p)
    bx, by = (x1+x2)//2-ox, (y1+y2)//2-oy
    best = min(np.uint16(np.around(c))[0],
               key=lambda v: (int(v[0])-bx)**2+(int(v[1])-by)**2)
    return ox+int(best[0]), oy+int(best[1]), int(best[2])

# =========================
# VALIDATE DETECTION
# =========================
def is_valid_ball(x1, y1, x2, y2, r):
    w = x2 - x1
    h = y2 - y1
    if w == 0 or h == 0:
        return False
    aspect = w / h
    if aspect < 0.4 or aspect > 2.5:
        return False
    if r < MIN_RADIUS or r > MAX_RADIUS:
        return False
    return True

# =========================
# DRAWING
# =========================
def draw(img, cx, cy, r, label, will_kick):
    ov = img.copy()
    cv2.circle(ov, (cx,cy), r, (0,255,0), -1)
    cv2.addWeighted(ov, .15, img, .85, 0, img)
    color = (0, 0, 255) if will_kick else (0, 255, 0)
    cv2.circle(img, (cx,cy), r, color, 3)
    cv2.line(img, (cx-8,cy), (cx+8,cy), (0,200,255), 2)
    cv2.line(img, (cx,cy-8), (cx,cy+8), (0,200,255), 2)
    (tw,th),_ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, .6, 2)
    cv2.rectangle(img, (cx-tw//2, cy-r-th-8), (cx+tw//2+6, cy-r), (0,60,0), -1)
    cv2.putText(img, label, (cx-tw//2+3, cy-r-5),
                cv2.FONT_HERSHEY_SIMPLEX, .6, (0,255,0), 2)

# =========================
# PIXEL TO CM CONVERSION
# =========================
def pixel_to_cm(cx, img_w=640):
    x_cm = ((cx - img_w / 2) / (img_w / 2)) * 10.0
    y_cm = -15.0
    return round(x_cm, 1), round(y_cm, 1)

# Smoothing
prev = [None, None, None]
S    = 0.35
sm   = lambda n, o: int(o*S + n*(1-S)) if o else n

# =========================
# START DASHBOARD
# =========================
start_dashboard(port=8080)

# =========================
# MAIN LOOP
# =========================
print("Running — press Q to quit")

while True:
    ret, frame = cap.read()
    if not ret:
        continue

    t_frame_start = time.time()

    frame = cv2.resize(frame, (640, 480))
    out   = frame.copy()

    boxes = yolo(frame, conf=0.20, verbose=False)[0].boxes
    found = False
    cx = cy = r = 0

    for b in boxes:
        if int(b.cls[0]) not in BALL_IDS or float(b.conf[0]) < 0.20:
            continue

        x1, y1, x2, y2 = map(int, b.xyxy[0])
        w = x2 - x1
        h = y2 - y1

        aspect = w / max(h, 1)
        if aspect < 0.4 or aspect > 2.5:
            continue

        cv2.rectangle(out, (x1,y1), (x2,y2), (255,100,0), 1)
        cv2.putText(out, f"{float(b.conf[0]):.2f}", (x1, y1-5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,100,0), 1)

        hit     = hough(frame, x1, y1, x2, y2)
        cx,cy,r = hit if hit else ((x1+x2)//2, (y1+y2)//2, min(w,h)//2)

        if not is_valid_ball(x1, y1, x2, y2, r):
            continue

        found = True
        break

    now = time.time()

    # =========================
    # STATE MACHINE TRANSITIONS
    # =========================
    if state == RobotState.SEARCHING:
        if found:
            consecutive_hits = 1
            transition(RobotState.DETECTING, "ball seen")

    elif state == RobotState.DETECTING:
        if found:
            consecutive_hits += 1
            if consecutive_hits >= CONFIRM_FRAMES:
                transition(RobotState.CONFIRMED, "frames confirmed")
        else:
            consecutive_hits = 0
            transition(RobotState.SEARCHING, "ball lost")

    elif state == RobotState.CONFIRMED:
        if not found:
            consecutive_hits = 0
            prev[:] = None, None, None
            transition(RobotState.SEARCHING, "ball lost")
        else:
            transition(RobotState.KICKING, "ball in camera FOV kick zone")

    elif state == RobotState.KICKING:
        ik_x, ik_y = pixel_to_cm(cx)
        print(f"KICK -> IK target: ({ik_x}, {ik_y})")
        trigger_kick(ik_x, ik_y)
        last_kick_time = now
        lock_time      = now
        kick_count    += 1
        transition(RobotState.COOLDOWN, "kick sent")

    elif state == RobotState.COOLDOWN:
        if now - lock_time > KICK_COOLDOWN:
            if found:
                transition(RobotState.CONFIRMED, "cooldown done")
            else:
                transition(RobotState.SEARCHING, "cooldown done, no ball")

    # =========================
    # DRAWING & METRICS
    # =========================
    latency_ms = (time.time() - t_frame_start) * 1000

    if state in (RobotState.CONFIRMED, RobotState.KICKING, RobotState.COOLDOWN) and found:
        cx, cy, r = sm(cx, prev[0]), sm(cy, prev[1]), sm(r, prev[2])
        prev[:] = cx, cy, r

        draw(out, cx, cy, r, "Ball — KICK ZONE", will_kick=True)

        ik_x, ik_y = pixel_to_cm(cx)
        cv2.putText(out, f"IK target: ({ik_x}, {ik_y})",
                    (10, 55), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 200, 255), 1)

        # Update telemetry
        update_telemetry(cx_px=cx, cy_px=cy, ik_x=ik_x, ik_y=ik_y,
                         latency_ms=round(latency_ms, 2), kick_count=kick_count)

        # Metrics log
        detection_log.append({
            "timestamp":  datetime.now().isoformat(),
            "cx_px":      cx,
            "cy_px":      cy,
            "latency_ms": round(latency_ms, 2)
        })
        if len(detection_log) % 30 == 0:
            with open(METRICS_FILE, "a", newline="") as f:
                w_csv = csv.DictWriter(f, fieldnames=["timestamp","cx_px","cy_px","latency_ms"])
                for row in detection_log[-30:]:
                    w_csv.writerow(row)
            print(f"[METRICS] Flushed {len(detection_log)} detections to CSV")

    else:
        update_telemetry(latency_ms=round(latency_ms, 2), kick_count=kick_count)

    if state == RobotState.DETECTING:
        cv2.putText(out, f"Confirming {consecutive_hits}/{CONFIRM_FRAMES}",
                    (10, 80), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 165, 255), 1)

    status_color = STATE_COLORS[state]
    cv2.putText(out, state.name, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, .8, status_color, 2)
    cv2.putText(out, "Kick zone: camera FOV",
                (10, 460), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 200, 255), 1)
    cv2.putText(out, f"Latency: {latency_ms:.1f}ms",
                (480, 30), cv2.FONT_HERSHEY_SIMPLEX, .45, (180, 180, 180), 1)

    cv2.imshow("Ball", out)
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

# Flush remaining metrics
if detection_log:
    remainder = len(detection_log) % 30
    if remainder > 0:
        with open(METRICS_FILE, "a", newline="") as f:
            w_csv = csv.DictWriter(f, fieldnames=["timestamp","cx_px","cy_px","latency_ms"])
            for row in detection_log[-remainder:]:
                w_csv.writerow(row)

    latencies = [r["latency_ms"] for r in detection_log]
    print("\n=== Vision Metrics Summary ===")
    print(f"Total confirmed detections : {len(detection_log)}")
    print(f"Avg latency per frame      : {sum(latencies)/len(latencies):.2f} ms")
    print(f"Min latency                : {min(latencies):.2f} ms")
    print(f"Max latency                : {max(latencies):.2f} ms")
    print(f"Full log saved to          : {METRICS_FILE}")
else:
    print("\n[METRICS] No confirmed detections were recorded this session.")