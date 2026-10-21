"""로봇 축구 점수 서버.

실행: python server.py            (기본 포트 8000)
  점수판:   http://<이 노트북 IP>:8000/
  심판용:   http://<이 노트북 IP>:8000/admin
"""
import argparse
import asyncio
import json
import socket
import time
from contextlib import asynccontextmanager
from pathlib import Path

import uvicorn
from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles

BASE = Path(__file__).resolve().parent
STATIC = BASE / "static"
STATE_FILE = BASE / "state.json"

SIDES = ("left", "right")
OTHER = {"left": "right", "right": "left"}
SIDE_KO = {"left": "왼쪽", "right": "오른쪽"}
DUPLICATE_WINDOW = 3.0  # 같은 골대에서 이 시간(초) 안에 다시 들어온 감지는 중복으로 보고 무시
DETECTOR_TIMEOUT = 5.0  # 이 시간 동안 하트비트가 없으면 감지기 오프라인


class Game:
    def __init__(self):
        self.teams = {
            "left": {"name": "RED", "color": "#e53935", "score": 0},
            "right": {"name": "BLUE", "color": "#1e88e5", "score": 0},
        }
        self.duration = 120
        self.status = "ready"  # ready | running | paused | finished
        self.remaining_at_pause = float(self.duration)
        self.end_at = None
        self.auto_only_running = True
        self.goal_seq = 0
        self.last_goal = None
        self.events = []
        self.seen_ids = {}
        self.last_detect_at = {side: 0.0 for side in SIDES}
        self.detectors = {}

    # ---------- 시간 ----------
    def remaining(self):
        if self.status == "running":
            return max(0.0, self.end_at - time.monotonic())
        return self.remaining_at_pause

    def game_clock(self):
        elapsed = self.duration - self.remaining()
        return f"{int(elapsed // 60)}:{int(elapsed % 60):02d}"

    def log(self, text):
        self.events.append({"t": time.strftime("%H:%M:%S"), "clock": self.game_clock(), "text": text})
        self.events = self.events[-100:]

    def start(self):
        if self.status in ("ready", "paused") and self.remaining_at_pause > 0:
            self.end_at = time.monotonic() + self.remaining_at_pause
            self.status = "running"
            self.log("경기 시작" if self.remaining_at_pause >= self.duration else "경기 재개")

    def pause(self):
        if self.status == "running":
            self.remaining_at_pause = self.remaining()
            self.status = "paused"
            self.log("일시정지")

    def reset(self):
        for team in self.teams.values():
            team["score"] = 0
        self.status = "ready"
        self.remaining_at_pause = float(self.duration)
        self.end_at = None
        self.last_goal = None
        self.events = []
        self.log("새 경기 준비")

    def tick(self):
        """경기 종료 시점이면 종료 처리. 상태가 바뀌었으면 True."""
        if self.status == "running" and self.remaining() <= 0:
            self.status = "finished"
            self.remaining_at_pause = 0.0
            left, right = self.teams["left"], self.teams["right"]
            if left["score"] == right["score"]:
                result = "무승부"
            else:
                winner = left if left["score"] > right["score"] else right
                result = f"{winner['name']} 승리"
            self.log(f"경기 종료 {left['score']}:{right['score']} ({result})")
            return True
        return False

    # ---------- 점수 ----------
    def add_score(self, side, delta, source):
        team = self.teams[side]
        before = team["score"]
        team["score"] = max(0, before + delta)
        if team["score"] == before:
            return
        if delta > 0:
            self.goal_seq += 1
            self.last_goal = {"seq": self.goal_seq, "team": side, "source": source}
            self.log(f"골! {team['name']} +1 ({source}) → {self.score_text()}")
        else:
            self.log(f"{team['name']} -1 ({source}) → {self.score_text()}")

    def score_text(self):
        return f"{self.teams['left']['score']}:{self.teams['right']['score']}"

    def detect(self, goal, event_id):
        if event_id and event_id in self.seen_ids:
            return self.seen_ids[event_id]
        now = time.monotonic()
        where = f"{SIDE_KO[goal]} 골대"
        if self.auto_only_running and self.status != "running":
            result = {"accepted": False, "reason": "not running"}
            self.log(f"{where} 감지 무시 (경기 중 아님)")
        elif now - self.last_detect_at[goal] < DUPLICATE_WINDOW:
            result = {"accepted": False, "reason": "duplicate"}
            self.log(f"{where} 감지 무시 (중복)")
        else:
            self.last_detect_at[goal] = now
            self.add_score(OTHER[goal], +1, f"자동·{where}")
            result = {"accepted": True, "reason": "ok"}
        if event_id:
            self.seen_ids[event_id] = result
            if len(self.seen_ids) > 500:
                self.seen_ids.pop(next(iter(self.seen_ids)))
        return result

    # ---------- 설정 ----------
    def apply_settings(self, body):
        for side in SIDES:
            patch = (body.get("teams") or {}).get(side) or {}
            if isinstance(patch.get("name"), str) and patch["name"].strip():
                self.teams[side]["name"] = patch["name"].strip()[:20]
            if isinstance(patch.get("color"), str) and patch["color"].startswith("#"):
                self.teams[side]["color"] = patch["color"][:9]
        if "duration" in body:
            duration = int(min(600, max(10, int(body["duration"]))))
            self.duration = duration
            if self.status == "ready":
                self.remaining_at_pause = float(duration)
        if "auto_only_running" in body:
            self.auto_only_running = bool(body["auto_only_running"])

    # ---------- 직렬화 ----------
    def snapshot(self):
        now = time.monotonic()
        detectors = {}
        for side in SIDES:
            info = self.detectors.get(side)
            if info:
                detectors[side] = {**info["data"], "online": now - info["seen"] < DETECTOR_TIMEOUT}
            else:
                detectors[side] = {"online": False}
        return {
            "type": "state",
            "teams": self.teams,
            "duration": self.duration,
            "status": self.status,
            "remaining_ms": int(self.remaining() * 1000),
            "auto_only_running": self.auto_only_running,
            "last_goal": self.last_goal,
            "events": self.events[-40:],
            "detectors": detectors,
        }

    def save(self):
        data = {
            "teams": self.teams,
            "duration": self.duration,
            "status": "paused" if self.status == "running" else self.status,
            "remaining": self.remaining(),
            "auto_only_running": self.auto_only_running,
            "goal_seq": self.goal_seq,
            "events": self.events,
        }
        tmp = STATE_FILE.with_suffix(".tmp")
        tmp.write_text(json.dumps(data, ensure_ascii=False, indent=1), encoding="utf-8")
        tmp.replace(STATE_FILE)

    def load(self):
        """서버가 꺼졌다 켜져도 점수가 남도록 이전 상태 복구 (진행 중이었으면 일시정지 상태로)."""
        if not STATE_FILE.exists():
            return
        try:
            data = json.loads(STATE_FILE.read_text(encoding="utf-8"))
            self.teams = data["teams"]
            self.duration = int(data["duration"])
            self.status = data["status"]
            self.remaining_at_pause = float(data["remaining"])
            self.auto_only_running = bool(data["auto_only_running"])
            self.goal_seq = int(data["goal_seq"])
            self.events = data["events"]
        except (KeyError, ValueError, TypeError, json.JSONDecodeError) as e:
            print(f"[경고] state.json 을 읽지 못해 새로 시작합니다: {e}")


game = Game()
clients: set[WebSocket] = set()


async def broadcast():
    message = json.dumps(game.snapshot(), ensure_ascii=False)
    for ws in list(clients):
        try:
            await ws.send_text(message)
        except Exception:
            clients.discard(ws)


async def changed():
    game.save()
    await broadcast()


async def ticker():
    last_sync = 0.0
    while True:
        await asyncio.sleep(0.2)
        if game.tick():
            await changed()
        elif time.monotonic() - last_sync > 2.0:
            # 주기적으로 보내서 시계 보정 + 감지기 온라인 상태 갱신
            last_sync = time.monotonic()
            await broadcast()


@asynccontextmanager
async def lifespan(_app):
    game.load()
    task = asyncio.create_task(ticker())
    yield
    task.cancel()
    game.save()


app = FastAPI(lifespan=lifespan)
app.mount("/static", StaticFiles(directory=STATIC), name="static")


@app.get("/")
async def scoreboard_page():
    return FileResponse(STATIC / "scoreboard.html")


@app.get("/admin")
async def admin_page():
    return FileResponse(STATIC / "admin.html")


@app.get("/api/state")
async def get_state():
    return game.snapshot()


@app.post("/api/detect")
async def detect(request: Request):
    body = await request.json()
    goal = body.get("goal")
    if goal not in SIDES:
        return JSONResponse({"error": "goal must be left or right"}, status_code=400)
    result = game.detect(goal, body.get("id"))
    await changed()
    return result


@app.post("/api/heartbeat")
async def heartbeat(request: Request):
    body = await request.json()
    goal = body.get("goal")
    if goal not in SIDES:
        return JSONResponse({"error": "goal must be left or right"}, status_code=400)
    data = {k: body[k] for k in ("armed", "ball", "fps", "camera") if k in body}
    game.detectors[goal] = {"seen": time.monotonic(), "data": data}
    return {"ok": True, "status": game.status}


@app.post("/api/admin")
async def admin(request: Request):
    body = await request.json()
    action = body.get("action")
    if action == "start":
        game.start()
    elif action == "pause":
        game.pause()
    elif action == "reset":
        game.reset()
    elif action == "score":
        side = body.get("team")
        if side not in SIDES:
            return JSONResponse({"error": "team must be left or right"}, status_code=400)
        game.add_score(side, 1 if int(body.get("delta", 1)) > 0 else -1, "심판")
    elif action == "settings":
        game.apply_settings(body)
    else:
        return JSONResponse({"error": f"unknown action {action}"}, status_code=400)
    await changed()
    return game.snapshot()


@app.websocket("/ws")
async def websocket(ws: WebSocket):
    await ws.accept()
    clients.add(ws)
    try:
        await ws.send_text(json.dumps(game.snapshot(), ensure_ascii=False))
        while True:
            await ws.receive_text()  # 클라이언트는 보내는 게 없음. 연결 유지용
    except WebSocketDisconnect:
        pass
    finally:
        clients.discard(ws)


def local_ips():
    ips = set()
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ips.add(info[4][0])
    except socket.gaierror:
        pass
    return sorted(ip for ip in ips if not ip.startswith("127."))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="로봇 축구 점수 서버")
    parser.add_argument("--port", type=int, default=8000)
    args = parser.parse_args()
    print("=" * 50)
    for ip in local_ips() or ["<IP 확인 실패: ipconfig 로 확인>"]:
        print(f" 점수판: http://{ip}:{args.port}/")
        print(f" 심판용: http://{ip}:{args.port}/admin")
    print("=" * 50)
    uvicorn.run(app, host="0.0.0.0", port=args.port, log_level="warning")
