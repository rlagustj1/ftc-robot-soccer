"""골대 웹캠으로 보라색 공을 감지해서 점수 서버에 골을 보낸다.

예)  python goal_detector.py --goal left --server http://192.168.0.10:8000
     python goal_detector.py --list-cameras

--goal 은 '이 골대를 지키는 팀'(점수판 기준 왼쪽/오른쪽). 이 골대에 공이 들어가면 상대 팀이 1점.

조작 (카메라 창이 선택된 상태에서)
  마우스 왼쪽 클릭 : 골 판정 영역(ROI) 꼭짓점 추가     오른쪽 클릭 : 마지막 꼭짓점 삭제
  p : 색 찍기 모드 (켠 상태에서 공을 클릭하면 그 색으로 HSV 범위 자동 설정)
  m : 마스크 창 보기/숨기기     r : 영역 초기화(화면 전체)
  s : 설정 저장                 g : 테스트 골 전송      a : 강제 재무장
  q / ESC : 종료
"""
import argparse
import json
import queue
import threading
import time
import urllib.error
import urllib.request
import uuid
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
WIN = "goal camera"
WIN_SET = "settings"
WIN_MASK = "mask"

DEFAULT_CONFIG = {
    "hsv_low": [125, 70, 40],   # OpenCV HSV: H 0~179, S/V 0~255. 보라색은 대략 H 125~165
    "hsv_high": [165, 255, 255],
    "min_area": 600,            # 이 면적(픽셀) 이상 덩어리가 보이면 공으로 봄 (640x480 기준)
    "enter_frames": 4,          # 연속 몇 프레임 보여야 골로 인정할지
    "exit_seconds": 1.5,        # 공이 이 시간 동안 안 보여야 다음 골 감지 재개
    "roi": [],                  # 골 판정 영역 꼭짓점 [[x, y], ...] (0~1 비율). 비어 있으면 화면 전체
}


# ---------------------------------------------------------------- 설정
def load_config(path):
    cfg = json.loads(json.dumps(DEFAULT_CONFIG))
    if path.exists():
        cfg.update(json.loads(path.read_text(encoding="utf-8")))
        print(f"설정 불러옴: {path}")
    return cfg


def save_config(path, cfg):
    path.write_text(json.dumps(cfg, indent=2), encoding="utf-8")
    print(f"설정 저장: {path}")


# ---------------------------------------------------------------- 감지
def color_mask(hsv, low, high):
    if low[0] <= high[0]:
        return cv2.inRange(hsv, np.array(low), np.array(high))
    # 색상(H)이 0/179 경계를 넘는 범위
    a = cv2.inRange(hsv, np.array([low[0], low[1], low[2]]), np.array([179, high[1], high[2]]))
    b = cv2.inRange(hsv, np.array([0, low[1], low[2]]), np.array([high[0], high[1], high[2]]))
    return cv2.bitwise_or(a, b)


KERNEL_OPEN = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
KERNEL_CLOSE = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11, 11))  # 구멍 뚫린 공을 한 덩어리로 메움


def detect_ball(frame, cfg, roi_mask):
    """프레임에서 공을 찾는다. (공 있음 여부, 가장 큰 덩어리 윤곽선, 면적, 마스크)"""
    hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (5, 5), 0), cv2.COLOR_BGR2HSV)
    mask = color_mask(hsv, cfg["hsv_low"], cfg["hsv_high"])
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL_OPEN)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL_CLOSE, iterations=2)
    if roi_mask is not None:
        mask = cv2.bitwise_and(mask, roi_mask)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return False, None, 0.0, mask
    best = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(best)
    return area >= cfg["min_area"], best, area, mask


def build_roi_mask(cfg, shape):
    if len(cfg["roi"]) < 3:
        return None
    h, w = shape[:2]
    pts = np.array([[int(x * w), int(y * h)] for x, y in cfg["roi"]], dtype=np.int32)
    mask = np.zeros((h, w), dtype=np.uint8)
    cv2.fillPoly(mask, [pts], 255)
    return mask


class GoalLogic:
    """공이 enter_frames 연속 보이면 골 1번. 그 뒤 공이 exit_seconds 동안 안 보여야 다시 무장."""

    def __init__(self):
        self.armed = True
        self.present = 0
        self.last_seen = 0.0

    def update(self, ball, cfg, now):
        if ball:
            self.present += 1
            self.last_seen = now
        else:
            self.present = 0
        if self.armed and self.present >= cfg["enter_frames"]:
            self.armed = False
            return True
        if not self.armed and now - self.last_seen >= cfg["exit_seconds"]:
            self.armed = True
        return False


# ---------------------------------------------------------------- 서버 통신
class ServerLink:
    """화면이 멈추지 않도록 별도 스레드에서 서버로 전송."""

    def __init__(self, server, goal):
        self.server = server.rstrip("/")
        self.goal = goal
        self.queue = queue.Queue()
        self.online = False
        self.last_result = "-"
        self.info = {}
        # 같은 네트워크 통신이므로 시스템 프록시 설정 무시
        self.opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        threading.Thread(target=self._run, daemon=True).start()

    def send_goal(self):
        self.queue.put({"goal": self.goal, "id": uuid.uuid4().hex})

    def _post(self, path, payload):
        req = urllib.request.Request(
            self.server + path,
            data=json.dumps(payload).encode(),
            headers={"Content-Type": "application/json"},
        )
        with self.opener.open(req, timeout=2) as res:
            return json.loads(res.read().decode())

    def _run(self):
        next_heartbeat = 0.0
        while True:
            try:
                item = self.queue.get(timeout=0.2)
            except queue.Empty:
                item = None
            if item:
                # 같은 id로 재전송하므로 서버에서 두 번 세지 않음
                for attempt in range(10):
                    try:
                        res = self._post("/api/detect", item)
                        self.online = True
                        reason = {"ok": "GOAL SENT", "not running": "IGNORED (game not running)",
                                  "duplicate": "IGNORED (duplicate)"}.get(res.get("reason"), str(res))
                        self.last_result = f"{time.strftime('%H:%M:%S')} {reason}"
                        print(f"[골 전송] {self.last_result}")
                        break
                    except (urllib.error.URLError, OSError, ValueError) as e:
                        self.online = False
                        self.last_result = f"SEND FAILED, retry {attempt + 1}/10"
                        print(f"[골 전송 실패] {e}")
                        time.sleep(1)
                else:
                    self.last_result = "GOAL LOST - add manually on /admin"
            if time.monotonic() >= next_heartbeat:
                next_heartbeat = time.monotonic() + 1.0
                try:
                    self._post("/api/heartbeat", {"goal": self.goal, **self.info})
                    self.online = True
                except (urllib.error.URLError, OSError, ValueError):
                    self.online = False


# ---------------------------------------------------------------- 카메라
def open_camera(index, width, height, exposure):
    cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)  # 윈도우에서 가장 빨리 열림
    if not cap.isOpened():
        cap = cv2.VideoCapture(index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, 30)
    if exposure is not None:
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # DirectShow에서 수동 노출
        cap.set(cv2.CAP_PROP_EXPOSURE, exposure)
    return cap


def list_cameras():
    print("카메라 확인 중... (번호를 --camera 로 넘기면 됨)")
    for i in range(6):
        cap = cv2.VideoCapture(i, cv2.CAP_DSHOW)
        ok, frame = cap.read() if cap.isOpened() else (False, None)
        if ok:
            print(f"  {i}번: 사용 가능 ({frame.shape[1]}x{frame.shape[0]})")
        cap.release()


# ---------------------------------------------------------------- 화면
TRACKBARS = [  # (이름, 최대값, 읽기, 쓰기)
    ("H low", 179, lambda c: c["hsv_low"][0], lambda c, v: c["hsv_low"].__setitem__(0, v)),
    ("H high", 179, lambda c: c["hsv_high"][0], lambda c, v: c["hsv_high"].__setitem__(0, v)),
    ("S low", 255, lambda c: c["hsv_low"][1], lambda c, v: c["hsv_low"].__setitem__(1, v)),
    ("S high", 255, lambda c: c["hsv_high"][1], lambda c, v: c["hsv_high"].__setitem__(1, v)),
    ("V low", 255, lambda c: c["hsv_low"][2], lambda c, v: c["hsv_low"].__setitem__(2, v)),
    ("V high", 255, lambda c: c["hsv_high"][2], lambda c, v: c["hsv_high"].__setitem__(2, v)),
    ("min area", 10000, lambda c: c["min_area"], lambda c, v: c.__setitem__("min_area", v)),
    ("enter frames", 30, lambda c: c["enter_frames"], lambda c, v: c.__setitem__("enter_frames", max(1, v))),
    ("exit x0.1s", 100, lambda c: int(c["exit_seconds"] * 10), lambda c, v: c.__setitem__("exit_seconds", max(1, v) / 10)),
]


def create_trackbars(cfg):
    cv2.namedWindow(WIN_SET, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN_SET, 420, 380)
    for name, maximum, get, _ in TRACKBARS:
        cv2.createTrackbar(name, WIN_SET, int(get(cfg)), maximum, lambda v: None)


def sync_trackbars_to_cfg(cfg):
    for name, _, _, set_ in TRACKBARS:
        set_(cfg, cv2.getTrackbarPos(name, WIN_SET))


def sync_cfg_to_trackbars(cfg):
    for name, _, get, _ in TRACKBARS:
        cv2.setTrackbarPos(name, WIN_SET, int(get(cfg)))


def put(img, text, y, color=(255, 255, 255), scale=0.6):
    cv2.putText(img, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(img, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)


def main():
    parser = argparse.ArgumentParser(description="골대 웹캠 골 감지기")
    parser.add_argument("--goal", choices=["left", "right"], help="이 골대를 지키는 팀 (점수판 기준 왼쪽/오른쪽)")
    parser.add_argument("--server", default="http://127.0.0.1:8000", help="점수 서버 주소")
    parser.add_argument("--camera", type=int, default=0, help="웹캠 번호 (--list-cameras 로 확인)")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--exposure", type=float, default=None, help="수동 노출값 (예: -6). 조명 깜빡임/자동노출 문제 있을 때만")
    parser.add_argument("--config", type=Path, default=None, help="설정 파일 (기본: config_<goal>.json)")
    parser.add_argument("--list-cameras", action="store_true")
    args = parser.parse_args()

    if args.list_cameras:
        list_cameras()
        return
    if not args.goal:
        parser.error("--goal left 또는 --goal right 가 필요합니다")

    config_path = args.config or HERE / f"config_{args.goal}.json"
    cfg = load_config(config_path)
    link = ServerLink(args.server, args.goal)
    logic = GoalLogic()

    cap = open_camera(args.camera, args.width, args.height, args.exposure)
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    create_trackbars(cfg)

    ui = {"pick": False, "show_mask": False, "frame": None, "roi_dirty": True, "flash_until": 0.0}

    def on_mouse(event, x, y, flags, param):
        frame = ui["frame"]
        if frame is None:
            return
        h, w = frame.shape[:2]
        if event == cv2.EVENT_LBUTTONDOWN and ui["pick"]:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            patch = hsv[max(0, y - 4):y + 5, max(0, x - 4):x + 5].reshape(-1, 3)
            hh, ss, vv = np.median(patch, axis=0).astype(int)
            cfg["hsv_low"] = [int((hh - 12) % 180), int(max(40, ss - 80)), int(max(30, vv - 90))]
            cfg["hsv_high"] = [int((hh + 12) % 180), 255, 255]
            sync_cfg_to_trackbars(cfg)
            print(f"색 찍음: H={hh} S={ss} V={vv} -> 범위 {cfg['hsv_low']} ~ {cfg['hsv_high']}")
        elif event == cv2.EVENT_LBUTTONDOWN:
            cfg["roi"].append([round(x / w, 4), round(y / h, 4)])
            ui["roi_dirty"] = True
        elif event == cv2.EVENT_RBUTTONDOWN and cfg["roi"]:
            cfg["roi"].pop()
            ui["roi_dirty"] = True

    cv2.setMouseCallback(WIN, on_mouse)

    roi_mask = None
    fps = 0.0
    last_time = time.monotonic()
    camera_ok = True
    last_reopen = 0.0

    print(__doc__)
    while True:
        ok, frame = cap.read()
        now = time.monotonic()
        if not ok:
            camera_ok = False
            link.info = {"armed": logic.armed, "ball": False, "fps": 0, "camera": False}
            blank = np.zeros((args.height, args.width, 3), np.uint8)
            put(blank, "NO CAMERA - check USB / --camera number", 40, (0, 0, 255), 0.7)
            cv2.imshow(WIN, blank)
            if now - last_reopen > 2:
                last_reopen = now
                cap.release()
                cap = open_camera(args.camera, args.width, args.height, args.exposure)
            if cv2.waitKey(30) & 0xFF in (ord("q"), 27):
                break
            continue
        camera_ok = True
        ui["frame"] = frame

        sync_trackbars_to_cfg(cfg)
        if ui["roi_dirty"] or (roi_mask is not None and roi_mask.shape[:2] != frame.shape[:2]):
            roi_mask = build_roi_mask(cfg, frame.shape)
            ui["roi_dirty"] = False

        ball, contour, area, mask = detect_ball(frame, cfg, roi_mask)
        if logic.update(ball, cfg, now):
            link.send_goal()
            ui["flash_until"] = now + 1.0
            print("골 감지!")

        dt = now - last_time
        last_time = now
        if dt > 0:
            fps = fps * 0.9 + (1 / dt) * 0.1
        link.info = {"armed": logic.armed, "ball": bool(ball), "fps": round(fps, 1), "camera": camera_ok}

        # ---- 화면 표시
        view = frame.copy()
        h, w = view.shape[:2]
        if len(cfg["roi"]) >= 1:
            pts = np.array([[int(x * w), int(y * h)] for x, y in cfg["roi"]], dtype=np.int32)
            cv2.polylines(view, [pts], len(pts) >= 3, (0, 255, 255), 2)
            for p in pts:
                cv2.circle(view, tuple(int(v) for v in p), 4, (0, 255, 255), -1)
        if contour is not None:
            cv2.drawContours(view, [contour], -1, (0, 255, 0) if ball else (0, 140, 255), 2)
        if now < ui["flash_until"]:
            cv2.rectangle(view, (0, 0), (w - 1, h - 1), (0, 255, 0), 14)
            put(view, "GOAL!", h // 2, (0, 255, 0), 2.0)

        state = "ARMED (waiting for ball)" if logic.armed else "BALL IN GOAL (remove to re-arm)"
        put(view, f"goal={args.goal}  {state}", 24, (0, 255, 0) if logic.armed else (0, 200, 255))
        put(view, f"ball={'YES' if ball else 'no'} area={int(area)} / min {cfg['min_area']}  fps={fps:.0f}", 48)
        put(view, f"server {'ONLINE' if link.online else 'OFFLINE'}  {args.server}", 72,
            (0, 255, 0) if link.online else (0, 0, 255))
        put(view, f"last: {link.last_result}", 96)
        mode = "PICK COLOR: click the ball" if ui["pick"] else "click=ROI point  rclick=undo"
        put(view, f"[{mode}]  p=pick m=mask r=clear s=save g=test q=quit", h - 12, (200, 200, 200), 0.5)
        cv2.imshow(WIN, view)
        if ui["show_mask"]:
            cv2.imshow(WIN_MASK, mask)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break
        elif key == ord("s"):
            save_config(config_path, cfg)
        elif key == ord("p"):
            ui["pick"] = not ui["pick"]
        elif key == ord("m"):
            ui["show_mask"] = not ui["show_mask"]
            if not ui["show_mask"]:
                cv2.destroyWindow(WIN_MASK)
        elif key == ord("r"):
            cfg["roi"] = []
            ui["roi_dirty"] = True
        elif key == ord("g"):
            link.send_goal()
            ui["flash_until"] = now + 1.0
        elif key == ord("a"):
            logic.armed = True

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
