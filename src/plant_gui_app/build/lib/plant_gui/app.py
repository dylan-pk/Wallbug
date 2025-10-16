#!/usr/bin/env python3
import sys, time, os, tempfile, json, subprocess
from dataclasses import dataclass
from typing import Dict, Tuple, Optional

import cv2
import numpy as np
from PySide6 import QtCore, QtGui, QtWidgets

# ---------------- Config ----------------
CROP_SIZE   = 320                 # camera analysis crop (square)
VIDEO_SIZE  = 320                 # square video panels (no side bars)
WINDOW_SIZE = (900, 580)

CANVAS_SIZE   = 360               # square paintable panel
STEP_PX       = 12                # pixels per arrow key press
BRUSH_RADIUS  = 10                # paint brush radius (px)

USE_NDVI_GREEN_BOOST = True

# Optional: use your external C++ detector
USE_CPP_DETECTOR   = False
PLANT_COLOUR_EXE   = "./plantColour"  # path to your compiled exe

# -------------- Theme (your colours) --------------
COL_BASE_BG   = "#EAE8E2"  # main background
COL_BORDER    = "#3A5A40"  # whole app border
COL_HEADING   = "#184E77"  # headings
COL_ACCENT    = "#669EBC"  # selection / accents

def apply_theme(app: QtWidgets.QApplication):
    style = f"""
        QWidget {{
            background: {COL_BASE_BG};
            color: #1f2933;
            font-size: 12px;
        }}

        QWidget#root {{
            border: 3px solid {COL_BORDER};
            border-radius: 10px;
            background: {COL_BASE_BG};
        }}

        QGroupBox {{
            background: transparent;
            color: {COL_HEADING};
            border: 1px solid rgba(0,0,0,0.15);
            border-radius: 8px;
            margin-top: 6px;
            padding-top: 6px;
            padding: 4px;
            font-weight: 600;
        }}
        QGroupBox::title {{
            subcontrol-origin: margin;
            left: 10px;
            padding: 0 4px;
            color: {COL_HEADING};
        }}

        QLabel#heading {{
            color: {COL_HEADING};
            font-weight: 700;
            font-size: 14px;
        }}

        QLabel[role="video"] {{
            background: {COL_BASE_BG};
            border: 1px solid {COL_ACCENT};
            border-radius: 8px;
        }}

        QToolTip {{
            background: {COL_BASE_BG};
            color: #0d1321;
            border: 1px solid {COL_HEADING};
        }}
    """
    app.setStyleSheet(style)

# -------------- Health / Color logic --------------
@dataclass
class HealthResult:
    perc_green: float
    perc_yellow: float
    perc_brown: float
    health_label: str

def center_crop(frame_bgr: np.ndarray, size: int = CROP_SIZE) -> np.ndarray:
    h, w = frame_bgr.shape[:2]
    s = min(h, w)
    y0 = (h - s) // 2
    x0 = (w - s) // 2
    crop = frame_bgr[y0:y0+s, x0:x0+s]
    if crop.shape[0] != size:
        crop = cv2.resize(crop, (size, size), interpolation=cv2.INTER_AREA)
    return crop

def hsv_mask(frame_bgr, lower, upper):
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    return cv2.inRange(hsv, np.array(lower), np.array(upper))

def analyze_colors_bgr_python(frame_bgr: np.ndarray) -> Dict[str, float]:
    green  = hsv_mask(frame_bgr, (35, 40, 40), (85, 255, 255))
    yellow = hsv_mask(frame_bgr, (20,100,100), (35, 255, 255))
    brown  = hsv_mask(frame_bgr, (10, 60, 20), (25, 200, 160))
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
    green  = cv2.morphologyEx(green,  cv2.MORPH_OPEN, k)
    yellow = cv2.morphologyEx(yellow, cv2.MORPH_OPEN, k)
    brown  = cv2.morphologyEx(brown,  cv2.MORPH_OPEN, k)
    total = max(frame_bgr.shape[0] * frame_bgr.shape[1], 1)
    perc = {
        "green":  100.0 * np.count_nonzero(green)  / total,
        "yellow": 100.0 * np.count_nonzero(yellow) / total,
        "brown":  100.0 * np.count_nonzero(brown)  / total,
    }
    s = sum(perc.values())
    if s > 0:
        for k_ in perc: perc[k_] *= (100.0 / s)
    return perc

def analyze_colors_bgr_cpp(frame_bgr: np.ndarray) -> Dict[str, float]:
    if not os.path.isfile(PLANT_COLOUR_EXE) or not os.access(PLANT_COLOUR_EXE, os.X_OK):
        raise FileNotFoundError(f"plantColour exe not found or not executable at: {PLANT_COLOUR_EXE}")
    fd, tmp_path = tempfile.mkstemp(suffix=".png"); os.close(fd)
    try:
        cv2.imwrite(tmp_path, frame_bgr)
        out = subprocess.check_output([PLANT_COLOUR_EXE, tmp_path, "--json"],
                                      stderr=subprocess.STDOUT, timeout=5.0)
        data = json.loads(out.decode().strip())
        g, y, b = float(data.get("green",0)), float(data.get("yellow",0)), float(data.get("brown",0))
        s = g + y + b
        return {"green": 100*g/s, "yellow": 100*y/s, "brown": 100*b/s} if s>0 else {"green":0,"yellow":0,"brown":0}
    finally:
        try: os.remove(tmp_path)
        except: pass

def detect_colors(frame_bgr: np.ndarray) -> Dict[str, float]:
    if USE_CPP_DETECTOR:
        try:   return analyze_colors_bgr_cpp(frame_bgr)
        except Exception as e:
            print(f"[WARN] C++ detector failed, fallback to Python HSV: {e}", file=sys.stderr)
    return analyze_colors_bgr_python(frame_bgr)

def compute_ndvi_proxy_from_rgb(rgb_frame_bgr: np.ndarray) -> np.ndarray:
    bgr = rgb_frame_bgr.astype(np.float32)
    g = bgr[:,:,1]; r = bgr[:,:,2]
    ndvi_star = (g - r) / (g + r + 1e-6)
    return np.clip(ndvi_star, -1.0, 1.0)

def health_from_percentages(perc: Dict[str, float], ndvi: Optional[np.ndarray]=None) -> HealthResult:
    g, y, br = perc.get("green",0.0), perc.get("yellow",0.0), perc.get("brown",0.0)
    if USE_NDVI_GREEN_BOOST and (ndvi is not None):
        high_ndvi_frac = float(np.mean(ndvi > 0.35)) * 100.0
        g += 0.25 * high_ndvi_frac
        s = g + y + br
        if s > 0:
            g, y, br = (100*g/s, 100*y/s, 100*br/s)
    if g >= 60 and g >= y and g >= br: label = "Healthy"
    elif br >= 25 and br >= g and br >= y: label = "Unhealthy"
    elif y >= 30 and y >= g: label = "Needs Attention"
    else: label = ["Healthy","Needs Attention","Unhealthy"][np.argmax([g,y,br])]
    return HealthResult(round(g,1), round(y,1), round(br,1), label)

def color_for_health(label: str) -> QtGui.QColor:
    if label == "Healthy":         return QtGui.QColor(40,170,60)   # green
    if label == "Needs Attention": return QtGui.QColor(220,190,40)  # yellow
    if label == "Unhealthy":       return QtGui.QColor(210,50,50)   # red (driven by brown)
    return QtGui.QColor(160,160,160)

# -------------- Widgets --------------
class VideoWidget(QtWidgets.QLabel):
    def __init__(self, title: str, parent=None):
        super().__init__(parent)
        self.setObjectName("video")
        self.setProperty("role", "video")
        self.setFixedSize(VIDEO_SIZE, VIDEO_SIZE)
        self.setAlignment(QtCore.Qt.AlignCenter)
        self.title = title
    def show_frame(self, frame_bgr: np.ndarray, overlay_text: Optional[str]=None):
        if frame_bgr is None: return
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        qimg = QtGui.QImage(rgb.data, rgb.shape[1], rgb.shape[0], rgb.shape[1]*3, QtGui.QImage.Format_RGB888)
        pix = QtGui.QPixmap.fromImage(qimg)
        p = QtGui.QPainter(pix)
        p.setPen(QtGui.QPen(QtGui.QColor("#222"))); p.setFont(QtGui.QFont("Inter",10,QtGui.QFont.Bold))
        p.drawText(8,16,self.title)
        if overlay_text:
            p.setFont(QtGui.QFont("Inter",9)); p.drawText(8,32,overlay_text)
        p.end()
        self.setPixmap(pix.scaled(self.width(), self.height(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation))

class PaintCanvas(QtWidgets.QWidget):
    """Continuous paint panel. The bot moves in pixel steps; each move stamps a coloured brush."""
    def __init__(self, size=CANVAS_SIZE, parent=None):
        super().__init__(parent)
        self.size_px = size
        self.setFixedSize(size, size)
        # Backing image we paint into (ARGB for transparency support)
        self.buffer = QtGui.QImage(size, size, QtGui.QImage.Format_ARGB32_Premultiplied)
        self.buffer.fill(QtCore.Qt.transparent)
        # Bot state
        self.x = size * 0.1  # start near bottom-left
        self.y = size * 0.9
        self.last_health = "Not Scanned"
        self.setMouseTracking(True)

    def clear(self):
        self.buffer.fill(QtCore.Qt.transparent)
        self.update()

    def set_bot(self, x: float, y: float):
        self.x = max(0, min(self.size_px-1, x))
        self.y = max(0, min(self.size_px-1, y))
        self.update()

    def stamp(self, color: QtGui.QColor):
        """Paint a filled circle at the current bot position."""
        painter = QtGui.QPainter(self.buffer)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        painter.setBrush(QtGui.QBrush(color))
        painter.setPen(QtCore.Qt.NoPen)
        painter.drawEllipse(QtCore.QPointF(self.x, self.y), BRUSH_RADIUS, BRUSH_RADIUS)
        painter.end()
        self.update()

    def paintEvent(self, e: QtGui.QPaintEvent):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        # subtle background grid
        p.fillRect(self.rect(), QtGui.QColor(234, 232, 226))  # match COL_BASE_BG
        grid_pen = QtGui.QPen(QtGui.QColor(0,0,0,28)); p.setPen(grid_pen)
        step = 24
        for t in range(0, self.size_px+1, step):
            p.drawLine(t, 0, t, self.size_px)
            p.drawLine(0, t, self.size_px, t)
        # painted trail
        p.drawImage(0, 0, self.buffer)
        # bot icon
        p.setPen(QtGui.QPen(QtGui.QColor("#333"), 1))
        p.setBrush(QtGui.QColor(COL_ACCENT))
        r = 8
        p.drawEllipse(QtCore.QPointF(self.x, self.y), r, r)
        p.end()

# -------------- Main Window --------------
class MainWindow(QtWidgets.QWidget):
    def __init__(self, webcam_index=0, parent=None):
        super().__init__(parent)
        self.setObjectName("root")
        self.setWindowTitle("Plant Health Console — Paint Panel + Wallbot (Teleop)")

        # Camera
        self.cap = cv2.VideoCapture(webcam_index, cv2.CAP_ANY)
        if not self.cap.isOpened(): self.cap = cv2.VideoCapture(webcam_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 960)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 540)

        # Left: video stack
        self.rgb_view  = VideoWidget("RGB (cropped 320×320)")
        self.ndvi_view = VideoWidget("NDVI* (proxy, cropped)")
        left = QtWidgets.QVBoxLayout()
        left.setSpacing(8); left.setContentsMargins(6,6,6,6)
        left.addWidget(self.rgb_view, 0, QtCore.Qt.AlignTop | QtCore.Qt.AlignHCenter)
        left.addWidget(self.ndvi_view, 0, QtCore.Qt.AlignTop | QtCore.Qt.AlignHCenter)
        left.addStretch(1)

        # Right: paint canvas
        self.canvas = PaintCanvas(CANVAS_SIZE)
        canvas_group = QtWidgets.QGroupBox("Plant Map (teleoperate to paint health along the wall)")
        canvas_layout = QtWidgets.QVBoxLayout(canvas_group)
        canvas_layout.setContentsMargins(6,2,6,6)
        canvas_layout.addWidget(self.canvas, alignment=QtCore.Qt.AlignHCenter)

        help_label = QtWidgets.QLabel(
            "Arrow keys move the wallbot and paint.\n"
            "R = reset panel. First stamp occurs automatically after the first camera frame."
        )
        help_label.setWordWrap(True)

        right = QtWidgets.QVBoxLayout()
        right.setSpacing(8); right.setContentsMargins(8,6,8,8)
        title = QtWidgets.QLabel("Controls & Status"); title.setObjectName("heading")
        right.addWidget(title)
        right.addWidget(canvas_group, 0)
        right.addWidget(help_label)

        # Main layout
        main = QtWidgets.QHBoxLayout(self)
        main.setSpacing(8); main.setContentsMargins(8,8,8,8)
        main.addLayout(left, 0)
        main.addLayout(right, 1)

        # Teleop shortcuts
        QtGui.QShortcut(QtGui.QKeySequence(QtCore.Qt.Key_Up),    self, activated=lambda: self.move_and_paint( 0, -STEP_PX))
        QtGui.QShortcut(QtGui.QKeySequence(QtCore.Qt.Key_Down),  self, activated=lambda: self.move_and_paint( 0,  STEP_PX))
        QtGui.QShortcut(QtGui.QKeySequence(QtCore.Qt.Key_Left),  self, activated=lambda: self.move_and_paint(-STEP_PX, 0))
        QtGui.QShortcut(QtGui.QKeySequence(QtCore.Qt.Key_Right), self, activated=lambda: self.move_and_paint( STEP_PX, 0))
        QtGui.QShortcut(QtGui.QKeySequence("R"),                 self, activated=self.reset_canvas)

        # Buffers
        self._last_rgb = None
        self._last_ndvi = None
        self._initial_stamp_done = False

        # Frame timer
        self.frame_timer = QtCore.QTimer(self)
        self.frame_timer.timeout.connect(self.update_frames)
        self.frame_timer.start(40)

    # --- Helpers ---
    def reset_canvas(self):
        self.canvas.clear()

    def move_and_paint(self, dx: int, dy: int):
        # Move bot
        self.canvas.set_bot(self.canvas.x + dx, self.canvas.y + dy)
        # Stamp current classification color
        self.stamp_current_color()

    def update_frames(self):
        ok, frame = self.cap.read()
        if not ok or frame is None: return
        crop = center_crop(frame, CROP_SIZE)
        self._last_rgb = crop.copy()
        ndvi_star = compute_ndvi_proxy_from_rgb(crop)
        self._last_ndvi = ndvi_star

        self.rgb_view.show_frame(crop, overlay_text="Arrow keys: move & paint | R: reset")
        ndvi_vis = ((ndvi_star + 1.0) * 127.5).astype(np.uint8)
        ndvi_vis = cv2.applyColorMap(ndvi_vis, cv2.COLORMAP_TURBO)
        self.ndvi_view.show_frame(ndvi_vis)

        if not self._initial_stamp_done:
            self._initial_stamp_done = True
            self.stamp_current_color()

    def stamp_current_color(self):
        if self._last_rgb is None: return
        perc   = detect_colors(self._last_rgb)
        result = health_from_percentages(perc, self._last_ndvi)
        color  = color_for_health(result.health_label)
        self.canvas.last_health = (f"{result.health_label} | "
                                   f"G {result.perc_green:.1f}%  "
                                   f"Y {result.perc_yellow:.1f}%  "
                                   f"B {result.perc_brown:.1f}%")
        self.canvas.setToolTip(self.canvas.last_health)
        self.canvas.stamp(color)

    def closeEvent(self, e: QtGui.QCloseEvent) -> None:
        try: self.cap.release()
        except: pass
        return super().closeEvent(e)

def main():
    app = QtWidgets.QApplication(sys.argv)
    apply_theme(app)
    win = MainWindow(webcam_index=0)
    win.resize(*WINDOW_SIZE)
    win.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
