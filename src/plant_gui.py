#!/usr/bin/env python3
import sys, time
from dataclasses import dataclass
from typing import Dict, Tuple, Optional

import cv2
import numpy as np
from PySide6 import QtCore, QtGui, QtWidgets

# ---- Config ----
CROP_SIZE = 320                  # analysis crop (square)
VIDEO_SIZE = 320                 # on-screen video widget size (square) -> no side bars
USE_NDVI_GREEN_BOOST = True      # set False to use RGB-only decisions
WINDOW_SIZE = (860, 560)         # compact window

# ---- Theme (your colours) ----
COL_BASE_BG   = "#EAE8E2"  # main background
COL_BORDER    = "#3A5A40"  # overall app border
COL_HEADING   = "#184E77"  # headings / group titles
COL_ACCENT    = "#669EBC"  # accents / selection border

def apply_theme(app: QtWidgets.QApplication):
    # Global stylesheet: child "text boxes" are transparent so they blend with parent bg
    style = f"""
        QWidget {{
            background: {COL_BASE_BG};
            color: #1f2933;
            font-size: 12px;
        }}

        /* Top-level window border */
        QWidget#root {{
            border: 3px solid {COL_BORDER};
            border-radius: 10px;
            background: {COL_BASE_BG};
        }}

        /* GroupBox: transparent fill so it matches whatever is behind it */
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

        /* Info panels / text boxes: transparent to match parent background */
        QFrame[card="true"],
        QLabel[role="panel"] {{
            background: transparent;
            border: 0px;
        }}

        /* Video widgets: match page bg; slim accent border */
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

# ---- Color / Health ----
@dataclass
class HealthResult:
    perc_green: float
    perc_yellow: float
    perc_brown: float   # we detect brown but render tile red for "Unhealthy"
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

def hsv_mask(frame_bgr: np.ndarray, lower: Tuple[int,int,int], upper: Tuple[int,int,int]) -> np.ndarray:
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    return cv2.inRange(hsv, np.array(lower), np.array(upper))

def analyze_colors_bgr(frame_bgr: np.ndarray) -> Dict[str, float]:
    # HSV bands (tune to your lighting)
    green  = hsv_mask(frame_bgr, (35, 40, 40), (85, 255, 255))     # healthy foliage
    yellow = hsv_mask(frame_bgr, (20, 100,100), (35, 255, 255))    # chlorosis/stress
    brown  = hsv_mask(frame_bgr, (10,  60, 20), (25, 200, 160))    # necrotic/dry tissue

    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
    green  = cv2.morphologyEx(green,  cv2.MORPH_OPEN, k)
    yellow = cv2.morphologyEx(yellow, cv2.MORPH_OPEN, k)
    brown  = cv2.morphologyEx(brown,  cv2.MORPH_OPEN, k)

    total = frame_bgr.shape[0] * frame_bgr.shape[1]
    total = max(total, 1)

    perc = {
        "green":  100.0 * np.count_nonzero(green)  / total,
        "yellow": 100.0 * np.count_nonzero(yellow) / total,
        "brown":  100.0 * np.count_nonzero(brown)  / total,
    }
    s = sum(perc.values())  # normalize over G/Y/Brown to ignore background
    if s > 0:
        for k_ in perc:
            perc[k_] = perc[k_] * (100.0 / s)
    return perc

def compute_ndvi_proxy_from_rgb(rgb_frame_bgr: np.ndarray) -> np.ndarray:
    """Proxy NDVI using RGB webcam: NDVI* ≈ (G - R) / (G + R) -> [-1, 1]."""
    bgr = rgb_frame_bgr.astype(np.float32)
    g = bgr[:,:,1]; r = bgr[:,:,2]
    eps = 1e-6
    ndvi_star = (g - r) / (g + r + eps)
    return np.clip(ndvi_star, -1.0, 1.0)

def health_from_percentages(perc: Dict[str, float], ndvi: Optional[np.ndarray]=None) -> HealthResult:
    g, y, br = perc["green"], perc["yellow"], perc["brown"]

    if USE_NDVI_GREEN_BOOST and (ndvi is not None):
        high_ndvi_frac = float(np.mean(ndvi > 0.35)) * 100.0
        g += 0.25 * high_ndvi_frac
        s = g + y + br
        if s > 0:
            g, y, br = (100*g/s, 100*y/s, 100*br/s)

    # Rule-based classification (brown drives "Unhealthy" → red tile)
    if g >= 60 and g >= y and g >= br:
        label = "Healthy"
    elif br >= 25 and br >= g and br >= y:
        label = "Unhealthy"
    elif y >= 30 and y >= g:
        label = "Needs Attention"
    else:
        label = ["Healthy", "Needs Attention", "Unhealthy"][np.argmax([g, y, br])]

    return HealthResult(
        perc_green=round(g,1),
        perc_yellow=round(y,1),
        perc_brown=round(br,1),
        health_label=label
    )

def tile_color_for_health(label: str) -> QtGui.QColor:
    if label == "Healthy":         return QtGui.QColor(40, 170, 60)   # green
    if label == "Needs Attention": return QtGui.QColor(220, 190, 40)  # yellow
    if label == "Unhealthy":       return QtGui.QColor(210, 50, 50)   # red (driven by brown)
    return QtGui.QColor(160, 160, 160)                                # grey

# ---- Qt Widgets ----
class VideoWidget(QtWidgets.QLabel):
    """Fixed-size square video panel so there are no side bars."""
    def __init__(self, title: str, parent=None):
        super().__init__(parent)
        self.setObjectName("video")
        self.setProperty("role", "video")
        self.setFixedSize(VIDEO_SIZE, VIDEO_SIZE)  # exact size -> no side bars
        self.setAlignment(QtCore.Qt.AlignCenter)
        self.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.title = title

    def show_frame(self, frame_bgr: np.ndarray, overlay_text: Optional[str]=None):
        if frame_bgr is None:
            return
        # Ensure the image is the same aspect as the widget (square)
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        qimg = QtGui.QImage(rgb.data, rgb.shape[1], rgb.shape[0], rgb.shape[1]*3, QtGui.QImage.Format_RGB888)
        pix = QtGui.QPixmap.fromImage(qimg)
        # Draw title/overlay directly onto pix
        painter = QtGui.QPainter(pix)
        painter.setPen(QtGui.QPen(QtGui.QColor("#222222")))
        painter.setFont(QtGui.QFont("Inter", 10, QtGui.QFont.Bold))
        painter.drawText(8, 16, self.title)
        if overlay_text:
            painter.setFont(QtGui.QFont("Inter", 9))
            painter.drawText(8, 32, overlay_text)
        painter.end()
        # Scale into a square of the same size as the crop -> no side bars
        self.setPixmap(pix.scaled(self.width(), self.height(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation))

class PlantTile(QtWidgets.QFrame):
    def __init__(self, index: int, parent=None):
        super().__init__(parent)
        self.index = index
        self.setFixedSize(68, 68)
        self.setFrameShape(QtWidgets.QFrame.Panel)
        self.setLineWidth(2)
        self._health = "Not Scanned"
        self._perc = (0.0, 0.0, 0.0)
        self._last_scan = None
        self._selected = False
        self._bg_color = QtGui.QColor(180,180,180)  # start grey
        self._apply_styles()
        self.setToolTip(f"Plant #{self.index+1}\nNot scanned yet.")

    def is_scanned(self) -> bool:
        return self._health != "Not Scanned"

    def set_selected(self, on: bool):
        self._selected = on
        self._apply_styles()

    def update_status(self, health: str, perc_g: float, perc_y: float, perc_brown: float):
        if self.is_scanned():
            return  # lock after first scan
        self._health = health
        self._perc = (perc_g, perc_y, perc_brown)
        self._last_scan = time.strftime("%Y-%m-%d %H:%M:%S")
        self._bg_color = tile_color_for_health(health)
        self._apply_styles()
        self.setToolTip(
            f"Plant #{self.index+1}\n"
            f"Health: {self._health}\n"
            f"Green: {perc_g:.1f}% | Yellow: {perc_y:.1f}% | Brown: {perc_brown:.1f}%\n"
            f"Last scan: {self._last_scan}"
        )

    def _apply_styles(self):
        r, g, b, a = self._bg_color.red(), self._bg_color.green(), self._bg_color.blue(), 255
        border_col = COL_ACCENT if self._selected else "#333333"
        self.setStyleSheet(f"background-color: rgba({r},{g},{b},{a}); border: 2px solid {border_col}; border-radius: 6px;")

# ---- Main Window (Teleop only) ----
class MainWindow(QtWidgets.QWidget):
    def __init__(self, webcam_index=0, parent=None):
        super().__init__(parent)
        self.setObjectName("root")
        self.setWindowTitle("Plant Health Console (Teleop Only — Compact)")
        # Webcam capture
        self.cap = cv2.VideoCapture(webcam_index, cv2.CAP_ANY)
        if not self.cap.isOpened():
            self.cap = cv2.VideoCapture(webcam_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  960)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 540)

        # Video panes (stacked RGB on top, NDVI* below) — fixed square size
        self.rgb_view  = VideoWidget("RGB (cropped 320×320)")
        self.ndvi_view = VideoWidget("NDVI* (proxy, cropped)")
        video_column = QtWidgets.QVBoxLayout()
        video_column.setSpacing(8)
        video_column.setContentsMargins(6, 6, 6, 6)
        video_column.addWidget(self.rgb_view, 0, QtCore.Qt.AlignTop | QtCore.Qt.AlignHCenter)
        video_column.addWidget(self.ndvi_view, 0, QtCore.Qt.AlignTop | QtCore.Qt.AlignHCenter)
        video_column.addStretch(1)

        # 4×4 grid (compact spacing)
        self.grid = QtWidgets.QGridLayout()
        self.grid.setSpacing(6)
        self.grid.setContentsMargins(6, 2, 6, 6)
        self.tiles = []
        for i in range(16):
            tile = PlantTile(i)
            self.tiles.append(tile)
            self.grid.addWidget(tile, i // 4, i % 4)

        grid_group = QtWidgets.QGroupBox("Plant Grid (Arrow keys to move; hover for details)")
        # Slightly reduced top/bottom margins to nudge the title upward
        grid_group.setContentsMargins(6, 2, 6, 6)
        grid_group.setLayout(self.grid)

        # Right panel (transparent card)
        right_card = QtWidgets.QFrame()
        right_card.setProperty("card", True)
        right_layout = QtWidgets.QVBoxLayout(right_card)
        right_layout.setSpacing(8)
        right_layout.setContentsMargins(8, 6, 8, 8)
        title = QtWidgets.QLabel("Controls & Status")
        title.setObjectName("heading")
        help_label = QtWidgets.QLabel(
            "Teleoperation only:\n• Arrow keys move the selector\n• Entering a tile scans once and locks colour\n"
            "• First tile scans automatically on startup\n• Brown → Unhealthy (red tile)"
        )
        help_label.setWordWrap(True)
        help_label.setProperty("role", "panel")

        right_layout.addWidget(title)
        right_layout.addWidget(grid_group, 1)
        right_layout.addWidget(help_label)

        # Main layout: videos left (fixed narrow), right card
        main = QtWidgets.QHBoxLayout(self)
        main.setSpacing(8)
        main.setContentsMargins(8, 8, 8, 8)  # leaves space to show the border
        main.addLayout(video_column, 0)
        main.addWidget(right_card, 1)

        # Teleoperation state (row, col); bottom-left start => row=3, col=0
        self.row = 3
        self.col = 0
        self.update_selection()

        # Arrow-key controls (manual only)
        QtGui.QShortcut(QtGui.QKeySequence(QtCore.Qt.Key_Up),    self, activated=self.move_up)
        QtGui.QShortcut(QtGui.QKeySequence(QtCore.Qt.Key_Down),  self, activated=self.move_down)
        QtGui.QShortcut(QtGui.QKeySequence(QtCore.Qt.Key_Left),  self, activated=self.move_left)
        QtGui.QShortcut(QtGui.QKeySequence(QtCore.Qt.Key_Right), self, activated=self.move_right)

        # Buffers
        self._last_rgb = None
        self._last_ndvi = None
        self._initial_scan_done = False

        # Timer
        self.frame_timer = QtCore.QTimer(self)
        self.frame_timer.timeout.connect(self.update_frames)
        self.frame_timer.start(40)  # ~25 FPS

    # Selection helpers
    def tile_index(self, r: int, c: int) -> int:
        return r * 4 + c

    def update_selection(self):
        for i, t in enumerate(self.tiles):
            r, c = divmod(i, 4)
            t.set_selected(r == self.row and c == self.col)

    # Movement (manual) — scanning happens on each move
    def move_up(self):
        if self.row > 0:
            self.row -= 1
            self.update_selection()
            self.scan_current_tile()

    def move_down(self):
        if self.row < 3:
            self.row += 1
            self.update_selection()
            self.scan_current_tile()

    def move_left(self):
        if self.col > 0:
            self.col -= 1
            self.update_selection()
            self.scan_current_tile()

    def move_right(self):
        if self.col < 3:
            self.col += 1
            self.update_selection()
            self.scan_current_tile()

    # Frame updates
    def update_frames(self):
        ret, frame = self.cap.read()
        if not ret or frame is None:
            return

        crop = center_crop(frame, CROP_SIZE)     # 320x320 analysis crop
        self._last_rgb = crop.copy()

        ndvi_star = compute_ndvi_proxy_from_rgb(crop)
        self._last_ndvi = ndvi_star

        self.rgb_view.show_frame(crop, overlay_text="Arrow keys to move/select")
        ndvi_vis = ((ndvi_star + 1.0) * 127.5).astype(np.uint8)
        ndvi_vis = cv2.applyColorMap(ndvi_vis, cv2.COLORMAP_TURBO)
        self.ndvi_view.show_frame(ndvi_vis)

        # First tile auto-scan on first valid frame
        if not self._initial_scan_done:
            self._initial_scan_done = True
            self.scan_current_tile()

    def scan_current_tile(self):
        if self._last_rgb is None:
            return
        idx = self.tile_index(self.row, self.col)
        if self.tiles[idx].is_scanned():
            return  # lock after first scan

        frame = self._last_rgb
        ndvi = self._last_ndvi

        perc = analyze_colors_bgr(frame)
        result = health_from_percentages(perc, ndvi)

        self.tiles[idx].update_status(
            result.health_label, result.perc_green, result.perc_yellow, result.perc_brown
        )

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        try:
            self.cap.release()
        except Exception:
            pass
        return super().closeEvent(event)

def main():
    app = QtWidgets.QApplication(sys.argv)
    apply_theme(app)
    win = MainWindow(webcam_index=0)  # change if your webcam isn't index 0
    win.resize(*WINDOW_SIZE)
    win.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
