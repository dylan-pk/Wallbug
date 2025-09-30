#!/usr/bin/env python3
import sys, time
from dataclasses import dataclass
from typing import Dict, Tuple, Optional

import cv2
import numpy as np
from PySide6 import QtCore, QtGui, QtWidgets

# ---------------------------
# Color / Health Computation
# ---------------------------

@dataclass
class HealthResult:
    perc_green: float
    perc_yellow: float
    perc_brown: float
    health_label: str

def hsv_mask(frame_bgr: np.ndarray, lower: Tuple[int,int,int], upper: Tuple[int,int,int]) -> np.ndarray:
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    return cv2.inRange(hsv, np.array(lower), np.array(upper))

def analyze_colors_bgr(frame_bgr: np.ndarray, roi: Optional[np.ndarray]=None) -> Dict[str, float]:
    """
    Returns % coverage of green/yellow/brown in the (optionally) masked ROI.
    """
    h, w = frame_bgr.shape[:2]
    if roi is None:
        roi = np.ones((h, w), dtype=np.uint8) * 255
    else:
        # ensure mask is 0/255 single channel
        if roi.ndim == 3:
            roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        roi = cv2.threshold(roi, 1, 255, cv2.THRESH_BINARY)[1].astype(np.uint8)

    # Reasonable HSV bands (tune for your lighting)
    green = hsv_mask(frame_bgr, (35, 40, 40), (85, 255, 255))
    yellow = hsv_mask(frame_bgr, (20, 100, 100), (35, 255, 255))
    # "Brown" in HSV; aims for lower value, moderate saturation oranges
    brown = hsv_mask(frame_bgr, (10, 80, 20), (25, 200, 160))

    # Apply ROI
    green = cv2.bitwise_and(green, roi)
    yellow = cv2.bitwise_and(yellow, roi)
    brown = cv2.bitwise_and(brown, roi)

    # Optional cleanup
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
    green = cv2.morphologyEx(green, cv2.MORPH_OPEN, k)
    yellow = cv2.morphologyEx(yellow, cv2.MORPH_OPEN, k)
    brown = cv2.morphologyEx(brown, cv2.MORPH_OPEN, k)

    total = np.count_nonzero(roi)
    total = max(total, 1)

    perc = {
        "green": 100.0 * np.count_nonzero(green) / total,
        "yellow": 100.0 * np.count_nonzero(yellow) / total,
        "brown": 100.0 * np.count_nonzero(brown) / total,
    }
    # Renormalise across detected colors (filters out non-G/Y/B)
    s = sum(perc.values())
    if s > 0:
        for k in perc:
            perc[k] = perc[k] * (100.0 / s)

    return perc

def compute_ndvi_proxy_from_rgb(rgb_frame_bgr: np.ndarray) -> np.ndarray:
    """
    Proxy NDVI using ordinary RGB webcam:
      NDVI* ≈ (G - R) / (G + R)
    This is NOT true NDVI, but gives a rough vegetation index feel for testing.
    Returns float32 in [-1, 1].
    """
    bgr = rgb_frame_bgr.astype(np.float32)
    g = bgr[:,:,1]
    r = bgr[:,:,2]
    eps = 1e-6
    ndvi_star = (g - r) / (g + r + eps)
    return np.clip(ndvi_star, -1.0, 1.0)

def health_from_percentages(perc: Dict[str, float], ndvi: Optional[np.ndarray]=None) -> HealthResult:
    g, y, b = perc["green"], perc["yellow"], perc["brown"]

    # Optional NDVI/NDVI* boost
    if ndvi is not None:
        high_ndvi_frac = float(np.mean(ndvi > 0.35)) * 100.0  # tune threshold as needed
        g += 0.25 * high_ndvi_frac
        # renormalize
        s = g + y + b
        if s > 0:
            g, y, b = (100*g/s, 100*y/s, 100*b/s)

    # Rule-based classification
    if g >= 60 and g >= y and g >= b:
        label = "Healthy"
    elif b >= 25 and b >= g:
        label = "Unhealthy"
    elif y >= 30 and y >= g:
        label = "Needs Attention"
    else:
        label = ["Healthy", "Needs Attention", "Unhealthy"][np.argmax([g, y, b])]

    return HealthResult(
        perc_green=round(g,1),
        perc_yellow=round(y,1),
        perc_brown=round(b,1),
        health_label=label
    )

def color_for_health(label: str) -> QtGui.QColor:
    if label == "Healthy":
        return QtGui.QColor(40, 170, 60)    # green
    if label == "Needs Attention":
        return QtGui.QColor(220, 190, 40)   # yellow
    if label == "Unhealthy":
        return QtGui.QColor(130, 80, 40)    # brown
    return QtGui.QColor(160, 160, 160)      # grey

# ---------------------------
# Motion-based "new plant seen"
# ---------------------------

class NewPlantDetector:
    """
    Triggers when the scene changes significantly, indicating a new plant came into view.
    Uses frame differencing with debounce.
    """
    def __init__(self, sensitivity: float=0.05, cooldown_sec: float=1.2):
        """
        sensitivity: fraction of pixels that must change (0.05 = 5%).
        cooldown_sec: minimum time between triggers.
        """
        self.prev_gray = None
        self.sensitivity = sensitivity
        self.cooldown_sec = cooldown_sec
        self.last_trigger_t = 0.0

    def update(self, frame_bgr: np.ndarray) -> Tuple[bool, float]:
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5,5), 0)
        if self.prev_gray is None:
            self.prev_gray = gray
            return False, 0.0

        diff = cv2.absdiff(gray, self.prev_gray)
        _, th = cv2.threshold(diff, 20, 255, cv2.THRESH_BINARY)
        changed_frac = float(np.count_nonzero(th)) / float(th.size)
        self.prev_gray = gray

        now = time.time()
        can_trigger = (now - self.last_trigger_t) >= self.cooldown_sec
        is_new = changed_frac >= self.sensitivity and can_trigger
        if is_new:
            self.last_trigger_t = now
        return is_new, changed_frac

# ---------------------------
# Qt Widgets
# ---------------------------

class VideoWidget(QtWidgets.QLabel):
    def __init__(self, title: str, parent=None):
        super().__init__(parent)
        self.setMinimumSize(480, 270)
        self.setAlignment(QtCore.Qt.AlignCenter)
        self.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.title = title

    def show_frame(self, frame_bgr: np.ndarray, overlay_text: Optional[str]=None):
        if frame_bgr is None:
            return
        # Convert BGR to RGB for Qt
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        qimg = QtGui.QImage(rgb.data, w, h, ch*w, QtGui.QImage.Format_RGB888)
        pix = QtGui.QPixmap.fromImage(qimg)
        # Add title + overlay
        painter = QtGui.QPainter(pix)
        painter.setPen(QtGui.QPen(QtCore.Qt.white))
        painter.setFont(QtGui.QFont("Sans", 11, QtGui.QFont.Bold))
        painter.drawText(10, 20, self.title)
        if overlay_text:
            painter.setFont(QtGui.QFont("Sans", 10))
            painter.drawText(10, 40, overlay_text)
        painter.end()
        self.setPixmap(pix.scaled(self.size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation))

class PlantTile(QtWidgets.QFrame):
    def __init__(self, index: int, parent=None):
        super().__init__(parent)
        self.index = index
        self.setFixedSize(80, 80)
        self.setFrameShape(QtWidgets.QFrame.Panel)
        self.setLineWidth(2)
        self._health = "Not Scanned"
        self._perc = (0.0, 0.0, 0.0)
        self._last_scan = None
        self.update_color(QtGui.QColor(180,180,180))  # grey
        self.setToolTip(f"Plant #{self.index+1}\nNot scanned yet.")

    def update_status(self, health: str, perc_g: float, perc_y: float, perc_b: float):
        self._health = health
        self._perc = (perc_g, perc_y, perc_b)
        self._last_scan = time.strftime("%Y-%m-%d %H:%M:%S")
        self.update_color(color_for_health(health))
        self.setToolTip(
            f"Plant #{self.index+1}\n"
            f"Health: {self._health}\n"
            f"Green: {perc_g:.1f}% | Yellow: {perc_y:.1f}% | Brown: {perc_b:.1f}%\n"
            f"Last scan: {self._last_scan}"
        )

    def update_color(self, color: QtGui.QColor):
        pal = self.palette()
        pal.setColor(QtGui.QPalette.Window, color)
        self.setAutoFillBackground(True)
        self.setPalette(pal)
        self.update()

class MainWindow(QtWidgets.QWidget):
    def __init__(self, webcam_index=0, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Plant Health Console (Webcam Test)")
        # Single webcam used for both panels
        self.cap = cv2.VideoCapture(webcam_index, cv2.CAP_ANY)
        if not self.cap.isOpened():
            # try fallback open
            self.cap = cv2.VideoCapture(webcam_index)
        # Adjust resolution (optional)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        # Video panes
        self.rgb_view = VideoWidget("RGB (webcam)")
        self.ndvi_view = VideoWidget("NDVI* (from RGB proxy)")

        # Camera "position" display (mock; can wire to real source)
        self.camera_pos_label = QtWidgets.QLabel("Camera Position: x=0.00m, y=0.00m")
        self.camera_pos_label.setStyleSheet("font-weight: 600;")

        # 4x4 grid of PlantTiles
        self.grid = QtWidgets.QGridLayout()
        self.grid.setSpacing(6)
        self.tiles = []
        for i in range(16):
            tile = PlantTile(i)
            self.tiles.append(tile)
            self.grid.addWidget(tile, i // 4, i % 4)

        grid_group = QtWidgets.QGroupBox("Plant Grid (Hover for details)")
        grid_group.setLayout(self.grid)

        # Layout
        vids = QtWidgets.QHBoxLayout()
        vids.addWidget(self.rgb_view, 1)
        vids.addWidget(self.ndvi_view, 1)

        right_panel = QtWidgets.QVBoxLayout()
        right_panel.addWidget(grid_group)
        right_panel.addStretch()
        right_panel.addWidget(self.camera_pos_label)

        main = QtWidgets.QHBoxLayout(self)
        main.addLayout(vids, 2)
        main.addLayout(right_panel, 1)

        # Timers
        self.frame_timer = QtCore.QTimer(self)
        self.frame_timer.timeout.connect(self.update_frames)
        self.frame_timer.start(33)  # ~30 FPS

        # Event-driven scanning
        self.detector = NewPlantDetector(sensitivity=0.05, cooldown_sec=1.2)
        self.current_plant_index = 0
        self._last_rgb = None
        self._last_ndvi = None

        # Keyboard shortcut: press 'N' to force next plant
        QtGui.QShortcut(QtGui.QKeySequence("N"), self, activated=self.force_next_scan)

        # Help text
        self.help_label = QtWidgets.QLabel("Tip: Move the camera or scene to trigger a new plant scan. Press 'N' to force.")
        self.help_label.setStyleSheet("color: #666;")
        right_panel.insertWidget(0, self.help_label)

    def force_next_scan(self):
        if self._last_rgb is None:
            return
        self.perform_scan(self._last_rgb, self._last_ndvi)

    # --- Replace this with real positioning data from your system
    def set_camera_position(self, x_m: float, y_m: float):
        self.camera_pos_label.setText(f"Camera Position: x={x_m:.2f}m, y={y_m:.2f}m")

    def update_frames(self):
        ret, frame = self.cap.read()
        if not ret or frame is None:
            return

        self._last_rgb = frame
        ndvi_star = compute_ndvi_proxy_from_rgb(frame)
        self._last_ndvi = ndvi_star

        # Motion detection to trigger "new plant seen"
        is_new, changed_frac = self.detector.update(frame)
        overlay = f"Change: {changed_frac*100:.1f}%  |  Press 'N' to force next"
        self.rgb_view.show_frame(frame, overlay_text=overlay)

        ndvi_vis = ((ndvi_star + 1.0) * 127.5).astype(np.uint8)  # map [-1,1] to [0,255]
        ndvi_vis = cv2.applyColorMap(ndvi_vis, cv2.COLORMAP_TURBO)
        self.ndvi_view.show_frame(ndvi_vis)

        # (Optional) update mock camera position
        t = time.time()
        x = 2.0 + 0.5*np.sin(t/5.0)
        y = 0.1*np.cos(t/7.0)
        self.set_camera_position(x, y)

        if is_new:
            self.perform_scan(frame, ndvi_star)

    def perform_scan(self, frame_bgr: np.ndarray, ndvi: Optional[np.ndarray] = None):
        roi = self.estimate_plant_roi(frame_bgr)
        perc = analyze_colors_bgr(frame_bgr, roi)
        result = health_from_percentages(perc, ndvi)

        idx = self.current_plant_index
        self.tiles[idx].update_status(result.health_label, result.perc_green, result.perc_yellow, result.perc_brown)

        self.current_plant_index = (self.current_plant_index + 1) % 16

    def estimate_plant_roi(self, frame_bgr: np.ndarray) -> np.ndarray:
        """
        Placeholder ROI estimator.
        If you already have detection/segmentation, replace this method to return a mask.
        For now, use the central square region to approximate the plant area.
        """
        h, w = frame_bgr.shape[:2]
        cx, cy = w // 2, h // 2
        sz = int(min(w, h) * 0.5)
        x0, y0 = max(cx - sz//2, 0), max(cy - sz//2, 0)
        x1, y1 = min(x0 + sz, w), min(y0 + sz, h)
        mask = np.zeros((h, w), dtype=np.uint8)
        mask[y0:y1, x0:x1] = 255
        return mask

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        try:
            self.cap.release()
        except Exception:
            pass
        return super().closeEvent(event)

def main():
    app = QtWidgets.QApplication(sys.argv)
    # Use your computer's default webcam for testing (index 0).
    win = MainWindow(webcam_index=0)
    win.resize(1280, 720)
    win.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
