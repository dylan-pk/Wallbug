#!/usr/bin/env python3
"""
Nav2 Mission GUI (PySide6 + ROS 2)

Purpose
-------
AMission panel that loads waypoints from CSV and navigates with Nav2.
Adds a live map with custom icons:
- Robot icon = ladybug
- Waypoints = leaf icons: default "clear" until scanned, then green/yellow/red by scan result
- Hover tooltip shows scan percentages per waypoint

Layout
------
- Left: RGB + NDVI camera tiles
- Right: Mission panel (Load, Start/Pause/Resume/Skip/Cancel, Loop) + live map
- Map: renders /map (OccupancyGrid), shows waypoints + robot pose from /amcl_pose or /odom

Key ROS Topics/Interfaces
-------------------------
- Action client: /navigate_to_pose (nav2_msgs/action/NavigateToPose)
- Subscriptions: /map (nav_msgs/msg/OccupancyGrid); /amcl_pose (geometry_msgs/msg/PoseWithCovarianceStamped)
                 fallback to /odom (nav_msgs/msg/Odometry) if /amcl_pose unavailable
- Publisher: /mission/photo_request (std_msgs/String) when a waypoint with photo=true is reached
- Subscription: /plant_scan_result (std_msgs/String) -> either CSV "name,green,yellow,brown" or JSON

Waypoint File Format (CSV)
--------------------------
Header row required. Columns:
  name,x,y,yaw_deg,frame_id,hold_sec,photo
"""

import csv
import math
import os
import sys
from dataclasses import dataclass
from typing import List, Optional

from PySide6 import QtCore, QtGui, QtWidgets

# --- NEW (camera display deps) ---
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# ROS 2
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
# (optional) from rclpy.qos import QoSDurabilityPolicy

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import String


# -----------------------------
# Data structures
# -----------------------------
@dataclass
class Waypoint:
    name: str
    x: float
    y: float
    yaw_deg: float = 0.0
    frame_id: str = "map"
    hold_sec: float = 0.0
    photo: bool = False

    def to_pose_stamped(self) -> PoseStamped:
        ps = PoseStamped()
        ps.header.frame_id = self.frame_id or "map"
        ps.pose.position.x = float(self.x)
        ps.pose.position.y = float(self.y)
        # yaw (deg) -> quaternion (z,w)
        yaw_rad = math.radians(self.yaw_deg)
        qz = math.sin(yaw_rad / 2.0)
        qw = math.cos(yaw_rad / 2.0)
        ps.pose.orientation = Quaternion(x=0.0, y=0.0, z=qz, w=qw)
        return ps


# -----------------------------
# ROS wrapper as QObject (composition, not multiple inheritance)
# -----------------------------
class MissionRos(QtCore.QObject):
    # Qt signals (QObject required)
    photoRequested    = QtCore.Signal(str)                        # waypoint name
    feedbackText      = QtCore.Signal(str)
    goalStatus        = QtCore.Signal(str)
    robotPoseUpdated  = QtCore.Signal(float, float, float)        # x, y, yaw (rad)
    mapUpdated        = QtCore.Signal(QtGui.QImage, float, float) # image, origin_x, origin_y
    plantScanned      = QtCore.Signal(str, float, float, float)   # name, green%, yellow%, brown%
    # --- NEW camera frame signals ---
    rgbFrame          = QtCore.Signal(QtGui.QImage)               # RGB pane (QImage)
    ndviFrame         = QtCore.Signal(QtGui.QImage)               # NDVI pane (QImage)

    def __init__(self):
        super().__init__()
        # Create an rclpy node we’ll use for all ROS work
        self._node = rclpy.create_node('mission_gui_node')

        # Nav2 action client lives on the node
        self._action_client = ActionClient(self._node, NavigateToPose, 'navigate_to_pose')
        self._current_goal_handle = None

        # QoS and subs
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.history = HistoryPolicy.KEEP_LAST

        # Pose sources
        self._node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl_pose, 10)
        self._node.create_subscription(Odometry, '/odom', self._on_odom, qos)

        # Map
        self._node.create_subscription(OccupancyGrid, '/map', self._on_map, 10)
        self._map_meta = None  # (res, w, h, origin_x, origin_y)

        # Optional: publish photo requests
        self._photo_pub = self._node.create_publisher(String, '/mission/photo_request', 10)

        # Subscribe to plant scan results (string payload: CSV or JSON)
        self._node.create_subscription(String, '/plant_scan_result', self._on_scan_result, 10)

        # --- NEW: camera subscribers (created via start_cameras) ---
        self._bridge = CvBridge()
        self._sub_rgb = None
        self._sub_ndvi = None
        self._rgb_topic = '/front_rgb/image_raw'
        self._ndvi_topic = '/front_ndvi/image_raw'

    @property
    def node(self):
        return self._node

    # ---------- Pose callbacks ----------
    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = self._yaw_from_quat(q.x, q.y, q.z, q.w)
        self.robotPoseUpdated.emit(x, y, yaw)

    def _on_odom(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = self._yaw_from_quat(q.x, q.y, q.z, q.w)
        self.robotPoseUpdated.emit(x, y, yaw)

    # ---------- Map callback ----------
    def _on_map(self, msg: OccupancyGrid):
        res = msg.info.resolution
        w   = msg.info.width
        h   = msg.info.height
        ox  = msg.info.origin.position.x
        oy  = msg.info.origin.position.y
        data = msg.data  # list[int] in [-1,100]

        # Render as RGB32 for easy qRgb writes
        img = QtGui.QImage(w, h, QtGui.QImage.Format_RGB32)
        for yy in range(h):
            base = yy * w
            for xx in range(w):
                v = data[base + xx]
                gray = 205 if v < 0 else int(255 - (v / 100.0) * 255)  # unknown=mid, occupied=dark
                img.setPixel(xx, h - 1 - yy, QtGui.qRgb(gray, gray, gray))  # flip Y for screen coords

        self._map_meta = (res, w, h, ox, oy)
        self.mapUpdated.emit(img, ox, oy)

    # ---------- Actions ----------
    def send_goal(self, wp: Waypoint):
        if not self._action_client.server_is_ready():
            self.goalStatus.emit('Nav2 action server not ready')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = wp.to_pose_stamped()
        goal_msg.pose.header.stamp = self._node.get_clock().now().to_msg()

        self.feedbackText.emit(f"Sending goal: {wp.name} @ ({wp.x:.2f}, {wp.y:.2f}) yaw {wp.yaw_deg:.1f}°")
        fut = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=lambda fb: self._on_feedback(wp, fb)
        )
        fut.add_done_callback(lambda f: self._on_goal_response(wp, f))

    def _on_goal_response(self, wp: Waypoint, fut):
        self._current_goal_handle = fut.result()
        if not self._current_goal_handle or not self._current_goal_handle.accepted:
            self.goalStatus.emit(f"Goal rejected: {wp.name}")
            return
        self.goalStatus.emit(f"Goal accepted: {wp.name}")
        rfut = self._current_goal_handle.get_result_async()
        rfut.add_done_callback(lambda r: self._on_result(wp, r))

    def _on_feedback(self, wp: Waypoint, feedback):
        fb = feedback.feedback
        dist = getattr(fb, 'distance_remaining', float('nan'))
        self.feedbackText.emit(f"→ {wp.name}: remaining {dist:.2f} m")

    def _on_result(self, wp: Waypoint, rfut):
        status = rfut.result().status
        if status == 4:  # STATUS_SUCCEEDED
            self.goalStatus.emit(f"Reached: {wp.name}")
            if wp.photo:
                self._photo_pub.publish(String(data=f"Photo at {wp.name}"))
                self.photoRequested.emit(wp.name)
            # scanning happens externally; results come on /plant_scan_result
        else:
            self.goalStatus.emit(f"Goal finished with status={status} for {wp.name}")

    def cancel_active_goal(self):
        if self._current_goal_handle:
            self._current_goal_handle.cancel_goal_async()

    @staticmethod
    def _yaw_from_quat(x, y, z, w) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    # ---------- Scan results ----------
    def _on_scan_result(self, msg: String):
        payload = msg.data.strip()
        name = None; g=y=b=None
        try:
            if payload.startswith('{'):
                import json
                d = json.loads(payload)
                name = d.get('name') or d.get('waypoint')
                g = float(d.get('green', d.get('green_pct')))
                y = float(d.get('yellow', d.get('yellow_pct')))
                b = float(d.get('brown', d.get('brown_pct', d.get('red', 0))))
            else:
                # CSV: name,green,yellow,brown
                parts = [p.strip() for p in payload.split(',')]
                if len(parts) >= 4:
                    name, g, y, b = parts[0], float(parts[1]), float(parts[2]), float(parts[3])
        except Exception:
            return
        if name is not None and all(v is not None for v in (g,y,b)):
            self.plantScanned.emit(name, g, y, b)

    # ---------- NEW: Camera helpers ----------
    def _to_qimage(self, arr: np.ndarray) -> QtGui.QImage:
        """
        Convert a numpy image to QImage (handles mono8 or RGB888).
        """
        if arr.ndim == 2:  # mono8
            h, w = arr.shape
            return QtGui.QImage(arr.data, w, h, w, QtGui.QImage.Format_Grayscale8).copy()
        elif arr.ndim == 3 and arr.shape[2] == 3:
            h, w, ch = arr.shape
            return QtGui.QImage(arr.data, w, h, ch * w, QtGui.QImage.Format_RGB888).copy()
        else:
            arr8 = arr.astype(np.uint8)
            h, w = arr8.shape[:2]
            return QtGui.QImage(arr8.data, w, h, w, QtGui.QImage.Format_Grayscale8).copy()

    def start_cameras(self, rgb_topic: str = None, ndvi_topic: str = None):
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.history = HistoryPolicy.KEEP_LAST
        # Optionally: qos.durability = QoSDurabilityPolicy.VOLATILE

        if rgb_topic:
            self._rgb_topic = rgb_topic
        if ndvi_topic:
            self._ndvi_topic = ndvi_topic

        self.stop_cameras()

        def _cb_rgb(msg: Image):
            try:
                img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            except Exception:
                img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            self.rgbFrame.emit(self._to_qimage(img))

        def _cb_ndvi(msg: Image):
            img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            self.ndviFrame.emit(self._to_qimage(img))

        self._sub_rgb  = self._node.create_subscription(Image, self._rgb_topic,  _cb_rgb,  qos)
        self._sub_ndvi = self._node.create_subscription(Image, self._ndvi_topic, _cb_ndvi, qos)

    def stop_cameras(self):
        if self._sub_rgb is not None:
            try:
                self._node.destroy_subscription(self._sub_rgb)
            except Exception:
                pass
        if self._sub_ndvi is not None:
            try:
                self._node.destroy_subscription(self._sub_ndvi)
            except Exception:
                pass
        self._sub_rgb = None
        self._sub_ndvi = None


# -----------------------------
# Map View (QGraphicsView)
# -----------------------------
class MapView(QtWidgets.QGraphicsView):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setRenderHint(QtGui.QPainter.Antialiasing)
        self._scene = QtWidgets.QGraphicsScene(self)
        self.setScene(self._scene)
        self._map_pixmap_item = None
        self._origin = (0.0, 0.0)
        self._resolution = 0.05  # default (used if map missing)
        self._waypoints: List[Waypoint] = []
        self._robot_pose = (0.0, 0.0, 0.0)

        # Icon sizing (tweak to taste)
        self._robot_scale = 0.55
        self._leaf_scale  = 0.50

        # Icons
        self._robot_pixmap: Optional[QtGui.QPixmap] = None
        self._leaf_clear: Optional[QtGui.QPixmap] = None
        self._leaf_green: Optional[QtGui.QPixmap] = None
        self._leaf_yellow: Optional[QtGui.QPixmap] = None
        self._leaf_red: Optional[QtGui.QPixmap] = None
        self._scan: dict[str, tuple[float,float,float]] = {}
        self._current_goal: Optional[tuple[float,float]] = None

        # Smooth pan/zoom
        self.setDragMode(QtWidgets.QGraphicsView.ScrollHandDrag)
        self.setTransformationAnchor(QtWidgets.QGraphicsView.AnchorUnderMouse)

    def wheelEvent(self, event: QtGui.QWheelEvent) -> None:
        factor = 1.25 if event.angleDelta().y() > 0 else 0.8
        self.scale(factor, factor)

    # --- Icon loaders ---
    def load_robot_icon(self, path: str):
        pm = QtGui.QPixmap(path)
        self._robot_pixmap = pm if not pm.isNull() else None
        self.redraw()

    def set_leaf_icons(self, clear_path: str, green_path: str, yellow_path: str, red_path: str):
        self._leaf_clear  = QtGui.QPixmap(clear_path)  if clear_path  else None
        self._leaf_green  = QtGui.QPixmap(green_path)  if green_path  else None
        self._leaf_yellow = QtGui.QPixmap(yellow_path) if yellow_path else None
        self._leaf_red    = QtGui.QPixmap(red_path)    if red_path    else None
        self.redraw()

    def set_current_goal(self, x: Optional[float], y: Optional[float]):
        self._current_goal = None if x is None or y is None else (x, y)
        self.redraw()

    def set_scan_result(self, name: str, green: float, yellow: float, brown: float):
        self._scan[name] = (green, yellow, brown)
        self.redraw()

    def update_map(self, qimage: Optional[QtGui.QImage], origin_x: float, origin_y: float, resolution: Optional[float] = None):
        if qimage is not None:
            pix = QtGui.QPixmap.fromImage(qimage)
            if self._map_pixmap_item is None:
                self._map_pixmap_item = self._scene.addPixmap(pix)
            else:
                self._map_pixmap_item.setPixmap(pix)
            # Place map so that world (origin_x,origin_y) -> scene (0,0)
            if self._resolution != 0:
                self._map_pixmap_item.setOffset(-origin_x / self._resolution, -(origin_y) / self._resolution)
        if resolution:
            self._resolution = resolution
        self._origin = (origin_x, origin_y)
        self.redraw()

    def set_waypoints(self, wps: List[Waypoint]):
        self._waypoints = wps
        self.redraw()

    def set_robot_pose(self, x: float, y: float, yaw: float):
        self._robot_pose = (x, y, yaw)
        self.redraw()

    def world_to_scene(self, x: float, y: float) -> QtCore.QPointF:
        # Convert world meters -> map pixels; assume 1/resolution pixels per meter
        sx = (x - self._origin[0]) / self._resolution
        sy = (y - self._origin[1]) / self._resolution
        # Flip Y to match QGraphics coordinates (top-down)
        return QtCore.QPointF(sx, -sy)

    def redraw(self):
        # Clear all items except the map pixmap (if present)
        for item in list(self._scene.items()):
            if item is not self._map_pixmap_item:
                self._scene.removeItem(item)

        pen_wp = QtGui.QPen(QtCore.Qt.darkCyan, 2)
        pen_path = QtGui.QPen(QtCore.Qt.gray, 1, QtCore.Qt.DashLine)
        pen_robot = QtGui.QPen(QtCore.Qt.red, 2)
        brush_wp = QtGui.QBrush(QtCore.Qt.cyan)
        brush_robot = QtGui.QBrush(QtCore.Qt.red)

        # Draw path between waypoints
        if len(self._waypoints) > 1:
            for i in range(len(self._waypoints) - 1):
                a = self.world_to_scene(self._waypoints[i].x, self._waypoints[i].y)
                b = self.world_to_scene(self._waypoints[i + 1].x, self._waypoints[i + 1].y)
                self._scene.addLine(QtCore.QLineF(a, b), pen_path)

        # Draw waypoints (leaf icons + tooltip) — scaled & centered
        for wp in self._waypoints:
            p = self.world_to_scene(wp.x, wp.y)
            pm = self._leaf_clear
            tip = f"{wp.name}: not scanned"
            if wp.name in self._scan:
                g, y, b = self._scan[wp.name]
                if self._leaf_green and g >= max(y, b):
                    pm = self._leaf_green
                elif self._leaf_yellow and y >= max(g, b):
                    pm = self._leaf_yellow
                elif self._leaf_red:
                    pm = self._leaf_red
                tip = f"{wp.name}:\n  green : {g:.1f}%\n  yellow: {y:.1f}%\n  brown : {b:.1f}%"
            if pm is not None and not pm.isNull():
                item = QtWidgets.QGraphicsPixmapItem(pm)
                w = pm.width(); h = pm.height()
                item.setTransformOriginPoint(w/2, h/2)
                item.setScale(self._leaf_scale)
                item.setRotation(0)            # leaves upright
                item.setOffset(-w/2, -h/2)     # center in item coords
                item.setPos(p)                 # then place at scene coords
                item.setToolTip(tip)
                self._scene.addItem(item)
            else:
                r = 0.15 / self._resolution
                ellipse = self._scene.addEllipse(p.x()-r, p.y()-r, 2*r, 2*r, pen_wp, brush_wp)
                ellipse.setToolTip(tip)

        # Draw robot (ladybug fixed UP, scaled & centered)
        rx, ry, rYaw = self._robot_pose
        pr = self.world_to_scene(rx, ry)
        if self._robot_pixmap is not None:
            item = QtWidgets.QGraphicsPixmapItem(self._robot_pixmap)
            w = self._robot_pixmap.width()
            h = self._robot_pixmap.height()
            item.setTransformOriginPoint(w/2, h/2)
            item.setScale(self._robot_scale)
            item.setRotation(0)        # always face up
            item.setOffset(-w/2, -h/2) # center
            item.setPos(pr)
            self._scene.addItem(item)
        else:
            rr = 0.2 / self._resolution  # 20 cm radius in pixels
            self._scene.addEllipse(pr.x() - rr, pr.y() - rr, 2 * rr, 2 * rr, pen_robot, brush_robot)

# -----------------------------
# Main Window (with theme + camera tiles)
# -----------------------------
class MissionWindow(QtWidgets.QMainWindow):
    def __init__(self, ros: MissionRos):
        super().__init__()
        self.ros = ros
        self.setWindowTitle("Mission GUI (Nav2 + Cameras)")
        self.resize(1200, 800)

        # ---- Theme ----
        COL_BASE_BG   = "#EAE8E2"   # main background
        COL_BORDER    = "#3A5A40"   # green border
        COL_HEADING   = "#184E77"   # blue headings
        COL_TEXT      = "#1B1B1B"
        COL_PANEL     = "#F7F5EF"   # soft panel bg
        COL_ACCENT    = "#3A5A40"   # teal buttons
        COL_ACCENT_D  = "#3A5A40"   # hover

        self.setStyleSheet(f"""
            QMainWindow {{ background: {COL_BASE_BG}; }}
            QGroupBox {{
                border: 1px solid {COL_BORDER};
                border-radius: 10px; margin-top: 10px; padding: 8px;
            }}
            QGroupBox::title {{
                subcontrol-origin: margin; left: 12px; padding: 2px 6px;
                color: {COL_HEADING}; font-weight: 600; letter-spacing: 0.2px;
            }}
            QPushButton {{
                background: {COL_ACCENT}; color: white; border: none;
                border-radius: 10px; padding: 8px 12px;
            }}
            QPushButton:hover {{ background: {COL_ACCENT_D}; }}
            QListWidget {{ background: {COL_PANEL}; border: 1px solid {COL_BORDER}; border-radius: 8px; }}
            QLabel {{ color: {COL_TEXT}; }}
        """)

        # State
        self._waypoints: List[Waypoint] = []
        self._current_index = -1
        self._loop = False
        self._paused = False

        # Root layout: splitter (left cameras, right mission+map)
        splitter = QtWidgets.QSplitter()
        splitter.setOrientation(QtCore.Qt.Horizontal)
        self.setCentralWidget(splitter)

        # Left: camera panel (RGB + NDVI stacked)
        self.camera_panel = self._build_camera_panel()
        splitter.addWidget(self.camera_panel)

        # Right: mission controls + map
        right = QtWidgets.QWidget()
        right_layout = QtWidgets.QVBoxLayout(right)
        splitter.addWidget(right)

        # Mission controls
        self.mission_group = QtWidgets.QGroupBox("Mission")
        mg = QtWidgets.QGridLayout(self.mission_group)

        self.btn_load = QtWidgets.QPushButton("Load Waypoints")
        self.btn_start = QtWidgets.QPushButton("Start Mission")
        self.btn_pause = QtWidgets.QPushButton("Pause")
        self.btn_resume = QtWidgets.QPushButton("Resume")
        self.btn_skip = QtWidgets.QPushButton("Skip →")
        self.btn_cancel = QtWidgets.QPushButton("Cancel Goal")
        self.chk_loop = QtWidgets.QCheckBox("Loop")

        self.list_wps = QtWidgets.QListWidget()
        self.label_status = QtWidgets.QLabel("Status: idle")
        self.label_feedback = QtWidgets.QLabel("Feedback: …")

        row = 0
        mg.addWidget(self.btn_load, row, 0)
        mg.addWidget(self.chk_loop, row, 1)
        row += 1
        mg.addWidget(self.btn_start, row, 0)
        mg.addWidget(self.btn_pause, row, 1)
        row += 1
        mg.addWidget(self.btn_resume, row, 0)
        mg.addWidget(self.btn_skip, row, 1)
        row += 1
        mg.addWidget(self.btn_cancel, row, 0)
    
        mg.addWidget(self.list_wps, row, 0, 1, 2)
        row += 1
        mg.addWidget(self.label_status, row, 0, 1, 2)
        row += 1
        mg.addWidget(self.label_feedback, row, 0, 1, 2)

        right_layout.addWidget(self.mission_group)

        # Map view
        self.map_view = MapView()
        right_layout.addWidget(self.map_view, 1)

        # Signals
        self.btn_load.clicked.connect(self.on_load)
        self.btn_start.clicked.connect(self.on_start)
        self.btn_pause.clicked.connect(self.on_pause)
        self.btn_resume.clicked.connect(self.on_resume)
        self.btn_skip.clicked.connect(self.on_skip)
        self.btn_cancel.clicked.connect(self.on_cancel)
        self.chk_loop.toggled.connect(self.on_loop)

        # ROS → UI
        self.ros.feedbackText.connect(self.on_feedback)
        self.ros.goalStatus.connect(self.on_goal_status)
        self.ros.robotPoseUpdated.connect(self.map_view.set_robot_pose)
        self.ros.mapUpdated.connect(self._on_map_updated)
        self.ros.photoRequested.connect(self._on_photo_requested)
        self.ros.plantScanned.connect(self.on_scan_update)

        # --- Wire camera frames to labels (with NDVI false-colour) ---
        self.ros.rgbFrame.connect(lambda q: self._set_cam_image(self.lbl_rgb, q, colormap=False))
        self.ros.ndviFrame.connect(lambda q: self._set_cam_image(self.lbl_ndvi, q, colormap=True))

        # --- Auto-start camera subscriptions after the UI is up ---
        QtCore.QTimer.singleShot(500, lambda: self.ros.start_cameras(
            rgb_topic="/front_rgb/image_raw",
            ndvi_topic="/front_ndvi/image_raw"
        ))

    # -------- Camera panel (RGB + NDVI) --------
    def _build_camera_panel(self) -> QtWidgets.QWidget:
        def cam_tile(title: str, label_attr: str) -> QtWidgets.QFrame:
            frame = QtWidgets.QFrame()
            frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
            frame.setStyleSheet("QFrame { background: #F7F5EF; border:1px solid #3A5A40; border-radius: 10px; }")
            lay = QtWidgets.QVBoxLayout(frame)
            hdr = QtWidgets.QLabel(title)
            hdr.setStyleSheet("color:#184E77; font-weight:600;")
            lbl = QtWidgets.QLabel("No feed yet")
            lbl.setAlignment(QtCore.Qt.AlignCenter)
            lbl.setStyleSheet("background:#222; color:#bbb; border-radius:8px; min-width:340px; min-height:220px;")
            setattr(self, label_attr, lbl)  # keep a handle for updates
            lay.addWidget(hdr)
            lay.addWidget(lbl, 1)
            return frame

        w = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(w)
        title = QtWidgets.QLabel("Cameras")
        title.setStyleSheet("font-weight: 700; font-size: 18px; color:#184E77")
        v.addWidget(title)

        # Stack vertically: RGB (top), NDVI (bottom)
        v.addWidget(cam_tile("RGB",  "lbl_rgb"))
        v.addWidget(cam_tile("NDVI", "lbl_ndvi"))

        btns = QtWidgets.QHBoxLayout()
        self.btn_cam_start = QtWidgets.QPushButton("Start Cameras")
        self.btn_cam_stop  = QtWidgets.QPushButton("Stop Cameras")
        btns.addWidget(self.btn_cam_start)
        btns.addWidget(self.btn_cam_stop)
        v.addLayout(btns)

        # Hook buttons
        self.btn_cam_start.clicked.connect(lambda: self.ros.start_cameras(
            rgb_topic="/front_rgb/image_raw",
            ndvi_topic="/front_ndvi/image_raw"
        ))
        self.btn_cam_stop.clicked.connect(self.ros.stop_cameras)

        v.addStretch(1)
        return w

    def _set_cam_image(self, lbl: QtWidgets.QLabel, qimg: QtGui.QImage, colormap: bool = False):
        """
        Update a QLabel with a QImage. If colormap=True and the image is mono,
        apply TURBO false colour for an NDVI-like heatmap look.
        """
        img = qimg
        if colormap and qimg.format() == QtGui.QImage.Format_Grayscale8:
            w, h = qimg.width(), qimg.height()
            ptr = qimg.constBits()
            arr = np.array(ptr).reshape(h, w)
            colored = cv2.applyColorMap(arr, cv2.COLORMAP_TURBO)
            colored = cv2.cvtColor(colored, cv2.COLOR_BGR2RGB)
            img = QtGui.QImage(colored.data, w, h, colored.strides[0], QtGui.QImage.Format_RGB888).copy()

        pm = QtGui.QPixmap.fromImage(img)
        pm = pm.scaled(lbl.size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation)
        lbl.setPixmap(pm)

    # -------- Mission logic --------
    def on_load(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Open Waypoints CSV", "", "CSV Files (*.csv);;All Files (*)")
        if not path:
            return
        try:
            wps = self._read_csv(path)
            self._waypoints = wps
            self._current_index = -1
            self.list_wps.clear()
            for wp in wps:
                self.list_wps.addItem(f"{wp.name}  (x={wp.x:.2f}, y={wp.y:.2f}, yaw={wp.yaw_deg:.0f}°, {wp.frame_id})")
            self.map_view.set_waypoints(wps)

            # Default icons from your icon folder
            icon_dir = "/home/anton/ros2_ws/src/Wallbug/src/icon"
            self.map_view.load_robot_icon(os.path.join(icon_dir, "ladybug.png"))
            self.map_view.set_leaf_icons(
                clear_path=os.path.join(icon_dir, "images.png"),
                green_path=os.path.join(icon_dir, "green-leaf.png"),
                yellow_path=os.path.join(icon_dir, "yellow-leaf.png"),
                red_path=os.path.join(icon_dir, "red-leaf.png"),
            )

            self.label_status.setText(f"Status: loaded {len(wps)} waypoints")
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Load Error", str(e))

    def _read_csv(self, path: str) -> List[Waypoint]:
        wps: List[Waypoint] = []
        with open(path, 'r', newline='') as f:
            reader = csv.DictReader(f)
            required = {'name', 'x', 'y', 'yaw_deg'}
            if not required.issubset(reader.fieldnames or {}):
                raise ValueError(f"CSV must contain headers: {sorted(required)}")
            for row in reader:
                name = row.get('name', '').strip() or f"wp{len(wps)+1}"
                x = float(row.get('x', 0))
                y = float(row.get('y', 0))
                yaw_deg = float(row.get('yaw_deg', 0))
                frame_id = (row.get('frame_id') or 'map').strip()
                hold_sec = float(row.get('hold_sec', 0) or 0)
                photo = str(row.get('photo', 'false')).lower() in ('1', 'true', 'yes')
                wps.append(Waypoint(name, x, y, yaw_deg, frame_id, hold_sec, photo))
        if not wps:
            raise ValueError("No waypoints found in CSV")
        return wps

    def on_start(self):
        if not self._waypoints:
            QtWidgets.QMessageBox.warning(self, "No waypoints", "Load a waypoint file first.")
            return
        if self._paused and 0 <= self._current_index < len(self._waypoints):
            self._paused = False
            self.label_status.setText("Status: resumed")
            return
        self._current_index = 0
        self._dispatch_current_goal()

    def _dispatch_current_goal(self):
        if not (0 <= self._current_index < len(self._waypoints)):
            if self._loop and self._waypoints:
                self._current_index = 0
            else:
                self.label_status.setText("Status: mission complete")
                self.map_view.set_current_goal(None, None)
                return
        wp = self._waypoints[self._current_index]
        self.list_wps.setCurrentRow(self._current_index)
        self.label_status.setText(f"Status: navigating to {wp.name}")
        self.map_view.set_current_goal(wp.x, wp.y)
        self.ros.send_goal(wp)

    def on_pause(self):
        self._paused = True
        self.label_status.setText("Status: paused (goal continues until cancelled)")

    def on_resume(self):
        if self._paused:
            self._paused = False
            self.label_status.setText("Status: resumed")

    def on_skip(self):
        self._current_index += 1
        self._dispatch_current_goal()

    def on_cancel(self):
        self.ros.cancel_active_goal()
        self.map_view.set_current_goal(None, None)
        self.label_status.setText("Status: goal cancelled")

    def on_loop(self, checked: bool):
        self._loop = checked

    # -------- ROS→UI callbacks --------
    def on_goal_status(self, text: str):
        self.label_status.setText(f"Status: {text}")
        if text.startswith("Reached:"):
            hold = 0
            if 0 <= self._current_index < len(self._waypoints):
                hold = self._waypoints[self._current_index].hold_sec
            QtCore.QTimer.singleShot(int(hold * 1000), self._advance_after_reach)

    def _advance_after_reach(self):
        if not self._paused:
            self._current_index += 1
            self._dispatch_current_goal()

    def on_feedback(self, text: str):
        self.label_feedback.setText(f"Feedback: {text}")

    def _on_map_updated(self, qimage: QtGui.QImage, origin_x: float, origin_y: float):
        self.map_view.update_map(qimage, origin_x, origin_y)

    def _on_photo_requested(self, name: str):
        print(f"[Mission] Photo requested at waypoint: {name}")

    # Scan update from ROS topic
    def on_scan_update(self, name: str, green: float, yellow: float, brown: float):
        self.map_view.set_scan_result(name, green, yellow, brown)

    # --- Icon pickers (optional overrides) ---
    def on_pick_robot_icon(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Select Robot Icon (PNG)", "", "Images (*.png *.jpg *.svg)")
        if path:
            self.map_view.load_robot_icon(path)

    def on_pick_plant_icon(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Select Plant Icon (PNG)", "", "Images (*.png *.jpg *.svg)")
        if path:
            self.map_view.set_leaf_icons(path, path, path, path)


# -----------------------------
# rclpy <-> Qt integration
# -----------------------------
class RosQtApp(QtWidgets.QApplication):
    def __init__(self, argv):
        super().__init__(argv)
        rclpy.init(args=None)
        self._ros = MissionRos()

        # Spin rclpy regularly without a dedicated thread
        self._spin_timer = QtCore.QTimer()
        self._spin_timer.timeout.connect(self._spin_once)
        self._spin_timer.start(20)  # ~50 Hz

    def _spin_once(self):
        rclpy.spin_once(self._ros.node, timeout_sec=0.001)

    def ros_node(self) -> MissionRos:
        return self._ros

    def quit(self) -> None:
        try:
            self._spin_timer.stop()
            self._ros.stop_cameras()
            self._ros.node.destroy_node()
            rclpy.shutdown()
        finally:
            super().quit()


# -----------------------------
# Main entry
# -----------------------------
if __name__ == '__main__':
    app = RosQtApp(sys.argv)
    win = MissionWindow(app.ros_node())
    win.show()
    sys.exit(app.exec())
