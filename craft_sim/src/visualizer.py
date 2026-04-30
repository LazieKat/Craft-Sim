#!/usr/bin/env python3
"""3D spacecraft visualizer with attitude/rate plots.

Subscribes to /current_attitude and /current_rates, renders the vehicle
orientation in a GLViewWidget and streams 6 time-series plots.
"""

import sys
import threading
import math
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3

from PyQt6.QtWidgets import QApplication, QMainWindow, QWidget, QHBoxLayout
from PyQt6.QtCore import pyqtSignal, QObject

import pyqtgraph as pg
import pyqtgraph.opengl as gl


# ── Spacecraft geometry ───────────────────────────────────────────────────────

def _box_mesh(dx: float, dy: float, dz: float) -> tuple[np.ndarray, np.ndarray]:
    """Return (verts, faces) for a box centred at origin."""
    hx, hy, hz = dx / 2, dy / 2, dz / 2
    verts = np.array([
        [-hx, -hy, -hz], [ hx, -hy, -hz], [ hx,  hy, -hz], [-hx,  hy, -hz],
        [-hx, -hy,  hz], [ hx, -hy,  hz], [ hx,  hy,  hz], [-hx,  hy,  hz],
    ], dtype=np.float32)
    faces = np.array([
        [0,1,2],[0,2,3],  # -Z
        [4,5,6],[4,6,7],  # +Z
        [0,1,5],[0,5,4],  # -Y
        [2,3,7],[2,7,6],  # +Y
        [0,3,7],[0,7,4],  # -X
        [1,2,6],[1,6,5],  # +X
    ], dtype=np.uint32)
    return verts, faces


def _build_spacecraft() -> list[tuple[np.ndarray, np.ndarray, tuple]]:
    """Return list of (verts, faces, color) for the spacecraft geometry."""
    parts = []

    # Main body — silver-grey
    v, f = _box_mesh(0.6, 0.6, 1.2)
    parts.append((v, f, (0.7, 0.75, 0.8, 1.0)))

    # Solar panel +Y
    v, f = _box_mesh(1.8, 0.04, 0.7)
    v[:, 1] += 0.52  # shift to +Y side
    parts.append((v, f, (0.1, 0.4, 0.8, 1.0)))

    # Solar panel −Y
    v, f = _box_mesh(1.8, 0.04, 0.7)
    v[:, 1] -= 0.52
    parts.append((v, f, (0.1, 0.4, 0.8, 1.0)))

    # Thruster nozzle at −Z
    v, f = _box_mesh(0.25, 0.25, 0.25)
    v[:, 2] -= 0.725
    parts.append((v, f, (0.9, 0.6, 0.1, 1.0)))

    return parts


# ── Qt signal carrier ─────────────────────────────────────────────────────────

class _Signals(QObject):
    new_data = pyqtSignal(float, float, float, float, float, float)  # att+rates


# ── ROS node ──────────────────────────────────────────────────────────────────

class VisualizerNode(Node):
    def __init__(self, signals: _Signals):
        super().__init__("visualizer")
        self._sig = signals
        self.create_subscription(Vector3, "/current_attitude", self._on_att, 10)
        self.create_subscription(Vector3, "/current_rates",    self._on_rates, 10)
        self._att  = [0.0, 0.0, 0.0]
        self._rates = [0.0, 0.0, 0.0]

    def _on_att(self, msg: Vector3):
        self._att = [msg.x, msg.y, msg.z]
        self._emit()

    def _on_rates(self, msg: Vector3):
        self._rates = [msg.x, msg.y, msg.z]

    def _emit(self):
        self._sig.new_data.emit(*self._att, *self._rates)


# ── Main window ───────────────────────────────────────────────────────────────

class MainWindow(QMainWindow):
    HISTORY_S = 10.0   # seconds of plot history
    DT        = 0.01   # 100 Hz
    MAX_PTS   = int(HISTORY_S / DT)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Craft-Sim Visualizer")
        self.resize(1600, 900)

        # ── Layout ────────────────────────────────────────────────────────────
        root = QWidget()
        self.setCentralWidget(root)
        layout = QHBoxLayout(root)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(4)

        # ── 3D view ───────────────────────────────────────────────────────────
        self._gl = gl.GLViewWidget()
        self._gl.setMinimumWidth(600)
        self._gl.setCameraPosition(distance=6, elevation=25, azimuth=45)
        self._gl.setBackgroundColor((30, 30, 40, 255))
        layout.addWidget(self._gl, stretch=2)

        grid = gl.GLGridItem()
        grid.setSize(10, 10)
        grid.setSpacing(1, 1)
        self._gl.addItem(grid)

        # Axis indicator (body frame)
        self._ax = gl.GLAxisItem()
        self._ax.setSize(2, 2, 2)
        self._gl.addItem(self._ax)

        # Spacecraft mesh items
        self._mesh_items: list[gl.GLMeshItem] = []
        self._mesh_offsets: list[np.ndarray] = []
        spacecraft_parts = _build_spacecraft()
        for verts, faces, color in spacecraft_parts:
            md = gl.MeshData(vertexes=verts, faces=faces)
            item = gl.GLMeshItem(
                meshdata=md,
                smooth=False,
                color=color,
                shader="shaded",
                drawEdges=False,
            )
            self._gl.addItem(item)
            self._mesh_items.append(item)
            self._mesh_offsets.append(verts.copy())

        # ── Plots ─────────────────────────────────────────────────────────────
        pg.setConfigOption("background", (30, 30, 40))
        pg.setConfigOption("foreground", "w")

        plot_widget = pg.GraphicsLayoutWidget()
        layout.addWidget(plot_widget, stretch=3)

        labels   = ["Roll (deg)", "Pitch (deg)", "Yaw (deg)",
                    "Roll Rate (deg/s)", "Pitch Rate (deg/s)", "Yaw Rate (deg/s)"]
        colors   = [(255, 80,  80), (80, 200,  80), (80, 120, 255),
                    (255, 160, 80), (80, 220, 200), (200, 80, 255)]

        Ylimits  = [(-180, 180), (-180, 180), (-180, 180),
                    (-10, 80), (-10, 80), (-10, 80)]

        self._curves: list[pg.PlotDataItem] = []
        self._bufs: list[deque] = [deque(maxlen=self.MAX_PTS) for _ in range(6)]
        self._tbuf: deque = deque(maxlen=self.MAX_PTS)
        self._t = 0.0

        for idx, (label, color) in enumerate(zip(labels, colors)):
            row, col = divmod(idx, 3)
            plot = plot_widget.addPlot(row=row, col=col, title=label)
            plot.setLabel("bottom", "t (s)")
            plot.showGrid(x=True, y=True, alpha=0.3)
            plot.setYRange(*Ylimits[idx])
            curve = plot.plot(pen=pg.mkPen(color=color, width=2))
            self._curves.append(curve)

    # ── Slot ─────────────────────────────────────────────────────────────────

    def on_new_data(self, roll: float, pitch: float, yaw: float,
                    p: float, q: float, r: float):
        self._t += self.DT
        self._tbuf.append(self._t)

        deg = math.degrees
        values = [deg(roll), deg(pitch), deg(yaw),
                  deg(p),    deg(q),     deg(r)]

        for i, v in enumerate(values):
            self._bufs[i].append(v)

        t_arr = np.array(self._tbuf)
        for i, curve in enumerate(self._curves):
            curve.setData(t_arr, np.array(self._bufs[i]))

        # Rotation matrix from Euler angles (ZYX: Rz*Ry*Rx)
        cr, sr = math.cos(roll),  math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw),   math.sin(yaw)

        R = np.array([
            [cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr],
            [sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr],
            [-sp,    cp*sr,              cp*cr           ],
        ], dtype=np.float32)

        # Build 4×4 transform
        T = np.eye(4, dtype=np.float32)
        T[:3, :3] = R

        for item in self._mesh_items:
            item.setTransform(T)

        self._ax.setTransform(T)


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)

    app = QApplication(sys.argv)

    signals = _Signals()
    ros_node = VisualizerNode(signals)

    window = MainWindow()
    signals.new_data.connect(window.on_new_data)
    window.show()

    ros_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    ros_thread.start()

    ret = app.exec()
    ros_node.destroy_node()
    rclpy.shutdown()
    sys.exit(ret)


if __name__ == "__main__":
    main()
