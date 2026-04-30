#!/usr/bin/env python3
"""Interactive flight controller GUI.

Publishes /desired_attitude and /vehicle_params whenever any slider changes.
Tkinter runs on the main thread; ROS spins in a background thread.
"""

import sys
import math
import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3

from craft_sim.msg import VehicleParams


# ── Slider specification ──────────────────────────────────────────────────────

_SLIDERS = {
    "Desired Attitude": [
        ("Roll  (deg)",  "roll",    -180.0, 180.0,  0.0),
        ("Pitch (deg)",  "pitch",    -90.0,  90.0,  0.0),
        ("Yaw   (deg)",  "yaw",    -180.0, 180.0,  0.0),
    ],
    "Attitude PID": [
        ("Kp", "att_kp",   0.0, 10.0,  2.0),
        ("Ki", "att_ki",   0.0,  2.0,  0.05),
        ("Kd", "att_kd",   0.0,  5.0,  0.5),
        ("FF", "att_ff",   0.0,  2.0,  0.0),
    ],
    "Rate PD": [
        ("Kp", "rate_kp",  0.0, 30.0, 10.0),
        ("Kd", "rate_kd",  0.0,  5.0,  0.5),
        ("FF", "rate_ff",  0.0,  2.0,  0.0),
    ],
    "Rate Limits": [
        ("Roll (deg/s)", "rate_limit_roll",  0.0, 50.0, 10.0),
        ("Pitch (deg/s)", "rate_limit_pitch",  0.0, 50.0, 10.0),
        ("Yaw (deg/s)", "rate_limit_yaw",  0.0, 50.0,  10.0),
    ],
    "Inertia Tensor (kg·m^2)": [
        ("Ixx", "ixx",  0.1, 10.0, 1.0),
        ("Iyy", "iyy",  0.1, 10.0, 2.0),
        ("Izz", "izz",  0.1, 10.0, 3.0),
    ],
}


# ── ROS node ──────────────────────────────────────────────────────────────────

class FlightControllerNode(Node):
    def __init__(self):
        super().__init__("attitude_controller")
        self._pub_att    = self.create_publisher(Vector3,       "/desired_attitude", 10)
        self._pub_params = self.create_publisher(VehicleParams, "/vehicle_params",   10)

    def publish(self, vals: dict):
        att = Vector3()
        att.x = math.radians(vals["roll"])
        att.y = math.radians(vals["pitch"])
        att.z = math.radians(vals["yaw"])
        self._pub_att.publish(att)

        p = VehicleParams()
        p.ixx     = vals["ixx"]
        p.iyy     = vals["iyy"]
        p.izz     = vals["izz"]
        p.att_kp  = vals["att_kp"]
        p.att_ki  = vals["att_ki"]
        p.att_kd  = vals["att_kd"]
        p.att_ff  = vals["att_ff"]
        p.rate_kp = vals["rate_kp"]
        p.rate_kd = vals["rate_kd"]
        p.rate_ff = vals["rate_ff"]
        p.rate_limit_roll = vals["rate_limit_roll"]
        p.rate_limit_pitch = vals["rate_limit_pitch"]
        p.rate_limit_yaw = vals["rate_limit_yaw"]
        self._pub_params.publish(p)


# ── Tkinter GUI ───────────────────────────────────────────────────────────────

class FlightControllerGUI:
    def __init__(self, node: FlightControllerNode):
        self._node = node
        self._vars: dict[str, tk.DoubleVar] = {}

        self._root = tk.Tk()
        self._root.title("Flight Controller")
        self._root.configure(bg="#1e1e2e")
        self._root.resizable(False, False)

        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TFrame",  background="#1e1e2e")
        style.configure("TLabel",  background="#1e1e2e", foreground="#cdd6f4",
                        font=("Helvetica", 10))
        style.configure("Section.TLabel", background="#1e1e2e", foreground="#89b4fa",
                        font=("Helvetica", 11, "bold"))
        style.configure("Val.TLabel", background="#1e1e2e", foreground="#a6e3a1",
                        font=("Helvetica", 10), width=8, anchor="e")
        style.configure("TScale",  background="#1e1e2e", troughcolor="#313244",
                        sliderthickness=14)

        outer = ttk.Frame(self._root, padding=12)
        outer.pack(fill="both", expand=True)

        for section, sliders in _SLIDERS.items():
            self._build_section(outer, section, sliders)

        self._root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_section(self, parent, title: str, sliders: list):
        frame = ttk.Frame(parent)
        frame.pack(fill="x", pady=(6, 0))

        ttk.Label(frame, text=title, style="Section.TLabel").grid(
            row=0, column=0, columnspan=3, sticky="w", pady=(0, 4))

        for row, (label, key, lo, hi, default) in enumerate(sliders, start=1):
            var = tk.DoubleVar(value=default)
            self._vars[key] = var

            ttk.Label(frame, text=label, width=16).grid(
                row=row, column=0, sticky="w")

            val_lbl = ttk.Label(frame, text=f"{default:.3f}", style="Val.TLabel")
            val_lbl.grid(row=row, column=2, padx=(4, 0))

            scale = ttk.Scale(
                frame, from_=lo, to=hi, orient="horizontal",
                variable=var, length=320,
                command=lambda _, v=var, lbl=val_lbl: self._on_change(v, lbl),
            )
            scale.grid(row=row, column=1, padx=4, pady=2)

    def _on_change(self, var: tk.DoubleVar, label: ttk.Label):
        v = var.get()
        label.configure(text=f"{v:.3f}")
        self._publish()

    def _publish(self):
        vals = {k: v.get() for k, v in self._vars.items()}
        self._node.publish(vals)

    def _on_close(self):
        self._root.destroy()

    def run(self):
        # Publish once with defaults so nodes receive initial params immediately
        self._publish()
        self._root.mainloop()


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = FlightControllerNode()

    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    gui = FlightControllerGUI(node)
    gui.run()

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0)


if __name__ == "__main__":
    main()
