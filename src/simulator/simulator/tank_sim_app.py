import tkinter as tk
from collections import deque

import numpy as np
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

import rclpy
from simulator.tank_node import TankNode
from water_tank_interface.msg import TankOutput


# ── Simulation constants ─────────────────────────────────────────────────────
TANK_AREA = 0.50
MAX_LEVEL = 5.0
MAX_FLOW_IN = 0.35
VALVE_TAU = .1
SENSOR_NOISE = 0.02
DT = 0.1
HISTORY_LEN = 300
UPDATE_MS = 100
ROS_POLL_MS = 10

DEFAULT_VALVE_PERCENT = 0.0   # used until the first /valve_cmd message arrives
GOAL_HIGH_DEFAULT = 4.5       # display-only reference lines (no effect on control)
GOAL_LOW_DEFAULT = 0.5


# ── Physics models ───────────────────────────────────────────────────────────
class LevelSensor:
    def __init__(self, noise_std=SENSOR_NOISE):
        self.noise_std = noise_std

    def read(self, level: float) -> float:
        return float(np.clip(
            level + np.random.normal(0, self.noise_std), 0.0, MAX_LEVEL
        ))


class ControlValve:
    """First-order inlet valve model."""

    def __init__(self, max_flow=MAX_FLOW_IN, tau=VALVE_TAU):
        self.max_flow = max_flow
        self.tau = tau
        self.demand = 0.0
        self.position = 0.0

    def set_demand(self, pct: float):
        self.demand = float(np.clip(pct, 0.0, 100.0))

    def update(self, dt: float):
        alpha = dt / (self.tau + dt)
        self.position += alpha * (self.demand - self.position)

    @property
    def flow(self) -> float:
        return (self.position / 100.0) * self.max_flow


class Tank:
    """Tank with a Torricelli-style outlet flow. Level is clamped to 0..MAX_LEVEL."""

    def __init__(self, h0=1.5, drain_coeff=0.04):
        self.level = float(h0)
        self.drain_coeff = float(drain_coeff)
        self.last_qout = 0.0

    def update(self, qin: float, dt: float, disturbance: float = 0.0):
        self.last_qout = max(
            0.0, self.drain_coeff * np.sqrt(max(0.0, self.level)) + disturbance
        )
        self.level = float(np.clip(
            self.level + (qin - self.last_qout) * dt / TANK_AREA,
            0.0,
            MAX_LEVEL,
        ))


# ── GUI application ──────────────────────────────────────────────────────────
class TankSimulationApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Tank Level Simulation — ROS controlled")
        self.configure(bg="#5e1e2e")
        self.geometry("1120x740")
        self.minsize(900, 640)

        rclpy.init()
        self.node = TankNode(default_cmd=DEFAULT_VALVE_PERCENT)

        self.tank = Tank(h0=1.5, drain_coeff=0.04)
        self.valve = ControlValve()
        self.sensor = LevelSensor()

        self.var_drain = tk.DoubleVar(value=0.04)
        self.var_dist = tk.DoubleVar(value=0.0)
        self.var_ll = tk.DoubleVar(value=GOAL_LOW_DEFAULT)    # goal line only
        self.var_hh = tk.DoubleVar(value=GOAL_HIGH_DEFAULT)   # goal line only

        self.running = False
        self.sim_time = 0.0

        self.t_buf = deque([0.0] * HISTORY_LEN, maxlen=HISTORY_LEN)
        self.level_buf = deque([1.5] * HISTORY_LEN, maxlen=HISTORY_LEN)
        self.valve_buf = deque([0.0] * HISTORY_LEN, maxlen=HISTORY_LEN)

        self._build_ui()
        self._draw_tank()
        self._update_readouts(1.5)
        self._update_status()

        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.after(ROS_POLL_MS, self._ros_poll)

    # ── ROS ──────────────────────────────────────────────────────────────────
    def _ros_poll(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)
        self.after(ROS_POLL_MS, self._ros_poll)

    def _on_close(self):
        self.running = False
        self.destroy()
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    # ── UI ───────────────────────────────────────────────────────────────────
    def _build_ui(self):
        left = tk.Frame(self, bg="#1e1e2e")
        right = tk.Frame(self, bg="#1e1e2e")
        left.pack(side=tk.LEFT, fill=tk.Y, padx=8, pady=8)
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 8), pady=8)

        tk.Label(
            left, text="TANK PROCESS", bg="#1e1e2e", fg="#cdd6f4",
            font=("Courier", 12, "bold")
        ).pack(pady=(0, 5))

        self.tank_canvas = tk.Canvas(
            left, width=320, height=390, bg="#11111b",
            highlightthickness=1, highlightbackground="#45475a"
        )
        self.tank_canvas.pack()

        readouts = tk.LabelFrame(
            left, text=" Live process values ", bg="#1e1e2e",
            fg="#cdd6f4", font=("Courier", 9, "bold")
        )
        readouts.pack(fill=tk.X, pady=(8, 0))
        self.ro_level = self._readout(readouts, "Measured level", "1.500 m")
        self.ro_actual = self._readout(readouts, "Actual level", "1.500 m")
        self.ro_valve = self._readout(readouts, "Valve position", "0.0 %")
        self.ro_inflow = self._readout(readouts, "Inlet flow", "0.0000 m³/s")
        self.ro_outflow = self._readout(readouts, "Outlet flow", "0.0000 m³/s")
        self.ro_time = self._readout(readouts, "Simulation time", "0.0 s")
        self.ro_command = self._readout(
            readouts, "ROS valve command", f"{self.node.get_cmd():.1f} %"
        )

        tk.Label(
            right, text="Live trends", bg="#1e1e2e", fg="#cdd6f4",
            font=("Courier", 11, "bold")
        ).pack(anchor="w")

        self.fig = Figure(figsize=(7.2, 4.4), dpi=95, facecolor="#111825")
        self.ax_level = self.fig.add_subplot(211)
        self.ax_valve = self.fig.add_subplot(212, sharex=self.ax_level)
        self.fig.subplots_adjust(hspace=0.45, left=0.10, right=0.97,
                                 top=0.93, bottom=0.11)

        for ax in (self.ax_level, self.ax_valve):
            ax.set_facecolor("#1e1e2e")
            ax.tick_params(colors="#a6adc8", labelsize=8)
            for spine in ax.spines.values():
                spine.set_color("#45475a")

        self.ax_level.set_title("Tank level", color="#cdd6f4", fontsize=9)
        self.ax_level.set_ylabel("Level (m)", color="#cdd6f4", fontsize=8)
        self.ax_level.set_ylim(0, MAX_LEVEL + 0.2)
        self.ax_valve.set_title("Inlet valve position", color="#cdd6f4", fontsize=9)
        self.ax_valve.set_ylabel("Opening (%)", color="#cdd6f4", fontsize=8)
        self.ax_valve.set_xlabel("Simulation time (s)", color="#cdd6f4", fontsize=8)
        self.ax_valve.set_ylim(-5, 105)

        t = list(self.t_buf)
        self.line_level, = self.ax_level.plot(
            t, list(self.level_buf), color="#89b4fa", lw=1.6, label="Measured level"
        )
        # Goal lines: visual reference only, nothing is triggered by them.
        self.ll_line = self.ax_level.axhline(
            self.var_ll.get(), color="#f9e2af", lw=0.9, ls=":", label="Low goal"
        )
        self.hh_line = self.ax_level.axhline(
            self.var_hh.get(), color="#f38ba8", lw=0.9, ls=":", label="High goal"
        )
        self.line_valve, = self.ax_valve.plot(
            t, list(self.valve_buf), color="#a6e3a1", lw=1.6
        )
        self.ax_level.legend(
            loc="upper right", fontsize=7, facecolor="#313244",
            labelcolor="#cdd6f4", framealpha=0.8
        )
        self.chart = FigureCanvasTkAgg(self.fig, master=right)
        self.chart.get_tk_widget().pack(fill=tk.BOTH, expand=True, pady=(4, 8))

        controls = tk.LabelFrame(
            right, text=" Simulation settings ",
            bg="#1e1e2e", fg="#cdd6f4", font=("Courier", 9, "bold")
        )
        controls.pack(fill=tk.X)

        button_row = tk.Frame(controls, bg="#1e1e2e")
        button_row.pack(fill=tk.X, padx=8, pady=6)
        self.btn_run = tk.Button(
            button_row, text="▶ Start", command=self.toggle_run,
            width=12, bg="#a6e3a1", fg="#1e1e2e",
            font=("Courier", 9, "bold")
        )
        self.btn_run.pack(side=tk.LEFT, padx=(0, 8))
        tk.Button(
            button_row, text="Reset simulation", command=self.reset_simulation,
            width=16, bg="#89b4fa", fg="#1e1e2e",
            font=("Courier", 9, "bold")
        ).pack(side=tk.LEFT)

        sliders = tk.Frame(controls, bg="#1e1e2e")
        sliders.pack(fill=tk.X, padx=8, pady=(0, 8))
        self._add_slider(sliders, 0, "Drain coefficient", self.var_drain, 0.0, 0.10, 0.001)
        self._add_slider(sliders, 1, "Outlet disturbance", self.var_dist, -0.02, 0.05, 0.001)
        self._add_slider(sliders, 2, "Low goal line (m)", self.var_ll, 0.0, 2.0, 0.05)
        self._add_slider(sliders, 3, "High goal line (m)", self.var_hh, 3.0, MAX_LEVEL, 0.05)

        self.var_ll.trace_add("write", self._update_goal_lines)
        self.var_hh.trace_add("write", self._update_goal_lines)

        tk.Label(
            controls,
            text="Valve command comes from topic /valve_cmd (water_tank_interface/TankInput)",
            bg="#1e1e2e", fg="#f9e2af", font=("Courier", 9, "bold")
        ).pack(anchor="w", padx=8, pady=(0, 8))

    def _readout(self, parent, label, initial):
        row = tk.Frame(parent, bg="#1e1e2e")
        row.pack(fill=tk.X, padx=5, pady=2)
        tk.Label(
            row, text=label + ":", width=17, anchor="w",
            bg="#1e1e2e", fg="#a6adc8", font=("Courier", 8)
        ).pack(side=tk.LEFT)
        value = tk.StringVar(value=initial)
        tk.Label(
            row, textvariable=value, width=14, anchor="w",
            bg="#1e1e2e", fg="#89dceb", font=("Courier", 8, "bold")
        ).pack(side=tk.LEFT)
        return value

    def _add_slider(self, parent, row, label, variable, low, high, resolution):
        frame = tk.Frame(parent, bg="#1e1e2e")
        frame.grid(row=row, column=0, sticky="ew", pady=2)
        parent.grid_columnconfigure(0, weight=1)
        tk.Label(
            frame, text=label, width=22, anchor="w",
            bg="#1e1e2e", fg="#cdd6f4", font=("Courier", 8)
        ).pack(side=tk.LEFT)
        tk.Scale(
            frame, variable=variable, from_=low, to=high,
            resolution=resolution, orient=tk.HORIZONTAL, length=270,
            showvalue=True, bg="#1e1e2e", fg="#cdd6f4",
            troughcolor="#45475a", highlightthickness=0
        ).pack(side=tk.LEFT, fill=tk.X, expand=True)

    def _update_goal_lines(self, *_):
        """Moves the goal lines on the trend chart. Display only."""
        self.ll_line.set_ydata([self.var_ll.get(), self.var_ll.get()])
        self.hh_line.set_ydata([self.var_hh.get(), self.var_hh.get()])
        self.chart.draw_idle()
        self._draw_tank()

    # ── Simulation ───────────────────────────────────────────────────────────
    def _step(self):
        measured_level = self.sensor.read(self.tank.level)
        
        # fill the output here
        output = TankOutput()
        output.level = float(measured_level)
        output.desired_level = float(self.var_hh.get())
        output.simulator_running = self.running
        self.node.publish_data(output)  
        
        if not self.running:
            return
        
        self.valve.set_demand(self.node.get_cmd())   # clips to 0-100
        self.valve.update(DT)
        self.tank.drain_coeff = self.var_drain.get()
        self.tank.update(self.valve.flow, DT, disturbance=self.var_dist.get())
        self.sim_time += DT

        self.t_buf.append(self.sim_time)
        self.level_buf.append(measured_level)
        self.valve_buf.append(self.valve.position)

        self._update_readouts(measured_level)
        self._update_status()
        self._draw_tank()
        self._update_chart()
        self.after(UPDATE_MS, self._step)

    def _update_readouts(self, measured_level):
        self.ro_level.set(f"{measured_level:.3f} m")
        self.ro_actual.set(f"{self.tank.level:.3f} m")
        self.ro_valve.set(f"{self.valve.position:.1f} %")
        self.ro_inflow.set(f"{self.valve.flow:.4f} m³/s")
        self.ro_outflow.set(f"{self.tank.last_qout:.4f} m³/s")
        self.ro_time.set(f"{self.sim_time:.1f} s")
        self.ro_command.set(f"{self.node.get_cmd():.1f} %")

    def _update_status(self):
        """Informational only: does not affect the simulation."""
        if self.tank.level >= MAX_LEVEL - 0.01:
            state = "TANK FULL"
        elif self.tank.level <= 0.01:
            state = "TANK EMPTY"
        else:
            state = "Normal operation"
        self.title(f"Tank Level Simulation — {state}")

    def _update_chart(self):
        t = list(self.t_buf)
        self.line_level.set_data(t, list(self.level_buf))
        self.line_valve.set_data(t, list(self.valve_buf))
        xmax = max(t[-1], 1.0)
        self.ax_level.set_xlim(t[0], xmax)
        self.ax_valve.set_xlim(t[0], xmax)
        self.chart.draw_idle()

    def _draw_tank(self):
        c = self.tank_canvas
        c.delete("all")
        x1, y1, x2, y2 = 85, 55, 235, 300
        c.create_rectangle(x1, y1, x2, y2, outline="#cdd6f4", width=3)
        water_height = int((y2 - y1) * self.tank.level / MAX_LEVEL)
        water_top = y2 - water_height
        c.create_rectangle(x1 + 3, water_top, x2 - 3, y2 - 3,
                           fill="#1e88e5", outline="")
        c.create_text(160, 30, text="TK-101", fill="#cdd6f4",
                      font=("Courier", 12, "bold"))
        c.create_text(160, max(y1 + 14, water_top + 15),
                      text=f"{self.tank.level:.2f} m", fill="white",
                      font=("Courier", 11, "bold"))
        c.create_text(160, 325, text=f"Valve command: {self.node.get_cmd():.1f}%",
                      fill="#a6e3a1", font=("Courier", 9, "bold"))

        # Inlet line and valve.
        c.create_line(20, 95, x1, 95, fill="#89dceb", width=5)
        c.create_polygon(48, 86, 60, 95, 48, 104, fill="#a6e3a1", outline="#cdd6f4")
        c.create_polygon(72, 86, 60, 95, 72, 104, fill="#a6e3a1", outline="#cdd6f4")
        c.create_text(45, 75, text="IN", fill="#89dceb", font=("Courier", 8))
        c.create_text(60, 117, text=f"{self.valve.position:.0f}%",
                      fill="#a6e3a1", font=("Courier", 8, "bold"))

        # Outlet line.
        c.create_line(160, y2, 160, 350, fill="#6c7086", width=5)
        c.create_text(160, 365, text="OUTLET", fill="#a6adc8",
                      font=("Courier", 8, "bold"))

        # Goal lines on the tank drawing (display only).
        for value, color, label in (
            (self.var_hh.get(), "#f38ba8", "HH goal"),
            (self.var_ll.get(), "#f9e2af", "LL goal"),
        ):
            y = y2 - int((y2 - y1) * value / MAX_LEVEL)
            c.create_line(x1, y, x2, y, fill=color, dash=(3, 3), width=1)
            c.create_text(x2 + 17, y, text=label, fill=color,
                          font=("Courier", 8, "bold"))

    def toggle_run(self):
        self.running = not self.running
        if self.running:
            self.btn_run.config(text="⏸ Pause", bg="#f9e2af")
            self._step()
        else:
            self.btn_run.config(text="▶ Start", bg="#a6e3a1")

    def reset_simulation(self):
        self.running = False
        self.btn_run.config(text="▶ Start", bg="#a6e3a1")
        self.tank = Tank(h0=1.5, drain_coeff=self.var_drain.get())
        self.valve = ControlValve()
        self.sim_time = 0.0
        self.t_buf = deque([0.0] * HISTORY_LEN, maxlen=HISTORY_LEN)
        self.level_buf = deque([1.5] * HISTORY_LEN, maxlen=HISTORY_LEN)
        self.valve_buf = deque([0.0] * HISTORY_LEN, maxlen=HISTORY_LEN)
        self._update_readouts(1.5)
        self._update_status()
        self._draw_tank()
        self._update_chart()


def main():
    app = TankSimulationApp()
    app.mainloop()


if __name__ == "__main__":
    main()
