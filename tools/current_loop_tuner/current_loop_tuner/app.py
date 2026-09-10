import queue
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

from .protocol import format_align, format_rate, format_set_id, format_set_iq, format_set_pi, format_status, format_stop
from .serial_worker import SerialWorker
from .trace import TraceBuffer
from .log_view import DEFAULT_LOG_MAX_LINES, append_log_line
from .plot_helpers import add_combined_legend


ALIGNMENT_TIMEOUT_S = 5.0

def alignment_request_should_release(response_kind: str | None, elapsed_s: float) -> bool:
    if response_kind in {"align_done", "align_error", "error"}:
        return True
    return response_kind is None and elapsed_s >= ALIGNMENT_TIMEOUT_S


class CurrentLoopApp:
    LOG_WINDOW_MAX_LINES = DEFAULT_LOG_MAX_LINES

    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("FOC 电流环调试器")
        self.root.geometry("1100x720")
        self.incoming = queue.Queue()
        self.worker = None
        self.trace = TraceBuffer(maxlen=5000)
        self.last_sample_time = None
        self.alignment_pending_since = None
        self.pi_values = {"id_kp": 2.0, "id_ki": 1.0, "iq_kp": 4.0, "iq_ki": 1.0}
        self.pi_window = None
        self.pi_vars = {}
        self.log_window = None
        self.log_text = None
        self.log_lines: list[str] = []
        self.log_paused = False
        self.log_pause_button = None
        self.log_auto_scroll = None
        self.connected = False
        self._build_controls()
        self._build_plot()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.after(50, self.poll_queue)
        self.root.after(100, self.update_plot)

    def _build_controls(self):
        controls = ttk.Frame(self.root, padding=8)
        controls.pack(fill=tk.X)

        self.port_var = tk.StringVar()
        self.baud_var = tk.StringVar(value="115200")
        self.id_var = tk.StringVar(value="50")
        self.iq_var = tk.StringVar(value="0")
        self.rate_var = tk.StringVar(value="50")
        self.status_var = tk.StringVar(value="未连接")
        self.data_var = tk.StringVar(value="没有遥测数据")

        ttk.Label(controls, text="串口").grid(row=0, column=0, padx=3, pady=3)
        self.port_box = ttk.Combobox(controls, textvariable=self.port_var, width=12)
        self.port_box.grid(row=0, column=1, padx=3, pady=3)
        ttk.Button(controls, text="刷新", command=self.refresh_ports).grid(row=0, column=2, padx=3)
        ttk.Label(controls, text="波特率").grid(row=0, column=3, padx=3)
        ttk.Entry(controls, textvariable=self.baud_var, width=9).grid(row=0, column=4, padx=3)
        self.connect_button = ttk.Button(controls, text="连接", command=self.toggle_connection)
        self.connect_button.grid(row=0, column=5, padx=8)

        ttk.Label(controls, text="Id目标(mA)").grid(row=1, column=0, padx=3, pady=3)
        ttk.Entry(controls, textvariable=self.id_var, width=9).grid(row=1, column=1, padx=3)
        ttk.Label(controls, text="Iq目标(mA)").grid(row=1, column=2, padx=3)
        ttk.Entry(controls, textvariable=self.iq_var, width=9).grid(row=1, column=3, padx=3)
        ttk.Button(controls, text="应用目标", command=self.apply_targets).grid(row=1, column=4, padx=5)
        ttk.Button(controls, text="STOP / 清零", command=self.stop_targets).grid(row=1, column=5, padx=5)

        ttk.Label(controls, text="遥测Hz").grid(row=2, column=0, padx=3, pady=3)
        ttk.Entry(controls, textvariable=self.rate_var, width=9).grid(row=2, column=1, padx=3)
        ttk.Button(controls, text="设置遥测率", command=self.apply_rate).grid(row=2, column=2, padx=5)
        self.align_button = ttk.Button(controls, text="电角度零点对齐", command=self.align_zero)
        self.align_button.grid(row=2, column=3, columnspan=2, padx=5, sticky=tk.W)
        ttk.Button(controls, text="串口日志", command=self.open_log_window).grid(row=2, column=5, padx=5)
        ttk.Button(controls, text="PI parameters", command=self.open_pi_window).grid(row=2, column=6, padx=5)
        ttk.Button(controls, text="Save CSV", command=self.save_csv).grid(row=2, column=7, padx=5)

        ttk.Label(controls, textvariable=self.status_var).grid(row=3, column=0, columnspan=6, sticky=tk.W, padx=3, pady=3)
        ttk.Label(controls, textvariable=self.data_var).grid(row=4, column=0, columnspan=6, sticky=tk.W, padx=3)
        controls.columnconfigure(5, weight=1)
        self.refresh_ports()

    def _build_plot(self):
        figure = Figure(figsize=(10, 6), dpi=100)
        self.ax_current = figure.add_subplot(211)
        self.ax_voltage = figure.add_subplot(212)
        self.ax_current.set_ylabel("Current (A)")
        self.ax_voltage.set_ylabel("Voltage (V)")
        self.ax_speed = self.ax_voltage.twinx()
        self.ax_speed.set_ylabel("Speed (rad/s)")
        self.ax_voltage.set_xlabel("Time (s)")
        self.ax_current.grid(True)
        self.ax_voltage.grid(True)
        self.current_lines = {
            "id_target": self.ax_current.plot([], [], label="Id target")[0],

            "id": self.ax_current.plot([], [], label="Id")[0],
            "iq_target": self.ax_current.plot([], [], label="Iq target")[0],

            "iq": self.ax_current.plot([], [], label="Iq")[0],
        }
        self.voltage_lines = {
            "vd": self.ax_voltage.plot([], [], label="Vd")[0],
            "vq": self.ax_voltage.plot([], [], label="Vq")[0],
            "speed": self.ax_speed.plot([], [], label="Speed", color="tab:green")[0],
        }
        self.ax_current.legend(loc="upper right", ncol=3, fontsize="small")
        add_combined_legend(self.ax_voltage, (self.ax_speed,))
        self.canvas = FigureCanvasTkAgg(figure, master=self.root)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=8, pady=8)
        self.figure = figure

    def refresh_ports(self):
        try:
            from serial.tools import list_ports
            ports = [item.device for item in list_ports.comports()]
        except ModuleNotFoundError:
            ports = []
            self.status_var.set("未连接；缺少 pyserial，请安装 requirements.txt")
        self.port_box["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def toggle_connection(self):
        if self.connected:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        port = self.port_var.get().strip()
        if not port:
            messagebox.showwarning("串口", "请选择 COM 口")
            return
        try:
            baudrate = int(self.baud_var.get())
            rate = int(self.rate_var.get())
            worker = SerialWorker(port, baudrate, self.incoming)
            worker.start()
            self.worker = worker
            self.connected = True
            self.trace.clear()
            self.last_sample_time = None
            self.alignment_pending_since = None
            self.connect_button.configure(text="断开")
            self.status_var.set(f"已连接 {port}；正在同步状态")
            self.worker.send(format_rate(rate))
            self.worker.send(format_status())
        except (ValueError, RuntimeError, OSError) as exc:
            if self.worker is not None:
                self.worker.stop()
                self.worker = None
            self.connected = False
            self.status_var.set(f"连接失败：{exc}")

    def disconnect(self):
        worker = self.worker
        self.worker = None
        self.connected = False
        self.alignment_pending_since = None
        if worker is not None:
            try:
                worker.send(format_stop())
            except Exception:
                pass
            worker.stop()
        self.connect_button.configure(text="连接")
        self.status_var.set("已断开")

    def _send(self, data: bytes):
        if not self.connected or self.worker is None:
            raise RuntimeError("请先连接串口")
        self.worker.send(data)

    def apply_targets(self):
        try:
            id_ma = int(self.id_var.get())
            iq_ma = int(self.iq_var.get())
            self._send(format_set_id(id_ma))
            self._send(format_set_iq(iq_ma))
            self.status_var.set(f"已发送 Id={id_ma} mA, Iq={iq_ma} mA；等待固件确认")
        except (ValueError, RuntimeError) as exc:
            messagebox.showerror("目标电流", str(exc))

    def stop_targets(self):
        try:
            self._send(format_stop())
            self.id_var.set("0")
            self.iq_var.set("0")
            self.status_var.set("已发送 STOP，目标正在回零")
        except (RuntimeError,) as exc:
            messagebox.showerror("STOP", str(exc))

    def apply_rate(self):
        try:
            rate = int(self.rate_var.get())
            self._send(format_rate(rate))
            self.status_var.set(f"已请求遥测率 {rate} Hz")
        except (ValueError, RuntimeError) as exc:
            messagebox.showerror("遥测率", str(exc))

    def align_zero(self):
        if not self.connected:
            messagebox.showerror("零点对齐", "请先连接串口")
            return
        confirmed = messagebox.askyesno(
            "确认零点对齐",
            "对齐期间电机将通电约 1 秒并可能轻微动作。请确认机械部分安全。",
        )
        if not confirmed:
            return
        try:
            self._send(format_align())
            self.alignment_pending_since = time.monotonic()
            self.align_button.configure(state=tk.DISABLED)
            self.status_var.set("已请求零点对齐：固件将先清零，再执行对齐")
        except RuntimeError as exc:
            messagebox.showerror("零点对齐", str(exc))

    def open_pi_window(self):
        if self.pi_window is not None and self.pi_window.winfo_exists():
            self.pi_window.deiconify()
            self.pi_window.lift()
            return

        window = tk.Toplevel(self.root)
        window.title("Current-loop PI parameters")
        window.geometry("360x240")
        frame = ttk.Frame(window, padding=12)
        frame.pack(fill=tk.BOTH, expand=True)

        labels = (
            ("Id Kp", "id_kp"),
            ("Id Ki", "id_ki"),
            ("Iq Kp", "iq_kp"),
            ("Iq Ki", "iq_ki"),
        )
        self.pi_vars = {}
        for row, (label, key) in enumerate(labels):
            ttk.Label(frame, text=label).grid(row=row, column=0, sticky=tk.W, padx=4, pady=5)
            variable = tk.StringVar(value=f"{self.pi_values[key]:.6f}")
            self.pi_vars[key] = variable
            ttk.Entry(frame, textvariable=variable, width=18).grid(
                row=row, column=1, sticky=tk.EW, padx=4, pady=5
            )
        ttk.Label(frame, text="Kp: 0..10    Ki: 0..50").grid(
            row=4, column=0, columnspan=2, sticky=tk.W, padx=4, pady=(4, 10)
        )
        ttk.Button(frame, text="Apply PI", command=self.apply_pi).grid(
            row=5, column=0, columnspan=2, pady=4
        )
        frame.columnconfigure(1, weight=1)

        def close():
            self.pi_window = None
            self.pi_vars = {}
            window.destroy()

        window.protocol("WM_DELETE_WINDOW", close)
        self.pi_window = window

    def _update_pi_from_fields(self, fields):
        names = (
            ("id_kp", "PI_ID_KP"),
            ("id_ki", "PI_ID_KI"),
            ("iq_kp", "PI_IQ_KP"),
            ("iq_ki", "PI_IQ_KI"),
        )
        for key, field_name in names:
            if field_name not in fields:
                continue
            try:
                self.pi_values[key] = float(fields[field_name])
            except (TypeError, ValueError):
                continue
            variable = self.pi_vars.get(key)
            if variable is not None:
                variable.set(f"{self.pi_values[key]:.6f}")

    def apply_pi(self):
        try:
            values = {
                key: float(variable.get())
                for key, variable in self.pi_vars.items()
            }
            command = format_set_pi(
                values["id_kp"],
                values["id_ki"],
                values["iq_kp"],
                values["iq_ki"],
            )
            self._send(command)
            self.status_var.set("PI parameters sent; waiting for firmware acknowledgement")
        except (KeyError, ValueError, RuntimeError) as exc:
            messagebox.showerror("PI parameters", str(exc))

    def open_log_window(self):
        if self.log_window is not None and self.log_window.winfo_exists():
            self.log_window.deiconify()
            self.log_window.lift()
            return
        window = tk.Toplevel(self.root)
        window.title("FOC 串口原始数据")
        window.geometry("1100x650")
        window.protocol("WM_DELETE_WINDOW", self.close_log_window)
        frame = ttk.Frame(window, padding=8)
        frame.pack(fill=tk.BOTH, expand=True)
        toolbar = ttk.Frame(frame)
        toolbar.grid(row=0, column=0, sticky="ew", pady=(0, 6))
        ttk.Button(toolbar, text="清空", command=self.clear_log_window).pack(side=tk.LEFT, padx=(0, 6))
        self.log_pause_button = ttk.Button(toolbar, text="暂停", command=self.toggle_log_pause)
        self.log_pause_button.pack(side=tk.LEFT, padx=(0, 6))
        self.log_auto_scroll = tk.BooleanVar(value=True)
        ttk.Checkbutton(toolbar, text="自动滚动到底部", variable=self.log_auto_scroll).pack(side=tk.LEFT)
        ttk.Label(toolbar, text=f"保留最近 {self.LOG_WINDOW_MAX_LINES} 行").pack(side=tk.RIGHT)

        text = tk.Text(frame, wrap=tk.NONE, state=tk.DISABLED, undo=False, font=("Consolas", 10))
        yscroll = ttk.Scrollbar(frame, orient=tk.VERTICAL, command=text.yview)
        xscroll = ttk.Scrollbar(frame, orient=tk.HORIZONTAL, command=text.xview)
        text.configure(yscrollcommand=yscroll.set, xscrollcommand=xscroll.set)
        text.grid(row=1, column=0, sticky="nsew")
        yscroll.grid(row=1, column=1, sticky="ns")
        xscroll.grid(row=2, column=0, sticky="ew")
        frame.rowconfigure(1, weight=1)
        frame.columnconfigure(0, weight=1)
        self.log_window = window
        self.log_text = text
        self._render_log_window()

    def close_log_window(self):
        if self.log_window is not None and self.log_window.winfo_exists():
            self.log_window.destroy()
        self.log_window = None
        self.log_text = None
        self.log_pause_button = None

    def clear_log_window(self):
        self.log_lines.clear()
        self._render_log_window()

    def toggle_log_pause(self):
        self.log_paused = not self.log_paused
        if self.log_pause_button is not None:
            self.log_pause_button.configure(text="继续" if self.log_paused else "暂停")
        if not self.log_paused:
            self._render_log_window()

    def append_log_line(self, line: str):
        self.log_lines = append_log_line(self.log_lines, line, self.LOG_WINDOW_MAX_LINES)
        if not self.log_paused:
            self._render_log_window()

    def _render_log_window(self):
        if self.log_text is None or self.log_window is None or not self.log_window.winfo_exists():
            return
        self.log_text.configure(state=tk.NORMAL)
        self.log_text.delete("1.0", tk.END)
        if self.log_lines:
            self.log_text.insert(tk.END, "\n".join(self.log_lines) + "\n")
        self.log_text.configure(state=tk.DISABLED)
        if self.log_auto_scroll is not None and self.log_auto_scroll.get():
            self.log_text.see(tk.END)
    def save_csv(self):
        path = filedialog.asksaveasfilename(
            title="保存电流环数据",
            defaultextension=".csv",
            filetypes=[("CSV", "*.csv"), ("All files", "*.*")],
        )
        if path:
            self.trace.write_csv(path)
            self.status_var.set(f"已保存 {len(self.trace.snapshot())} 条数据")

    def poll_queue(self):
        while True:
            try:
                kind, payload = self.incoming.get_nowait()
            except queue.Empty:
                break
            if kind == "telemetry":
                self.append_log_line(payload.raw_line)
                self.trace.append(payload)
                self.last_sample_time = time.monotonic()
                self.data_var.set(
                    f"Id={payload.id_a * 1000:.0f}/{payload.id_target_a * 1000:.0f} mA | "
                    f"Iq={payload.iq_a * 1000:.0f}/{payload.iq_target_a * 1000:.0f} mA | "
                    f"speed={payload.speed_rad_s:.2f} rad/s | "
                    f"age={payload.current_age_us}/{payload.angle_age_us} us"
                )
            elif kind == "response":
                response = payload
                self.append_log_line(response.message)
                if response.kind == "ok_pi":
                    self._update_pi_from_fields(response.fields)
                    self.status_var.set(response.message)
                elif response.kind == "align_done":
                    self.alignment_pending_since = None
                    self.align_button.configure(state=tk.NORMAL)
                    offset = response.fields.get("offset", "?")
                    self.status_var.set(f"零点对齐完成 offset={offset} rad；目标保持 0")
                    messagebox.showinfo(
                        "零点对齐成功",
                        f"电角度零点对齐成功\n零点偏置：{offset} rad",
                    )
                elif response.kind == "align_error":
                    self.alignment_pending_since = None
                    self.align_button.configure(state=tk.NORMAL)
                    self.status_var.set(f"零点对齐失败：{response.message}")
                elif response.kind == "status":
                    self._update_pi_from_fields(response.fields)
                    self.status_var.set(response.message)
                elif response.kind == "error":
                    if alignment_request_should_release(response.kind, 0.0):
                        self.alignment_pending_since = None
                        self.align_button.configure(state=tk.NORMAL)
                    self.status_var.set(f"固件拒绝命令：{response.message}")
                else:
                    self.status_var.set(response.message)
            elif kind == "error":
                self.append_log_line(f"SERIAL ERROR: {payload}")
                self.status_var.set(f"串口错误：{payload}")
                self.disconnect()
            elif kind == "log":
                self.append_log_line(payload)
        if self.alignment_pending_since is not None:
            elapsed_s = time.monotonic() - self.alignment_pending_since
            if alignment_request_should_release(None, elapsed_s):
                self.alignment_pending_since = None
                self.align_button.configure(state=tk.NORMAL)
                self.status_var.set("零点对齐等待超时，请确认固件串口协议和设备状态")
        if self.last_sample_time is None:
            stale = True
        else:
            stale = time.monotonic() - self.last_sample_time > 0.5
        if self.connected and stale:
            self.status_var.set("已连接，但遥测数据超时（>500 ms）")
        self.root.after(50, self.poll_queue)

    def update_plot(self):
        samples = self.trace.snapshot()
        if samples:
            start = samples[0].timestamp_us
            x = [(sample.timestamp_us - start) / 1_000_000.0 for sample in samples]
            values = {
                "id_target": [s.id_target_a for s in samples],

                "id": [s.id_a for s in samples],
                "iq_target": [s.iq_target_a for s in samples],

                "iq": [s.iq_a for s in samples],
            }
            for name, line in self.current_lines.items():
                line.set_data(x, values[name])
            voltage_values = {
                "vd": [s.vd_v for s in samples],
                "vq": [s.vq_v for s in samples],
                "speed": [s.speed_rad_s for s in samples],
            }
            for name, line in self.voltage_lines.items():
                line.set_data(x, voltage_values[name])
            right = max(x[-1], 1.0)
            self.ax_current.set_xlim(max(0.0, right - 20.0), right)
            self.ax_voltage.set_xlim(max(0.0, right - 20.0), right)
            self.ax_speed.set_xlim(max(0.0, right - 20.0), right)
            self.ax_current.relim()
            self.ax_current.autoscale_view(scalex=False, scaley=True)
            self.ax_voltage.relim()
            self.ax_voltage.autoscale_view(scalex=False, scaley=True)
            self.ax_speed.relim()
            self.ax_speed.autoscale_view(scalex=False, scaley=True)
            self.canvas.draw_idle()
        self.root.after(100, self.update_plot)

    def on_close(self):
        self.disconnect()
        self.close_log_window()
        self.root.destroy()


def main():
    root = tk.Tk()
    CurrentLoopApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()