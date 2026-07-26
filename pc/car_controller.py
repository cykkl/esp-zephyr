#!/usr/bin/env python3
"""Windows controller for the ESP-NOW car link."""

from __future__ import annotations

import argparse
import queue
import re
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

import serial


COMMANDS = ("STOP", "FORWARD", "BACKWARD", "LEFT", "RIGHT")
ANSI_ESCAPE = re.compile(r"\x1b\[[0-9;]*m")


class EspSerialLink:
    def __init__(self, port: str, baud: int) -> None:
        self.port = port
        self.baud = baud
        self.sequence = 0
        self.lines: queue.Queue[str] = queue.Queue()
        self._serial: serial.Serial | None = None
        self._write_lock = threading.Lock()
        self._stop_reader = threading.Event()
        self._reader: threading.Thread | None = None

    @property
    def connected(self) -> bool:
        return self._serial is not None and self._serial.is_open

    def connect(self) -> None:
        if self.connected:
            return

        connection = serial.Serial()
        connection.port = self.port
        connection.baudrate = self.baud
        connection.bytesize = serial.EIGHTBITS
        connection.parity = serial.PARITY_NONE
        connection.stopbits = serial.STOPBITS_ONE
        connection.timeout = 0.1
        connection.write_timeout = 0.5
        connection.dtr = False
        connection.rts = False
        connection.open()

        self._serial = connection
        self._stop_reader.clear()
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

    def disconnect(self) -> None:
        if self.connected:
            try:
                self.send_command("STOP", 0)
            except (OSError, serial.SerialException):
                pass

        self._stop_reader.set()
        connection = self._serial
        self._serial = None
        if connection is not None:
            connection.close()

    def send_command(self, command: str, speed: int) -> int:
        if command not in COMMANDS:
            raise ValueError(f"Unknown command: {command}")
        if not self.connected:
            raise serial.SerialException("Serial port is not connected")

        speed = 0 if command == "STOP" else max(0, min(int(speed), 100))
        sequence = self.sequence
        self.sequence = (self.sequence + 1) & 0xFFFF
        message = f"CAR,{sequence},{command},{speed}\n".encode("ascii")

        with self._write_lock:
            assert self._serial is not None
            self._serial.write(message)
            self._serial.flush()

        return sequence

    def _read_loop(self) -> None:
        while not self._stop_reader.is_set():
            connection = self._serial
            if connection is None:
                return

            try:
                raw = connection.readline()
            except (OSError, serial.SerialException) as error:
                self.lines.put(f"[串口错误] {error}")
                return

            if raw:
                line = raw.decode("utf-8", errors="replace").strip()
                line = ANSI_ESCAPE.sub("", line)
                if line:
                    self.lines.put(line)


class CarControllerApp:
    REPEAT_MS = 200

    def __init__(self, root: tk.Tk, port: str, baud: int) -> None:
        self.root = root
        self.link = EspSerialLink(port, baud)
        self.current_command = "STOP"

        root.title("ESP-NOW 小车控制器")
        root.minsize(620, 520)
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.bind("<FocusOut>", self._focus_out)

        self.port_var = tk.StringVar(value=port)
        self.status_var = tk.StringVar(value="未连接")
        self.command_var = tk.StringVar(value="STOP")
        self.speed_var = tk.IntVar(value=40)

        self._build_ui()
        self._bind_keys()
        self.root.after(100, self._poll_logs)
        self.root.after(self.REPEAT_MS, self._repeat_command)

    def _build_ui(self) -> None:
        connection = ttk.LabelFrame(self.root, text="连接")
        connection.pack(fill="x", padx=12, pady=10)

        ttk.Label(connection, text="串口").pack(side="left", padx=(10, 4), pady=8)
        ttk.Entry(connection, textvariable=self.port_var, width=10).pack(
            side="left", pady=8
        )
        self.connect_button = ttk.Button(
            connection, text="连接", command=self.toggle_connection
        )
        self.connect_button.pack(side="left", padx=8, pady=8)
        ttk.Label(connection, textvariable=self.status_var).pack(
            side="left", padx=8
        )

        controls = ttk.LabelFrame(self.root, text="方向控制（W/A/S/D 或方向键）")
        controls.pack(fill="x", padx=12, pady=4)

        button_frame = ttk.Frame(controls)
        button_frame.pack(side="left", padx=30, pady=12)

        forward = ttk.Button(button_frame, text="前进 W", width=13)
        left = ttk.Button(button_frame, text="左转 A", width=13)
        stop = ttk.Button(button_frame, text="停止 Space", width=13)
        right = ttk.Button(button_frame, text="右转 D", width=13)
        backward = ttk.Button(button_frame, text="后退 S", width=13)

        forward.grid(row=0, column=1, padx=4, pady=4)
        left.grid(row=1, column=0, padx=4, pady=4)
        stop.grid(row=1, column=1, padx=4, pady=4)
        right.grid(row=1, column=2, padx=4, pady=4)
        backward.grid(row=2, column=1, padx=4, pady=4)

        self._bind_button(forward, "FORWARD")
        self._bind_button(left, "LEFT")
        self._bind_button(right, "RIGHT")
        self._bind_button(backward, "BACKWARD")
        stop.configure(command=self.stop)

        speed_frame = ttk.Frame(controls)
        speed_frame.pack(side="left", fill="both", expand=True, padx=20, pady=12)
        ttk.Label(speed_frame, text="速度 0–100").pack()
        ttk.Scale(
            speed_frame,
            from_=0,
            to=100,
            orient="horizontal",
            variable=self.speed_var,
        ).pack(fill="x", pady=8)
        ttk.Label(
            speed_frame,
            textvariable=self.command_var,
            font=("Segoe UI", 18, "bold"),
        ).pack(pady=10)

        log_frame = ttk.LabelFrame(self.root, text="ESP 日志与 ACK")
        log_frame.pack(fill="both", expand=True, padx=12, pady=10)
        self.log = tk.Text(log_frame, height=12, state="disabled", wrap="none")
        self.log.pack(fill="both", expand=True, padx=6, pady=6)

    def _bind_button(self, button: ttk.Button, command: str) -> None:
        button.bind("<ButtonPress-1>", lambda _event: self.move(command))
        button.bind(
            "<ButtonRelease-1>",
            lambda _event: self.stop_if_current(command),
        )

    def _bind_keys(self) -> None:
        key_commands = {
            "w": "FORWARD",
            "Up": "FORWARD",
            "s": "BACKWARD",
            "Down": "BACKWARD",
            "a": "LEFT",
            "Left": "LEFT",
            "d": "RIGHT",
            "Right": "RIGHT",
        }
        for key, command in key_commands.items():
            self.root.bind(
                f"<KeyPress-{key}>",
                lambda _event, selected=command: self.move(selected),
            )
            self.root.bind(
                f"<KeyRelease-{key}>",
                lambda _event, selected=command: self.stop_if_current(selected),
            )
        self.root.bind("<KeyPress-space>", lambda _event: self.stop())

    def toggle_connection(self) -> None:
        if self.link.connected:
            self.stop()
            self.link.disconnect()
            self.status_var.set("未连接")
            self.connect_button.configure(text="连接")
            return

        self.link.port = self.port_var.get().strip()
        try:
            self.link.connect()
        except (OSError, serial.SerialException) as error:
            messagebox.showerror("串口连接失败", str(error))
            return

        self.status_var.set(f"已连接 {self.link.port} @ {self.link.baud}")
        self.connect_button.configure(text="断开")
        self._append_log("[PC] 已连接，按住方向键控制，松开自动停止")
        sequence = self.link.send_command("STOP", 0)
        self._append_log(f"[PC TX] seq={sequence} STOP speed=0")

    def move(self, command: str) -> None:
        if not self.link.connected:
            return
        if command == self.current_command:
            return
        self.current_command = command
        self.command_var.set(f"{command}  {self.speed_var.get()}%")
        self._send_current()

    def stop_if_current(self, command: str) -> None:
        if self.current_command == command:
            self.stop()

    def stop(self) -> None:
        was_moving = self.current_command != "STOP"
        self.current_command = "STOP"
        self.command_var.set("STOP")
        if was_moving:
            self._send_current()

    def _send_current(self) -> None:
        if not self.link.connected:
            return
        try:
            sequence = self.link.send_command(
                self.current_command, self.speed_var.get()
            )
            self._append_log(
                f"[PC TX] seq={sequence} {self.current_command} "
                f"speed={0 if self.current_command == 'STOP' else self.speed_var.get()}"
            )
        except (OSError, serial.SerialException) as error:
            self.status_var.set(f"串口错误：{error}")
            self.link.disconnect()
            self.connect_button.configure(text="连接")

    def _repeat_command(self) -> None:
        if self.current_command != "STOP":
            self._send_current()
        self.root.after(self.REPEAT_MS, self._repeat_command)

    def _poll_logs(self) -> None:
        while True:
            try:
                self._append_log(self.link.lines.get_nowait())
            except queue.Empty:
                break
        self.root.after(100, self._poll_logs)

    def _focus_out(self, _event: tk.Event) -> None:
        # Focus moving between widgets inside this window is harmless. Check
        # after Tk has completed the focus transition and stop only when the
        # entire application lost focus.
        self.root.after_idle(self._stop_if_window_unfocused)

    def _stop_if_window_unfocused(self) -> None:
        if self.root.focus_displayof() is None:
            self.stop()

    def _append_log(self, line: str) -> None:
        self.log.configure(state="normal")
        self.log.insert("end", line + "\n")
        self.log.see("end")
        self.log.configure(state="disabled")

    def close(self) -> None:
        self.stop()
        self.link.disconnect()
        self.root.destroy()


def run_cli(port: str, baud: int, command: str, speed: int, duration: float) -> int:
    link = EspSerialLink(port, baud)
    link.connect()
    command = command.upper()
    deadline = time.monotonic() + duration
    next_send = 0.0

    try:
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_send:
                sequence = link.send_command(command, speed)
                print(f"PC TX seq={sequence} {command} speed={speed}")
                next_send = now + 0.2

            try:
                print(link.lines.get(timeout=0.05))
            except queue.Empty:
                pass
    finally:
        sequence = link.send_command("STOP", 0)
        print(f"PC TX seq={sequence} STOP speed=0")
        time.sleep(0.2)
        while not link.lines.empty():
            print(link.lines.get_nowait())
        link.disconnect()

    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM17")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument(
        "--command",
        choices=("forward", "backward", "left", "right", "stop"),
        help="Run without the GUI and send one command repeatedly.",
    )
    parser.add_argument("--speed", type=int, default=40)
    parser.add_argument("--duration", type=float, default=1.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.command:
        return run_cli(
            args.port, args.baud, args.command, args.speed, args.duration
        )

    root = tk.Tk()
    CarControllerApp(root, args.port, args.baud)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
