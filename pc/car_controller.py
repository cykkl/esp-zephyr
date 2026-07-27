#!/usr/bin/env python3
"""Windows controller for the ESP-NOW car link."""

from __future__ import annotations

import argparse
import csv
import math
import queue
import re
import struct
import threading
import time
import tkinter as tk
from collections import deque
from tkinter import filedialog, messagebox
from typing import Any

import serial


COMMANDS = (
    "STOP",
    "FORWARD",
    "BACKWARD",
    "LEFT",
    "RIGHT",
    "TRACK_ON",
    "TRACK_OFF",
)
ANSI_ESCAPE = re.compile(r"\x1b\[[0-9;]*m")
FRAME_SYNC = b"\xA5\x5A"
FRAME_VERSION = 3
FRAME_TYPE_COMMAND = 1
FRAME_TYPE_TELEMETRY = 3
FRAME_TYPE_ACK = 4
FRAME_TYPE_PARAMETER = 5
FRAME_HEADER_SIZE = 7
FRAME_MAX_PAYLOAD = 70
COMMAND_IDS = {name: index for index, name in enumerate(COMMANDS)}
PARAMETER_IDS = {
    "BASE": 0,
    "LIMIT": 1,
    "MINIMUM": 2,
    "SLEW": 3,
    "KP": 4,
    "KD": 5,
    "GYRO": 6,
    "SPEED_FULL_SCALE": 9,
    "SPEED_KP": 10,
    "SPEED_KI": 11,
    "HEADING_KP": 12,
    "HEADING_KI": 13,
    "HEADING_KD": 14,
    "HEADING_LIMIT": 15,
    "HEADING_SIGN": 16,
    "TURN_SPEED": 17,
    "TURN_ANGLE": 18,
    "TURN_TOLERANCE": 19,
    "TURN_DETECT_CYCLES": 20,
    "TURN_SIGN": 21,
    "LINE_TRIM_LIMIT": 22,
}
TRACK_STATES = (
    "STOP",
    "FORWARD",
    "LINE_LEFT",
    "LINE_RIGHT",
    "TURN_LEFT",
    "TURN_RIGHT",
    "LINE_STOP",
    "LOST",
    "OBSTACLE",
    "SENSOR_FAULT",
    "TURN_TIMEOUT",
    "MANUAL",
)


def decode_telemetry(payload: bytes, sequence: int) -> dict[str, Any]:
    """Decode the v3 fixed-layout telemetry payload."""
    i16 = lambda offset: struct.unpack_from("<h", payload, offset)[0]
    u16 = lambda offset: struct.unpack_from("<H", payload, offset)[0]
    i8 = lambda offset: struct.unpack_from("<b", payload, offset)[0]
    return {
        "sequence": sequence,
        "uptime_ms": struct.unpack_from("<I", payload, 0)[0],
        "gyro_x": i16(4),
        "gyro_y": i16(6),
        "gyro_z": i16(8),
        "roll": i16(10),
        "pitch": i16(12),
        "yaw": i16(14),
        "flags": payload[16],
        "heading_target": i16(17),
        "heading_error": i16(19),
        "heading_correction": i8(21),
        "left_duty": i8(22),
        "right_duty": i8(23),
        "tracking_enabled": payload[24],
        "line_bits": payload[25],
        "line_error": i8(26),
        "line_active": payload[27],
        "track_state": payload[28],
        "line_correction": i8(29),
        "target_left_cps": i16(30),
        "target_right_cps": i16(32),
        "measured_left_cps": i16(34),
        "measured_right_cps": i16(36),
        "base_speed": payload[38],
        "motor_limit": payload[39],
        "motor_minimum": payload[40],
        "output_slew": payload[41],
        "kp_percent": u16(42),
        "kd_percent": u16(44),
        "gyro_percent": u16(46),
        "speed_full_scale_cps": u16(48),
        "speed_kp_percent": u16(50),
        "speed_ki_percent": u16(52),
        "heading_kp_percent": u16(54),
        "heading_ki_percent": u16(56),
        "heading_kd_percent": u16(58),
        "heading_limit": payload[60],
        "turn_speed": payload[61],
        "turn_angle_deg": payload[62],
        "turn_tolerance_deg": payload[63],
        "turn_sign": i8(64),
        "turn_detect_cycles": payload[65],
        "imu_age_ms": u16(66),
        "line_trim_limit": payload[68],
        "heading_sign": i8(69),
    }


def frame_crc16(data: bytes | bytearray) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = (
                ((crc << 1) ^ 0x1021) & 0xFFFF
                if crc & 0x8000
                else (crc << 1) & 0xFFFF
            )
    return crc


def encode_frame(frame_type: int, sequence: int, payload: bytes) -> bytes:
    header = struct.pack(
        "<2sBBBH",
        FRAME_SYNC,
        FRAME_VERSION,
        frame_type,
        len(payload),
        sequence,
    )
    body = header + payload
    return body + struct.pack("<H", frame_crc16(body[2:]))


class EspSerialLink:
    def __init__(self, port: str, baud: int) -> None:
        self.port = port
        self.baud = baud
        self.sequence = 0
        self.lines: queue.Queue[Any] = queue.Queue()
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

        speed = (
            0
            if command in ("STOP", "TRACK_ON", "TRACK_OFF")
            else max(0, min(int(speed), 100))
        )
        sequence = self.sequence
        self.sequence = (self.sequence + 1) & 0xFFFF
        payload = struct.pack("<BB", COMMAND_IDS[command], speed)
        message = encode_frame(FRAME_TYPE_COMMAND, sequence, payload)

        with self._write_lock:
            assert self._serial is not None
            self._serial.write(message)
            self._serial.flush()

        return sequence

    def send_parameter(self, name: str, value: int) -> int:
        if name not in PARAMETER_IDS:
            raise ValueError(f"Unknown parameter: {name}")
        if not self.connected:
            raise serial.SerialException("Serial port is not connected")
        sequence = self.sequence
        self.sequence = (self.sequence + 1) & 0xFFFF
        payload = struct.pack("<Bi", PARAMETER_IDS[name], int(value))
        message = encode_frame(FRAME_TYPE_PARAMETER, sequence, payload)
        with self._write_lock:
            assert self._serial is not None
            self._serial.write(message)
            self._serial.flush()
        return sequence

    def _read_loop(self) -> None:
        frame = bytearray()
        expected_length = 0
        text = bytearray()

        def emit_text() -> None:
            if not text:
                return
            line = text.decode("utf-8", errors="replace").strip()
            text.clear()
            line = ANSI_ESCAPE.sub("", line)
            if line:
                self.lines.put(line)

        def emit_frame(raw: bytes) -> None:
            frame_type = raw[3]
            payload_size = raw[4]
            sequence = struct.unpack_from("<H", raw, 5)[0]
            payload = raw[
                FRAME_HEADER_SIZE:FRAME_HEADER_SIZE + payload_size
            ]
            received_crc = struct.unpack_from(
                "<H", raw, FRAME_HEADER_SIZE + payload_size
            )[0]
            if received_crc != frame_crc16(raw[2:-2]):
                self.lines.put("[FRAME CRC ERROR]")
                return
            if frame_type == FRAME_TYPE_ACK and payload_size == 3:
                command_id, speed, status = struct.unpack("<BBB", payload)
                command = (
                    COMMANDS[command_id]
                    if command_id < len(COMMANDS)
                    else "INVALID"
                )
                prefix = "ESP,ACK" if status == 0 else "ESP,NACK"
                self.lines.put(
                    f"{prefix},{sequence},{command},{speed},{status}"
                )
            elif frame_type == FRAME_TYPE_TELEMETRY and payload_size == 70:
                self.lines.put(decode_telemetry(payload, sequence))
            else:
                self.lines.put(
                    f"[FRAME] type={frame_type} seq={sequence} "
                    f"len={payload_size}"
                )

        while not self._stop_reader.is_set():
            connection = self._serial
            if connection is None:
                return

            try:
                raw = connection.read(256)
            except (OSError, serial.SerialException) as error:
                self.lines.put(f"[串口错误] {error}")
                return

            for value in raw:
                if frame or value == FRAME_SYNC[0]:
                    if not frame:
                        frame.append(value)
                        continue
                    if len(frame) == 1 and value != FRAME_SYNC[1]:
                        frame.clear()
                        expected_length = 0
                        if value == FRAME_SYNC[0]:
                            frame.append(value)
                        elif value == 10:
                            emit_text()
                        elif value != 13:
                            text.append(value)
                        continue
                    frame.append(value)
                    if len(frame) == FRAME_HEADER_SIZE:
                        payload_size = frame[4]
                        if (
                            frame[2] != FRAME_VERSION
                            or payload_size > FRAME_MAX_PAYLOAD
                        ):
                            frame.clear()
                            expected_length = 0
                            continue
                        expected_length = (
                            FRAME_HEADER_SIZE + payload_size + 2
                        )
                    if (
                        expected_length
                        and len(frame) == expected_length
                    ):
                        emit_frame(bytes(frame))
                        frame.clear()
                        expected_length = 0
                    elif (
                        len(frame)
                        > FRAME_HEADER_SIZE + FRAME_MAX_PAYLOAD + 2
                    ):
                        frame.clear()
                        expected_length = 0
                    continue

                if value == 10:
                    emit_text()
                elif value != 13:
                    text.append(value)


class SpeedGauge(tk.Canvas):
    """Neon circular setpoint gauge."""

    BG = "#ffffff"
    TRACK = "#dce7f0"
    CYAN = "#00a6c7"
    TEXT = "#11243d"
    MUTED = "#7890a8"

    def __init__(self, parent: tk.Widget, value: int) -> None:
        super().__init__(
            parent,
            width=340,
            height=270,
            bg=self.BG,
            highlightthickness=0,
        )
        self._arc = None
        self._value_text = None
        self._draw()
        self.set_value(value)

    def _draw(self) -> None:
        cx, cy, radius = 170, 138, 104
        box = (cx - radius, cy - radius, cx + radius, cy + radius)
        self.create_arc(
            box,
            start=210,
            extent=-240,
            style="arc",
            width=16,
            outline=self.TRACK,
        )
        self._arc = self.create_arc(
            box,
            start=210,
            extent=0,
            style="arc",
            width=16,
            outline=self.CYAN,
        )

        for tick in range(11):
            angle = math.radians(210 - tick * 24)
            inner = radius - (18 if tick % 5 == 0 else 12)
            outer = radius + 2
            x1 = cx + math.cos(angle) * inner
            y1 = cy - math.sin(angle) * inner
            x2 = cx + math.cos(angle) * outer
            y2 = cy - math.sin(angle) * outer
            self.create_line(
                x1,
                y1,
                x2,
                y2,
                fill=self.TEXT if tick % 5 == 0 else self.MUTED,
                width=2,
            )

        self._value_text = self.create_text(
            cx,
            cy - 4,
            text="0",
            fill=self.TEXT,
            font=("Bahnschrift SemiBold", 54),
        )
        self.create_text(
            cx,
            cy + 42,
            text="POWER  /  %",
            fill=self.MUTED,
            font=("Bahnschrift", 10),
        )
        self.create_text(
            44,
            239,
            text="0",
            fill=self.MUTED,
            font=("Bahnschrift", 9),
        )
        self.create_text(
            296,
            239,
            text="100",
            fill=self.MUTED,
            font=("Bahnschrift", 9),
        )

    def set_value(self, value: int) -> None:
        value = max(0, min(int(float(value)), 100))
        assert self._arc is not None and self._value_text is not None
        color = "#f02d8c" if value >= 80 else self.CYAN
        self.itemconfigure(self._arc, extent=-240 * value / 100, outline=color)
        self.itemconfigure(self._value_text, text=str(value))


class HistoryPlot(tk.Canvas):
    """Small dependency-free scrolling plot rendered by Tk itself."""

    def __init__(
        self,
        parent: tk.Widget,
        title: str,
        series: tuple[tuple[str, str, str, float], ...],
    ) -> None:
        super().__init__(
            parent,
            height=190,
            bg="#ffffff",
            highlightbackground="#cbd9e6",
            highlightthickness=1,
        )
        self.title = title
        self.series = series

    def redraw(self, samples: list[dict[str, Any]]) -> None:
        self.delete("all")
        width = max(self.winfo_width(), 420)
        height = max(self.winfo_height(), 190)
        left, top, right, bottom = 54, 28, width - 16, height - 28
        self.create_text(
            12, 9, anchor="nw", text=self.title,
            fill="#11243d", font=("Bahnschrift SemiBold", 10),
        )
        if len(samples) < 2:
            self.create_text(
                width / 2, height / 2, text="等待遥测数据",
                fill="#7890a8", font=("Microsoft YaHei UI", 10),
            )
            return

        values: list[float] = []
        for key, _label, _color, scale in self.series:
            values.extend(float(item[key]) * scale for item in samples)
        low, high = min(values), max(values)
        if math.isclose(low, high):
            low -= 1.0
            high += 1.0
        margin = (high - low) * 0.12
        low -= margin
        high += margin

        for grid in range(5):
            y = top + (bottom - top) * grid / 4
            value = high - (high - low) * grid / 4
            self.create_line(left, y, right, y, fill="#e6edf3")
            self.create_text(
                left - 5, y, anchor="e", text=f"{value:.1f}",
                fill="#7890a8", font=("Cascadia Mono", 7),
            )

        count = len(samples)
        for key, label, color, scale in self.series:
            points: list[float] = []
            for index, item in enumerate(samples):
                x = left + (right - left) * index / (count - 1)
                value = float(item[key]) * scale
                y = bottom - (value - low) * (bottom - top) / (high - low)
                points.extend((x, y))
            self.create_line(*points, fill=color, width=2, smooth=False)

        legend_x = right
        for _key, label, color, _scale in reversed(self.series):
            legend_x -= 78
            self.create_line(
                legend_x, 14, legend_x + 15, 14, fill=color, width=3
            )
            self.create_text(
                legend_x + 19, 14, anchor="w", text=label,
                fill="#52677d", font=("Bahnschrift", 8),
            )


class TuningWindow:
    PARAMS = (
        ("BASE", "base_speed", 0, 100),
        ("LIMIT", "motor_limit", 1, 100),
        ("MINIMUM", "motor_minimum", 0, 80),
        ("SLEW", "output_slew", 1, 20),
        ("LINE_TRIM_LIMIT", "line_trim_limit", 0, 15),
        ("HEADING_KP", "heading_kp_percent", 0, 300),
        ("HEADING_KI", "heading_ki_percent", 0, 300),
        ("HEADING_KD", "heading_kd_percent", 0, 300),
        ("HEADING_LIMIT", "heading_limit", 0, 40),
        ("HEADING_SIGN", "heading_sign", -1, 1),
        ("TURN_SPEED", "turn_speed", 1, 60),
        ("TURN_ANGLE", "turn_angle_deg", 30, 180),
        ("TURN_TOLERANCE", "turn_tolerance_deg", 1, 20),
        ("TURN_DETECT_CYCLES", "turn_detect_cycles", 1, 20),
        ("TURN_SIGN", "turn_sign", -1, 1),
        ("SPEED_FULL_SCALE", "speed_full_scale_cps", 100, 10000),
        ("SPEED_KP", "speed_kp_percent", 0, 300),
        ("SPEED_KI", "speed_ki_percent", 0, 300),
    )

    def __init__(self, app: "CarControllerApp") -> None:
        self.app = app
        self.window = tk.Toplevel(app.root)
        self.window.title("循迹实时曲线与参数调节")
        self.window.geometry("1280x820")
        self.window.configure(bg=app.BG)
        self.window.protocol("WM_DELETE_WINDOW", self.close)
        self.variables: dict[str, tk.IntVar] = {}
        self.entries: dict[str, tk.Entry] = {}

        charts = tk.Frame(self.window, bg=app.BG)
        charts.pack(side="left", fill="both", expand=True, padx=12, pady=12)
        self.line_plot = HistoryPlot(
            charts, "灰度位置 / 修正",
            (
                ("line_error", "line error", app.CYAN, 1.0),
                ("line_correction", "line trim", app.MAGENTA, 1.0),
            ),
        )
        self.heading_plot = HistoryPlot(
            charts, "IMU 航向闭环",
            (
                ("heading_error", "yaw error", app.ORANGE, 0.01),
                ("gyro_z", "gyro Z", app.CYAN, 0.1),
                ("heading_correction", "correction", app.MAGENTA, 1.0),
            ),
        )
        self.motor_plot = HistoryPlot(
            charts, "编码器目标 / 实测速度",
            (
                ("target_left_cps", "target L", app.MAGENTA, 1.0),
                ("measured_left_cps", "speed L", "#8b5cf6", 1.0),
                ("target_right_cps", "target R", app.CYAN, 1.0),
                ("measured_right_cps", "speed R", app.LIME, 1.0),
            ),
        )
        for plot in (self.line_plot, self.heading_plot, self.motor_plot):
            plot.pack(fill="both", expand=True, pady=(0, 10))

        controls = tk.Frame(
            self.window, bg=app.PANEL,
            highlightbackground=app.EDGE, highlightthickness=1,
        )
        controls.pack(side="right", fill="y", padx=(0, 12), pady=12)
        tk.Label(
            controls, text="参数在线调节", bg=app.PANEL, fg=app.TEXT,
            font=("Microsoft YaHei UI", 13, "bold"),
        ).grid(row=0, column=0, columnspan=3, sticky="w", padx=12, pady=12)

        for row, (name, _key, minimum, maximum) in enumerate(
            self.PARAMS, start=1
        ):
            tk.Label(
                controls, text=name, bg=app.PANEL, fg=app.MUTED,
                font=("Cascadia Mono", 8),
            ).grid(row=row, column=0, sticky="w", padx=(12, 6), pady=2)
            variable = tk.IntVar(value=0)
            entry = tk.Entry(
                controls, textvariable=variable, width=8,
                justify="right", relief="solid", bd=1,
            )
            entry.grid(row=row, column=1, padx=4, pady=2)
            tk.Button(
                controls, text="SET",
                command=lambda selected=name: self.apply(selected),
                bg=app.CYAN, fg="#ffffff", relief="flat", padx=8,
            ).grid(row=row, column=2, padx=(4, 12), pady=2)
            self.variables[name] = variable
            self.entries[name] = entry
            entry.bind(
                "<Return>", lambda _event, selected=name: self.apply(selected)
            )
            entry.configure(
                validate="focusout",
                validatecommand=(
                    self.window.register(
                        lambda text, lo=minimum, hi=maximum:
                        text.lstrip("-").isdigit()
                        and lo <= int(text) <= hi
                    ),
                    "%P",
                ),
            )

        button_row = len(self.PARAMS) + 1
        tk.Button(
            controls, text="导出当前曲线 CSV", command=self.export_csv,
            bg=app.MAGENTA, fg="#ffffff", relief="flat", pady=8,
        ).grid(
            row=button_row, column=0, columnspan=3,
            sticky="ew", padx=12, pady=(12, 6),
        )
        self.status = tk.StringVar(value="等待遥测")
        tk.Label(
            controls, textvariable=self.status, bg=app.PANEL,
            fg=app.ORANGE, font=("Microsoft YaHei UI", 9),
            wraplength=260, justify="left",
        ).grid(
            row=button_row + 1, column=0, columnspan=3,
            sticky="w", padx=12, pady=(2, 12),
        )
        self.window.after(100, self.refresh)

    def apply(self, name: str) -> None:
        try:
            value = self.variables[name].get()
            if name in ("HEADING_SIGN", "TURN_SIGN") and value not in (-1, 1):
                raise ValueError(f"{name} 只能为 -1 或 1")
            sequence = self.app.link.send_parameter(name, value)
        except (ValueError, tk.TclError, OSError, serial.SerialException) as error:
            self.status.set(f"发送失败：{error}")
            return
        self.status.set(f"已发送 #{sequence}  {name}={value}，等待遥测确认")
        self.app._append_log(
            f"[PARAM TX] seq={sequence} {name}={value}", "tx"
        )

    def refresh_config(self, sample: dict[str, Any]) -> None:
        for name, key, _minimum, _maximum in self.PARAMS:
            if self.window.focus_get() is self.entries[name]:
                continue
            self.variables[name].set(int(sample[key]))

    def refresh(self) -> None:
        if not self.window.winfo_exists():
            return
        samples = list(self.app.telemetry_history)
        visible = samples[-500:]
        for plot in (self.line_plot, self.heading_plot, self.motor_plot):
            plot.redraw(visible)
        if samples:
            sample = samples[-1]
            state_id = int(sample["track_state"])
            state = (
                TRACK_STATES[state_id]
                if 0 <= state_id < len(TRACK_STATES)
                else f"STATE_{state_id}"
            )
            self.status.set(
                f"状态 {state}  灰度 {sample['line_bits']:07b}  "
                f"IMU延迟 {sample['imu_age_ms']} ms"
            )
            self.refresh_config(sample)
        self.window.after(100, self.refresh)

    def export_csv(self) -> None:
        samples = list(self.app.telemetry_history)
        if not samples:
            messagebox.showinfo("导出", "当前还没有遥测数据")
            return
        path = filedialog.asksaveasfilename(
            parent=self.window,
            title="保存循迹遥测",
            defaultextension=".csv",
            filetypes=(("CSV", "*.csv"),),
            initialfile=f"tmx_telemetry_{time.strftime('%Y%m%d_%H%M%S')}.csv",
        )
        if not path:
            return
        with open(path, "w", newline="", encoding="utf-8-sig") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(samples[0].keys()))
            writer.writeheader()
            writer.writerows(samples)
        self.status.set(f"已导出 {len(samples)} 帧：{path}")

    def close(self) -> None:
        self.app.tuning_window = None
        self.window.destroy()


class CarControllerApp:
    REPEAT_MS = 200
    TELEMETRY_TIMEOUT_S = 0.8

    BG = "#edf3f8"
    PANEL = "#ffffff"
    PANEL_ALT = "#f4f8fc"
    EDGE = "#cbd9e6"
    TEXT = "#11243d"
    MUTED = "#6f8298"
    CYAN = "#00a6c7"
    MAGENTA = "#f02d8c"
    ORANGE = "#ef7b22"
    LIME = "#63a915"
    DANGER = "#e33b5f"
    IMU_FAULT_NAMES = {
        0: "NO STATUS",
        1: "I2C NOT READY",
        2: "I2C NACK",
        3: "I2C TIMEOUT",
        4: "DATA STALE",
        5: "I2C I/O",
    }

    def __init__(self, root: tk.Tk, port: str, baud: int) -> None:
        self.root = root
        self.link = EspSerialLink(port, baud)
        self.current_command = "STOP"
        self.last_radio_rx = 0.0
        self.last_telemetry_rx = 0.0
        self.last_imu_valid: bool | None = None
        self.last_imu_fault: int | None = None
        self.tracking_enabled = False
        self.movement_buttons: list[tk.Button] = []
        self.telemetry_history: deque[dict[str, Any]] = deque(maxlen=3000)
        self.tuning_window: TuningWindow | None = None

        root.title("APEX // ESP-NOW VEHICLE CONTROL")
        screen_width = root.winfo_screenwidth()
        screen_height = root.winfo_screenheight()
        window_width = min(1380, screen_width - 80)
        window_height = min(860, screen_height - 120)
        window_x = max(0, (screen_width - window_width) // 2)
        window_y = max(0, (screen_height - window_height) // 2)
        root.geometry(
            f"{window_width}x{window_height}+{window_x}+{window_y}"
        )
        root.minsize(min(1180, window_width), min(740, window_height))
        root.configure(bg=self.BG)
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.bind("<FocusOut>", self._focus_out)

        self.port_var = tk.StringVar(value=port)
        self.status_var = tk.StringVar(value="BASE OFFLINE")
        self.radio_var = tk.StringVar(value="RADIO STANDBY")
        self.command_var = tk.StringVar(value="STOP")
        self.vector_var = tk.StringVar(value="VEHICLE SAFE")
        self.ack_var = tk.StringVar(value="NO ACK")
        self.gyro_x_var = tk.StringVar(value="--°/s")
        self.gyro_y_var = tk.StringVar(value="--°/s")
        self.gyro_z_var = tk.StringVar(value="--°/s")
        self.roll_var = tk.StringVar(value="--°")
        self.pitch_var = tk.StringVar(value="--°")
        self.yaw_var = tk.StringVar(value="--°")
        self.imu_status_var = tk.StringVar(value="IMU OFFLINE")
        self.heading_var = tk.StringVar(value="FREE")
        self.tracking_var = tk.StringVar(value="开启循迹  //  OFF")
        self.speed_var = tk.IntVar(value=40)

        self._build_ui()
        self._bind_keys()
        self._set_connected_visual(False)
        self.root.after(100, self._poll_logs)
        self.root.after(self.REPEAT_MS, self._repeat_command)
        self.root.after(400, self._refresh_radio_state)
        self.root.after(250, self._refresh_telemetry_state)

    def _build_ui(self) -> None:
        tk.Frame(self.root, bg=self.MAGENTA, height=4).pack(fill="x")

        header = tk.Frame(self.root, bg=self.BG, height=88)
        header.pack(fill="x", padx=28, pady=(16, 8))
        header.pack_propagate(False)

        title_group = tk.Frame(header, bg=self.BG, width=360)
        title_group.pack(side="left", fill="y")
        title_group.pack_propagate(False)
        tk.Label(
            title_group,
            text="APEX",
            bg=self.BG,
            fg=self.TEXT,
            font=("Bahnschrift SemiBold", 28),
        ).pack(side="left", anchor="s")
        tk.Label(
            title_group,
            text="// LINK",
            bg=self.BG,
            fg=self.MAGENTA,
            font=("Bahnschrift SemiBold", 28),
        ).pack(side="left", anchor="s", padx=(7, 0))
        tk.Label(
            title_group,
            text="ESP-NOW  /  VEHICLE CONTROL SYSTEM",
            bg=self.BG,
            fg=self.MUTED,
            font=("Bahnschrift", 10),
        ).place(x=2, y=58)

        connection = tk.Frame(
            header,
            bg=self.PANEL,
            highlightbackground=self.EDGE,
            highlightthickness=1,
        )
        connection.pack(side="right", pady=8)

        self.base_dot = tk.Canvas(
            connection, width=18, height=18, bg=self.PANEL, highlightthickness=0
        )
        self.base_dot.pack(side="left", padx=(14, 4))
        self.base_dot_item = self.base_dot.create_oval(
            4, 4, 14, 14, fill=self.MUTED, outline=""
        )
        tk.Label(
            connection,
            textvariable=self.status_var,
            bg=self.PANEL,
            fg=self.TEXT,
            font=("Bahnschrift SemiBold", 10),
        ).pack(side="left", padx=(0, 12))

        self.radio_dot = tk.Canvas(
            connection, width=18, height=18, bg=self.PANEL, highlightthickness=0
        )
        self.radio_dot.pack(side="left", padx=(8, 4))
        self.radio_dot_item = self.radio_dot.create_oval(
            4, 4, 14, 14, fill=self.MUTED, outline=""
        )
        tk.Label(
            connection,
            textvariable=self.radio_var,
            bg=self.PANEL,
            fg=self.TEXT,
            font=("Bahnschrift SemiBold", 10),
        ).pack(side="left", padx=(0, 14))

        self.port_entry = tk.Entry(
            connection,
            textvariable=self.port_var,
            width=8,
            bg=self.PANEL_ALT,
            fg=self.CYAN,
            insertbackground=self.CYAN,
            selectbackground=self.MAGENTA,
            relief="flat",
            justify="center",
            font=("Bahnschrift SemiBold", 11),
        )
        self.port_entry.pack(side="left", ipady=8, padx=(0, 8))
        self.connect_button = tk.Button(
            connection,
            text="CONNECT",
            command=self.toggle_connection,
            bg=self.CYAN,
            fg="#ffffff",
            activebackground="#00bddc",
            activeforeground="#ffffff",
            relief="flat",
            bd=0,
            cursor="hand2",
            font=("Bahnschrift SemiBold", 10),
            padx=18,
            pady=9,
        )
        self.connect_button.pack(side="left", padx=(0, 10))

        body = tk.Frame(self.root, bg=self.BG)
        body.pack(fill="both", expand=True, padx=28, pady=(0, 12))
        body.grid_columnconfigure(0, weight=3)
        body.grid_columnconfigure(1, weight=2)
        body.grid_rowconfigure(0, weight=1)

        drive_panel = self._make_panel(body)
        drive_panel.grid(row=0, column=0, sticky="nsew", padx=(0, 8))
        power_panel = self._make_panel(body)
        power_panel.grid(row=0, column=1, sticky="nsew", padx=(8, 0))

        self._build_drive_panel(drive_panel)
        self._build_power_panel(power_panel)

        self._build_imu_panel()
        self._build_log_panel()

    def _make_panel(self, parent: tk.Widget) -> tk.Frame:
        return tk.Frame(
            parent,
            bg=self.PANEL,
            highlightbackground=self.EDGE,
            highlightthickness=1,
        )

    def _section_header(
        self, parent: tk.Widget, index: str, title: str, accent: str
    ) -> None:
        row = tk.Frame(parent, bg=self.PANEL)
        row.pack(fill="x", padx=22, pady=(18, 4))
        tk.Label(
            row,
            text=index,
            bg=accent,
            fg=self.BG,
            font=("Bahnschrift SemiBold", 9),
            padx=7,
            pady=2,
        ).pack(side="left")
        tk.Label(
            row,
            text=title,
            bg=self.PANEL,
            fg=self.TEXT,
            font=("Bahnschrift SemiBold", 13),
        ).pack(side="left", padx=9)
        tk.Frame(row, bg=accent, height=1).pack(
            side="left", fill="x", expand=True, padx=(10, 0)
        )

    def _build_drive_panel(self, panel: tk.Frame) -> None:
        self._section_header(panel, "01", "DRIVE VECTOR", self.MAGENTA)

        state = tk.Frame(panel, bg=self.PANEL)
        state.pack(fill="x", padx=24, pady=(5, 8))
        tk.Label(
            state,
            textvariable=self.command_var,
            bg=self.PANEL,
            fg=self.TEXT,
            font=("Bahnschrift SemiBold", 32),
        ).pack(side="left")
        tk.Label(
            state,
            textvariable=self.vector_var,
            bg=self.PANEL,
            fg=self.MUTED,
            font=("Bahnschrift", 10),
        ).pack(side="left", padx=14, pady=(12, 0))
        self.tracking_button = tk.Button(
            state,
            textvariable=self.tracking_var,
            command=self.toggle_tracking,
            bg=self.PANEL_ALT,
            fg=self.ORANGE,
            activebackground=self.ORANGE,
            activeforeground="#ffffff",
            disabledforeground="#aab8c7",
            relief="flat",
            bd=0,
            cursor="hand2",
            font=("Microsoft YaHei UI", 10, "bold"),
            padx=18,
            pady=10,
        )
        self.tracking_button.pack(side="right")
        self.movement_buttons.append(self.tracking_button)

        pad = tk.Frame(panel, bg=self.PANEL)
        pad.pack(pady=(2, 8))
        forward = self._drive_button(pad, "▲\n前进  W", self.CYAN)
        left = self._drive_button(pad, "◀\n左转  A", self.CYAN)
        center = self._drive_button(pad, "■\n制动", self.DANGER)
        right = self._drive_button(pad, "▶\n右转  D", self.CYAN)
        backward = self._drive_button(pad, "▼\n后退  S", self.CYAN)

        forward.grid(row=0, column=1, padx=6, pady=6)
        left.grid(row=1, column=0, padx=6, pady=6)
        center.grid(row=1, column=1, padx=6, pady=6)
        right.grid(row=1, column=2, padx=6, pady=6)
        backward.grid(row=2, column=1, padx=6, pady=6)

        self._bind_button(forward, "FORWARD")
        self._bind_button(left, "LEFT")
        self._bind_button(right, "RIGHT")
        self._bind_button(backward, "BACKWARD")
        center.configure(command=self.stop)

        emergency = tk.Button(
            panel,
            text="EMERGENCY STOP   //   SPACE",
            command=self.stop,
            bg=self.DANGER,
            fg="#ffffff",
            activebackground="#f05a78",
            activeforeground="#ffffff",
            disabledforeground="#f2b9c5",
            relief="flat",
            bd=0,
            cursor="hand2",
            font=("Bahnschrift SemiBold", 12),
            pady=10,
        )
        emergency.pack(fill="x", padx=24, pady=(8, 18))
        self.movement_buttons.append(emergency)

    def _drive_button(
        self, parent: tk.Widget, text: str, accent: str
    ) -> tk.Button:
        button = tk.Button(
            parent,
            text=text,
            width=11,
            height=3,
            bg=self.PANEL_ALT,
            fg=accent,
            activebackground=accent,
            activeforeground=self.BG,
            disabledforeground="#aab8c7",
            relief="flat",
            bd=0,
            cursor="hand2",
            font=("Microsoft YaHei UI", 11, "bold"),
        )
        self.movement_buttons.append(button)
        return button

    def _build_power_panel(self, panel: tk.Frame) -> None:
        self._section_header(panel, "02", "POWER OUTPUT", self.CYAN)
        tk.Button(
            panel,
            text="实时曲线 / 参数调节",
            command=self.open_tuning_window,
            bg=self.ORANGE,
            fg="#ffffff",
            activebackground="#ff9348",
            activeforeground="#ffffff",
            relief="flat",
            bd=0,
            cursor="hand2",
            font=("Microsoft YaHei UI", 9, "bold"),
            padx=14,
            pady=6,
        ).pack(anchor="e", padx=24, pady=(0, 2))
        self.gauge = SpeedGauge(panel, self.speed_var.get())
        self.gauge.pack(pady=(0, 0))

        scale_frame = tk.Frame(panel, bg=self.PANEL)
        scale_frame.pack(fill="x", padx=28)
        tk.Label(
            scale_frame,
            text="MIN",
            bg=self.PANEL_ALT,
            fg=self.MUTED,
            font=("Bahnschrift", 9),
        ).pack(side="left")
        tk.Label(
            scale_frame,
            text="MAX",
            bg=self.PANEL,
            fg=self.MUTED,
            font=("Bahnschrift", 9),
        ).pack(side="right")

        speed_controls = tk.Frame(panel, bg=self.PANEL)
        speed_controls.pack(fill="x", padx=24, pady=(0, 12))

        self.speed_minus_button = tk.Button(
            speed_controls,
            text="−",
            command=lambda: self._nudge_speed(-5),
            bg=self.PANEL_ALT,
            fg=self.CYAN,
            activebackground=self.CYAN,
            activeforeground=self.BG,
            relief="flat",
            bd=0,
            cursor="hand2",
            font=("Bahnschrift SemiBold", 18),
            width=3,
        )
        self.speed_minus_button.pack(side="left", padx=(0, 10))

        self.speed_scale = tk.Scale(
            speed_controls,
            from_=0,
            to=100,
            orient="horizontal",
            variable=self.speed_var,
            command=self._speed_changed,
            showvalue=False,
            bg=self.PANEL,
            fg=self.TEXT,
            activebackground=self.MAGENTA,
            troughcolor="#b9e1e9",
            highlightthickness=0,
            bd=0,
            sliderrelief="raised",
            sliderlength=30,
            width=12,
            cursor="hand2",
        )
        self.speed_scale.pack(side="left", fill="x", expand=True)
        self.speed_scale.bind("<MouseWheel>", self._speed_mousewheel)

        self.speed_plus_button = tk.Button(
            speed_controls,
            text="+",
            command=lambda: self._nudge_speed(5),
            bg=self.PANEL_ALT,
            fg=self.MAGENTA,
            activebackground=self.MAGENTA,
            activeforeground=self.BG,
            relief="flat",
            bd=0,
            cursor="hand2",
            font=("Bahnschrift SemiBold", 18),
            width=3,
        )
        self.speed_plus_button.pack(side="left", padx=(10, 0))

        telemetry = tk.Frame(panel, bg=self.PANEL_ALT)
        telemetry.pack(fill="x", padx=24, pady=(4, 18))
        self._metric(telemetry, "HEADING", self.heading_var, self.ORANGE).pack(
            side="left", fill="both", expand=True, padx=(0, 1)
        )
        self._metric(telemetry, "LAST ACK", self.ack_var, self.LIME).pack(
            side="left", fill="both", expand=True, padx=(1, 0)
        )

    def _metric(
        self,
        parent: tk.Widget,
        label: str,
        value: str | tk.StringVar,
        accent: str,
    ) -> tk.Frame:
        frame = tk.Frame(parent, bg=self.PANEL_ALT)
        tk.Frame(frame, bg=accent, width=3).pack(side="left", fill="y")
        content = tk.Frame(frame, bg=self.PANEL_ALT)
        content.pack(fill="both", expand=True, padx=9, pady=8)
        tk.Label(
            content,
            text=label,
            bg=self.PANEL_ALT,
            fg=self.MUTED,
            font=("Bahnschrift", 8),
        ).pack(anchor="w")
        tk.Label(
            content,
            text=value if isinstance(value, str) else None,
            textvariable=value if isinstance(value, tk.StringVar) else None,
            bg=self.PANEL_ALT,
            fg=self.TEXT,
            font=("Bahnschrift SemiBold", 11),
        ).pack(anchor="w")
        return frame

    def _build_imu_panel(self) -> None:
        shell = tk.Frame(
            self.root,
            bg=self.PANEL,
            highlightbackground=self.EDGE,
            highlightthickness=1,
        )
        shell.pack(fill="x", padx=28, pady=(0, 12))

        bar = tk.Frame(shell, bg=self.PANEL_ALT)
        bar.pack(fill="x")
        tk.Label(
            bar,
            text="03  //  IMU LIVE DATA",
            bg=self.PANEL_ALT,
            fg=self.MAGENTA,
            font=("Bahnschrift SemiBold", 9),
        ).pack(side="left", padx=12, pady=7)
        self.imu_status_label = tk.Label(
            bar,
            textvariable=self.imu_status_var,
            bg=self.PANEL_ALT,
            fg=self.MUTED,
            font=("Bahnschrift SemiBold", 9),
        )
        self.imu_status_label.pack(side="right", padx=12)

        metrics = tk.Frame(shell, bg=self.PANEL_ALT)
        metrics.pack(fill="x", padx=12, pady=(0, 10))
        imu_metrics = (
            ("GYRO X", self.gyro_x_var, self.CYAN),
            ("GYRO Y", self.gyro_y_var, self.CYAN),
            ("GYRO Z", self.gyro_z_var, self.CYAN),
            ("ROLL", self.roll_var, self.MAGENTA),
            ("PITCH", self.pitch_var, self.MAGENTA),
            ("YAW", self.yaw_var, self.MAGENTA),
        )
        for index, (label, value, accent) in enumerate(imu_metrics):
            metric = self._metric(metrics, label, value, accent)
            metric.pack(
                side="left",
                fill="both",
                expand=True,
                padx=(0 if index == 0 else 1, 0 if index == 5 else 1),
            )

    def _build_log_panel(self) -> None:
        shell = tk.Frame(self.root, bg=self.BG)
        shell.pack(fill="both", padx=28, pady=(0, 20))
        bar = tk.Frame(shell, bg=self.PANEL_ALT, height=30)
        bar.pack(fill="x")
        tk.Label(
            bar,
            text="04  //  LINK LOG",
            bg=self.PANEL_ALT,
            fg=self.CYAN,
            font=("Bahnschrift SemiBold", 9),
        ).pack(side="left", padx=12, pady=7)
        tk.Label(
            bar,
            text="ESP-NOW  CH 06     UART  115200 8N1",
            bg=self.PANEL_ALT,
            fg=self.MUTED,
            font=("Bahnschrift", 9),
        ).pack(side="right", padx=12)

        self.log = tk.Text(
            shell,
            height=4,
            bg="#f9fbfd",
            fg=self.MUTED,
            insertbackground=self.CYAN,
            selectbackground=self.MAGENTA,
            relief="flat",
            bd=0,
            padx=12,
            pady=8,
            state="disabled",
            wrap="none",
            font=("Cascadia Mono", 9),
        )
        self.log.pack(fill="both")
        self.log.tag_configure("tx", foreground=self.CYAN)
        self.log.tag_configure("ack", foreground=self.LIME)
        self.log.tag_configure("warn", foreground=self.ORANGE)
        self.log.tag_configure("error", foreground=self.DANGER)
        self.log.tag_configure("system", foreground=self.MAGENTA)

    def _bind_button(self, button: tk.Button, command: str) -> None:
        button.bind("<ButtonPress-1>", lambda _event: self.move(command))
        button.bind(
            "<ButtonRelease-1>",
            lambda _event: self.stop_if_current(command),
        )

    def open_tuning_window(self) -> None:
        if self.tuning_window is not None:
            self.tuning_window.window.lift()
            self.tuning_window.window.focus_force()
            return
        self.tuning_window = TuningWindow(self)

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
            self._set_connected_visual(False)
            return

        self.link.port = self.port_var.get().strip()
        try:
            self.link.connect()
        except (OSError, serial.SerialException) as error:
            messagebox.showerror("串口连接失败", str(error))
            return

        self._set_connected_visual(True)
        self._append_log(
            "[SYSTEM] Base online. Hold a direction key; release to stop.",
            "system",
        )
        sequence = self.link.send_command("STOP", 0)
        self._append_log(f"[PC TX] seq={sequence} STOP speed=0", "tx")

    def _set_connected_visual(self, connected: bool) -> None:
        if connected:
            self.status_var.set(f"BASE {self.link.port}")
            self.base_dot.itemconfigure(self.base_dot_item, fill=self.LIME)
            self._set_imu_status("IMU WAITING", self.ORANGE)
            self._set_tracking_visual(False)
            self.connect_button.configure(
                text="DISCONNECT",
                bg=self.MAGENTA,
                activebackground="#f65aa8",
            )
            self.port_entry.configure(state="disabled")
        else:
            self.status_var.set("BASE OFFLINE")
            self.radio_var.set("RADIO STANDBY")
            self.base_dot.itemconfigure(self.base_dot_item, fill=self.MUTED)
            self.radio_dot.itemconfigure(self.radio_dot_item, fill=self.MUTED)
            self.connect_button.configure(
                text="CONNECT",
                bg=self.CYAN,
                activebackground="#00bddc",
            )
            self.port_entry.configure(state="normal")
            self.current_command = "STOP"
            self.command_var.set("STOP")
            self.vector_var.set("VEHICLE SAFE")
            self.ack_var.set("NO ACK")
            self.heading_var.set("FREE")
            self.last_telemetry_rx = 0.0
            self.last_imu_valid = None
            self.last_imu_fault = None
            self.tracking_enabled = False
            self._clear_imu_values()
            self._set_imu_status("IMU OFFLINE", self.MUTED)
            self._set_tracking_visual(False)

        for button in self.movement_buttons:
            button.configure(state="normal" if connected else "disabled")

    def move(self, command: str) -> None:
        if not self.link.connected or command == self.current_command:
            return
        if self.tracking_enabled:
            self._send_tracking(False)
        self.current_command = command
        self.command_var.set(command)
        self.vector_var.set(f"VECTOR LOCKED  /  {self.speed_var.get():02d}%")
        self._send_current()

    def stop_if_current(self, command: str) -> None:
        if self.current_command == command:
            self.stop()

    def stop(self) -> None:
        if self.tracking_enabled and self.link.connected:
            self._send_tracking(False)
        was_moving = self.current_command != "STOP"
        self.current_command = "STOP"
        self.command_var.set("STOP")
        self.vector_var.set("VEHICLE SAFE")
        if was_moving:
            self._send_current()

    def toggle_tracking(self) -> None:
        if not self.link.connected:
            return
        enable = not self.tracking_enabled
        if enable and self.current_command != "STOP":
            self.stop()
        self._send_tracking(enable)

    def _send_tracking(self, enabled: bool) -> None:
        try:
            command = "TRACK_ON" if enabled else "TRACK_OFF"
            sequence = self.link.send_command(command, 0)
            self.tracking_enabled = enabled
            self._set_tracking_visual(enabled)
            if enabled:
                self.command_var.set("TRACK")
                self.vector_var.set("LINE FOLLOWING ACTIVE")
            else:
                self.command_var.set("STOP")
                self.vector_var.set("VEHICLE SAFE")
            self._append_log(
                f"[PC TX] seq={sequence:05d}  {command}", "tx"
            )
        except (OSError, serial.SerialException) as error:
            self._append_log(f"[SERIAL ERROR] {error}", "error")
            self.link.disconnect()
            self._set_connected_visual(False)

    def _set_tracking_visual(self, enabled: bool) -> None:
        self.tracking_enabled = enabled
        if enabled:
            self.tracking_var.set("循迹开启  //  点击关闭")
            self.tracking_button.configure(
                bg=self.LIME,
                fg="#ffffff",
                activebackground="#7fbe38",
                activeforeground="#ffffff",
            )
        else:
            self.tracking_var.set("循迹关闭  //  点击开启")
            self.tracking_button.configure(
                bg=self.PANEL_ALT,
                fg=self.ORANGE,
                activebackground=self.ORANGE,
                activeforeground="#ffffff",
            )

    def _speed_changed(self, value: str) -> None:
        speed = int(float(value))
        self.gauge.set_value(speed)
        if self.current_command != "STOP":
            self.vector_var.set(f"VECTOR LOCKED  /  {speed:02d}%")
            self._send_current()

    def _nudge_speed(self, delta: int) -> None:
        speed = max(0, min(100, self.speed_var.get() + delta))
        self.speed_var.set(speed)
        self._speed_changed(str(speed))

    def _speed_mousewheel(self, event: tk.Event) -> str:
        self._nudge_speed(5 if event.delta > 0 else -5)
        return "break"

    def _send_current(self) -> None:
        if not self.link.connected:
            return
        try:
            speed = (
                0 if self.current_command == "STOP" else self.speed_var.get()
            )
            sequence = self.link.send_command(self.current_command, speed)
            self._append_log(
                f"[PC TX] seq={sequence:05d}  "
                f"{self.current_command:<8}  power={speed:03d}%",
                "tx",
            )
        except (OSError, serial.SerialException) as error:
            self._append_log(f"[SERIAL ERROR] {error}", "error")
            self.link.disconnect()
            self._set_connected_visual(False)

    def _repeat_command(self) -> None:
        if self.current_command != "STOP":
            self._send_current()
        self.root.after(self.REPEAT_MS, self._repeat_command)

    def _poll_logs(self) -> None:
        while True:
            try:
                item = self.link.lines.get_nowait()
            except queue.Empty:
                break

            if isinstance(item, dict):
                self._update_telemetry(item)
                continue
            line = str(item)
            if "RX heartbeat" in line:
                self.last_radio_rx = time.monotonic()
            if line.startswith("ESP,ACK,"):
                parts = line.split(",")
                if len(parts) >= 3:
                    self.ack_var.set(f"#{parts[2]}")
                tag = "ack"
            elif "ERROR" in line or "<err>" in line or "NACK" in line:
                tag = "error"
            elif "<wrn>" in line or "WARN" in line:
                tag = "warn"
            else:
                tag = ""
            self._append_log(line, tag)
        self.root.after(100, self._poll_logs)

    def _update_telemetry(self, sample: dict[str, Any]) -> None:
        """刷新仪表并保存一帧可绘图、可导出的完整遥测。"""
        gyro_x = int(sample["gyro_x"]) / 10.0
        gyro_y = int(sample["gyro_y"]) / 10.0
        gyro_z = int(sample["gyro_z"]) / 10.0
        roll = int(sample["roll"]) / 100.0
        pitch = int(sample["pitch"]) / 100.0
        yaw = int(sample["yaw"]) / 100.0
        flags = int(sample["flags"])
        imu_fault = (flags >> 2) & 0x3F
        target = int(sample["heading_target"]) / 100.0
        error = int(sample["heading_error"]) / 100.0
        correction = int(sample["heading_correction"])
        tracking_enabled = int(sample["tracking_enabled"])
        if tracking_enabled not in (0, 1):
            return

        sample["pc_time_s"] = time.time()
        self.telemetry_history.append(sample)
        now = time.monotonic()
        self.last_radio_rx = now
        self.last_telemetry_rx = now

        if flags & 0x01:
            self.gyro_x_var.set(f"{gyro_x:+.1f}°/s")
            self.gyro_y_var.set(f"{gyro_y:+.1f}°/s")
            self.gyro_z_var.set(f"{gyro_z:+.1f}°/s")
            self.roll_var.set(f"{roll:+.2f}°")
            self.pitch_var.set(f"{pitch:+.2f}°")
            self.yaw_var.set(f"{yaw:+.2f}°")
            self._set_imu_status("IMU LIVE", self.LIME)
            if self.last_imu_valid is False:
                self._append_log("CAR IMU RECOVERED", "ack")
            self.last_imu_valid = True
            self.last_imu_fault = 0
        else:
            fault_name = self.IMU_FAULT_NAMES.get(
                imu_fault, f"UNKNOWN {imu_fault}"
            )
            self._clear_imu_values()
            self._set_imu_status(f"IMU ERROR  //  {fault_name}", self.DANGER)
            if (
                self.last_imu_valid is not False
                or imu_fault != self.last_imu_fault
            ):
                self._append_log(
                    f"CAR IMU ERROR: {fault_name} ({imu_fault})", "error"
                )
            self.last_imu_valid = False
            self.last_imu_fault = imu_fault

        if flags & 0x02:
            self.heading_var.set(
                f"LOCK {target:+.2f}°  E{error:+.2f}°  C{correction:+d}"
            )
        else:
            self.heading_var.set("FREE")

        self._set_tracking_visual(bool(tracking_enabled))
        state_id = int(sample["track_state"])
        state = (
            TRACK_STATES[state_id]
            if 0 <= state_id < len(TRACK_STATES)
            else f"STATE_{state_id}"
        )
        if tracking_enabled:
            self.command_var.set("TRACK")
            self.vector_var.set(
                f"{state} / LINE {int(sample['line_bits']):07b}"
            )
        elif self.command_var.get() == "TRACK":
            self.command_var.set("STOP")
            self.vector_var.set("VEHICLE SAFE")

    def _clear_imu_values(self) -> None:
        self.gyro_x_var.set("--°/s")
        self.gyro_y_var.set("--°/s")
        self.gyro_z_var.set("--°/s")
        self.roll_var.set("--°")
        self.pitch_var.set("--°")
        self.yaw_var.set("--°")

    def _set_imu_status(self, text: str, color: str) -> None:
        self.imu_status_var.set(text)
        self.imu_status_label.configure(fg=color)

    def _refresh_telemetry_state(self) -> None:
        if not self.link.connected:
            self._set_imu_status("IMU OFFLINE", self.MUTED)
        elif self.last_telemetry_rx == 0.0:
            self._set_imu_status("IMU WAITING", self.ORANGE)
        elif (
            time.monotonic() - self.last_telemetry_rx
            > self.TELEMETRY_TIMEOUT_S
        ):
            if self.imu_status_var.get() != "IMU STALE":
                self._clear_imu_values()
                self.heading_var.set("FREE")
                self._set_imu_status("IMU STALE", self.DANGER)
                self._append_log(
                    "[TELEMETRY] No fresh IMU data for 800 ms", "error"
                )
        self.root.after(250, self._refresh_telemetry_state)

    def _refresh_radio_state(self) -> None:
        active = (
            self.link.connected
            and time.monotonic() - self.last_radio_rx < 3.2
        )
        self.radio_var.set("RADIO ACTIVE" if active else "RADIO STANDBY")
        self.radio_dot.itemconfigure(
            self.radio_dot_item,
            fill=self.CYAN if active else self.MUTED,
        )
        self.root.after(400, self._refresh_radio_state)

    def _focus_out(self, _event: tk.Event) -> None:
        self.root.after_idle(self._stop_if_window_unfocused)

    def _stop_if_window_unfocused(self) -> None:
        if self.root.focus_displayof() is None:
            self.stop()

    def _append_log(self, line: str, tag: str = "") -> None:
        self.log.configure(state="normal")
        self.log.insert("end", line + "\n", tag)
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
        choices=(
            "forward",
            "backward",
            "left",
            "right",
            "stop",
            "track_on",
            "track_off",
        ),
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
