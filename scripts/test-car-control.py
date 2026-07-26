#!/usr/bin/env python3
"""End-to-end PC UART -> ESP-NOW -> car UART integration test."""

import argparse
import re
import threading
import time

import serial


ANSI_ESCAPE = re.compile(r"\x1b\[[0-9;]*m")


def read_port(connection, output, stop_event):
    while not stop_event.is_set():
        raw = connection.readline()
        if raw:
            text = raw.decode("utf-8", errors="replace").strip()
            text = ANSI_ESCAPE.sub("", text)
            if text:
                output.append(text)


def open_port(name):
    connection = serial.Serial()
    connection.port = name
    connection.baudrate = 115200
    connection.timeout = 0.05
    connection.dtr = False
    connection.rts = False
    connection.open()
    connection.reset_input_buffer()
    return connection


def reset_pair(connections):
    for connection in connections:
        connection.rts = True
    time.sleep(0.15)
    for connection in connections:
        connection.rts = False


def send_command(connection, sequence, command, speed):
    line = f"CAR,{sequence},{command},{speed}\n"
    print(f"PC -> base ESP: {line.strip()}")
    connection.write(line.encode("ascii"))
    connection.flush()


def require(log, text, label):
    if not any(text in line for line in log):
        raise RuntimeError(f"{label} missing expected log: {text}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-port", default="COM17")
    parser.add_argument("--car-port", default="COM19")
    args = parser.parse_args()

    base = open_port(args.base_port)
    car = open_port(args.car_port)
    outputs = {args.base_port: [], args.car_port: []}
    stop_event = threading.Event()
    threads = [
        threading.Thread(
            target=read_port,
            args=(base, outputs[args.base_port], stop_event),
            daemon=True,
        ),
        threading.Thread(
            target=read_port,
            args=(car, outputs[args.car_port], stop_event),
            daemon=True,
        ),
    ]

    try:
        for thread in threads:
            thread.start()
        reset_pair((base, car))
        time.sleep(1.5)

        # A single movement command must eventually trigger the car-side
        # failsafe because it is not refreshed within 700 ms.
        send_command(base, 10, "FORWARD", 35)
        time.sleep(1.1)

        # A normally refreshed steering command remains active, then STOP is
        # sent explicitly as it would be on key/button release.
        for sequence in range(11, 14):
            send_command(base, sequence, "LEFT", 25)
            time.sleep(0.2)
        send_command(base, 14, "STOP", 0)
        time.sleep(1.0)
    finally:
        stop_event.set()
        for thread in threads:
            thread.join(timeout=0.3)
        base.close()
        car.close()

    for port, lines in outputs.items():
        print(f"\n===== {port} relevant log =====")
        for line in lines:
            if any(
                marker in line
                for marker in (
                    "PC control",
                    "PC command",
                    "Control TX",
                    "Control RX",
                    "TI command",
                    "failsafe",
                    "Car UART",
                    "ESP,ACK",
                )
            ):
                print(line)

    base_log = outputs[args.base_port]
    car_log = outputs[args.car_port]
    require(base_log, "PC command seq=10 FORWARD speed=35", "base ESP")
    require(base_log, "Control TX seq=14 STOP speed=0", "base ESP")
    require(car_log, "Control RX seq=10 FORWARD speed=35", "car ESP")
    require(car_log, "TI command seq=10 FORWARD speed=35", "car ESP")
    require(car_log, "Control timeout: failsafe STOP", "car ESP")
    require(car_log, "TI command seq=14 STOP speed=0", "car ESP")
    print("\nPASS: PC UART -> ESP-NOW -> car UART queue and failsafe")


if __name__ == "__main__":
    main()
