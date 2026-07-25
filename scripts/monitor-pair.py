#!/usr/bin/env python3
"""Reset and monitor two ESP32-C6 UART ports concurrently."""

import argparse
import threading
import time

import serial


def monitor(port, baud, duration, start_event, results, errors):
    try:
        with serial.Serial(port, baud, timeout=0.05) as connection:
            connection.dtr = False
            connection.reset_input_buffer()
            start_event.wait()

            # CH343 RTS is wired to EN on these boards.
            connection.rts = True
            time.sleep(0.15)
            connection.rts = False

            deadline = time.monotonic() + duration
            data = bytearray()
            while time.monotonic() < deadline:
                waiting = connection.in_waiting
                chunk = connection.read(waiting or 1)
                if chunk:
                    data.extend(chunk)
            results[port] = data.decode("utf-8", errors="replace")
    except Exception as error:  # Keep the other monitor alive for diagnostics.
        errors[port] = str(error)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("ports", nargs=2)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--duration", type=float, default=10.0)
    args = parser.parse_args()

    start_event = threading.Event()
    results = {}
    errors = {}
    threads = [
        threading.Thread(
            target=monitor,
            args=(port, args.baud, args.duration, start_event, results, errors),
            daemon=True,
        )
        for port in args.ports
    ]

    for thread in threads:
        thread.start()
    time.sleep(0.3)
    start_event.set()
    for thread in threads:
        thread.join()

    for port in args.ports:
        print(f"\n===== {port} =====")
        if port in errors:
            print(f"ERROR: {errors[port]}")
        else:
            print(results.get(port, "").rstrip())

    if errors:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
