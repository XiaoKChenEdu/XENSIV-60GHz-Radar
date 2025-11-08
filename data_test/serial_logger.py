#!/usr/bin/env python3
"""Capture radar UART output to a file without echoing it to the terminal."""

import argparse
import sys
from typing import Optional

import serial


def _build_start_command(frames: Optional[int]) -> bytes:
    if frames is None:
        return b"start\r\n"

    return f"start {frames}\r\n".encode("ascii")


def _should_stop(buffer: bytearray) -> bool:
    return (b"Capture completed" in buffer) or (b"Capture stopped." in buffer)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Log serial output to disk without printing it."
    )
    parser.add_argument(
        "--port",
        required=True,
        help="Serial port connected to the board (e.g. COM8 or /dev/ttyUSB0).",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="UART baud rate. Must match CY_RETARGET_IO_BAUDRATE (default: 115200).",
    )
    parser.add_argument(
        "--output",
        required=True,
        help="Destination file for the raw UART capture.",
    )
    parser.add_argument(
        "--frames",
        type=int,
        help="Number of frames to request. Omit or set to 0 for continuous capture.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=0.1,
        help="Serial read timeout in seconds (default: 0.1).",
    )
    parser.add_argument(
        "--stop-on-exit",
        action="store_true",
        help="Send a 'stop' command when the script exits.",
    )
    args = parser.parse_args()

    if args.frames is not None and args.frames < 0:
        parser.error("--frames must be >= 0")

    frames_arg = None if args.frames in (None, 0) else args.frames
    start_command = _build_start_command(frames_arg)
    stop_command = b"stop\r\n"

    try:
        with serial.Serial(args.port, baudrate=args.baud, timeout=args.timeout) as ser, \
                open(args.output, "wb") as outfile:

            ser.reset_input_buffer()
            ser.reset_output_buffer()

            ser.write(start_command)
            ser.flush()

            sys.stderr.write(
                f"Started capture -> port={args.port}, baud={args.baud}, "
                f"frames={'continuous' if frames_arg is None else frames_arg}, "
                f"output='{args.output}'\n"
            )
            sys.stderr.flush()

            completion_watch = bytearray()

            try:
                while True:
                    data = ser.read(ser.in_waiting or 1)
                    if data:
                        outfile.write(data)
                        outfile.flush()

                        if frames_arg is not None:
                            completion_watch.extend(data)

                            if _should_stop(completion_watch):
                                break

                            if len(completion_watch) > 1024:
                                completion_watch = completion_watch[-512:]
                    else:
                        continue
            except KeyboardInterrupt:
                sys.stderr.write("Capture interrupted by user.\n")
            finally:
                if args.stop_on_exit:
                    try:
                        ser.write(stop_command)
                        ser.flush()
                    except serial.SerialException:
                        pass

    except serial.SerialException as exc:
        sys.stderr.write(f"Serial error: {exc}\n")
        return 1
    except OSError as exc:
        sys.stderr.write(f"I/O error: {exc}\n")
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())

