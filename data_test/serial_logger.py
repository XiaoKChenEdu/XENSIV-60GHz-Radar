#!/usr/bin/env python3
"""Capture binary radar frames and optionally decode them to text."""

import argparse
import struct
import sys
import time
from contextlib import nullcontext
from typing import Optional, Sequence

import serial


HEADER_STRUCT = struct.Struct("<4sHHII")
HEADER_MAGIC = b"RADR"
HEADER_SIZE = HEADER_STRUCT.size
SUPPORTED_HEADER_VERSION = 1


def _build_start_command(frames: Optional[int]) -> bytes:
    if frames is None:
        return b"start\r\n"

    return f"start {frames}\r\n".encode("ascii")


def _align_stream(port: serial.Serial) -> tuple[str, bytearray]:
    """Read from the serial port until the binary header magic is found."""

    status_bytes = bytearray()
    buffer = bytearray()

    while True:
        chunk = port.read(1024)
        if not chunk:
            time.sleep(0.005)
            continue

        buffer.extend(chunk)
        magic_idx = buffer.find(HEADER_MAGIC)

        if magic_idx == -1:
            status_bytes.extend(buffer)
            buffer.clear()
            continue

        status_bytes.extend(buffer[:magic_idx])
        del buffer[:magic_idx]
        # buffer now starts with HEADER_MAGIC and may already contain more data
        return status_bytes.decode("ascii", errors="ignore"), buffer


def _fill_buffer(port: serial.Serial, buffer: bytearray, target_size: int) -> None:
    """Ensure buffer has at least target_size bytes by reading from the port."""

    while len(buffer) < target_size:
        chunk = port.read(target_size - len(buffer))

        if chunk:
            buffer.extend(chunk)
            continue

        time.sleep(0.005)


def _unpack_samples(payload: bytes, sample_count: int, sample_size: int) -> Sequence[int]:
    if len(payload) != sample_count * sample_size:
        raise RuntimeError("Payload size does not match header metadata.")

    if sample_size == 2:
        return struct.unpack(f"<{sample_count}H", payload)
    if sample_size == 1:
        return struct.unpack(f"<{sample_count}B", payload)
    if sample_size == 4:
        return struct.unpack(f"<{sample_count}I", payload)

    raise RuntimeError(f"Unsupported sample size: {sample_size} bytes")


def _write_formatted_frame(
    handle,
    *,
    frame_index: int,
    samples: Sequence[int],
    rx_antennas: int,
    samples_per_chirp: int,
) -> None:
    sample_count = len(samples)
    handle.write(f"Frame {frame_index} ({sample_count} samples)\n")

    if rx_antennas <= 0 or samples_per_chirp <= 0:
        # Fall back to a simple linear dump.
        for i, value in enumerate(samples):
            handle.write(f"  Sample {i:05d}: {value}\n")
        handle.write("\n")
        return

    if sample_count % rx_antennas != 0:
        handle.write("  !! Sample count not divisible by RX antenna count. Dumping linear data.\n")
        for i, value in enumerate(samples):
            handle.write(f"  Sample {i:05d}: {value}\n")
        handle.write("\n")
        return

    samples_per_frame = sample_count // rx_antennas

    if samples_per_frame % samples_per_chirp != 0:
        handle.write("  !! Sample layout mismatch. Dumping linear data.\n")
        for i, value in enumerate(samples):
            handle.write(f"  Sample {i:05d}: {value}\n")
        handle.write("\n")
        return

    chirps = samples_per_frame // samples_per_chirp
    idx = 0

    for chirp in range(chirps):
        handle.write(f"  Chirp {chirp:03d}:\n")

        for sample_idx in range(samples_per_chirp):
            row = samples[idx: idx + rx_antennas]
            idx += rx_antennas
            values = ", ".join(str(value) for value in row)
            handle.write(f"    Sample {sample_idx:03d}: [{values}]\n")

        handle.write("\n")

    handle.write("\n")


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
        "--formatted-output",
        help="Optional path for a human-readable dump of each frame.",
    )
    parser.add_argument(
        "--rx-antennas",
        type=int,
        default=3,
        help="Number of RX antennas used (default: 3).",
    )
    parser.add_argument(
        "--samples-per-chirp",
        type=int,
        default=128,
        help="Number of ADC samples per chirp (default: 128).",
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
                open(args.output, "wb") as outfile, \
                (open(args.formatted_output, "w", encoding="utf-8") if args.formatted_output else nullcontext()) as formatted:

            ser.reset_input_buffer()
            ser.reset_output_buffer()

            ser.write(start_command)
            ser.flush()

            status_text, buffer = _align_stream(ser)

            sys.stderr.write(
                f"Started capture -> port={args.port}, baud={args.baud}, "
                f"frames={'continuous' if frames_arg is None else frames_arg}, "
                f"output='{args.output}'\n"
            )

            if status_text:
                sys.stderr.write(status_text)
                if not status_text.endswith("\n"):
                    sys.stderr.write("\n")

            sys.stderr.flush()

            pending_frames = frames_arg
            frames_captured = 0

            pending_header = None

            if len(buffer) >= HEADER_SIZE:
                pending_header = bytes(buffer[:HEADER_SIZE])
                del buffer[:HEADER_SIZE]
            else:
                _fill_buffer(ser, buffer, HEADER_SIZE)
                pending_header = bytes(buffer[:HEADER_SIZE])
                del buffer[:HEADER_SIZE]

            try:
                while pending_frames is None or pending_frames > 0:
                    if pending_header is not None:
                        header_bytes = pending_header
                        pending_header = None
                    else:
                        _fill_buffer(ser, buffer, HEADER_SIZE)
                        header_bytes = bytes(buffer[:HEADER_SIZE])
                        del buffer[:HEADER_SIZE]

                    magic, version, sample_size, frame_index, sample_count = HEADER_STRUCT.unpack(header_bytes)

                    if magic != HEADER_MAGIC:
                        raise RuntimeError("Stream out of sync: header magic mismatch.")

                    if version != SUPPORTED_HEADER_VERSION:
                        raise RuntimeError(
                            f"Unsupported header version {version}; expected {SUPPORTED_HEADER_VERSION}."
                        )

                    payload_size = sample_count * sample_size
                    _fill_buffer(ser, buffer, payload_size)
                    payload = bytes(buffer[:payload_size])
                    del buffer[:payload_size]

                    outfile.write(header_bytes)
                    outfile.write(payload)
                    outfile.flush()

                    frames_captured += 1
                    if pending_frames is not None:
                        pending_frames -= 1

                    sys.stderr.write(
                        f"Captured frame {frame_index} (#{frames_captured} in session, {sample_count} samples).\n"
                    )
                    sys.stderr.flush()

                    if formatted:
                        samples = _unpack_samples(payload, sample_count, sample_size)
                        _write_formatted_frame(
                            formatted,
                            frame_index=frame_index,
                            samples=samples,
                            rx_antennas=args.rx_antennas,
                            samples_per_chirp=args.samples_per_chirp,
                        )
                        formatted.flush()

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

