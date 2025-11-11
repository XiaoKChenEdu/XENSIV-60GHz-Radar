#!/usr/bin/env python3
"""Decode captured radar binary frames into human-readable text or CSV."""

from __future__ import annotations

import argparse
import csv
import struct
import sys
from pathlib import Path
from typing import Any, Iterable, Optional, Sequence

HEADER_STRUCT = struct.Struct("<4sHHII")
HEADER_MAGIC = b"RADR"
SUPPORTED_VERSION = 1
SUPPORTED_SAMPLE_SIZES = {1: "B", 2: "H", 4: "I"}


class FrameDecodeError(RuntimeError):
    """Raised when a captured log cannot be decoded."""


def _read_exact(stream, size: int) -> bytes:
    data = stream.read(size)
    if len(data) != size:
        raise FrameDecodeError("Unexpected end of file while reading frame data")
    return data


def _unpack_samples(payload: bytes, sample_size: int, sample_count: int, signed: bool) -> Sequence[int]:
    expected = sample_size * sample_count
    if len(payload) != expected:
        raise FrameDecodeError(
            f"Payload size mismatch: expected {expected} bytes, got {len(payload)}"
        )

    if sample_size not in SUPPORTED_SAMPLE_SIZES:
        raise FrameDecodeError(f"Unsupported sample size: {sample_size} bytes")

    type_char = SUPPORTED_SAMPLE_SIZES[sample_size]
    if signed:
        type_char = type_char.lower()

    return struct.unpack(f"<{sample_count}{type_char}", payload)


def _write_frame_text(
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
        for idx, value in enumerate(samples):
            handle.write(f"  Sample {idx:05d}: {value}\n")
        handle.write("\n")
        return

    if sample_count % rx_antennas != 0:
        handle.write(
            "  !! Sample count not divisible by RX antenna count. Dumping linear data.\n"
        )
        for idx, value in enumerate(samples):
            handle.write(f"  Sample {idx:05d}: {value}\n")
        handle.write("\n")
        return

    per_antenna = sample_count // rx_antennas

    if per_antenna % samples_per_chirp != 0:
        handle.write("  !! Sample layout mismatch. Dumping linear data.\n")
        for idx, value in enumerate(samples):
            handle.write(f"  Sample {idx:05d}: {value}\n")
        handle.write("\n")
        return

    chirps = per_antenna // samples_per_chirp
    cursor = 0

    for chirp in range(chirps):
        handle.write(f"  Chirp {chirp:03d}:\n")
        for sample_idx in range(samples_per_chirp):
            row = samples[cursor : cursor + rx_antennas]
            cursor += rx_antennas
            joined = ", ".join(str(num) for num in row)
            handle.write(f"    Sample {sample_idx:03d}: [{joined}]\n")
        handle.write("\n")

    handle.write("\n")


def _maybe_write_csv(
    csv_writer: Optional[Any],
    frame_index: int,
    samples: Sequence[int],
    rx_antennas: int,
    samples_per_chirp: int,
) -> None:
    if csv_writer is None:
        return

    sample_count = len(samples)
    if rx_antennas <= 0 or samples_per_chirp <= 0:
        for idx, value in enumerate(samples):
            csv_writer.writerow((frame_index, idx, 0, 0, value))
        return

    if sample_count % rx_antennas != 0:
        for idx, value in enumerate(samples):
            csv_writer.writerow((frame_index, idx, 0, 0, value))
        return

    per_antenna = sample_count // rx_antennas
    if per_antenna % samples_per_chirp != 0:
        for idx, value in enumerate(samples):
            csv_writer.writerow((frame_index, idx, 0, 0, value))
        return

    chirps = per_antenna // samples_per_chirp
    cursor = 0

    for chirp in range(chirps):
        for sample_idx in range(samples_per_chirp):
            for rx in range(rx_antennas):
                csv_writer.writerow((frame_index, chirp, sample_idx, rx, samples[cursor + rx]))
            cursor += rx_antennas


def _iter_frames(stream) -> Iterable[tuple[int, int, int, int, bytes]]:
    while True:
        header_bytes = stream.read(HEADER_STRUCT.size)
        if not header_bytes:
            break
        if len(header_bytes) != HEADER_STRUCT.size:
            raise FrameDecodeError("Trailing bytes detected while reading header")

        magic, version, sample_size, frame_index, sample_count = HEADER_STRUCT.unpack(
            header_bytes
        )

        if magic != HEADER_MAGIC:
            raise FrameDecodeError("Bad header magic. Is this a valid capture?")
        if version != SUPPORTED_VERSION:
            raise FrameDecodeError(
                f"Unsupported header version {version}; expected {SUPPORTED_VERSION}"
            )

        payload = _read_exact(stream, sample_size * sample_count)
        yield frame_index, sample_size, sample_count, len(payload), payload


def decode_frames(
    input_path: Path,
    *,
    rx_antennas: int,
    samples_per_chirp: int,
    signed: bool,
    output_text: Optional[Path],
    output_csv: Optional[Path],
    limit: Optional[int],
) -> dict[str, int]:
    stats = {
        "frames": 0,
        "first_frame": None,
        "last_frame": None,
        "bytes": input_path.stat().st_size,
        "samples": 0,
    }

    text_handle = open(output_text, "w", encoding="utf-8") if output_text else None
    csv_handle = open(output_csv, "w", encoding="utf-8", newline="") if output_csv else None
    csv_writer = None

    try:
        if csv_handle:
            csv_writer = csv.writer(csv_handle)
            csv_writer.writerow(("frame", "chirp", "sample", "rx", "value"))

        with input_path.open("rb") as stream:
            for frame_idx, sample_size, sample_count, payload_size, payload in _iter_frames(stream):
                stats["frames"] += 1
                stats["samples"] += sample_count
                stats["first_frame"] = frame_idx if stats["first_frame"] is None else stats["first_frame"]
                stats["last_frame"] = frame_idx

                samples = _unpack_samples(payload, sample_size, sample_count, signed=signed)

                if text_handle:
                    _write_frame_text(
                        text_handle,
                        frame_index=frame_idx,
                        samples=samples,
                        rx_antennas=rx_antennas,
                        samples_per_chirp=samples_per_chirp,
                    )

                _maybe_write_csv(
                    csv_writer,
                    frame_idx,
                    samples,
                    rx_antennas,
                    samples_per_chirp,
                )

                if limit is not None and stats["frames"] >= limit:
                    break
    finally:
        if text_handle:
            text_handle.close()
        if csv_handle:
            csv_handle.close()

    return stats


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Decode binary radar frames captured by serial_logger.py",
    )
    parser.add_argument("--input", required=True, type=Path, help="Path to the binary capture file.")
    parser.add_argument(
        "--output-text", type=Path, help="Optional destination for a human-readable dump.",
    )
    parser.add_argument(
        "--output-csv", type=Path, help="Optional destination for a CSV dump (frame, chirp, sample, rx, value).",
    )
    parser.add_argument(
        "--rx-antennas",
        type=int,
        default=3,
        help="Number of RX antennas configured during capture (default: 3).",
    )
    parser.add_argument(
        "--samples-per-chirp",
        type=int,
        default=128,
        help="ADC samples per chirp (default: 128).",
    )
    parser.add_argument(
        "--signed",
        action="store_true",
        help="Interpret samples as signed integers instead of unsigned.",
    )
    parser.add_argument(
        "--limit",
        type=int,
        help="Only decode the first N frames (useful for quick inspection).",
    )
    parser.add_argument(
        "--summary-only",
        action="store_true",
        help="Just report frame statistics without writing decoded output.",
    )
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)

    if not args.input.exists():
        sys.stderr.write(f"Input file not found: {args.input}\n")
        return 1

    if args.summary_only and (args.output_text or args.output_csv):
        sys.stderr.write("--summary-only cannot be combined with output options.\n")
        return 1

    try:
        stats = decode_frames(
            args.input,
            rx_antennas=args.rx_antennas,
            samples_per_chirp=args.samples_per_chirp,
            signed=args.signed,
            output_text=None if args.summary_only else args.output_text,
            output_csv=None if args.summary_only else args.output_csv,
            limit=args.limit,
        )
    except FrameDecodeError as exc:
        sys.stderr.write(f"Decode failed: {exc}\n")
        return 1

    summary = [
        f"Frames decoded: {stats['frames']}",
        f"Samples total: {stats['samples']}",
        f"First frame index: {stats['first_frame']}",
        f"Last frame index: {stats['last_frame']}",
        f"Input bytes: {stats['bytes']}",
    ]

    if not args.summary_only and args.output_text is None and args.output_csv is None:
        for line in summary:
            print(line)
    else:
        sys.stderr.write("\n".join(summary) + "\n")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
