"""Utilities for rewriting radar capture logs into a compact hierarchy."""

import argparse
import re
from collections import defaultdict


FRAME_HEADER_RE = re.compile(r'=+\s*Frame\s+(\d+)[^=]*=+')
OLD_FRAME_SPLIT_RE = re.compile(r'\[.*?\]\s*=+\s*Frame\s+(\d+)\s*=+')
OLD_RX_SPLIT_RE = re.compile(r'---\s*RX Antenna\s*(\d+)\s*---')
OLD_CHIRP_RE = re.compile(r'Chirp\s+(\d+):\s+([\d\s]+?)(?=Chirp|$)', re.DOTALL)
CHIRP_LINE_RE = re.compile(r'^Chirp\s+(\d+):')
SAMPLE_LINE_RE = re.compile(r'^Sample\s+(\d+):\s*\[([^\]]+)\]')


def parse_old_format(content):
    frames = {}
    parts = re.split(OLD_FRAME_SPLIT_RE, content)
    for idx in range(1, len(parts), 2):
        frame_num = int(parts[idx])
        frame_body = parts[idx + 1] if idx + 1 < len(parts) else ""
        frame_entry = {1: {}, 2: {}, 3: {}}
        rx_sections = re.split(OLD_RX_SPLIT_RE, frame_body)
        for rx_idx in range(1, len(rx_sections), 2):
            rx_num = int(rx_sections[rx_idx])
            rx_body = rx_sections[rx_idx + 1] if rx_idx + 1 < len(rx_sections) else ""
            for match in re.finditer(OLD_CHIRP_RE, rx_body):
                chirp_num = int(match.group(1))
                values = match.group(2).strip().split()
                frame_entry.setdefault(rx_num, {})[chirp_num] = values
        frames[frame_num] = frame_entry
    return frames


def parse_new_format(content):
    frames = defaultdict(lambda: {1: defaultdict(list), 2: defaultdict(list), 3: defaultdict(list)})
    current_frame = None
    current_chirp = None

    for raw_line in content.splitlines():
        line = raw_line.strip()
        if not line:
            continue
        if ']' in line:
            line = line.split(']', 1)[1].strip()
            if not line:
                continue

        frame_match = FRAME_HEADER_RE.match(line)
        if frame_match:
            current_frame = int(frame_match.group(1))
            current_chirp = None
            continue

        if current_frame is None:
            continue

        chirp_match = CHIRP_LINE_RE.match(line)
        if chirp_match:
            current_chirp = int(chirp_match.group(1))
            # ensure containers exist for this chirp
            frame_data = frames[current_frame]
            for rx in (1, 2, 3):
                frame_data[rx][current_chirp] = frame_data[rx][current_chirp]
            continue

        sample_match = SAMPLE_LINE_RE.match(line)
        if sample_match and current_chirp is not None:
            values_segment = sample_match.group(2)
            values = [value.strip() for value in values_segment.split(',') if value.strip()]
            if len(values) == 3:
                frame_data = frames[current_frame]
                frame_data[1][current_chirp].append(values[0])
                frame_data[2][current_chirp].append(values[1])
                frame_data[3][current_chirp].append(values[2])

    # convert nested defaultdicts to plain dicts
    normalized_frames = {}
    for frame_num, rx_map in frames.items():
        normalized_frames[frame_num] = {
            rx_num: dict(chirp_map) for rx_num, chirp_map in rx_map.items()
        }
    return normalized_frames


def write_frames(frames, output_file):
    with open(output_file, 'w', encoding='utf-8') as out:
        for frame_num in sorted(frames):
            out.write(f"[Frame {frame_num}:]\n")
            frame_data = frames[frame_num]
            for rx_num in sorted(frame_data):
                chirps = frame_data[rx_num]
                if not chirps:
                    continue
                out.write(f"RX{rx_num}:\n")
                for chirp_num in sorted(chirps):
                    values = ' '.join(str(value) for value in chirps[chirp_num])
                    out.write(f"  Chirp{chirp_num}: {values}\n")
                out.write("\n")
            out.write("\n")


def parse_radar_data(input_file, output_file):
    with open(input_file, 'r', encoding='utf-8') as src:
        content = src.read()

    if 'Sample' in content:
        frames = parse_new_format(content)
    else:
        frames = parse_old_format(content)

    write_frames(frames, output_file)


def main():
    parser = argparse.ArgumentParser(description="Reformat radar capture logs without timestamps.")
    parser.add_argument('input', nargs='?', default='example_data.log', help='Input log file path')
    parser.add_argument('output', nargs='?', default='formatted_output.txt', help='Destination file path')
    args = parser.parse_args()

    print(f"Parsing {args.input}...")
    parse_radar_data(args.input, args.output)
    print(f"Output written to {args.output}")


if __name__ == '__main__':
    main()
