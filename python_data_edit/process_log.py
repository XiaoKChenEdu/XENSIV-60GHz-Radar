"""
Script to process radar log files by keeping only chirp_1 through chirp_16 data without timestamps.
"""

import re
import os

def process_log_file(input_file, output_file=None):
    """Process log file to keep chirp_1 through chirp_16 data without timestamps."""
    if output_file is None:
        base_name = os.path.splitext(input_file)[0]
        output_file = f"{base_name}_processed.txt"

    with open(input_file, 'r', encoding='utf-8') as f:
        lines = f.readlines()

    start_index = None
    end_index = None
    chirp_start = re.compile(r'chirp_1:', re.IGNORECASE)
    chirp_end = re.compile(r'chirp_16:', re.IGNORECASE)

    for i, line in enumerate(lines):
        if start_index is None and chirp_start.search(line):
            start_index = i
        if chirp_end.search(line):
            end_index = i

    if start_index is None or end_index is None or start_index > end_index:
        print("Warning: Could not locate chirp_1 through chirp_16 in the file")
        return None

    timestamp_pattern = re.compile(r'^\[[\d\-]+ [\d:]+\]\s*')
    trimmed_lines = lines[start_index:end_index + 1]
    processed_lines = [timestamp_pattern.sub('', line) for line in trimmed_lines]

    with open(output_file, 'w', encoding='utf-8') as f:
        f.writelines(processed_lines)

    print(f"Processed file saved to: {output_file}")
    return output_file


def main():
    """Main function to process log files"""
    # Example usage - process the log file in the current directory
    log_file = "103cm-5hz-1cmpp.log"
    
    # Check if file exists
    if not os.path.exists(log_file):
        print(f"Error: File '{log_file}' not found")
        print(f"Current directory: {os.getcwd()}")
        return
    
    # Process the file
    output_file = process_log_file(log_file)
    
    if output_file:
        print(f"\nProcessing complete!")
        print(f"Input file:  {log_file}")
        print(f"Output file: {output_file}")


if __name__ == "__main__":
    main()
