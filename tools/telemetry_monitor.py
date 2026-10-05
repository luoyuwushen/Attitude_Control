"""Read J280 UART frames, verify CRC, and optionally record a CSV for acceptance."""
import argparse
import csv
import sys
import time
from pathlib import Path

# Keep direct script invocation working as well as importing tools as a package.
if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from host.core import BASE_FIELDS, EXTENDED_FIELDS, StreamDecoder, _decode_frame, crc16


def decode(frame):
    """Decode one v1/v2 frame, retaining the legacy public time field."""
    record = _decode_frame(frame)
    record['time'] = time.time()
    return record


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True, help='e.g. COM5')
    parser.add_argument('--csv', type=Path)
    parser.add_argument('--command', choices=['D', 'U', 'G', 'H', 'S', 'R', 'F', 'B', 'J', 'K', 'L', 'M'])
    parser.add_argument('--seconds', type=float, default=10)
    args = parser.parse_args()
    import serial
    decoder = StreamDecoder()
    output = args.csv.open('w', newline='', encoding='utf-8-sig') if args.csv else None
    writer = None
    # Serial open does not intentionally assert reset; board's CH340 is UART only.
    try:
        with serial.Serial(args.port, 115200, timeout=0.1) as port:
            if args.command:
                port.write(args.command.encode('ascii'))
            deadline = time.monotonic() + args.seconds
            while time.monotonic() < deadline:
                for record in decoder.feed(port.read(128)):
                    print(f"seq={record['sequence']} state={record['state']} "
                          f"ADC={record['adc']} theta={record['theta_deg']:.2f}deg "
                          f"arm={record['arm_deg']:.2f}deg pwm={record['command_permille']} "
                          f"cal={record['calibrated']} fault=0x{record['fault']:02x}")
                    if output:
                        if writer is None:
                            writer = csv.DictWriter(
                                output, fieldnames=BASE_FIELDS + EXTENDED_FIELDS,
                                restval='', extrasaction='ignore')
                            writer.writeheader()
                        writer.writerow(record)
    finally:
        if output:
            output.close()


if __name__ == '__main__':
    main()
