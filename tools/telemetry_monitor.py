"""Read J280 UART frames, verify CRC, and optionally record a CSV for acceptance."""
import argparse
import csv
import math
import struct
import sys
import time
from pathlib import Path


def crc16(data):
    crc = 0xffff
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021 if crc & 0x8000 else crc << 1) & 0xffff
    return crc


def decode(frame):
    if len(frame) != 24 or frame[:4] != b'\xaa\x55\x01\x18':
        raise ValueError('Invalid frame header or size')
    if crc16(frame[:22]) != int.from_bytes(frame[22:], 'little'):
        raise ValueError('CRC mismatch')
    sequence, adc, theta, omega, arm, arm_speed, command = struct.unpack('<HHhhhhh', frame[4:18])
    return {
        'time': time.time(), 'sequence': sequence, 'adc': adc,
        'theta_deg': theta / 1024 * 180 / math.pi,
        'omega_rad_s': omega / 1024, 'arm_deg': arm / 1024 * 180 / math.pi,
        'arm_speed_rad_s': arm_speed / 1024, 'command_permille': command,
        'state': frame[18], 'calibrated': frame[19] & 1, 'fault': frame[20],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True, help='e.g. COM5')
    parser.add_argument('--csv', type=Path)
    parser.add_argument('--command', choices=['D', 'U', 'G', 'S', 'R', 'F', 'B'])
    parser.add_argument('--seconds', type=float, default=10)
    args = parser.parse_args()
    import serial
    buffer = bytearray()
    output = args.csv.open('w', newline='', encoding='utf-8-sig') if args.csv else None
    writer = None
    # Serial open does not intentionally assert reset; board's CH340 is UART only.
    try:
        with serial.Serial(args.port, 115200, timeout=0.1) as port:
            if args.command:
                port.write(args.command.encode('ascii'))
            deadline = time.monotonic() + args.seconds
            while time.monotonic() < deadline:
                buffer.extend(port.read(128))
                while len(buffer) >= 24:
                    header = buffer.find(b'\xaa\x55\x01\x18')
                    if header < 0:
                        del buffer[:-3]
                        break
                    del buffer[:header]
                    if len(buffer) < 24:
                        break
                    try:
                        record = decode(bytes(buffer[:24]))
                    except ValueError:
                        del buffer[0]
                        continue
                    del buffer[:24]
                    print(f"seq={record['sequence']} state={record['state']} "
                          f"ADC={record['adc']} theta={record['theta_deg']:.2f}deg "
                          f"arm={record['arm_deg']:.2f}deg pwm={record['command_permille']} "
                          f"cal={record['calibrated']} fault=0x{record['fault']:02x}")
                    if output:
                        if writer is None:
                            writer = csv.DictWriter(output, fieldnames=record)
                            writer.writeheader()
                        writer.writerow(record)
    finally:
        if output:
            output.close()


if __name__ == '__main__':
    main()
