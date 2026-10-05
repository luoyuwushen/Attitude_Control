"""Versioned measurement status shared by admission, tuning and history.

Raw OTR is evidence, not a hard-fault verdict in project-v0.3.0. Only the
device's accepted-sample bits can establish that a conditioned value is usable.
"""

ADC_CONDITIONING_FIRMWARE = 0x00030000


def has_adc_conditioning(record):
    return type(record.get('firmware_version')) is int and record['firmware_version'] == ADC_CONDITIONING_FIRMWARE


def in_blind_zone(record):
    flags = record.get('sensor_flags')
    return has_adc_conditioning(record) and type(flags) is int and 0 <= flags <= 65535 and bool(flags & 0x20)


def measurement_ready(record):
    flags = record.get('sensor_flags')
    if type(flags) is not int or not 0 <= flags <= 65535:
        return False
    if has_adc_conditioning(record):
        return flags & ~0x7f == 0 and flags & 0x2d == 0x0c
    return flags & 0x1f == 0x0c
