"""Explicit tuning aliases; targets are known controller semantics, not sensors."""
import math
from host.adc_status import measurement_ready

TUNING_FIELDS = ('actual', 'target', 'out', 'algorithm', 'actual_valid', 'arm_actual_deg',
                 'arm_target_deg', 'integral_permille', 'active', 'error_deg',
                 'arm_error_deg', 'requested_saturated', 'parameter_profile')
TUNING_CSV_FIELDS = ('time_utc', 'time', 'host_monotonic', 'device_time_ms', 'sequence',
                     'firmware_version', 'algorithm', 'active', 'actual_valid',
                     'actual', 'target', 'out', 'arm_actual_deg', 'arm_target_deg',
                     'integral_permille', 'omega_rad_s', 'arm_speed_rad_s',
                     'motor_command_permille', 'h_control_flags', 'error_deg',
                     'arm_error_deg', 'requested_saturated', 'parameter_profile',
                     'state', 'fault', 'first_fault')
H_LQI_PROFILE = 'project-v0.2.6/H_LQI_Qi1'
H_LQI_S110_PROFILE = 'project-v0.2.7/H_LQI_S110'
H_LQI_S120_PROFILE = 'project-v0.2.8/H_LQI_S120'
H_LQI_TRACE_PROFILE = 'project-v0.2.9/H_LQI_S120'
H_LQI_CONDITIONED_PROFILE = 'project-v0.3.0/H_LQI_S120_ADC'
H_LQI_PROFILES = {0x00020006: H_LQI_PROFILE, 0x00020007: H_LQI_S110_PROFILE,
                  0x00020008: H_LQI_S120_PROFILE, 0x00020009: H_LQI_TRACE_PROFILE,
                  0x00030000: H_LQI_CONDITIONED_PROFILE}


def _handover_profile(version, speed_gain):
    """Static definitions for exact known firmware IDs, never parameter readback."""
    return {
        'source': 'known_firmware_definition', 'firmware_version': version,
        'algorithm': 'H_LQI', 'physical_validation': 'candidate_not_accepted',
        'state_order': ['theta_q10', 'omega_q10', 'arm_minus_capture_q10', 'arm_speed_q10'],
        'gain_q10': [904758, 77909, -34213, speed_gain],
        'integral_step_coefficient': 237, 'ki_permille_per_rad_s': 237 / 16.384,
        'integral_bound_permille': 100, 'integral_rate_accumulator_limit': 335544,
        'integral_rate_permille_per_s': 335544 * 1000 / 2**24,
        'integral_start_ms': 384, 'control_period_ms': 1,
        'arm_stiffness_ramp_ms': [128, 256, 384], 'output_bound_permille': 1000,
    }


TUNING_SCHEMA = {
    'schema_version': 1,
    'actual': {'unit': 'deg', 'source': 'theta_deg', 'meaning': 'Calibrated sensor angle, not independent physical ground truth.'},
    'target': {'unit': 'deg', 'source': 'known_H_LQI_control_law',
               'meaning': 'Zero only for known firmware 0x00020006, 0x00020007, 0x00020008, 0x00020009 or 0x00030000 with complete H observations, no reserved flags and active H. Otherwise blank; not a measurement.'},
    'out': {'unit': 'permille', 'source': 'command_permille',
            'meaning': 'Requested PWM command, not gated motor command, measured duty or torque.'},
    'algorithm': {'unit': 'name', 'meaning': 'H_LQI only for confirmed active H; otherwise unknown. This is not a PID controller.'},
    'actual_valid': {'unit': 'boolean', 'meaning': 'Calibrated, accepted measurement valid and ready, no bad sample or blind zone, fault zero. Raw OTR/rejection warnings do not invalidate accepted project-v0.3.0 measurements. Invalid actual values remain for diagnosis.'},
    'arm_actual_deg': {'unit': 'deg', 'source': 'arm_deg', 'meaning': 'Angle using the nominal encoder scale.'},
    'arm_target_deg': {'unit': 'deg', 'source': 'h_capture_arm_q10', 'meaning': 'Captured reference only for active H.'},
    'integral_permille': {'unit': 'permille', 'source': 'h_integral_q8 / 256',
                          'meaning': 'Command-aligned integral observation, quantized to Q8; absent input stays blank. Not an exact integer command contribution.'},
    'active': {'unit': 'boolean_or_null',
               'meaning': 'Known complete H group with H-active flag and state=2/fault=0; missing or unknown identity/flags stay blank.'},
    'error_deg': {'unit': 'deg', 'source': 'target - actual', 'meaning': 'Only known active H with actual_valid.'},
    'arm_error_deg': {'unit': 'deg', 'source': 'arm_target_deg - arm_actual_deg', 'meaning': 'Only known active H with actual_valid.'},
    'omega_rad_s': {'unit': 'rad/s', 'source': 'omega_rad_s'},
    'arm_speed_rad_s': {'unit': 'rad/s', 'source': 'arm_speed_rad_s'},
    'motor_command_permille': {'unit': 'permille', 'meaning': 'Gated software output, not measured PWM.'},
    'h_control_flags': {'unit': 'bits', 'meaning': 'bit0 H active; bit1 integral update allowed; bit2 antiwindup frozen; bit3 command-aligned integral at bound.'},
    'requested_saturated': {'unit': 'boolean_or_null', 'meaning': 'abs(out)>=1000 only while state=2; other states blank.'},
    'parameter_profile': {'unit': 'name', 'meaning': 'Known firmware definition only, not runtime readback or serial-writable parameters.'},
    'profile_definitions': {
        H_LQI_PROFILE: _handover_profile(0x00020006, -51473),
        H_LQI_S110_PROFILE: _handover_profile(0x00020007, -56620),
        H_LQI_S120_PROFILE: _handover_profile(0x00020008, -61768),
        H_LQI_TRACE_PROFILE: _handover_profile(0x00020009, -61768),
        H_LQI_CONDITIONED_PROFILE: _handover_profile(0x00030000, -61768),
    },
}


def tuning_values(record):
    def finite(key):
        value = record.get(key)
        return value if type(value) in (int, float) and math.isfinite(value) else None

    def integer(key, low, high):
        value = record.get(key)
        return type(value) is int and low <= value <= high

    version = record.get('firmware_version')
    profile = H_LQI_PROFILES.get(version) if type(version) is int else None
    known = (profile is not None and
             integer('h_integral_q8', -32768, 32767) and
             integer('h_capture_arm_q10', -32768, 32767) and
             integer('h_control_age_ms', 0, 65535) and
             integer('h_control_flags', 0, 15))
    active = (bool(record['h_control_flags'] & 1) and record.get('state') == 2 and
              record.get('fault') == 0) if known else None
    values = {
        'actual': finite('theta_deg'), 'target': 0.0 if active else None,
        'out': finite('command_permille'), 'algorithm': 'H_LQI' if active else 'unknown',
        'actual_valid': (record.get('calibrated') == 1 and record.get('fault') == 0 and
                         integer('sensor_flags', 0, 65535) and
                         measurement_ready(record) and finite('theta_deg') is not None),
        'arm_actual_deg': finite('arm_deg'),
        'arm_target_deg': record['h_capture_arm_q10'] / 1024 * 180 / math.pi if active else None,
        'integral_permille': record['h_integral_q8'] / 256
            if integer('h_integral_q8', -32768, 32767) else None,
        'active': active,
    }
    values.update(error_deg=values['target'] - values['actual']
                  if active and values['actual_valid'] else None,
                  arm_error_deg=values['arm_target_deg'] - values['arm_actual_deg']
                  if active and values['actual_valid'] and values['arm_actual_deg'] is not None else None,
                  requested_saturated=abs(values['out']) >= 1000
                  if record.get('state') == 2 and values['out'] is not None else None,
                  parameter_profile=profile if active else None)
    return values
