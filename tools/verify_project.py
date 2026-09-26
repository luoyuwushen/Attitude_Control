"""Check project membership and every package pad against audited J280 sources."""
from pathlib import Path
import hashlib
import json
import re
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT / 'Project/Attitude_Control'


def verify():
    project = ET.parse(PROJECT / 'Attitude_Control.gprj').getroot()
    device = project.find('Device')
    assert device.attrib['pn'] == 'GW2A-LV55PG484C8/I7'
    listed = {entry.attrib['path'] for entry in project.find('FileList')}
    actual = {'src/' + p.name for p in (PROJECT / 'src').iterdir()
              if p.suffix in {'.v', '.cst', '.sdc'}}
    assert listed == actual, f'GPRJ membership mismatch: {listed ^ actual}'
    assert all((PROJECT / path).is_file() for path in listed)
    expected = {
        'clk_50m': 'M19', 'rst_n': 'AB3',
        'key_sw[0]': 'T18', 'key_sw[1]': 'R18',
        'key_sw[2]': 'U20', 'key_sw[3]': 'T20',
        'uart_tx': 'K20', 'uart_rx': 'L20', 'adc_clk': 'G18',
        'adc_oe_n': 'H20', 'adc_otr': 'F18',
        'enc1_a': 'Y20', 'enc1_b': 'AA20',
        'AN1': 'Y17', 'AN2': 'Y19', 'PWMA': 'Y18',
        'BN1': 'AA17', 'BN2': 'V15', 'PWMB': 'V14',
    }
    expected.update({f'adc_data_in[{i}]': pin for i, pin in enumerate(
        ['L21', 'M21', 'J20', 'J19', 'D19', 'H18', 'H19', 'G19', 'G17', 'E19'])})
    cst = (PROJECT / 'src/attitude_control.cst').read_text(encoding='utf-8')
    entries = re.findall(r'IO_LOC\s+"([^"]+)"\s+(\w+)\s*;', cst)
    assert len(entries) == len(dict(entries)), 'Duplicate port constraint'
    assert len({pad for _, pad in entries}) == len(entries), 'Duplicate package pad'
    assert dict(entries) == expected, 'Pin mapping differs from audited schematic'
    io = dict(re.findall(r'IO_PORT\s+"([^"]+)"\s+([^;]+);', cst))
    assert io.keys() == expected.keys()
    for port, attributes in io.items():
        standard, voltage = ('LVCMOS15', '1.5') if port.startswith('key_sw') else ('LVCMOS33', '3.3')
        assert 'IO_TYPE=' + standard in attributes.replace(' ', '')
        assert 'BANK_VCCIO=' + voltage in attributes.replace(' ', '')
    top = (PROJECT / 'src/top.v').read_text(encoding='utf-8')
    port_section = top.split(')(\n', 1)[1].split(');', 1)[0]
    top_ports = set()
    for direction, width, names in re.findall(
            r'\b(input|output)\s+wire\s*(\[\d+:\d+\])?\s+([^\n]+)', port_section):
        names = names.split('//')[0].strip().rstrip(',')
        for name in names.split(','):
            name = name.strip()
            if width:
                high, low = map(int, re.findall(r'\d+', width))
                top_ports.update(f'{name}[{i}]' for i in range(low, high + 1))
            else:
                top_ports.add(name)
    assert top_ports == expected.keys(), f'Unconstrained or unused ports: {top_ports ^ expected.keys()}'
    # Prevent stale timing exceptions from silently matching nonexistent ports.
    sdc = (PROJECT / 'src/attitude_control.sdc').read_text(encoding='utf-8')
    base_ports = {name.split('[')[0] for name in top_ports}
    for group in re.findall(r'get_ports\s+\{([^}]+)\}', sdc):
        for name in group.split():
            assert name.split('[')[0] in base_ports, f'Unknown SDC port: {name}'
    manifest = json.loads((ROOT / 'docs/reference_manifest.json').read_text(encoding='utf-8'))
    present = 0
    for name, checksum in manifest.items():
        path = ROOT / name
        if path.is_file():
            assert hashlib.sha256(path.read_bytes()).hexdigest() == checksum, f'Reference changed: {name}'
            present += 1
    print(f'PASS: {len(listed)} project files, {len(expected)} unique pads, voltage standards, '
          f'{present}/{len(manifest)} local reference checksums')


if __name__ == '__main__':
    verify()
