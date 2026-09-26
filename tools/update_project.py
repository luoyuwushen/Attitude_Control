"""Refresh the Gowin project file list without touching board constraints."""
from pathlib import Path
import xml.etree.ElementTree as ET

p = Path(__file__).resolve().parents[1] / 'Project/Attitude_Control'
project = ET.Element('Project')
ET.SubElement(project, 'Template').text = 'FPGA'
ET.SubElement(project, 'Version').text = '5'
ET.SubElement(project, 'Device', name='GW2A-55C', pn='GW2A-LV55PG484C8/I7').text = 'gw2a55c-005'
files = ET.SubElement(project, 'FileList')
for file in sorted((p / 'src').iterdir()):
    if file.suffix in {'.v', '.cst', '.sdc'}:
        kind = 'verilog' if file.suffix == '.v' else file.suffix[1:]
        ET.SubElement(files, 'File', path='src/' + file.name, type='file.' + kind, enable='1')
ET.indent(project, space='    ')
text = '<?xml version="1.0" encoding="UTF-8"?>\n<!DOCTYPE gowin-fpga-project>\n'
(p / 'Attitude_Control.gprj').write_text(text + ET.tostring(project, encoding='unicode') + '\n', encoding='utf-8')
