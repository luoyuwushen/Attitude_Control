"""Validated serial line settings shared by the interface and worker."""
from dataclasses import asdict, dataclass


@dataclass(frozen=True)
class SerialConfig:
    baudrate: int = 115200
    bytesize: int = 8
    parity: str = 'N'
    stopbits: float = 1.0

    def __post_init__(self):
        self.validate()

    def validate(self):
        if type(self.baudrate) is not int or not 0 < self.baudrate <= 4_000_000:
            raise ValueError('波特率必须为 1 至 4000000 的整数')
        if type(self.bytesize) is not int or self.bytesize not in (5, 6, 7, 8):
            raise ValueError('数据位必须为 5、6、7 或 8')
        if self.parity not in ('N', 'E', 'O', 'M', 'S'):
            raise ValueError('校验位必须为 N、E、O、M 或 S')
        if type(self.stopbits) not in (int, float) or self.stopbits not in (1, 1.5, 2):
            raise ValueError('停止位必须为 1、1.5 或 2')
        return self

    def as_dict(self):
        return asdict(self)

    @property
    def is_project_default(self):
        return (self.baudrate, self.bytesize, self.parity, self.stopbits) == (115200, 8, 'N', 1)

    @property
    def display_label(self):
        return f'{self.baudrate} / {self.bytesize}{self.parity}{self.stopbits:g}'

    @property
    def label(self):
        return self.display_label
