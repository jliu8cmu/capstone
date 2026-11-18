import binascii
import struct
from dataclasses import dataclass
import numpy as np  


VERSION = 1
HEADER_LEN = 8
SAMPLE_LEN = 12
TRAILER_LEN = 4
MAX_SAMPLES = 6
FLAG_IMU_ERR = 1 << 0
FLAG_OVERRUN = 1 << 1
FLAG_LOW_BATT = 1 << 2

ACC_G_PER_COUNT = 0.000244
GYR_DPS_PER_COUNT = 0.070

HDR = struct.Struct('<BHIB')
TRL = struct.Struct('<BB')
CRC = struct.Struct('<H')

assert HDR.size == HEADER_LEN

class PacketError(ValueError): pass
class CrcError(PacketError): pass
class FormatError(PacketError): pass


@dataclass
class Packet:
    version: int
    seq: int
    t0_ms: int
    samples: np.ndarray
    batt: int
    flags:int

    @property
    def n(self):
        return len(self.samples)

    @property
    def accel_g(self):
        return self.samples[:, :3] * ACC_G_PER_COUNT

    @property
    def gyro_dps(self):
        return self.samples[:,3:] * GYR_DPS_PER_COUNT

    @property
    def batt_mv(self):
        return 2500 + 10 * self.batt

def crc16(data):
    return binascii.crc_hqx(data, 0xFFFF)

def encoded_len(n):
    return HEADER_LEN + n * SAMPLE_LEN + TRAILER_LEN

def decode(data: bytes) -> Packet:
    if not (encoded_len(0) <= len(data) <= encoded_len(MAX_SAMPLES)) or (len(data) - encoded_len(0)) % SAMPLE_LEN:
        raise FormatError(f"Length mismatch length {len(data)}")
    got = int.from_bytes(data[-2:], 'little')
    calc = crc16(data[:-2])
    if got != calc:
        raise CrcError(f"crc got {got:#06x} != {calc:#06x}")

    version, seq, t0_ms, n = HDR.unpack_from(data, 0)
    if(version != VERSION):
        raise FormatError("Version Mismatch")
    if(n > MAX_SAMPLES):
        raise FormatError("n > MaxSamples")
    if(len(data) != encoded_len(n)):
        raise FormatError("length mismatch")

    samples = np.frombuffer(data, dtype='<i2', count=n*6, offset = HEADER_LEN).reshape(n, 6).copy()


    batt, flags = data[-4], data[-3]

    return Packet(version, seq, t0_ms, samples, batt, flags)

def decode_hex(text:str ) -> Packet:
    s = text.strip().strip('"')
    try:
        data =  bytes.fromhex(s)
    except ValueError as e:
        raise FormatError(f"bad hex {e}") from None
    return decode(data)

def encode(p: Packet) -> bytes:
    samples = np.asarray(p.samples, dtype = '<i2').reshape(-1, 6)
    n = len(samples)
    if n > MAX_SAMPLES:
        raise FormatError(f"n={n} > {MAX_SAMPLES}")


    body = HDR.pack(p.version, p.seq, p.t0_ms, n) + samples.tobytes() + TRL.pack(p.batt, p.flags)
    return body + CRC.pack(crc16(body))


    