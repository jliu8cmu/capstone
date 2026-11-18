import csv
from dataclasses import dataclass
from typing import Iterator, Optional, Tuple

from swimtrack import packet as pk

AD_TYPE_MANUFACTUER = 0xFF
COMPANY_ID = 0xFFFF

def iter_ad(data: bytes)->Iterator[Tuple[int, bytes]]:
    i = 0
    while (i < len(data)):
        length = data[i]
        if length ==0:
            return
        end = i + 1 + length
        if end > len(data):
            raise pk.FormatError(f"AD {i} past end ({end} > {len(data)})")
        yield data[i+1], bytes(data[i+2:end])
        i = end

def extract_packet_bytes(adv: bytes) -> bytes:
    for ad_type, body in iter_ad(adv):
        if ad_type == AD_TYPE_MANUFACTUER and len(body) >= 2:
            company =  body[0] | (body[1] << 8)
            if(company == COMPANY_ID) :
                return body[2:]
    raise pk.FormatError("manu data wrong")


def decode_adv_hex(text: str) -> pk.Packet:
    s = text.strip().strip('"')
    try:
        adv = bytes.fromhex(s)
    except ValueError as e:
        raise pk.FormatError(f"bad hex: {e}") from None
    return pk.decode(extract_packet_bytes(adv))


ANGLE_PREFIX = "+UUDF:"
PAYLOAD_PREFIX = "+UUDFP:"
_CURLY_QUOTES = str.maketrans({"\u201c": '"', "\u201d": '"'})

@dataclass
class AngleReading:
    tag: str
    rssi: int
    azimuth: int
    elevation: int
    channel: int
    anchor_id: str
    timestamp_ms: int
    counter:int

def parse_angle_line(line: str)->Optional[AngleReading]:
    line = line.strip().translate(_CURLY_QUOTES)
    if not line.startswith(ANGLE_PREFIX):
        return None

    fields = next(csv.reader([line[len(ANGLE_PREFIX):]]))
    if len(fields) < 10:
        raise pk.FormatError(f"angle line only {len(fields)}, exp 10")
    try:
        return AngleReading(tag=fields[0], rssi=int(fields[1]), azimuth = int(fields[2]), elevation = int(fields[3]),
                            channel = int(fields[5]), anchor_id = fields[6], timestamp_ms = int(fields[8]), 
                            counter = int(fields[9]))
    except ValueError as e:
        raise pk.FormatError(f"bad angle field {e}") from None





def parse_payload_line(line: str) -> Optional[Tuple[str,str]]:
    line = line.strip()
    if not line.startswith(PAYLOAD_PREFIX):
        return None
    tag, sep, hex_part = line[len(PAYLOAD_PREFIX):].partition(",")
    if not sep or not tag:
        return None
    return tag, hex_part