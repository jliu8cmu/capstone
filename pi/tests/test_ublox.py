import pytest

from swimtrack import packet as pk
from swimtrack import ublox

GOLDEN = bytes.fromhex(
    "01efbe2143658705ff7f0080fffff9ff0000ffffff7f0080e703f2ffd204ffffff7f0080"
    "cf07ebffa409ffffff7f0080b70be4ff760effffff7f00809f0fddff4813ffffc8029b36")


def mfr(payload, company=b"\xff\xff"):
    body = bytes([0xFF]) + company + payload
    return bytes([len(body)]) + body


def test_iter_ad_splits_structures():
    adv = bytes([2, 0x01, 0x06]) + mfr(b"\xAB")
    assert list(ublox.iter_ad(adv)) == [(0x01, b"\x06"), (0xFF, b"\xff\xff\xab")]


def test_iter_ad_stops_at_zero_padding():
    adv = mfr(b"\xAB") + b"\x00\x00\x00"
    assert len(list(ublox.iter_ad(adv))) == 1


def test_iter_ad_truncated():
    with pytest.raises(pk.FormatError):
        list(ublox.iter_ad(bytes([10, 0xFF, 0x01])))


def test_extract_only_structure():
    assert ublox.extract_packet_bytes(mfr(GOLDEN)) == GOLDEN


def test_extract_skips_other_structures():
    adv = bytes([2, 0x01, 0x06]) + bytes([4, 0x09]) + b"tag" + mfr(GOLDEN)
    assert ublox.extract_packet_bytes(adv) == GOLDEN


def test_extract_wrong_company():
    with pytest.raises(pk.FormatError):
        ublox.extract_packet_bytes(mfr(GOLDEN, company=b"\x59\x00"))


def test_extract_missing():
    with pytest.raises(pk.FormatError):
        ublox.extract_packet_bytes(bytes([2, 0x01, 0x06]))


def test_decode_adv_hex_end_to_end():
    p = ublox.decode_adv_hex(' "' + mfr(GOLDEN).hex().upper() + '"\r\n')
    assert (p.seq, p.n, p.flags) == (0xBEEF, 5, pk.FLAG_OVERRUN)


def test_decode_adv_hex_bad_crc():
    adv = bytearray(mfr(GOLDEN))
    adv[10] ^= 0x01
    with pytest.raises(pk.CrcError):
        ublox.decode_adv_hex(adv.hex())


def test_decode_adv_hex_garbage():
    with pytest.raises(pk.FormatError):
        ublox.decode_adv_hex("xyz")


# ---- text lines (examples from the anchor teammate) -------------------------
ANGLE_EXAMPLE = '+UUDF:000000000001,-71,-12,-1,0,11,"80A197B210EC","",123456,50224\r\n'
PAYLOAD_EXAMPLE = "+UUDFP:000000000001,0BFFFFFF013412A086010000\r\n"


def test_parse_angle_line_example():
    r = ublox.parse_angle_line(ANGLE_EXAMPLE)
    assert (r.tag, r.rssi, r.azimuth, r.elevation, r.channel) == ("000000000001", -71, -12, -1, 11)
    assert (r.anchor_id, r.timestamp_ms, r.counter) == ("80A197B210EC", 123456, 50224)


def test_parse_angle_line_curly_quotes():
    line = "+UUDF:000000000001,-71,-12,-1,0,11,“80A197B210EC“,”“,123456,50224"
    r = ublox.parse_angle_line(line)
    assert r.anchor_id == "80A197B210EC" and r.counter == 50224


def test_parse_angle_line_ignores_payload_and_other_lines():
    assert ublox.parse_angle_line(PAYLOAD_EXAMPLE) is None
    assert ublox.parse_angle_line("OK") is None


@pytest.mark.parametrize("line", ["+UUDF:000000000001,-71,-12",
                                  '+UUDF:000000000001,x,-12,-1,0,11,"A","",1,2'])
def test_parse_angle_line_malformed(line):
    with pytest.raises(pk.FormatError):
        ublox.parse_angle_line(line)


def test_parse_payload_line_example():
    assert ublox.parse_payload_line(PAYLOAD_EXAMPLE) == ("000000000001", "0BFFFFFF013412A086010000")


def test_parse_payload_line_ignores_angle_line():
    assert ublox.parse_payload_line(ANGLE_EXAMPLE) is None


def test_teammate_payload_example_extracts():
    tag, hex_part = ublox.parse_payload_line(PAYLOAD_EXAMPLE)
    assert ublox.extract_packet_bytes(bytes.fromhex(hex_part)) == bytes.fromhex("013412A086010000")
