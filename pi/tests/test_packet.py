import numpy as np
import pytest

from swimtrack import packet as pk

GOLDEN = ("01efbe2143658705ff7f0080fffff9ff0000ffffff7f0080e703f2ffd204ffffff7f0080"
          "cf07ebffa409ffffff7f0080b70be4ff760effffff7f00809f0fddff4813ffffc8029b36")


def make_test_packet(n=5):
    samples = np.array([[32767, -32768, 1000 * i - 1, -7 * (i + 1), 1234 * i, -1]
                        for i in range(n)], dtype=np.int16).reshape(n, 6)
    return pk.Packet(pk.VERSION, 0xBEEF, 0x87654321, samples, 0xC8, pk.FLAG_OVERRUN)


def test_crc_check_value():
    assert pk.crc16(b"123456789") == 0x29B1


def test_encode_golden():
    assert pk.encode(make_test_packet()).hex() == GOLDEN


def test_decode_golden():
    p = pk.decode(bytes.fromhex(GOLDEN))
    ref = make_test_packet()
    assert (p.version, p.seq, p.t0_ms, p.n, p.batt, p.flags) == \
           (1, 0xBEEF, 0x87654321, 5, 0xC8, pk.FLAG_OVERRUN)
    np.testing.assert_array_equal(p.samples, ref.samples)
    assert p.samples.dtype == np.int16


@pytest.mark.parametrize("n", range(0, 7))
def test_roundtrip_all_n(n):
    data = pk.encode(make_test_packet(n))
    assert len(data) == 12 + 12 * n
    assert pk.encode(pk.decode(data)) == data


def test_bad_crc():
    data = bytearray.fromhex(GOLDEN)
    data[10] ^= 0x01
    with pytest.raises(pk.CrcError):
        pk.decode(bytes(data))


@pytest.mark.parametrize("length", [0, 11, 13, 85, 96])
def test_bad_length(length):
    with pytest.raises(pk.FormatError):
        pk.decode(bytes(length))


def test_n_mismatch():
    body = bytearray.fromhex(GOLDEN)[:-2]
    body[7] = 4
    data = bytes(body) + pk.CRC.pack(pk.crc16(bytes(body)))
    with pytest.raises(pk.FormatError):
        pk.decode(data)


def test_bad_version():
    body = bytearray.fromhex(GOLDEN)[:-2]
    body[0] = 2
    data = bytes(body) + pk.CRC.pack(pk.crc16(bytes(body)))
    with pytest.raises(pk.FormatError):
        pk.decode(data)


def test_decode_hex_quotes_and_case():
    p = pk.decode_hex(f' "{GOLDEN.upper()}"\r\n')
    assert p.seq == 0xBEEF


def test_decode_hex_garbage():
    with pytest.raises(pk.FormatError):
        pk.decode_hex("zz")


def test_units():
    p = pk.decode(bytes.fromhex(GOLDEN))
    assert p.accel_g[0, 0] == pytest.approx(32767 * 0.244e-3)
    assert p.gyro_dps[0, 0] == pytest.approx(-7 * 0.070)
    assert p.batt_mv == 2500 + 10 * 0xC8


def test_samples_writable():
    p = pk.decode(bytes.fromhex(GOLDEN))
    p.samples[0, 0] = 0