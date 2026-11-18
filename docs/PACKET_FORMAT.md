## UART framing (Pico → nRF52833-DK, prototype only)

Link: 115200 baud, 8N1, no flow control. Pico GP4 (TX) → 1 kΩ → DK uart1 RX.

Each packet is sent as one frame:

| Offset | Size | Field   | Value                                   |
|--------|------|---------|-----------------------------------------|
| 0      | 1    | sync1   | 0xAA                                    |
| 1      | 1    | sync2   | 0x55                                    |
| 2      | 1    | len     | number of packet bytes that follow (12–84) |
| 3      | len  | packet  | output of packet_encode (CRC included)  |

Receiver:
1. Scan for 0xAA followed by 0x55.
2. Read len; if len < 12 or len > 84, discard and resume scanning.
3. Read len bytes; verify the packet CRC.
4. On CRC failure, discard and resume scanning from the byte after the 0xAA.
5. Keep only the most recent valid packet.

Rate: one frame every 50 ms (20 Hz). Max frame = 87 bytes ≈ 7.6 ms at 115200.

**Wiring:** Pico GP4 → 1 kΩ → DK RX (currently P0.08; final uart1 pins up to DK owner),
Pico GND ↔ DK GND (required). 3.3 V logic. One-way, no ACK.
If using P0.08 with DK USB connected: SW6 = nRF ONLY (DEFAULT to flash).

**CRC:** CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflect, xorout 0)
over packet[0 .. len-3]; stored in the last 2 bytes, little-endian.
Check: "123456789" → 0x29B1.

**Test frame** (n = 5, len = 72, CRC 0x369B):
aa554801efbe2143658705ff7f0080fffff9ff0000ffffff7f0080e703f2ffd204ffffff7f0080cf07ebffa409ffffff7f0080b70be4ff760effffff7f00809f0fddff4813ffffc8029b36

**Forwarding:** treat the packet as opaque; put all `len` bytes (no AA 55 len header)
into the periodic advertising data. If no new packet since the last event, re-send
the previous one (Pi drops duplicate seq). CTE must run every event regardless.
Confirm max payload (84 B) fits the +UUDFP limit.