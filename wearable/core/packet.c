#include "packet.h"
#include "crc16.h"

static void put_u16_le(uint8_t *p, uint16_t value) {
    p[0] = (uint8_t)(value & 0xFF);
    p[1] = (uint8_t)(value  >> 8);

}


static void put_u32_le(uint8_t *p, uint32_t value) {
    p[0] = (uint8_t)(value & 0xFF);
    p[1] = (uint8_t)((value  >> 8) & 0xFF);
    p[2] = (uint8_t)((value  >> 16) & 0xFF);
    p[3] = (uint8_t)((value  >> 24) & 0xFF);
}

static uint16_t get_u16_le(const uint8_t *p) {
    return (uint16_t)(p[0] | (p[1] << 8));
}

static uint32_t get_u32_le(const uint8_t *p) {
    return (uint32_t)((uint32_t)(p[3]) << 24 | (uint32_t)(p[2]) << 16 | p[1] << 8 | p[0]);
}

int packet_encode(const packet_t *pkt, uint8_t *buf, size_t buf_size) {
    size_t pos = 0;
    if(pkt->n > PKT_MAX_SAMPLES)
        return PKT_ERR_FORMAT;
    size_t total_len = PKT_HEADER_LEN + pkt->n * PKT_SAMPLE_LEN + PKT_TRAILER_LEN;
    if(total_len > buf_size) {
        return PKT_ERR_BUF_SMALL;
    }

    buf[pos]  = PKT_VERSION; 
    pos+=1;
    put_u16_le(&buf[pos], pkt->seq);
    pos += 2;
    put_u32_le(&buf[pos], pkt->t0_ms);
    pos += 4;
    buf[pos] = pkt->n;
    pos += 1;

    for(size_t i = 0; i < pkt->n; i++) {
        put_u16_le(&buf[pos], (pkt->samples)[i].ax);
        pos += 2;
        put_u16_le(&buf[pos], (pkt->samples)[i].ay);
        pos += 2;
        put_u16_le(&buf[pos], (pkt->samples)[i].az);
        pos += 2;
        put_u16_le(&buf[pos], (pkt->samples)[i].gx);
        pos += 2;
        put_u16_le(&buf[pos], (pkt->samples)[i].gy);
        pos += 2;
        put_u16_le(&buf[pos], (pkt->samples)[i].gz);
        pos += 2;
    }
    
    buf[pos] = pkt->batt;
    pos += 1;
    buf[pos] = pkt->flags;
    pos += 1;
    put_u16_le(&buf[pos], crc16_ccitt(buf, pos));
    pos += 2;
    return pos;
}

int packet_decode(const uint8_t *buf, size_t len, packet_t *pkt) {
    if(len < PKT_HEADER_LEN + PKT_TRAILER_LEN ){ 
        return PKT_ERR_FORMAT;
    }

     
    if(crc16_ccitt(buf, len-2) != get_u16_le(buf + len - 2)) {
        return PKT_ERR_CRC;
    }

    size_t pos = 0;

    pkt->version = buf[pos];
    pos += 1;

    pkt->seq = get_u16_le(buf + pos);
    pos += 2;

    pkt->t0_ms = get_u32_le(buf + pos);

    pos += 4;

    pkt->n = buf[pos];
    pos += 1;

    if(pkt->version != PKT_VERSION || pkt->n > PKT_MAX_SAMPLES || len != 
            PKT_HEADER_LEN + pkt->n * PKT_SAMPLE_LEN + PKT_TRAILER_LEN) {
                return PKT_ERR_FORMAT;
            }

    for(size_t i = 0; i < pkt->n; i++) {
        pkt->samples[i].ax = (int16_t)(get_u16_le(buf + pos));
        pos += 2;
        pkt->samples[i].ay = (int16_t)(get_u16_le(buf + pos));
        pos += 2;
        pkt->samples[i].az = (int16_t)(get_u16_le(buf + pos));
        pos += 2;
        pkt->samples[i].gx = (int16_t)(get_u16_le(buf + pos));
        pos += 2;
        pkt->samples[i].gy = (int16_t)(get_u16_le(buf + pos));
        pos += 2;
        pkt->samples[i].gz = (int16_t)(get_u16_le(buf + pos));
        pos += 2;
    }

    pkt->batt = buf[pos];
    pos +=1 ;

    pkt->flags = buf[pos];
    pos += 1;

    return 0;

}
