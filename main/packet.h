#ifndef __PACKET_H
#define __PACKET_H

#include "main.h"

typedef struct {
    uint16_t header;
    uint8_t cnt;
    uint8_t len;
    uint8_t brake;
    uint8_t ledbar;
    int16_t forward;
    int16_t clockwise;
} packet_t;

void pakcet_decoding(packet_t * packet, uint8_t * spp_data, uint16_t len);

#endif /* __PACKET_H */