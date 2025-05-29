/*
 * config.h
 *
 *  Created on: May 29, 2025
 *      Author: crist
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

#include "stdint.h"


/* Shared variables and string to be sent format */
#define PACKET_HEADER 0xAABBCCDD
#define PACKET_FOOTER 0XEE8899FF



typedef struct __attribute__((packed)) {
    uint32_t header;      // 0xAABBCCDD
    uint32_t timestamp;
    float ang[3];
    float gyr[3];
    float acc[3];
    uint32_t footer;       // 0XEE8899FF
} BinaryPacket;







#endif /* INC_CONFIG_H_ */
