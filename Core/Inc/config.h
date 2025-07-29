/*
 * config.h
 *
 *  Created on: May 29, 2025
 *      Author: crist
 */

#ifndef INC_CONFIG_H_        ///< Include guard start
#define INC_CONFIG_H_

#include "stdint.h"          ///< Standard integer types

/* -------------------------------------------------------------------------- */
/*                           Shared Definitions                               */
/* -------------------------------------------------------------------------- */

/**
 * @brief Packet header used to identify the start of a binary data frame.
 */
#define PACKET_HEADER 0xAABBCCDD

/**
 * @brief Packet footer used to identify the end of a binary data frame.
 */
#define PACKET_FOOTER 0XEE8899FF

/**
 * @brief Packed structure to represent a binary data packet.
 *
 * The packet contains:
 * - Header identifier
 * - Timestamp in milliseconds
 * - Roll, Pitch, Yaw angles (3 floats)
 * - Gyroscope data: X, Y, Z axes (3 floats)
 * - Accelerometer data: X, Y, Z axes (3 floats)
 * - adc values (4 floats)
 * - Footer identifier
 */
typedef struct __attribute__((packed)) {
    uint32_t header;     ///< Packet start marker (should be PACKET_HEADER: 0xAABBCCDD)
    uint32_t timestamp;  ///< Timestamp in milliseconds
    float ang[3];        ///< Orientation angles: Roll, Pitch, Yaw
    float gyr[3];        ///< Gyroscope data: X, Y, Z
    float acc[3];        ///< Accelerometer data: X, Y, Z
    float adc[4];	 	 ///< Weight measurements from MCP3564R sensor
    uint32_t footer;     ///< Packet end marker (should be PACKET_FOOTER: 0XEE8899FF)
} BinaryPacket;

#endif /* INC_CONFIG_H_ */   ///< Include guard end
