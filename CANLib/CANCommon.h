/* File:         CANCommon.h
 * Authors:      Jaden Bottemiller, Benton Kwong, Dylan Tomberlin.
 * Organization: Husky Robotics Team
 * 
 * This file includes fuction prototypes for Common Mode CAN Communication
 * using the Hindsight CAN Communication standard.
 * Documentation: https://huskyroboticsteam.slite.com/app/channels/iU0BryG7M9/collections/aXvWTcIR6c/notes/4otlSFsSp2
 */

#pragma once

#include "CANPacket.h"

void AssembleEmergencyStopPacket(CANPacket *packet,
    uint8_t targetDeviceGroup,
    uint8_t targetDeviceSerialNumber,
    uint8_t errorCode);
void AssembleGroupBroadcastingEmergencyStopPacket(CANPacket *packet, 
    uint8_t groupCode, 
    uint8_t errorCode);
void AssembleBrodcastEmergencyStopPacket(CANPacket *packet,
    uint8_t errorCode);
uint8_t GetEmergencyStopErrorCode(CANPacket *packet);

uint32_t GetTimeBetweenHeartbeatPacket(CANPacket *packet, uint32_t lastHeartbeat);
uint32_t GetHeartbeatTimeStamp(CANPacket *packet);
void AssembleHeartbeatPacket(CANPacket *packetToAssemble, 
    int broadcast, 
    uint8_t heartbeatLeniencyCode,
    uint32_t timestamp);

void AssembleFailReportPacket(CANPacket *packetToAssemble, 
    uint8_t targetGroup, 
    uint8_t targetSerial,
    uint8_t failedPacketID);

void AssembleOverrideProtectionPacket(CANPacket *packetToAssemble, uint8_t targetGroup, uint8_t targetSerial);

//Chip type pull
void AssembleChipTypePullPacket(CANPacket *packetToAssemble,
    uint8_t targetDeviceGroup,
    uint8_t targetDeviceSerial);
void AssembleChipTypeReportPacket(CANPacket *packetToAssemble,
    uint8_t targetDeviceGroup,
    uint8_t targetDeviceSerial);
uint8_t GetChipTypeFromPacket(CANPacket *packet);

void AssembleTelemetryTimingPacket(CANPacket *packetToAssemble, 
    uint8_t targetGroup, 
    uint8_t targetSerial, 
    uint8_t telemetryTypeCode,
    uint32_t msBetweenReports);
uint32_t GetTelemetryTimingFromPacket(CANPacket *packetToAssemble);

void AssembleTelemetryPullPacket(CANPacket *packetToAssemble, 
    uint8_t targetGroup, 
    uint8_t targetSerial, 
    uint8_t telemetryTypeCode);

void AssembleTelemetryReportPacket(CANPacket *packetToAssemble, 
    uint8_t targetGroup, 
    uint8_t targetSerial,
    uint8_t telemetryTypeCode,
    int32_t data);

int32_t DecodeTelemetryDataSigned(CANPacket *packet);
uint32_t DecodeTelemetryDataUnsigned(CANPacket *packet);
uint8_t DecodeTelemetryType(CANPacket *packet);

void AssembleRGBColorPacket(CANPacket *packetToAssemble,
    uint8_t targetGroup,
    uint8_t targetSerial,
    uint8_t addrLED,
    uint8_t R,
    uint8_t G,
    uint8_t B);

// Common Mode Packet IDs
#define ID_ESTOP                        (uint8_t) 0x30
#define ID_HEARTBEAT                    (uint8_t) 0x31
#define ID_FAIL_REPORT                  (uint8_t) 0x32
#define ID_OVRD_PROTECTION              (uint8_t) 0x33
#define ID_TELEMETRY_TIMING             (uint8_t) 0x34
#define ID_TELEMETRY_PULL               (uint8_t) 0x35
#define ID_TELEMETRY_REPORT             (uint8_t) 0x36
#define ID_LED_COLOR                    (uint8_t) 0x37
#define ID_CHIP_TYPE_PULL               (uint8_t) 0x38
#define ID_CHIP_TYPE_REP                (uint8_t) 0x39

// DLC Common Mode Packets 
#define DLC_ESTOP                        (uint8_t) 0x03
#define DLC_HEARTBEAT                    (uint8_t) 0x07
#define DLC_FAIL_REPORT                  (uint8_t) 0x03
#define DLC_OVRD_PROTECTION              (uint8_t) 0x01
#define DLC_TELEMETRY_TIMING             (uint8_t) 0x06
#define DLC_TELEMETRY_PULL               (uint8_t) 0x03
#define DLC_TELEMETRY_REPORT             (uint8_t) 0x07
#define DLC_LED_COLOR                    (uint8_t) 0x06
#define DLC_CHIP_TYPE_PULL               (uint8_t) 0x03
#define DLC_CHIP_TYPE_REP                (uint8_t) 0x03

//Packet priorities
#define PRIO_CHIP_TYPE_REP               PACKET_PRIORITY_NORMAL

// Telemetry Types
#define PACKET_TELEMETRY_VOLTAGE        (uint8_t) 0x00
#define PACKET_TELEMETRY_CURRENT        (uint8_t) 0x01
#define PACKET_TELEMETRY_PWR_RAIL_STATE (uint8_t) 0x02
#define PACKET_TELEMETRY_TEMPERATURE    (uint8_t) 0x03
#define PACKET_TELEMETRY_ANG_POSITION   (uint8_t) 0x04
#define PACKET_TELEMETRY_GPS_LAT        (uint8_t) 0x05
#define PACKET_TELEMETRY_GPS_LON        (uint8_t) 0x06
#define PACKET_TELEMETRY_MAG_DIR        (uint8_t) 0x07
#define PACKET_TELEMETRY_ACCEL_X        (uint8_t) 0x08
#define PACKET_TELEMETRY_ACCEL_Y        (uint8_t) 0x09
#define PACKET_TELEMETRY_ACCEL_Z        (uint8_t) 0x0A
#define PACKET_TELEMETRY_GYRO_X         (uint8_t) 0x0B
#define PACKET_TELEMETRY_GYRO_Y         (uint8_t) 0x0C
#define PACKET_TELEMETRY_GYRO_Z         (uint8_t) 0x0D
#define PACKET_TELEMETRY_LIM_SW_STATE   (uint8_t) 0x0E

// ESTOP ERROR CODES
#define ESTOP_ERR_GENERAL               (uint8_t) 0x00
// MORE TBD...
