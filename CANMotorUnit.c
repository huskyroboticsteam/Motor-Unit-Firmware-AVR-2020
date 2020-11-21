/* File:         CANMotorUnit.c
 * Authors:      Jaden Bottemiller, Benton Kwong, Dylan Tomberlin.
 * Organization: Husky Robotics Team
 * 
 * This file includes function definitions for CAN Packet manipulation
 * using the Hindsight CAN Communication standard. Specific files
 * for the motor unit boards.
 * Documentation: https://huskyroboticsteam.slite.com/app/channels/iU0BryG7M9/collections/aXvWTcIR6c/notes/4otlSFsSp2
 */

#include "CANPacket.h"
#include "CANMotorUnit.h"
#include "Port.h"

void AssembleModeSetPacket(CANPacket *packetToAssemble,
    uint8_t targetDeviceGroup,
    uint8_t targetDeviceSerial,
    uint8_t mode)
{
    packetToAssemble->id = ConstructCANID(PRIO_MOTOR_UNIT_MODE_SEL, targetDeviceGroup, targetDeviceSerial);
    packetToAssemble->dlc = DLC_MOTOR_UNIT_MODE_SEL;
    int nextByte = WritePacketIDOnly(packetToAssemble->data, ID_MOTOR_UNIT_MODE_SEL);
    packetToAssemble->data[nextByte] = mode;
}

uint8_t GetModeFromPacket(CANPacket *packet)
{
    return packet->data[1];
}


void AssemblePWMDirSetPacket(CANPacket *packetToAssemble, 
    uint8_t targetDeviceGroup,
    uint8_t targetDeviceSerial,
    int32_t PWMSet)
{
    packetToAssemble->id = ConstructCANID(PRIO_MOTOR_UNIT_PWM_DIR_SET, targetDeviceGroup, targetDeviceSerial);
    packetToAssemble->dlc = DLC_MOTOR_UNIT_PWM_DIR_SET;
    int nextByte = WritePacketIDOnly(packetToAssemble->data, DLC_MOTOR_UNIT_PWM_DIR_SET);
    PackShortIntoDataMSBFirst(packetToAssemble->data, PWMSet, nextByte);
}

int32_t GetPWMFromPacket(CANPacket *packet)
{
    return DecodeBytesToIntMSBFirst(packet->data, 1, 2);
}

//Returns 2's compliment MSB (0 for stopped or forward, 1 for reverse)
int32_t GetDirectionFromPacket(CANPacket *packet)
{
    return ((packet->data[1]) >> 7) & 0x1;
}

void AssemblePIDTargetSetPacket(CANPacket *packetToAssemble,
    uint8_t targetDeviceGroup,
    uint8_t targetDeviceSerial,
    int32_t target)
{
    packetToAssemble->id = ConstructCANID(PACKET_PRIORITY_NORMAL, targetDeviceGroup, targetDeviceSerial);
    packetToAssemble->dlc = DLC_MOTOR_UNIT_PID_POS_TGT_SET;
    int nextByte = WritePacketIDOnly(packetToAssemble->data, ID_MOTOR_UNIT_PID_POS_TGT_SET);
    PackIntIntoDataMSBFirst(packetToAssemble->data, target, nextByte);
}

uint8_t GetPIDTargetFromPacket(CANPacket *packet)
{
    return DecodeBytesToIntMSBFirst(packet->data, DLC_MOTOR_UNIT_PID_POS_TGT_SET - 4, DLC_MOTOR_UNIT_PID_POS_TGT_SET);
}

void AssemblePSetPacket(CANPacket *packetToAssemble,
    uint8_t targetDeviceGroup,
    uint8_t targetDeviceSerial,
    int32_t pCoef)
{
    packetToAssemble->id = ConstructCANID(PACKET_PRIORITY_NORMAL, targetDeviceGroup, targetDeviceSerial);
    packetToAssemble->dlc = DLC_MOTOR_UNIT_PID_P_SET;
    int nextByte = WritePacketIDOnly(packetToAssemble->data, ID_MOTOR_UNIT_PID_P_SET);
    PackIntIntoDataMSBFirst(packetToAssemble->data, pCoef, nextByte);
}

int32_t GetPFromPacket(CANPacket *packet)
{
    return DecodeBytesToIntMSBFirst(packet->data, DLC_MOTOR_UNIT_PID_P_SET - 4, DLC_MOTOR_UNIT_PID_P_SET );
}

void AssembleISetPacket(CANPacket *packetToAssemble,
    uint8_t targetDeviceGroup,
    uint8_t targetDeviceSerial,
    int32_t iCoef)
{
    packetToAssemble->id = ConstructCANID(PACKET_PRIORITY_NORMAL, targetDeviceGroup, targetDeviceSerial);
    packetToAssemble->dlc = DLC_MOTOR_UNIT_PID_I_SET;
    int nextByte = WritePacketIDOnly(packetToAssemble->data, ID_MOTOR_UNIT_PID_I_SET);
    PackIntIntoDataMSBFirst(packetToAssemble->data, iCoef, nextByte);
}

int32_t GetIFromPacket(CANPacket *packet)
{
    return DecodeBytesToIntMSBFirst(packet->data, DLC_MOTOR_UNIT_PID_I_SET - 4, DLC_MOTOR_UNIT_PID_I_SET );
}

void AssembleDSetPacket(CANPacket *packetToAssemble,
    uint8_t targetDeviceGroup,
    uint8_t targetDeviceSerial,
    int32_t dCoef)
{
    packetToAssemble->id = ConstructCANID(PACKET_PRIORITY_NORMAL, targetDeviceGroup, targetDeviceSerial);
    packetToAssemble->dlc = DLC_MOTOR_UNIT_PID_D_SET;
    int nextByte = WritePacketIDOnly(packetToAssemble->data, ID_MOTOR_UNIT_PID_D_SET);
    PackIntIntoDataMSBFirst(packetToAssemble->data, dCoef, nextByte);
}

int32_t GetDFromPacket(CANPacket *packet)
{
    return DecodeBytesToIntMSBFirst(packet->data, DLC_MOTOR_UNIT_PID_D_SET - 4, DLC_MOTOR_UNIT_PID_D_SET);
}

void AssembleInitializePacket(CANPacket *packetToAssemble,
    uint8_t targetDeviceGroup,
    uint8_t targetDeviceSerial,
    uint8_t initMode)
{
    packetToAssemble->id = ConstructCANID(PRIO_MOTOR_UNIT_INIT, targetDeviceGroup, targetDeviceSerial);
    packetToAssemble->dlc = DLC_MOTOR_UNIT_INIT;
    int nextByte = WritePacketIDOnly(packetToAssemble->data, ID_MOTOR_UNIT_INIT);
    packetToAssemble->data[nextByte] = initMode;
}
uint8_t GetInitModeFromPacket(CANPacket *packet)
{
    return packet->data[1];
}

void AssembleLimitSwitchAlertPacket(CANPacket *packetToAssemble,
    uint8_t targetDeviceGroup,
    uint8_t targetDeviceSerial,
    uint8_t switches)
{
    packetToAssemble->id = ConstructCANID(PRIO_MOTOR_UNIT_LIM_ALERT, targetDeviceGroup, targetDeviceSerial);
    packetToAssemble->dlc = DLC_MOTOR_UNIT_LIM_ALERT;
    int nextByte = WriteSenderSerialAndPacketID(packetToAssemble->data, ID_MOTOR_UNIT_LIM_ALERT);
    packetToAssemble->data[nextByte] = switches; 
}
uint8_t GetLimStatusFromPacket(CANPacket *packet)
{
    return packet->data[1];
}

void AssembleEncoderPPJRSetPacket(CANPacket *packetToAssemble,
    uint8_t targetDeviceGroup,
    uint8_t targetDeviceSerial,
    uint32_t pulses)
{
    packetToAssemble->id = ConstructCANID(PRIO_MOTOR_UNIT_ENC_PPJR_SET, targetDeviceGroup, targetDeviceSerial);
    packetToAssemble->dlc = DLC_MOTOR_UNIT_ENC_PPJR_SET;
    int nextByte = WritePacketIDOnly(packetToAssemble->data, ID_MOTOR_UNIT_ENC_PPJR_SET);
    PackIntIntoDataMSBFirst(packetToAssemble->data, pulses, nextByte);
}

uint32_t GetEncoderPPJRFromPacket(CANPacket *packet)
{
    return DecodeBytesToIntMSBFirst(packet->data, 1, 4);
}

void AssembleMaxJointRevolutionPacket(CANPacket *packetToAssemble,
    uint8_t targetDeviceGroup,
    uint8_t targetDeviceSerial,
    uint32_t revolutions)
{
    packetToAssemble->id = ConstructCANID(PRIO_MOTOR_UNIT_MAX_JNT_REV_SET, targetDeviceGroup, targetDeviceSerial);
    packetToAssemble->id = DLC_MOTOR_UNIT_MAX_JNT_REV_SET;
    int nextByte = WritePacketIDOnly(packetToAssemble->data, ID_MOTOR_UNIT_MAX_JNT_REV_SET);
    PackIntIntoDataMSBFirst(packetToAssemble->data, revolutions, nextByte);
    
}
uint32_t GetMaxJointRevolutionsFromPacket(CANPacket *packet)
{
    return DecodeBytesToIntMSBFirst(packet->data, 1, 4);
}
