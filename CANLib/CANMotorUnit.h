/* File:         CANMotorUnit.h
 * Authors:      Jaden Bottemiller, Benton Kwong, Dylan Tomberlin.
 * Organization: Husky Robotics Team
 * 
 * This file includes fuction prototypes for CAN Packet manipulation
 * using the Hindsight CAN Communication standard. Specific files
 * for the motor unit boards.
 * Documentation: https://huskyroboticsteam.slite.com/app/channels/iU0BryG7M9/collections/aXvWTcIR6c/notes/4otlSFsSp2
 */

#include "CANPacket.h"

// TODO: Add parameters to packet assembly
//Mode set (PWM or PID)
void AssembleModeSetPacket(CANPacket *packetToAssemble,
    uint8_t targetDeviceGroup,
    uint8_t targetDeviceSerial,
    uint8_t mode);
uint8_t GetModeFromPacket(CANPacket *packet);

//PWM value and direction set
void AssemblePWMDirSetPacket(CANPacket *packetToAssemble, 
    uint8_t targetDeviceGroup,
    uint8_t targetDeviceSerial,
    int32_t PWMSet);
int32_t GetPWMFromPacket(CANPacket *packet);
int32_t GetDirectionFromPacket(CANPacket *packet);

//PID postional target set
void AssemblePIDTargetSetPacket(CANPacket *packetToAssemble,
    uint8_t targetDeviceGroup,
    uint8_t targetDeviceSerial,
    int32_t target);
uint8_t GetPIDTargetFromPacket(CANPacket *packet);

//P coeffiecent set
void AssemblePSetPacket(CANPacket *packetToAssemble,
    uint8_t targetDeviceGroup,
    uint8_t targetDeviceSerial,
    int32_t pCoef);
int32_t GetPFromPacket(CANPacket *packet);

//I coeffiecent set
void AssembleISetPacket(CANPacket *packetToAssemble,
    uint8_t targetDeviceGroup,
    uint8_t targetDeviceSerial,
    int32_t iCoef);
int32_t GetIFromPacket(CANPacket *packet);

//D coeffiecent set
void AssembleDSetPacket(CANPacket *packetToAssemble,
    uint8_t targetDeviceGroup,
    uint8_t targetDeviceSerial,
    int32_t dCoef);
int32_t GetDFromPacket(CANPacket *packet);

//Initialize with mode (motors shall not move until have been inited)
void AssembleInitializePacket(CANPacket *packetToAssemble,
    uint8_t targetDeviceGroup,
    uint8_t targetDeviceSerial,
    uint8_t initMode);
uint8_t GetInitModeFromPacket(CANPacket *packet);

//Limit switch alert 
//each bit represents one limit switch, 1 for closed, 0 for open,
//switch number corresponds to the bit number
void AssembleLimitSwitchAlertPacket(CANPacket *packetToAssemble,
    uint8_t targetDeviceGroup,
    uint8_t targetDeviceSerial,
    uint8_t switches);
uint8_t GetLimStatusFromPacket(CANPacket *packet);

//Encoder pulses per joint revolution set
void AssembleEncoderPPJRSetPacket(CANPacket *packetToAssemble,
    uint8_t targetDeviceGroup,
    uint8_t targetDeviceSerial,
    uint32_t pulses);
uint32_t GetEncoderPPJRFromPacket(CANPacket *packet);

//Maximum joint rotations set
void AssembleMaxJointRevolutionPacket(CANPacket *packetToAssemble,
    uint8_t targetDeviceGroup,
    uint8_t targetDeviceSerial,
    uint32_t revolutions);
uint32_t GetMaxJointRevolutionsFromPacket(CANPacket *packet);

// Motor Unit Packet IDs
#define ID_MOTOR_UNIT_MODE_SEL          (uint8_t) 0x00
#define ID_MOTOR_UNIT_PWM_DIR_SET       (uint8_t) 0x03
#define ID_MOTOR_UNIT_PID_POS_TGT_SET   (uint8_t) 0x04
#define ID_MOTOR_UNIT_PID_P_SET         (uint8_t) 0x05
#define ID_MOTOR_UNIT_PID_I_SET         (uint8_t) 0x06
#define ID_MOTOR_UNIT_PID_D_SET         (uint8_t) 0x07
#define ID_MOTOR_UNIT_INIT              (uint8_t) 0x08
#define ID_MOTOR_UNIT_LIM_ALERT         (uint8_t) 0x09
#define ID_MOTOR_UNIT_ENC_PPJR_SET      (uint8_t) 0x0A
#define ID_MOTOR_UNIT_MAX_JNT_REV_SET   (uint8_t) 0x0B

// Packet DLCs
#define DLC_MOTOR_UNIT_MODE_SEL          (uint8_t) 0x02
#define DLC_MOTOR_UNIT_PWM_DIR_SET       (uint8_t) 0x05
#define DLC_MOTOR_UNIT_PID_POS_TGT_SET   (uint8_t) 0x05
#define DLC_MOTOR_UNIT_PID_P_SET         (uint8_t) 0x05
#define DLC_MOTOR_UNIT_PID_I_SET         (uint8_t) 0x05
#define DLC_MOTOR_UNIT_PID_D_SET         (uint8_t) 0x05
#define DLC_MOTOR_UNIT_INIT              (uint8_t) 0x02
#define DLC_MOTOR_UNIT_LIM_ALERT          (uint8_t) 0x03
#define DLC_MOTOR_UNIT_ENC_PPJR_SET      (uint8_t) 0x05
#define DLC_MOTOR_UNIT_MAX_JNT_REV_SET   (uint8_t) 0x02

//Packet priorities
#define PRIO_MOTOR_UNIT_MODE_SEL          PACKET_PRIORITY_NORMAL
#define PRIO_MOTOR_UNIT_PWM_DIR_SET       PACKET_PRIORITY_NORMAL
#define PRIO_MOTOR_UNIT_PID_POS_TGT_SET   PACKET_PRIORITY_NORMAL
#define PRIO_MOTOR_UNIT_PID_P_SET         PACKET_PRIORITY_NORMAL
#define PRIO_MOTOR_UNIT_PID_I_SET         PACKET_PRIORITY_NORMAL
#define PRIO_MOTOR_UNIT_PID_D_SET         PACKET_PRIORITY_NORMAL
#define PRIO_MOTOR_UNIT_INIT              PACKET_PRIORITY_NORMAL
#define PRIO_MOTOR_UNIT_LIM_ALERT          PACKET_PRIORITY_HIGH
#define PRIO_MOTOR_UNIT_ENC_PPJR_SET      PACKET_PRIORITY_NORMAL
#define PRIO_MOTOR_UNIT_MAX_JNT_REV_SET   PACKET_PRIORITY_NORMAL

// Motor Unit Mode IDs
#define MOTOR_UNIT_MODE_PWM             (uint8_t) 0x00
#define MOTOR_UNIT_MODE_PID             (uint8_t) 0x01
