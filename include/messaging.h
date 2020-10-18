#ifndef MESSAGING_H
#define MESSAGING_H

#include "CANPacket.h"

void handle_CAN_message(CANPacket *m);
int send_CAN_message(uint8_t target, uint8_t length, void *buffer, uint8_t priority);
void send_model_number(uint8_t target);
void send_int8_packet(uint8_t target, uint8_t pn, uint8_t b, uint8_t priority);
void send_int16_packet(uint8_t target, uint8_t pn, uint16_t n, uint8_t priority);
void send_int32_packet(uint8_t target, uint8_t pn, uint32_t n, uint8_t priority);
void send_CAN_error(uint8_t error, uint8_t param);
void send_encoder_count();
void send_telemetry();

#endif