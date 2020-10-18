#ifndef CAN_H
#define CAN_H

#define CAN_1000_BAUD 0x020413L
#define CAN_500_BAUD 0x060413L
#define CAN_250_BAUD 0x0E0413L
#define CAN_125_BAUD 0x1E0413L
//#define CAN_100_BAUD 0x080C37
#define CAN_100_BAUD 0x260413L

#define CAN_RTR 1
#define CAN_LISTEN 1

struct CAN_msg {
	uint16_t id;
	uint8_t flags;
	uint8_t length;
	uint8_t data[8];
};

void init_CAN(uint32_t rate, uint8_t txmobs, uint8_t mode);
uint8_t get_dip_switch();
void select_mob(uint8_t mob);
int8_t find_free_mob();
uint8_t CAN_msg_available();
uint8_t CAN_get_msg(struct CAN_msg *buf);
uint8_t CAN_send_msg(struct CAN_msg *buf);
uint8_t CAN_msg_available();
void enable_mob_interrupt(uint8_t mob);
void disable_mob_interrupt(uint8_t mob);
void CAN_set_RX_filter(uint16_t mask, uint16_t tag);
void CAN_dump_state();
void dump_CAN_message(struct CAN_msg m);

#endif