#include "config.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <usart.h>
#include "can.h"
#include "timers.h"

volatile uint8_t msgs_av; //Number of messages unclaimed messages

uint16_t RX_mask, RX_tag; //Recieve mask and tag bits

volatile uint8_t rxed_mobs[2]; //Tracks which MObs have messages recieved

ISR(CANIT_vect){
	uint8_t canpage = CANPAGE; //Save CAN page
	if((CANHPMOB & 0xF0) != 0xF0){ //Message io?
		int mob = (CANHPMOB >> 4);
		select_mob(mob);
		if(CANSTMOB & (1 << TXOK)){ //TX
			/*Reset the MOb*/
			CANSTMOB &= 0;
			CANCDMOB = 0;
			enable_mob_interrupt(mob);
			} else { //RX
			msgs_av++; //Increase count of messages
			rxed_mobs[!!(mob & 8)] |= (1 << (mob & 7)); // Mark which MOb has a message
			CANSTMOB &= 0; //Reset the MOb
			disable_mob_interrupt(mob);
		}
		} else {
		CANGIT |= 0; //Error interrupt - Handle these?
	}
	CANPAGE = canpage; //restore CAN page
}

/*Initalizes and enables the CAN controller
Parameters:
uint32_t rate: the baud rate selection
uint8_t txmobs: how many MOBs to dedicate to transmission
uint8_t mode: The mode to operate the CAN controller in
*/
void init_CAN(uint32_t rate, uint8_t txmobs, uint8_t mode){
	CANGCON |= (1<<SWRES);
	delay_mS(100); //Needed for unknown reason?!
	CANBT1 = (uint32_t)(rate & 0xFF0000L) >> 16;
	CANBT2 = (uint32_t)(rate & 0x00FF00L) >> 8;
	CANBT3 = (uint32_t)(rate & 0x0000FFL);
	CANGIE = (1 << CANIT) | (1 << ENRX) | (1 << ENTX); //Enable CAN interrupts
	CANTCON = 255; //Set the can timer to run at 1/2048th of F_CPU
	rxed_mobs[0] = rxed_mobs[1] = 0;
	uint8_t i;
	/*Initialize MOBs*/
	for(i = 0;i < 15;i++){
		select_mob(i);
		CANSTMOB &= 0;
		CANIDM4 = 0;
		CANIDM3 = 0;
		CANIDT4 = 0;
		CANIDT3 = 0;
		CANIDT2 = ((RX_tag & 7) << 5);
		CANIDT1 = ((RX_tag & 0x7F8) >> 3);
		CANIDM2 = ((RX_mask & 7) << 5);
		CANIDM1 = ((RX_mask & 0x7F8) >> 3);
		if(i > txmobs){
			CANCDMOB = (1 << CONMOB1); //Mark RX mobs
			enable_mob_interrupt(i);
			} else {
			CANCDMOB = 0;
		}
	}
	msgs_av = 0;
	//Enable the CAN controller
	CANGCON = (1 << ENASTB);
	if(mode & CAN_LISTEN){
		CANGCON |= (1<<LISTEN);
	}
}

/*Returns the value of the binary representation of the dipswitch*/
uint8_t get_dip_switch(){
	return (~PINA) & 15;
}

/*Selects the MOB to operate on*/
void select_mob(uint8_t mob){
	CANPAGE = ((mob & 0x0F) << 4);
}

/*Disables the interrupt for the specified MOb*/
void disable_mob_interrupt(uint8_t mob){
	if(mob < 8){
		CANIE2 &= ~(1 << mob);
		} else {
		CANIE1 &= ~(1 << (mob - 8));
	}
}

/*Enable the interrupt for the specified MOb*/
void enable_mob_interrupt(uint8_t mob){
	if(mob < 8){
		CANIE2 |= (1 << mob);
		} else {
		CANIE1 |= (1 << (mob - 8));
	}
}

/*Returns whether the interrupt for the specified MOb is enabled*/
int mob_interrupt_enabled(uint8_t mob){
	if(mob < 8){
		return !!(CANIE2 & (1 << mob));
		} else {
		return !!(CANIE1 & (1 << (mob - 8)));
	}
}

/*Returns whether a specified MOb is marked as enabled*/
uint8_t mob_enabled(uint8_t mob){
	if(mob < 8){
		return !!(CANEN2 & (1 << mob));
		} else {
		return !!(CANEN1 & (1 << (mob - 8)));
	}
}

/*Returns the number of CAN s waiting*/
uint8_t CAN_msg_available(){
	return msgs_av;
}

/*Copies a message received on the CAN bus to buf*/
uint8_t CAN_get_msg(struct CAN_msg *buf){
	uint8_t i, mob;
	if(!CAN_msg_available()) return 0;
	/*Find a MOb with a message*/
	for(i = 0;i < 16;i++){
		if(i == 15) return 0;
		select_mob(i);
		if(rxed_mobs[!!(i & 8)] & (1 << (i & 7))){
			break;
		}
	}
	mob = i;
	//tprintf("rx mob=%d\n", mob);
	buf->length = CANCDMOB & 0x0F; //Length in the lower 8 bits
	buf->id = (CANIDT2 >> 5) | ((uint16_t)CANIDT1 << 3);
	if(CANIDT4 & (1<<RTRTAG)){ //Was this a Request To Transmit?
		buf->flags |= CAN_RTR;
		} else {
		buf->flags &= ~CAN_RTR;
	}
	for(i = 0;i < buf->length;i++){
		buf->data[i] = CANMSG; //Get the data from the MOb and copy it into the buffer
	}
	//Atomically decrement the number of messages available
	cli();
	msgs_av--;
	sei();
	/*Reset the MOb*/
	CANIDT4 = 0;
	CANIDT3 = 0;
	CANIDT2 = (RX_tag & 0x07) << 5;
	CANIDT1 = (RX_tag & 0x7F8) >> 3;
	enable_mob_interrupt(mob);
	rxed_mobs[!!(mob & 8)] &= ~(1 << (mob & 7)); //Mark that the message has been taken
	CANCDMOB = (1<<CONMOB1); //Re-enable recieve
	return 1;
}

/*Finds a free MOb or returns -1 if they're all used*/
int8_t find_free_mob(){
	uint8_t i;
	uint8_t status;
	for(i = 0;i < 15;i++){
		select_mob(i);
		status = CANCDMOB;
		//tprintf("mob %d: CONMOB1:%d, CONMOB0:%d\n", i, !!(status & (1 << CONMOB1)), !!(status & (1 << CONMOB0)));
		if(!(status & ((1 << CONMOB1) | (1 << CONMOB0)))){
			return i;
		}
		//if(status & (1 << CONMOB0) && !mob_enabled(i)){
		//}
	}
	return -1;
}

/*Send a message over the can bus*/
uint8_t CAN_send_msg(struct CAN_msg *buf){
	uint8_t i;
	int8_t mob = find_free_mob();
	//tprintf("free mob=%d\n", mob);
	if(mob == -1){
		return 0; //No free MObs
	}
	select_mob(mob);
	CANSTMOB &= 0;
	CANCDMOB = buf->length & 0x0F;
	for(i = 0;i < buf->length;i++){ //Copy the data into the MOb
		CANMSG = buf->data[i];
	}
	CANIDT4 = 0; //CAN v2.0 - we don't care
	CANIDT3 = (buf->flags & CAN_RTR)? (1<<RTRTAG): 0; //Sending an RTR?
	CANIDT2 = ((buf->id & 7) << 5);
	CANIDT1 = ((buf->id & 0x7F8) >> 3);
	CANCDMOB |= (1<<CONMOB0);
	enable_mob_interrupt(mob); //Enable MOb interrupt
	return 1;
}

/*Set the RX filter mask and tag. Updates current RX mobs and future RX MObs*/
void CAN_set_RX_filter(uint16_t mask, uint16_t tag){
	RX_mask = mask;
	RX_tag = tag;
	int i;
	for(i = 0;i < 15;i++){
		/*Only modify receive mobs which are enabled and not occupied*/
		if(mob_enabled(i)){
			select_mob(i);
			if(CANCDMOB & (1<<CONMOB1)){
				CANIDT2 = ((tag & 7) << 5);
				CANIDT1 = ((tag & 0x7F8) >> 3);
				CANIDM2 = ((mask & 7) << 5);
				CANIDM1 = ((mask & 0x7F8) >> 3);
			}
		}
	}
}
/*List of CAN status code names*/
const char *canstmob_names[] = {"ACK ERR", "FORM ERR", "CRC ERR", "STF ERR", "BIT ERR", "RXOK", "TXOK", "DLCW"};
/*Dumps the state of the CAN controller (for debugging)*/
void CAN_dump_state(){
	int i,j;
	for(i = 0;i < 15;i++){
		select_mob(i);
		uint32_t canstmob = CANSTMOB;
		uint32_t cancdmob = CANCDMOB;
		tprintf("mob %d: (%c), I=%c, IDT=%d, IDM=%d, DLC=%d, CANST=0x%X", i, mob_enabled(i)? 'E': 'D', mob_interrupt_enabled(i)? 'E': 'D', (uint16_t)((CANIDT2 >> 5) | ((uint16_t)CANIDT1 << 3)), (uint16_t)((CANIDM2 >> 5) | ((uint16_t)CANIDM1 << 3)), (int)(cancdmob & 15), canstmob);
		if(canstmob != 0){
			tprintf("{");
				for(j = 0;j < 8;j++){
					if(canstmob & (1<<j)){
						tprintf("%s ", canstmob_names[j]);
					}
				}
			tprintf("}");
		}
		tprintf(", CANCD=0x%X {", (long)cancdmob);
			if(cancdmob & (1<<5)){
				tprintf("RPLV ");
			}
			if(cancdmob & (1<<4)){
				tprintf("2.0B ");
			}
			switch((cancdmob & 0xC0)>>6){
				case 0:
				tprintf("DIS");
				break;
				case 1:
				tprintf("ENTX");
				break;
				case 2:
				tprintf("ENRX");
				break;
				case 3:
				tprintf("FBRX");
				break;
			}
		tprintf("}\n");
	}
}

void dump_CAN_message(struct CAN_msg m){
	int i;
	tprintf("\n-CAN MESSAGE-\n");
	tprintf("id=0x%X, flags=0x%X, length=%d\n", (long)m.id, (long)m.flags, m.length);
	tprintf("HEX={");
		for(i = 0;i < m.length;i++){
			tprintf("%X%s", (long)m.data[i], i == m.length-1?"": " ");
		}
		tprintf("}, ASCII={");
		for(i = 0;i < m.length;i++){
			tprintf("%c", (m.data[i] > 31 && m.data[i] < 128)?m.data[i]: '.');
		}
	tprintf("}\n-END CAN MESSAGE-\n\n");
}