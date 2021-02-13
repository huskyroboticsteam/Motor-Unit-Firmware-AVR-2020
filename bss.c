/*
 * bss.c
 *
 * Created: 5/24/2019 9:51:29 PM
 *  Author: John
 */

#include <stdint.h>
#include "config.h"
#include "bss.h"
#include "servo.h"
#include "motor.h"
#include "adc.h"
#include "util.h"
#include "usart.h"
#include "timers.h"

/*Tries to detect the board revision
Return:
1 if it is likely a revision 1 board or 2 if it is a rev. 2 board
*/
uint8_t detect_board(){
	internalAREF();
	uint16_t a = read_ADC(0);
	if(a > 384 && a < 640){
		return 2;
	}
	return 1;
}

/*Does setup specific to the board
  Parameters:
  the CAN address of the board
*/
void do_board_specific_setup(uint8_t addr){
		set_LED(3, 3);
		set_Kp(120, 0);
		set_Ki(20, 0);
		set_Kd(170, 0);
		switch(addr){
			case 0x0: //Base rotation:
				set_ticks_per_10degrees(172);
				break;
			case 0x2: //Elbow
				set_motor_reverse(1);
				set_ticks_per_10degrees(85);
				break;
			case 0x3: //Forearm rotation
				set_ticks_per_10degrees(68);
				break;
			case 0x6: //Hand
				init_servo();
				set_servo_position(0);
				break;
		}
		#ifdef REV_2
		tprintf("REV. 2 firmware\n");
		#endif
		uint8_t rev = detect_board();
		tprintf("Detecting board rev: %d\n", rev);
		#ifdef REV_2
		if(rev != 2){
			tprintf("***WARNING*** Rev. 2 firmware on Rev. 1 board\n");
			set_LED(0, 3);
			delay_mS(1000);
		}
		#else
		if(rev == 1){
			tprintf("***WARNING*** Rev. 1 firmware on Rev. 2 board\n");
			set_LED(0, 3);
			delay_mS(1000);
		}
		#endif
}
