#ifndef CONFIG_H
#define CONFIG_H

#define F_CPU 16000000L

#define VS_PIN 1

#define MODEL_NUMBER 0x02 //Change this to change the model number
#define BEAGLEBONE_ADDRESS 0x02 //CAN address of the BBB

#define MOTOR_PWM_PORT PORTE
#define MOTOR_PWM_DDR DDRE
#define MOTOR_PWM PE3
#define MOTOR_DIR_PORT PORTE
#define MOTOR_DIR_DDR DDRE
#define MOTOR_DIR PE2
#define MOTOR_CS_DDR DDRF //Motor current sense mode register
#define MOTOR_CS 0 //Motor current sense pin

#define DEFAULT_MOTOR_CURRENT_LIMIT 5000 //5A
#define DEFAULT_MOTOR_VELOCITY 250 //250 ticks/sec

#define MOTOR_SET_TIMEOUT 1250 //number of milliseconds until motor is automatically disabled after the last motor set

#define DEFAULT_Kp_1 140 //Default Kp for position PID
#define DEFAULT_Ki_1 20  //Default Ki for position PID
#define DEFAULT_Kd_1 100 //Default Kd for position PID

//#define DUAL_PID //Define to use dual PID mode. Otherwise uses single PID mode

#define DEFAULT_Kp_2 1 //Default Kp for velocity PID
#define DEFAULT_Ki_2 1 //Default Ki for velocity PID
#define DEFAULT_Kd_2 1 //Default Kd for velocity PID

#define USART_TX_BUF_SZ 16 //UART TX buffer size
#define USART_RX_BUF_SZ 16 //UART RX buffer size

//#define L298_MC //Use L298 motor driver code for testing

#define CAN_ERROR_OVERCURRENT 1
#define CAN_ERROR_COMMAN_FAILED 2
#define CAN_ERROR_INVALID_ARGUMENT 3

#ifndef DEBUG

//#define DEBUG //Debug flag

#endif

//Define REV_2 to use Revision 2 firmware
#define REV_2

#endif