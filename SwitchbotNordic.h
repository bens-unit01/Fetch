/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */
#ifndef REV
#define REV
//--
#include "nrf_gpio.h"

#define LED_left          20
#define LED_head	 24
#define LED_right   	 22// 16//22
#define LED_tail         23 // 17 //23

#define	RGB_RED 	24 //	8
#define RGB_GREEN 	24 //	7
#define RGB_BLUE 	24 //	6
#define Switch 		24 //	5

//jason board
 #define	IRM_head						19					//IRM receive head
 #define	IRM_tail						24					//IRM receive tail
 #define	IRM_left						16					//IRM receive left
 #define	IRM_right						21					//IRM receive right


//new board
//#define	IRM_right						21					//IRM receive right
//#define	IRM_tail						19					//IRM receive tail
//#define	IRM_left						16					//IRM receive left
//#define	IRM_head						2					//IRM receive head

//#define	IRM_down						3					//IRM receive pointed down
//#define	IRM_forward						1					//IRM receive forward

//  --- rx-tx for dev board
//#define RX_PIN_NUMBER  16//16    // UART RX pin number.
//#define TX_PIN_NUMBER  17//8//6//17    // UART TX pin number.
//------------------------------------

// --- rx-tx for SwitchBot ----
#define RX_PIN_NUMBER  22 // 16 //16    // UART RX pin number.
#define TX_PIN_NUMBER  23 //17 //8//6//17    // UART TX pin number.
//------------------------------
#define CTS_PIN_NUMBER  18    // UART Clear To Send pin number. Not used if HWFC is set to false
#define RTS_PIN_NUMBER  25//19    // Not used if HWFC is set to false
#define HWFC           false // UART hardware flow control


/* yes, thrust is inverted */
#define THRUST_MIN	0xe1
#define THRUST_MAX	0x00

#define YAW_MIN		0x00
#define YAW_MAX		0xe1
#define PITCH_MIN	0x00
#define PITCH_MAX	0xe1
#define ROLL_MIN	0x00
#define ROLL_MAX	0xe1

//   dog settings
#define PWMA1          6 //0
#define AIN1           8 //2
#define AIN2           7 //1

#define PWMB1          11 //5
#define BIN1           9  //3
#define BIN2           10 //4

#define PWMA2          0 //6
#define AIN3           2 //8
#define AIN4           1 //7


#define PWMB2          5 //11
#define BIN3           3 //9
#define BIN4           4 //10

#define STDBY          15



// setup for the dev board
/*
#define PWMA1          LED_0
#define AIN1           24
#define AIN2           24

#define PWMB1          LED_1
#define BIN1           24
#define BIN2           24

#define PWMA2          LED_2
#define AIN3           24 //8
#define AIN4           24


#define PWMB2          LED_3
#define BIN3           24
#define BIN4           24


#define STDBY          LED_4

*/
struct qr_cmd {
	uint16_t thrust;					/* left joystick, up/down */
	uint16_t yaw;					/* left joystick, left/right */
	uint16_t pitch;					/* right joystick, up/down */
	uint16_t roll;					/* right joystick, left/right */
	uint16_t aux1;
	uint16_t aux2;
	uint16_t aux3;
	uint16_t aux4;

};


#define NOTF_GET_STATUS 0x51
#define NOTF_SET_STATUS 0x52
#define NOTF_ACTIVATE_ADB 0x53
#define STAND_UP        0x61
#define KNEEL               0x62
#define LEAN            0x63
#define ESTOP           0x65
#define CLEAR_ESTOP     0x66
#define DRIVE_TEST      0x79
#define DRIVE_3COM      0xf0
#endif  // REV
