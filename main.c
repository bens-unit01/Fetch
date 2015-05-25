#define NEVER_SLEEP
//#define BOARD_NRF6310
#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_gpio.h"

#include "nrf_delay.h"
#include "nrf51_bitfields.h"

#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "app_button.h"
#include "ble_nus.h"

#include "ble_error_log.h"
#include "ble_debug_assert_handler.h"

#include "ble_app.h"

#include "SwitchbotNordic.h"
//#include "app_error.h"
//#include "ble.h"
#include "app_gpiote.h"

#include "simple_uart.h"
#include "boards.h"
#include "SEGGER_RTT.h"
#include "math.h"


//#define ADVERTISING_LED_PIN_NO          LED_1                                       /**< LED to indicate advertising state. */
//#define CONNECTED_LED_PIN_NO            LED_2                                       /**< LED to indicate connected state. */
#define BUTTON_PULL    					NRF_GPIO_PIN_NOPULL


#define BUTTON_0            			30                           /**< LED to indicate advertising state. */
#define BUTTON_1			            0                                       /**< LED to indicate connected state. */


//#define DEVICE_NAME			 	"Fetch-02"					/* Name of device. Will be included in the advertising data. */
#define DEVICE_NAME			 	"Chip-01"					/* Name of device. Will be included in the advertising data. */
//#define DEVICE_NAME			 	"Fetch-02"					/* Name of device. Will be included in the advertising data. */


#define APP_ADV_INTERVAL				64						/* The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS		180						/* The advertising timeout (in units of seconds). */

#define APP_TIMER_PRESCALER				0						/* Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS			4						/* Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE			4						/* Size of timer operation queues. */

#define MIN_CONN_INTERVAL				MSEC_TO_UNITS(7.5, UNIT_1_25_MS)		/* Minimum acceptable connection interval (7.5ms). */
#define MAX_CONN_INTERVAL				MSEC_TO_UNITS(40, UNIT_1_25_MS)			/* Maximum acceptable connection interval (40ms). */
#define SLAVE_LATENCY					0						/* Slave latency. */
#define CONN_SUP_TIMEOUT				MSEC_TO_UNITS(300, UNIT_10_MS)			/* Connection supervisory timeout (300ms). */
#define FIRST_CONN_PARAMS_UPDATE_DELAY	APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER)	/* Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (1 second). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY	APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)	/* Time between each call to sd_ble_gap_conn_param_update after the first call (5 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT	3						/* Number of attempts before giving up the connection parameter negotiation. */

#define BATTERY_LEVEL_MEAS_INTERVAL		APP_TIMER_TICKS(2000, APP_TIMER_PRESCALER)	/* Battery level measurement interval (ticks). */

#define APP_GPIOTE_MAX_USERS			1						/* Maximum number of users of the GPIOTE handler. */

#define BUTTON_DETECTION_DELAY			APP_TIMER_TICKS(50, APP_TIMER_PRESCALER)	/* Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define SEC_PARAM_TIMEOUT				30						/* Timeout for Pairing Request or Security Request (in seconds). */
#define SEC_PARAM_BOND					1						/* Perform bonding. */
#define SEC_PARAM_MITM					0						/* Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES		BLE_GAP_IO_CAPS_NONE				/* No I/O capabilities. */
#define SEC_PARAM_OOB					0						/* Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE			7						/* Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE			16						/* Maximum encryption key size. */
//--
#define DEAD_BEEF						0xDEADBEEF					/* Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

static ble_gap_sec_params_t 			m_sec_params;			/* Security requirements for this application. */
static uint16_t 						m_conn_handle = BLE_CONN_HANDLE_INVALID;	/* Handle of the current connection. */

static ble_nus_t                        m_nus;  
static ble_app_t 						app;                                    /**< Structure to identify the Nordic UART Service. */

uint8_t									r1,r2,r3,r4;
uint8_t									rangeAll,rangeOld;
uint8_t									intCount,perCount;
uint8_t									s1,s2,s3,s4;
uint8_t									IR[4];
int 									Head, Range, easyRange;

bool 									fNewData = false;
bool 									fDecoy = false;

uint16_t									temp, noDataCount;

char                                    Tir1_string[4];
int16_t									fwd_bwd;
int16_t 								lft_rgt;
uint8_t									command;


static uint8_t                          UART_data_index = 3;

static app_timer_id_t                   m_pwm_timer_id;                       /**< PWM timer. */
struct qr_cmd cur_cmd;

//static bool charging;									/* true if we're charging */

app_timer_id_t tap_tid;									/* take a pic timer id */
app_timer_id_t cind_tid;								/* charge indicator timer id */
app_timer_id_t batt_tid;								/* battery timer ID */

#define SCHED_MAX_EVENT_DATA_SIZE	sizeof(app_timer_event_t)			/* Maximum size of scheduler events. Note that scheduler BLE stack events do not contain any data, as the events are being pulled from the stack in the event handler. */
#define SCHED_QUEUE_SIZE		10						/* Maximum number of events in the scheduler queue. */

// Persistent storage system event handler
void pstorage_sys_event_handler (uint32_t p_evt);

static void receiver_IRM_coord(void);
void ParseCommandUART(uint8_t cr);

int _write(int file, char *ptr, int len) {
    uint32_t err_code = ble_nus_send_string(&m_nus, ptr, len);
    APP_ERROR_CHECK(err_code);
    return len;
}
/*
static void log(uint8_t d){
  uint8_t data[3] = {0, 0, d};
  uint8_t  err_code = ble_nus_send_string(&m_nus, data, 3);
}
*/
static uint32_t TIMER_INTERVAL_MIN = 10;
static uint32_t TIMER_INTERVAL_MAX = 200;
static uint32_t counter = 0;
static uint32_t pwm_a1 = 0;
static uint32_t pwm_b1 = 0;
static uint32_t pwm_a2 = 0;
static uint32_t pwm_b2 = 0;

/*
int  print( const char * sFormat, ...) {
  va_list ParamList;
  va_start(ParamList, sFormat);
  return SEGGER_RTT_printf(0, sFormat, &ParamList);
}
*/
static void timer1_init(void)
{
    uint32_t p_is_running;
    uint32_t err_code;

    // Configure timer
    NRF_TIMER1->MODE      = TIMER_MODE_MODE_Timer;
    NRF_TIMER1->BITMODE   = TIMER_BITMODE_BITMODE_16Bit;
    NRF_TIMER1->PRESCALER = 3;

    // Clear the timer
    NRF_TIMER1->TASKS_CLEAR = 1;

 //   NRF_TIMER1->CC[0] = TIMER_INTERVAL_MIN;
    NRF_TIMER1->CC[1] = 2*TIMER_INTERVAL_MAX;

    NRF_TIMER1->INTENSET = TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos;
    NRF_TIMER1->INTENSET = TIMER_INTENSET_COMPARE1_Enabled << TIMER_INTENSET_COMPARE1_Pos;

   // NRF_TIMER1->SHORTS = (TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos) & TIMER_SHORTS_COMPARE0_CLEAR_Msk;
    NRF_TIMER1->SHORTS = (TIMER_SHORTS_COMPARE1_CLEAR_Enabled << TIMER_SHORTS_COMPARE1_CLEAR_Pos) & TIMER_SHORTS_COMPARE1_CLEAR_Msk;

    NRF_TIMER1->TASKS_START = 1;

    err_code = sd_nvic_SetPriority(TIMER1_IRQn,APP_IRQ_PRIORITY_LOW);
    APP_ERROR_CHECK(err_code);
    err_code = sd_nvic_ClearPendingIRQ(TIMER1_IRQn);
    APP_ERROR_CHECK(err_code);
    err_code = sd_nvic_EnableIRQ(TIMER1_IRQn);
    APP_ERROR_CHECK(err_code);

    // gpio setup
    nrf_gpio_cfg_output(PWMA1);
    nrf_gpio_cfg_output(AIN1);
    nrf_gpio_cfg_output(AIN2);

    nrf_gpio_cfg_output(PWMB1);
    nrf_gpio_cfg_output(BIN1);
    nrf_gpio_cfg_output(BIN2);

    nrf_gpio_cfg_output(PWMA2);
    nrf_gpio_cfg_output(AIN3);
    nrf_gpio_cfg_output(AIN4);

    nrf_gpio_cfg_output(PWMB2);
    nrf_gpio_cfg_output(BIN3);
    nrf_gpio_cfg_output(BIN4);

    nrf_gpio_cfg_output(STDBY);
    nrf_gpio_pin_clear(STDBY); // turning off motors

    nrf_gpio_pin_set(AIN1);
    nrf_gpio_pin_clear(AIN2);

    nrf_gpio_pin_set(BIN1);
    nrf_gpio_pin_clear(BIN2);


    nrf_gpio_pin_set(AIN3);
    nrf_gpio_pin_clear(AIN4);


    nrf_gpio_pin_set(BIN3);
    nrf_gpio_pin_clear(BIN4);


}

void TIMER1_IRQHandler(void)
{
  //  NRF_TIMER1->CC[0] += TIMER_INTERVAL;

   counter++;

   if(counter >= pwm_a1){
        nrf_gpio_pin_clear(PWMA1);
   }

   if(counter >= pwm_b1 ){
      nrf_gpio_pin_clear(PWMB1);
   }

   if(counter >= pwm_a2){
          nrf_gpio_pin_clear(PWMA2);
     }

   if(counter >= pwm_b2){
          nrf_gpio_pin_clear(PWMB2);
     }


   if(counter >= TIMER_INTERVAL_MAX){
       counter = 0;
        nrf_gpio_pin_set(PWMA1);
        nrf_gpio_pin_set(PWMB1);
        nrf_gpio_pin_set(PWMA2);
        nrf_gpio_pin_set(PWMB2);
   }

   if ((NRF_TIMER1->EVENTS_COMPARE[1] == 1) && ((NRF_TIMER1->INTENSET & TIMER_INTENSET_COMPARE1_Msk) != 0))
       {
	     NRF_TIMER1->EVENTS_COMPARE[1] = 0;
         // nrf_gpio_pin_set(LED_0);

     }


}


void
set_pwm (uint8_t ch, uint32_t pwm)
{
  switch (ch)
    {
    case 0: pwm_a1 = pwm;
          break;
    case 1: pwm_b1 = pwm;
          break;

    case 2: pwm_a2 = pwm;
          break;
     case 3: pwm_b2 = pwm;
          break;

    default:
      break;
    }
}

CalculateTestDrive(uint8_t motor1, uint8_t motor2,uint8_t motor3,uint8_t motor4){

	uint8_t m1 = 0, m2 = 0, m3 = 0, m4 = 0;

   if(motor1 >=0 && motor1 <= 60){
	    m1 =  motor1;
	    nrf_gpio_pin_set(AIN1);
	    nrf_gpio_pin_clear(AIN2);
   }else{  // motor1 >= 61 && motor1 <= 121
        m1 = motor1 - 61;
	    nrf_gpio_pin_set(AIN2);
	    nrf_gpio_pin_clear(AIN1);
   }

   if(motor2 >=0 && motor2 <= 60){
	    m2 = motor2;
	    nrf_gpio_pin_set(BIN1);
	    nrf_gpio_pin_clear(BIN2);
   }else{
	    m2 = motor2 - 61;
	    nrf_gpio_pin_set(BIN2);
	    nrf_gpio_pin_clear(BIN1);
   }

   if(motor3 >=0 && motor3 <= 60){
       m3 = motor3;
	   nrf_gpio_pin_set(AIN3);
	    nrf_gpio_pin_clear(AIN4);
   }else{
	   m3 = motor3 - 61;
	    nrf_gpio_pin_set(AIN4);
	    nrf_gpio_pin_clear(AIN3);
   }

   if(motor4 >=0 && motor4 <= 60){
       m4 = motor4;
	   nrf_gpio_pin_set(BIN3);
	    nrf_gpio_pin_clear(BIN4);
   }else{
	   m4 = motor4 - 61;
	    nrf_gpio_pin_set(BIN4);
	    nrf_gpio_pin_clear(BIN3);
   }

   m1 = (m1 * 10)/ 3;
   m2 = (m2 * 10)/ 3;
   m3 = (m3 * 10)/ 3;
   m4 = (m4 * 10)/ 3;

   set_pwm(0, m1);
   set_pwm(1, m2);
   set_pwm(2, m3);
   set_pwm(3, m4);

  nrf_gpio_pin_set(STDBY);
}

CalculateDrive3Com(uint8_t fb, uint8_t lr, uint8_t swipe) {

	// fb, lr, swipe are from 0 to 100

	// SEGGER_RTT_printf(0, "CalculateDrive3Com  %d  %d   %d        \r\n", fb, lr,
	int16_t sw_speed = 0;
	int16_t fb_speed = 0, throttle = 0;
	int16_t leftmotor_speed = 0, rightmotor_speed = 0;
	bool is_swipe = false;

	sw_speed = swipe - 50;  // swipe speed    0 - 50
	is_swipe = (swipe != 50) ? true : false;
	fb_speed = fb - 50;

	throttle = (!is_swipe)? lr - 50 : swipe - 50;

	leftmotor_speed = ((fb_speed + throttle) * 4); // speed goes from 0  to 200
	rightmotor_speed = ((fb_speed - throttle) * 4);

	if ( abs(leftmotor_speed) > 200) {
			leftmotor_speed = (leftmotor_speed > 0) ? 200 : -200;
	}

	if (abs(rightmotor_speed) > 200) {
			rightmotor_speed = (rightmotor_speed > 0) ? 200 : -200;
	}


	if (!is_swipe) { // we handle forward/backward without the translation movement

       if (rightmotor_speed> 0) {  // right motor forward
			nrf_gpio_pin_set(AIN1);
			nrf_gpio_pin_clear(AIN2);
			set_pwm(0, rightmotor_speed);

			nrf_gpio_pin_set(AIN3);
			nrf_gpio_pin_clear(AIN4);
			set_pwm(2, rightmotor_speed);
		} else {                   // right motor backward
			rightmotor_speed = abs(rightmotor_speed);
			nrf_gpio_pin_set(AIN2);
			nrf_gpio_pin_clear(AIN1);
			set_pwm(0, rightmotor_speed);

			nrf_gpio_pin_set(AIN4);
			nrf_gpio_pin_clear(AIN3);
			set_pwm(2, rightmotor_speed);
		}

        if(leftmotor_speed > 0){  // left motor forward
				nrf_gpio_pin_set(BIN1);
				nrf_gpio_pin_clear(BIN2);
				set_pwm(1, leftmotor_speed);

				nrf_gpio_pin_set(BIN3);
				nrf_gpio_pin_clear(BIN4);
				set_pwm(3, leftmotor_speed);
        } else {                   // left motor backward
        	leftmotor_speed = abs(leftmotor_speed);
			nrf_gpio_pin_set(BIN2);
			nrf_gpio_pin_clear(BIN1);
			set_pwm(1, leftmotor_speed);

			nrf_gpio_pin_set(BIN4);
			nrf_gpio_pin_clear(BIN3);
			set_pwm(3, leftmotor_speed);

        }



	} else {  // we handle forward/backward combined with  translation movement

		leftmotor_speed = abs(leftmotor_speed);
		rightmotor_speed = abs(rightmotor_speed);

		if(!(sw_speed > 0)) {  	    // left translation
        // motor 0 & 3  forward
         nrf_gpio_pin_set(AIN1);
         nrf_gpio_pin_clear(AIN2);
         set_pwm(0, leftmotor_speed);


         nrf_gpio_pin_set(BIN3);
         nrf_gpio_pin_clear(BIN4);
         set_pwm(3, rightmotor_speed);

      // motor 1 & 2 backward
         nrf_gpio_pin_set(AIN4);
         nrf_gpio_pin_clear(AIN3);
         set_pwm(2, leftmotor_speed);

         nrf_gpio_pin_set(BIN2);
         nrf_gpio_pin_clear(BIN1);
         set_pwm(1, rightmotor_speed);
		} else {                         // right translation

            // motor 0 & 3 backward
			  nrf_gpio_pin_set (AIN2);
			  nrf_gpio_pin_clear (AIN1);
			  set_pwm (0, leftmotor_speed);

		      nrf_gpio_pin_set (BIN4);
			  nrf_gpio_pin_clear (BIN3);
			  set_pwm (3, rightmotor_speed);

			  // motors 1 & 2 forward
			  nrf_gpio_pin_set (AIN3);
			  nrf_gpio_pin_clear (AIN4);
			  set_pwm (2, leftmotor_speed);

			  nrf_gpio_pin_set (BIN1);
			  nrf_gpio_pin_clear (BIN2);
			  set_pwm (1, rightmotor_speed);
		}

	}

    //   SEGGER_RTT_printf(0, " l - r %d    %d  %d \r\n", leftmotor_speed, rightmotor_speed, is_swipe);

  nrf_gpio_pin_set(STDBY);
}


CalculateDriveValues ( )
{
  int16_t rightmotor_speed;
  int16_t leftmotor_speed;

  leftmotor_speed = 800;
  rightmotor_speed = 800;
  bool right_shift  = false;
  bool left_shift = false;
  bool forward = false;

  //Scale Drive values fwd_bwd
  //0x01 (forward slowest) - 0x20 (forward fastest) OR
  //0x21 (backward slowest) - 0x40 (backward fastest)

  //simple_uart_put(lft_rgt);

  if (fwd_bwd >= 0x00 && fwd_bwd <= 0x20)
    {
      forward = true;
      leftmotor_speed += (fwd_bwd * 7);
      rightmotor_speed += (fwd_bwd * 7);

    }
  else
    { //must be backwards
      leftmotor_speed -= ((fwd_bwd - 0x20) * 7);
      rightmotor_speed -= ((fwd_bwd - 0x20) * 7);
    }

  //Scale Drive values lft_rgt

  if (lft_rgt != 0)
    {
      //0x41 (right spin slowest) - 0x60 (right spin fastest) OR
      //0x61 (left spin slowest) - 0x80 (left spin fastest)
      if (lft_rgt >= 0x40 && lft_rgt <= 0x60)  //turn to right
	{
	  leftmotor_speed = (leftmotor_speed + ((lft_rgt - 0x40) * 8));
	  rightmotor_speed = (rightmotor_speed - ((lft_rgt - 0x40) * 8));
	  right_shift = true;

	}
      else
	{ //must be left turn
	  leftmotor_speed = (leftmotor_speed - ((lft_rgt - 0x60) * 8));
	  rightmotor_speed = (rightmotor_speed + ((lft_rgt - 0x60) * 8));
	  left_shift = true;
	}
    }

  if (rightmotor_speed >= 800) //Right Motor Forward
    {

      rightmotor_speed = rightmotor_speed - 800;
      rightmotor_speed = (rightmotor_speed * 3 / 2) ;
      if(rightmotor_speed > 200) rightmotor_speed = 200;

    }
  else
    { //reverse

      rightmotor_speed = 800 - rightmotor_speed;
      rightmotor_speed = (rightmotor_speed * 3 / 2) ;
      if(rightmotor_speed > 200) rightmotor_speed = 200;

    }

  if (leftmotor_speed >= 800) //Left Motor Forward
    {

      leftmotor_speed = leftmotor_speed - 800;
      rightmotor_speed = (leftmotor_speed * 3 / 2) ;
      if(leftmotor_speed > 200) leftmotor_speed = 200;
    }
  else
    { //reverse

      leftmotor_speed = 800 - leftmotor_speed;
      rightmotor_speed = (leftmotor_speed * 3 / 2) ;
      if(leftmotor_speed > 200) leftmotor_speed = 200;

    }


    if(left_shift || right_shift){   // not a straight move
            if(right_shift){

           // turn right
			nrf_gpio_pin_set(AIN2);
			nrf_gpio_pin_clear(AIN1);
			set_pwm(0, leftmotor_speed);

			nrf_gpio_pin_set(AIN4);
			nrf_gpio_pin_clear(AIN3);
			set_pwm(2, leftmotor_speed);

			nrf_gpio_pin_set(BIN1);
			nrf_gpio_pin_clear(BIN2);
			set_pwm(1, rightmotor_speed);

			nrf_gpio_pin_set(BIN3);
			nrf_gpio_pin_clear(BIN4);
			set_pwm(3, rightmotor_speed);

            // handling shifts
            	/*
              // motor 0 & 3  forward
               nrf_gpio_pin_set(AIN2);
               nrf_gpio_pin_clear(AIN1);
               set_pwm(0, leftmotor_speed);


               nrf_gpio_pin_set(BIN4);
               nrf_gpio_pin_clear(BIN3);
               set_pwm(3, rightmotor_speed);

            // motor 1 & 2 backward
               nrf_gpio_pin_set(AIN3);
               nrf_gpio_pin_clear(AIN4);
               set_pwm(2, leftmotor_speed);

               nrf_gpio_pin_set(BIN1);
               nrf_gpio_pin_clear(BIN2);
               set_pwm(1, rightmotor_speed);
              */
            }
      else
	{ // left shift

            // turn left
  		    nrf_gpio_pin_set(AIN1);
  			nrf_gpio_pin_clear(AIN2);
  			set_pwm(0, leftmotor_speed);

  			nrf_gpio_pin_set(AIN3);
  			nrf_gpio_pin_clear(AIN4);
  			set_pwm(2, leftmotor_speed);

  			nrf_gpio_pin_set(BIN2);
  			nrf_gpio_pin_clear(BIN1);
  			set_pwm(1, rightmotor_speed);

  			nrf_gpio_pin_set(BIN4);
  			nrf_gpio_pin_clear(BIN3);
  			set_pwm(3, rightmotor_speed);
    // handling shift
    	  /*
	  // motors 0 & 3 backward
	  nrf_gpio_pin_set (AIN1);
	  nrf_gpio_pin_clear (AIN2);
	  set_pwm (0, leftmotor_speed);

      nrf_gpio_pin_set (BIN3);
	  nrf_gpio_pin_clear (BIN4);
	  set_pwm (3, rightmotor_speed);

	  // motors 1 & 2 forward
	  nrf_gpio_pin_set (AIN4);
	  nrf_gpio_pin_clear (AIN3);
	  set_pwm (2, leftmotor_speed);

	  nrf_gpio_pin_set (BIN2);
	  nrf_gpio_pin_clear (BIN1);
	  set_pwm (1, rightmotor_speed);
      */


	}

    }else{ // straight forward/backward
	if( forward ){

	  nrf_gpio_pin_set(AIN1);
	  nrf_gpio_pin_clear(AIN2);
	  set_pwm(0, rightmotor_speed);

	  nrf_gpio_pin_set(BIN1);
	  nrf_gpio_pin_clear(BIN2);
	  set_pwm(1, rightmotor_speed);

	  nrf_gpio_pin_set(AIN3);
	  nrf_gpio_pin_clear(AIN4);
	  set_pwm(2, rightmotor_speed);

	  nrf_gpio_pin_set(BIN3);
	  nrf_gpio_pin_clear(BIN4);
	  set_pwm(3, rightmotor_speed);


	}else{// straight backward
	    nrf_gpio_pin_set(AIN2);
	    nrf_gpio_pin_clear(AIN1);
	    set_pwm(0, rightmotor_speed);

	    nrf_gpio_pin_set(BIN2);
	    nrf_gpio_pin_clear(BIN1);
	    set_pwm(1, rightmotor_speed);

	    nrf_gpio_pin_set(AIN4);
	    nrf_gpio_pin_clear(AIN3);
	    set_pwm(2, rightmotor_speed);

	    nrf_gpio_pin_set(BIN4);
	    nrf_gpio_pin_clear(BIN3);
	    set_pwm(3, rightmotor_speed);
	}

    }

// print("m speed %d  %d \r\n", rightmotor_speed, leftmotor_speed);
	// SEGGER_RTT_printf(0, "CalculateDrive3Com  %d       \r\n",   10);
	// SEGGER_RTT_printf(0, "CalculateDrive3Com  %d  %d   %d  %d      \r\n", rightmotor_speed, left_shift, right_shift, forward);
  nrf_gpio_pin_set(STDBY);


}

static void pwm_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);

		Stop();
		app_timer_stop(m_pwm_timer_id);


}

static void timers_init(void)
{

	    uint32_t err_code;

	   // Initialize timer module
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);

    // Create timers
    err_code = app_timer_create(&m_pwm_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                pwm_timeout_handler);
    APP_ERROR_CHECK(err_code);
}

static void pwm_timers_start(uint32_t time)
{
    uint32_t err_code;

    // Start application timers
    err_code = app_timer_start(m_pwm_timer_id, APP_TIMER_TICKS(time, APP_TIMER_PRESCALER), NULL);
    APP_ERROR_CHECK(err_code);

}

void
Drive (uint32_t time)
{

  // use Timer 0 peripheral to generate 100 ms delay.
  pwm_timers_start (time);

}

void Stop(void)
{
/*
				//		 nrf_gpio_pin_clear(STDBY);  was a very useful way to activate. need something with the new style PWM....
						 nrf_pwm_set_value(0, 0);
						 nrf_pwm_set_value(1, 0);
						 nrf_pwm_set_value(2, 0);
						// nrf_pwm_set_value(3, 0);


*/

  nrf_gpio_pin_clear(STDBY);

  /*
  set_pwm(0, 0);
  set_pwm(1, 0);
  set_pwm(2, 0);
  set_pwm(3, 0);
  */
}


void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t *p_file_name)
{
	// This call can be used for debug purposes during application development.
	// @note CAUTION: Activating this code will write the stack to flash on an error.
	//                This function should NOT be used in a final product.
	//                It is intended STRICTLY for development/debugging purposes.
	//                The flash write will happen EVEN if the radio is active, thus interrupting
	//                any communication.
	//                Use with care. Un-comment the line below to use.
	//ble_debug_assert_handler(error_code, line_num, p_file_name);

	// On assert, the system can only recover with a reset.
	//NVIC_SystemReset();
}


void assert_nrf_callback(uint16_t line_num, const uint8_t *p_file_name)
{
	app_error_handler(DEAD_BEEF, line_num, p_file_name);
}




static void battery_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    battery_start();
}








static void gap_params_init(void)
{
	uint32_t err_code;
	ble_gap_conn_params_t gap_conn_params;
	ble_gap_conn_sec_mode_t sec_mode;

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

	err_code = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *)DEVICE_NAME, strlen(DEVICE_NAME));
	APP_ERROR_CHECK(err_code);

	memset(&gap_conn_params, 0, sizeof(gap_conn_params));

	gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
	gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
	gap_conn_params.slave_latency = SLAVE_LATENCY;
	gap_conn_params.conn_sup_timeout = CONN_SUP_TIMEOUT;

	err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
	APP_ERROR_CHECK(err_code);
}


static void advertising_init(void)
{
	uint32_t err_code;
	ble_advdata_t advdata;
	ble_advdata_t scanrsp;
	uint8_t flags = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

	ble_uuid_t adv_uuids[] = {{BLE_UUID_NUS_SERVICE, m_nus.uuid_type}};

	// Build and set advertising data
	memset(&advdata, 0, sizeof(advdata));

	advdata.name_type = BLE_ADVDATA_FULL_NAME;
	advdata.include_appearance = true;  //false in Davin's code
	advdata.flags.size = sizeof(flags);
	advdata.flags.p_data = &flags;

	memset(&scanrsp, 0, sizeof(scanrsp));
	scanrsp.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
	scanrsp.uuids_complete.p_uuids  = adv_uuids;

	err_code = ble_advdata_set(&advdata, &scanrsp);
	APP_ERROR_CHECK(err_code);
}



//static void cmd_handler(ble_app_t *app, uint8_t *cmd, uint8_t cmdlen)
//{
/**@brief    Function for handling the data from the Nordic UART Service.
 *
 * @details  This function will process the data received from the Nordic UART BLE Service and send
 *           it to the UART module.
 */
/**@snippet [Handling the data received over BLE] */
void nus_data_handler(ble_nus_t * p_nus, uint8_t * cmd, uint16_t cmdlen)
{

	static uint8_t buf[20];			/* static buffer for data sent back over BLE */
	uint8_t len;				/* how many bytes to return (if any) */
	struct qr_cmd new_cmd;
	uint32_t err_code;

	switch (cmd[0]) {

	case APP_CMD_ARM:

		len = 0;
		break;
	case NOTF_GET_STATUS:
	case NOTF_ACTIVATE_ADB:
	case STAND_UP       :
	case KNEEL          :
	case LEAN           :
	case ESTOP          :
	case CLEAR_ESTOP    :
	        simple_uart_put(cmd[0]);
	        simple_uart_put(cmd[1]);
	        simple_uart_put(0x00);
	        break;

	case 'x':
                fwd_bwd = cmd[1];
                lft_rgt = cmd[2];
                CalculateDriveValues();
                Drive(200);
	        /*
	        simple_uart_put('x');
		simple_uart_put(cmd[1]);
		simple_uart_put(cmd[2]);
		*/

		break;
	case DRIVE_TEST:
         CalculateTestDrive(cmd[1], cmd[2], cmd[3], cmd[4] );
         Drive(200);
		break;

	case DRIVE_3COM:

		/*
		 *
		 *
		byte 1: 0xf0
		byte 2: thrust (0xe1 - 0x00)
		byte 3: yaw (0x00 - 0xe1)    --> swipe left/right
		byte 4: pitch (0x00 - 0xe1)  -->  fwd_bwd
		byte 5: roll (0x00 - 0xe1)   -->  lft_rgt
		 *
		 * */
		CalculateDrive3Com(cmd[3], cmd[4], cmd[2]);
		break;
	case 0x06:

		if(cmd[1] == 0x01)
		{

		//eStop
		simple_uart_put(0x65);
		simple_uart_put(0x00);
		simple_uart_put(0x00);


		}

		if(cmd[1] == 0x02)
		{

		//Clear estop
		simple_uart_put(0x66);
		simple_uart_put(0x00);
		simple_uart_put(0x00);
		}


		break;	

	case 'q':  //TIR String

        printf("[r] %d",IR[0]);
        printf("[t] %d",IR[1]);
        printf("[l] %d",IR[2]);
        printf("[h] %d",IR[3]);
										
		len = 0;
		break;	

	case 'r':  //TIR String

        printf("[H] %d",Head);
        printf("[ER] %d",easyRange);	
		len = 0;
		break;



	default:
		len = 0;
		break;
	};

	if (len) {
		//ble_app_tx_blob(app, buf, len);
	}
}




/*
 * toplevel function which initializes all BLE services
 */
static void services_init(void)
{
    uint32_t         err_code;
    ble_nus_init_t   nus_init;
    
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;
    
    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}


/*
 * toplevel function which initializes all BLE services
 */
 /*
static void services_init(void)
{
	ble_app_init_t ai;

//	dis_init();
//	bas_init();

	ai.cmd_handler = cmd_handler;
	ble_app_init(&app, &ai);
}
*/

static void sec_params_init(void)
{
	m_sec_params.timeout = SEC_PARAM_TIMEOUT;
	m_sec_params.bond = SEC_PARAM_BOND;
	m_sec_params.mitm = SEC_PARAM_MITM;
	m_sec_params.io_caps = SEC_PARAM_IO_CAPABILITIES;
	m_sec_params.oob = SEC_PARAM_OOB;
	m_sec_params.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
	m_sec_params.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
}


static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
	if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED) {
		uint32_t err_code;

		err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
		APP_ERROR_CHECK(err_code);
	}
}


static void conn_params_error_handler(uint32_t nrf_error)
{
	APP_ERROR_HANDLER(nrf_error);
}


static void conn_params_init(void)
{
	uint32_t err_code;
	ble_conn_params_init_t cp_init;

	memset(&cp_init, 0, sizeof(cp_init));

	cp_init.p_conn_params = NULL;
	cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
	cp_init.next_conn_params_update_delay = NEXT_CONN_PARAMS_UPDATE_DELAY;
	cp_init.max_conn_params_update_count = MAX_CONN_PARAMS_UPDATE_COUNT;
	cp_init.start_on_notify_cccd_handle = BLE_GATT_HANDLE_INVALID;
	cp_init.disconnect_on_fail = false;
	cp_init.evt_handler = on_conn_params_evt;
	cp_init.error_handler = conn_params_error_handler;

	err_code = ble_conn_params_init(&cp_init);
	APP_ERROR_CHECK(err_code);
}


static void advertising_start(void)
{
	uint32_t err_code;
	ble_gap_adv_params_t adv_params;

	memset(&adv_params, 0, sizeof(adv_params));

	adv_params.type = BLE_GAP_ADV_TYPE_ADV_IND;
	adv_params.p_peer_addr = NULL;
	adv_params.fp = BLE_GAP_ADV_FP_ANY;
	adv_params.interval = APP_ADV_INTERVAL;
	adv_params.timeout = APP_ADV_TIMEOUT_IN_SECONDS;

	err_code = sd_ble_gap_adv_start(&adv_params);
	APP_ERROR_CHECK(err_code);
}


static void on_ble_evt(ble_evt_t *evt)
{
	uint32_t err_code;
	static ble_gap_evt_auth_status_t m_auth_status;
	ble_gap_enc_info_t *enc_info;

	switch (evt->header.evt_id) {
	case BLE_GAP_EVT_CONNECTED:
	        //nrf_gpio_pin_set(CONNECTED_LED_PIN_NO);
            //nrf_gpio_pin_clear(LED_1);
		SEGGER_RTT_printf(0, "connected ...\r\n");
		break;

	case BLE_GAP_EVT_DISCONNECTED:
		    //nrf_gpio_pin_clear(CONNECTED_LED_PIN_NO);
			advertising_start();
		break;

	case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
		err_code = sd_ble_gap_sec_params_reply(app.conn_handle, BLE_GAP_SEC_STATUS_SUCCESS, &m_sec_params);
		APP_ERROR_CHECK(err_code);
		break;

	case BLE_GATTS_EVT_SYS_ATTR_MISSING:
		err_code = sd_ble_gatts_sys_attr_set(app.conn_handle, NULL, 0);
		APP_ERROR_CHECK(err_code);
		break;

	case BLE_GAP_EVT_AUTH_STATUS:
		m_auth_status = evt->evt.gap_evt.params.auth_status;
		break;

	case BLE_GAP_EVT_SEC_INFO_REQUEST:
		enc_info = &m_auth_status.periph_keys.enc_info;
		if (enc_info->div == evt->evt.gap_evt.params.sec_info_request.div) {
			err_code = sd_ble_gap_sec_info_reply(app.conn_handle, enc_info, NULL);
			APP_ERROR_CHECK(err_code);
		} else {
			// No keys found for this device
			err_code = sd_ble_gap_sec_info_reply(app.conn_handle, NULL, NULL);
			APP_ERROR_CHECK(err_code);
		}
		break;

	case BLE_GAP_EVT_TIMEOUT:
		if (evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISEMENT) {
#ifdef NEVER_SLEEP
			advertising_init();
			advertising_start();
#else
			if (charging) {
				advertising_init();
				advertising_start();
			} else {
				/* make sure the LEDs are off */
				//nrf_gpio_pin_clear(LED_1);
				//nrf_gpio_pin_clear(LED_2);

				// Configure buttons with sense level low as wakeup source.
				nrf_gpio_cfg_sense_input(BUTTON_0, BUTTON_PULL, NRF_GPIO_PIN_SENSE_LOW);
				nrf_gpio_cfg_sense_input(BUTTON_1, BUTTON_PULL, NRF_GPIO_PIN_SENSE_LOW);

				// Go to system-off mode (this function will not return; wakeup will cause a reset)
				err_code = sd_power_system_off();
				APP_ERROR_CHECK(err_code);
			}
#endif
		}
		break;

	case BLE_EVT_TX_COMPLETE:
		break;

	default:
		// No implementation needed.
		break;
	};
}


/*static void ble_evt_dispatch(ble_evt_t *evt)
{
	on_ble_evt(evt);
	ble_conn_params_on_ble_evt(evt);
	ble_app_on_ble_evt(&app, evt);
}*/

static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_nus_on_ble_evt(&m_nus, p_ble_evt);
    on_ble_evt(p_ble_evt);
}	


static void sys_evt_dispatch(uint32_t sys_evt)
{
	pstorage_sys_event_handler(sys_evt);
}


static void ble_stack_init(void)
{
	uint32_t err_code;

	// Initialize the SoftDevice handler module.
	SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_RC_250_PPM_4000MS_CALIBRATION, false);

	// Register with the SoftDevice handler module for BLE events.
	err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
	APP_ERROR_CHECK(err_code);

	// Register with the SoftDevice handler module for BLE events.
	err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
	APP_ERROR_CHECK(err_code);
}


static void scheduler_init(void)
{
	APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}




static void gpiote_init(void)
{
	APP_GPIOTE_INIT(APP_GPIOTE_MAX_USERS);
}




static void power_manage(void)
{
	uint32_t err_code = sd_app_evt_wait();
	APP_ERROR_CHECK(err_code);
}


int main(void)
{
        //pwm_init();
 //      uart_init();
	timers_init();

	gpiote_init();

	ble_stack_init();
	scheduler_init();
	gap_params_init();
	services_init();
	advertising_init();
	conn_params_init();
	sec_params_init();

    timer1_init();

	app_button_enable();
	advertising_start();

	nrf_delay_ms(170);

  //  motors_test();
	 SEGGER_RTT_printf(0, "App started  %%c,         'S' : %c.\r\n", 'S');





/*
	   uint8_t testValues[9][3] = {
	       {50, 50, 50},
	       {20, 50, 50},
	       {30, 10, 50},
	       {30, 45, 50},
	       {60, 60, 50},
	       {90, 20, 50},
	       {90, 50, 50},
	       {100, 50, 50},
	       {90, 100, 50}
	   };

	 for(int i = 0; i < 9; i++){
         CalculateDrive3Com(testValues[i][0], testValues[i][1], testValues[i][2]);
	 }
*/


	while (1) {
		nrf_delay_ms(50);
		app_sched_execute();

		power_manage();


	};
}

void motors_test(){

	   nrf_gpio_pin_set(AIN1);
	   nrf_gpio_pin_clear(AIN2);
	   nrf_gpio_pin_set(BIN1);
	   nrf_gpio_pin_clear(BIN2);
	   nrf_gpio_pin_set(AIN3);
	   nrf_gpio_pin_clear(AIN4);
       nrf_gpio_pin_set(BIN3);
	   nrf_gpio_pin_clear(BIN4);


	   set_pwm(0, 120);
	   set_pwm(1, 120);
	   set_pwm(2, 120);
	   set_pwm(3, 120);

	   nrf_gpio_pin_set(STDBY);

}
