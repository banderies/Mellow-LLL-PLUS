/**
  ***************************************************************************************
  * @file    buffer.h
  * @author  lijihu
  * @version V1.0.0
  * @date    2025/05/10
  * @brief   Buffer board functionality
  *          Hall sensor: blocked=1, unblocked=0
  *          Filament switch: filament present=0, absent=1
  *          Button: pressed=0, released=1
  *
  *          Pin mapping:
  *          HALL1 --> PB2 (hall sensor 3)
  *          HALL2 --> PB3 (hall sensor 2)
  *          HALL3 --> PB4 (hall sensor 1)
  *          ENDSTOP_3 --> PB7 (filament switch)
  *          KEY1 --> PB13 (reverse)
  *          KEY2 --> PB12 (forward)
  *
  * @note
  ***************************************************************************************
  * COPYRIGHT 2024 xxx@126.com
  ***************************************************************************************
**/

#ifndef __BUFFER_H__
#define __BUFFER_H__

#include <TMCStepper.h>
#include <Arduino.h>
#include <EEPROM.h>

#define HALL1       PB2 //hall sensor 3
#define HALL2       PB3 //hall sensor 2
#define HALL3       PB4 //hall sensor 1

#define ENDSTOP_3   PB7 //filament switch (proximal, buffer side of extruder gears)
#define DISTAL_SWITCH PB14 //filament switch (distal, hotend side of extruder gears)

#define KEY1        PB13 //reverse
#define KEY2        PB12 //forward

#define EN_PIN      PA6 //enable
#define DIR_PIN     PA7 //direction
#define STEP_PIN    PC13 //step
#define UART        PB1 //software serial

#define DUANLIAO    PB15 //filament runout output
#define DULIAO    	PB15 //blockage output
#define ERR_LED     PA15 //indicator LED
#define START_LED   PA8  //indicator LED

#define EXTENSION_PIN1 PA2 //extension pin 1
#define EXTENSION_PIN2 PA3 //extension pin 2
#define EXTENSION_PIN3 PB11 //extension pin 3
#define EXTENSION_PIN4 PB10 //extension pin 4
#define EXTENSION_PIN5 PA5 //extension pin 5
#define EXTENSION_PIN6 PA4 //extension pin 6
#define EXTENSION_PIN7 PB14 //extension pin 7

//blockage detection
#define PULSE1_PIN EXTENSION_PIN5  	//pulse input pin 1, receives controller pulses
#define SIG_DIR_PIN EXTENSION_PIN3	//direction pin, extrude=1, retract=0
#define PULSE2_PIN EXTENSION_PIN4 	//pulse input pin 2, receives MDM module pulses
#define MDM_DPIN EXTENSION_PIN6 	//MDM filament pin 1=filament present, 0=absent

//signal detection
#define FRONT_SIGNAL_PIN PB5 //forward signal pin
#define BACK_SIGNAL_PIN PB6 //reverse signal pin

//SIGNAL_COUNT(GPIO)
#define SIGNAL_COUNT_DIR_CLK_ENABLE()		__HAL_RCC_GPIOB_CLK_ENABLE()	//PB11
#define SIGNAL_COUNT_DIR_GPIO_Port			(GPIOB)
#define SIGNAL_COUNT_DIR_Pin				(GPIO_PIN_11)
#define SIGNAL_COUNT_DIR_Get_IRQn			(EXTI4_15_IRQn)	//EXTI11 interrupt

//SIGNAL_COUNT(AFIO & TIM)
#define SIGNAL_COUNT_PUL_CLK_ENABLE()		__HAL_RCC_GPIOA_CLK_ENABLE()	//PA5
#define SIGNAL_COUNT_PUL_GPIO_Port			(GPIOA)
#define SIGNAL_COUNT_PUL_Pin						(GPIO_PIN_5)
#define SIGNAL_COUNT_TIM_CLK_ENABLE()		__HAL_RCC_TIM2_CLK_ENABLE()		//TIM2
#define	SIGNAL_COUNT_Get_TIM						(TIM2)
#define	SIGNAL_COUNT_Get_HTIM						(htim2)




#define DRIVER_ADDRESS 0b00 // TMC Driver address according to MS1 and MS2
#define R_SENSE 0.11f // Match to your driver

static int32_t SPEED=260;  //speed (RPM)
#define Move_Divide_NUM			((int32_t)(64))		//microsteps per step
static int32_t VACTRUAL_VALUE=(uint32_t)(SPEED*Move_Divide_NUM*200/60/0.715) ;  //VACTUAL register value

#define STOP 0				//stop
#define WRITE_EN_PIN(x) digitalWrite(EN_PIN,x)//set EN pin
#define FORWARD		1//filament forward direction
#define BACK		0


#define DEBUG 0

//struct to store buffer sensor states
typedef struct Buffer
{
	//buffer1
	bool buffer1_pos1_sensor_state;	
	bool buffer1_pos2_sensor_state;		
	bool buffer1_pos3_sensor_state;		
	bool buffer1_material_swtich_state;
	bool distal_switch_state;
	bool key1;
	bool key2;
	
}Buffer;

//motor state control enum
typedef enum
{
	Forward=0,//forward
	Stop,		//stop
	Back		//reverse
}Motor_State;

//blockage detection struct
typedef struct BlockageDetect
{
	int32_t target_distance; //target distance
	int32_t actual_distance; //actual distance
	int32_t distance_error; //distance error
	float allow_error; //allowed error
	bool blockage_flag; //blockage flag

	int32_t mdm_pulse_cnt=0;//pulses received from MDM module
	int16_t last_pulse_cnt=0;//previous pulse count
	int16_t pulse_cnt=0;//pulses received from controller
	int16_t pulse_cnt_sub=0;//pulse difference
	int32_t extrusion_pulse_cnt=0;//extrusion pulse count
	float encoder_length; //encoder length (mm/pulse)

}BlockageDetect;

struct Buffer_Parameter{
	uint32_t timeout;
	uint32_t steps;
	float encoder_length;
	float allow_error_scale;
	int32_t SPEED;
	uint32_t I_CURRENT;
	bool DUANLIAO_OUT_STATE;
	uint32_t coast_delay;
	uint16_t magic_number;
};

extern void buffer_sensor_init();
extern void buffer_motor_init();

extern void read_sensor_state(void);
extern void motor_control(void);

extern void buffer_init();
extern void buffer_loop(void);
extern void timer_it_callback();
extern void buffer_debug(void);

extern bool is_error;
extern uint32_t front_time;//forward feed time
extern uint32_t timeout;
extern bool is_front;
extern TMC2209Stepper driver;


#endif