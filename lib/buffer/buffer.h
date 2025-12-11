/**
  ***************************************************************************************
  * @file    buffer.cpp
  * @author  lijihu
  * @version V1.0.0
  * @date    2025/05/10
  * @brief   Implements buffer functionality (translated)
			  *Buffer description: (translated)
				Light sensor: blocked=1, unblocked=0; (translated)
				Filament switch: filament present=0, filament absent=1; (translated)
				Button: pressed=0, released=1; (translated)

				Pins: (translated)
				HALL1 --> PB2 (sensor 3) (translated)
				HALL2 --> PB3 (sensor 2) (translated)
				HALL3 --> PB4 (sensor 1) (translated)
				ENDSTOP_3 --> PB7 (filament switch) (translated)
				KEY1 --> PB13 (reverse) (translated)
				KEY2 --> PB12 (forward) (translated)
  *
  * @note
  ***************************************************************************************
  * Copyright COPYRIGHT 2024 xxx@126.com (translated)
  ***************************************************************************************
**/

#ifndef __BUFFER_H__
#define __BUFFER_H__

#include <TMCStepper.h>
#include <Arduino.h>
#include <EEPROM.h>

#define HALL1       PB2 //sensor 3 (translated)
#define HALL2       PB3 //sensor 2 (translated)
#define HALL3       PB4 //sensor 1 (translated)

#define ENDSTOP_3   PB7 //filament switch (translated)

#define KEY1        PB13 //reverse (translated)
#define KEY2        PB12 //forward (translated)

#define EN_PIN      PA6 //enable (translated)
#define DIR_PIN     PA7 //direction (translated)
#define STEP_PIN    PC13 //step (translated)
#define UART        PB1 //software serial (translated)

#define DUANLIAO    PB15 //filament runout (translated)
#define DULIAO    	PB15 //blockage (translated)
#define ERR_LED     PA15 //indicator LED (translated)
#define START_LED   PA8  //indicator LED (translated)

#define EXTENSION_PIN1 PA2 //extension pin 1 (translated)
#define EXTENSION_PIN2 PA3 //extension pin 2 (translated)
#define EXTENSION_PIN3 PB11 //extension pin 3 (translated)
#define EXTENSION_PIN4 PB10 //extension pin 4 (translated)
#define EXTENSION_PIN5 PA5 //extension pin 5 (translated)
#define EXTENSION_PIN6 PA4 //extension pin 6 (translated)
#define EXTENSION_PIN7 PB14 //extension pin 7 (translated)

//blockage detection (translated)
#define PULSE1_PIN EXTENSION_PIN5  	//pulse input pin 1, receives main controller pulses (translated)
#define BLOCKAGE_DIR_PIN EXTENSION_PIN3	//direction pin, extrude=1, retract=0 (translated)
#define PULSE2_PIN EXTENSION_PIN4 	//pulse input pin 2, receives MDM module pulses (translated)
#define MDM_DPIN EXTENSION_PIN6 	//MDM filament pin, 1=filament present, 0=filament absent (translated)

//signal detection (translated)
#define FRONT_SIGNAL_PIN PB5 //front signal pin (translated)
#define BACK_SIGNAL_PIN PB6 //back signal pin (translated)

//SIGNAL_COUNT(GPIO)
#define SIGNAL_COUNT_DIR_CLK_ENABLE()		__HAL_RCC_GPIOB_CLK_ENABLE()	//PB11
#define SIGNAL_COUNT_DIR_GPIO_Port			(GPIOB)
#define SIGNAL_COUNT_DIR_Pin				(GPIO_PIN_11)
#define SIGNAL_COUNT_DIR_Get_IRQn			(EXTI4_15_IRQn)	//EXTI11 interrupt (translated)

//SIGNAL_COUNT(AFIO & TIM)
#define SIGNAL_COUNT_PUL_CLK_ENABLE()		__HAL_RCC_GPIOA_CLK_ENABLE()	//PA5
#define SIGNAL_COUNT_PUL_GPIO_Port			(GPIOA)
#define SIGNAL_COUNT_PUL_Pin						(GPIO_PIN_5)
#define SIGNAL_COUNT_TIM_CLK_ENABLE()		__HAL_RCC_TIM2_CLK_ENABLE()		//TIM2
#define	SIGNAL_COUNT_Get_TIM						(TIM2)
#define	SIGNAL_COUNT_Get_HTIM						(htim2)




#define DRIVER_ADDRESS 0b00 // TMC Driver address according to MS1 and MS2
#define R_SENSE 0.11f // Match to your driver

static int32_t SPEED=260;  //speed (unit: r/min) (translated)
#define Move_Divide_NUM			((int32_t)(64))		//microsteps per step (translated)
static int32_t VACTRUAL_VALUE=(uint32_t)(SPEED*Move_Divide_NUM*200/60/0.715) ;  //VACTUAL register value (translated)

#define STOP 0				//stop (translated)
#define I_CURRENT (500)		//current (translated)
#define WRITE_EN_PIN(x) digitalWrite(EN_PIN,x)//enable EN pin (translated)
#define FORWARD		1//filament direction (translated)
#define BACK		0

#define DEBUG 0

//struct to store buffer sensor states (translated)
typedef struct Buffer
{
	//buffer1
	bool buffer1_pos1_sensor_state;
	bool buffer1_pos2_sensor_state;
	bool buffer1_pos3_sensor_state;
	bool buffer1_material_swtich_state;
	bool key1;
	bool key2;

}Buffer;

//motor state control enum (translated)
typedef enum
{
	Forward=0,//forward (translated)
	Stop,		//stop (translated)
	Back		//reverse (translated)
}Motor_State;

//blockage detection struct (translated)
typedef struct BlockageDetect
{
	int32_t target_distance; //target distance (translated)
	int32_t actual_distance; //actual distance (translated)
	int32_t distance_error; //distance error (translated)
	float allow_error; //allowed error (translated)
	bool blockage_flag; //blockage flag (translated)

	int32_t mdm_pulse_cnt=0;//pulses received from MDM module (translated)
	int16_t last_pulse_cnt=0;//previous pulse count (translated)
	int16_t pulse_cnt=0;//pulses received from main controller (translated)
	int16_t pulse_cnt_sub=0;//pulse difference (translated)
	int32_t extrusion_pulse_cnt=0;//extrusion pulse count (translated)
	float encoder_length; //encoder length (translated)

}BlockageDetect;

extern void buffer_sensor_init();
extern void buffer_motor_init();

extern void read_sensor_state(void);
extern void motor_control(void);

extern void buffer_init();
extern void buffer_loop(void);
extern void timer_it_callback();
extern void buffer_debug(void);

extern bool is_error;
extern uint32_t front_time;//forward time (translated)
extern uint32_t timeout;
extern bool is_front;
extern TMC2209Stepper driver;


#endif