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
  * Copyright COPYRIGHT 2025 xxx@126.com (translated)
  ***************************************************************************************
**/


#include "buffer.h"

//GPIO input (translated)
#define SIGNAL_COUNT_READ_DIR_IO()	(SIGNAL_COUNT_DIR_GPIO_Port -> IDR & SIGNAL_COUNT_DIR_Pin)
//TIM input (translated)
#define SIGNAL_COUNT_READ_COUNT()		(SIGNAL_COUNT_Get_TIM -> CNT)
//TIM output (translated)
#define SIGNAL_COUNT_UP()						(SIGNAL_COUNT_Get_TIM -> CR1 &= ~(TIM_CR1_DIR))
#define SIGNAL_COUNT_DOWN()					(SIGNAL_COUNT_Get_TIM -> CR1 |=  (TIM_CR1_DIR))




TMC2209Stepper driver(UART, UART, R_SENSE, DRIVER_ADDRESS);
Buffer buffer={0};//stores sensor states (translated)
Motor_State motor_state=Stop;
static Motor_State last_motor_state=Stop;

bool is_front=false;//forward flag (translated)
uint32_t front_time=0;//forward time (translated)
const uint32_t DEFAULT_TIMEOUT = 60000;
uint32_t timeout=60000;//timeout in ms (translated)
bool is_error=false;//error flag, if filament feeds for 60s continuously without stopping, considered an error (translated)
String serial_buf;

static HardwareTimer timer(TIM6);//timeout error (translated)
TIM_HandleTypeDef htim2;//hardware timer receives pulses (translated)

bool key1_press_flag=false;
bool key2_press_flag=false;
bool key1_release_flag=false;
bool key2_release_flag=false;
uint32_t key1_press_times=0;
uint32_t key2_press_times=0;
uint32_t key1_release_times=0;
uint32_t key2_release_times=0;
uint8_t key1_press_cnt=0;
uint8_t key2_press_cnt=0;

// Auto-feed/retract state
bool auto_feed_active=false;    // Auto-feed forward until buffer triggers
bool auto_retract_active=false; // Auto-retract until runout sensor triggers

uint32_t inform_flag=false;
uint32_t inform_times=0;

const uint32_t DEFAULT_STEPS = 916;
uint32_t steps=916;//pulses per mm (translated)
BlockageDetect blockage_detect={0};//blockage detection struct (translated)
bool connet_mdm_flag=false;
uint32_t blockage_inform_times=0;


const float DEFAULT_ENCODER_LENGTH = 1.73;
float encoder_length=1.73;//MDM module filament movement per pulse (mm/pulse) (translated)


const float DEFAULT_ALLOW_ERROR_SCALE = 2;
float allow_error_scale=2;//allowed error scale (translated)

const int EEPROM_ADDR_TIMEOUT = 0;
const int EEPROM_ADDR_STEPS = 4;
const int EEPROM_ADDR_ENCODER_LENGTH = 8;
const int EEPROM_ADDR_ERROR_SCALE = 12;
const int EEPROM_ADDR_SPEED = 16;


//independent watchdog (translated)
#include "stm32f0xx_hal_iwdg.h"

IWDG_HandleTypeDef hiwdg;
static volatile uint32_t g_run_cnt=0;

void iwdg_init(void)
{
	// 1. Enable write access (translated)
	IWDG->KR = 0x5555;

	// 2. Set prescaler (translated)
	IWDG->PR = IWDG_PRESCALER_256;

	// 3. Set reload value (translated)
	IWDG->RLR = 125*2-1;  //timeout 2s (translated)

	// 4. Start IWDG (translated)
	IWDG->KR = 0xCCCC;
}



//function declarations (translated)
void key1_it_callback(void);
void key2_it_callback(void);

void USB_Serial_Analys(void);
bool Check_Connet_MDM(void);
void REIN_TIM_SIGNAL_COUNT_Init(void);
void REIN_TIM_SIGNAL_COUNT_DeInit(void);
void Pulse_Receive_Init(void);
void Blockage_Detect(void);
void Main_Logic(void);
float fastAtof(const char *s);
void Signal_Dir_Init(void);
// void Buffer_S3_IT_Callback(void);
// void Buffer_S2_IT_Callback(void);
// void Buffer_S1_IT_Callback(void);

void buffer_init(){

	NVIC_SetPriority(TIM6_DAC_IRQn,0);//timeout error, independent watchdog (translated)
	// NVIC_SetPriority(TIM2_IRQn,1);//receive high frequency pulses (translated)
	NVIC_SetPriority(EXTI4_15_IRQn,1);//key1, key2, dir, blockage pulses (translated)

	if(Check_Connet_MDM()){
		connet_mdm_flag=true;
		Pulse_Receive_Init();
		Serial.println("MDM connected"); //(translated)
	}
	else{
		connet_mdm_flag=false;
		Serial.println("MDM not connected"); //(translated)

	}


  buffer_sensor_init();
  buffer_motor_init();
  Signal_Dir_Init();
  delay(1000);

  EEPROM.get(EEPROM_ADDR_TIMEOUT, timeout);
  // Check if value is valid (e.g., before first write it may be 0xFFFFFFFF or 0) (translated)
  if (timeout == 0xFFFFFFFF || timeout == 0) {
    timeout = DEFAULT_TIMEOUT;
    EEPROM.put(EEPROM_ADDR_TIMEOUT, timeout);
    // Serial.println("EEPROM is empty");
  } else {
    // Serial.print("read timeout: ");
    // Serial.println(timeout);
  }

  EEPROM.get(EEPROM_ADDR_SPEED, SPEED);
  if (SPEED < 0 || SPEED > 1000) {
    SPEED=260;
  }
  VACTRUAL_VALUE=(uint32_t)(SPEED*Move_Divide_NUM*200/60/0.715) ;  //VACTUAL register value (translated)



  timer.pause();
  timer.setPrescaleFactor(4800);//4800分频  48000000/4800=10000
  timer.setOverflow(1000);//100ms
  timer.attachInterrupt(&timer_it_callback);
  timer.resume();

  iwdg_init();
}

void buffer_loop()
{
	uint32_t lastToggleTime = 0; // record last toggle time (translated)

	while (1)
	{
		uint32_t nowTime=millis();
		if (blockage_detect.blockage_flag &&nowTime - lastToggleTime >= 50)
		{							   // blockage detected, rapid flash (translated)
			lastToggleTime = millis(); // record current time (translated)
			digitalToggle(ERR_LED);

		}
		else{
			if(connet_mdm_flag){//MDM module connected, flash twice per second (translated)
				static uint8_t led_state=0;
				if(led_state==0){
					digitalWrite(ERR_LED,HIGH);
					led_state=1;
					lastToggleTime = millis();
				}
				else if(led_state==1&&nowTime-lastToggleTime>=100){
					digitalWrite(ERR_LED,LOW);
					led_state=2;
					lastToggleTime = millis();
				}
				else if(led_state==2&&nowTime-lastToggleTime>=100){
					digitalWrite(ERR_LED,HIGH);
					led_state=3;
					lastToggleTime = millis();
				}
				else if(led_state==3&&nowTime-lastToggleTime>=100){
					digitalWrite(ERR_LED,LOW);
					led_state=4;
					lastToggleTime = millis();
				}
				else if(led_state==4&&nowTime-lastToggleTime>=600){
					led_state=0;
				}

			}
			else if(millis() - lastToggleTime >= 500) //not connected, flash once per second (translated)
			{
				lastToggleTime = millis(); // record current time (translated)
				digitalToggle(ERR_LED);
				// Serial.println("CNT:"+String(TIM2->CNT));
				// Serial.println("	mdm_pulse_cnt:"+String(blockage_detect.mdm_pulse_cnt));
			}


		}
		// 1. Read all sensor values (translated)
		read_sensor_state();
		if(connet_mdm_flag) Blockage_Detect();
		motor_control();
		USB_Serial_Analys();

		// Blockage output turns off after 3s (translated)
		if (blockage_detect.blockage_flag)
		{
			if (millis() - blockage_inform_times > 3000)
			{
				digitalWrite(DULIAO, HIGH);
				blockage_detect.blockage_flag = false;
			}
		}

		g_run_cnt++;
	}
}

void buffer_sensor_init(){
  //sensor initialization (translated)
  pinMode(HALL1,INPUT);
  pinMode(HALL2,INPUT);
  pinMode(HALL3,INPUT);
  pinMode(ENDSTOP_3,INPUT);

//   attachInterrupt(HALL1,&Buffer_S3_IT_Callback,RISING);
//   attachInterrupt(HALL2,&Buffer_S2_IT_Callback,RISING);
//   attachInterrupt(HALL3,&Buffer_S1_IT_Callback,RISING);

  pinMode(KEY1,INPUT);
  pinMode(KEY2,INPUT);

  attachInterrupt(KEY1,&key1_it_callback,CHANGE);
  attachInterrupt(KEY2,&key2_it_callback,CHANGE);

  //filament indicator LED initialization (translated)
  pinMode(DUANLIAO,OUTPUT);
  pinMode(ERR_LED,OUTPUT);
  pinMode(START_LED,OUTPUT);
  pinMode(DULIAO,OUTPUT);

  //extension pin initialization (translated)
  pinMode(EXTENSION_PIN1,OUTPUT);
  pinMode(EXTENSION_PIN2,OUTPUT);

  digitalWrite(EXTENSION_PIN1,LOW);
  digitalWrite(EXTENSION_PIN2,HIGH);
  digitalWrite(DULIAO,HIGH);
  digitalWrite(DUANLIAO,HIGH);

  //configure as pull-up input, signal controls buffer, low level triggers feed or retract action (translated)
  pinMode(FRONT_SIGNAL_PIN,INPUT_PULLUP);
  pinMode(BACK_SIGNAL_PIN,INPUT_PULLUP);

}

void buffer_motor_init(){
  //motor driver pin initialization (translated)
  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);      // Enable driver in hardware

  //motor driver initialization (translated)
  driver.begin();                  // UART: Init SW UART (if selected) with default 115200 baudrate
  driver.beginSerial(9600);
  driver.I_scale_analog(false);
  driver.toff(5);                 // Enables driver in software
  driver.rms_current(I_CURRENT);        // Set motor RMS current
  driver.microsteps(Move_Divide_NUM);          // Set microsteps to 1/16th
  driver.VACTUAL(STOP);           // Set velocity
  driver.en_spreadCycle(true);
  driver.pwm_autoscale(true);
 
}

/**
  * @brief  Read all sensor states (translated)
  * @param  NULL
  * @retval NULL
**/
void read_sensor_state(void)
{
	buffer.buffer1_pos1_sensor_state= digitalRead(HALL3);
	buffer.buffer1_pos2_sensor_state= digitalRead(HALL2);	
	buffer.buffer1_pos3_sensor_state= digitalRead(HALL1);		
	buffer.buffer1_material_swtich_state=digitalRead(ENDSTOP_3);	
	buffer.key1=digitalRead(KEY1);
	buffer.key2=digitalRead(KEY2);
}

/**
  * @brief  Motor control (translated)
  * @param  NULL
  * @retval NULL
**/
void motor_control(void)
{
	static uint32_t cur_times=0;
	cur_times=millis();

	//notification signal off (translated)
	if(inform_flag&&cur_times-inform_times>=3000){
		inform_flag=false;
		digitalWrite(EXTENSION_PIN2,HIGH);
		digitalWrite(EXTENSION_PIN1,LOW);
	}

	//button controls motor (translated)
	//key1 short press then release (translated)
	if(key1_release_flag&&millis()-key1_release_times>500){

		key1_release_flag=false;
		//short press once then release (translated)
		if(key1_press_cnt==1){
			digitalWrite(EXTENSION_PIN2,LOW);
			inform_times=millis();
			inform_flag=true;
			is_error=false;
		}
		else if(key1_press_cnt>=2){//short press twice or more then release (translated)
			is_error=true;
		}

		key1_press_cnt=0;
	}

 	//key2 short press then release (translated)
	if(key2_release_flag&&cur_times-key2_release_times>500){

		key2_release_flag=false;
		//short press once then release (translated)
		if(key2_press_cnt==1){
			digitalWrite(EXTENSION_PIN1,HIGH);
			inform_times=millis();
			inform_flag=true;
			is_error=false;
		}
		else if(key2_press_cnt>=2){//short press twice or more then release (translated)
			is_error=true;
		}

		key2_press_cnt=0;

	}

	//key1 long press (KEY1 = reverse/back) (translated)
	if(key1_press_flag&&cur_times-key1_press_times>=500||digitalRead(BACK_SIGNAL_PIN)==LOW)
	{

		WRITE_EN_PIN(0);//enable (translated)
    	driver.VACTUAL(STOP);	//stop (translated)

    	driver.shaft(BACK);
    	driver.VACTUAL(VACTRUAL_VALUE);

		// Check for auto-retract latch: if KEY2 is clicked while KEY1 is held
		while(key1_press_flag||digitalRead(BACK_SIGNAL_PIN)==LOW){
			delay(1);
			g_run_cnt++;
			// Check if KEY2 is pressed to trigger auto-retract
			if(key2_press_flag){
				auto_retract_active=true;
				break;
			}
		}

		// Auto-retract mode: continue until runout sensor triggers (no filament)
		if(auto_retract_active){
			// Clear any pending button states before entering auto mode
			key1_press_flag=false;
			key2_press_flag=false;
			key1_press_cnt=0;
			key2_press_cnt=0;
			while(!digitalRead(ENDSTOP_3)){ // While filament is present (ENDSTOP_3=0)
				delay(1);
				g_run_cnt++;
				// Allow cancellation by clicking either button (detected via press count)
				if(key1_press_cnt > 0 || key2_press_cnt > 0){
					auto_retract_active=false;
					key1_press_cnt=0;
					key2_press_cnt=0;
					break;
				}
			}
			auto_retract_active=false;
		}

		driver.VACTUAL(STOP);	//stop (translated)
		motor_state=Stop;

		is_front=false;
		front_time=0;
		is_error=false;
		WRITE_EN_PIN(1);//disable (translated)
		is_error=true;

	}
	else if(key2_press_flag&&cur_times-key2_press_times>=500||digitalRead(FRONT_SIGNAL_PIN)==LOW)//key2 long press (KEY2 = forward) (translated)
	{

		WRITE_EN_PIN(0);
		driver.VACTUAL(STOP);	//stop (translated)

    	driver.shaft(FORWARD);
		driver.VACTUAL(VACTRUAL_VALUE);

		// Check for auto-feed latch: if KEY1 is clicked while KEY2 is held
		while(key2_press_flag||digitalRead(FRONT_SIGNAL_PIN)==LOW){
			delay(1);
			g_run_cnt++;
			// Check if KEY1 is pressed to trigger auto-feed
			if(key1_press_flag){
				auto_feed_active=true;
				break;
			}
		}

		// Auto-feed mode: continue until buffer position 2 triggers (HALL2)
		if(auto_feed_active){
			// Clear any pending button states before entering auto mode
			key1_press_flag=false;
			key2_press_flag=false;
			key1_press_cnt=0;
			key2_press_cnt=0;
			while(!digitalRead(HALL2)){ // While buffer not at position 2
				delay(1);
				g_run_cnt++;
				// Allow cancellation by clicking either button (detected via press count)
				if(key1_press_cnt > 0 || key2_press_cnt > 0){
					auto_feed_active=false;
					key1_press_cnt=0;
					key2_press_cnt=0;
					break;
				}
			}
			auto_feed_active=false;
		}

		driver.VACTUAL(STOP);	//stop (translated)
		motor_state=Stop;

		is_front=false;
		front_time=0;
		is_error=false;
		WRITE_EN_PIN(1);
	}

	if(connet_mdm_flag){//MDM blockage detection module connected (translated)
		//check filament (translated)
		if(digitalRead(ENDSTOP_3)&&!digitalRead(MDM_DPIN))
		{
			//no filament, stop motor (translated)
			driver.VACTUAL(STOP);	//stop (translated)
			motor_state=Stop;

			//filament runout pin output low (translated)
			digitalWrite(DUANLIAO,0);

			//turn off indicator LED (translated)
			digitalWrite(START_LED,0);

			is_front=false;
			front_time=0;
			is_error=false;
			WRITE_EN_PIN(1);


			return;//no filament, exit (translated)
		}
		else if(!blockage_detect.blockage_flag){
			//filament present, runout pin output high (translated)
			digitalWrite(DUANLIAO,1);

			//turn on indicator LED (translated)
			digitalWrite(START_LED,1);

		}

	}
	else{
		//check filament (translated)
		if(digitalRead(ENDSTOP_3))
		{
			//no filament, stop motor (translated)
			driver.VACTUAL(STOP);	//stop (translated)
			motor_state=Stop;

			//filament runout pin output low (translated)
			digitalWrite(DUANLIAO,0);

			//turn off indicator LED (translated)
			digitalWrite(START_LED,0);

			is_front=false;
			front_time=0;
			is_error=false;
			WRITE_EN_PIN(1);


			return;//no filament, exit (translated)
		}

		//filament present, runout pin output high (translated)
		digitalWrite(DUANLIAO,1);

		//turn on indicator LED (translated)
		digitalWrite(START_LED,1);
	}




	//check for errors (translated)
	if(is_error){
		//stop motor (translated)
		driver.VACTUAL(STOP);	//stop (translated)
		motor_state=Stop;
		WRITE_EN_PIN(1);
		return ;
	}

	//buffer position tracking (translated)
	if(buffer.buffer1_pos1_sensor_state)	//buffer position 1, push filament forward (translated)
	{
		last_motor_state=motor_state;		//record previous state (translated)
		motor_state=Forward;
		is_front=true;

	}
	else if(buffer.buffer1_pos2_sensor_state)	//buffer position 2, motor stops (translated)
	{
		last_motor_state=motor_state;		//record previous state (translated)
		motor_state=Stop;
		is_front=false;
		front_time=0;
	}
	else if(buffer.buffer1_pos3_sensor_state)	//buffer position 3, retract filament (translated)
	{
		last_motor_state=motor_state;		//record previous state (translated)
		motor_state=Back;
		is_front=false;
		front_time=0;
	}

	if(motor_state==last_motor_state)//if state unchanged, no need to send control command, exit function (translated)
		return;
	
	static uint8_t write_cnt=0;
	uint8_t retry_count=9;

	//motor control (translated)
	switch(motor_state)
	{
		case Forward://forward (translated)
		{
			WRITE_EN_PIN(0);
			if(last_motor_state==Back)	driver.VACTUAL(STOP);//was reversing, stop first then go forward (translated)
			driver.shaft(FORWARD);
			write_cnt=driver.IFCNT();
			driver.VACTUAL(VACTRUAL_VALUE);
			while(write_cnt==driver.IFCNT()&&retry_count--){//retry on send failure (translated)
				driver.VACTUAL(VACTRUAL_VALUE);
			}

		}break;
		case Stop://stop (translated)
		{
			write_cnt=driver.IFCNT();
			driver.VACTUAL(STOP);
			while(write_cnt==driver.IFCNT()&&retry_count--){//retry on send failure (translated)
				driver.VACTUAL(STOP);
			}
			WRITE_EN_PIN(1);

		}break;
		case Back://reverse (translated)
		{
			WRITE_EN_PIN(0);
			if(last_motor_state==Forward)	driver.VACTUAL(STOP);;//was forward, stop first then reverse (translated)
			driver.shaft(BACK);
			write_cnt=driver.IFCNT();
			driver.VACTUAL(VACTRUAL_VALUE);
			while(write_cnt==driver.IFCNT()&&retry_count--){//retry on send failure (translated)
				driver.VACTUAL(VACTRUAL_VALUE);
			}
		}break;

	}
	last_motor_state=motor_state;		//record previous state (translated)
	
}

void timer_it_callback(){

	//feed watchdog (every 100ms) (translated)
	static uint32_t i=0;
	i++;
	//check g_run_cnt value every 5 seconds (translated)
	if(i>=50)
	{
		//main program exception detected (translated)
		if(g_run_cnt == 0)
		{
			Serial.println("program excepiton,iwdg trigger reset cpu\r\n");
			//wait for watchdog timeout to trigger reset (translated)
			while(1){
				delay(1);
			}
		}

		g_run_cnt=0;

		i=0;
	}
	// HAL_IWDG_Refresh(&hiwdg);
	IWDG->KR = 0xAAAA;   // equivalent to HAL_IWDG_Refresh(&hiwdg) (translated)





	//long feed time error (translated)
	if (is_front)
	{ // if feeding forward (translated)
		front_time+=100;
		if (front_time > timeout)
		{ // if timeout (translated)
			is_error = true;
		}
	}
}

void key1_it_callback(void){
	if(!digitalRead(KEY1)){//falling edge (translated)
		key1_press_times=millis();
		key1_press_cnt++;
		key1_press_flag=true;
	}
	else{//rising edge (translated)
		if(millis()-key1_press_times<=500){
			key1_release_flag=true;
			key1_release_times=millis();
		}
		else{
			key1_release_flag=false;
			key1_press_cnt=0;

		}
		key1_press_flag=false;
	}
}


void key2_it_callback(void){
	if(!digitalRead(KEY2)){//falling edge (translated)
		key2_press_times=millis();
		key2_press_cnt++;
		key2_press_flag=true;
	}
	else{//rising edge (translated)
		if(millis()-key2_press_times<=500){
			key2_release_flag=true;
			key2_release_times=millis();

		}
		else{
			key2_release_flag=false;
			key2_press_cnt=0;
		}
		key2_press_flag=false;
	}
}

void Recv_MDM_Pulse_IT_Callback(void){
	blockage_detect.mdm_pulse_cnt++;
	// Serial.println("mdm_pulse_cnt:"+String(blockage_detect.mdm_pulse_cnt));
}

void Dir_IT_Callback(void){
	//change timer count direction (translated)
	if(SIGNAL_COUNT_READ_DIR_IO())	SIGNAL_COUNT_UP();		//DIR high - configure count up (translated)
	else 	SIGNAL_COUNT_DOWN();	//DIR low - configure count down (translated)
}

// void Buffer_S3_IT_Callback(void){
// 	last_motor_state=motor_state;		//record previous state (translated)
// 	motor_state=Back;
// 	is_front=false;
// 	front_time=0;
// }

// void Buffer_S2_IT_Callback(void){
// 	last_motor_state=motor_state;		//record previous state (translated)
// 	motor_state=Stop;
// 	is_front=false;
// 	front_time=0;
// }

// void Buffer_S1_IT_Callback(void){
// 	last_motor_state=motor_state;		//record previous state (translated)
// 	motor_state=Forward;
// 	is_front=true;

// }



void buffer_debug(void){
	// Serial.print("buffer1_pos1_sensor_state:");Serial.println(buffer.buffer1_pos1_sensor_state);
	// Serial.print("buffer1_pos2_sensor_state:");Serial.println(buffer.buffer1_pos2_sensor_state);
	// Serial.print("buffer1_pos3_sensor_state:");Serial.println(buffer.buffer1_pos3_sensor_state);
	// Serial.print("buffer1_material_swtich_state:");Serial.println(buffer.buffer1_material_swtich_state);
	// Serial.print("key1:");Serial.println(buffer.key1);
	// Serial.print("key2:");Serial.println(buffer.key2);
	static int i=0;
	if(i<0x1ff){
		Serial.print("i:");
		Serial.println(i);
		driver.GCONF(i);
		driver.PWMCONF(i);
		i++;
	}
	uint32_t gconf = driver.GCONF();
	uint32_t chopconf=driver.CHOPCONF();
	uint32_t pwmconf = driver.PWMCONF();
	if(driver.CRCerror){
		Serial.println("CRCerror");
	}
	else{
		Serial.print("GCONF():0x");
		Serial.println(gconf,HEX);
		Serial.print("CHOPCONF():0x");
		char buf[11];  // "0x" + 8 digits + null terminator
		sprintf(buf, "%08lX", chopconf);  // %08lX -> 8位大写十六进制（long unsigned）
		Serial.println(buf);
		Serial.print("PWMCONF():0x");
		sprintf(buf, "%08lX", pwmconf);  // %08lX -> 8位大写十六进制（long unsigned）
		Serial.println(buf);
		Serial.println("");
	}
  	delay(1000);
}


/**
  * @brief  USB serial receive parsing (translated)
  * @param  null
  * @retval null
**/
void USB_Serial_Analys(void){

	static String serial_buf;
	if(Serial.available()){
		char c=Serial.read();
		if(c=='\n'){
			if(strstr(serial_buf.c_str(),"rt")){
				Serial.print("read timeout=");
				Serial.println(timeout);				
			}
			else if(strstr(serial_buf.c_str(),"timeout")){
				int index=serial_buf.indexOf(" ");
				if(index==-1){
					serial_buf="";
					Serial.println("Error: Invalid timeout value.");
				}
				serial_buf=serial_buf.substring(index+1);
				int64_t num=serial_buf.toInt();
				if(num<0||num>0xffffffff){
					serial_buf="";
					Serial.println("Error: Invalid timeout value.");
				}
				timeout=num;
				EEPROM.put(EEPROM_ADDR_TIMEOUT, timeout);
				serial_buf="";
				Serial.print("set timeout succeed! timeout=");
				Serial.println(timeout);

			}
			else if(strstr(serial_buf.c_str(),"steps")){
				int index=serial_buf.indexOf(" ");
				if(index==-1){
					serial_buf="";
					Serial.println("Error: Invalid steps value.");
				}
				serial_buf=serial_buf.substring(index+1);
				int64_t num=serial_buf.toInt();
				if(num<0||num>51200){
					serial_buf="";
					Serial.println("Error: Invalid steps value.");
				}
				steps=num;
				EEPROM.put(EEPROM_ADDR_STEPS, steps);
				serial_buf="";
				Serial.print("set steps succeed! steps=");
				Serial.println(steps);
				REIN_TIM_SIGNAL_COUNT_DeInit();
				REIN_TIM_SIGNAL_COUNT_Init();
			}		
			else if(strstr(serial_buf.c_str(),"clear")){
				//reset counter (translated)
				blockage_detect.actual_distance=0;
				blockage_detect.target_distance=0;
				blockage_detect.mdm_pulse_cnt=0;
				blockage_detect.pulse_cnt=0;				
			}
			else if(strstr(serial_buf.c_str(),"encoder")){
				int index=serial_buf.indexOf(" ");
				if(index==-1){
					serial_buf="";
					Serial.println("Error: Invalid encoder value.");
				}
				serial_buf=serial_buf.substring(index+1);
				// float num = serial_buf.toFloat();
				float num = fastAtof(serial_buf.c_str());
				if(num<0){
					serial_buf="";
					Serial.println("Error: Invalid encoder length value.");
				}
				encoder_length=num;
				EEPROM.put(EEPROM_ADDR_ENCODER_LENGTH, encoder_length);
				serial_buf="";
				blockage_detect.allow_error = encoder_length*allow_error_scale;
				Serial.print("set encoder length succeed! encoder_length=");
				Serial.println(encoder_length);
			}
			else if(strstr(serial_buf.c_str(),"info")){
				Serial.println("encoder_length="+String(encoder_length));
				Serial.println("timeout="+String(timeout));
				Serial.println("steps="+String(steps));
				Serial.println("allow_error_scale="+String(allow_error_scale));
				Serial.println("allow_error="+String(blockage_detect.allow_error));
			}			
			else if(strstr(serial_buf.c_str(),"scale")){
				int index=serial_buf.indexOf(" ");
				if(index==-1){
					serial_buf="";
					Serial.println("Error: Invalid scale value.");
				}
				serial_buf=serial_buf.substring(index+1);
				// float num = serial_buf.toFloat();
				float num = fastAtof(serial_buf.c_str());
				if(num<0){
					serial_buf="";
					Serial.println("Error: Invalid scale length value.");
				}
				allow_error_scale=num;
				EEPROM.put(EEPROM_ADDR_ERROR_SCALE, allow_error_scale);
				serial_buf="";
				blockage_detect.allow_error = encoder_length*allow_error_scale;
				Serial.print("set scale length succeed! allow_error_scale=");
				Serial.println(allow_error_scale);

			}
			else if(strstr(serial_buf.c_str(),"speed")){
				int index=serial_buf.indexOf(" ");
				if(index==-1){
					serial_buf="";
					Serial.println("speed: "+String(SPEED));
					return ;
				}
				serial_buf=serial_buf.substring(index+1);
				// float num = serial_buf.toFloat();
				float num = fastAtof(serial_buf.c_str());
				if(num<0){
					serial_buf="";
					Serial.println("Error: Invalid speed  value.");
				}
				SPEED=num;
				VACTRUAL_VALUE=(uint32_t)(SPEED*Move_Divide_NUM*200/60/0.715) ;  //VACTUAL register value (translated)
				EEPROM.put(EEPROM_ADDR_SPEED, SPEED);
				serial_buf="";
				Serial.print("set speed  succeed! speed=");
				Serial.println(SPEED);
			}			


			else{
				Serial.println(serial_buf.c_str());
				Serial.println("command error!");
				Serial.print("\n+---------------------------------------------+\n");
				Serial.print("|         Fly Buffer Command Set              |\n");
				Serial.print("|     set steps per mm: <steps nnn CRLF>      |\n");
				Serial.print("|     set encoder length: <encoder nnn CRLF>  |\n");
				Serial.print("|     set timeout : <timeout nnn CRLF>        |\n");
				Serial.print("|     read timeout: <rt CRLF>                 |\n");
				Serial.print("|     show all info : <info CRLF>             |\n");
				Serial.print("|     set scale: <scale nnn CRLF>             |\n");
				Serial.print("|     set speed(r/min): <speed nnn CRLF>      |\n");
				Serial.print("+-----------------------------------------------+\n\n");
			}
			serial_buf="";

		}
		else  serial_buf+=c;
	}
}



/**
  * @brief  Check if MDM blockage detection module is connected (translated)
  * @param  null
  * @retval true: connected, false: not connected (translated)
**/
bool Check_Connet_MDM(void){

	//wait for power stabilization (translated)
	delay(1000);

	//read MDM filament pin state (translated)
	pinMode(MDM_DPIN,INPUT);
	bool mdm_state=digitalRead(MDM_DPIN);
	// Serial.print("mdm_state:");
	// Serial.println(mdm_state);

	//configure opposite pull direction and re-read, if level unchanged then connected, otherwise not connected (translated)
	if(mdm_state){//high level, configure pull-down (translated)
		pinMode(MDM_DPIN,INPUT_PULLDOWN);
		Serial.println(digitalRead(MDM_DPIN));
		if(digitalRead(MDM_DPIN))	return true;//level unchanged, connected (translated)
		else 						return false;
	}
	else{
		pinMode(MDM_DPIN,INPUT_PULLUP);
		Serial.println(digitalRead(MDM_DPIN));
		if(digitalRead(MDM_DPIN))	return false;
		else 						return true;//level unchanged, connected (translated)
	}
}

/**
 * @brief  TIM_SIGNAL_PUL initialization (translated)
 * @param  NULL
 * @retval NULL
 **/
void REIN_TIM_SIGNAL_COUNT_Init(void)
{
	/* GPIO initialization (translated) */
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	/* GPIO Ports Clock Enable*/
	SIGNAL_COUNT_PUL_CLK_ENABLE(); // enable SIGNAL_COUNT_PUL port clock (translated)
	/*Configure GPIO pin*/
	GPIO_InitStruct.Pin = SIGNAL_COUNT_PUL_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; // alternate function push-pull mode (translated)
	GPIO_InitStruct.Pull = GPIO_NOPULL;		// disable pull-up/pull-down (translated)
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM2;
	HAL_GPIO_Init(SIGNAL_COUNT_PUL_GPIO_Port, &GPIO_InitStruct);

	/* TIM initialization (translated) */
	TIM_SlaveConfigTypeDef sSlaveConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	SIGNAL_COUNT_TIM_CLK_ENABLE(); // enable TIM clock (translated)
	SIGNAL_COUNT_Get_HTIM.Instance = SIGNAL_COUNT_Get_TIM;
	SIGNAL_COUNT_Get_HTIM.Init.Prescaler = 0;									   // prescaler: 0 (translated)
	SIGNAL_COUNT_Get_HTIM.Init.CounterMode = TIM_COUNTERMODE_UP;				   // count up (translated)
	SIGNAL_COUNT_Get_HTIM.Init.Period = 65536-1;									   // counter period (translated)
	SIGNAL_COUNT_Get_HTIM.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;			   // no division (translated)
	SIGNAL_COUNT_Get_HTIM.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE; // disable auto reload (translated)
	if (HAL_TIM_Base_Init(&SIGNAL_COUNT_Get_HTIM) != HAL_OK)
	{
		Error_Handler();
	}
	sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;		   // external clock mode (translated)
	sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;				   // TI1FP1
	sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_RISING; // rising edge trigger (translated)
	sSlaveConfig.TriggerFilter = 4;							   // filter parameter (FDIV2_N6) (translated)
	if (HAL_TIM_SlaveConfigSynchro(&SIGNAL_COUNT_Get_HTIM, &sSlaveConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;			 // master mode trigger reset (translated)
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE; // disable master mode (translated)
	if (HAL_TIMEx_MasterConfigSynchronization(&SIGNAL_COUNT_Get_HTIM, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/*begin work*/
	// HAL_TIM_Base_Start_IT(&SIGNAL_COUNT_Get_HTIM);
	HAL_TIM_Base_Start(&SIGNAL_COUNT_Get_HTIM);

}

/**
  * @brief  TIM_SIGNAL_COUNT cleanup (translated)
  * @param  NULL
  * @retval NULL
**/
void REIN_TIM_SIGNAL_COUNT_DeInit(void)
{
	// HAL_TIM_Base_Stop_IT(&SIGNAL_COUNT_Get_HTIM);										//stop TIM (translated)
	HAL_TIM_Base_Stop(&SIGNAL_COUNT_Get_HTIM);										//stop TIM (translated)
	HAL_GPIO_DeInit(SIGNAL_COUNT_PUL_GPIO_Port, SIGNAL_COUNT_PUL_Pin);	//reset GPIO (translated)
}

/**
  * @brief  GPIO initialization (SIGNAL_COUNT) (translated)
  * @param  NULL
  * @retval NULL
*/
void Signal_Dir_Init(void)
{
	pinMode(BLOCKAGE_DIR_PIN,INPUT);
	attachInterrupt(BLOCKAGE_DIR_PIN,&Dir_IT_Callback,CHANGE);
}


/**
  * @brief  Pulse receive initialization (translated)
  * @param  null
  * @retval null
**/
void Pulse_Receive_Init(void){

	EEPROM.get(EEPROM_ADDR_ERROR_SCALE, allow_error_scale);
	// Check if value is valid (e.g., before first write it may be 0xFFFFFFFF or 0) (translated)
	if (allow_error_scale == 0||isnan(allow_error_scale))
	{
		allow_error_scale = DEFAULT_ALLOW_ERROR_SCALE;
		EEPROM.put(EEPROM_ADDR_ERROR_SCALE, allow_error_scale);
		Serial.println("EEPROM is empty");
	}
	else
	{
		Serial.print("read allow_error_scale: ");
		Serial.println(allow_error_scale);
	}




	// PULSE2_PIN initialization (translated)
	pinMode(PULSE2_PIN,INPUT);
	attachInterrupt(PULSE2_PIN,&Recv_MDM_Pulse_IT_Callback,RISING);
	EEPROM.get(EEPROM_ADDR_ENCODER_LENGTH, encoder_length);
	// Check if value is valid (e.g., before first write it may be 0xFFFFFFFF or 0) (translated)
	if (encoder_length == 0||isnan(encoder_length))
	{
		encoder_length = DEFAULT_ENCODER_LENGTH;
		EEPROM.put(EEPROM_ADDR_ENCODER_LENGTH, encoder_length);
		Serial.println("EEPROM is empty");
	}
	else
	{
		Serial.print("read encoder_length: ");
		Serial.println(encoder_length);
	}



	// PULSE1_PIN initialization (translated)
	// use hardware timer to receive (translated)
	EEPROM.get(EEPROM_ADDR_STEPS, steps);
	// Check if value is valid (e.g., before first write it may be 0xFFFFFFFF or 0) (translated)
	if (steps > 51200 || timeout == 0)
	{
		steps = DEFAULT_STEPS;
		EEPROM.put(EEPROM_ADDR_STEPS, steps);
		Serial.println("EEPROM is empty");
	}
	else
	{
		Serial.print("read steps: ");
		Serial.println(steps);
	}
	
	REIN_TIM_SIGNAL_COUNT_Init();

	blockage_detect.allow_error = encoder_length*allow_error_scale;
}


void Blockage_Detect(void){

	static uint32_t last_target_distance=blockage_detect.target_distance;
	static uint32_t last_timer_cnt=TIM2->CNT;
	static uint32_t last_time=0;

	//if no pulse received within 500ms, reset counter (translated)
	if(last_timer_cnt==TIM2->CNT){//CNT unchanged, no pulse (translated)
		if(millis()-last_time>=500){
			blockage_detect.actual_distance=0;
			blockage_detect.target_distance=0;
			blockage_detect.mdm_pulse_cnt=0;
			blockage_detect.extrusion_pulse_cnt=0;
		}
	}
	else{//CNT changed, pulse received, record timestamp (translated)
		last_timer_cnt=TIM2->CNT;
		last_time=millis();

		//calculate extrusion pulse count (translated)
		blockage_detect.last_pulse_cnt=blockage_detect.pulse_cnt;
		blockage_detect.pulse_cnt=TIM2->CNT;
		blockage_detect.pulse_cnt_sub=blockage_detect.pulse_cnt-blockage_detect.last_pulse_cnt;
		blockage_detect.extrusion_pulse_cnt+=blockage_detect.pulse_cnt_sub;


		//only calculate extrusion distance, if extrusion_pulse_cnt is negative, treat as retract, don't calculate (translated)
		if(blockage_detect.extrusion_pulse_cnt<0) blockage_detect.target_distance=0;
		else blockage_detect.target_distance=(blockage_detect.extrusion_pulse_cnt)/steps;

		// Serial.println("extrusion_pulse_cnt:"+String(blockage_detect.extrusion_pulse_cnt));
	}

	blockage_detect.actual_distance=blockage_detect.mdm_pulse_cnt*encoder_length;//actual distance (translated)
	blockage_detect.distance_error=blockage_detect.actual_distance-blockage_detect.target_distance;//distance difference (translated)




	static bool detect_blockage=false;
	static uint32_t detect_blockage_time=0;


	//if blockage detected twice consecutively, confirm blockage, otherwise false trigger (translated)
	if(!detect_blockage){//blockage not yet detected (translated)
		if(blockage_detect.target_distance!=last_target_distance){
			// Serial.println("dir:"+String((bool)SIGNAL_COUNT_READ_DIR_IO()));

			// Serial.print("target_distance:"+String(blockage_detect.target_distance));
			// Serial.println("	actual_distance:"+String(blockage_detect.actual_distance));

			//blockage check (translated)
			if(abs(blockage_detect.distance_error)>blockage_detect.allow_error&&blockage_detect.target_distance>=blockage_detect.allow_error){//blockage detected (translated)

				detect_blockage=true;
				detect_blockage_time=millis();

				// Serial.print("target_distance:"+String(blockage_detect.target_distance));
				// Serial.println("	actual_distance:"+String(blockage_detect.actual_distance));
				// Serial.println("detect over error:"+String(blockage_detect.distance_error));

				//reset counter (translated)
				blockage_detect.actual_distance=0;
				blockage_detect.target_distance=0;
				blockage_detect.mdm_pulse_cnt=0;
				blockage_detect.extrusion_pulse_cnt=0;
			}

			last_target_distance=blockage_detect.target_distance;
		}

	}
	else if(blockage_detect.target_distance>blockage_detect.allow_error){//blockage detected, recheck after 100ms, if still blocked trigger alarm, otherwise false trigger, clear flag (translated)
		if(millis()-detect_blockage_time>=100){
			if(abs(blockage_detect.distance_error)>blockage_detect.allow_error){//blockage detected (translated)
				//blockage triggered (translated)
				// Serial.println("blockage trigger");
				blockage_detect.blockage_flag=true;
				blockage_inform_times=millis();
				digitalWrite(DULIAO,LOW);
				detect_blockage=false;
				detect_blockage_time=0;

				//reset counter (translated)
				blockage_detect.actual_distance=0;
				blockage_detect.target_distance=0;
				blockage_detect.mdm_pulse_cnt=0;
				blockage_detect.extrusion_pulse_cnt=0;
			}
			else{//blockage not detected, false trigger (translated)
				//clear detection flag (translated)
				detect_blockage=false;
				detect_blockage_time=0;
			}
		}

	}

}


float fastAtof(const char *s) {
    float val = 0.0f;
    int sign = 1;

    if (*s == '-') { sign = -1; s++; }
    else if (*s == '+') { s++; }

    // integer part (translated)
    while (*s >= '0' && *s <= '9') {
        val = val * 10.0f + (*s - '0');
        s++;
    }

    // decimal part (translated)
    if (*s == '.') {
        s++;
        float frac = 1.0f;
        while (*s >= '0' && *s <= '9') {
            frac *= 0.1f;
            val += (*s - '0') * frac;
            s++;
        }
    }

    return sign * val;
}