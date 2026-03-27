/**
  ***************************************************************************************
  * @file    buffer.cpp
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
  * COPYRIGHT 2025 xxx@126.com
  ***************************************************************************************
**/


#include "buffer.h"
#define VERSION "1.1.5"

//GPIO input
#define SIGNAL_COUNT_READ_DIR_IO()	(SIGNAL_COUNT_DIR_GPIO_Port -> IDR & SIGNAL_COUNT_DIR_Pin)
//TIM input
#define SIGNAL_COUNT_READ_COUNT()		(SIGNAL_COUNT_Get_TIM -> CNT)
//TIM output
#define SIGNAL_COUNT_UP()						(SIGNAL_COUNT_Get_TIM -> CR1 &= ~(TIM_CR1_DIR))
#define SIGNAL_COUNT_DOWN()					(SIGNAL_COUNT_Get_TIM -> CR1 |=  (TIM_CR1_DIR))




TMC2209Stepper driver(UART, UART, R_SENSE, DRIVER_ADDRESS);
Buffer buffer={0};//stores sensor states
Motor_State motor_state=Stop;
static Motor_State last_motor_state=Stop;

bool is_front=false;//forward movement flag
uint32_t front_time=0;//forward feed time
const uint32_t DEFAULT_TIMEOUT = 60000;
uint32_t timeout=60000;//timeout in ms
bool is_error=false;//error flag, set if feeding continuously for 60s without stopping

static HardwareTimer timer(TIM6);//timeout error timer
TIM_HandleTypeDef htim2;//hardware timer for pulse reception

bool key1_press_flag=false;
bool key2_press_flag=false;
uint32_t key1_press_times=0;
uint32_t key2_press_times=0;

// Device state machine
typedef enum {
	DS_Empty,            // No filament, motor off
	DS_PrimingForward,   // Advancing filament to distal switch
	DS_Primed,           // Both switches closed, motor off, filament in gears
	DS_Loaded,           // Normal buffer operation (hall sensors)
	DS_Retracting,       // Backing out from Loaded, waiting for proximal open
	DS_Repriming,        // Re-advancing after retraction, waiting for distal trigger
	DS_Unloading,        // Backing out from Primed to Empty
	DS_Halted,           // Motor stopped, waiting for button input
	DS_StartupProbe,     // Retracting on boot to determine Primed vs Loaded
	DS_DeadmanForward,   // Override: motor forward while held
	DS_DeadmanBack       // Override: motor backward while held
} DeviceState;

static DeviceState device_state = DS_Empty;

static const char* state_name(DeviceState s) {
	switch(s) {
		case DS_Empty: return "Empty";
		case DS_PrimingForward: return "PrimingForward";
		case DS_Primed: return "Primed";
		case DS_Loaded: return "Loaded";
		case DS_Retracting: return "Retracting";
		case DS_Repriming: return "Repriming";
		case DS_Unloading: return "Unloading";
		case DS_Halted: return "Halted";
		case DS_StartupProbe: return "StartupProbe";
		case DS_DeadmanForward: return "DeadmanFwd";
		case DS_DeadmanBack: return "DeadmanBack";
		default: return "Unknown";
	}
}

// Persisted device state values (stored in buffer_para.saved_device_state)
const uint8_t SAVED_STATE_UNKNOWN = 0;
const uint8_t SAVED_STATE_PRIMED = 1;
const uint8_t SAVED_STATE_LOADED = 2;

const uint32_t DEFAULT_STEPS = 916;
uint32_t steps=916;//pulses per mm
BlockageDetect blockage_detect={0};//blockage detection struct
bool connet_mdm_flag=false;
uint32_t blockage_inform_times=0;


const float DEFAULT_ENCODER_LENGTH = 1.73;
float encoder_length=1.73;//MDM module filament movement per pulse (mm/pulse)


const float DEFAULT_ALLOW_ERROR_SCALE = 2;
float allow_error_scale=2;//allowed error scale factor

uint32_t I_CURRENT = 500;		//motor current (mA)

const uint32_t DEFAULT_COAST_DELAY = 500;
uint32_t coast_delay = 500;	//ms to keep coils energized after buffer stop




//independent watchdog (IWDG)
#include "stm32f0xx_hal_iwdg.h"

IWDG_HandleTypeDef hiwdg;
static volatile uint32_t g_run_cnt=0;
bool DUANLIAO_OUT_STATE = 0; //filament runout output state
Buffer_Parameter buffer_para;

void iwdg_init(void)
{
	// 1. Enable write access
	IWDG->KR = 0x5555;

	// 2. Set prescaler
	IWDG->PR = IWDG_PRESCALER_256;

	// 3. Set reload value
	IWDG->RLR = 125*10-1;  //2s timeout

	// 4. Start IWDG
	IWDG->KR = 0xCCCC;
}



//function declarations
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

void buffer_parameter_init(Buffer_Parameter &buffer_para){
	EEPROM.get(0, buffer_para);
	if(buffer_para.magic_number!=0x55AB){
		buffer_para={0x55AB,DEFAULT_TIMEOUT,DEFAULT_STEPS,DEFAULT_ENCODER_LENGTH,DEFAULT_ALLOW_ERROR_SCALE,260,I_CURRENT,DUANLIAO_OUT_STATE,DEFAULT_COAST_DELAY,SAVED_STATE_UNKNOWN};
		EEPROM.put(0, buffer_para);
	}
	timeout=buffer_para.timeout;
	steps=buffer_para.steps;
	encoder_length=buffer_para.encoder_length;
	allow_error_scale=buffer_para.allow_error_scale;
	SPEED=buffer_para.SPEED;
	I_CURRENT=buffer_para.I_CURRENT;
	DUANLIAO_OUT_STATE=buffer_para.DUANLIAO_OUT_STATE;
	coast_delay=buffer_para.coast_delay;
}

void buffer_init(){

	NVIC_SetPriority(TIM6_DAC_IRQn,0);//timeout error + watchdog
	// NVIC_SetPriority(TIM2_IRQn,1);//high-frequency pulse reception
	NVIC_SetPriority(EXTI4_15_IRQn,1);//KEY1, KEY2, dir, blockage pulses

	if(Check_Connet_MDM()){
		connet_mdm_flag=true;
		Serial.println("MDM connected");
	}
	else{
		connet_mdm_flag=false;
		Serial.println("MDM not connected");
		
	}

  buffer_parameter_init(buffer_para);
  Pulse_Receive_Init();
  buffer_sensor_init();
  buffer_motor_init();
  Signal_Dir_Init();
  delay(1000);

  VACTRUAL_VALUE=(uint32_t)(SPEED*Move_Divide_NUM*200/60/0.715f) ;  //VACTUAL register value



  timer.pause();
  timer.setPrescaleFactor(4800);//4800 prescaler: 48000000/4800=10000Hz
  timer.setOverflow(1000);//100ms
  timer.attachInterrupt(&timer_it_callback);
  timer.resume();

  iwdg_init();
}

void buffer_loop()
{
	uint32_t lastToggleTime = 0; // last toggle timestamp
	
	while (1)
	{
		uint32_t nowTime=millis();
		if (blockage_detect.blockage_flag &&nowTime - lastToggleTime >= 50)
		{							   // blockage detected, rapid flash
			lastToggleTime = millis();
			digitalToggle(ERR_LED);
			
		}
		else{
			if(connet_mdm_flag){//MDM connected, blink twice per second
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
			else if(millis() - lastToggleTime >= 500) //no MDM connection, blink once per second
			{
				lastToggleTime = millis();
				digitalToggle(ERR_LED);
			}


		}
		// 1. Read sensor values
		read_sensor_state();
		if(connet_mdm_flag) Blockage_Detect();
		motor_control();
		USB_Serial_Analys();

		// Turn off blockage output after 3s
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
  //sensor initialization
  pinMode(HALL1,INPUT);
  pinMode(HALL2,INPUT);
  pinMode(HALL3,INPUT);
  pinMode(ENDSTOP_3,INPUT);
  pinMode(DISTAL_SWITCH,INPUT_PULLUP); // distal filament switch on PB14; HIGH=absent, LOW=present


  pinMode(KEY1,INPUT);
  pinMode(KEY2,INPUT);

  attachInterrupt(KEY1,&key1_it_callback,CHANGE);
  attachInterrupt(KEY2,&key2_it_callback,CHANGE);

  //indicator LED initialization
  pinMode(DUANLIAO,OUTPUT);
  pinMode(ERR_LED,OUTPUT);
  pinMode(START_LED,OUTPUT);
  pinMode(DULIAO,OUTPUT);

  //extension pin initialization
  pinMode(EXTENSION_PIN1,OUTPUT);
  pinMode(EXTENSION_PIN2,OUTPUT);

  digitalWrite(EXTENSION_PIN1,LOW);
  digitalWrite(EXTENSION_PIN2,HIGH);
  digitalWrite(DULIAO,HIGH);
  digitalWrite(DUANLIAO,HIGH);

  //pull-up input for signal control; LOW triggers forward/reverse action
  pinMode(FRONT_SIGNAL_PIN,INPUT_PULLUP);
  pinMode(BACK_SIGNAL_PIN,INPUT_PULLUP);

}

void buffer_motor_init(){

  //motor driver pin initialization
  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);      // Enable driver in hardware

  //motor driver initialization
  driver.begin();                  // UART: Init SW UART (if selected) with default 115200 baudrate
  driver.beginSerial(9600);
  driver.I_scale_analog(false);
  driver.toff(5);                 // Enables driver in software
  driver.rms_current(I_CURRENT);        // Set motor RMS current
  driver.microsteps(Move_Divide_NUM);          // Set microsteps to 1/16th
  driver.VACTUAL(STOP);           // Set velocity
  driver.en_spreadCycle(true);
  driver.pwm_autoscale(true);

  digitalWrite(EN_PIN, HIGH);     // Disable driver until state machine needs it
}

/**
  * @brief  Read all sensor states
  * @param  NULL
  * @retval NULL
**/
void read_sensor_state(void)
{
	buffer.buffer1_pos1_sensor_state= digitalRead(HALL3);
	buffer.buffer1_pos2_sensor_state= digitalRead(HALL2);	
	buffer.buffer1_pos3_sensor_state= digitalRead(HALL1);		
	buffer.buffer1_material_swtich_state=digitalRead(ENDSTOP_3);
	buffer.distal_switch_state=digitalRead(DISTAL_SWITCH);
	buffer.key1=digitalRead(KEY1);
	buffer.key2=digitalRead(KEY2);
}

// --- Motor command helpers ---

static void cmd_motor_forward() {
	WRITE_EN_PIN(0);
	if(motor_state == Back) driver.VACTUAL(STOP);
	driver.shaft(FORWARD);
	uint8_t cnt = driver.IFCNT();
	driver.VACTUAL(VACTRUAL_VALUE);
	uint8_t retries = 9;
	while(cnt == driver.IFCNT() && retries--) {
		driver.VACTUAL(VACTRUAL_VALUE);
	}
	motor_state = Forward;
	last_motor_state = Forward;
}

static void cmd_motor_back() {
	WRITE_EN_PIN(0);
	if(motor_state == Forward) driver.VACTUAL(STOP);
	driver.shaft(BACK);
	uint8_t cnt = driver.IFCNT();
	driver.VACTUAL(VACTRUAL_VALUE);
	uint8_t retries = 9;
	while(cnt == driver.IFCNT() && retries--) {
		driver.VACTUAL(VACTRUAL_VALUE);
	}
	motor_state = Back;
	last_motor_state = Back;
}

// Soft stop: VACTUAL=0 but EN stays LOW (coils stay energized with hold current).
// Use for hall sensor buffer cycling to avoid audible click from EN toggling.
static void cmd_motor_coast() {
	uint8_t cnt = driver.IFCNT();
	driver.VACTUAL(STOP);
	uint8_t retries = 3;
	while(cnt == driver.IFCNT() && retries--) {
		driver.VACTUAL(STOP);
	}
	motor_state = Stop;
	last_motor_state = Stop;
}

// Full stop: VACTUAL=0 and EN HIGH (coils de-energized).
// Use for state transitions, errors, and safety shutdowns.
static void cmd_motor_stop() {
	cmd_motor_coast();
	WRITE_EN_PIN(1); // Disable driver via EN pin (hardware kill, works even if UART fails)
}

/**
  * @brief  Motor control state machine
  *
  * States: Empty → PrimingForward → Primed → Loaded → Retracting → Repriming → Primed
  *
  * Buttons (KEY1=back, KEY2=forward):
  *   Short press: state transition (or halt if in transition)
  *   Hold 2s: deadman switch (motor runs while held)
  *
  * @param  NULL
  * @retval NULL
**/
void motor_control(void)
{
	// Read switches from cached sensor state (both sampled at same time in read_sensor_state)
	// true = filament present
	bool proximal = !buffer.buffer1_material_swtich_state;
	bool distal = !buffer.distal_switch_state;

	// Determine initial state on first run (NEVER advance filament on startup)
	static bool first_run = true;
	if(first_run) {
		first_run = false;
		uint8_t last_saved_state = buffer_para.saved_device_state;

		if(proximal && distal) {
			if(last_saved_state == SAVED_STATE_PRIMED) {
				// Previously Primed — trust it, no motor movement
				device_state = DS_Primed;
			} else if(last_saved_state == SAVED_STATE_LOADED) {
				// Previously Loaded — resume buffer operation (hall sensors manage slack)
				// Safe: not advancing new filament, just resuming buffer management
				device_state = DS_Loaded;
			} else if(buffer.buffer1_pos1_sensor_state) {
				// Unknown state but buffer is retracted — default to Primed (safe, motor off)
				device_state = DS_Primed;
			} else {
				// Unknown state, buffer not retracted — probe to determine
				device_state = DS_StartupProbe;
			}
		} else if(proximal && !distal) {
			device_state = DS_Halted; // Don't auto-advance on startup
		} else {
			device_state = DS_Empty;
		}
	}

	// --- Button edge detection (press = falling edge, release = rising edge) ---
	static bool key1_prev = false, key2_prev = false;
	bool key1_held = key1_press_flag;
	bool key2_held = key2_press_flag;
	bool key1_just_pressed = key1_held && !key1_prev;
	bool key2_just_pressed = key2_held && !key2_prev;
	bool key1_just_released = !key1_held && key1_prev;
	bool key2_just_released = !key2_held && key2_prev;
	key1_prev = key1_held;
	key2_prev = key2_held;

	// Track if a press was consumed during a transition (prevents release from triggering a new transition)
	static bool key1_consumed = false, key2_consumed = false;
	if(key1_just_released) key1_consumed = false;
	if(key2_just_released) key2_consumed = false;

	// --- Deadman switch detection (2s hold, only one at a time) ---
	// Capture state at press time (before back_press/fwd_press transitions fire),
	// so deadman has the correct pre-press state even though the initial press edge
	// may have already triggered a state transition.
	static DeviceState state_at_press = DS_Empty;
	static DeviceState pre_deadman_state = DS_Empty;
	if(key1_just_pressed || key2_just_pressed) {
		state_at_press = device_state;
	}
	if(device_state != DS_DeadmanBack && device_state != DS_DeadmanForward &&
	   key1_held && !key2_held && millis() - key1_press_times >= 2000) {
		pre_deadman_state = state_at_press;
		cmd_motor_back();
		is_front = false;
		key1_consumed = true;
		device_state = DS_DeadmanBack;
	}
	if(device_state != DS_DeadmanForward && device_state != DS_DeadmanBack &&
	   key2_held && !key1_held && millis() - key2_press_times >= 2000) {
		pre_deadman_state = state_at_press;
		cmd_motor_forward();
		is_front = false;
		key2_consumed = true;
		device_state = DS_DeadmanForward;
	}

	// Handle deadman release → determine state from sensors
	// Only restore pre-deadman state for Loaded (indistinguishable from Primed by sensors alone)
	if(device_state == DS_DeadmanBack && key1_just_released) {
		cmd_motor_stop();
		is_front = false;
		front_time = 0;
		if(!proximal && !distal)                                device_state = DS_Empty;
		else if(proximal && distal && pre_deadman_state == DS_Loaded) device_state = DS_Loaded;
		else if(proximal && distal)                             device_state = DS_Primed;
		else                                                    device_state = DS_Halted;
		return;
	}
	if(device_state == DS_DeadmanForward && key2_just_released) {
		cmd_motor_stop();
		is_front = false;
		front_time = 0;
		if(!proximal && !distal)                                device_state = DS_Empty;
		else if(proximal && distal && pre_deadman_state == DS_Loaded) device_state = DS_Loaded;
		else if(proximal && distal)                             device_state = DS_Primed;
		else                                                    device_state = DS_Halted;
		return;
	}

	// Deadman active: keep motor running, skip state machine
	if(device_state == DS_DeadmanBack || device_state == DS_DeadmanForward) {
		return;
	}

	// --- Button press events (on press, not release) ---
	// Only fire if not already consumed and cooldown elapsed (prevents multi-state jumps from mashing)
	static uint32_t last_button_action = 0;
	const uint32_t BUTTON_COOLDOWN = 300; // ms between accepted button events
	uint32_t now = millis();
	bool cooled = (now - last_button_action >= BUTTON_COOLDOWN);
	bool back_press = key1_just_pressed && !key1_consumed && cooled;
	bool fwd_press = key2_just_pressed && !key2_consumed && cooled;
	if(back_press || fwd_press) last_button_action = now;

	// --- PB5/PB6 external signal control (blocking deadman) ---
	if(digitalRead(BACK_SIGNAL_PIN) == LOW) {
		cmd_motor_back();
		while(digitalRead(BACK_SIGNAL_PIN) == LOW) { delay(1); g_run_cnt++; }
		cmd_motor_stop();
		is_front = false;
		front_time = 0;
		device_state = DS_Halted;
		return;
	}
	if(digitalRead(FRONT_SIGNAL_PIN) == LOW) {
		cmd_motor_forward();
		while(digitalRead(FRONT_SIGNAL_PIN) == LOW) { delay(1); g_run_cnt++; }
		cmd_motor_stop();
		is_front = false;
		front_time = 0;
		device_state = DS_Halted;
		return;
	}

	// --- Global sensor validation (runs FIRST, before timeout) ---
	// If both switches open, force to Empty regardless of state.
	// Catches edge cases where rapid filament removal leaves device in inconsistent state.
	if(!proximal && !distal &&
	   device_state != DS_Empty &&
	   device_state != DS_DeadmanForward &&
	   device_state != DS_DeadmanBack) {
		cmd_motor_stop(); // Always force stop (EN pin HIGH), even if motor_state thinks it's stopped
		is_front = false;
		front_time = 0;
		is_error = false;
		device_state = DS_Empty;
	}

	// --- Timeout check ---
	// Only apply timeout in states where is_front is actively set (PrimingForward, Loaded-Forward, Repriming)
	// Clear stale is_error in states that don't use forward timeout
	if(is_error) {
		if(device_state == DS_Retracting || device_state == DS_Unloading ||
		   device_state == DS_Primed || device_state == DS_Empty ||
		   device_state == DS_Halted || device_state == DS_StartupProbe) {
			// Stale error from a previous state — just clear it
			is_error = false;
			is_front = false;
			front_time = 0;
		} else {
			// Genuine timeout in an active forward state
			cmd_motor_stop();
			is_front = false;
			front_time = 0;
			is_error = false;
			device_state = DS_Halted;
			return;
		}
	}

	// --- State machine ---
	static uint32_t transition_start = 0; // Timestamp when a transition began
	static const uint32_t TRANSITION_TIMEOUT = 5000; // 5s timeout for priming/unloading
	static DeviceState prev_device_state = DS_Empty;
	static uint32_t state_entry_time = 0;
	if(device_state != prev_device_state) {
		state_entry_time = millis();
		prev_device_state = device_state;
	}

	switch(device_state) {

		case DS_Empty:
			if(motor_state != Stop) cmd_motor_stop();
			is_front = false;
			front_time = 0;
			digitalWrite(DUANLIAO, DUANLIAO_OUT_STATE);
			digitalWrite(START_LED, 0);

			// Auto-advance when filament inserted (proximal triggers)
			if(proximal && !distal) {
				cmd_motor_forward();
				front_time = 0;
				transition_start = millis();
				device_state = DS_PrimingForward;
			} else if(proximal && distal) {
				// Both already closed (edge case)
				device_state = DS_Primed;
			}
			break;

		case DS_PrimingForward:
			is_front = true; // Timeout safety
			digitalWrite(DUANLIAO, DUANLIAO_OUT_STATE);
			digitalWrite(START_LED, 0);

			// Any button during transition = halt
			if(fwd_press || back_press) {
				cmd_motor_stop();
				is_front = false;
				if(fwd_press) key2_consumed = true;
				if(back_press) key1_consumed = true;
				device_state = DS_Halted;
				break;
			}

			// 5s timeout: filament didn't reach gears
			if(millis() - transition_start >= TRANSITION_TIMEOUT) {
				cmd_motor_stop();
				is_front = false;
				device_state = DS_Halted;
				break;
			}

			// Target: distal triggered → Primed
			if(distal) {
				cmd_motor_stop();
				is_front = false;
				device_state = DS_Primed;
			}
			// Filament removed during priming
			if(!proximal && !distal) {
				cmd_motor_stop();
				is_front = false;
				device_state = DS_Empty;
			}
			break;

		case DS_Primed:
			if(motor_state != Stop) cmd_motor_stop();
			is_front = false;
			front_time = 0;
			digitalWrite(DUANLIAO, DUANLIAO_OUT_STATE);
			digitalWrite(START_LED, 0);

			// Sensor validation: react to switch changes
			// (skip distal check briefly after entering to avoid switch bounce)
			if(!proximal) {
				device_state = DS_Empty;
				break;
			}
			if(!distal && millis() - state_entry_time >= 50) {
				cmd_motor_forward();
				front_time = 0;
				transition_start = millis();
				device_state = DS_PrimingForward;
				break;
			}

			if(fwd_press) {
				device_state = DS_Loaded;
			} else if(back_press) {
				cmd_motor_back();
				transition_start = millis();
				device_state = DS_Unloading;
			}
			break;

		case DS_Loaded: {
			// === BUFFER ACTIVE ===
			// Motor is 100% driven by hall sensors (matches upstream logic).
			// Only exits: back button press, proximal opens (filament runout), or forward timeout.
			digitalWrite(DUANLIAO, !DUANLIAO_OUT_STATE);
			digitalWrite(START_LED, 1);

			// Back button → exit buffer active, start retract sequence
			if(back_press) {
				cmd_motor_stop();
				is_front = false;
				front_time = 0;
				transition_start = millis();
				device_state = DS_Retracting;
				break;
			}

			// Filament runout (proximal opened) → emergency stop
			if(!proximal) {
				cmd_motor_stop();
				is_front = false;
				front_time = 0;
				device_state = DS_Empty;
				break;
			}

			// Hall sensor buffer logic (upstream behavior):
			// - Sensor active → change motor state + send command only on change
			// - No sensor active → motor continues in current direction (no default-to-stop)
			static uint32_t coast_since = 0;
			if(buffer.buffer1_pos1_sensor_state) {
				is_front = true;
				coast_since = 0;
				if(motor_state != Forward) cmd_motor_forward();
			} else if(buffer.buffer1_pos2_sensor_state) {
				is_front = false;
				front_time = 0;
				if(motor_state != Stop) {
					cmd_motor_coast();
					coast_since = millis();
				}
			} else if(buffer.buffer1_pos3_sensor_state) {
				is_front = false;
				front_time = 0;
				coast_since = 0;
				if(motor_state != Back) cmd_motor_back();
			}

			// Delayed EN disable: keep coils energized for smooth restarts,
			// then fully de-energize after settling (works in dead zones too)
			if(motor_state == Stop && coast_since && millis() - coast_since >= coast_delay) {
				WRITE_EN_PIN(1);
				coast_since = 0;
			}
			break;
		}

		case DS_Retracting:
			if(motor_state != Back) cmd_motor_back();
			is_front = false;
			front_time = 0;
			digitalWrite(DUANLIAO, DUANLIAO_OUT_STATE);
			digitalWrite(START_LED, 0);

			// Any button during transition = halt
			if(fwd_press || back_press) {
				cmd_motor_stop();
				if(fwd_press) key2_consumed = true;
				if(back_press) key1_consumed = true;
				device_state = DS_Halted;
				break;
			}

			// No timeout on Retracting — retraction from hotend can take a long time.
			// User can halt with a button press. Global check catches filament removal.

			// Distal opened → filament pulled back past gears
			// Stop, then reverse to re-prime (push filament back until distal triggers)
			if(!distal) {
				cmd_motor_stop();
				delay(50); // Brief pause before reversing
				cmd_motor_forward();
				front_time = 0;
				transition_start = millis();
				device_state = DS_Repriming;
			}
			break;

		case DS_Repriming:
			is_front = true; // Timeout safety
			digitalWrite(DUANLIAO, DUANLIAO_OUT_STATE);
			digitalWrite(START_LED, 0);

			// Sensor validation: filament yanked out (either or both switches open)
			if(!proximal) {
				cmd_motor_stop();
				is_front = false;
				device_state = DS_Empty;
				break;
			}

			// Any button during transition = halt
			if(fwd_press || back_press) {
				cmd_motor_stop();
				is_front = false;
				if(fwd_press) key2_consumed = true;
				if(back_press) key1_consumed = true;
				device_state = DS_Halted;
				break;
			}

			// 5s timeout: distal didn't trigger (filament may have broken or missed gears)
			if(millis() - transition_start >= TRANSITION_TIMEOUT) {
				cmd_motor_stop();
				is_front = false;
				device_state = DS_Halted;
				break;
			}

			// Target: distal triggered → Primed
			if(distal) {
				cmd_motor_stop();
				is_front = false;
				device_state = DS_Primed;
			}
			break;

		case DS_Unloading:
			is_front = false;
			front_time = 0;
			digitalWrite(DUANLIAO, DUANLIAO_OUT_STATE);
			digitalWrite(START_LED, 0);

			// Any button during transition = halt
			if(fwd_press || back_press) {
				cmd_motor_stop();
				if(fwd_press) key2_consumed = true;
				if(back_press) key1_consumed = true;
				device_state = DS_Halted;
				break;
			}

			// 5s timeout: user didn't pull filament out
			if(millis() - transition_start >= TRANSITION_TIMEOUT) {
				cmd_motor_stop();
				device_state = DS_Halted;
				break;
			}

			// Proximal opened → filament removed → Empty
			if(!proximal) {
				cmd_motor_stop();
				device_state = DS_Empty;
			}
			break;

		case DS_StartupProbe: {
			// Retract on boot to determine if filament is loaded to hotend.
			// NEVER advances on startup (safety: distal switch could be disconnected).
			static uint32_t probe_start = 0;
			static const uint32_t PROBE_TIMEOUT = 10000; // 10s timeout for startup probe
			digitalWrite(DUANLIAO, DUANLIAO_OUT_STATE);
			digitalWrite(START_LED, 0);
			is_front = false;

			if(motor_state != Back) {
				cmd_motor_back();
				probe_start = millis();
			}

			// 10s timeout: couldn't determine state, halt and let user decide
			if(millis() - probe_start >= PROBE_TIMEOUT) {
				cmd_motor_stop();
				device_state = DS_Halted;
				break;
			}

			// Any button during probe = halt
			if(fwd_press || back_press) {
				cmd_motor_stop();
				if(fwd_press) key2_consumed = true;
				if(back_press) key1_consumed = true;
				device_state = DS_Halted;
				break;
			}

			// Buffer reached fully retracted position (pos1/HALL3)
			// → filament is trapped by hotend → Primed (user presses forward to enter Loaded)
			if(buffer.buffer1_pos1_sensor_state) {
				cmd_motor_stop();
				device_state = DS_Primed;
				break;
			}

			// Distal opened during retraction → filament not at hotend
			// Stop and re-prime
			if(!distal) {
				cmd_motor_stop();
				delay(50);
				cmd_motor_forward();
				front_time = 0;
				transition_start = millis();
				device_state = DS_Repriming;
				break;
			}

			// Proximal opened → filament fully removed
			if(!proximal) {
				cmd_motor_stop();
				device_state = DS_Empty;
				break;
			}
			break;
		}

		case DS_Halted:
			if(motor_state != Stop) cmd_motor_stop();
			is_front = false;
			front_time = 0;
			digitalWrite(DUANLIAO, DUANLIAO_OUT_STATE);
			digitalWrite(START_LED, 0);

			// Sensor validation: if both switches open, transition to Empty
			// (enables auto-advance when user reinserts filament)
			if(!proximal && !distal) {
				device_state = DS_Empty;
				break;
			}

			if(fwd_press) {
				if(proximal && !distal) {
					cmd_motor_forward();
					front_time = 0;
					transition_start = millis();
					device_state = DS_PrimingForward;
				} else if(proximal && distal) {
					device_state = DS_Loaded;
				} else {
					device_state = DS_Empty;
				}
			} else if(back_press) {
				if(proximal && distal) {
					cmd_motor_back();
					device_state = DS_Retracting;
				} else if(proximal && !distal) {
					cmd_motor_back();
					transition_start = millis();
					device_state = DS_Unloading;
				} else {
					device_state = DS_Empty;
				}
			}
			break;

		default:
			device_state = DS_Empty;
			break;
	}

	// Persist state to EEPROM when entering Primed or Loaded (only on change)
	if(device_state == DS_Primed && buffer_para.saved_device_state != SAVED_STATE_PRIMED) {
		buffer_para.saved_device_state = SAVED_STATE_PRIMED;
		EEPROM.put(0, buffer_para);
	} else if(device_state == DS_Loaded && buffer_para.saved_device_state != SAVED_STATE_LOADED) {
		buffer_para.saved_device_state = SAVED_STATE_LOADED;
		EEPROM.put(0, buffer_para);
	}
}

void timer_it_callback(){

	//feed watchdog (every 100ms)
	static uint32_t i=0;
	i++;
	//check g_run_cnt every 5 seconds
	if(i>=50)
	{
		//main loop has stalled
		if(g_run_cnt == 0)
		{
			Serial.println("program excepiton,iwdg trigger reset cpu\r\n");
			//wait for watchdog timeout to trigger reset
			while(1){
				delay(1);
			}
		}
		
		g_run_cnt=0;
		
		i=0;
	}
	// HAL_IWDG_Refresh(&hiwdg);
	IWDG->KR = 0xAAAA;   // equivalent to HAL_IWDG_Refresh(&hiwdg);





	//forward feed timeout detection
	if (is_front)
	{
		front_time+=100;
		if (front_time > timeout)
		{
			is_error = true;
		}
	}
}

void key1_it_callback(void){
	static uint32_t last_edge = 0;
	uint32_t now = millis();
	if(now - last_edge < 50) return; // Debounce 50ms
	last_edge = now;

	if(!digitalRead(KEY1)){  // Falling edge (press)
		key1_press_times=now;
		key1_press_flag=true;
	}
	else{  // Rising edge (release)
		key1_press_flag=false;
	}
}

void key2_it_callback(void){
	static uint32_t last_edge = 0;
	uint32_t now = millis();
	if(now - last_edge < 50) return; // Debounce 50ms
	last_edge = now;

	if(!digitalRead(KEY2)){  // Falling edge (press)
		key2_press_times=now;
		key2_press_flag=true;
	}
	else{  // Rising edge (release)
		key2_press_flag=false;
	}
}

void Recv_MDM_Pulse_IT_Callback(void){
	blockage_detect.mdm_pulse_cnt++;
}

void Dir_IT_Callback(void){
	//update timer count direction
	if(SIGNAL_COUNT_READ_DIR_IO())	SIGNAL_COUNT_UP();		//DIR high = count up
	else 	SIGNAL_COUNT_DOWN();	//DIR low = count down
}



/**
  * @brief  USB serial command parser
  * @param  null
  * @retval null
**/
// Helper: find argument after first space in command buffer, or NULL if none
static const char* cmd_arg(const char* buf) {
	const char* sp = strchr(buf, ' ');
	return sp ? sp + 1 : NULL;
}

void USB_Serial_Analys(void){

	static char serial_buf[64];
	static uint8_t serial_buf_len = 0;

	if(Serial.available()){
		char c=Serial.read();
		if(c=='\n'){
			serial_buf[serial_buf_len] = '\0';
			const char* arg = cmd_arg(serial_buf);

			if(strstr(serial_buf,"rt")){
				Serial.print("read timeout=");
				Serial.println(timeout);
			}
			else if(strstr(serial_buf,"timeout")){
				if(!arg){
					Serial.println("Error: Invalid timeout value.");
				} else {
					int64_t num = atol(arg);
					if(num<0||num>(int64_t)0xffffffff){
						Serial.println("Error: Invalid timeout value.");
					} else {
						buffer_para.timeout=num;
						timeout=num;
						EEPROM.put(0, buffer_para);
						Serial.print("set timeout succeed! timeout=");
						Serial.println(timeout);
					}
				}
			}
			else if(strstr(serial_buf,"steps")){
				if(!arg){
					Serial.println("Error: Invalid steps value.");
				} else {
					int64_t num = atol(arg);
					if(num<0||num>51200){
						Serial.println("Error: Invalid steps value.");
					} else {
						buffer_para.steps=num;
						steps=num;
						EEPROM.put(0, buffer_para);
						Serial.print("set steps succeed! steps=");
						Serial.println(steps);
						if(connet_mdm_flag){
							REIN_TIM_SIGNAL_COUNT_DeInit();
							REIN_TIM_SIGNAL_COUNT_Init();
						}
					}
				}
			}
			else if(strstr(serial_buf,"clear")){
				blockage_detect.actual_distance=0;
				blockage_detect.target_distance=0;
				blockage_detect.mdm_pulse_cnt=0;
				blockage_detect.pulse_cnt=0;
			}
			else if(strstr(serial_buf,"encoder")){
				if(!arg){
					Serial.println("Error: Invalid encoder value.");
				} else {
					float num = fastAtof(arg);
					if(num<0){
						Serial.println("Error: Invalid encoder length value.");
					} else {
						buffer_para.encoder_length=num;
						encoder_length=num;
						EEPROM.put(0, buffer_para);
						blockage_detect.allow_error = encoder_length*allow_error_scale;
						Serial.print("set encoder length succeed! encoder_length=");
						Serial.println(encoder_length);
					}
				}
			}
			else if(strstr(serial_buf,"info")){
				Serial.print("encoder_length="); Serial.println(encoder_length);
				Serial.print("timeout="); Serial.println(timeout);
				Serial.print("steps="); Serial.println(steps);
				Serial.print("speed="); Serial.println(SPEED);
				Serial.print("allow_error_scale="); Serial.println(allow_error_scale);
				Serial.print("allow_error="); Serial.println(blockage_detect.allow_error);
				Serial.print("DUANLIAO_OUT_STATE="); Serial.println(buffer_para.DUANLIAO_OUT_STATE);
				Serial.print("coast_delay="); Serial.print(coast_delay); Serial.println("ms");
				Serial.print("device_state="); Serial.println(state_name(device_state));
				Serial.print("proximal_switch(PB7)=");
				Serial.println(digitalRead(ENDSTOP_3) ? "OPEN (no filament)" : "CLOSED (filament present)");
				Serial.print("distal_switch(PB14)=");
				Serial.println(digitalRead(DISTAL_SWITCH) ? "OPEN (no filament)" : "CLOSED (filament present)");
			}
			else if(strstr(serial_buf,"scale")){
				if(!arg){
					Serial.println("Error: Invalid scale value.");
				} else {
					float num = fastAtof(arg);
					if(num<0){
						Serial.println("Error: Invalid scale length value.");
					} else {
						buffer_para.allow_error_scale=num;
						allow_error_scale=num;
						EEPROM.put(0, buffer_para);
						blockage_detect.allow_error = encoder_length*allow_error_scale;
						Serial.print("set scale length succeed! allow_error_scale=");
						Serial.println(allow_error_scale);
					}
				}
			}
			else if(strstr(serial_buf,"speed")){
				if(!arg){
					Serial.print("speed: "); Serial.println(SPEED);
				} else {
					float num = fastAtof(arg);
					if(num<0){
						Serial.println("Error: Invalid speed value.");
					} else {
						buffer_para.SPEED=num;
						SPEED=num;
						VACTRUAL_VALUE=(uint32_t)(SPEED*Move_Divide_NUM*200/60/0.715f);
						EEPROM.put(0, buffer_para);
						Serial.print("set speed succeed! speed=");
						Serial.println(SPEED);
					}
				}
			}
			else if(strstr(serial_buf,"I")){
				if(!arg){
					Serial.print("I_CURRENT="); Serial.println(I_CURRENT);
				} else {
					int32_t num = atoi(arg);
					if(num<0||num>3000){
						Serial.println("Error: Invalid I_CURRENT value, range: 0-3000mA");
					} else {
						buffer_para.I_CURRENT=num;
						I_CURRENT=num;
						EEPROM.put(0, buffer_para);
						Serial.print("set I_CURRENT succeed! I_CURRENT=");
						Serial.println(I_CURRENT);
					}
				}
			}
			else if(strstr(serial_buf,"out")){
				if(!arg){
					Serial.print("DUANLIAO_OUT_STATE="); Serial.println(buffer_para.DUANLIAO_OUT_STATE);
				} else {
					bool state = atoi(arg);
					buffer_para.DUANLIAO_OUT_STATE=state;
					DUANLIAO_OUT_STATE=state;
					EEPROM.put(0, buffer_para);
					Serial.print("set DUANLIAO_OUT_STATE succeed! DUANLIAO_OUT_STATE=");
					Serial.println(DUANLIAO_OUT_STATE);
				}
			}
			else if(strstr(serial_buf,"coast")){
				if(!arg){
					Serial.print("coast_delay="); Serial.print(coast_delay); Serial.println("ms");
				} else {
					int32_t num = atoi(arg);
					if(num<0||num>10000){
						Serial.println("Error: Invalid coast_delay value, range: 0-10000ms");
					} else {
						buffer_para.coast_delay=num;
						coast_delay=num;
						EEPROM.put(0, buffer_para);
						Serial.print("set coast_delay succeed! coast_delay=");
						Serial.print(coast_delay);
						Serial.println("ms");
					}
				}
			}
			else if(strstr(serial_buf,"version")){
				Serial.print("version: "); Serial.println(VERSION);
			}
			else{
				Serial.println(serial_buf);
				Serial.println("command error!");
				Serial.print(
					"\n+-----------------------------------------------+\n"
					"|         Fly Buffer Command Set                |\n"
					"|  timeout <ms>    - set feed timeout            |\n"
					"|  rt              - read timeout                |\n"
					"|  steps <n>       - set steps/mm                |\n"
					"|  encoder <n>     - set encoder length          |\n"
					"|  scale <n>       - set error scale             |\n"
					"|  speed <rpm>     - set motor speed             |\n"
					"|  I <mA>          - set motor current           |\n"
					"|  out <0|1>       - set endstop polarity        |\n"
					"|  coast <ms>      - set coast delay             |\n"
					"|  info            - show all parameters         |\n"
					"|  clear           - reset blockage counters     |\n"
					"|  version         - show firmware version       |\n"
					"+-----------------------------------------------+\n\n"
				);
			}
			serial_buf_len = 0;

		}
		else if(serial_buf_len < sizeof(serial_buf) - 1) {
			serial_buf[serial_buf_len++] = c;
		}
	}
}



/**
  * @brief  Check if MDM blockage detection module is connected
  * @param  null
  * @retval true=connected, false=not connected
**/
bool Check_Connet_MDM(void){

	//wait for power-on stabilization
	delay(1000);

	//read MDM filament pin state
	pinMode(MDM_DPIN,INPUT);
	bool mdm_state=digitalRead(MDM_DPIN);

	//configure opposite pull direction and re-read; unchanged level = connected
	if(mdm_state){//HIGH, configure pull-down
		pinMode(MDM_DPIN,INPUT_PULLDOWN);
		Serial.println(digitalRead(MDM_DPIN));
		if(digitalRead(MDM_DPIN))	return true;//level unchanged = connected
		else 						return false;
	}
	else{
		pinMode(MDM_DPIN,INPUT_PULLUP);
		Serial.println(digitalRead(MDM_DPIN));
		if(digitalRead(MDM_DPIN))	return false;
		else 						return true;//level unchanged = connected
	}
}

/**
 * @brief  TIM_SIGNAL_PUL initialization (pulse counter)
 * @param  NULL
 * @retval NULL
 **/
void REIN_TIM_SIGNAL_COUNT_Init(void)
{
	/* GPIO initialization */
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	/* GPIO Ports Clock Enable*/
	SIGNAL_COUNT_PUL_CLK_ENABLE(); // enable SIGNAL_COUNT_PUL port clock
	/*Configure GPIO pin*/
	GPIO_InitStruct.Pin = SIGNAL_COUNT_PUL_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; // alternate function push-pull
	GPIO_InitStruct.Pull = GPIO_NOPULL;		// no pull-up/pull-down
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM2;
	HAL_GPIO_Init(SIGNAL_COUNT_PUL_GPIO_Port, &GPIO_InitStruct);

	/* TIM initialization */
	TIM_SlaveConfigTypeDef sSlaveConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	SIGNAL_COUNT_TIM_CLK_ENABLE(); // enable TIM clock
	SIGNAL_COUNT_Get_HTIM.Instance = SIGNAL_COUNT_Get_TIM;
	SIGNAL_COUNT_Get_HTIM.Init.Prescaler = 0;									   // no prescaler
	SIGNAL_COUNT_Get_HTIM.Init.CounterMode = TIM_COUNTERMODE_UP;				   // count up
	SIGNAL_COUNT_Get_HTIM.Init.Period = 65536-1;									   // counter period
	SIGNAL_COUNT_Get_HTIM.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;			   // no clock division
	SIGNAL_COUNT_Get_HTIM.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE; // disable auto-reload preload
	if (HAL_TIM_Base_Init(&SIGNAL_COUNT_Get_HTIM) != HAL_OK)
	{
		Error_Handler();
	}
	sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;		   // external clock mode
	sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;				   // TI1FP1
	sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_RISING; // rising edge trigger
	sSlaveConfig.TriggerFilter = 4;							   // filter setting (FDIV2_N6)
	if (HAL_TIM_SlaveConfigSynchro(&SIGNAL_COUNT_Get_HTIM, &sSlaveConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;			 // master mode trigger reset
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE; // disable master/slave mode
	if (HAL_TIMEx_MasterConfigSynchronization(&SIGNAL_COUNT_Get_HTIM, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/*begin work*/
	// HAL_TIM_Base_Start_IT(&SIGNAL_COUNT_Get_HTIM);
	HAL_TIM_Base_Start(&SIGNAL_COUNT_Get_HTIM);

}

/**
  * @brief  TIM_SIGNAL_COUNT cleanup
  * @param  NULL
  * @retval NULL
**/
void REIN_TIM_SIGNAL_COUNT_DeInit(void)
{
	// HAL_TIM_Base_Stop_IT(&SIGNAL_COUNT_Get_HTIM);										//stop TIM
	HAL_TIM_Base_Stop(&SIGNAL_COUNT_Get_HTIM);										//stop TIM
	HAL_GPIO_DeInit(SIGNAL_COUNT_PUL_GPIO_Port, SIGNAL_COUNT_PUL_Pin);	//reset GPIO
}

/**
  * @brief  GPIO initialization (SIGNAL_COUNT direction pin)
  * @param  NULL
  * @retval NULL
*/
void Signal_Dir_Init(void)
{
	pinMode(SIG_DIR_PIN,INPUT);
	attachInterrupt(SIG_DIR_PIN,&Dir_IT_Callback,CHANGE);
}


/**
  * @brief  Pulse reception initialization
  * @param  null
  * @retval null
**/
void Pulse_Receive_Init(void){

	// PULSE2_PIN initialization
	if(connet_mdm_flag){
		pinMode(PULSE2_PIN,INPUT);
		attachInterrupt(PULSE2_PIN,&Recv_MDM_Pulse_IT_Callback,RISING);
		REIN_TIM_SIGNAL_COUNT_Init();
	}

	blockage_detect.allow_error = encoder_length*allow_error_scale;
}


void Blockage_Detect(void){

	static uint32_t last_target_distance=blockage_detect.target_distance;
	static uint32_t last_timer_cnt=TIM2->CNT;
	static uint32_t last_time=0;

	//reset counters if no pulses received for 500ms
	if(last_timer_cnt==TIM2->CNT){//CNT unchanged, no pulses
		if(millis()-last_time>=500){
			blockage_detect.actual_distance=0;
			blockage_detect.target_distance=0;
			blockage_detect.mdm_pulse_cnt=0;
			blockage_detect.extrusion_pulse_cnt=0;
		}
	}
	else{//CNT changed, pulses detected, record timestamp
		last_timer_cnt=TIM2->CNT;
		last_time=millis();

		//calculate extrusion pulse count
		blockage_detect.last_pulse_cnt=blockage_detect.pulse_cnt;
		blockage_detect.pulse_cnt=TIM2->CNT;
		blockage_detect.pulse_cnt_sub=blockage_detect.pulse_cnt-blockage_detect.last_pulse_cnt;
		blockage_detect.extrusion_pulse_cnt+=blockage_detect.pulse_cnt_sub;
		

		//only calculate extrusion distance; negative = retraction, skip
		if(blockage_detect.extrusion_pulse_cnt<0) blockage_detect.target_distance=0;
		else blockage_detect.target_distance=(blockage_detect.extrusion_pulse_cnt)/steps;

	}

	blockage_detect.actual_distance=blockage_detect.mdm_pulse_cnt*encoder_length;//actual distance
	blockage_detect.distance_error=blockage_detect.actual_distance-blockage_detect.target_distance;//distance difference




	static bool detect_blockage=false;
	static uint32_t detect_blockage_time=0;
	
	
	//require two consecutive blockage detections to confirm; single = false trigger
	if(!detect_blockage){//no blockage detected yet
		if(blockage_detect.target_distance!=last_target_distance){

			//blockage check
			if(abs(blockage_detect.distance_error)>blockage_detect.allow_error&&blockage_detect.target_distance>=blockage_detect.allow_error){//blockage detected
	
				detect_blockage=true;
				detect_blockage_time=millis();


				//reset counters
				blockage_detect.actual_distance=0;
				blockage_detect.target_distance=0;
				blockage_detect.mdm_pulse_cnt=0;
				blockage_detect.extrusion_pulse_cnt=0;
			}

			last_target_distance=blockage_detect.target_distance;
		}

	}
	else if(blockage_detect.target_distance>blockage_detect.allow_error){//first detection occurred; re-check after delay to confirm or dismiss as false trigger
		if(millis()-detect_blockage_time>=100){
			if(abs(blockage_detect.distance_error)>blockage_detect.allow_error){//blockage confirmed
				//trigger blockage alarm
;
				blockage_detect.blockage_flag=true;
				blockage_inform_times=millis();
				digitalWrite(DULIAO,LOW);			
				detect_blockage=false;
				detect_blockage_time=0;	

				//reset counters
				blockage_detect.actual_distance=0;
				blockage_detect.target_distance=0;
				blockage_detect.mdm_pulse_cnt=0;
				blockage_detect.extrusion_pulse_cnt=0;				
			}
			else{//no blockage on re-check, dismiss as false trigger
				//clear detection flag
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

    // integer part
    while (*s >= '0' && *s <= '9') {
        val = val * 10.0f + (*s - '0');
        s++;
    }

    // decimal part
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