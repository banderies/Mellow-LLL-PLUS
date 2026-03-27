/**
 * TMCStepper library by @teemuatlut
 * PWMCONF.cpp - PWM Configuration
 * TMC2208 (TMC2209)
 */
#include "../TMCStepper.h"
#include "TMC_MACROS.h"

#define SET_REG(SETTING) PWMCONF_register.SETTING = B; write(PWMCONF_register.address, PWMCONF_register.sr)
#define GET_REG(SETTING) return PWMCONF_register.SETTING

//
// TMC2208 (TMC2209)
//
#define GET_REG_2208(SETTING) TMC2208_n::PWMCONF_t r{}; r.sr = PWMCONF(); return r.SETTING

uint32_t TMC2208Stepper::PWMCONF() {
	return read(PWMCONF_register.address);
}
void TMC2208Stepper::PWMCONF(uint32_t input) {
	PWMCONF_register.sr = input;
	write(PWMCONF_register.address, PWMCONF_register.sr);
}

void TMC2208Stepper::pwm_ofs		( uint8_t B ) { SET_REG(pwm_ofs);		}
void TMC2208Stepper::pwm_grad		( uint8_t B ) { SET_REG(pwm_grad);		}
void TMC2208Stepper::pwm_freq		( uint8_t B ) { SET_REG(pwm_freq);		}
void TMC2208Stepper::pwm_autoscale	( bool 	  B ) { SET_REG(pwm_autoscale);	}
void TMC2208Stepper::pwm_autograd	( bool    B ) { SET_REG(pwm_autograd);	}
void TMC2208Stepper::freewheel		( uint8_t B ) { SET_REG(freewheel);		}
void TMC2208Stepper::pwm_reg		( uint8_t B ) { SET_REG(pwm_reg);		}
void TMC2208Stepper::pwm_lim		( uint8_t B ) { SET_REG(pwm_lim);		}

uint8_t TMC2208Stepper::pwm_ofs()		{ GET_REG_2208(pwm_ofs);		}
uint8_t TMC2208Stepper::pwm_grad()		{ GET_REG_2208(pwm_grad);		}
uint8_t TMC2208Stepper::pwm_freq()		{ GET_REG_2208(pwm_freq);		}
bool 	TMC2208Stepper::pwm_autoscale()	{ GET_REG_2208(pwm_autoscale);	}
bool 	TMC2208Stepper::pwm_autograd()	{ GET_REG_2208(pwm_autograd);	}
uint8_t TMC2208Stepper::freewheel()		{ GET_REG_2208(freewheel);		}
uint8_t TMC2208Stepper::pwm_reg()		{ GET_REG_2208(pwm_reg);		}
uint8_t TMC2208Stepper::pwm_lim()		{ GET_REG_2208(pwm_lim);		}
