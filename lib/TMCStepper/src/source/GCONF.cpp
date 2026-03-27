/**
 * TMCStepper library by @teemuatlut
 * ENCMODE.cpp - Driver Status
 * TMC2208 (TMC2209)
 */
#include "../TMCStepper.h"
#include "TMC_MACROS.h"

#define GET_REG(SETTING) GCONF_t r{}; r.sr = GCONF(); return r.SETTING
#define SET_REG(SETTING) GCONF_register.SETTING = B; write(GCONF_register.address, GCONF_register.sr)

//
// TMC2208 (TMC2209)
//
#define GET_REG_2208(SETTING) TMC2208_n::GCONF_t r{}; r.sr = GCONF(); return r.SETTING

uint32_t TMC2208Stepper::GCONF() {
	return read(GCONF_register.address);
}
void TMC2208Stepper::GCONF(uint32_t input) {
	GCONF_register.sr = input;
	write(GCONF_register.address, GCONF_register.sr);
}

void TMC2208Stepper::I_scale_analog(bool B)		{ SET_REG(i_scale_analog);	}
void TMC2208Stepper::internal_Rsense(bool B)	{ SET_REG(internal_rsense);	}
void TMC2208Stepper::en_spreadCycle(bool B)		{ SET_REG(en_spreadcycle);	}
void TMC2208Stepper::shaft(bool B) 				{ SET_REG(shaft);			}
void TMC2208Stepper::index_otpw(bool B)			{ SET_REG(index_otpw);		}
void TMC2208Stepper::index_step(bool B)			{ SET_REG(index_step);		}
void TMC2208Stepper::pdn_disable(bool B)		{ SET_REG(pdn_disable);		}
void TMC2208Stepper::mstep_reg_select(bool B)	{ SET_REG(mstep_reg_select);}
void TMC2208Stepper::multistep_filt(bool B)		{ SET_REG(multistep_filt);	}

bool TMC2208Stepper::I_scale_analog()	{ GET_REG_2208(i_scale_analog);		}
bool TMC2208Stepper::internal_Rsense()	{ GET_REG_2208(internal_rsense);	}
bool TMC2208Stepper::en_spreadCycle()	{ GET_REG_2208(en_spreadcycle);		}
bool TMC2208Stepper::shaft()			{ GET_REG_2208(shaft);				}
bool TMC2208Stepper::index_otpw()		{ GET_REG_2208(index_otpw);			}
bool TMC2208Stepper::index_step()		{ GET_REG_2208(index_step);			}
bool TMC2208Stepper::pdn_disable()		{ GET_REG_2208(pdn_disable);		}
bool TMC2208Stepper::mstep_reg_select()	{ GET_REG_2208(mstep_reg_select);	}
bool TMC2208Stepper::multistep_filt()	{ GET_REG_2208(multistep_filt);		}
