/**
 * TMCStepper library by @teemuatlut
 * CHOPCONF.cpp - CHOPCONF Chopper Configuration
 * TMC2208 (TMC2209)
 */
#include "../TMCStepper.h"
#include "TMC_MACROS.h"

#define GET_REG(SETTING) CHOPCONF_t r{}; r.sr = CHOPCONF(); return r.SETTING
#define SET_REG(SETTING) CHOPCONF_register.SETTING = B; write(CHOPCONF_register.address, CHOPCONF_register.sr)

//
// TMC2208 (TMC2209)
//
#define GET_REG_2208(SETTING) TMC2208_n::CHOPCONF_t r{}; r.sr = CHOPCONF(); return r.SETTING

uint32_t TMC2208Stepper::CHOPCONF() { return read(CHOPCONF_register.address); }
void TMC2208Stepper::CHOPCONF(uint32_t input) {
	CHOPCONF_register.sr = input;
	write(CHOPCONF_register.address, CHOPCONF_register.sr);
}
void TMC2208Stepper::toff	( uint8_t  B )	{ SET_REG(toff);	}
void TMC2208Stepper::hstrt	( uint8_t  B )	{ SET_REG(hstrt);	}
void TMC2208Stepper::hend	( uint8_t  B )	{ SET_REG(hend);	}
void TMC2208Stepper::tbl	( uint8_t  B )	{ SET_REG(tbl);	}
void TMC2208Stepper::vsense	( bool     B )	{ SET_REG(vsense);	}
void TMC2208Stepper::mres	( uint8_t  B )	{ SET_REG(mres);	}
void TMC2208Stepper::intpol	( bool     B )	{ SET_REG(intpol);	}
void TMC2208Stepper::dedge	( bool     B )	{ SET_REG(dedge);	}
void TMC2208Stepper::diss2g	( bool     B )	{ SET_REG(diss2g);	}
void TMC2208Stepper::diss2vs( bool     B )	{ SET_REG(diss2vs); }

uint8_t TMC2208Stepper::toff()		{ GET_REG_2208(toff);		}
uint8_t TMC2208Stepper::hstrt()		{ GET_REG_2208(hstrt);		}
uint8_t TMC2208Stepper::hend()		{ GET_REG_2208(hend);		}
uint8_t TMC2208Stepper::tbl()		{ GET_REG_2208(tbl);		}
bool	TMC2208Stepper::vsense()	{ GET_REG_2208(vsense);		}
uint8_t TMC2208Stepper::mres()		{ GET_REG_2208(mres);		}
bool	TMC2208Stepper::intpol()	{ GET_REG_2208(intpol);		}
bool	TMC2208Stepper::dedge()		{ GET_REG_2208(dedge);		}
bool	TMC2208Stepper::diss2g()	{ GET_REG_2208(diss2g);		}
bool	TMC2208Stepper::diss2vs()	{ GET_REG_2208(diss2vs);	}
