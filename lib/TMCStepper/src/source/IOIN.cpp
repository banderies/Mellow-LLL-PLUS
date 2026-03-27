/**
 * TMCStepper library by @teemuatlut
 * IOIN.cpp - Driver Control
 * TMC2208, TMC2209
 */
#include "../TMCStepper.h"
#include "TMC_MACROS.h"

#define GET_REG_NS(NS, SETTING) NS::IOIN_t r{}; r.sr = IOIN(); return r.SETTING

//
// TMC2208
//
#define GET_REG_2208(SETTING) GET_REG_NS(TMC2208_n, SETTING)

uint32_t TMC2208Stepper::IOIN()		{ return read(TMC2208_n::IOIN_t::address); }
bool	TMC2208Stepper::enn()		{ GET_REG_2208(enn);		}
bool	TMC2208Stepper::ms1()		{ GET_REG_2208(ms1);		}
bool	TMC2208Stepper::ms2()		{ GET_REG_2208(ms2);		}
bool	TMC2208Stepper::diag()		{ GET_REG_2208(diag);		}
bool	TMC2208Stepper::pdn_uart()	{ GET_REG_2208(pdn_uart);	}
bool	TMC2208Stepper::step()		{ GET_REG_2208(step);		}
bool	TMC2208Stepper::sel_a()		{ GET_REG_2208(sel_a);		}
bool	TMC2208Stepper::dir()		{ GET_REG_2208(dir);		}
uint8_t	TMC2208Stepper::version() 	{ GET_REG_2208(version);	}

//
// TMC2209
//
#define GET_REG_2209(SETTING) GET_REG_NS(TMC2209_n, SETTING)

uint32_t TMC2209Stepper::IOIN()		{ return read(TMC2209_n::IOIN_t::address); }
bool	TMC2209Stepper::enn()		{ GET_REG_2209(enn);		}
bool	TMC2209Stepper::ms1()		{ GET_REG_2209(ms1);		}
bool	TMC2209Stepper::ms2()		{ GET_REG_2209(ms2);		}
bool	TMC2209Stepper::diag()		{ GET_REG_2209(diag);		}
bool	TMC2209Stepper::pdn_uart()	{ GET_REG_2209(pdn_uart);	}
bool	TMC2209Stepper::step()		{ GET_REG_2209(step);		}
bool	TMC2209Stepper::spread_en()	{ GET_REG_2209(spread_en);	}
bool	TMC2209Stepper::dir()		{ GET_REG_2209(dir);		}
uint8_t	TMC2209Stepper::version() 	{ GET_REG_2209(version);	}
