/**
 * TMCStepper library by @teemuatlut
 * SLAVECONF.cpp
 * TMC2208 (TMC2209)
 */
#include "../TMCStepper.h"
#include "TMC_MACROS.h"

#define SET_REG(SETTING) SLAVECONF_register.SETTING = B; write(SLAVECONF_register.address, SLAVECONF_register.sr)
#define GET_REG(SETTING) return SLAVECONF_register.SETTING

//
// TMC2208 (TMC2209)
//

uint16_t TMC2208Stepper::SLAVECONF() { return SLAVECONF_register.sr; }
void TMC2208Stepper::SLAVECONF(uint16_t input) {
	SLAVECONF_register.sr = input & 0xF00;
	write(SLAVECONF_register.address, SLAVECONF_register.sr);
}
uint8_t TMC2208Stepper::senddelay()			{ GET_REG(senddelay); }
void TMC2208Stepper::senddelay(uint8_t B)	{ SET_REG(senddelay); }
