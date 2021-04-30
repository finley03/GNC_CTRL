#include "time.h"


void set_clock_48m() {
	// set wait states for flash to 1
	// see datasheet table 37-42

	NVMCTRL->CTRLB.bit.RWS = 1;

	


	// enable external 32khz oscillator

	SYSCTRL->XOSC32K.reg = SYSCTRL_XOSC32K_STARTUP(0x4) | SYSCTRL_XOSC32K_EN32K | SYSCTRL_XOSC32K_XTALEN;
	// seperate write
	SYSCTRL->XOSC32K.bit.ENABLE = 1;

	// wait until done
	while(!SYSCTRL->PCLKSR.bit.XOSC32KRDY);





	// configure GCLK1

	// set divide to 1 (no division)
	GCLK->GENDIV.reg = GCLK_GENDIV_ID(0) | GCLK_GENDIV_DIV(0);

	// set GCLK1 to use external 32k oscillator
	GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(1) | GCLK_GENCTRL_SRC_XOSC32K | GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN;

	// wait for data write to complete
	while(GCLK->STATUS.bit.SYNCBUSY);



	// send GCLK1 to DFLL
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_DFLL48 | GCLK_CLKCTRL_GEN_GCLK1 | GCLK_CLKCTRL_CLKEN;



	// set up DFLL

	// mush reset value before configuration
	while(!SYSCTRL->PCLKSR.bit.DFLLRDY);
	SYSCTRL->DFLLCTRL.reg = SYSCTRL_DFLLCTRL_ENABLE;
	while(!SYSCTRL->PCLKSR.bit.DFLLRDY);


	// set clock multiplier and trims
	// SYSCTRL_DFLLMUL_MUL = factor of multiplication
	// SYSCTRL_DFLLMUL_FSTEP = fine step
	// SYSCTRL_SFLLMUL_CSTEP = coarse step
	SYSCTRL->DFLLMUL.reg = SYSCTRL_DFLLMUL_MUL(1465) | SYSCTRL_DFLLMUL_FSTEP(511) | SYSCTRL_DFLLMUL_CSTEP(31);

	// wait for write to finish
	while(!SYSCTRL->PCLKSR.bit.DFLLRDY);



	// set default DFLL values from fuses
	// revisit for full understanding
	uint32_t coarse = (*((uint32_t *)FUSES_DFLL48M_COARSE_CAL_ADDR) & FUSES_DFLL48M_COARSE_CAL_Msk) >> FUSES_DFLL48M_COARSE_CAL_Pos;
	
	SYSCTRL->DFLLVAL.bit.COARSE = coarse;

	while(!SYSCTRL->PCLKSR.bit.DFLLRDY);

	

	// turn on DFLL and set to closed loop mode

	SYSCTRL->DFLLCTRL.reg |= SYSCTRL_DFLLCTRL_MODE | SYSCTRL_DFLLCTRL_WAITLOCK | SYSCTRL_DFLLCTRL_ENABLE;

	// wait for frequency lock
	while(!SYSCTRL->PCLKSR.bit.DFLLLCKC || !SYSCTRL->PCLKSR.bit.DFLLLCKF);


	// switch GCLK0 to use DFLL

	GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(0) | GCLK_GENCTRL_SRC_DFLL48M | GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN;

	// wait for write to complete
	while(GCLK->STATUS.bit.SYNCBUSY);
}


void delay_8c(uint32_t n) {
	__asm (
	"loop:				\n" // loop for delay loop
	"	sub r0, r0, #1	\n" // n is loaded into r0, so each loop, subtract 1
	"	nop				\n"
	"	nop				\n"
	"	nop				\n"
	"	nop				\n"
	"	nop				\n"
	"	bne loop		\n" // if counter is not 0, loop
	);
}