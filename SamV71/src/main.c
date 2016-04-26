/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# "Insert system clock initialization code here" comment
 * -# Minimal main function that starts with a call to board_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#include <asf.h>
#include <inttypes.h>
void timer_init_test (void);
uint32_t counter = 0 ;


int main (void)
{
	sysclk_init();
 	board_init();
	udc_start();
	
	timer_init_test();


	char data [100] ;
	while(1)
	{
		int data_size = sprintf(data,"counter : %" PRIu32 " TC0.0 = %" PRIu32 "|TC0.1 = %" PRIu32 "|TC0.2 = %" PRIu32 " \r"
		,counter
		,TC0->TC_CHANNEL[0].TC_CV
		,TC0->TC_CHANNEL[1].TC_CV
		,TC0->TC_CHANNEL[2].TC_CV);
		
		for (int i = 0 ; i < data_size ; i++)
		{
			udi_cdc_putc(data[i]);
			delay_ms(1);
		}	
	}
}


void TC0_Handler	(void)
{
	TC0->TC_CHANNEL[0].TC_SR;
	
	counter ++ ;
	
	if (counter == 750)
	{
		counter = 0;
		
		ioport_set_pin_level(LED0_GPIO, !ioport_get_pin_level(LED0_GPIO) );
	}
}

void TC1_Handler	(void)
{
	TC0->TC_CHANNEL[1].TC_SR;
}

void TC2_Handler	(void)
{
	TC0->TC_CHANNEL[2].TC_SR;
}


void timer_init_test (void)
{
	NVIC_SetPriority(TC0_IRQn, 0);
	NVIC_EnableIRQ(TC0_IRQn);
	NVIC_SetPriority(TC1_IRQn, 1);
	NVIC_EnableIRQ(TC1_IRQn);
	NVIC_SetPriority(TC2_IRQn, 2);
	NVIC_EnableIRQ(TC2_IRQn);
		
	pmc_enable_periph_clk(ID_TC0);
	pmc_enable_periph_clk(ID_TC1);
	pmc_enable_periph_clk(ID_TC2);
	// 	pmc_enable_periph_clk(ID_TC3);
	// 	pmc_enable_periph_clk(ID_TC4);
	// 	pmc_enable_periph_clk(ID_TC5);
	// 	pmc_enable_periph_clk(ID_TC6);
	// 	pmc_enable_periph_clk(ID_TC7);
	// 	pmc_enable_periph_clk(ID_TC8);
	// 	pmc_enable_periph_clk(ID_TC9);
	// 	pmc_enable_periph_clk(ID_TC10);
	// 	pmc_enable_periph_clk(ID_TC11);
		
	tc_init(TC0,0,TC_CMR_TCCLKS_TIMER_CLOCK4 | TC_CMR_WAVE |TC_CMR_WAVSEL_UP_RC);
	TC0->TC_CHANNEL[0].TC_RC = 15625;
	tc_enable_interrupt(TC0,0,TC_IER_CPCS);
	tc_start(TC0,0);
		
	tc_init(TC0,1,TC_CMR_TCCLKS_TIMER_CLOCK4 | TC_CMR_WAVE |TC_CMR_WAVSEL_UP_RC);
	TC0->TC_CHANNEL[1].TC_RC=65000;
	tc_enable_interrupt(TC0,1,TC_IER_CPCS);
	tc_start(TC0,1);
		
	tc_init(TC0,2,TC_CMR_TCCLKS_TIMER_CLOCK4 | TC_CMR_WAVE |TC_CMR_WAVSEL_UP_RC);
	TC0->TC_CHANNEL[2].TC_RC=65000;
	tc_enable_interrupt(TC0,2,TC_IER_CPCS);
	tc_start(TC0,2);
}