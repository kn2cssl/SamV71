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
	ioport_set_pin_dir(PIO_PC5_IDX,IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(PIO_PC6_IDX,IOPORT_DIR_OUTPUT);

	char data [100] ;
	int tim [4];
	while(1)
	{
		int data_size = sprintf(data," %" PRIu32 ", %" PRIu32 ", %" PRIu32 ", %" PRIu32 ", %" PRIu32 " \r"
		,tim[0]//TC2->TC_CHANNEL[0].TC_RA
		,tim[1]//TC2->TC_CHANNEL[0].TC_RB
		,tim[2]//ioport_get_pin_level(PIO_PC29_IDX)
		,tim[3]//ioport_get_pin_level(PIO_PC30_IDX)
		,TC2->TC_CHANNEL[0].TC_SR);
		
		for (int i = 0 ; i < data_size ; i++)
		{
			udi_cdc_putc(data[i]);
			//delay_ms(1);
		}	
		delay_ms(1);
		ioport_set_pin_level(PIO_PC5_IDX, HIGH );
		tim[0]=TC2->TC_CHANNEL[0].TC_RA;
		
		delay_ms(2);
		ioport_set_pin_level(PIO_PC5_IDX, LOW );
		tim[1]=TC2->TC_CHANNEL[0].TC_RA;
		
		delay_ms(3);
		ioport_set_pin_level(PIO_PC5_IDX, HIGH );
		tim[2]=TC2->TC_CHANNEL[0].TC_RA;
		
		delay_ms(4);
		ioport_set_pin_level(PIO_PC5_IDX, LOW );
		tim[3]=TC2->TC_CHANNEL[0].TC_RA;
		//delay_us(100);
		
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
		
	// for timing
	pmc_enable_periph_clk(ID_TC0);
	pmc_enable_periph_clk(ID_TC1);
	pmc_enable_periph_clk(ID_TC2);
	
	// for PWM
	pmc_enable_periph_clk(ID_TC3);
	pmc_enable_periph_clk(ID_TC4);
	pmc_enable_periph_clk(ID_TC5);
	
	pmc_enable_periph_clk(ID_TC6);
	pmc_enable_periph_clk(ID_TC7);
	//	pmc_enable_periph_clk(ID_TC8);
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
	
	
	// PWM
	tc_init(TC1,0,TC_CMR_TCCLKS_TIMER_CLOCK4 | TC_CMR_WAVE |TC_CMR_WAVSEL_UP_RC | TC_CMR_ACPC_SET | TC_CMR_ACPA_CLEAR | TC_CMR_BCPC_SET | TC_CMR_BCPB_CLEAR | TC_CMR_EEVT_XC2);
	TC1->TC_CHANNEL[0].TC_RC = 26250;
	TC1->TC_CHANNEL[0].TC_RA = 26250/8;
	TC1->TC_CHANNEL[0].TC_RB = 26250/4;
	tc_start(TC1,0);
	
	ioport_set_pin_mode(PIO_PC23_IDX, IOPORT_MODE_MUX_B);
	ioport_disable_pin(PIO_PC23_IDX);
	
	ioport_set_pin_mode(PIO_PC24_IDX, IOPORT_MODE_MUX_B);
	ioport_disable_pin(PIO_PC24_IDX);
	
	tc_init(TC1,1,TC_CMR_TCCLKS_TIMER_CLOCK4 | TC_CMR_WAVE |TC_CMR_WAVSEL_UP_RC | TC_CMR_ACPC_SET | TC_CMR_ACPA_CLEAR | TC_CMR_BCPC_SET | TC_CMR_BCPB_CLEAR | TC_CMR_EEVT_XC2);
	TC1->TC_CHANNEL[1].TC_RC = 26250;
	TC1->TC_CHANNEL[1].TC_RA = 26250/200;
	TC1->TC_CHANNEL[1].TC_RB = 26250/400;
	tc_start(TC1,1);
	
	ioport_set_pin_mode(PIO_PC26_IDX, IOPORT_MODE_MUX_B);
	ioport_disable_pin(PIO_PC26_IDX);
	
	ioport_set_pin_mode(PIO_PC27_IDX, IOPORT_MODE_MUX_B);
	ioport_disable_pin(PIO_PC27_IDX);
	
	tc_init(TC1,2,TC_CMR_TCCLKS_TIMER_CLOCK4 | TC_CMR_WAVE |TC_CMR_WAVSEL_UP_RC | TC_CMR_ACPC_SET | TC_CMR_ACPA_CLEAR | TC_CMR_BCPC_SET | TC_CMR_BCPB_CLEAR | TC_CMR_EEVT_XC2);
	TC1->TC_CHANNEL[2].TC_RC = 26250;
	TC1->TC_CHANNEL[2].TC_RA = 26250/10;
	TC1->TC_CHANNEL[2].TC_RB = 26250/100;
	tc_start(TC1,2);
	
	ioport_set_pin_mode(PIO_PC29_IDX, IOPORT_MODE_MUX_B);
	ioport_disable_pin(PIO_PC29_IDX);
	
	ioport_set_pin_mode(PIO_PC30_IDX, IOPORT_MODE_MUX_B);
	ioport_disable_pin(PIO_PC30_IDX);
	
	// Counter
	tc_init(TC2,0,TC_CMR_TCCLKS_TIMER_CLOCK4 | TC_CMR_ABETRG | TC_CMR_ETRGEDG_EDGE | TC_CMR_LDRA_EDGE );
	tc_start(TC2,0);
	
		
	// Frequency : TC_CMR_TCCLKS_TIMER_CLOCK2 = MCK / 8 = 150MHz / 8 = 18.75MHz  => 
	// Duration  : 26250 / 18750000 = 0.0014 s = 3t
	// t = L_motor / R_motor = 0.00056 mH / 1.2 ohm =  0.0014 
}