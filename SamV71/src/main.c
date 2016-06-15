#include <asf.h>
#include <inttypes.h>
void timer_init_test (void);
void pwm_init_test(void);
void adc_init_test (void);
void dac_init_test (void);
void wireless_connection ( void );
void spi_init_pins(void);
void spi_init_module(void);
void pin_edge_handler(const uint32_t id, const uint32_t index);
void nrf_init (void);
void module_id_set(void);
uint32_t counter = 0 ;
uint32_t dac_counter=0;
//! DAC channel used for test
#define DACC_CHANNEL        0 // (PB13)
//#define DACC_CHANNEL        1 // (PD00)
	char data [100] ;
	int tim [4];
	char Address[_Address_Width] = { 0x11, 0x22, 0x33, 0x44, 0x55};
	char spi_rx_buf[_Buffer_Size] ;
int main (void)
{
	sysclk_init();
 	board_init();
	udc_start();
	pwm_init_test();
	adc_init_test();
	dac_init_test();
	spi_init_pins();
	spi_init_module();
	module_id_set();
	nrf_init ();
		
	timer_init_test();
	ioport_set_pin_dir(PIO_PC5_IDX,IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(PIO_PC6_IDX,IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(PIO_PC31_IDX,IOPORT_DIR_INPUT);
	ioport_set_pin_mode(PIO_PC31_IDX,IOPORT_MODE_PULLDOWN);
	ioport_disable_pin(PIO_PC31_IDX);
	



	while(1)
	{
		ioport_set_pin_level(LED1_GPIO, HIGH);
		dac_counter ++;
		afec_start_software_conversion(AFEC1);
		//while (afec_get_interrupt_status(AFEC1) & (1 << AFEC_CHANNEL_6));
		uint32_t result = afec_channel_get_value(AFEC1, AFEC_CHANNEL_6);
		
		dacc_write_conversion_data(DACC, dac_counter, DACC_CHANNEL);
		
		int data_size = sprintf(data," %ld, %ld, %ld, %ld, %ld \r"
		,spi_rx_buf[0]//result//tim[0]//TC2->TC_CHANNEL[0].TC_RA
		,spi_rx_buf[1]//tim[1]//TC2->TC_CHANNEL[0].TC_RB
		,spi_rx_buf[2]//tim[2]//ioport_get_pin_level(PIO_PC29_IDX)
		,spi_rx_buf[3]//tim[3]//ioport_get_pin_level(PIO_PC30_IDX)
		,spi_rx_buf[4]//TC2->TC_CHANNEL[0].TC_SR);
		);
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
	
	if (counter == 75)
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


void pin_edge_handler(const uint32_t id, const uint32_t index)
{
	if ((id == ID_PIOB) && (index == ioport_pin_to_mask(IRQ))){
			wireless_connection();
	}
}


void wireless_connection ( void )
{
	uint8_t status = NRF24L01_WriteReg(W_REGISTER | STATUSe, _TX_DS|_MAX_RT|_RX_DR);
	if((status & _RX_DR) == _RX_DR)
	{

		//wdt_reset();
		NRF24L01_Read_RX_Buf(spi_rx_buf, _Buffer_Size);
		if(spi_rx_buf[0] == MODULE_ID )
		{
			ioport_set_pin_level(LED1_GPIO, LOW);
			NRF24L01_Write_TX_Buf(spi_rx_buf, _Buffer_Size);
			//signal_strength++;
		}
	}
	
	if ((status&_MAX_RT) == _MAX_RT)
	{
		NRF24L01_Flush_TX();
	}
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
	
// 	ioport_set_pin_mode(PIO_PC23_IDX, IOPORT_MODE_MUX_B);
// 	ioport_disable_pin(PIO_PC23_IDX);
	
// 	ioport_set_pin_mode(PIO_PC24_IDX, IOPORT_MODE_MUX_B);
// 	ioport_disable_pin(PIO_PC24_IDX);
	
	tc_init(TC1,1,TC_CMR_TCCLKS_TIMER_CLOCK4 | TC_CMR_WAVE |TC_CMR_WAVSEL_UP_RC | TC_CMR_ACPC_SET | TC_CMR_ACPA_CLEAR | TC_CMR_BCPC_SET | TC_CMR_BCPB_CLEAR | TC_CMR_EEVT_XC2);
	TC1->TC_CHANNEL[1].TC_RC = 26250;
	TC1->TC_CHANNEL[1].TC_RA = 26250/200;
	TC1->TC_CHANNEL[1].TC_RB = 26250/400;
	tc_start(TC1,1);
	
	//ioport_set_pin_mode(PIO_PC26_IDX, IOPORT_MODE_MUX_B);
	//ioport_disable_pin(PIO_PC26_IDX);
	//
	//ioport_set_pin_mode(PIO_PC27_IDX, IOPORT_MODE_MUX_B);
	//ioport_disable_pin(PIO_PC27_IDX);
	
	tc_init(TC1,2,TC_CMR_TCCLKS_TIMER_CLOCK4 | TC_CMR_WAVE |TC_CMR_WAVSEL_UP_RC | TC_CMR_ACPC_SET | TC_CMR_ACPA_CLEAR | TC_CMR_BCPC_SET | TC_CMR_BCPB_CLEAR | TC_CMR_EEVT_XC2);
	TC1->TC_CHANNEL[2].TC_RC = 26250;
	TC1->TC_CHANNEL[2].TC_RA = 26250/10;
	TC1->TC_CHANNEL[2].TC_RB = 26250/100;
	tc_start(TC1,2);
	
// 	ioport_set_pin_mode(PIO_PC29_IDX, IOPORT_MODE_MUX_B);
// 	ioport_disable_pin(PIO_PC29_IDX);
// 	
// 	ioport_set_pin_mode(PIO_PC30_IDX, IOPORT_MODE_MUX_B);
// 	ioport_disable_pin(PIO_PC30_IDX);
	
	// Counter
	tc_init(TC2,0,TC_CMR_TCCLKS_TIMER_CLOCK4 | TC_CMR_ABETRG | TC_CMR_ETRGEDG_EDGE | TC_CMR_LDRA_EDGE );
	tc_start(TC2,0);
}


// Frequency : TC_CMR_TCCLKS_TIMER_CLOCK2 = MCK / 8 = 150MHz / 8 = 18.75MHz  =>
// Duration  : 26250 / 18750000 = 0.0014 s = 3t
// t = L_motor / R_motor = 0.00056 mH / 1.2 ohm =  0.0014
void pwm_init_test(void)
{
	pwm_channel_t pwm_channel_instance;
	pmc_enable_periph_clk(ID_PWM0);
	pwm_channel_disable(PWM0, PWM_CHANNEL_0);
	pwm_clock_t clock_setting = {
		.ul_clka = 10000 * 100,
		.ul_clkb = 0,
		.ul_mck = sysclk_get_cpu_hz()
	};
	pwm_init(PWM0, &clock_setting);
	pwm_channel_instance.ul_prescaler = PWM_CMR_CPRE_CLKA;
	pwm_channel_instance.ul_period = 100;
	pwm_channel_instance.ul_duty = 90;
	pwm_channel_instance.channel = PWM_CHANNEL_0;
	pwm_channel_instance.polarity = PWM_HIGH;
	pwm_channel_instance.ul_spread = 0;
	pwm_channel_init(PWM0, &pwm_channel_instance);
	
	
	ioport_set_pin_mode(PIO_PA0_IDX, IOPORT_MODE_MUX_A);
	ioport_disable_pin(PIO_PA0_IDX);
	
	ioport_set_pin_mode(PIO_PA19_IDX, IOPORT_MODE_MUX_B);
	ioport_disable_pin(PIO_PA19_IDX);
	
	pwm_channel_enable(PWM0, PWM_CHANNEL_0);
}


void adc_init_test (void)
{
	afec_enable(AFEC1);
	struct afec_config afec_cfg;
	afec_get_config_defaults(&afec_cfg);
	afec_init(AFEC1, &afec_cfg);
	struct afec_ch_config afec_ch_cfg;
	afec_ch_get_config_defaults(&afec_ch_cfg);
	/*
	 * Because the internal AFEC offset is 0x200, it should cancel it and shift
	 * down to 0.
	 */
	afec_channel_set_analog_offset(AFEC1, AFEC_CHANNEL_6, 0x200);
	afec_ch_cfg.gain = AFEC_GAINVALUE_0;
	afec_ch_set_config(AFEC1, AFEC_CHANNEL_6, &afec_ch_cfg);
	afec_set_trigger(AFEC1, AFEC_TRIG_SW);
	/* Enable channel for AFEC_CHANNEL_6. */
	afec_channel_enable(AFEC1, AFEC_CHANNEL_6);
}

void dac_init_test (void)
{
	/* Enable clock for DACC */
	sysclk_enable_peripheral_clock(ID_DACC);
	/* Reset DACC registers */
	dacc_reset(DACC);
	/* Half word transfer mode */
	dacc_set_transfer_mode(DACC, 0);
	/* Enable output channel DACC_CHANNEL */
	dacc_enable_channel(DACC, DACC_CHANNEL);
	/* Set up analog current */
	dacc_set_analog_control(DACC, (DACC_ACR_IBCTLCH0(0x02) | DACC_ACR_IBCTLCH1(0x02)));	
}

void spi_init_pins(void)
{
	pio_set_peripheral(PIOD,PIO_PERIPH_B,ioport_pin_to_mask(SPI0_MISO_GPIO));
	pio_set_peripheral(PIOD,PIO_PERIPH_B,ioport_pin_to_mask(SPI0_MOSI_GPIO));
	pio_set_peripheral(PIOD,PIO_PERIPH_B,ioport_pin_to_mask(SPI0_NPCS1_GPIO));
	pio_set_peripheral(PIOD,PIO_PERIPH_B,ioport_pin_to_mask(SPI0_SPCK_GPIO));
		//pmc_enable_periph_clk(ID_PIOB);
		pio_set_input(PIOB,ioport_pin_to_mask(IRQ), PIO_PULLUP);
		pio_handler_set(PIOB, ID_PIOB, ioport_pin_to_mask(IRQ), PIO_IT_FALL_EDGE, pin_edge_handler);
		pio_enable_interrupt(PIOB, ioport_pin_to_mask(IRQ));
		//NVIC_SetPriority(PIOB_IRQn,6);
		NVIC_EnableIRQ(PIOB_IRQn);
		
		ioport_enable_pin(CE);
		ioport_set_pin_dir(CE,IOPORT_DIR_OUTPUT);


}
void spi_init_module(void)
{
	struct spi_device nrf24l01_spi_device = {
		.id =  1
	};
	spi_master_init(SPI0);
	spi_master_setup_device(SPI0, &nrf24l01_spi_device, SPI_MODE_0, 8000000UL, 0);
	spi_enable(SPI0);
}

void nrf_init (void)
{

	delay_ms(11);
	NRF24L01_Clear_Interrupts();
	NRF24L01_Flush_TX();
	NRF24L01_Flush_RX();
	NRF24L01_Init_milad(_RX_MODE, _CH_1, _2Mbps, Address, _Address_Width, _Buffer_Size, RF_PWR_MAX);
	NRF24L01_WriteReg(W_REGISTER | EN_AA, 0x01);
	NRF24L01_WriteReg(W_REGISTER | DYNPD,0x01);
	NRF24L01_WriteReg(W_REGISTER | FEATURE,0x06);

	NRF24L01_CE_HIGH;//rx mode  ?
	delay_us(130);
}

void module_id_set(void)
{
	Address[4] = (MODULE_ID << 4 ) | MODULE_ID ;
}
