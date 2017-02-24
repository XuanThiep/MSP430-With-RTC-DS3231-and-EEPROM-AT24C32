#include "mymsp430.h"

/* Private Global Variable */
volatile uint32_t time_keeper=0;





/* @Brief	: 	Initialize clock module, run in mode external crystal on XT2
 * @Para	:	XT2_Clock_Hz - This is value of crystal on XT2 (Hz)
 * @Return	:	None
 * @Note	:   None
 */
void Clk_Using_Crystal_Init(uint32_t XT2_Clock_Hz)
{
	// All to port P5

	/* PIN5 -> XT1 OUT, PIN4 -> XT1 IN */
	/* PIN3 -> XT2 OUT, PIN2 -> XT2 IN */

	/* Configure pins for Crystals XT1 and XT2 */
	GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5,GPIO_PIN2|GPIO_PIN4);
	GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P5,GPIO_PIN3|GPIO_PIN5);
	UCS_setExternalClockSource(32768,XT2_Clock_Hz);

	/* Initialize the crystals */
	UCS_turnOnXT2(UCS_XT2_DRIVE_4MHZ_8MHZ);
	UCS_turnOnLFXT1(UCS_XT1_DRIVE_0,UCS_XCAP_3);

	/* Use the crystals to set the clock */

	// Master clock run direct from XT2 clock */
	UCS_initClockSignal(UCS_MCLK,UCS_XT2CLK_SELECT,UCS_CLOCK_DIVIDER_1);

	// Sub Master clock run direct from XT2 clock */
	UCS_initClockSignal(UCS_SMCLK,UCS_XT2CLK_SELECT,UCS_CLOCK_DIVIDER_1);

	// Auxilary clock run direct from XT1 clock */
	UCS_initClockSignal(UCS_ACLK,UCS_XT1CLK_SELECT,UCS_CLOCK_DIVIDER_1);

}



/* @Brief	: 	Initialize clock module, run in mode DCO
 *
 * @Para	:	+ MCLK_FREQ_KHZ - This is value of desired master clock (KHz)
 * 				+ XT2_FREQ_KHZ  - This is value of crystal on XT2 (KHz)
 * 				+ SMCLK_CLOCK_DIVIDER_X - This is value of prescaler of Sub Master clock. F (submasterclock) = F (masterclock)/SMCLK_CLOCK_DIVIDER_X.
 * @Return	:	None
 * @Note	:   None
 */
void Clk_Using_DCO_Init(uint32_t MCLK_FREQ_KHZ,uint32_t XT2_FREQ_KHZ,SMCLK_DIVIDER SMCLK_CLOCK_DIVIDER_X )
{
	uint32_t MCLK_FLLREF_RATIO = MCLK_FREQ_KHZ/(XT2_FREQ_KHZ/4);

	/* Set core power mode */
	PMM_setVCore(PMM_CORE_LEVEL_3);

	/* Connect Pins to Crystals */
	GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5,GPIO_PIN4|GPIO_PIN2);
	GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P5,GPIO_PIN5|GPIO_PIN3);

	/* Unit is Hz */
	UCS_setExternalClockSource(32768,XT2_FREQ_KHZ*1000);

	/* Turn on XT crystals */
	UCS_turnOnXT2(UCS_XT2_DRIVE_4MHZ_8MHZ);
	UCS_turnOnLFXT1(UCS_XT1_DRIVE_0,UCS_XCAP_3);

	UCS_initClockSignal(
			UCS_FLLREF,  // The reference for Frequency Locked Loop
			UCS_XT2CLK_SELECT,  // Select XT2
			UCS_CLOCK_DIVIDER_4 // FLL ref.  (XT2/4)
	);

	/* Init FLL Automatic for MCLK and SMCLK */
	UCS_initFLLSettle(MCLK_FREQ_KHZ,MCLK_FLLREF_RATIO);

	/* Optional: set SMCLK to something else than full speed */
	UCS_initClockSignal(UCS_SMCLK, UCS_DCOCLKDIV_SELECT, SMCLK_CLOCK_DIVIDER_X);

	/* Set auxiliary clock */
	UCS_initClockSignal(UCS_ACLK,UCS_XT1CLK_SELECT,UCS_CLOCK_DIVIDER_1);

}



/* @Brief	: 	Initialize timerA2, run in mode timer basic count up for delay function
 * @Para	:	None
 * @Return	:	None
 * @Note	:   User must be implement function Delay_Using_TimerA2_ISR()
 * 				at vector=TIMER2_A1_VECTOR. Then call Delay_ms() to start delay.
 */
void Delay_Using_TimerA2_Init(void)
{
	Timer_A_clearTimerInterrupt(TIMER_A2_BASE);

	Timer_A_initUpModeParam param;
	param.clockSource = TIMER_A_CLOCKSOURCE_SMCLK; //SMCLK = 8Mhz
	param.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_64;

	/* fcpu=fMCLK=16E6Hz, ftimer=fSMCLK/64= (fMCLK/2)/64 */

	param.timerPeriod = 124; //1ms
	param.timerInterruptEnable_TAIE= TIMER_A_TAIE_INTERRUPT_ENABLE;
	param.timerClear = TIMER_A_DO_CLEAR;
	param.startTimer= false;
	Timer_A_initUpMode(TIMER_A2_BASE,&param);
	__enable_interrupt();
}



/* @Brief	: 	Timer A2 Interrupt Service Rountie for Delay_ms() function
 * @Para	:	None
 * @Return	:	None
 * @Note	:   User must be implement function Delay_Using_TimerA2_ISR()
 *  			at vector=TIMER2_A1_VECTOR.
 */
void Delay_Using_TimerA2_ISR(void)
{
	if(Timer_A_getInterruptStatus(TIMER_A2_BASE))
	{
		if(time_keeper!=0)
		{
			time_keeper--;
		}
		else
		{
			/* If done, We stop timer to reduce power consumption */
			Timer_A_stop(TIMER_A2_BASE);
		}
		Timer_A_clearTimerInterrupt(TIMER_A2_BASE);
	}
}



/* @Brief	: 	Delay function
 * @Para	:	Time to delay (millis seconds)
 * @Return	:	None
 * @Note	:   None
 */
void Delay_ms(uint32_t time_in_milliseconds)
{
	time_keeper=time_in_milliseconds;

	/* Reset Counter register of Timer */
	HWREG16(TIMER_A2_BASE + OFS_TAxR) = 0;

	/* Start Timer A2 */
	Timer_A_startCounter(TIMER_A2_BASE,TIMER_A_UP_MODE);

	while(time_keeper);
}




/* @Brief	: 	Initialize USCI run in mode SPI (CPOL=0,CPHA=0)
 *
 * @Para	:	+ USCI_X_BASE - This is base address of USCI Module.
 *
 * 				+ MOSI_PORT - This is base address of MOSI Port
 * 				+ MOSI_PIN - This is pin number of MOSI Pin
 *
 * 				+ MISO_PORT - This is base address of MISO Port
 * 				+ MISO_PIN - This is pin number of MISO Pin
 *
 * 				+ SCK_PORT - This is base address of SCK Port
 * 				+ SCK_PIN - This is pin number of SCK Pin
 *
 * 				+ desiredSpiClock_Hz - This is clock frequency on SCK pin.
 * @Return	:	None
 * @Note	:   None
 */
void Spi_Master_Init(uint16_t USCI_X_BASE,
		uint8_t MOSI_PORT,uint16_t MOSI_PIN,
		uint8_t MISO_PORT,uint16_t MISO_PIN,
		uint8_t SCK_PORT,uint16_t SCK_PIN,
		uint32_t desiredSpiClock_Hz)
{
	/* Configure Pins for MOSI, MISO, SCK as Module Function Input */
	GPIO_setAsPeripheralModuleFunctionInputPin(MOSI_PORT,MOSI_PIN);
	GPIO_setAsPeripheralModuleFunctionInputPin(MISO_PORT,MISO_PIN);
	GPIO_setAsPeripheralModuleFunctionInputPin(SCK_PORT,SCK_PIN);

	if((USCI_X_BASE==USCI_B0_BASE)||(USCI_X_BASE==USCI_B1_BASE))
	{
		USCI_B_SPI_initMasterParam para;

		para.selectClockSource = USCI_B_SPI_CLOCKSOURCE_SMCLK;
		para.clockSourceFrequency = UCS_getSMCLK();
		para.desiredSpiClock = desiredSpiClock_Hz;
		para.msbFirst = USCI_B_SPI_MSB_FIRST;
		para.clockPhase = USCI_B_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT;
		para.clockPolarity =  USCI_B_SPI_CLOCKPOLARITY_INACTIVITY_LOW;

		USCI_B_SPI_initMaster(USCI_X_BASE,&para);

		USCI_B_SPI_enable(USCI_X_BASE);
	}
	else if ((USCI_X_BASE==USCI_A0_BASE)||(USCI_X_BASE==USCI_A1_BASE))
	{
		USCI_A_SPI_initMasterParam para;

		para.selectClockSource = USCI_A_SPI_CLOCKSOURCE_SMCLK;
		para.clockSourceFrequency = UCS_getSMCLK();
		para.desiredSpiClock = desiredSpiClock_Hz;
		para.msbFirst = USCI_A_SPI_MSB_FIRST;
		para.clockPhase = USCI_A_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT;
		para.clockPolarity =  USCI_A_SPI_CLOCKPOLARITY_INACTIVITY_LOW;

		USCI_A_SPI_initMaster(USCI_X_BASE,&para);

		USCI_A_SPI_enable(USCI_X_BASE);
	}
}


/* @Brief	: 	Send and receive single or multi bytes to slave device.
 *
 * @Para	:	+ 	USCI_X_BASE - This is base address of USCI Module.
 *
 * 				+	CS_PORT - This is GPIO PORT of CS pin, for select slave device.
 *
 *				+	CS_PIN - This is GPIO PIN of CS pin, for select slave device.
 *
 *				+ 	send_buffer - This is pointer to buffer, which data is written.
 *
 *				+ 	receive_buffer - This is pointer to buffer, which data is stored.
 *					If user only want send data, puts NULL parameter in this position.
 *
 *				+	length	- The length of bytes want to send/receives.
 *
 * @Return	:	None
 *
 * @Note	:   + This is fullduplex mode.
 * 				+ User must be initialize gpio CS_PIN as output before call this function.
 */
void Spi_Master_Send_Receive_Data(uint16_t USCI_X_BASE,
		uint8_t CS_PORT,uint16_t CS_PIN,
		uint8_t* send_buffer,uint8_t* receive_buffer,
		uint8_t length)
{
	if((USCI_X_BASE==USCI_B0_BASE)||(USCI_X_BASE==USCI_B1_BASE))
	{
		/* Empty receive buffer */
		uint8_t temp = USCI_B_SPI_receiveData(USCI_X_BASE);

		/* Enable chip select */
		GPIO_setOutputLowOnPin(CS_PORT,CS_PIN);

		for(uint8_t i=0;i<length;i++)
		{
			USCI_B_SPI_transmitData(USCI_X_BASE,*(send_buffer+i));
			while(USCI_B_SPI_isBusy(USCI_X_BASE));
			if(receive_buffer != NULL)
			{
				*(receive_buffer+i) = USCI_B_SPI_receiveData(USCI_X_BASE);
			}
		}

		/* Disable chip select */
		GPIO_setOutputHighOnPin(CS_PORT,CS_PIN);
	}
	else if ((USCI_X_BASE==USCI_A0_BASE)||(USCI_X_BASE==USCI_A1_BASE))
	{
		/* Empty receive buffer */
		uint8_t temp = USCI_A_SPI_receiveData(USCI_X_BASE);

		/* Enable chip select */
		GPIO_setOutputLowOnPin(CS_PORT,CS_PIN);

		for(uint8_t i=0;i<length;i++)
		{
			USCI_A_SPI_transmitData(USCI_X_BASE,*(send_buffer+i));
			while(USCI_A_SPI_isBusy(USCI_X_BASE));
			if(receive_buffer != NULL)
			{
				*(receive_buffer+i) = USCI_A_SPI_receiveData(USCI_X_BASE);
			}
		}

		/* Disable chip select */
		GPIO_setOutputHighOnPin(CS_PORT,CS_PIN);
	}
}




/* @Brief	: 	Initialize I2C Module run in mode master with frequency = 400Khz
 *
 * @Para	:	+ USCI_base_address - This is base address of USCI Module.
 *
 * 				+ SDA_PORT - This is GPIO PORT of SDA pin
 * 				+ SDA_PIN  - This is GPIO PIN of SDA pin
 *
 * 				+ SCL_PORT - This is GPIO PORT of SCL pin
 * 				+ SCL_PIN  - This is GPIO PIN of SCL pin
 * @Return	:	None
 * @Note	:   None
 */
void I2c_Init(uint16_t USCI_base_address,
		uint8_t SDA_PORT,uint16_t SDA_PIN,
		uint8_t SCL_PORT,uint16_t SCL_PIN)
{
	/* Config gpio pin for I2C Peripheral */
	GPIO_setAsPeripheralModuleFunctionInputPin(SDA_PORT,SDA_PIN);
	GPIO_setAsPeripheralModuleFunctionInputPin(SCL_PORT,SCL_PIN);

	USCI_B_I2C_initMasterParam para;
	para.selectClockSource = USCI_B_I2C_CLOCKSOURCE_SMCLK;
	para.i2cClk = UCS_getSMCLK();
	para.dataRate = USCI_B_I2C_SET_DATA_RATE_400KBPS; // 400Khz on SCL pin
	USCI_B_I2C_initMaster(USCI_base_address,&para);
	USCI_B_I2C_enable(USCI_base_address);
}




/* @Brief	: 	Send one or multi byte to slave
 *
 * @Para	:	+ USCI_base_address - This is base address of USCI Module.
 *
 *				+ _7bit_slave_add - This is 7 bit address of slave device.
 *				Normaly this is value in device datasheet.
 *
 *				+ slave_memory_address_start - This is address of register
 *				in slave device, which is started to write.
 *
 *				+ slave_memory_address_size - This is type of address of
 *				register in slave device, 8 bit or 16 bit.
 *
 *				+ data_buffer - This is pointer to buffer, which data is written
 *
 *				+ length - The length of bytes want to write.
 *
 * @Return	:	+ 0 - If write OK
 * 				+ !0 - If error occur
 *
 * @Note	:   A short delay need (about 2ms) if user call this function continuous or
 * 				between this function and I2c_master_receive_multibyte_from_slave function.
 */
uint8_t I2c_Master_Send_Multibyte_To_Slave(
		uint16_t USCI_base_address,
		uint8_t _7bit_slave_add,
		uint16_t slave_memory_address_start,
		slave_memory_address_size slave_memory_address_size,
		uint8_t* data_buffer,
		uint8_t length)
{
	uint16_t time_out= 1000;

	//Specify slave address
	HWREG16(USCI_base_address + OFS_UCBxI2CSA) = (_7bit_slave_add);

	//Set Transmit mode
	USCI_B_I2C_setMode(USCI_base_address,USCI_B_I2C_TRANSMIT_MODE);

	//Send start condition.
	HWREG8(USCI_base_address + OFS_UCBxCTL1) |= UCTR + UCTXSTT;

	//Poll for transmit interrupt flag.
	while((!(HWREG8(USCI_base_address + OFS_UCBxIFG) & UCTXIFG)) && (--time_out));

	if(!time_out) return 1;

	//Send memory address
	if(slave_memory_address_size == SLAVE_MEMORY_ADDRESS_SIZE_8BIT)
	{
		//Send 8bit memory address
		HWREG8(USCI_base_address + OFS_UCBxTXBUF) = slave_memory_address_start;
	}
	else
	{
		//Send MSB of memory address.
		HWREG8(USCI_base_address + OFS_UCBxTXBUF) = slave_memory_address_start>>8;

		//Poll for transmit interrupt flag.
		while((!(HWREG8(USCI_base_address + OFS_UCBxIFG) & UCTXIFG)) && (--time_out));

		if(!time_out) return 1;

		//Send LSB of memory address.
		HWREG8(USCI_base_address + OFS_UCBxTXBUF) = slave_memory_address_start & 0x00ff;
	}

	if(length == 1)
	{
		//Poll for transmit interrupt flag.
		while((!(HWREG8(USCI_base_address + OFS_UCBxIFG) & UCTXIFG)) && (--time_out));

		if(!time_out) return 1;

		//Send single byte data.
		HWREG8(USCI_base_address + OFS_UCBxTXBUF) = *data_buffer;
	}
	else
	{
		for(uint8_t i=0;i<length-1;i++)
		{
			//Poll for transmit interrupt flag.
			while((!(HWREG8(USCI_base_address + OFS_UCBxIFG) & UCTXIFG)) && (--time_out));

			if(!time_out) return 1;

			//Send single byte data.
			HWREG8(USCI_base_address + OFS_UCBxTXBUF) = *(data_buffer+i);
		}

		//Poll for transmit interrupt flag then send the last byte

		//Poll for transmit interrupt flag.
		while((!(HWREG8(USCI_base_address + OFS_UCBxIFG) & UCTXIFG)) && (--time_out));

		if(!time_out) return 1;

		//Send single byte data.
		HWREG8(USCI_base_address + OFS_UCBxTXBUF) = *(data_buffer+length-1);
	}

	//Poll for transmit interrupt flag.
	while((!(HWREG8(USCI_base_address + OFS_UCBxIFG) & UCTXIFG)) && (--time_out));

	if(!time_out) return 1;

	//Send stop condition.
	HWREG8(USCI_base_address + OFS_UCBxCTL1) |= UCTXSTP;

	//Wait for Stop to finish
	while((HWREG8(USCI_base_address + OFS_UCBxCTL1) & UCTXSTP) && (--time_out));

	if(!time_out) return 1;

	return 0;
}





/* @Brief	: 	Read one or multi byte from slave
 *
 * @Para	:	+ USCI_base_address - This is base address of USCI Module.
 *
 *				+ _7bit_slave_add - This is 7 bit address of slave device.
 *				Normaly this is value in device datasheet.
 *
 *				+ slave_memory_address_start - This is address of register
 *				in slave device, which is started to read.
 *
 *				+ slave_memory_address_size - This is type of address of
 *				register in slave device, 8 bit or 16 bit.
 *
 *				+ data_buffer - This is pointer to buffer, which data is stored
 *
 *				+ length - The length of bytes want to read.
 *
 * @Return	:	+ 0 - If write OK
 * 				+ !0 - If error occur
 *
 * @Note	:   A short delay need (about 2ms)
 * 				between this function and I2c_Master_Send_Multibyte_To_Slave function.
 */
uint8_t I2c_master_receive_multibyte_from_slave(uint16_t USCI_base_address,
		uint8_t _7bit_slave_add,
		uint16_t slave_memory_address_start,
		slave_memory_address_size slave_memory_address_size,
		uint8_t* data_buffer,
		uint8_t length)
{
	uint16_t time_out=1000;

	USCI_B_I2C_setSlaveAddress(USCI_base_address,_7bit_slave_add);

	//Set transmit mode
	USCI_B_I2C_setMode(USCI_base_address,USCI_B_I2C_TRANSMIT_MODE);

	//Send start condition.
	HWREG8(USCI_base_address + OFS_UCBxCTL1) |= UCTR + UCTXSTT;

	//Poll for transmit interrupt flag.
	while((!(HWREG8(USCI_base_address + OFS_UCBxIFG) & UCTXIFG)) && (--time_out));
	if(!time_out) return 1;

	switch(slave_memory_address_size)
	{
	case SLAVE_MEMORY_ADDRESS_SIZE_8BIT:

		//Send 8bit memory address
		HWREG8(USCI_base_address + OFS_UCBxTXBUF) = slave_memory_address_start;

		break;
	case SLAVE_MEMORY_ADDRESS_SIZE_16BIT:

		//Send MSB of memory address.
		HWREG8(USCI_base_address + OFS_UCBxTXBUF) = slave_memory_address_start>>8;

		//Poll for transmit interrupt flag.
		while((!(HWREG8(USCI_base_address + OFS_UCBxIFG) & UCTXIFG)) &&(--time_out));
		if(!time_out) return 1;

		//Send LSB of memory address.
		HWREG8(USCI_base_address + OFS_UCBxTXBUF) = slave_memory_address_start & 0x00ff;

		break;
	}

	//Poll for transmit interrupt flag.
	while((!(HWREG8(USCI_base_address + OFS_UCBxIFG) & UCTXIFG)) && (--time_out));
	if(!time_out) return 1;

	/* Request recevei from slave */

	//Set USCI in Receive mode
	HWREG8(USCI_base_address + OFS_UCBxCTL1) &= ~UCTR;

	//Send start condition.
	HWREG8(USCI_base_address + OFS_UCBxCTL1) |= UCTXSTT;

	//Poll for Start bit to complete
	while((HWREG8(USCI_base_address + OFS_UCBxCTL1) & UCTXSTT) &&(--time_out));
	if(!time_out) return 1;

	if(length == 1)
	{
		//Send stop condition.
		HWREG8(USCI_base_address + OFS_UCBxCTL1) |= UCTXSTP;

		/* Wait and receive data from buffer of I2C */
		while((!(HWREG8(USCI_base_address + OFS_UCBxIFG) & UCRXIFG)) && (--time_out));
		if(!time_out) return 1;

		//Read a byte.
		*data_buffer = HWREG8(USCI_base_address + OFS_UCBxRXBUF);
	}
	else
	{
		for(uint8_t i=0;i<length-1;i++)
		{
			//Poll for receive interrupt flag.
			while((!(HWREG8(USCI_base_address + OFS_UCBxIFG) & UCRXIFG)) && (--time_out));
			if(!time_out) return 1;

			// Receive data
			*(data_buffer+i) = HWREG8(USCI_base_address + OFS_UCBxRXBUF);
		}

		//Send stop condition.
		HWREG8(USCI_base_address + OFS_UCBxCTL1) |= UCTXSTP;

		//Poll for receive interrupt flag
		while((!(HWREG8(USCI_base_address + OFS_UCBxIFG) & UCRXIFG)) && (--time_out));
		if(!time_out) return 1;

		//Read the last byte data
		*(data_buffer+length-1) = HWREG8(USCI_base_address + OFS_UCBxRXBUF);

		//Wait for Stop to finish
		while((HWREG8(USCI_base_address + OFS_UCBxCTL1) & UCTXSTP) && (--time_out));
		if(!time_out) return 1;
	}

	return 0;
}
