#include "driverlib.h"
#include "mymsp430.h"

#define DS3231_ADD		0x68 //8bit memory address
#define AT24C32_ADD 	0x50 //16bit memory address

uint8_t receive_data[7];
uint8_t send_data[7];



#pragma vector=TIMER2_A1_VECTOR
__interrupt void TimerA2_ISR(void)
{
	Delay_Using_TimerA2_ISR();
}



void main( void )
{
	/* Stop watchdog timer */
	WDT_A_hold(WDT_A_BASE);

	Clk_Using_DCO_Init(16000,8000,SMCLK_CLOCK_DIVIDER_2);

	Delay_Using_TimerA2_Init();

	/* Initialize gpio for led, to indicate system status
	 * P1.4 on, P1.5 off indicate system status ok
	 * P1.4 off, P1.5 on indicate system status error
	 */
	GPIO_setAsOutputPin(GPIO_PORT_P1,GPIO_PIN4|GPIO_PIN5);

	/* Normaly System status is ok */
	GPIO_setOutputHighOnPin(GPIO_PORT_P1,GPIO_PIN4);
	GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN5);

	/* P3.0 as USCI-SDA
	 * P3.1 as USCI-SCL */
	I2c_Init(USCI_B0_BASE,
			GPIO_PORT_P3,GPIO_PIN0,
			GPIO_PORT_P3,GPIO_PIN1);

	send_data[0]=1;
	send_data[1]=2;
	send_data[2]=3;
	send_data[3]=4;
	send_data[4]=5;
	send_data[5]=6;
	send_data[6]=7;

	// Test send multi byte
	//I2c_Master_Send_Multibyte_To_Slave(USCI_B0_BASE,AT24C32_ADD,0,SLAVE_MEMORY_ADDRESS_SIZE_16BIT,send_data,7);

	//Test send one byte with loop
	for(uint8_t i=0;i<7;i++)
	{
		if(I2c_Master_Send_Multibyte_To_Slave(USCI_B0_BASE,AT24C32_ADD,
				i,
				SLAVE_MEMORY_ADDRESS_SIZE_16BIT,
				&send_data[i],
				1))
		{
			/* System status is error */
			GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN4);
			GPIO_setOutputHighOnPin(GPIO_PORT_P1,GPIO_PIN5);
		}
		Delay_ms(2); //Nees delay between continuous call
	}

	//Delay_ms(2);
	while(1)
	{
		// Test receive multi byte
		//		I2c_master_receive_multibyte_from_slave(USCI_B0_BASE,
		//				AT24C32_ADD,
		//				0x00,
		//				SLAVE_MEMORY_ADDRESS_SIZE_16BIT,
		//				receive_data,
		//				7);

		//	Test receive one byte with loop
		for(uint8_t i=0;i<7;i++)
		{
			if(I2c_master_receive_multibyte_from_slave(USCI_B0_BASE,
					AT24C32_ADD,
					i,
					SLAVE_MEMORY_ADDRESS_SIZE_16BIT,
					&receive_data[i],
					1))
			{
				/* System status is error */
				GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN4);
				GPIO_setOutputHighOnPin(GPIO_PORT_P1,GPIO_PIN5);
			}
		}
		Delay_ms(300);
	}
}



