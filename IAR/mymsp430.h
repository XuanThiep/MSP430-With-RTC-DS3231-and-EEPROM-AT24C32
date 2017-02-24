#ifndef __MY_MSP430_H
#define __MY_MSP430_H


#include "driverlib.h"
#include "stdio.h"

/* Define Private enum */
typedef enum
{
	SMCLK_CLOCK_DIVIDER_1 = DIVM__1,
	SMCLK_CLOCK_DIVIDER_2 = DIVM__2,
	SMCLK_CLOCK_DIVIDER_4 = DIVM__4,
	SMCLK_CLOCK_DIVIDER_8 = DIVM__8,
	SMCLK_CLOCK_DIVIDER_16 = DIVM__16,
	SMCLK_CLOCK_DIVIDER_32 = DIVM__32
}SMCLK_DIVIDER;

typedef enum
{
	SLAVE_MEMORY_ADDRESS_SIZE_8BIT=	0x00,
	SLAVE_MEMORY_ADDRESS_SIZE_16BIT=  0x01
}slave_memory_address_size;


/* @Brief	: 	Initialize clock module, run in mode external crystal on XT2
 * @Para	:	XT2_Clock_Hz - This is value of crystal on XT2 (Hz)
 * @Return	:	None
 * @Note	:   None
 */
void Clk_Using_Crystal_Init(uint32_t XT2_Clock_Hz);


/* @Brief	: 	Initialize clock module, run in mode DCO
 *
 * @Para	:	+ MCLK_FREQ_KHZ - This is value of desired master clock (KHz)
 * 				+ XT2_FREQ_KHZ  - This is value of crystal on XT2 (KHz)
 * 				+ SMCLK_CLOCK_DIVIDER_X - This is value of prescaler of Sub Master clock. F (submasterclock) = F (masterclock)/SMCLK_CLOCK_DIVIDER_X.
 * @Return	:	None
 * @Note	:   None
 */
void Clk_Using_DCO_Init(uint32_t MCLK_FREQ_KHZ,uint32_t XT2_FREQ_KHZ,SMCLK_DIVIDER SMCLK_CLOCK_DIVIDER_X);


/* @Brief	: 	Initialize timerA2, run in mode timer basic count up for delay function
 * @Para	:	None
 * @Return	:	None
 * @Note	:   User must be implement function Delay_Using_TimerA2_ISR()
 * 				at vector=TIMER2_A1_VECTOR. Then call Delay_ms() to start delay.
 */
void Delay_Using_TimerA2_Init(void);


/* @Brief	: 	Timer A2 Interrupt Service Rountie for Delay_ms() function
 * @Para	:	None
 * @Return	:	None
 * @Note	:   User must be implement function Delay_Using_TimerA2_ISR()
 *  			at vector=TIMER2_A1_VECTOR.
 */
void Delay_Using_TimerA2_ISR(void);


/* @Brief	: 	Delay function
 * @Para	:	Time to delay (millis seconds)
 * @Return	:	None
 * @Note	:   None
 */
void Delay_ms(uint32_t time_in_milliseconds);



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
		uint32_t desiredSpiClock_Hz);



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
		uint8_t length);



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
		uint8_t SCL_PORT,uint16_t SCL_PIN);



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
 * 				+ 1 - If error occur
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
		uint8_t length);






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
 * 				+ 1 - If error occur
 *
 * @Note	:   A short delay need (about 2ms)
 * 				between this function and I2c_Master_Send_Multibyte_To_Slave function.
 */
uint8_t I2c_master_receive_multibyte_from_slave(uint16_t USCI_base_address,
		uint8_t _7bit_slave_add,
		uint16_t slave_memory_address_start,
		slave_memory_address_size slave_memory_address_size,
		uint8_t* data_buffer,
		uint8_t length);

#endif


