/*****************************************************************************
*
* Atmel Corporation
*
* File              : USI_TWI_Master.c
* Date              : $Date: 2016-7-15 $
* Updated by        : $Author: Atmel $
*
* Support mail      : avr@atmel.com
*
* Supported devices : All device with USI module can be used.
*                     The example is written for the ATmega169, ATtiny26 and ATtiny2313
*
* AppNote           : AVR310 - Using the USI module as a TWI Master
*
* Description       : This is an implementation of an TWI master using
*                     the USI module as basis. The implementation assumes the AVR to
*                     be the only TWI master in the system and can therefore not be
*                     used in a multi-master system.
* Usage             : Initialize the USI module by calling the USI_TWI_Master_Initialise()
*                     function. Hence messages/data are transceived on the bus using
*                     the USI_TWI_Transceive() function. The transceive function
*                     returns a status byte, which can be used to evaluate the
*                     success of the transmission.
*
****************************************************************************/
#include "USI_TWI_Master.h"

#if __GNUC__
#ifndef F_CPU
#define F_CPU 4000000
#endif
#include <avr/io.h>
#include <util/delay.h>
#else
#include <inavr.h>
#include <ioavr.h>
#endif

#include <avr/pgmspace.h>

//********** Defines **********//
// Defines controlling timing limits
#define TWI_FAST_MODE

#define SYS_CLK 4000.0 // [kHz]

#ifdef TWI_FAST_MODE                            // TWI FAST mode timing limits. SCL = 100-400kHz
#define T2_TWI ((SYS_CLK * 1300) / 1000000) + 1 // >1,3us
#define T4_TWI ((SYS_CLK * 600) / 1000000) + 1  // >0,6us

#else                                           // TWI STANDARD mode timing limits. SCL <= 100kHz
#define T2_TWI ((SYS_CLK * 4700) / 1000000) + 1 // >4,7us
#define T4_TWI ((SYS_CLK * 4000) / 1000000) + 1 // >4,0us
#endif

// Defines controling code generating
//#define PARAM_VERIFICATION
//#define NOISE_TESTING
#define SIGNAL_VERIFY

// USI_TWI messages and flags and bit masks
//#define SUCCESS   7
//#define MSG       0
/****************************************************************************
  Bit and byte definitions
****************************************************************************/
#define TWI_ADR_BITS 1 // Bit position for LSB of the slave address bits in the init byte.
#define TWI_NACK_BIT 0 // Bit position for (N)ACK bit.

// Device dependant defines
#if __GNUC__
#include "usi_io.h"
#else //__GNUC__
#if defined(__AT90Mega169__) || defined(__ATmega169__) || defined(__AT90Mega165__) || defined(__ATmega165__)           \
    || defined(__ATmega325__) || defined(__ATmega3250__) || defined(__ATmega645__) || defined(__ATmega6450__)          \
    || defined(__ATmega329__) || defined(__ATmega3290__) || defined(__ATmega649__) || defined(__ATmega6490__)
#define DDR_USI DDRE
#define PORT_USI PORTE
#define PIN_USI PINE
#define PORT_USI_SDA PORTE5
#define PORT_USI_SCL PORTE4
#define PIN_USI_SDA PINE5
#define PIN_USI_SCL PINE4
#endif

#if defined(__ATtiny25__) || defined(__ATtiny45__) || defined(__ATtiny85__) || defined(__AT90Tiny26__)                 \
    || defined(__ATtiny26__)
#define DDR_USI DDRB
#define PORT_USI PORTB
#define PIN_USI PINB
#define PORT_USI_SDA PORTB0
#define PORT_USI_SCL PORTB2
#define PIN_USI_SDA PINB0
#define PIN_USI_SCL PINB2
#endif

#if defined(__AT90Tiny2313__) || defined(__ATtiny2313__)
#define DDR_USI DDRB
#define PORT_USI PORTB
#define PIN_USI PINB
#define PORT_USI_SDA PORTB5
#define PORT_USI_SCL PORTB7
#define PIN_USI_SDA PINB5
#define PIN_USI_SCL PINB7
#endif
#ifndef DDR_USI_CL
#define DDR_USI_CL DDR_USI
#define PORT_USI_CL PORT_USI
#define PIN_USI_CL PIN_USI
#endif
#endif //__GNUC__

// General defines
#define TRUE 1
#define FALSE 0

#if __GNUC__
#define DELAY_T2TWI (_delay_us(T2_TWI / 4))
#define DELAY_T4TWI (_delay_us(T4_TWI / 4))
#else
#define DELAY_T2TWI (__delay_cycles(T2_TWI))
#define DELAY_T4TWI (__delay_cycles(T4_TWI))
#endif

typedef enum TransceiveMode {
	WRITE = 0,
	READ = 1,
} TransceiveMode;

static TransceiveResult USI_TWI_Master_Write_Byte(uint8_t value);
static uint8_t USI_TWI_Master_Transfer();
static TransceiveResult USI_TWI_Master_Stop();

void USI_TWI_Master_Initialise()
{
	PORT_USI |= (1 << PIN_USI_SDA); // Enable pullup on SDA, to set high as released state.
	PORT_USI_CL |= (1 << PIN_USI_SCL); // Enable pullup on SCL, to set high as released state.

	DDR_USI_CL |= (1 << PIN_USI_SCL); // Enable SCL as output.
	DDR_USI |= (1 << PIN_USI_SDA); // Enable SDA as output.

	USIDR = 0xFF;                                           // Preload dataregister with "released level" data.
	USICR = (0 << USISIE) | (0 << USIOIE) |                 // Disable Interrupts.
	        (1 << USIWM1) | (0 << USIWM0) |                 // Set USI in Two-wire mode.
	        (1 << USICS1) | (0 << USICS0) | (1 << USICLK) | // Software stobe as counter clock source
	        (0 << USITC);
	USISR = (1 << USISIF) | (1 << USIOIF) | (1 << USIPF) | (1 << USIDC) | // Clear flags,
	        (0x0 << USICNT0);                                             // and reset counter.
}

// Prepare register value to: Clear flags, and set USI to shift 8 bits i.e.
// count 16 clock edges.
#define USISR_COUNT_8_BITS (\
		(1 << USISIF) | (1 << USIOIF) | (1 << USIPF) | (1 << USIDC) | \
		(0x0 << USICNT0))

// Prepare register value to: Clear flags, and set USI to shift 1 bits i.e.
// count 2 clock edges.
#define USISR_COUNT_1_BIT (\
		(1 << USISIF) | (1 << USIOIF) | (1 << USIPF) | (1 << USIDC) | \
		(0xE << USICNT0))

static TransceiveResult USI_TWI_Master_Prepare_Read_Write() {
#ifdef PARAM_VERIFICATION
	if (msg > (uint8_t *)RAMEND) // Test if address is outside SRAM space
	{
		return USI_TWI_MASTER_DATA_OUT_OF_BOUND;
	}
	if (msgSize == 0) // Test if the transmission buffer is empty
	{
		return USI_TWI_MASTER_NO_DATA;
	}
#endif

#ifdef NOISE_TESTING // Test if any unexpected conditions have arrived prior to this execution.
	if (USISR & (1 << USISIF)) {
		return USI_TWI_MASTER_UE_START_CON;
	}
	if (USISR & (1 << USIPF)) {
		return USI_TWI_MASTER_UE_STOP_CON;
	}
	if (USISR & (1 << USIDC)) {
		return USI_TWI_MASTER_UE_DATA_COL;
	}
#endif

	DDR_USI |= (1 << PIN_USI_SDA);

	/* Release SCL to ensure that (repeated) Start can be performed */
	PORT_USI_CL |= (1 << PIN_USI_SCL); // Release SCL.
	while (!(PIN_USI_CL & (1 << PIN_USI_SCL)))
		; // Verify that SCL becomes high.
#ifdef TWI_FAST_MODE
	DELAY_T4TWI; // Delay for T4TWI if TWI_FAST_MODE
#else
	DELAY_T2TWI; // Delay for T2TWI if TWI_STANDARD_MODE
#endif

	/* Generate Start Condition */
	PORT_USI &= ~(1 << PIN_USI_SDA); // Force SDA LOW.
	DELAY_T4TWI;
	PORT_USI_CL &= ~(1 << PIN_USI_SCL); // Pull SCL LOW.
	PORT_USI |= (1 << PIN_USI_SDA);  // Release SDA.

#ifdef SIGNAL_VERIFY
	if (!(USISR & (1 << USISIF))) {
		return USI_TWI_MASTER_MISSING_START_CON;
	}
#endif
	return USI_TWI_MASTER_SUCCESS;
}

TransceiveResult USI_TWI_Master_Read(
		uint8_t address,
		uint8_t* result,
		uint8_t resultSize,
		bool sendStop) {
	TransceiveResult prepareResult = USI_TWI_Master_Prepare_Read_Write();
	if (prepareResult != USI_TWI_MASTER_SUCCESS) {
		return prepareResult;
	}

	// Send the address byte.
	TransceiveResult addressResult =
			USI_TWI_Master_Write_Byte((address << 1) | 1);
	if (addressResult != USI_TWI_MASTER_SUCCESS) {
		return addressResult == USI_TWI_MASTER_NO_ACK_ON_DATA
				? USI_TWI_MASTER_NO_ACK_ON_ADDRESS : addressResult;
	}

	// If we need to write N data bytes, then we need to transmit N + 1 bytes in
	// total: 1 byte for the address + N bytes of actual data. But we only need
	// to write the address once.
	do {
		// If this is a read cycle.
		DDR_USI &= ~(1 << PIN_USI_SDA); // Enable SDA as input.
		USISR = USISR_COUNT_8_BITS;
		*(result++) = USI_TWI_Master_Transfer();

		/* Prepare to generate ACK (or NACK in case of End Of Transmission) */
		if (resultSize == 1) {
			// If transmission of last byte was performed.
			USIDR = 0xFF; // Load NACK to confirm End Of Transmission.
		} else {
			USIDR = 0x00; // Load ACK. Set data register bit 7 (output for SDA) low.
		}
		USISR = USISR_COUNT_1_BIT;
		USI_TWI_Master_Transfer(); // Generate ACK/NACK.
	} while (--resultSize > 0); // Until all data is sent.

	// Send a STOP condition on the TWI bus if requested, otherwise return.
	return sendStop ? USI_TWI_Master_Stop() : USI_TWI_MASTER_SUCCESS;
}

static TransceiveResult USI_TWI_Master_Write_Internal(
		uint8_t address,
		const uint8_t* data,
		uint8_t dataSize,
		bool sendStop,
		uint16_t repeat,
		uint8_t repeatOffset,
		bool isProgmem);

inline TransceiveResult USI_TWI_Master_Write(
		uint8_t address,
		const uint8_t* data,
		uint8_t dataSize,
		bool sendStop) {
	return USI_TWI_Master_Write_Internal(
			address, data, dataSize, sendStop, 0, 0, false /* isProgmem */);
}

inline TransceiveResult USI_TWI_Master_Write_P(
		uint8_t address,
		const uint8_t* data,
		uint8_t dataSize,
		bool sendStop) {
	return USI_TWI_Master_Write_Internal(
			address, data, dataSize, sendStop, 0, 0, true /* isProgmem */);
}

inline TransceiveResult USI_TWI_Master_Write_Repeated(
		uint8_t address,
		const uint8_t* data,
		uint8_t dataSize,
		bool sendStop,
		uint16_t repeat,
		uint8_t repeatOffset) {
	return USI_TWI_Master_Write_Internal(
			address, data, dataSize, sendStop, repeat, repeatOffset,
			false /* isProgmem */);
}

inline TransceiveResult USI_TWI_Master_Write_Repeated_P(
		uint8_t address,
		const uint8_t* data,
		uint8_t dataSize,
		bool sendStop,
		uint16_t repeat,
		uint8_t repeatOffset) {
	return USI_TWI_Master_Write_Internal(
			address, data, dataSize, sendStop, repeat, repeatOffset,
			true /* isProgmem */);
}

TransceiveResult USI_TWI_Master_Write_Internal(
		uint8_t address,
		const uint8_t* data,
		uint8_t dataSize,
		bool sendStop,
		uint16_t repeat,
		uint8_t repeatOffset,
		bool isProgmem) {
	TransceiveResult prepareResult = USI_TWI_Master_Prepare_Read_Write();

	if (prepareResult != USI_TWI_MASTER_SUCCESS) {
		return prepareResult;
	}

	// Send the address byte.
	TransceiveResult addressResult =
			USI_TWI_Master_Write_Byte((address << 1) | 0);
	if (addressResult != USI_TWI_MASTER_SUCCESS) {
		return addressResult == USI_TWI_MASTER_NO_ACK_ON_DATA
				? USI_TWI_MASTER_NO_ACK_ON_ADDRESS : addressResult;
	}

	uint16_t cyclesLeft = repeat + 1;
	do {
		uint8_t bytesLeft =
			cyclesLeft == repeat + 1 ? dataSize : dataSize - repeatOffset;
		const uint8_t* dataPtr =
			cyclesLeft == repeat + 1 ? data : data + repeatOffset;
		do {
			uint8_t byte = isProgmem ? pgm_read_byte_near(dataPtr++) : *(dataPtr++);
			TransceiveResult writeResult = USI_TWI_Master_Write_Byte(byte);
			if (writeResult != USI_TWI_MASTER_SUCCESS) {
				return writeResult;
			}
		} while (--bytesLeft > 0); // Until all data sent/received.
	} while(--cyclesLeft > 0); // And all cycles have been completed.

	// Send a STOP condition on the TWI bus if requested, otherwise return.
	return sendStop ? USI_TWI_Master_Stop() : USI_TWI_MASTER_SUCCESS;
}

static TransceiveResult USI_TWI_Master_Write_Byte(uint8_t value) {
	PORT_USI_CL &= ~(1 << PIN_USI_SCL);         // Pull SCL LOW.
	USISR = USISR_COUNT_8_BITS;
	USIDR = value;
	USI_TWI_Master_Transfer();

	/* Clock and verify (N)ACK from slave */
	DDR_USI &= ~(1 << PIN_USI_SDA); // Enable SDA as input.
	USISR = USISR_COUNT_1_BIT;
	if (USI_TWI_Master_Transfer() & (1 << TWI_NACK_BIT)) {
		return USI_TWI_MASTER_NO_ACK_ON_DATA;
	}
	return USI_TWI_MASTER_SUCCESS;
}

/**
 * Core function for shifting data in and out from the USI. The USISR register
 * must be initialised to send the necessary number of bits prior to calling
 * this function. Data to be sent has to be placed into the USIDR prior to
 * calling this function. Data read, will be return'ed from the function.
 */
static uint8_t USI_TWI_Master_Transfer()
{
	// Prepare clocking.
	uint8_t usicrStrobeClock =
			(0 << USISIE) | (0 << USIOIE) |                 // Interrupts disabled
			(1 << USIWM1) | (0 << USIWM0) |                 // Set USI in Two-wire mode.
			(1 << USICS1) | (0 << USICS0) | (1 << USICLK) | // Software clock strobe as source.
			(1 << USITC);                                   // Toggle Clock Port.
	do {
		DELAY_T2TWI;
		USICR = usicrStrobeClock; // Generate positve SCL edge.
		while (!(PIN_USI_CL & (1 << PIN_USI_SCL)))
			; // Wait for SCL to go high.
		DELAY_T4TWI;
		USICR = usicrStrobeClock; // Generate negative SCL edge.
	} while (!(USISR & (1 << USIOIF))); // Check for transfer complete.

	DELAY_T2TWI;
	USIDR = 0xFF;                  // Release SDA.
	DDR_USI |= (1 << PIN_USI_SDA); // Enable SDA as output.

	return USIBR; // Return the data from the USIBR buffer register.
}

/**
 * Function for generating a TWI Stop Condition. Used to release the TWI bus.
 */
static TransceiveResult USI_TWI_Master_Stop(void)
{
	PORT_USI &= ~(1 << PIN_USI_SDA); // Pull SDA low.
	PORT_USI_CL |= (1 << PIN_USI_SCL);  // Release SCL.
	while (!(PIN_USI_CL & (1 << PIN_USI_SCL)))
		; // Wait for SCL to go high.
	DELAY_T4TWI;
	PORT_USI |= (1 << PIN_USI_SDA); // Release SDA.
	DELAY_T2TWI;

#ifdef SIGNAL_VERIFY
	if (!(USISR & (1 << USIPF))) {
		return USI_TWI_MASTER_MISSING_STOP_CON;
	}
#endif

	return USI_TWI_MASTER_SUCCESS;
}
