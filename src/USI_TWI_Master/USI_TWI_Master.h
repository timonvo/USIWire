/*****************************************************************************
*
* Atmel Corporation
*
* File              : USI_TWI_Master.h
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
*                     the USI_TWI_Start_Transceiver_With_Data() function. If the transceiver
*                     returns with a fail, then use USI_TWI_Get_Status_Info to evaluate the
*                     couse of the failure.
*
****************************************************************************/
#ifndef USI_TWI_MASTER_H
#define USI_TWI_MASTER_H

#include <stdbool.h>
#include <inttypes.h>

typedef enum TransceiveResult {
	USI_TWI_MASTER_SUCCESS = 0,
	USI_TWI_MASTER_NO_DATA = 1,           // Transmission buffer is empty
	USI_TWI_MASTER_DATA_OUT_OF_BOUND = 2, // Transmission buffer is outside SRAM space
	USI_TWI_MASTER_UE_START_CON = 3,      // Unexpected Start Condition
	USI_TWI_MASTER_UE_STOP_CON = 4,       // Unexpected Stop Condition
	USI_TWI_MASTER_UE_DATA_COL = 5,       // Unexpected Data Collision (arbitration)
	USI_TWI_MASTER_NO_ACK_ON_DATA = 6,    // The slave did not acknowledge  all data
	USI_TWI_MASTER_NO_ACK_ON_ADDRESS = 7, // The slave did not acknowledge  the address
	USI_TWI_MASTER_MISSING_START_CON = 8, // Generated Start Condition not detected on bus
	USI_TWI_MASTER_MISSING_STOP_CON = 9,  // Generated Stop Condition not detected on bus
} TransceiveResult;

/**
 * Initialise the USI for TWI Master mode.
 *
 * Must be called before any of the other functions.
 */
void USI_TWI_Master_Initialise();

/**
 * Reads a number of bytes from a slave.
 *
 * This function generates a (possibly repeated) Start Condition, sends address
 * and read bit, reads resultSize bytes of data, and sends ACK.
 *
 * The sendStop parameter determines whether a Stop Condition is sent at the end
 * of the transmission. By setting this to false repeated starts can be
 * initiated.
 *
 * Success or error code (defined above) is returned.
 */
TransceiveResult USI_TWI_Master_Read(
		uint8_t address,
		uint8_t* result,
		uint8_t resultSize,
		bool sendStop);

/**
 * Writes a number of bytes to a slave.
 *
 * This function generates a (possibly repeated) Start Condition, sends address
 * and write bit, writes resultSize bytes of data, and sends ACK.
 *
 * The sendStop parameter determines whether a Stop Condition is sent at the end
 * of the transmission. By setting this to false repeated starts can be
 * initiated.
 *
 * Success or error code (defined above) is returned.
 */
TransceiveResult USI_TWI_Master_Write(
		uint8_t address,
		const uint8_t* data,
		uint8_t dataSize,
		bool sendStop);

/** Like USI_TWI_Master_Write, but expects the input data to be in PROGMEM. */
TransceiveResult USI_TWI_Master_Write_P(
		uint8_t address,
		const uint8_t* data,
		uint8_t dataSize,
		bool sendStop);

/**
 * Like USI_TWI_Master_Write, but can write the same input data more than once.
 *
 * This allows repeated patterns of data (e.g. a string of 0x00 bytes) to be
 * sent more efficiently.
 *
 * The repeat parameter determines the number of times the content pointed to by
 * the data parameter is written. The repeatOffset parameter determines which
 * parts of the data are repeated after the first iteration has completed. E.g.
 * if repeatOffset == 0, then the whole data will be repeated. If repeatOffset
 * == 1, then bytes 1 through dataSize will be repeated after the first
 * iteration.
 */
TransceiveResult USI_TWI_Master_Write_Repeated(
		uint8_t address,
		const uint8_t* data,
		uint8_t dataSize,
		bool sendStop,
		uint16_t repeat,
		uint8_t repeatOffset);

/**
 * Like USI_TWI_Master_Write_Repeated, but expects the input data to be in
 * PROGMEM.
 */
TransceiveResult USI_TWI_Master_Write_Repeated_P(
		uint8_t address,
		const uint8_t* data,
		uint8_t dataSize,
		bool sendStop,
		uint16_t repeat,
		uint8_t repeatOffset);

#endif // USI_TWI_MASTER_H
