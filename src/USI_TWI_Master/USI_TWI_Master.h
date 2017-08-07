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

#define USI_TWI_NO_DATA 0x00           // Transmission buffer is empty
#define USI_TWI_DATA_OUT_OF_BOUND 0x01 // Transmission buffer is outside SRAM space
#define USI_TWI_UE_START_CON 0x02      // Unexpected Start Condition
#define USI_TWI_UE_STOP_CON 0x03       // Unexpected Stop Condition
#define USI_TWI_UE_DATA_COL 0x04       // Unexpected Data Collision (arbitration)
#define USI_TWI_NO_ACK_ON_DATA 0x05    // The slave did not acknowledge  all data
#define USI_TWI_NO_ACK_ON_ADDRESS 0x06 // The slave did not acknowledge  the address
#define USI_TWI_MISSING_START_CON 0x07 // Generated Start Condition not detected on bus
#define USI_TWI_MISSING_STOP_CON 0x08  // Generated Stop Condition not detected on bus

/**
 * Initialise the USI for TWI Master mode.
 *
 * Must be called before any of the other functions.
 */
void USI_TWI_Master_Initialise(void);

/**
 * Returns the error code from the last transmission.
 */
unsigned char USI_TWI_Get_State_Info(void);

/**
 * USI Transmit and receive function.
 *
 * LSB of first byte in data indicates if a read or write cycles is performed.
 * If set a read operation is performed.
 *
 * Function generates a (possibly repeated) Start Condition, sends address and
 * R/W bit, reads/writes Data, and verifies/sends ACK.
 *
 * The stop parameter determines whether a Stop Condition is sent at the end of
 * the transmission. By setting this to false repeated starts can be initiated.
 *
 * Success or error code (defined above) is returned.
 */
unsigned char USI_TWI_Start_Transceiver_With_Data(
		unsigned char *msg, unsigned char msgSize, unsigned char stop);
#endif // USI_TWI_MASTER_H
