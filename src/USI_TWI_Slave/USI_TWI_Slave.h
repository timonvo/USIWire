// This file has been prepared for Doxygen automatic documentation generation.
/*! \file ********************************************************************
 *
 * Atmel Corporation
 *
 * File              : USI_TWI_Slave.h
 * Compiler          : IAR EWAAVR 4.11A
 * Revision          : $Revision: 6351 $
 * Date              : $Date: 2010-01-29 20:15:43 +0800 (Fri, 29 Jan 2010) $
 * Updated by        : $Author: hskinnemoen $
 *
 * Support mail      : avr@atmel.com
 *
 * Supported devices : All device with USI module can be used.
 *                     The example is written for the ATmega169, ATtiny26 & ATtiny2313
 *
 * AppNote           : AVR312 - Using the USI module as a TWI slave
 *
 * Description       : Header file for USI_TWI driver
 *
 *
 *
 ****************************************************************************/

#ifndef USI_TWI_SLAVE_H
#define USI_TWI_SLAVE_H

/**
 * Initialise USI for TWI Slave mode.
 *
 * Must be called before any of the other functions.
 */
void          USI_TWI_Slave_Initialise(unsigned char);

/**
 * Disable the USI after having been initialised by USI_TWI_Slave_Initialise.
 */
void          USI_TWI_Slave_Disable();

/**
 * Puts data in the transmission buffer, Waits if buffer is full.
 */
void          USI_TWI_Transmit_Byte(unsigned char);

/**
 * Returns a byte from the receive buffer. Waits if buffer is empty.
 */
unsigned char USI_TWI_Receive_Byte(void);

/**
 * Returns a byte from the receive buffer without incrementing TWI_RxTail. Waits
 * if buffer is empty.
 */
unsigned char USI_TWI_Peek_Receive_Byte(void);

/**
 * Check if there is data in the receive buffer.
 */
unsigned char USI_TWI_Data_In_Receive_Buffer(void);

/**
 * Check if there is space in the transmission buffer.
 */
unsigned char USI_TWI_Space_In_Transmission_Buffer(void);

/**
 * Check if there is an active data session.
 */
unsigned char USI_TWI_Slave_Is_Active();

/**
 * Called when the master asks the slave to transmit a byte of data.
 *
 * Implementors of this callback must queue data to be transmitted using
 * USI_TWI_Transmit_Byte, and must do so only *after* this callback is called,
 * not before. Otherwise the read request from the master will not be ACKed.
 */
void (*USI_TWI_On_Slave_Transmit)(void);

/**
 * Called when the master finished a transmission to the slave.
 *
 * The tranmission may consist of multiple bytes, and they will all be available
 * in the received buffer, which can be accessed using USI_TWI_Receive_Byte or
 * USI_TWI_Peek_Receive_Byte.
 */
void (*USI_TWI_On_Slave_Receive)(int);

#endif // USI_TWI_SLAVE_H
