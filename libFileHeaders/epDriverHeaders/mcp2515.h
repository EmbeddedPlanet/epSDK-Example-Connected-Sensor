/****************************************************************************                                                                     *
 * Copyright (c) 2022 Embedded Planet, Inc.                                 *
 * SPDX-License-Identifier: Apache-2.0                                      *
 *                                                                          *
 * Licensed under the Apache License, Version 2.0 (the "License");          *
 * you may not use this file except in compliance with the License.         *
 * You may obtain a copy of the License at                                  *
 *                                                                          *
 *     http://www.apache.org/licenses/LICENSE-2.0                           *
 *                                                                          *
 * Unless required by applicable law or agreed to in writing, software      *
 * distributed under the License is distributed on an "AS IS" BASIS,        *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. *
 * See the License for the specific language governing permissions and      *
 * limitations under the License.                                           *
 ****************************************************************************/

/**
 * @file    mcp2515.h
 * @version 1.3.0
 * @author  Embedded Planet, Inc.
 * @author  M. Timieski
 * @date    01 MAR 2023
 * 
 * @brief Contains functions for the configuration and control of the MCP2515 SPI to CAN controller.
 * versions:
 *  1.0.0 - Initial
 *  1.1.0 - added extended addressing through additional functions
 *  1.2.0 - updated extended addressing through address switch mechanism, removed 'extended' functions
 *  1.2.1 - add #define for RXBxCTRL and RXBxDLC addresses
 *  1.3.0 - fix extended address concatenation in mcp2515_id_regs_write, add message read flag, add delay after each SPI read
 *  1.4.0 - Add ID decode function
 * 
 * Built for use with the nRF SDK 17.1
 * 
 * @see Data sheet:     https://ww1.microchip.com/downloads/en/DeviceDoc/MCP2515-Stand-Alone-CAN-Controller-with-SPI-20001801J.pdf
 * @see Error codes:    https://infocenter.nordicsemi.com/index.jsp?topic=%2Fsdk_nrf5_v17.1.0%2Fgroup__nrfx__error__codes.html
 * @see Driver Specific codes: found in header file.
 * 
 * Equation 5-2 is shown incorrectly in spec sheet, should be: TQ = 2 * (BRP + 1) * TOSC
 */

#ifndef MCP2515_H
#define MCP2515_H

#include "nrfx_spim.h"

/** @brief Used to hold configuration data of a single MCP2515. Required prior to call to mcp2515_init.
 */
typedef struct{
    nrfx_spim_t spi;            /** < SPI interface that the MCP2515 is using */ 
    uint8_t tx_message[10];    /** < Holds the current pin output data, and is used to send data to the MCP2515 */
    uint8_t rx_message[10];    /** < Holds the return data from the MCP2515. */
    uint8_t mcp_registers[64]; /** < Shadow copy of config registers, used for restoring config after (power down) sleep. */
    bool address_mode;         /** < Sets the address mode: CAN_MESSAGE_ID_STANDARD = standard (11 bit), CAN_MESSAGE_ID_EXTENDED = extended (29 bit)*/
    nrfx_err_t err;            /** < Holds the error code, refer to the Global Error Codes document at the top of this file */
} MCP2515;

/** @brief Used to hold CAN bus data of a single MCP2515. Required prior to call to mcp2515_init.
 */
typedef struct{
    uint32_t rx_id;         /** < Holds the CAN ID of the responding node */
    uint8_t rx_message[8];  /** < Holds the return data from the MCP2515. */
    bool rcv;               /** < RETURN_PASS when message received, otherwise RETURN_FAULT for receive timeout */
    bool err;               /** < RETURN_PASS when successful, otherwise RETURN_FAULT */
} MCP2515_CAN;

/**
 * @brief   Inits a MCP2515 struct with the SPI interface used, then sets up controller.
 * 
 * @param mcp2515       Pointer to the MCP2515 struct
 * @param spi           SPI interface being used
 * 
 * @return  true if successful. 
 */
bool  mcp2515_init(MCP2515 *mcp2515, nrfx_spim_t spi);

/**
 * @brief   Restores the existing config registers from a shadow copy.  
 *          This function is used when the device comes out of a sleep (power off) state.
 *          This function will not restore registers that are modified with the mcp2515_bit_modify_reg
 *          function (use mcp2515_bit_modify_reg specifically for CANCTRL and clearing interrupts only!)  
 * 
 * @param mcp2515       Pointer to the MCP2515 struct
 * 
 * @return  RETURN_PASS when successful, otherwise RETURN_FAULT.
 */
bool mcp2515_restore_config(MCP2515 *mcp2515);

/**
 * @brief   reset the controller through the SPI interface.
 * 
 * @param mcp2515       Pointer to the MCP2515 struct
 * 
 * @return  RETURN_PASS when successful, otherwise RETURN_FAULT.
 */
bool mcp2515_reset(MCP2515 *mcp2515);

/**
 * @brief   sets the address mode encoding/decoding for CAN messages.
 *          Note that the address mode can also be set by directly through
 *          MCP2515.address_mode in source code, this function is provided 
 *          for consistent access of the MCP2515 parameters.
 * 
 * @param mcp2515       Pointer to the MCP2515 struct
 * @param address_mode  Sets the address mode: CAN_MESSAGE_ID_STANDARD = standard (11 bit), CAN_MESSAGE_ID_EXTENDED = extended (29 bit)
 * 
 * @return  RETURN_PASS when successful, otherwise RETURN_FAULT.
 */
bool mcp2515_set_address_mode(MCP2515 *mcp2515, bool address_mode);

/**
 * @brief   gets the address mode encoding/decoding for CAN messages.
 *          Note that the address mode can also be read directly through
 *          MCP2515.address_mode in source code, this function is provided 
 *          for consistent access of the MCP2515 parameters.
 * 
 * @param mcp2515       Pointer to the MCP2515 struct
 * 
 * @return  CAN_MESSAGE_ID_STANDARD or CAN_MESSAGE_ID_EXTENDED.
 */
bool mcp2515_get_address_mode(MCP2515 *mcp2515);

/**
 * @brief   Read MCP2515 status register for status of transmission and reception of data over the CAN bus. 
 * 
 * @param mcp2515       Pointer to the MCP2515 struct
 * 
 * @return  8bit value read from register:
 *          bit 0: CANINTF.RX0IF (MCP2515_STATUS_CANINTF_RX0IF)
 *          bit 1: CANINTF.RX1IF (MCP2515_STATUS_CANINTF_RX1IF)
 *          bit 2: TXB0CNTRL.TXREQ (MCP2515_STATUS_TXB0CTL_TXREQ)
 *          bit 3: CANINTF.TX0IF (MCP2515_STATUS_CANINTF_TX0IF)
 *          bit 4: TXB1CNTRL.TXREQ (MCP2515_STATUS_TXB1CTL_TXREQ)
 *          bit 5: CANINTF.TX1IF (MCP2515_STATUS_CANINTF_TX1IF)
 *          bit 6: TXB2CNTRL.TXREQ (MCP2515_STATUS_TXB2CTL_TXREQ)
 *          bit 7: CANINTF.TX2IF (MCP2515_STATUS_CANINTF_TX2IF)
 *          Read Error is populated in MCP2515 structure "err" variable:
 *          NRFX_SUCCESS if successful. For all others, refer to the Global Error Codes document at the top of this file.
 */
uint8_t mcp2515_read_status(MCP2515 *mcp2515);

/**
 * @brief   Read MCP2515 register. 
 * 
 * @param mcp2515       Pointer to the MCP2515 struct
 * @param reg           register per the "Register Memory Map"
 * 
 * @return  8bit value read from register.  Read Error is populated in MCP2515 structure "err" variable:
 *          NRFX_SUCCESS if successful. For all others, refer to the Global Error Codes document at the top of this file.
 */
uint8_t mcp2515_read_reg(MCP2515 *mcp2515, uint8_t reg);

/**
 * @brief   Write MCP2515 register. 
 * 
 * @param mcp2515       Pointer to the MCP2515 struct
 * @param reg           register per the "Register Memory Map"
 * @param data          8bit register value
 * 
 * @return  RETURN_PASS when successful, otherwise RETURN_FAULT. 
 */
bool mcp2515_write_reg(MCP2515 *mcp2515, uint8_t reg, uint8_t data);

/**
 * @brief   Read MCP2515 mask and filter register set (RXFn and RXMn). The mcp2515 struct rx_message is populated with the data read.
 * 
 * @param mcp2515       Pointer to the MCP2515 struct
 * @param reg           register per the "Register Memory Map" (MCP2515_RXF0, ..RXF1, ..RXF2, ..RXF3, ..RXF4, ..RXF5, ..RXM0, ..RXM1)
 * 
 * @return  RETURN_PASS when successful, otherwise RETURN_FAULT.
 */
bool mcp2515_read_reg_set(MCP2515 *mcp2515, uint8_t reg);

/**
 * @brief   Write MCP2515 mask and filter register set (RXFn and RXMn).
 * 
 * @param mcp2515       Pointer to the MCP2515 struct
 * @param reg           register per the "Register Memory Map" (MCP2515_RXF0, ..RXF1, ..RXF2, ..RXF3, ..RXF4, ..RXF5, ..RXM0, ..RXM1)
 * @param sidh          Data sent to the register, SIDH register 
 * @param sidl          Data sent to the register, SIDL register 
 * @param eid8          Data sent to the register, EID8 register 
 * @param eid0          Data sent to the register, EID0 register 
 * 
 * @return  RETURN_PASS when successful, otherwise RETURN_FAULT.
 */
bool mcp2515_write_reg_set(MCP2515 *mcp2515, uint8_t reg, uint8_t sidh, uint8_t sidl , uint8_t eid8, uint8_t eid0);

/**
 * @brief   Write MCP2515 transmit buffer data register set (TXBnD).
 * 
 * @param mcp2515     Pointer to the MCP2515 struct
 * @param reg         register per the "Register Memory Map" (MCP2515_TXB0D, MCP2515_TXB1D, MCP2515_TXB2D)
 * @param d0..d7      Data sent to the CAN transmit buffer 
 * 
 * @return  RETURN_PASS when successful, otherwise RETURN_FAULT.
 */
bool mcp2515_write_tx_set(MCP2515 *mcp2515, uint8_t reg, uint8_t d0, uint8_t d1 , uint8_t d2, uint8_t d3, uint8_t d4, uint8_t d5, uint8_t d6,uint8_t d7);

/**
 * @brief   Read MCP2515 receive buffer data register set (RXBnD).  The mcp2515 struct rx_message is populated with the data read.
 * 
 * @param mcp2515       Pointer to the MCP2515 struct
 * @param reg           register per the "Register Memory Map" (MCP2515_RXB0D, MCP2515_RXB1D)
 * 
 * @return  RETURN_PASS when successful, otherwise RETURN_FAULT.
 */
bool mcp2515_read_rx_set(MCP2515 *mcp2515, uint8_t reg);

/**
 * @brief   Modify bits in MCP2515 registers: TXB0CTRL, TXB1CTRL, TXB2CTRL, RXB0CTRL, RXB1CTRL, CNF1, CNF2, CNF3, CANINTe, CANINTF, CFPCTRL, TXRTSCTRL, EFLG, CANCTRL
 * 
 * @param mcp2515       Pointer to the MCP2515 struct
 * @param reg           register per the "Register Memory Map"
 * @param mask          Each bit set allows the following data parameter to be set/reset
 * @param data          Data that is qualified with mask 
 * 
 * @return  true if successful.
 */
bool mcp2515_bit_modify_reg(MCP2515 *mcp2515, uint8_t reg, uint8_t mask, uint8_t data);

/**
 * 
 * d0      Number of bytes 
 * d1      CAN service code:
 *         0x01 for current data, 
 *         0x02 for freeze frame,
 *         0x03 Show stored Diagnostic Trouble Codes (DTCs),
 *         0x04 Clear Diagnostic Trouble Codes and stored values,
 *         0x05 not used
 *         0x09 Request vehicle information
 * d2      CAN PID number
 * 
 * 
 * @brief   write query to the CAN bus. 
 * 
 * @param mcp2515       Pointer to the MCP2515 struct
 * @param id            CAN ID (used as broadcast address)
 * @param d0..d7        Data sent to the CAN transmit buffer
 * 
 * @return  RETURN_PASS when successful, otherwise RETURN_FAULT.
*/
bool mcp2515_query_write(MCP2515 *mcp2515, uint32_t id, uint8_t d0, uint8_t d1 , uint8_t d2, uint8_t d3, uint8_t d4, uint8_t d5, uint8_t d6,uint8_t d7);

/**
 * @brief   read query results captured from CAN bus. 
 * 
 * @param mcp2515       Pointer to the MCP2515 struct
 * 
 * @return  The function returns a MCP2515_CAN structure: 
 *          Element err contains RETURN_PASS when successful, otherwise RETURN_FAULT.
 *          Element rx_message contains an 8 byte array that is teh CAN message.
 *          Element id contains the node that responded.
*/
MCP2515_CAN mcp2515_query_read(MCP2515 *mcp2515);

/**
 * 
 * d0      Number of bytes 
 * d1      CAN service code:
 *         0x01 for current data, 
 *         0x02 for freeze frame,
 *         0x03 Show stored Diagnostic Trouble Codes (DTCs),
 *         0x04 Clear Diagnostic Trouble Codes and stored values,
 *         0x05 not used
 *         0x09 Request vehicle information
 * d2      CAN PID number
 * 
 * 
 * @brief   write query, ready query to the CAN bus. Retry if fail.
 * 
 * @param mcp2515       Pointer to the MCP2515 struct
 * @param id            CAN ID (used as broadcast address)
 * @param d0..d7        Data sent to the CAN transmit buffer
 * @param retries       Amount of retries to attempt for successful communication
 * 
 * @return  The function returns a MCP2515_CAN structure: 
 *          Element err contains RETURN_PASS when successful, otherwise RETURN_FAULT.
 *          Element rx_message contains an 8 byte array that is teh CAN message.
 *          Element id contains the node that responded.
*/
MCP2515_CAN mcp2515_query_read_write_retries(MCP2515 *mcp2515, uint32_t id, uint8_t d0, uint8_t d1 , uint8_t d2, uint8_t d3, uint8_t d4, uint8_t d5, uint8_t d6,uint8_t d7,uint8_t retries);

/**
 * @brief   Set registers that hold ID data (Register Sets for Mask, Filters, and CAN query transmissions).  Mask bits '1' to pass to filter, '0' accept all.  Filter bits '1' accept '1', '0' accept '0'. 
 * 
 * @param mcp2515       Pointer to the MCP2515 struct
 * @param reg           Register set for mask or filter (MCP2515_RXF0, MCP2515_RXF1, MCP2515_RXF2, MCP2515_RXF3, MCP2515_RXF4, MCP2515_RXF5, MCP2515_RXM0, MCP2515_RXM1)
 * @param val           Value of mask/filter as 32bit unsigned number
  * 
 * @return  RETURN_PASS when successful, otherwise RETURN_FAULT.
 *  
 * @par Example
 * Mask   0x7FE   111 1111 1110
 * Filter 0x040   000 0100 0000
 * 
 * Recieved ID    000 0100 0000  Valid
 * Recieved ID    000 0100 0001  Valid (mask don't care on LSB allows this ID)
 * Recieved ID    000 0100 0010  not Valid (fails filter: bit 2 needs to be 0)
*/
bool mcp2515_id_regs_write(MCP2515 *mcp2515, uint8_t reg, uint32_t val);

/**
 * @brief   Supply individual register values and return the mask/filter 32bit unsigned number.
 * 
 * @param mcp2515       Pointer to the MCP2515 struct 
 * @param sidh          Data sent to the register, SIDH register 
 * @param sidl          Data sent to the register, SIDL register 
 * @param eid8          Data sent to the register, EID8 register 
 * @param eid0          Data sent to the register, EID0 register 
 * 
 * @return  returns the id value as uint32_t (as those supplied using the mcp2515_id_regs_write function).
 */
uint32_t mcp2515_id_decode(MCP2515 *mcp2515, uint8_t sidh, uint8_t sidl , uint8_t eid8, uint8_t eid0);

/**
 * @brief   Setup the CAN clock for the MCP2515.
 * 
 * @param mcp2515       Pointer to the MCP2515 struct
 * @param fosc          MCP2515 oscillator frequency
 * @param bitrate       CAN bitrate in bits per second
 * 
 * @return  RETURN_PASS when successful, otherwise RETURN_FAULT.
 */
bool mcp2515_can_speed_write(MCP2515 *mcp2515, uint32_t fosc, uint32_t bitrate);

/**
 * @brief   Return a register set name (a register set is a group of registers commonly used together) given a base register address.
 * 
 * @param reg lowest byte register used in a set
 * 
 * @return  the name of the register in a character array.
 */
char * register_set_name(uint8_t reg);

/**
 * @brief   Return a register name given the register address.
 * 
 * @param reg  the register address
 * 
 * @return  the name of the register in a character array.
 */
char * register_name(uint8_t reg);

/* 
 * Register Constants
 */

/* Error Codes (written so that multiple errors can be added)*/
#define RETURN_FAULT 0x01
#define RETURN_PASS  0x00

/*CAN address type*/
#define CAN_MESSAGE_ID_STANDARD 0x00
#define CAN_MESSAGE_ID_EXTENDED 0x01

/* User configuration */
#define CAN_SPI_CS_PORTBIT BIT4
#define CAN_SPI_CS_PORTOUT P2OUT
#define CAN_SPI_CS_PORTDIR P2DIR

/* SPI commands */
#define MCP2515_SPI_WRITE       0x02
#define MCP2515_SPI_READ        0x03
#define MCP2515_SPI_BITMOD      0x05
#define MCP2515_SPI_RTS         0x80
#define MCP2515_SPI_READ_STATUS 0xA0
#define MCP2515_SPI_RESET       0xC0

//Register Set Map
#define MCP2515_RXF0  0x00
#define MCP2515_RXF1  0x04
#define MCP2515_RXF2  0x08
#define MCP2515_RXF3  0x10
#define MCP2515_RXF4  0x14
#define MCP2515_RXF5  0x18
#define MCP2515_RXM0  0x20
#define MCP2515_RXM1  0x24
#define MCP2515_TXB0  0x31
#define MCP2515_TXB0D 0x36
#define MCP2515_TXB1  0x41
#define MCP2515_TXB1D 0x46
#define MCP2515_TXB2  0x51
#define MCP2515_TXB2D 0x56
#define MCP2515_RXB0  0x61
#define MCP2515_RXB0D 0x66
#define MCP2515_RXB1  0x71
#define MCP2515_RXB1D 0x76

//Single Register Map
#define MCP2515_BFPCTRL   0x0C
#define MCP2515_TXRTSCTRL 0x0D
#define MCP2515_CANSTAT   0x0E
#define MCP2515_CANCTRL   0x0F
#define MCP2515_CNF3      0x28
#define MCP2515_CNF2      0x29
#define MCP2515_CNF1      0x2A
#define MCP2515_CANINTE   0x2B
#define MCP2515_CANINTF   0x2C
#define MCP2515_EFLG      0x2D
#define MCP2515_TXB0CTRL  0x30
#define MCP2515_TXB0DLC   0x35
#define MCP2515_TXB1CTRL  0x40
#define MCP2515_TXB1DLC   0x45
#define MCP2515_TXB2CTRL  0x50
#define MCP2515_TXB2DLC   0x55
#define MCP2515_RXB0CTRL  0x60
#define MCP2515_RXB0DLC   0x65
#define MCP2515_RXB1CTRL  0x70
#define MCP2515_RXB1DLC   0x75

//Control Register Mask
#define MCP2515_TXBNCTRL_TXP0   0x01
#define MCP2515_TXBNCTRL_TXP1   0x02
#define MCP2515_TXBNCTRL_TXREQ  0x08
#define MCP2515_TXBNCTRL_TXERR  0x10
#define MCP2515_TXBNCTRL_MLOA   0x20
#define MCP2515_TXBNCTRL_ABTF   0x40

//Status Register Bits
#define MCP2515_STATUS_CANINTF_RX0IF   0x01
#define MCP2515_STATUS_CANINTF_RX1IF   0x02
#define MCP2515_STATUS_TXB0CTL_TXREQ   0x04
#define MCP2515_STATUS_CANINTF_TX0IF   0x08
#define MCP2515_STATUS_TXB1CTL_TXREQ   0x10
#define MCP2515_STATUS_CANINTF_TX1IF   0x20
#define MCP2515_STATUS_TXB2CTL_TXREQ   0x40
#define MCP2515_STATUS_CANINTF_TX2IF   0x80

//Control Register Bits
#define MCP2515_CANCTRL_REQOP_NORMAL 0x00
#define MCP2515_CANCTRL_REQOP_CONFIG 0x04
#define MCP2515_CANCTRL_REQOP_MASK 0xE0
#define MCP2515_CANCTRL_REQOP_CONFIGURATION 0x80

//Config 2 Register Bits
#define MCP2515_CNF2_BTLMODE    0x80
#define MCP2515_CNF2_SAM        0x40
#define MCP2515_CNF2_PRSEG_MASK 0x07
#define MCP2515_CNF2_PHSEG_MASK 0x38

#endif
