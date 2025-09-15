/*
 * mcp2515_driver.h
 *
 *  Created on: Sep 10, 2025
 *      Author: eldon
 */

#ifndef MCP2515_DRIVER_MCP2515_DRIVER_H_
#define MCP2515_DRIVER_MCP2515_DRIVER_H_

#include <stdint.h>
#include <string.h>
#include "stm32wbxx_hal.h"	//HAL library for STM32WBxx series, Replace with your STM32 series HAL library

//SPI CS (Slave Select)  and INT pin defines
#define CAN_INT_Pin GPIO_PIN_0
#define CAN_INT_GPIO_Port GPIOD
#define CAN_INT_EXTI_IRQn EXTI0_IRQn
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOA

#define MAX_ADDRESS 	0x7D //Maximum register address

#define MCP2515_INTERNAL_CLK_FREQ	8000000 //in Hz. MCP2515 module installed oscillator, modify based on your installed oscillator on module

extern SPI_HandleTypeDef hspi1; 	//extern SPI handle

//@SPI_Commands_t
typedef enum{
	SPI_COMMAND_RESET 			= 0xC0,
	SPI_COMMAND_READ 			= 0x03,
	SPI_COMMAND_READ_RX_BUFFER 	= 0x90,
	SPI_COMMAND_WRITEREGISTER 	= 0x02,
	SPI_COMMAND_LOAD_TX_BUFFER 	= 0x40,
	SPI_COMMAND_RTS 			= 0x80,
	SPI_COMMAND_READ_STATUS 	= 0xA0,
	SPI_COMMAND_RX_STATUS 		= 0xB0,
	SPI_COMMAND_BIT_MODIFY 		= 0x05
}SPI_Commands_t;

//@Read_RxBufferLoc_t
typedef enum{
	START_RXB0SIDH  = SPI_COMMAND_READ_RX_BUFFER | 0x00, //Receive Buffer 0, start at RXB0SIDH (Receive Buffer 0 Data Byte 0 Register: 0x61 Address)
	START_RXB0D0 	= SPI_COMMAND_READ_RX_BUFFER | 0X02, //Receive Buffer 0, start at RXB0D0 (Receive Buffer 0 Data Byte 0 Register: 0x66 Address)
	START_RXB1SIDH	= SPI_COMMAND_READ_RX_BUFFER | 0X04, //Receive Buffer 1, start at RXB1SIDH (Receive Buffer 1 Standard Identifier High Register: 0x71 Address)
	START_RXB1D0	= SPI_COMMAND_READ_RX_BUFFER | 0x06	 //Receive Buffer 1, start at RXB1D0 (Receive Buffer 1 Data Byte 0 Register: 0x76 Address)
}Read_RxBufferLoc_t;

//@Load_TxBufferLoc_t
typedef enum{
	START_TXB0SIDH	= SPI_COMMAND_LOAD_TX_BUFFER | 0X00, //TX Buffer 0, Start at TXB0SIDH (Transmit Buffer 0 Standard Identifier High Register: 0x31 Address)
	START_TXB0D0	= SPI_COMMAND_LOAD_TX_BUFFER | 0X01, //TX Buffer 0, Start at TXB0D0 (Transmit Buffer 0 Data Byte 0 Register: 0x36 Address)
	START_TXB1SIDH	= SPI_COMMAND_LOAD_TX_BUFFER | 0X02, //TX Buffer 1, Start at TXB1SIDH (Transmit Buffer 1 Standard Identifier High Register: 0x41 Address)
	START_TXB1D0	= SPI_COMMAND_LOAD_TX_BUFFER | 0X03, //TX Buffer 1, Start at TXB1D0 (Transmit Buffer 1 Data Byte 0 Register: 0x46 Address)
	START_TXB2SIDH	= SPI_COMMAND_LOAD_TX_BUFFER | 0X04, //TX Buffer 2, Start at TXB2SIDH (Transmit Buffer 2 Standard Identifier High Register: 0x51 Address)
	START_TXB2D0	= SPI_COMMAND_LOAD_TX_BUFFER | 0X05  //TX Buffer 2, Start at TXB2D0 (Transmit Buffer 2 Data Byte 0 Register: 0x56 Address)
}Load_TxBufferLoc_t;

//@RTS_TxBuffer_t
typedef enum{
	RTS_TXB0		= SPI_COMMAND_RTS | 0X01, //Request to Send for TX Buffer 0
	RTS_TXB1		= SPI_COMMAND_RTS | 0X02, //Request to Send for TX Buffer 1
	RTS_TXB2		= SPI_COMMAND_RTS | 0X04, //Request to Send for TX Buffer 2
	RTS_TXB0_TXB1	= SPI_COMMAND_RTS | 0X03, //Request to Send for TX Buffers 0 and 1
	RTS_TXB0_TXB2	= SPI_COMMAND_RTS | 0X05, //Request to Send for TX Buffers 0 and 2
	RTS_TXB1_TXB2	= SPI_COMMAND_RTS | 0X06, //Request to Send for TX Buffers 1 and 2
	RTS_ALL			= SPI_COMMAND_RTS | 0X07  //Request to Send for All TX Buffers
}RTS_TxBuffer_t;


//Structure for storing RxBuffer0 filter values. Does not support Extended CAN frame identifiers.
typedef struct{
	uint16_t  	RxBuffer0_Filt0;		//RxBuffer0 Standard 11-bit Identifier Filter 0
	uint16_t  	RxBuffer0_Filt1;		//RxBuffer0 Standard 11-bit Identifier Filter 1
	uint16_t  	RxBuffer0_AcceptMask;	//RxBuffer0 Standard 11-bit Identifier Acceptance Mask
}RxBuffer0_Filter_Val_t;


//Structure for storing RxBuffer1 filter values. Does not support Extended CAN frame identifiers.
typedef struct{
	uint16_t  	RxBuffer1_Filt2;		//RxBuffer1 Standard 11-bit Identifier Filter 2
	uint16_t  	RxBuffer1_Filt3;		//RxBuffer1 Standard 11-bit Identifier Filter 3
	uint16_t  	RxBuffer1_Filt4;		//RxBuffer1 Standard 11-bit Identifier Filter 4
	uint16_t  	RxBuffer1_Filt5;		//RxBuffer1 Standard 11-bit Identifier Filter 5
	uint16_t  	RxBuffer0_AcceptMask;	//RxBuffer1 Standard 11-bit Identifier Acceptance Mask
}RxBuffer1_Filter_Val_t;


//contains user defined configurations for MCP2515
typedef struct{

	//Filter Configuration Parameters
	uint8_t RxBUFFERn_FILT_CFG;		//Specifies which RxBuffer filters to configure. Ref @RxBuffer_Filt_CFG
	RxBuffer0_Filter_Val_t;			//Input 11-bit Standard Identifier filters for RxBuffer0
	RxBuffer1_Filter_Val_t;			//Input 11-bit Standard Identifier filters for RxBuffer1

	//CAN Bus configuration parameters
	uint8_t CAN_Speed_Kbps;			//Input desired CAN bus speed. Ref @CAN_Speed_CFG
	uint8_t MCP2515_Module_Osc; 	//Input oscillator frequency value installed on your MCP2515 module/hardware

	//Interrupt Configuration Parameters


}MCP2515_CFG_Handle_t;

//@RxBuffer_Filt_CFG
#define RxBUFFER0_FILT_CFG	0X01 	//Configure RxBuffer0 Filters when this is set
#define RxBUFFER1_FILT_CFG	0X02	//Configure RxBuffer1 Filters when this is set

//@CAN_Speed_CFG
#define CAN_SPEED_500Kbps	0x05	//Flag for configuring CAN speed to 500Kbps
#define CAN_SPEED_250Kbps	0x02	//Flag for configuring CAN speed to 250Kbps




/**********************MCP2515 REGISTER ADDRESSES*******************/

//@register_address
/*****Acceptance Filter Registers*****/
//Receive Buffer0 Filters
#define RXF0SIDH_ADDR     	0x00 //Filter 0 Standard Identifier High Register
#define RXF0SIDL_ADDR     	0x01 //Filter 0 Standard Identifier Low Register
#define RXF0EID8_ADDR    	0x02 //Filter 0 Extended Identifier High Register
#define RXF0EID0_ADDR     	0x03 //Filter 0 Extended Identifier Low Register
#define RXF1SIDH_ADDR    	0x04 //Filter 1 Standard Identifier High Register
#define RXF1SIDL_ADDR    	0x05 //Filter 1 Standard Identifier Low Register
#define RXF1EID8_ADDR    	0x06 //Filter 1 Extended Identifier High Register
#define RXF1EID0_ADDR    	0x07 //Filter 1 Extended Identifier Low Register

//Receive Buffer1  Filters
#define RXF2SIDH_ADDR     	0x08 //Filter 2 Standard Identifier High Register
#define RXF2SIDL_ADDR    	0x09 //Filter 2 Standard Identifier Low Register
#define RXF2EID8_ADDR     	0x0A //Filter 2 Extended Identifier High Register
#define RXF2EID0_ADDR     	0x0B //Filter 2 Extended Identifier Low Register
#define RXF3SIDH_ADDR     	0x10 //Filter 3 Standard Identifier High Register
#define RXF3SIDL_ADDR     	0x11 //Filter 3 Standard Identifier Low Register
#define RXF3EID8_ADDR     	0x12 //Filter 3 Extended Identifier High Register
#define RXF3EID0_ADDR     	0x13 //Filter 3 Extended Identifier Low Register
#define RXF4SIDH_ADDR     	0x14 //Filter 4 Standard Identifier High Register
#define RXF4SIDL_ADDR     	0x15 //Filter 4 Standard Identifier Low Register
#define RXF4EID8_ADDR     	0x16 //Filter 4 Extended Identifier High Register
#define RXF4EID0_ADDR     	0x17 //Filter 4 Extended Identifier Low Register
#define RXF5SIDH_ADDR     	0x18 //Filter 5 Standard Identifier High Register
#define RXF5SIDL_ADDR     	0x19 //Filter 5 Standard Identifier Low Register
#define RXF5EID8_ADDR     	0x1A //Filter 5 Extended Identifier High Register
#define RXF5EID0_ADDR     	0x1B //Filter 5 Extended Identifier Low Register

/*****Acceptance Filter Mask Registers*****/
#define RXM0SIDH_ADDR     	0x20 //Receive Buffer 0 Mask Standard Identifier High Register
#define RXM0SIDL_ADDR     	0x21 //Receive Buffer 0 Mask Standard Identifier Low Register
#define RXM0EID8_ADDR     	0x22 //Receive Buffer 0 Mask Extended Identifier High Register
#define RXM0EID0_ADDR     	0x23 //Receive Buffer 0 Mask Extended Identifier Low Register
#define RXM1SIDH_ADDR     	0x24 //Receive Buffer 1 Mask Standard Identifier High Register
#define RXM1SIDL_ADDR     	0x25 //Receive Buffer 1 Mask Standard Identifier Low Register
#define RXM1EID8_ADDR     	0x26 //Receive Buffer 1 Mask Extended Identifier High Register
#define RXM1EID0_ADDR     	0x27 //Receive Buffer 1 Mask Extended Identifier Low Register

/******************Transmit Buffer Registers******************/
//Transmit Buffer 0
#define TXB0SIDH_ADDR     	0x31 //Transmit Buffer 0 Standard Identifier High Register
#define TXB0SIDL_ADDR     	0x32 //Transmit Buffer 0 Standard Identifier Low Register
#define TXB0EID8_ADDR     	0x33 //Transmit Buffer 0 Extended Identifier High Register
#define TXB0EID0_ADDR     	0x34 //Transmit Buffer 0 Extended Identifier Low Register
#define TXB0DLC_ADDR      	0x35 //Transmit Buffer 0 Data Length Code Register
#define TXB0D0_ADDR       	0x36 //Transmit Buffer 0 Data Byte 0 Register
#define TXB0D1_ADDR       	0x37 //Transmit Buffer 0 Data Byte 1 Register
#define TXB0D2_ADDR       	0x38 //Transmit Buffer 0 Data Byte 2 Register
#define TXB0D3_ADDR       	0x39 //Transmit Buffer 0 Data Byte 3 Register
#define TXB0D4_ADDR       	0x3A //Transmit Buffer 0 Data Byte 4 Register
#define TXB0D5_ADDR       	0x3B //Transmit Buffer 0 Data Byte 5 Register
#define TXB0D6_ADDR       	0x3C //Transmit Buffer 0 Data Byte 6 Register
#define TXB0D7_ADDR       	0x3D //Transmit Buffer 0 Data Byte 7 Register

//Transmit Buffer 1
#define TXB1SIDH_ADDR     	0x41 //Transmit Buffer 1 Standard Identifier High Register
#define TXB1SIDL_ADDR     	0x42 //Transmit Buffer 1 Standard Identifier Low Register
#define TXB1EID8_ADDR     	0x43 //Transmit Buffer 1 Extended Identifier High Register
#define TXB1EID0_ADDR     	0x44 //Transmit Buffer 1 Extended Identifier Low Register
#define TXB1DLC_ADDR      	0x45 //Transmit Buffer 1 Data Length Code Register
#define TXB1D0_ADDR       	0x46 //Transmit Buffer 1 Data Byte 0 Register
#define TXB1D1_ADDR       	0x47 //Transmit Buffer 1 Data Byte 1 Register
#define TXB1D2_ADDR       	0x48 //Transmit Buffer 1 Data Byte 2 Register
#define TXB1D3_ADDR       	0x49 //Transmit Buffer 1 Data Byte 3 Register
#define TXB1D4_ADDR       	0x4A //Transmit Buffer 1 Data Byte 4 Register
#define TXB1D5_ADDR       	0x4B //Transmit Buffer 1 Data Byte 5 Register
#define TXB1D6_ADDR       	0x4C //Transmit Buffer 1 Data Byte 6 Register
#define TXB1D7_ADDR       	0x4D //Transmit Buffer 1 Data Byte 7 Register

//Transmit Buffer 2
#define TXB2SIDH_ADDR     	0x51 //Transmit Buffer 2 Standard Identifier High Register
#define TXB2SIDL_ADDR     	0x52 //Transmit Buffer 2 Standard Identifier Low Register
#define TXB2EID8_ADDR     	0x53 //Transmit Buffer 2 Extended Identifier High Register
#define TXB2EID0_ADDR     	0x54 //Transmit Buffer 2 Extended Identifier Low Register
#define TXB2DLC_ADDR      	0x55 //Transmit Buffer 2 Data Length Code Register
#define TXB2D0_ADDR       	0x56 //Transmit Buffer 2 Data Byte 0 Register
#define TXB2D1_ADDR       	0x57 //Transmit Buffer 2 Data Byte 1 Register
#define TXB2D2_ADDR       	0x58 //Transmit Buffer 2 Data Byte 2 Register
#define TXB2D3_ADDR       	0x59 //Transmit Buffer 2 Data Byte 3 Register
#define TXB2D4_ADDR       	0x5A //Transmit Buffer 2 Data Byte 4 Register
#define TXB2D5_ADDR       	0x5B //Transmit Buffer 2 Data Byte 5 Register
#define TXB2D6_ADDR       	0x5C //Transmit Buffer 2 Data Byte 6 Register
#define TXB2D7_ADDR       	0x5D //Transmit Buffer 2 Data Byte 7 Register

/******************Receive Buffer Registers******************/
//Receive Buffer 0
#define RXB0SIDH_ADDR     	0x61 //Receive Buffer 0 Standard Identifier High Register
#define RXB0SIDL_ADDR     	0x62 //Receive Buffer 0 Standard Identifier Low Register
#define RXB0EID8_ADDR     	0x63 //Receive Buffer 0 Extended Identifier High Register
#define RXB0EID0_ADDR     	0x64 //Receive Buffer 0 Extended Identifier Low Register
#define RXB0DLC_ADDR      	0x65 //Receive Buffer 0 Data Length Code Register
#define RXB0D0_ADDR       	0x66 //Receive Buffer 0 Data Byte 0 Register
#define RXB0D1_ADDR       	0x67 //Receive Buffer 0 Data Byte 1 Register
#define RXB0D2_ADDR       	0x68 //Receive Buffer 0 Data Byte 2 Register
#define RXB0D3_ADDR       	0x69 //Receive Buffer 0 Data Byte 3 Register
#define RXB0D4_ADDR       	0x6A //Receive Buffer 0 Data Byte 4 Register
#define RXB0D5_ADDR       	0x6B //Receive Buffer 0 Data Byte 5 Register
#define RXB0D6_ADDR       	0x6C //Receive Buffer 0 Data Byte 6 Register
#define RXB0D7_ADDR       	0x6D //Receive Buffer 0 Data Byte 7 Register

//Receive Buffer 1
#define RXB1SIDH_ADDR     	0x71 //Receive Buffer 1 Standard Identifier High Register
#define RXB1SIDL_ADDR     	0x72 //Receive Buffer 1 Standard Identifier Low Register
#define RXB1EID8_ADDR     	0x73 //Receive Buffer 1 Extended Identifier High Register
#define RXB1EID0_ADDR     	0x74 //Receive Buffer 1 Extended Identifier Low Register
#define RXB1DLC_ADDR      	0x75 //Receive Buffer 1 Data Length Code Register
#define RXB1D0_ADDR       	0x76 //Receive Buffer 1 Data Byte 0 Register
#define RXB1D1_ADDR       	0x77 //Receive Buffer 1 Data Byte 1 Register
#define RXB1D2_ADDR       	0x78 //Receive Buffer 1 Data Byte 2 Register
#define RXB1D3_ADDR       	0x79 //Receive Buffer 1 Data Byte 3 Register
#define RXB1D4_ADDR       	0x7A //Receive Buffer 1 Data Byte 4 Register
#define RXB1D5_ADDR       	0x7B //Receive Buffer 1 Data Byte 5 Register
#define RXB1D6_ADDR       	0x7C //Receive Buffer 1 Data Byte 6 Register
#define RXB1D7_ADDR       	0x7D //Receive Buffer 1 Data Byte 7 Register


/******************I/O Control and Status Registers******************/
#define BFPCTRL_ADDR_BTM	0x0C //RXnBF Pin Control and Status Register. BIT MODIFY (BTM) capable.
#define TXRTSCTRL_ADDR_BTM	0x0D //TXnRTS Pin Control and Status Register. BIT MODIFY (BTM) capable.

/****************CAN Control and Status Registers****************/
#define CANSTAT_ADDR      	0x0E //CAN Status Register. READ ONLY.
#define CANCTRL_ADDR_BTM  	0x0F //CAN Control Register. BIT MODIFY (BTM) capable.

#define CNF3_ADDR_BTM     	0x28 //Bit Timing Configuration Register 3. BIT MODIFY (BTM) capable.
#define CNF2_ADDR_BTM     	0x29 //Bit Timing Configuration Register 2. BIT MODIFY (BTM) capable.
#define CNF1_ADDR_BTM     	0x2A //Bit Timing Configuration Register 1. BIT MODIFY (BTM) capable.

#define CANINTE_ADDR_BTM  	0x2B //CAN Interrupt Enable Register. BIT MODIFY (BTM) capable.
#define CANINTF_ADDR_BTM  	0x2C //CAN Interrupt Flag Register. BIT MODIFY (BTM) capable.

/******************Transmit Buffer CTRL Registers******************/
#define TXB0CTRL_ADDR_BTM  	0x30 //Transmit Buffer 0 Control Register. BIT MODIFY (BTM) capable.
#define TXB1CTRL_ADDR_BTM  	0x40 //Transmit Buffer 1 Control Register. BIT MODIFY (BTM) capable.
#define TXB2CTRL_ADDR_BTM  	0x50 //Transmit Buffer 2 Control Register. BIT MODIFY (BTM) capable.

/******************Receive Buffer CTRL Registers******************/
#define RXB0CTRL_ADDR_BTM  	0x60 //Receive Buffer 0 Control Register. BIT MODIFY (BTM) capable.
#define RXB1CTRL_ADDR_BTM  	0x70 //Receive Buffer 1 Control Register. BIT MODIFY (BTM) capable.

/**********************Error and Status Registers*******************/
#define EFLG_ADDR_BTM     	0x2D //Error Flag Register. BIT MODIFY (BTM) capable.
#define TEC_ADDR 	     	0x1C //Transmit Error Counter Register. READ ONLY.
#define REC_ADDR 	     	0x1D //Receive Error Counter Register. READ ONLY.

/**********************END MCP2515 REGISTER ADDRESSES*******************/

//@BitModify_Capable_Register
typedef enum{
	BFPCTRL_BTM 	= BFPCTRL_ADDR_BTM,
	TXRTSCTRL_BTM 	= TXRTSCTRL_ADDR_BTM,
	CANCTRL_BTM 	= CANCTRL_ADDR_BTM,
	CNF3_BTM 		= CNF3_ADDR_BTM,
	CNF2_BTM 		= CNF2_ADDR_BTM,
	CNF1_BTM 		= CNF1_ADDR_BTM,
	CANINTE_BTM 	= CANINTE_ADDR_BTM,
	CANINTF_BTM 	= CANINTF_ADDR_BTM,
	EFLG_BTM 		= EFLG_ADDR_BTM,
	TXB0CTRL_BTM 	= TXB0CTRL_ADDR_BTM,
	TXB1CTRL_BTM 	= TXB1CTRL_ADDR_BTM,
	TXB2CTRL_BTM 	= TXB2CTRL_ADDR_BTM,
	RXB0CTRL_BTM 	= RXB0CTRL_ADDR_BTM,
	RXB1CTRL_BTM 	= RXB1CTRL_ADDR_BTM
}BTM_Registers_t;


/*********************Register Bit Masks**************************
 * @BIT_MASK
 * Naming Convention: #define RegisterName_Bitfield_BITPOSITION		Mask
 * Note: If bit field occupies multiple bits, classify in the bit fields in BITPOSITION section from MSB to LSB
 *****************************************************************/
//BFPCTRL: RXnBF Pin Control and Status Register
#define BFPCTRL_B0BFM			0x1 << 0 	//RX0BF Pin Mode
#define BFPCTRL_B1BFM			0x1 << 1 	//RX1BF Pin Mode
#define BFPCTRL_B0BFE			0x1 << 2 	//RX0BF Pin Enable
#define BFPCTRL_B1BFE			0x1 << 3 	//RX1BF Pin Enable
#define BFPCTRL_B0BF			0x1 << 4 	//RX0BF Pin State
#define BFPCTRL_B1BF			0x1 << 5 	//RX1BF Pin State


//TXRTSCTRL: TXnRTS Pin Control and Status Register
#define TXRTSCTRL_B0RTSM		1 << 0 //TX0RTS Pin Mode
#define TXRTSCTRL_B1RTSM		1 << 1 //TX1RTS Pin Mode
#define TXRTSCTRL_B2RTSM		1 << 2 //TX2RTS Pin Mode
#define TXRTSCTRL_B0RTS			1 << 3 //TX0RTS Pin State
#define TXRTSCTRL_B1RTS			1 << 4 //TX1RTS Pin State
#define TXRTSCTRL_B2RTS			1 << 5 //TX2RTS Pin State

//CANCTRL: CAN Control Register
#define CANCTRL_CLKPRE_10		0x3 << 0 //Clock Prescaler Bit 0
#define CANCTRL_CLKEN			0x1 << 1 //CLKOUT Pin Enable bit
#define CANCTRL_OSM				0x1 << 2 //One-Shot Mode Enable bit
#define CANCTRL_ABAT			0x1 << 4 //Abort All Pending Transmissions bit
#define CANCTRL_REQOP_75		0x7 << 5 //Requested Operating Mode bits

//CNF3: CONFIGURATION REGISTER 3
#define CNF3_PHSEG2_20			0x7 << 0 // PS2 Length bits
#define	CNF3_WAKFIL				0x1 << 6 // Wake-Up Filter Enable bit
#define CNF3_SOF				0x1 << 7 // Start-of-Frame Signal Enable bit

//CNF2: CONFIGURATION REGISTER 2
#define CNF2_PRSEG_20			0x7 << 0 // Propagation Segment Length bits
#define CNF2_PHSEG_53           0x7 << 3 // PS1 Length bits
#define CNF2_SAM				0x1 << 6 // Sample Point Configuration bit
#define CNF2_BTLMODE			0x1 << 7 // PS2 Time Select bit

//CNF1: CONFIGURATION REGISTER 1
#define CNF1_BRP_50				0x3F << 0	// Baud Rate Prescaler bits
#define CNF1_SJW_20				0x3 << 6 	// Synchronization Jump Width bits

//CANINTE: CAN INTERRUPT ENABLE REGISTER
#define CANINTE_RX0IE			0x1 << 0 //Receive Buffer 0 Full Interrupt Enable bit
#define CANINTE_RX1IE			0x1 << 1 //Receive Buffer 1 Full Interrupt Enable bit
#define CANINTE_TX0IE			0x1 << 2 //Transmit Buffer 0 Empty Interrupt Enable bit
#define CANINTE_TX1IE			0x1 << 3 //Transmit Buffer 1 Empty Interrupt Enable bit
#define CANINTE_TX2IE			0x1 << 4 //Transmit Buffer 2 Empty Interrupt Enable bit
#define CANINTE_ERRIE			0x1 << 5 //Error Interrupt Enable bit
#define CANINTE_WAKIE			0x1 << 6 //Wake-Up Interrupt Enable bit
#define CANINTE_MERRE			0x1 << 7 //Message Error Interrupt Enable bit

//CANINTF: CAN INTERRUPT FLAG REGISTER
#define CANINTF_RX0IF			0x1 << 0 //Receive Buffer 0 Full Interrupt Flag bit
#define CANINTF_RX1IF			0x1 << 1 //Receive Buffer 1 Full Interrupt Flag bit
#define CANINTF_TX0IF			0x1 << 2 //Transmit Buffer 0 Empty Interrupt Flag bit
#define CANINTF_TX1IF			0x1 << 3 //Transmit Buffer 1 Empty Interrupt Flag bit
#define CANINTF_TX2IF			0x1 << 4 //Transmit Buffer 2 Empty Interrupt Flag bit
#define CANINTF_ERRIF			0x1 << 5 //Error Interrupt Flag bit
#define CANINTF_WAKIF			0x1 << 6 //Wake-Up Interrupt Flag bit
#define CANINTF_MERRF			0x1 << 7 //Message Error Interrupt Flag bit

//EFLG: ERROR FLAG REGISTER
#define EFLG_EWARN				0x1 << 0 //Error Warning Flag bit
#define EFLG_RXWAR				0x1 << 1 //Receive Error Warning Flag bit
#define EFLG_TXWAR				0x1 << 2 //Transmit Error Warning Flag bit
#define EFLG_RXEP				0x1 << 3 //Receive Error Passive Flag bit
#define EFLG_TXEP				0x1 << 4 //Transmit Error Passive Flag bit
#define EFLG_TXBO				0x1 << 5 //Bus-Off Error Flag bit
#define EFLG_RX0OVR				0x1 << 6 //Receive Buffer 0 Overflow Flag bit
#define EFLG_RX1OVR				0x1 << 7 //Receive Buffer 1 Overflow Flag bit

//TXBnCTRL: TRANSMIT BUFFER n CONTROL REGISTER
#define TXBCTRL_TXP_10			0x3 << 0 //Transmit Buffer Priority bits
#define TXBCTRL_TXREQ			0x1 << 3 //Transmit Request bit
#define TXBCTRL_TXERR			0x1 << 4 //Transmission Error Detected bit
#define TXBCTRL_MLOA			0x1 << 5 //Message Lost Arbitration bit
#define TXBCTRL_ABTF			0x1 << 6 //Message Aborted Flag bit

//RXB0CTRL: RECEIVE BUFFER 0 CONTROL REGISTER
#define RXB0CTRL_FILHIT0		0x1 << 0 //Filter Hit bit (indicates which acceptance filter enabled reception of message)
#define RXB0CTRL_BUKT1       	0x1 << 1 //Read-Only Copy of BUKT bit (used internally by the MCP2515
#define RXB0CTRL_BUKT       	0x1 << 2 //Rollover Enable bit
#define RXB0CTRL_RXRTR			0x1 << 3 //Received Remote Transfer Request bit
#define RXB0CTRL_RXM_65			0x3 << 5 //Receive Buffer Operating Mode bits

//RXB1CTRL: RECEIVE BUFFER 1 CONTROL REGISTER
#define RXB1CTRL_FILHIT_20		0x7 << 0 //Filter Hit bit (indicates which acceptance filter enabled reception of message)
#define RXB1CTRL_RXRTR			0x1 << 3 //Received Remote Transfer Request bit
#define RXB1CTRL_RXM_65			0x3 << 5 //Receive Buffer Operating Mode bits

/*********************END Register Bit Masks**************************/


/***************************************************************
 * APIs Function Prototypes
 **************************************************************/

/**********************MCP2515 SPI COMMANDS************************/
void MCP2515_SPI_WriteRegister(uint8_t start_address, uint8_t *DataBuffer, uint8_t data_length);
void MCP2515_SPI_LoadTxBuffer(Load_TxBufferLoc_t TxBuffer_Loc,  uint8_t *DataBuffer, uint8_t data_length);
void MCP2515_SPI_BitModify(BTM_Registers_t RegisterAddress, uint8_t maskByte, uint8_t dataByte);

void MCP2515_SPI_Reset(void);
void MCP2515_SPI_RxStatus(uint8_t *RxBuffer, uint8_t rxbuffer_size);
void MCP2515_SPI_ReadStatus(uint8_t *RxBuffer, uint8_t rxbuffer_size);

void MCP2515_SPI_ReadRegister(uint8_t start_address, uint8_t *RxBuffer, uint8_t rxbuffer_size);
void MCP2515_SPI_ReadRxBuffer(Read_RxBufferLoc_t RxBuffer_Loc, uint8_t *RxBuffer, uint8_t rxbuffer_size);

void MCP2515_SPI_RequestToSend(RTS_TxBuffer_t TxBuffer);

void MCP2515_Init(MCP2515_Handle_t *MCP2515_handle);
#endif /* MCP2515_DRIVER_MCP2515_DRIVER_H_ */
