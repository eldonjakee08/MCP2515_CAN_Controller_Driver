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

//dummy byte for clocking out slave data during reception
#define dummy_byte 		0x00

#define MCP2515_INTERNAL_CLK_FREQ	8000000 //in Hz. MCP2515 module installed oscillator, modify based on your installed oscillator on module

extern SPI_HandleTypeDef hspi1; 	//extern SPI handle

typedef enum{
	TXBUFFER_BUSY 			= 0x40,
	TX_SUCCESS	 			= 0x41,
	ERR_ARBITRATION_LOSS	= 0x42,
	ERR_TRANSMISSION		= 0x43,
	ERR_INVALID_INPUT		= 0x44
}CAN_Tx_Status_t;

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
	RTS_TxBUFFER0				= SPI_COMMAND_RTS | 0X01, //Request to Send for TX Buffer 0
	RTS_TxBUFFER1				= SPI_COMMAND_RTS | 0X02, //Request to Send for TX Buffer 1
	RTS_TxBUFFER2				= SPI_COMMAND_RTS | 0X04, //Request to Send for TX Buffer 2
	RTS_TxBUFFER0_TxBUFFER1		= SPI_COMMAND_RTS | 0X03, //Request to Send for TX Buffers 0 and 1
	RTS_TxBUFFER0_TxBUFFER2		= SPI_COMMAND_RTS | 0X05, //Request to Send for TX Buffers 0 and 2
	RTS_TxBUFFER1_TxBUFFER2		= SPI_COMMAND_RTS | 0X06, //Request to Send for TX Buffers 1 and 2
	RTS_ALL						= SPI_COMMAND_RTS | 0X07  //Request to Send for All TX Buffers
}RTS_TxBuffer_t;


//Structure for storing RxBuffer0 filter values. Does not support Extended CAN frame identifiers.
typedef struct{
	uint16_t  	RxBuffer0_Filt[2];		//RxBuffer0 Standard 11-bit Identifier Filter 0 to Filter 1
	uint16_t  	RxBuffer0_AcceptMask;	//RxBuffer0 Standard 11-bit Identifier Acceptance Mask
}RxBuffer0_Filter_Val_t;


//Structure for storing RxBuffer1 filter values. Does not support Extended CAN frame identifiers.
typedef struct{
	uint16_t  	RxBuffer1_Filt[4];		//RxBuffer1 Standard 11-bit Identifier Filter 2 to Filter 5
	uint16_t  	RxBuffer1_AcceptMask;	//RxBuffer1 Standard 11-bit Identifier Acceptance Mask
}RxBuffer1_Filter_Val_t;


//contains user defined configurations for MCP2515
typedef struct{
	//Filter Configuration Parameters
	uint8_t RxBUFFERn_FILT_CFG;						//Specifies which RxBuffer filters to configure. Ref @RxBuffer_Filt_CFG
	RxBuffer0_Filter_Val_t RxBuffer0_FILT;			//Input 11-bit Standard Identifier filters for RxBuffer0
	RxBuffer1_Filter_Val_t RxBuffer1_FILT;			//Input 11-bit Standard Identifier filters for RxBuffer1

	//CAN Bus timing configuration parameters
	uint8_t CAN_Baud_PreScaler;		//CNF1: BRP[5:0] Baud Rate Prescaler bits. Values 0 to 63
	uint8_t ProgDelay_TQ_Length;	//Propagation Delay length in Time Quanta. Ref @Propagation_Delay_Segment_Length
	uint8_t PH1_Seg_TQ_Length;		//Phase 1 Segment length in Time Quanta. Ref @PHASEn_Segment_Length
	uint8_t PH2_Seg_TQ_Length;		//Phase 2 Segment length in Time Quanta. Ref @PHASEn_Segment_Length
	uint8_t SJW_TQ_Length;			//Synchronization Jump Width in Time Quanta. Ref @Synchronization_Jump_Width_Length
	uint32_t CAN_Speed_Kbps;		//Input desired CAN bus speed in Hz. Ref @CAN_Speed_CFG
	uint32_t MCP2515_Osc; 			//Input oscillator frequency in Hz installed on your MCP2515 module/hardware

	//Interrupt Configuration Parameters
	uint8_t INT_Enable_Mask;		//Specifies which interrupt events are to be enabled. Ref @Interrupt_Enable_Bits
}MCP2515_CFG_Handle_t;


/**********************MCP2515 REGISTER ADDRESSES*******************/
//@register_address
/*****Acceptance Filter Registers*****/
//Receive Buffer0 Filters
//TODO: input shorthand notation of register name in comments
#define RxBUFFER0_FILTER0_SIDH_REG_ADDR     0x00 //Filter 0 Standard Identifier High Register
#define RxBUFFER0_FILTER0_SIDL_REG_ADDR     0x01 //Filter 0 Standard Identifier Low Register
#define RxBUFFER0_FILTER0_EID8_REG_ADDR  	0x02 //Filter 0 Extended Identifier High Register
#define RxBUFFER0_FILTER0_EID0_REG_ADDR     0x03 //Filter 0 Extended Identifier Low Register
#define RxBUFFER0_FILTER1_SIDH_REG_ADDR    	0x04 //Filter 1 Standard Identifier High Register
#define RxBUFFER0_FILTER1_SIDL_REG_ADDR    	0x05 //Filter 1 Standard Identifier Low Register
#define RxBUFFER0_FILTER1_EID8_REG_ADDR    	0x06 //Filter 1 Extended Identifier High Register
#define RxBUFFER0_FILTER1_EID0_REG_ADDR    	0x07 //Filter 1 Extended Identifier Low Register

//Receive Buffer1  Filters
//TODO: input shorthand notation of register name in comments
#define RxBUFFER1_FILTER2_SIDH_REG_ADDR     0x08 //Filter 2 Standard Identifier High Register
#define RxBUFFER1_FILTER2_SIDL_REG_ADDR    	0x09 //Filter 2 Standard Identifier Low Register
#define RxBUFFER1_FILTER2_EID8_REG_ADDR     0x0A //Filter 2 Extended Identifier High Register
#define RxBUFFER1_FILTER2_EID0_REG_ADDR     0x0B //Filter 2 Extended Identifier Low Register
#define RxBUFFER1_FILTER3_SIDH_REG_ADDR     0x10 //Filter 3 Standard Identifier High Register
#define RxBUFFER1_FILTER3_SIDL_REG_ADDR     0x11 //Filter 3 Standard Identifier Low Register
#define RxBUFFER1_FILTER3_EID8_REG_ADDR     0x12 //Filter 3 Extended Identifier High Register
#define RxBUFFER1_FILTER3_EID0_REG_ADDR     0x13 //Filter 3 Extended Identifier Low Register
#define RxBUFFER1_FILTER4_SIDH_REG_ADDR     0x14 //Filter 4 Standard Identifier High Register
#define RxBUFFER1_FILTER4_SIDL_REG_ADDR     0x15 //Filter 4 Standard Identifier Low Register
#define RxBUFFER1_FILTER4_EID8_REG_ADDR     0x16 //Filter 4 Extended Identifier High Register
#define RxBUFFER1_FILTER4_EID0_REG_ADDR     0x17 //Filter 4 Extended Identifier Low Register
#define RxBUFFER1_FILTER5_SIDH_REG_ADDR     0x18 //Filter 5 Standard Identifier High Register
#define RxBUFFER1_FILTER5_SIDL_REG_ADDR     0x19 //Filter 5 Standard Identifier Low Register
#define RxBUFFER1_FILTER5_EID8_REG_ADDR     0x1A //Filter 5 Extended Identifier High Register
#define RxBUFFER1_FILTER5_EID0_REG_ADDR     0x1B //Filter 5 Extended Identifier Low Register

/*****Acceptance Filter Mask Registers*****/
//TODO: input shorthand notation of register name in comments
#define RxBUFFER0_MASK_SIDH_REG_ADDR     	0x20 //Receive Buffer 0 Mask Standard Identifier High Register
#define RxBUFFER0_MASK_SIDL_REG_ADDR     	0x21 //Receive Buffer 0 Mask Standard Identifier Low Register
#define RxBUFFER0_MASK_EID8_REG_ADDR     	0x22 //Receive Buffer 0 Mask Extended Identifier High Register
#define RxBUFFER0_MASK_EID0_REG_ADDR     	0x23 //Receive Buffer 0 Mask Extended Identifier Low Register
#define RxBUFFER1_MASK_SIDH_REG_ADDR     	0x24 //Receive Buffer 1 Mask Standard Identifier High Register
#define RxBUFFER1_MASK_SIDL_REG_ADDR     	0x25 //Receive Buffer 1 Mask Standard Identifier Low Register
#define RxBUFFER1_MASK_EID8_REG_ADDR     	0x26 //Receive Buffer 1 Mask Extended Identifier High Register
#define RxBUFFER1_MASK_EID0_REG_ADDR     	0x27 //Receive Buffer 1 Mask Extended Identifier Low Register

/******************Transmit Buffer Registers******************/
//Transmit Buffer 0
//TODO Change name to less cryptic like previous register names
#define TXB0SIDH_ADDR     	0x31 //TXB0SIDH: Transmit Buffer 0 Standard Identifier High Register
#define TXB0SIDL_ADDR     	0x32 //TXB0SIDL: Transmit Buffer 0 Standard Identifier Low Register
#define TXB0EID8_ADDR     	0x33 //TXB0EID8: Transmit Buffer 0 Extended Identifier High Register
#define TXB0EID0_ADDR     	0x34 //TXB0EID0: Transmit Buffer 0 Extended Identifier Low Register
#define TXB0DLC_ADDR      	0x35 //TXB0DLC: Transmit Buffer 0 Data Length Code Register
#define TXB0D0_ADDR       	0x36 //TXB0D0: Transmit Buffer 0 Data Byte 0 Register
#define TXB0D1_ADDR       	0x37 //TXB0D1: Transmit Buffer 0 Data Byte 1 Register
#define TXB0D2_ADDR       	0x38 //TXB0D2: Transmit Buffer 0 Data Byte 2 Register
#define TXB0D3_ADDR       	0x39 //TXB0D3: Transmit Buffer 0 Data Byte 3 Register
#define TXB0D4_ADDR       	0x3A //TXB0D4: Transmit Buffer 0 Data Byte 4 Register
#define TXB0D5_ADDR       	0x3B //TXB0D5: Transmit Buffer 0 Data Byte 5 Register
#define TXB0D6_ADDR       	0x3C //TXB0D6: Transmit Buffer 0 Data Byte 6 Register
#define TXB0D7_ADDR       	0x3D //TXB0D7: Transmit Buffer 0 Data Byte 7 Register

//Transmit Buffer 1
//TODO Change name to less cryptic like previous register names
//TODO: input shorthand notation of register name in comments
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
//TODO Change name to less cryptic like previous register names
//TODO: input shorthand notation of register name in comments
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
//TODO Change name to less cryptic like previous register names
//TODO: input shorthand notation of register name in comments
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
//TODO Change name to less cryptic like previous register names
//TODO: input shorthand notation of register name in comments
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
//TODO: input shorthand notation of register name in comments

#define BFPCTRL_REG_ADDR_BTM	0x0C //RXnBF Pin Control and Status Register. BIT MODIFY (BTM) capable.
#define TXRTSCTRL_REG_ADDR_BTM	0x0D //TXnRTS Pin Control and Status Register. BIT MODIFY (BTM) capable.

/****************CAN Control and Status Registers****************/
#define CANSTATUS_REG_ADDR      	0x0E //CAN Status Register. READ ONLY.
#define CANCTRL_REG_ADDR_BTM  		0x0F //CAN Control Register. BIT MODIFY (BTM) capable.

#define CNF3_REG_ADDR_BTM     	0x28 //Bit Timing Configuration Register 3. BIT MODIFY (BTM) capable.
#define CNF2_REG_ADDR_BTM     	0x29 //Bit Timing Configuration Register 2. BIT MODIFY (BTM) capable.
#define CNF1_REG_ADDR_BTM     	0x2A //Bit Timing Configuration Register 1. BIT MODIFY (BTM) capable.

#define CAN_INTEN_REG_ADDR_BTM  	0x2B //CAN Interrupt Enable Register. BIT MODIFY (BTM) capable.
#define CAN_INTF_REG_ADDR_BTM  		0x2C //CAN Interrupt Flag Register. BIT MODIFY (BTM) capable.

/******************Transmit Buffer CTRL Registers******************/
#define TXBUFFER0_CTRL_REG_ADDR_BTM  	0x30 //Transmit Buffer 0 Control Register. BIT MODIFY (BTM) capable.
#define TXBUFFER1_CTRL_REG_ADDR_BTM  	0x40 //Transmit Buffer 1 Control Register. BIT MODIFY (BTM) capable.
#define TXBUFFER2_CTRL_REG_ADDR_BTM  	0x50 //Transmit Buffer 2 Control Register. BIT MODIFY (BTM) capable.

/******************Receive Buffer CTRL Registers******************/
#define RXBUFFER0_CTRL_REG_ADDR_BTM  	0x60 //Receive Buffer 0 Control Register. BIT MODIFY (BTM) capable.
#define RXBUFFER1_CTRL_REG_ADDR_BTM  	0x70 //Receive Buffer 1 Control Register. BIT MODIFY (BTM) capable.

/**********************Error and Status Registers*******************/
#define EFLG_REG_ADDR_BTM     	0x2D //Error Flag Register. BIT MODIFY (BTM) capable.
#define TEC_REG_ADDR 	     	0x1C //Transmit Error Counter Register. READ ONLY.
#define REC_REG_ADDR 	     	0x1D //Receive Error Counter Register. READ ONLY.

/**********************END MCP2515 REGISTER ADDRESSES*******************/


/*********************Register Bit Masks**************************
 * @BIT_MASK
 * Naming Convention: #define RegisterName_Bitfield_BITPOSITION		Mask
 * Note: If bit field occupies multiple bits, classify in the bit fields in BITPOSITION section from MSB to LSB
 *****************************************************************/
//BFPCTRL: RXnBF Pin Control and Status Register
#define BFPCTRL_B0BFM_MASK			0x1 << 0 	//RX0BF Pin Mode
#define BFPCTRL_B1BFM_MASK			0x1 << 1 	//RX1BF Pin Mode
#define BFPCTRL_B0BFE_MASK			0x1 << 2 	//RX0BF Pin Enable
#define BFPCTRL_B1BFE_MASK			0x1 << 3 	//RX1BF Pin Enable
#define BFPCTRL_B0BF_MASK			0x1 << 4 	//RX0BF Pin State
#define BFPCTRL_B1BF_MASK			0x1 << 5 	//RX1BF Pin State

//TXRTSCTRL: TXnRTS Pin Control and Status Register
#define TXRTSCTRL_B0RTSM_MASK		1 << 0 //TX0RTS Pin Mode
#define TXRTSCTRL_B1RTSM_MASK		1 << 1 //TX1RTS Pin Mode
#define TXRTSCTRL_B2RTSM_MASK		1 << 2 //TX2RTS Pin Mode
#define TXRTSCTRL_B0RTS_MASK		1 << 3 //TX0RTS Pin State
#define TXRTSCTRL_B1RTS_MASK		1 << 4 //TX1RTS Pin State
#define TXRTSCTRL_B2RTS_MASK		1 << 5 //TX2RTS Pin State

//CANCTRL: CAN Control Register
#define CANCTRL_CLKPRE_10_MASK		0x3 << 0 //Clock Prescaler Bit 0
#define CANCTRL_CLKEN_MASK			0x1 << 1 //CLKOUT Pin Enable bit
#define CANCTRL_OSM_MASK			0x1 << 2 //One-Shot Mode Enable bit
#define CANCTRL_ABAT_MASK			0x1 << 4 //Abort All Pending Transmissions bit
#define CANCTRL_REQOP_75_MASK		0x7 << 5 //Requested Operating Mode bits

//CNF3: CONFIGURATION REGISTER 3
#define CNF3_PHSEG2_20_MASK			0x7 << 0 // PS2 Length bits
#define	CNF3_WAKFIL_MASK			0x1 << 6 // Wake-Up Filter Enable bit
#define CNF3_SOF_MASK				0x1 << 7 // Start-of-Frame Signal Enable bit

//CNF2: CONFIGURATION REGISTER 2
#define CNF2_PRSEG_20_MASK			0x7 << 0 // Propagation Segment Length bits
#define CNF2_PHSEG1_53_MASK         0x7 << 3 // PS1 Length bits
#define CNF2_SAM_MASK				0x1 << 6 // Sample Point Configuration bit
#define CNF2_BTLMODE_MASK			0x1 << 7 // PS2 Time Select bit

//CNF1: CONFIGURATION REGISTER 1
#define CNF1_BRP_50_MASK			0x3F << 0	// Baud Rate Prescaler bits
#define CNF1_SJW_20_MASK			0x3 << 6 	// Synchronization Jump Width bits

//CANINTE: CAN INTERRUPT ENABLE REGISTER
#define CANINTE_RX0IE_MASK			0x1 << 0 //Receive Buffer 0 Full Interrupt Enable bit
#define CANINTE_RX1IE_MASK			0x1 << 1 //Receive Buffer 1 Full Interrupt Enable bit
#define CANINTE_TX0IE_MASK			0x1 << 2 //Transmit Buffer 0 Empty Interrupt Enable bit
#define CANINTE_TX1IE_MASK			0x1 << 3 //Transmit Buffer 1 Empty Interrupt Enable bit
#define CANINTE_TX2IE_MASK			0x1 << 4 //Transmit Buffer 2 Empty Interrupt Enable bit
#define CANINTE_ERRIE_MASK			0x1 << 5 //Error Interrupt Enable bit
#define CANINTE_WAKIE_MASK			0x1 << 6 //Wake-Up Interrupt Enable bit
#define CANINTE_MERRE_MASK			0x1 << 7 //Message Error Interrupt Enable bit

//CANINTF: CAN INTERRUPT FLAG REGISTER
#define CANINTF_RX0IF_MASK			0x1 << 0 //Receive Buffer 0 Full Interrupt Flag bit
#define CANINTF_RX1IF_MASK			0x1 << 1 //Receive Buffer 1 Full Interrupt Flag bit
#define CANINTF_TX0IF_MASK			0x1 << 2 //Transmit Buffer 0 Empty Interrupt Flag bit
#define CANINTF_TX1IF_MASK			0x1 << 3 //Transmit Buffer 1 Empty Interrupt Flag bit
#define CANINTF_TX2IF_MASK			0x1 << 4 //Transmit Buffer 2 Empty Interrupt Flag bit
#define CANINTF_ERRIF_MASK			0x1 << 5 //Error Interrupt Flag bit
#define CANINTF_WAKIF_MASK			0x1 << 6 //Wake-Up Interrupt Flag bit
#define CANINTF_MERRF_MASK			0x1 << 7 //Message Error Interrupt Flag bit

//EFLG: ERROR FLAG REGISTER
#define EFLG_EWARN_MASK				0x1 << 0 //Error Warning Flag bit
#define EFLG_RXWAR_MASK				0x1 << 1 //Receive Error Warning Flag bit
#define EFLG_TXWAR_MASK				0x1 << 2 //Transmit Error Warning Flag bit
#define EFLG_RXEP_MASK				0x1 << 3 //Receive Error Passive Flag bit
#define EFLG_TXEP_MASK				0x1 << 4 //Transmit Error Passive Flag bit
#define EFLG_TXBO_MASK				0x1 << 5 //Bus-Off Error Flag bit
#define EFLG_RX0OVR_MASK				0x1 << 6 //Receive Buffer 0 Overflow Flag bit
#define EFLG_RX1OVR_MASK				0x1 << 7 //Receive Buffer 1 Overflow Flag bit

//TXBnCTRL: TRANSMIT BUFFER n CONTROL REGISTER
#define TXBnCTRL_TXP_10_MASK		0x3 << 0 //Transmit Buffer Priority bits
#define TXBnCTRL_TXREQ_MASK			0x1 << 3 //Transmit Request bit
#define TXBnCTRL_TXERR_MASK			0x1 << 4 //Transmission Error Detected bit
#define TXBnCTRL_MLOA_MASK			0x1 << 5 //Message Lost Arbitration bit
#define TXBnCTRL_ABTF_MASK			0x1 << 6 //Message Aborted Flag bit

//RXB0CTRL: RECEIVE BUFFER 0 CONTROL REGISTER
#define RXB0CTRL_FILHIT0_MASK		0x1 << 0 //Filter Hit bit (indicates which acceptance filter enabled reception of message)
#define RXB0CTRL_BUKT1_MASK       	0x1 << 1 //Read-Only Copy of BUKT bit (used internally by the MCP2515
#define RXB0CTRL_BUKT_MASK       	0x1 << 2 //Rollover Enable bit
#define RXB0CTRL_RXRTR_MASK			0x1 << 3 //Received Remote Transfer Request bit
#define RXB0CTRL_RXM_65_MASK		0x3 << 5 //Receive Buffer Operating Mode bits

//RXB1CTRL: RECEIVE BUFFER 1 CONTROL REGISTER
#define RXB1CTRL_FILHIT_20_MASK		0x7 << 0 //Filter Hit bit (indicates which acceptance filter enabled reception of message)
#define RXB1CTRL_RXRTR_MASK			0x1 << 3 //Received Remote Transfer Request bit
#define RXB1CTRL_RXM_65_MASK		0x3 << 5 //Receive Buffer Operating Mode bits

/*********************END Register Bit Masks**************************/


/*******************Initialization/Set-up Enum & Macros***********************/
//@RxBuffer_Filt_CFG
#define RxBUFFER0_FILT_CFG_MASK		0X01 	//Configure RxBuffer0 Filters when this is set
#define RxBUFFER1_FILT_CFG_MASK		0X02	//Configure RxBuffer1 Filters when this is set

//@CAN_Speed_CFG
#define CAN_SPEED_500Kbps			500000	//CAN bus speed 500Kbps
#define CAN_SPEED_250Kbps			250000	//CAN bus speed 250Kbps

//@Interrupt_Enable_Bits
#define RxBUFFER0_FULL_IE			1 << 0	//Receive Buffer 0 Full Interrupt Enable bit
#define RxBUFFER1_FULL_IE			1 << 1	//Receive Buffer 1 Full Interrupt Enable bit
#define TxBUFFER0_EMPTY_IE			1 << 2	//Transmit Buffer 0 Empty Interrupt Enable bit
#define TxBUFFER1_EMPTY_IE			1 << 3	//Transmit Buffer 1 Empty Interrupt Enable bit
#define TxBUFFER2_EMPTY_IE			1 << 4	//Transmit Buffer 2 Empty Interrupt Enable bit
#define ERROR_IE					1 << 5	//Error Interrupt Enable bit (multiple sources in EFLG register)
#define WAKEUP_IE					1 << 6	//Wake-up Interrupt Enable bit
#define MESSAGE_ERR_IE				1 << 7	//Message Error Interrupt Enable bit

//@MCP2515_Modes
#define NORMAL_MODE					0x0 << 5
#define SLEEP_MODE					0x1 << 5
#define LOOPBACK_MODE				0x2 << 5
#define LISTEN_ONLY_MODE			0x3 << 5
#define CONFIG_MODE					0x4 << 5

//@PHASEn_Segment_Length
#define PHASEn_SEG_LEN_1TQ			0x00	//PHASEn Segment 1TQ length
#define PHASEn_SEG_LEN_2TQ			0x01	//PHASEn Segment 2TQ length
#define PHASEn_SEG_LEN_3TQ			0x02	//PHASEn Segment 3TQ length
#define PHASEn_SEG_LEN_4TQ			0x03	//PHASEn Segment 4TQ length
#define PHASEn_SEG_LEN_5TQ			0x04	//PHASEn Segment 5TQ length
#define PHASEn_SEG_LEN_6TQ			0x05	//PHASEn Segment 6TQ length
#define PHASEn_SEG_LEN_7TQ			0x06	//PHASEn Segment 7TQ length
#define PHASEn_SEG_LEN_8TQ			0x07	//PHASEn Segment 8TQ length

//@Propagation_Delay_Segment_Length
#define PROG_DELAY_SEG_LEN_1TQ		0x00	//Propagation delay segment 1TQ length
#define PROG_DELAY_SEG_LEN_2TQ		0x01	//Propagation delay segment 2TQ length
#define PROG_DELAY_SEG_LEN_3TQ		0x02	//Propagation delay segment 3TQ length
#define PROG_DELAY_SEG_LEN_4TQ		0x03	//Propagation delay segment 4TQ length
#define PROG_DELAY_SEG_LEN_5TQ		0x04	//Propagation delay segment 5TQ length
#define PROG_DELAY_SEG_LEN_6TQ		0x05	//Propagation delay segment 6TQ length
#define PROG_DELAY_SEG_LEN_7TQ		0x06	//Propagation delay segment 7TQ length
#define PROG_DELAY_SEG_LEN_8TQ		0x07	//Propagation delay segment 8TQ length

//@Synchronization_Jump_Width_Length
#define SJW_LEN_1TQ				0x00	//Synchronization Jump Width 1TQ length
#define SJW_LEN_2TQ				0x01	//Synchronization Jump Width 2TQ length
#define SJW_LEN_3TQ				0x02	//Synchronization Jump Width 3TQ length
#define SJW_LEN_4TQ				0x03	//Synchronization Jump Width 4TQ length

//@Send_TxBuffer
#define SEND_TxBUFFER0			TXB0SIDH_ADDR
#define SEND_TxBUFFER1			TXB1SIDH_ADDR
#define SEND_TxBUFFER2			TXB2SIDH_ADDR


//@BitModify_Capable_Register
typedef enum{
	BFPCTRL_BTM 	= BFPCTRL_REG_ADDR_BTM,
	TXRTSCTRL_BTM 	= TXRTSCTRL_REG_ADDR_BTM,
	CANCTRL_BTM 	= CANCTRL_REG_ADDR_BTM,
	CNF3_BTM 		= CNF3_REG_ADDR_BTM,
	CNF2_BTM 		= CNF2_REG_ADDR_BTM,
	CNF1_BTM 		= CNF1_REG_ADDR_BTM,
	CANINTE_BTM 	= CAN_INTEN_REG_ADDR_BTM,
	CANINTF_BTM 	= CAN_INTF_REG_ADDR_BTM,
	EFLG_BTM 		= EFLG_REG_ADDR_BTM,
	TXB0CTRL_BTM 	= TXBUFFER0_CTRL_REG_ADDR_BTM,
	TXB1CTRL_BTM 	= TXBUFFER1_CTRL_REG_ADDR_BTM,
	TXB2CTRL_BTM 	= TXBUFFER2_CTRL_REG_ADDR_BTM,
	RXB0CTRL_BTM 	= RXBUFFER0_CTRL_REG_ADDR_BTM,
	RXB1CTRL_BTM 	= RXBUFFER1_CTRL_REG_ADDR_BTM
}BTM_Registers_t;

/***************************************************************
 * APIs Function Prototypes
 **************************************************************/

/**********************MCP2515 SPI COMMANDS************************/
void MCP2515_SPI_WriteRegister(uint8_t start_address, uint8_t *DataBuffer, uint8_t DataLength);
void MCP2515_SPI_LoadTxBuffer(Load_TxBufferLoc_t TxBuffer_Loc,  uint8_t *DataBuffer, uint8_t DataLength);
void MCP2515_SPI_BitModify(BTM_Registers_t RegisterAddress, uint8_t maskByte, uint8_t dataByte);

void MCP2515_SPI_Reset(void);
void MCP2515_SPI_RxStatus(uint8_t *RxBuffer, uint8_t rxbuffer_size);
void MCP2515_SPI_ReadStatus(uint8_t *RxBuffer, uint8_t rxbuffer_size);

void MCP2515_SPI_ReadRegister(uint8_t start_address, uint8_t *RxBuffer, uint8_t rxbuffer_size);
uint8_t MCP2515_SPI_Read_SingleRegister(uint8_t start_address);
void MCP2515_SPI_ReadRxBuffer(Read_RxBufferLoc_t RxBuffer_Loc, uint8_t *RxBuffer, uint8_t rxbuffer_size);

void MCP2515_SPI_RequestToSend(RTS_TxBuffer_t TxBuffer);

void MCP2515_Init(MCP2515_CFG_Handle_t *MCP2515_handle);

CAN_Tx_Status_t MCP2515_CAN_Transmit_Single_TxBuffer(uint8_t Send_TxBuffern, uint16_t ArbitrationID, uint8_t *DataBuffer, uint8_t DataLength);
void MCP2515_CAN_Receive_INT(void);
#endif /* MCP2515_DRIVER_MCP2515_DRIVER_H_ */
