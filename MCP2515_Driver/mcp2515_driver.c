/*
 * mcp2515_driver.c
 *
 *  Created on: Sep 10, 2025
 *      Author: eldon
 */

#include "mcp2515_driver.h"

/***Static Function Prototypes***/
static void MCP2515_SPI_Transmit(uint8_t *dataByte, uint8_t data_length);
static void MCP2515_Filters_Init(MCP2515_CFG_Handle_t *MCP2515_handle);
static void MCP2515_SPI_TransmitReceive(uint8_t *TxBuffer, uint8_t command_byte_length, uint8_t *RxBuffer, uint8_t rxbuffer_size);


//dummy byte for clocking out slave data during reception
const uint8_t dummy_byte = 0x00;


//Register mask temp
static uint8_t register_mask_temp[3] = {0};

/****************************************************************
 * @brief 	Transmit data to MCP2515 through SPI
 *
 * @param	DataBuffer: Pointer to the data buffer array to be transmitted.
 * @param	data_length: Number of bytes to transmit.
 */
static void MCP2515_SPI_Transmit(uint8_t *DataBuffer, uint8_t data_length){

	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET); //Pull CS low to enable MCP2515

	//send command through SPI polling mode, DMA for future optimization
	HAL_SPI_Transmit(&hspi1, DataBuffer,data_length, HAL_MAX_DELAY);

	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET); //Pull CS high end of transmission

}


/****************************************************************
 * @brief 	Transmit and receive data to/from MCP2515 through SPI
 *
 * @param	TxBuffer: Pointer to the data byte array to be transmitted.
 * @param 	command_byte_length: number of command bytes
 * @param 	RxBuffer: user defined buffer to store received data.
 * @param	rxbuffer_size: size of user define RxBuffer in bytes
 */
static void MCP2515_SPI_TransmitReceive(uint8_t *TxBuffer, uint8_t command_byte_length, uint8_t *RxBuffer, uint8_t rxbuffer_size) {

	if(command_byte_length == 1){
		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET); //Pull CS low to enable MCP2515

		/*********************************************************************
		 * send command through SPI polling mode, DMA for future optimization
		 * rxbuffer_size + 1, to make space for command byte.
		 * Txbuffer is also receive buffer, data inside TxBuffer will just get overwritten when receiving
		 *********************************************************************/
		HAL_SPI_TransmitReceive(&hspi1, TxBuffer, TxBuffer, (rxbuffer_size + 1),HAL_MAX_DELAY);

		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET); //Pull CS high end of transmission
		/*********************************************************************
		 * copy contents starting from TxBuffer[1] to end into user RxBuffer.
		 * TxBuffer[0] data is trash, ignore
		 *********************************************************************/
		memcpy(RxBuffer, &TxBuffer[1], rxbuffer_size);
	}
	else if(command_byte_length == 2){
		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET); //Pull CS low to enable MCP2515
		/*********************************************************************
		 * send command through SPI polling mode, DMA for future optimization
		 * rxbuffer_size + 2, to make space for command bytes.
		 * Txbuffer is also receive buffer, data inside TxBuffer will just get overwritten when receiving
		 *********************************************************************/
		HAL_SPI_TransmitReceive(&hspi1, TxBuffer, TxBuffer, (rxbuffer_size + 2),HAL_MAX_DELAY);

		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET); //Pull CS high end of transmission
		/*********************************************************************
		 * copy contents starting from TxBuffer[2] (to end) into user RxBuffer.
		 * TxBuffer[0] & TxBuffer[1] data is trash, ignore
		 *********************************************************************/
		memcpy(RxBuffer, &(TxBuffer[2]), rxbuffer_size);
	}
}


/**********************************************************************
 * @brief 	Resets internal registers to the default state. After reset, MCP2515 is in Configuration mode.
 */
void MCP2515_SPI_Reset(void){
	uint8_t commandByte = SPI_COMMAND_RESET;
	MCP2515_SPI_Transmit(&commandByte, 1);
}


/**********************************************************************
 * @brief 	Sends the contents of one or more transmit buffers onto the CAN bus.
 *
 * @param	TxBuffer: The transmit buffer(s) to send. @RTS_TxBuffer_t
 */
void MCP2515_SPI_RequestToSend(RTS_TxBuffer_t TxBuffer){

	uint8_t TxBuffer_index = TxBuffer;

	MCP2515_SPI_Transmit(&TxBuffer_index, 1);
}


/**********************************************************************
 * @brief 	Allows to set or clear individual bits in a register without a read-modify-write sequence.
 *
 * @param	RegisterAddress: The address of the register to modify.
 * 			Register have to be Bit Modify Capable. Ref @BTM_Registers_t
 * @param	maskByte: A mask byte that indicates which bits are to be modified. Ref @BIT_MASK
 * @param	dataByte: A data byte that indicates the values to write to the bits specified by the mask.
 */
void MCP2515_SPI_BitModify(BTM_Registers_t RegisterAddress, uint8_t maskByte, uint8_t dataByte){

	uint8_t command_data_byte_buffer[4];

	//construct the SPI data frame, command byte + address byte + mask byte + data byte
	command_data_byte_buffer[0] = SPI_COMMAND_BIT_MODIFY;
	command_data_byte_buffer[1] = RegisterAddress;
	command_data_byte_buffer[2] = maskByte;
	command_data_byte_buffer[3] = dataByte;

	MCP2515_SPI_Transmit(command_data_byte_buffer, 4);
}


/**********************************************************************
 * @brief 	Writes data to MCP2515 register beginning at the selected address.
 *
 * @param	start_address: The address from which to start writing. Ref @register_address
 * @param	*DataBuffer: Pointer to the data buffer to be written into register.
 * @param	data_length: Number of bytes to write.
 */
void MCP2515_SPI_WriteRegister(uint8_t start_address, uint8_t *DataBuffer, uint8_t data_length){

	//create an array to hold command byte, address byte, and data bytes
	uint8_t command_data_byte_buffer[2 + data_length];

	//construct the command & address section of array
	command_data_byte_buffer[0] = SPI_COMMAND_WRITEREGISTER;
	command_data_byte_buffer[1] = start_address;

	//copy contents of data buffer into data section of array
	memcpy(&(command_data_byte_buffer[2]), DataBuffer, data_length);

	//transmit the whole buffer to SPI
	MCP2515_SPI_Transmit(command_data_byte_buffer, (2 + data_length));
}


/**********************************************************************
 * @brief 	When loading a transmit buffer, reduces the overhead of a normal WRITE command
 * 			by placing the Address Pointer at one of six TxBuffer locations.
 *
 * @param	TxBuffer_Loc: The location from which to start writing. Refer to @Load_TxBufferLoc_t for TxBuffer locations
 * @param	*DataBuffer: Pointer to the data buffer to be written into register.
 * @param	data_length: Number of bytes to write.
 */
void MCP2515_SPI_LoadTxBuffer(Load_TxBufferLoc_t TxBuffer_Loc,  uint8_t *DataBuffer, uint8_t data_length){

	//create an array to hold command byte and data bytes
	uint8_t command_data_byte_buffer[1 + data_length];

	//construct the command section of array
	command_data_byte_buffer[0] = TxBuffer_Loc;

	//copy contents of data buffer into data section of array
	memcpy(&(command_data_byte_buffer[1]), DataBuffer, data_length);

	//transmit the whole buffer to SPI
	MCP2515_SPI_Transmit(command_data_byte_buffer, (data_length + 1));
}

/***********************************************************************
 * @brief 	Allows quick polling of the MCP2515 to determine the status of the receive and transmit buffers.
 *
 * @param 	*RxBuffer: pointer to Rxbuffer to store received data.
 * @param	rxBuffer_size: Size of RxBuffer in bytes.
 *
 * @NOTE: 	This will fill out all of RxBuffer elements
 */
void MCP2515_SPI_ReadStatus(uint8_t *RxBuffer, uint8_t rxbuffer_size){

	//command byte & rxBuffer dummy bytes (to clock out data from slave)
	uint8_t command_data_byte_buffer[1 + rxbuffer_size];

	//construct the SPI data frame, command byte + dummy byte (for slave data clock out)
	command_data_byte_buffer[0] = SPI_COMMAND_READ_STATUS;
	memset(&(command_data_byte_buffer[1]), dummy_byte, rxbuffer_size);

	//transmit the whole buffer to SPI and receive data into RxBuffer
	MCP2515_SPI_TransmitReceive(command_data_byte_buffer, 1, RxBuffer, rxbuffer_size);
}


/***********************************************************************
 * @brief 	Quick polling command that indicates filter match and message type
 * 			(standard, extended and/or remote) of received message.
 *
 * @param 	*RxBuffer: pointer to Rxbuffer to store received data.
 * @param	rxBuffer_size: Size of RxBuffer in bytes.
 *
 * @NOTE: 	This will fill out all of RxBuffer elements
 */
void MCP2515_SPI_RxStatus(uint8_t *RxBuffer, uint8_t rxbuffer_size){

	//command byte & rxBuffer dummy bytes (to clock out data from slave)
	uint8_t command_data_byte_buffer[1 + rxbuffer_size];

	//construct the SPI data frame, command byte + dummy byte (for slave data clock out)
	command_data_byte_buffer[0] = SPI_COMMAND_RX_STATUS;
	memset(&(command_data_byte_buffer[1]), dummy_byte, rxbuffer_size);

	//transmit the whole buffer to SPI and receive data into RxBuffer
	MCP2515_SPI_TransmitReceive(command_data_byte_buffer, 1, RxBuffer, rxbuffer_size);
}


/**********************************************************************
 * @brief 	Reads data from the register beginning at selected address.
 *
 * @param	start_address: The address from which to start reading. Ref @register_address
 * @param 	*RxBuffer: pointer to Rxbuffer to store received data.
 * @param	rxbuffer_size: Size of RxBuffer in bytes
 *
 * @NOTE: 	Ensure that start_address + rxbuffer_size does not exceed 0x7D (last register address)
 * @NOTE: 	This will fill out all of RxBuffer elements
 */
void MCP2515_SPI_ReadRegister(uint8_t start_address, uint8_t *RxBuffer, uint8_t rxbuffer_size){

	if(start_address + rxbuffer_size > MAX_ADDRESS) return; //ensure read does not exceed last register address

	//command byte, address byte & rxBuffer dummy bytes to clock out data from slave
	uint8_t command_data_byte_buffer[2 + rxbuffer_size];

	//construct the SPI data frame, command byte + start address byte + dummy byte (for slave data clock out)
	command_data_byte_buffer[0] = SPI_COMMAND_READ;
	command_data_byte_buffer[1] = start_address;
	memset(&(command_data_byte_buffer[2]), dummy_byte, rxbuffer_size); //fill rest of array with dummy bytes

	//transmit the whole buffer to SPI and receive data into RxBuffer
	MCP2515_SPI_TransmitReceive(command_data_byte_buffer, 2, RxBuffer, rxbuffer_size);
}


/**********************************************************************T
 * @brief 	When reading a receive buffer, reduces the overhead of a normal READ command
 * 			by placing the Address Pointer at one of four locations.
 *
 * @param	RxBuffer_Loc: RxBuffer location which to start reading. @Read_RxBufferLoc_t
 * @param 	*RxBuffer: pointer to Rxbuffer to store received data.
 * @param	rxbuffer_size: Size of RxBuffer in bytes
 *
 * @NOTE: 	Ensure that start_address + rxbuffer_size does not exceed 0x7D (last register address)
 * @NOTE: 	This will fill out all of RxBuffer elements
 */
void MCP2515_SPI_ReadRxBuffer(Read_RxBufferLoc_t RxBuffer_Loc, uint8_t *RxBuffer, uint8_t rxbuffer_size){

	if(RxBuffer_Loc + rxbuffer_size > MAX_ADDRESS) return; //ensure read does not exceed last RxBuffer register

	//command byte & rxBuffer dummy bytes (to clock out data from slave)
	uint8_t command_data_byte_buffer[1 + rxbuffer_size];

	//construct the SPI data frame, command byte + dummy byte (for slave data clock out)
	command_data_byte_buffer[0] = RxBuffer_Loc;
	memset(&(command_data_byte_buffer[1]), dummy_byte, rxbuffer_size);

	//transmit the whole buffer to SPI and receive data into RxBuffer
	MCP2515_SPI_TransmitReceive(command_data_byte_buffer, 1, RxBuffer, rxbuffer_size);
}


/**********************************************************************T
 * @brief 	Initializes MCP2515 according the user defined configurations.
 *
 * @param	*MCP2515_handle: Pointer to MCP2515_CFG_Handle_t which holds the user defined configurations.
 *
 * @NOTE: 	ONLY supports Standard CAN frame format. Does not support Extended CAN Frame for now.
 */
void MCP2515_Init(MCP2515_CFG_Handle_t *MCP2515_handle){

	//reset MCP2515 and force into CONFIG MODE
	MCP2515_SPI_Reset();

	HAL_Delay(5); //wait for device to settle

	MCP2515_Filters_Init(MCP2515_handle);

	//enable interrupts specified by user
	if(MCP2515_handle->INT_Enable_Mask != 0){
		//write into CAN interrupt enable register
		MCP2515_SPI_BitModify(CANINTE_BTM, 0xFF, MCP2515_handle->INT_Enable_Mask);
	}

	//TODO: CAN TIMING and BAUDRATE initializations

	//set to Normal mode after all config is done
	MCP2515_SPI_BitModify(CANCTRL_BTM, 0xE0, NORMAL_MODE);


}


/**********************************************************************T
 * @brief 	Helper function for filter initialization and configuration
 *
 * @param	*MCP2515_handle: Pointer to MCP2515_CFG_Handle_t which holds the user defined configurations.
 *
 * @NOTE: 	ONLY supports Standard CAN frame format. Does not support Extended CAN Frame for now.
 */
static void MCP2515_Filters_Init(MCP2515_CFG_Handle_t *MCP2515_handle){
	//Configure RxBuffer0 Filters if specified
	if( (MCP2515_handle->RxBUFFERn_FILT_CFG & RxBUFFER0_FILT_CFG_MASK) != 0){

		//extract bits [10:3] of 11-Bit mask to write into RXM0SIDH  (Receive Buffer 0 Mask Standard Identifier High Register)
		register_mask_temp[0] = (uint8_t)(MCP2515_handle->RxBuffer0_FILT.RxBuffer0_AcceptMask >> 3);

		//extract bits [2:0] of 11-Bit mask to write into RXM0SIDH
		register_mask_temp[1] = (uint8_t)(MCP2515_handle->RxBuffer0_FILT.RxBuffer0_AcceptMask << 5);

		//write acceptance mask into registers RXM0SIDH to RXM0SIDL
		MCP2515_SPI_WriteRegister(RxBUFFER0_MASK_SIDH_REG_ADDR, register_mask_temp, 2);

		//Rxbuffer0 filter 0
		if(MCP2515_handle->RxBuffer0_FILT.RxBuffer0_Filt0 != 0){
			//extract bits [10:3] of 11-Bit SID (Standard Identifier) filter to write into RXF0SIDH
			register_mask_temp[0] = (uint8_t)(MCP2515_handle->RxBuffer0_FILT.RxBuffer0_Filt0 >> 3);

			//extract bits [2:0] of 11-Bit SID filter to write into RXF0SIDL
			register_mask_temp[1] = (uint8_t)(MCP2515_handle->RxBuffer0_FILT.RxBuffer0_Filt0 << 5);

			//write filter into registers RXF0SIDH to RXF0SIDL
			MCP2515_SPI_WriteRegister(RxBUFFER0_FILTER_SIDH_REG_ADDR, register_mask_temp, 2);
		}

		//RxBuffer0 filter 1
		if(MCP2515_handle->RxBuffer0_FILT.RxBuffer0_Filt1 != 0){
			//extract bits [10:3] of 11-Bit SID (Standard Identifier) filter to write into RXF1SIDH
			register_mask_temp[0] = (uint8_t)(MCP2515_handle->RxBuffer0_FILT.RxBuffer0_Filt1 >> 3);

			//extract bits [2:0] of 11-Bit SID filter to write into RXF1SIDL
			register_mask_temp[1] = (uint8_t)(MCP2515_handle->RxBuffer0_FILT.RxBuffer0_Filt1 << 5);

			//write filter into registers RXF1SIDH to RXF1SIDL
			MCP2515_SPI_WriteRegister(RxBUFFER0_FILTER_SIDH_REG_ADDR, register_mask_temp, 2);
		}
	}

	//Configure RxBuffer1 Filters if specified
	if( (MCP2515_handle->RxBUFFERn_FILT_CFG & RxBUFFER1_FILT_CFG_MASK) != 0){

		//extract bits [10:3] of 11-Bit mask to write into RXM1SIDH  (Receive Buffer 0 Mask Standard Identifier High Register)
		register_mask_temp[0] = (uint8_t)(MCP2515_handle->RxBuffer1_FILT.RxBuffer1_AcceptMask >> 3);

		//extract bits [2:0] of 11-Bit mask to write into RXM1SIDH
		register_mask_temp[1] = (uint8_t)(MCP2515_handle->RxBuffer1_FILT.RxBuffer1_AcceptMask << 5);

		//write acceptance mask into registers RXM1SIDH to RXM1SIDL
		MCP2515_SPI_WriteRegister(RxBUFFER1_MASK_SIDH_REG_ADDR, register_mask_temp, 2);

		//RxBuffer1 filter 2
		if(MCP2515_handle->RxBuffer1_FILT.RxBuffer1_Filt2 != 0){
			//extract bits [10:3] of 11-Bit SID (Standard Identifier) filter to write into RXF2SIDH
			register_mask_temp[0] = (uint8_t)(MCP2515_handle->RxBuffer1_FILT.RxBuffer1_Filt2 >> 3);

			//extract bits [2:0] of 11-Bit SID filter to write into RXF2SIDL
			register_mask_temp[1] = (uint8_t)(MCP2515_handle->RxBuffer1_FILT.RxBuffer1_Filt2 << 5);

			//write filter into registers RXF2SIDH to RXF2SIDL
			MCP2515_SPI_WriteRegister(RxBUFFER2_FILTER_SIDH_REG_ADDR, register_mask_temp, 2);
		}

		//RxBuffer1 filter 3
		if(MCP2515_handle->RxBuffer1_FILT.RxBuffer1_Filt3 != 0){
			//extract bits [10:3] of 11-Bit SID (Standard Identifier) filter to write into RXF3SIDH
			register_mask_temp[0] = (uint8_t)(MCP2515_handle->RxBuffer1_FILT.RxBuffer1_Filt3 >> 3);

			//extract bits [2:0] of 11-Bit SID filter to write into RXF3SIDL
			register_mask_temp[1] = (uint8_t)(MCP2515_handle->RxBuffer1_FILT.RxBuffer1_Filt3 << 5);

			//write filter into registers RXF3SIDH to RXF3SIDL
			MCP2515_SPI_WriteRegister(RxBUFFER3_FILTER_SIDH_REG_ADDR, register_mask_temp, 2);
		}

		//RxBuffer1 filter 4
		if(MCP2515_handle->RxBuffer1_FILT.RxBuffer1_Filt4 != 0){
			//extract bits [10:3] of 11-Bit SID (Standard Identifier) filter to write into RXF4SIDH
			register_mask_temp[0] = (uint8_t)(MCP2515_handle->RxBuffer1_FILT.RxBuffer1_Filt4 >> 3);

			//extract bits [2:0] of 11-Bit SID filter to write into RXF4SIDL
			register_mask_temp[1] = (uint8_t)(MCP2515_handle->RxBuffer1_FILT.RxBuffer1_Filt4 << 5);

			//write filter into registers RXF4SIDH to RXF4SIDL
			MCP2515_SPI_WriteRegister(RxBUFFER4_FILTER_SIDH_REG_ADDR, register_mask_temp, 2);
		}

		//RxBuffer1 filter 5
		if(MCP2515_handle->RxBuffer1_FILT.RxBuffer1_Filt5 != 0){
			//extract bits [10:3] of 11-Bit SID (Standard Identifier) filter to write into RXF5SIDH
			register_mask_temp[0] = (uint8_t)(MCP2515_handle->RxBuffer1_FILT.RxBuffer1_Filt5 >> 3);

			//extract bits [2:0] of 11-Bit SID filter to write into RXF5SIDL
			register_mask_temp[1] = (uint8_t)(MCP2515_handle->RxBuffer1_FILT.RxBuffer1_Filt5 << 5);

			//write filter into registers RXF5SIDH to RXF5SIDL
			MCP2515_SPI_WriteRegister(RxBUFFER5_FILTER_SIDH_REG_ADDR, register_mask_temp, 2);
		}
	}
}

/********************************INTERRUPT HANDLER*****************************************/


