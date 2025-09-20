/*
 * mcp2515_driver.c
 *
 *  Created on: Sep 10, 2025
 *      Author: eldon
 */

#include "mcp2515_driver.h"

/***Static Function Prototypes***/
static void MCP2515_SPI_Transmit(uint8_t *dataByte, uint8_t DataLength);
static void MCP2515_Filters_Init(MCP2515_CFG_Handle_t *MCP2515_handle);
static void MCP2515_SPI_TransmitReceive(uint8_t *TxBuffer, uint8_t command_byte_length, uint8_t *RxBuffer, uint8_t rxbuffer_size);

//Temp storage for register value before writing
static uint8_t register_value_temp[2] = {0};

//for debug purposes only
//uint8_t global_rx_buffer[20] = {0};

/***************************************************************************************
 * @brief 	Transmit data to MCP2515 through SPI
 *
 * @param	DataBuffer: Pointer to the data buffer array to be transmitted.
 * @param	DataLength: Number of bytes to transmit.
 */
static void MCP2515_SPI_Transmit(uint8_t *DataBuffer, uint8_t DataLength){

	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET); //Pull CS low to enable MCP2515

	//send command through SPI polling mode, DMA for future optimization
	HAL_SPI_Transmit(&hspi1, DataBuffer,DataLength, HAL_MAX_DELAY);

	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET); //Pull CS high end of transmission

}


/***************************************************************************************
 * @brief 	Transmit and receive data to/from MCP2515 through SPI
 *
 * @param	TxBuffer: Pointer to the data byte array to be transmitted.
 * @param 	command_byte_length: number of command bytes
 * @param 	RxBuffer: user defined buffer to store received data.
 * @param	rxbuffer_size: size of user define RxBuffer in bytes
 */
static void MCP2515_SPI_TransmitReceive(uint8_t *TxBuffer, uint8_t command_byte_length, uint8_t *RxBuffer, uint8_t rxbuffer_size) {

	switch (command_byte_length) {
	case 1:
		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET); //Pull CS low to enable MCP2515

		/*********************************************************************
		 * send command through SPI polling mode, DMA for future optimization
		 * rxbuffer_size + 1, to make space for command byte.
		 * Txbuffer is also receive buffer, data inside TxBuffer will just get overwritten when receiving
		 *********************************************************************/
		HAL_SPI_TransmitReceive(&hspi1, TxBuffer, TxBuffer, (rxbuffer_size + 1), HAL_MAX_DELAY);

		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET); //Pull CS high end of transmission
		/*********************************************************************
		 * copy contents starting from TxBuffer[1] to end into user RxBuffer.
		 * TxBuffer[0] data is trash, ignore
		 *********************************************************************/
		memcpy(RxBuffer, &TxBuffer[1], rxbuffer_size);
		break;
	case 2:
		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET); //Pull CS low to enable MCP2515
		/*********************************************************************
		 * send command through SPI polling mode, DMA for future optimization
		 * rxbuffer_size + 2, to make space for command bytes.
		 * Txbuffer is also receive buffer, data inside TxBuffer will just get overwritten when receiving
		 *********************************************************************/
		HAL_SPI_TransmitReceive(&hspi1, TxBuffer, TxBuffer, (rxbuffer_size + 2), HAL_MAX_DELAY);

		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET); //Pull CS high end of transmission
		/*********************************************************************
		 * copy contents starting from TxBuffer[2] (to end) into user RxBuffer.
		 * TxBuffer[0] & TxBuffer[1] data is trash, ignore
		 *********************************************************************/
		memcpy(RxBuffer, &TxBuffer[2], rxbuffer_size);
		break;
	}
}


/***************************************************************************************
 * @brief 	Resets internal registers to the default state. After reset, MCP2515 is in Configuration mode.
 */
void MCP2515_SPI_Reset(void){


	uint8_t commandByte = SPI_COMMAND_RESET;
	MCP2515_SPI_Transmit(&commandByte, 1);
}


/***************************************************************************************
 * @brief 	Sends the contents of one or more transmit buffers onto the CAN bus.
 *
 * @param	TxBuffer: The transmit buffer(s) to send. @RTS_TxBuffer_t
 * TODO: not yet verified if it works
 */
void MCP2515_SPI_RequestToSend(RTS_TxBuffer_t TxBuffer){

	//uint8_t TxBuffer_index = TxBuffer;

	MCP2515_SPI_Transmit((uint8_t*)&TxBuffer, 1);
}


/***************************************************************************************
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


/***************************************************************************************
 * @brief 	Writes data to MCP2515 register beginning at the selected address.
 *
 * @param	start_address: The address from which to start writing. Ref @register_address
 * @param	*DataBuffer: Pointer to the data buffer to be written into register.
 * @param	DataLength: Number of bytes to write.
 */
void MCP2515_SPI_WriteRegister(uint8_t start_address, uint8_t *DataBuffer, uint8_t DataLength){

	//create an array to hold command byte, address byte, and data bytes
	uint8_t command_data_byte_buffer[2 + DataLength];

	//construct the command & address section of array
	command_data_byte_buffer[0] = SPI_COMMAND_WRITEREGISTER;
	command_data_byte_buffer[1] = start_address;

	//copy contents of data buffer into data section of array
	memcpy(&(command_data_byte_buffer[2]), DataBuffer, DataLength);

	//transmit the whole buffer to SPI
	MCP2515_SPI_Transmit(command_data_byte_buffer, (2 + DataLength));
}


/***************************************************************************************
 * @brief 	When loading a transmit buffer, reduces the overhead of a normal WRITE command
 * 			by placing the Address Pointer at one of six TxBuffer locations.
 *
 * @param	TxBuffer_Loc: The location from which to start writing. Refer to @Load_TxBufferLoc_t for TxBuffer locations
 * @param	*DataBuffer: Pointer to the data buffer to be written into register.
 * @param	DataLength: Number of bytes to write.
 * TODO: not yet verified if it works
 */
void MCP2515_SPI_LoadTxBuffer(Load_TxBufferLoc_t TxBuffer_Loc,  uint8_t *DataBuffer, uint8_t DataLength){

	//create an array to hold command byte and data bytes
	uint8_t command_data_byte_buffer[1 + DataLength];

	//construct the command section of array
	command_data_byte_buffer[0] = TxBuffer_Loc;

	//copy contents of data buffer into data section of array
	memcpy(&(command_data_byte_buffer[1]), DataBuffer, DataLength);

	//transmit the whole buffer to SPI
	MCP2515_SPI_Transmit(command_data_byte_buffer, (DataLength + 1));
}

/***************************************************************************************
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


/***************************************************************************************
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


/***************************************************************************************
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


/***************************************************************************************
 * @brief 	Reads current value of a single register
 *
 * @param	register_address: address of register to be read. Ref @register_address
 *
 * @NOTE: 	unlike MCP2515_SPI_ReadRegister, which can read multiple register values,
 * 			this function only reads a single register and returns the value as one byte.
 */
uint8_t MCP2515_SPI_Read_SingleRegister(uint8_t start_address){

	if(start_address > MAX_ADDRESS) return 0x00; //ensure read does not exceed last register address

	//command byte, address byte & dummy bytes to clock out data from slave
	uint8_t command_data_byte_buffer[3];

	//construct the SPI data frame, command byte + start address byte + dummy byte (for slave data clock out)
	command_data_byte_buffer[0] = SPI_COMMAND_READ;
	command_data_byte_buffer[1] = start_address;
	command_data_byte_buffer[2] = dummy_byte;

	//transmit the whole buffer to SPI and receive data into RxBuffer
	MCP2515_SPI_TransmitReceive(command_data_byte_buffer, 2, &command_data_byte_buffer[2], 1);

	return command_data_byte_buffer[2];
}


/***************************************************************************************
 * @brief 	Reads the RxBuffer contents.
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


/***************************************************************************************
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

	/*********CAN Timing Initialization***************/
	//period for 1 Time Quanta, multiply by 1e9 to up-scale
	uint32_t TQ_period = ( ( 2 * (MCP2515_handle->CAN_Baud_PreScaler + 1) * 1000000000 ) / MCP2515_handle->MCP2515_Osc );

	//Period for 1 bit time, multiply by 1e9 to up-scale
	uint32_t Bit_period = (1000000000 / MCP2515_handle->CAN_Speed_Kbps);

	//number of time quanta per bit
	uint8_t  TQ = Bit_period / TQ_period;

	//Time Quanta length per bit should be >= 8 as per MCP2515 specification
	if(TQ >= 8){
		//check if Segments TQ lengthS conform to MCP2515 requirements
		if( (MCP2515_handle->ProgDelay_TQ_Length + MCP2515_handle->PH1_Seg_TQ_Length + 2) < (MCP2515_handle->PH2_Seg_TQ_Length + 1)   ||
			(MCP2515_handle->ProgDelay_TQ_Length + MCP2515_handle->PH1_Seg_TQ_Length + 2) < (MCP2515_handle->ProgDelay_TQ_Length + 1) ||
			(MCP2515_handle->PH2_Seg_TQ_Length + 1) < (MCP2515_handle->SJW_TQ_Length + 1) ||
			(MCP2515_handle->PH2_Seg_TQ_Length + 1) < 2 ) return;

		//set BTLMODE for PH2_Seg length to be independent of PH1_Seg and IPT
		MCP2515_SPI_BitModify(CNF2_BTM, CNF2_BTLMODE_MASK, CNF2_BTLMODE_MASK);

		//calculate byte for CNF1 register
		register_value_temp[0] = ( (MCP2515_handle->SJW_TQ_Length << 6) ) | (MCP2515_handle->CAN_Baud_PreScaler & 0x1F);
		//write into CNF1 register
		MCP2515_SPI_BitModify(CNF1_BTM, (CNF1_SJW_20_MASK | CNF1_BRP_50_MASK), register_value_temp[0]);

		//calculate byte for CNF2 register
		register_value_temp[0] = ( (MCP2515_handle->PH1_Seg_TQ_Length << 3) ) | (MCP2515_handle->ProgDelay_TQ_Length & 0x07);
		//write into CNF2 register
		MCP2515_SPI_BitModify(CNF2_BTM, (CNF2_PHSEG1_53_MASK | CNF2_PRSEG_20_MASK), register_value_temp[0]);

		//write PHASE2 Segment Length into CNF3 register
		MCP2515_SPI_BitModify(CNF3_BTM, CNF3_PHSEG2_20_MASK,  MCP2515_handle->PH2_Seg_TQ_Length);
	}

	 /*********TxBuffer Priority Init***************
	 * TxBuffer0 - Highest Priority
	 * TxBuffer1 - Mid Priority
	 * TxBuffer2 - Lowest Priority
	 **********************************************/
	uint8_t prio_bits;
	for(uint8_t i=0; i<3 ;i++){
		//prio_bits decrement every iteration with TxBuffer0 having the highest priority value 3
		prio_bits = 3 - i;

		//+16 to TxBuffern CTRL register address every iteration since address gap is 16
		MCP2515_SPI_BitModify((TXB0CTRL_BTM + (i * 16)), TXBnCTRL_TXP_10_MASK, prio_bits);
	}

	//set to Normal mode after all config is done
	MCP2515_SPI_BitModify(CANCTRL_BTM, CANCTRL_REQOP_75_MASK, NORMAL_MODE);
}


/***************************************************************************************
 * @brief 	Sends data into the CAN bus using a single TxBuffer
 *
 * @param	Send_TxBuffern: TxBuffer number used to transmit to CAN bus. Ref @Send_TxBuffer
 * @param	ArbitrationID: Standard 11-bit identifier for the CAN frame
 * @param	*DataBuffer: pointer to the data buffer to be transmitted
 * @param	DataLength: length of data to be transmitted in bytes. Make sure to set DataLength <= sizeof(DataBuffer)
 *
 * @NOTE: 	ONLY supports Standard CAN frame format. Does not support Extended CAN Frame for now.
 * TODO for testing and verification
 */
CAN_Tx_Status_t MCP2515_CAN_Transmit_Single_TxBuffer(uint8_t Send_TxBuffern, uint16_t ArbitrationID, uint8_t *DataBuffer, uint8_t DataLength){

	//bounds check for Data length. Max value is 8
	if(DataLength > 8) DataLength = 8;

	/*********************************************************
	 * Temporary storage for data before sending into CAN bus
	 * TxBuffer_temp[0]: contains TxBuffer start address
	 * TxBuffer_temp[1]: contains TXBnSIDH data
	 * TxBuffer_temp[2]: contains TXBnSIDH data
	 * TxBuffer_temp[3]: contains TXBnEID8, set value as 0x00 since its not needed
	 * TxBuffer_temp[4]: contains TXBnEID0, set value as 0x00 since its not needed
	 * TxBuffer_temp[5]: contains TXBnEID0 data
	 * TxBuffer_temp[6] to TxBuffer_temp[13]: contains data pay-load, end index depends on data length with max index of 13
	 * TxBuffer_temp[RTS_Index]: contains RTS command byte, selects which TxBuffer is triggered to send
	 */
	uint8_t TxBuffer_temp[7 + DataLength];

	//Index where RTS command byte is stored
	uint8_t RTS_Index = (7 + DataLength) - 1;

	//holds the selected TxBuffer address
	uint8_t TxBuffer_address;

	//holds the TxBuffer register value. For status checking
	uint8_t TxBuffer_register_val;

	switch (Send_TxBuffern) {
	case SEND_TxBUFFER0:
		TxBuffer_address = TXBUFFER0_CTRL_REG_ADDR_BTM;
		if(MCP2515_SPI_Read_SingleRegister(TxBuffer_address) & TXBnCTRL_TXREQ_MASK){
			//Buffer busy skip or retry again
			return TXBUFFER_BUSY;
		}
		//Load_TxBufferLoc_t command
		TxBuffer_temp[0] = START_TXB0SIDH;

		//RTS_TxBuffer_t command, equation below calculates the last index of array.
		TxBuffer_temp[RTS_Index] = RTS_TxBUFFER0;
		break;

	case SEND_TxBUFFER1:
		TxBuffer_address = TXBUFFER1_CTRL_REG_ADDR_BTM;
		if(MCP2515_SPI_Read_SingleRegister(TxBuffer_address) & TXBnCTRL_TXREQ_MASK){
			//Buffer busy skip or retry again
			return TXBUFFER_BUSY;
		}
		//Load_TxBufferLoc_t command
		TxBuffer_temp[0] = START_TXB1SIDH;

		//RTS_TxBuffer_t command
		TxBuffer_temp[RTS_Index] = RTS_TxBUFFER1;
		break;

	case SEND_TxBUFFER2:
		TxBuffer_address = TXBUFFER2_CTRL_REG_ADDR_BTM;
		if(MCP2515_SPI_Read_SingleRegister(TxBuffer_address) & TXBnCTRL_TXREQ_MASK){
			//Buffer busy skip or retry again
			return TXBUFFER_BUSY;
		}
		//Load_TxBufferLoc_t command
		TxBuffer_temp[0] = START_TXB2SIDH;

		//RTS_TxBuffer_t command
		TxBuffer_temp[RTS_Index] = RTS_TxBUFFER2;
		break;

	default:
		return ERR_INVALID_INPUT;
		break;
	}

	//to be written into TXBnSIDH:TRANSMIT BUFFER n STANDARD IDENTIFIER REGISTER HIGH
	TxBuffer_temp[1] = (uint8_t)((ArbitrationID & 0x7F8) >> 3);

	//to be written into TXBnSIDH:TRANSMIT BUFFER n STANDARD IDENTIFIER REGISTER LOW
	TxBuffer_temp[2] = (uint8_t)((ArbitrationID & 0x07) << 5);

	//TXBnEID8(TxBuffer_temp[3]) & TXBnEID0(TxBuffer_temp[4]) set as 0x00 since its not needed
	TxBuffer_temp[3] = 0x00;
	TxBuffer_temp[4] = 0x00;

	//to be written into TXBnDLC:TRANSMIT BUFFER n DATA LENGTH CODE REGISTER
	TxBuffer_temp[5] = DataLength & 0x0F;

	//Store Data Payload into TxBuffer_temp to be written into TXBnDm:TRANSMIT BUFFER n DATA BYTE m REGISTER
	memcpy(&TxBuffer_temp[6], DataBuffer, DataLength);

	//Load the selected TxBuffer
	MCP2515_SPI_Transmit(TxBuffer_temp, (6 + DataLength));

	//Trigger to send Selected TxBuffern
	MCP2515_SPI_RequestToSend(TxBuffer_temp[RTS_Index]);

	TxBuffer_register_val = MCP2515_SPI_Read_SingleRegister(TxBuffer_address);

	//if TXREQ bit = 1 after RTS command, transmission has failed
	if(TxBuffer_register_val & TXBnCTRL_TXREQ_MASK){
		//Check if Arbitration loss
		if(TxBuffer_register_val & TXBnCTRL_MLOA_MASK){
			return ERR_ARBITRATION_LOSS;
		}
		//Check if Message Error
		if(TxBuffer_register_val & TXBnCTRL_TXERR_MASK){
			return ERR_TRANSMISSION;
		}
	}

	return TX_SUCCESS;
}


/**********************************************************************
 * @brief 	Helper function for filter initialization and configuration
 *
 * @param	*MCP2515_handle: Pointer to MCP2515_CFG_Handle_t which holds the user defined configurations.
 *
 * @NOTE: 	ONLY supports Standard CAN frame format. Does not support Extended CAN Frame for now.
 */
static void MCP2515_Filters_Init(MCP2515_CFG_Handle_t *MCP2515_handle){

	uint8_t register_address = 0;

	//Configure RxBuffer0 Filters if specified
	if( (MCP2515_handle->RxBUFFERn_FILT_CFG & RxBUFFER0_FILT_CFG_MASK) != 0){

		//extract bits [10:3] of 11-Bit mask to write into RXM0SIDH  (Receive Buffer 0 Mask Standard Identifier High Register)
		register_value_temp[0] = (uint8_t)((MCP2515_handle->RxBuffer0_FILT.RxBuffer0_AcceptMask & 0x7F8) >> 3);

		//extract bits [2:0] of 11-Bit mask to write into RXM0SIDH
		register_value_temp[1] = (uint8_t)((MCP2515_handle->RxBuffer0_FILT.RxBuffer0_AcceptMask & 0x07) << 5);

		//write acceptance mask into registers RXM0SIDH to RXM0SIDL
		MCP2515_SPI_WriteRegister(RxBUFFER0_MASK_SIDH_REG_ADDR, register_value_temp, 2);

		//loop through all filters in Rxbuffer0
		for(uint8_t i = 0; i < 2; i++){
			if(MCP2515_handle->RxBuffer0_FILT.RxBuffer0_Filt[i] != 0){
				//extract bits [10:3] of 11-Bit SID (Standard Identifier) filter to write into RXF0SIDH
				register_value_temp[0] = (uint8_t)((MCP2515_handle->RxBuffer0_FILT.RxBuffer0_Filt[i] & 0x7F8) >> 3);

				//extract bits [2:0] of 11-Bit SID filter to write into RXF0SIDL
				register_value_temp[1] = (uint8_t)((MCP2515_handle->RxBuffer0_FILT.RxBuffer0_Filt[i] & 0x07) << 5);

				//increment to RxBUFFER0_FILTER1_SIDH_REG_ADDR
				register_address = ( RxBUFFER0_FILTER0_SIDH_REG_ADDR + (i * 4) );

				//write filter into registers RXF0SIDH to RXF0SIDL
				MCP2515_SPI_WriteRegister(register_address, register_value_temp, 2);
			}
		}
	}

	//Configure RxBuffer1 Filters if specified
	if( (MCP2515_handle->RxBUFFERn_FILT_CFG & RxBUFFER1_FILT_CFG_MASK) != 0){

		//extract bits [10:3] of 11-Bit mask to write into RXM1SIDH  (Receive Buffer 0 Mask Standard Identifier High Register)
		register_value_temp[0] = (uint8_t)((MCP2515_handle->RxBuffer1_FILT.RxBuffer1_AcceptMask & 0x7F8) >> 3);

		//extract bits [2:0] of 11-Bit mask to write into RXM1SIDH
		register_value_temp[1] = (uint8_t)((MCP2515_handle->RxBuffer1_FILT.RxBuffer1_AcceptMask & 0x07) << 5);

		//write acceptance mask into registers RXM1SIDH to RXM1SIDL
		MCP2515_SPI_WriteRegister(RxBUFFER1_MASK_SIDH_REG_ADDR, register_value_temp, 2);

		//loop through all filters in Rxbuffer1
		for(uint8_t i = 0; i < 4; i++){

			if(MCP2515_handle->RxBuffer1_FILT.RxBuffer1_Filt[i] != 0){
				//extract bits [10:3] of 11-Bit SID (Standard Identifier) filter to write into RXF0SIDH
				register_value_temp[0] = (uint8_t)((MCP2515_handle->RxBuffer1_FILT.RxBuffer1_Filt[i] & 0x7F8) >> 3);

				//extract bits [2:0] of 11-Bit SID filter to write into RXF0SIDL
				register_value_temp[1] = (uint8_t)((MCP2515_handle->RxBuffer1_FILT.RxBuffer1_Filt[i] & 0x07)<< 5);

				//if at Filter 3 and up
				if(i > 0){
					register_address = ( RxBUFFER1_FILTER3_SIDH_REG_ADDR + ( (i-1) * 4 ) );
				}else{
					//if at i = 0, use address RxBUFFER1_FILTER2_SIDH_REG_ADDR
					register_address = RxBUFFER1_FILTER2_SIDH_REG_ADDR;
				}

				//write filter into registers RXF0SIDH to RXF0SIDL
				MCP2515_SPI_WriteRegister(register_address, register_value_temp, 2);
			}

		}
	}
}

/********************************INTERRUPT HANDLER*****************************************/

/**********************************************************************
 * @brief 	Interrupt service routine, at the event of successful reception (filter match) MCP2515 generates an interrupt
 */
void EXTI0_IRQHandler(void)
{

  HAL_GPIO_EXTI_IRQHandler(CAN_INT_Pin);



}
//TODO: implement this ISR
//Message reception interrupt service routine
void MCP2515_CAN_Receive_INT(void){

}
