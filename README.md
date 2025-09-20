# MCP2515 CAN Controller Driver 
MCP2515 standalone CAN controller driver ported for STM32WB5MMG MCU which will be utilized for my MotoHUD project.

Repo Link: https://github.com/eldonjakee08/MOTOHUD_Project

# Hot To Use The Driver
1. Define a MCP2515_CFG_Handle_t, this will be used to store the configuration setting for MCP2515.
   <img width="439" height="96" alt="image" src="https://github.com/user-attachments/assets/9ea7857c-e5d1-4751-9010-4b6c8cd3e7cc" /><br>

2. Create an init helper function and call it during the initialization phase in main.c.<br>
   <img width="475" height="355" alt="image" src="https://github.com/user-attachments/assets/77a9b812-6a8d-4791-a042-56c4d520006e"/><br>

3. Inside the init helper function, fill the MCP2515_CFG_Handle_t with the desired configuration settings of MCP2515. Call the MCP2515_Init() function with the handle pointer as input argument to initialize MCP2515. 
   <img width="926" height="559" alt="image" src="https://github.com/user-attachments/assets/765f0277-e228-40c6-8741-3be3618c24eb" /><br>

   Note: refer to MCP2515_CFG_Handle_t in mcp2515_driver.h file for the configuration parameters definition. <br>

4. After initialization you can now call the MCP2515 APIs.<br>

5. Message reception from CAN bus is handle through an Interrupt Service Routine (on-going development)
   
# MCP2515 APIs
1. MCP2515_SPI_Reset()
   <img width="1078" height="104" alt="image" src="https://github.com/user-attachments/assets/e4dc67b8-7ae9-4561-ae77-9ca8d208520f" /><br>

2. MCP2515_SPI_RequestToSend()
   <img width="909" height="145" alt="image" src="https://github.com/user-attachments/assets/999271b7-a81c-411d-b459-d85956801204" /><br>

3. MCP2515_SPI_BitModify()
   <img width="1049" height="216" alt="image" src="https://github.com/user-attachments/assets/db7f58fc-6fce-4b41-8fb5-27844e43e332" /><br>

4. MCP2515_SPI_WriteRegister()
   <img width="987" height="190" alt="image" src="https://github.com/user-attachments/assets/4abbe8f1-5c9b-466e-8e3a-e6a7049eafbb" /><br>

5. MCP2515_SPI_LoadTxBuffer()
   <img width="1213" height="218" alt="image" src="https://github.com/user-attachments/assets/618ff21b-b373-4973-a0bc-3b1858cecb61" /><br>

6. MCP2515_SPI_ReadStatus()
   <img width="1100" height="218" alt="image" src="https://github.com/user-attachments/assets/3e47edd7-1c70-4021-a690-79511a6bdfe0" /><br>

7. MCP2515_SPI_RxStatus()
   <img width="893" height="222" alt="image" src="https://github.com/user-attachments/assets/04fd60bb-0dbb-45b1-8745-0d39668b4bef" /><br>

8. MCP2515_SPI_ReadRegister()
   <img width="1017" height="258" alt="image" src="https://github.com/user-attachments/assets/b61391be-f145-4787-93a2-02a22ed996f2" /><br>

9. MCP2515_SPI_Read_SingleRegister()
   <img width="927" height="208" alt="image" src="https://github.com/user-attachments/assets/a610119c-0ba8-4c24-88f9-58d12d51a2d4" /><br>

10. MCP2515_SPI_ReadRxBuffer()
    <img width="1091" height="259" alt="image" src="https://github.com/user-attachments/assets/ae8cba44-9048-410a-97ce-c130084778af" /><br>

11. MCP2515_Init()
    <img width="1090" height="187" alt="image" src="https://github.com/user-attachments/assets/041a35bb-ee90-4cfd-b879-31fdca06c024" /><br>

12. MCP2515_CAN_Transmit_Single_TxBuffer()
    <img width="1438" height="261" alt="image" src="https://github.com/user-attachments/assets/d49573ea-490c-494a-9463-a05313c6e0ac" /><br>
