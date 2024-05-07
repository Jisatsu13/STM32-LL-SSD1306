#include "i2c_help_fun.h"

/* Start I2C function.
 * @param I2Cx 	   Selected I2C
 */
void I2C_Start (I2C_TypeDef *I2Cx)
{
	LL_I2C_AcknowledgeNextData(I2Cx, LL_I2C_ACK); // Enable the ACK
  LL_I2C_GenerateStartCondition(I2Cx);          // Generate START
  while(!LL_I2C_IsActiveFlag_SB(I2Cx)){};
}

/* Stop I2C function.
 * @param I2Cx 	   Selected I2C
 */
void I2C_Stop (I2C_TypeDef *I2Cx)
{
	LL_I2C_GenerateStopCondition(I2Cx);  // Stop I2C
}

/* Only write byte I2C function.
 * @param I2Cx 	   Selected I2C
 */
void I2C_Write (I2C_TypeDef *I2Cx, uint8_t data)
{
	while(!LL_I2C_IsActiveFlag_TXE(I2Cx)){};    // wait for TXE bit to set
	LL_I2C_TransmitData8(I2Cx, data);
	while (!LL_I2C_IsActiveFlag_BTF(I2Cx)) {};  // wait for BTF bit to set
}

/* Choise Slave Adress I2C function.
 * @param I2Cx 	   Selected I2C
 */
void I2C_Address (I2C_TypeDef *I2Cx,uint8_t Address)
{
	LL_I2C_TransmitData8(I2Cx, Address);      //  send the address
	while(!LL_I2C_IsActiveFlag_ADDR(I2Cx)){}; // wait for ADDR bit to set
	LL_I2C_ClearFlag_ADDR(I2Cx);              // read SR1 and SR2 to clear the ADDR bit
}

/* Only read from slave I2C function.
 * @param I2Cx 	   Selected I2C
 * @param Address  I2C slave device address
 * @param buffer 	 Data from register
 * @param size 		 How many bytes will write
 */
void I2C_Read (I2C_TypeDef *I2Cx, uint8_t Address, uint8_t *buffer, uint8_t size)
{	
	
	int remaining = size;
	I2C_Address(I2Cx, Address);                           //send the address device
	
	if (size == 1)
	{
		LL_I2C_AcknowledgeNextData(I2Cx, LL_I2C_NACK);      // clear the ACK bit 
		LL_I2C_GenerateStopCondition(I2Cx);                 // Stop I2C
		while(!LL_I2C_IsActiveFlag_RXNE(I2Cx)){};           // wait for RxNE to set
		buffer[size-remaining] = LL_I2C_ReceiveData8(I2Cx); // Read the data from the DATA REGISTER
	}	
	else 
	{
		while (remaining>2)
		{
			
			while(!LL_I2C_IsActiveFlag_RXNE(I2Cx)){};          // wait for RxNE to set
			buffer[size-remaining] = LL_I2C_ReceiveData8(I2Cx);// copy the data into the buffer			
			LL_I2C_AcknowledgeNextData(I2Cx, LL_I2C_ACK);     // Set the ACK bit to Acknowledge the data received
		
			remaining--;
		}
		while(!LL_I2C_IsActiveFlag_RXNE(I2Cx)){};          // wait for RxNE to set
		buffer[size-remaining] = LL_I2C_ReceiveData8(I2Cx);// copy the data into the buffer		
		LL_I2C_AcknowledgeNextData(I2Cx, LL_I2C_NACK);     // clear the ACK bit 
		LL_I2C_GenerateStopCondition(I2Cx);                // Stop I2C
		remaining--;
		
		// Read the Last BYTE
		while(!LL_I2C_IsActiveFlag_RXNE(I2Cx)){};          // wait for RxNE to set
		buffer[size-remaining] = LL_I2C_ReceiveData8(I2Cx);// copy the data into the buffer
	}	
}

/** Write 8-bit to device register.
 * @param I2Cx 	   Selected I2C
 * @param Address  I2C slave device address
 * @param Reg   	 In reg write data
 * @param Data 		 Data write into register
 */
void I2C_WriteData(I2C_TypeDef *I2Cx, uint8_t Address, uint8_t Reg, uint8_t Data)
{
	I2C_Start (I2Cx);
	I2C_Address (I2Cx, Address);
	I2C_Write (I2Cx, Reg);
	I2C_Write (I2Cx, Data);
	I2C_Stop (I2Cx);
}
/** Read from device register.
 * @param I2Cx 	   Selected I2C
 * @param Address  I2C slave device address
 * @param Reg   	 Register to read from
 * @param buffer 	 Data from register
 * @param size 		 How many bytes will write
 */
void I2C_ReadData(I2C_TypeDef *I2Cx, uint8_t Address, uint8_t Reg, uint8_t *buffer, uint8_t size)
{
	I2C_Start (I2Cx);
	I2C_Address (I2Cx, Address);
	I2C_Write (I2Cx, Reg);
	I2C_Start (I2Cx);
	I2C_Read (I2Cx, Address+0x01, buffer, size);
	I2C_Stop (I2Cx);
}
