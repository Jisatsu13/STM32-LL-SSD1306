Low level(ll) library for OLED display and STM32.
Here i use STM32f103. If u have another device, just change this library:

//ssd1306.h
#include "stm32f1xx_ll_utils.h"

//i2c_help_fun.h
#include "stm32f1xx_ll_i2c.h"
#include "stm32f1xx_ll_gpio.h"

Also i used 128x32 OLED display? so change some line(i commented they) for ur device.

Example

int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),15, 0));

  /** NOJTAG: JTAG-DP Disabled and SW-DP Enabled
  */
  LL_GPIO_AF_Remap_SWJ_NOJTAG();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	
	ssd1306_Init(I2C1);
	LL_SPI_Enable(SPI1);
	LL_SPI_Enable(SPI2);
	
	char buf[7];
	
		l3g4200d_ctrl_reg_t control;
		l3g4200d_status_reg_t status;		
//		_L3G4200D_GET_CTRL(SPI2, &control);
//		control.reg1.PD  = 1;
//		control.reg1.XEN = 1;
//		control.reg1.YEN = 1;
//		control.reg1.ZEN = 0;
//		_L3G4200D_SET_CTRL(SPI2, _L3G4200D_CTRL_REG1, &control);

	uint8_t reg[2] = {0x01, 0x1d};
	
	uint16_t regx;
	uint16_t regy;
	uint16_t regz;
	
	uint8_t ans1[6];
	uint8_t ans2[2];
	
	uint8_t dt = 0x80;
	float res;
		
	
	I2C_WriteBytesToAddr(I2C2, 0x1a, 0x0b, &reg[0], 1);
	I2C_WriteBytesToAddr(I2C2, 0x1a, 0x09, &reg[1], 1);
	
	LL_mDelay(100);
	
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	
	
//		LL_mDelay(100);	
//		_L3G4200D_GetStatus(SPI2, &status);
//		
//		if(status.XDA){ myData.x = L3G4200D_ReadData_Reg( SPI2,_L3G4200D_REG_OUT_X_H, _L3G4200D_REG_OUT_X_L ); }
//		if(status.YDA){ myData.y = L3G4200D_ReadData_Reg( SPI2,_L3G4200D_REG_OUT_Y_H, _L3G4200D_REG_OUT_Y_L ); }
//		if(status.ZDA){ myData.z = L3G4200D_ReadData_Reg( SPI2,_L3G4200D_REG_OUT_Z_H, _L3G4200D_REG_OUT_Z_L ); }		

//		if(status.XDA){ myData.x = LIS3DH_ReadData_Reg( SPI1, LIS3DH_OUT_X_H_ADDR, LIS3DH_OUT_X_L_ADDR ); }
//		if(status.YDA){ myData.y = LIS3DH_ReadData_Reg( SPI1, LIS3DH_OUT_Y_H_ADDR, LIS3DH_OUT_Y_L_ADDR ); }
//		if(status.ZDA){ myData.z = LIS3DH_ReadData_Reg( SPI1, LIS3DH_OUT_Z_H_ADDR, LIS3DH_OUT_Z_L_ADDR ); }			
//	
//		
		I2C_ReadData(I2C2, 0x1a, 0x06, ans1, 1);
		if(ans1[0] & 0x01)
		{ 
			I2C_ReadData(I2C2, 0x1a, 0x00, ans1, 6); 
			regx = (ans1[1] << 8) | ans1[0];
			regy = (ans1[3] << 8) | ans1[2];
			regz = (ans1[5] << 8) | ans1[4];
			
			res = atan2f(regy, regx) * 180.0f/3.14f;
			
			if(res < 0)
			{
				res += 360.0f;
			}
		}	
		
		float_to_str(res, buf);
		ssd1306_DrawText(buf,  FONT1, 40, 0 , White);
		
		hex16_to_str(regx, buf);
		ssd1306_DrawText(buf,  FONT1, 0, 14 , White);
		
		hex16_to_str(regy, buf);
		ssd1306_DrawText(buf,  FONT1, 84, 14 , White);
		
//		ssd1306_DrawText("gyro is ", FONT1, 0, 0 , White);
//				
//		int_to_str(myData.x, buf);
//		ssd1306_DrawText(buf,  FONT1, 0, 14 , White);
//		
//		int_to_str(myData.y, buf);
//		ssd1306_DrawText(buf,  FONT1, 44, 14 , White);
//		
//		int_to_str(myData.z, buf);
//		ssd1306_DrawText(buf,  FONT1, 84, 14 , White);
		
  	ssd1306_UpdateScreen(I2C1);
		LL_mDelay(500);
	
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}
