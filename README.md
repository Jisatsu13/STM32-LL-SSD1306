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

	ssd1306_Init(I2C1);
	LL_mDelay(100);

  while (1)
  {
                ssd1306_Fill(Black);
		ssd1306_DrawText("HELLO",  FONT1, 0, 0 , White);
		
		ssd1306_DrawText("WORLD",  FONT1, 0, 14 , White);
		
  	        ssd1306_UpdateScreen(I2C1);
		LL_mDelay(500);

  
                ssd1306_Fill(Black);
		ssd1306_DrawText("LIBRARY",  FONT1, 0, 0 , White);
		
		ssd1306_DrawText("IS WORK",  FONT1, 0, 14 , White);
		
  	        ssd1306_UpdateScreen(I2C1);
		LL_mDelay(500);
  
  
	
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}
