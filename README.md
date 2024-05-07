Low level(ll) library for OLED display and STM32.
Here i use STM32f103. If u have another device, just change this library:

//ssd1306.h
#include "stm32f1xx_ll_utils.h"

//i2c_help_fun.h
#include "stm32f1xx_ll_i2c.h"
#include "stm32f1xx_ll_gpio.h"

Also i used 128x32 OLED display? so change some line(i commented they) for ur device.

Example

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
  	}
   }
