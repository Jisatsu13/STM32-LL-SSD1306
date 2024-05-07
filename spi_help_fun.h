#ifndef SPI_HELP_FUN
#define SPI_HELP_FUN

#include "stm32f1xx_ll_spi.h"
#include "stm32f1xx_ll_utils.h"

uint8_t SPI_Data_Transmit_8BIT(SPI_TypeDef* SPI, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms);
uint8_t SPI_Data_Receive_8BIT(SPI_TypeDef* SPI, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms);

#endif