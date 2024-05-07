#include "spi_help_fun.h"

/*----------------------------------------------------------------------*/
/*---------------------SPI Data Transmit 8 Bit--------------------------*/
/*----------------------------------------------------------------------*/
uint8_t SPI_Data_Transmit_8BIT(SPI_TypeDef* SPIx, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms) {
	
	if (!READ_BIT(SPIx->SR, SPI_SR_BSY)) {
		for (uint16_t i = 0; i < Size_data; i++) {
			
			while (!READ_BIT(SPIx->SR, SPI_SR_TXE)) { LL_mDelay(Timeout_ms); }
			LL_SPI_TransmitData8(SPIx , *(data + i) ); 
		}
		
		while (!LL_SPI_IsActiveFlag_TXE(SPIx)) 
		{ LL_mDelay(Timeout_ms); }
		
		while (READ_BIT(SPIx->SR, SPI_SR_BSY)) { LL_mDelay(Timeout_ms); }
		return 1;
	}
	else {
		return 0;
	}
}

/*----------------------------------------------------------------------*/
/*---------------------SPI Data Receive 8 Bit---------------------------*/
/*----------------------------------------------------------------------*/
uint8_t SPI_Data_Receive_8BIT(SPI_TypeDef* SPIx, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms) {
	if (!READ_BIT(SPIx->SR, SPI_SR_BSY)) {

		if (READ_BIT(SPIx->SR, SPI_SR_OVR) || READ_BIT(SPIx->SR, SPI_SR_RXNE)) 
		{
			SPIx->DR;
		}

		for (uint16_t i = 0; i < Size_data; i++) {
			LL_SPI_TransmitData8(SPIx , 0 );
			while (!LL_SPI_IsActiveFlag_RXNE(SPIx)) {  LL_mDelay(Timeout_ms); }
			*(data + i) = LL_SPI_ReceiveData8(SPIx);
		}

		while (READ_BIT(SPIx->SR, SPI_SR_BSY)) { LL_mDelay(Timeout_ms); }
		return 1;
	}
	else {
		return 0;
	}
}