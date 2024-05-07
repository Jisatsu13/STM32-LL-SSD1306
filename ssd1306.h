#ifndef ssd1306
#define ssd1306

#include "i2c_help_fyn.h"
#include "stm32f1xx_ll_utils.h"

// I2c address
#define SSD1306_I2C_ADDR        0x78
// SSD1306 width in pixels
#define SSD1306_WIDTH           128
// SSD1306 LCD height in pixels
#define SSD1306_HEIGHT          32 //or 64

#define I2C_REQUEST_WRITE                       0x00
#define I2C_REQUEST_READ                        0x01


//
//  Enumeration for screen colors
//
typedef enum {
	Black = 0x00, // Black color, no pixel
	White = 0x01  //Pixel is set. Color depends on LCD
} SSD1306_COLOR;

//
//  Struct to store transformations
//
typedef struct {
	uint16_t CurrentX;
	uint16_t CurrentY;
	uint8_t Inverted;
	uint8_t Initialized;
} SSD1306_t;


//
//  Function definitions
//
uint8_t ssd1306_Init(I2C_TypeDef *I2Cx);
void ssd1306_Fill(SSD1306_COLOR color);
void ssd1306_UpdateScreen(I2C_TypeDef *I2Cx);
void ssd1306_DrawPixel(uint8_t x, uint8_t y, SSD1306_COLOR color);
void ssd1306_SetCursor(uint8_t x, uint8_t y);
void ssd1306_DrawChar(char ch, const uint8_t font[], uint8_t X, uint8_t Y, SSD1306_COLOR color);
void ssd1306_DrawText(const char* str, const uint8_t font[], uint8_t X, uint8_t Y,  SSD1306_COLOR color);

void SSD1306_ScrollRight(I2C_TypeDef *I2Cx, uint8_t start_row, uint8_t end_row);
void SSD1306_ScrollLeft(I2C_TypeDef *I2Cx, uint8_t start_row, uint8_t end_row);
void SSD1306_Scrolldiagright(I2C_TypeDef *I2Cx, uint8_t start_row, uint8_t end_row);
void SSD1306_Scrolldiagleft(I2C_TypeDef *I2Cx, uint8_t start_row, uint8_t end_row);
void SSD1306_Stopscroll(I2C_TypeDef *I2Cx);

void SSD1306_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, SSD1306_COLOR c);
void SSD1306_DrawRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, SSD1306_COLOR c);
void SSD1306_DrawFilledRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, SSD1306_COLOR c) ;
void SSD1306_DrawTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, SSD1306_COLOR color);
void SSD1306_DrawFilledTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, SSD1306_COLOR color);
void SSD1306_DrawCircle(int16_t x0, int16_t y0, int16_t r, SSD1306_COLOR c);
void SSD1306_DrawFilledCircle(int16_t x0, int16_t y0, int16_t r, SSD1306_COLOR c);
void SSD1306_DrawBitmap(int16_t x, int16_t y, const unsigned char* bitmap, int16_t w, int16_t h, uint16_t color);

void SSD1306_InvertDisplay (I2C_TypeDef *I2Cx, int i);

void SSD1306_Clear (I2C_TypeDef *I2Cx);

void SSD1306_ON(I2C_TypeDef *I2Cx);
void SSD1306_OFF(I2C_TypeDef *I2Cx);

#endif
