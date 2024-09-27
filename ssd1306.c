#include "ssd1306.h"

#define ABS(x)   ((x) > 0 ? (x) : -(x))

// Screenbuffer
static uint8_t SSD1306_Buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];

// Screen object
static SSD1306_t SSD1306;

//
//  Send a byte to the command register
//
static void ssd1306_WriteCommand(I2C_TypeDef *I2Cx, uint8_t command)
{
	I2C_WriteData(I2Cx, SSD1306_I2C_ADDR, 0x00, command);
}
//
//	Initialize the oled screen
//
uint8_t ssd1306_Init(I2C_TypeDef *I2Cx)
{	
	// Wait for the screen to boot
	LL_mDelay(100);
	
	/* Init LCD */
	ssd1306_WriteCommand(I2Cx, 0xAE); //display off
	ssd1306_WriteCommand(I2Cx, 0x20); //Set Memory Addressing Mode
	ssd1306_WriteCommand(I2Cx, 0x10); //00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
	ssd1306_WriteCommand(I2Cx, 0xB0); //Set Page Start Address for Page Addressing Mode,0-7
	ssd1306_WriteCommand(I2Cx, 0xC8); //Set COM Output Scan Direction
	ssd1306_WriteCommand(I2Cx, 0x00); //---set low column address
	ssd1306_WriteCommand(I2Cx, 0x10); //---set high column address
	ssd1306_WriteCommand(I2Cx, 0x40); //--set start line address
	ssd1306_WriteCommand(I2Cx, 0x81); //--set contrast control register
	ssd1306_WriteCommand(I2Cx, 0xFF);
	ssd1306_WriteCommand(I2Cx, 0xA1); //--set segment re-map 0 to 127
	ssd1306_WriteCommand(I2Cx, 0xA6); //--set normal display
	ssd1306_WriteCommand(I2Cx, 0xA8); //--set multiplex ratio(1 to 64)
	ssd1306_WriteCommand(I2Cx, 0x3F); //--- height 64
	//ssd1306_WriteCommand(I2Cx, 0x1F); // --- height 32
	ssd1306_WriteCommand(I2Cx, 0xA4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
	ssd1306_WriteCommand(I2Cx, 0xD3); //-set display offset
	ssd1306_WriteCommand(I2Cx, 0x00); //-not offset
	ssd1306_WriteCommand(I2Cx, 0xD5); //--set display clock divide ratio/oscillator frequency
	ssd1306_WriteCommand(I2Cx, 0xF0); //--set divide ratio
	ssd1306_WriteCommand(I2Cx, 0xD9); //--set pre-charge period
	ssd1306_WriteCommand(I2Cx, 0x22); //
	ssd1306_WriteCommand(I2Cx, 0xDA); //--set com pins hardware configuration
	ssd1306_WriteCommand(I2Cx, 0x12); // -- height 64
	//ssd1306_WriteCommand(I2Cx, 0x02); // -- height 32
	ssd1306_WriteCommand(I2Cx, 0xDB); //--set vcomh
	ssd1306_WriteCommand(I2Cx, 0x20); //0x20,0.77xVcc
	ssd1306_WriteCommand(I2Cx, 0x8D); //--set DC-DC enable
	ssd1306_WriteCommand(I2Cx, 0x14); //
	ssd1306_WriteCommand(I2Cx, 0xAF); //--turn on SSD1306 panel

	// Clear screen
	ssd1306_Fill(Black);

	// Flush buffer to screen
	ssd1306_UpdateScreen(I2Cx);

	// Set default values for screen object
	SSD1306.CurrentX = 0;
	SSD1306.CurrentY = 0;

	SSD1306.Initialized = 1;
	
	return 1;
}

//
//  Fill the whole screen with the given color
//
void ssd1306_Fill(SSD1306_COLOR color) 
{
	/* Set memory */
	uint32_t i;

	for(i = 0; i < sizeof(SSD1306_Buffer); i++)
	{
		SSD1306_Buffer[i] = (color == Black) ? 0x00 : 0xFF;
	}
}

//
//  Write the screenbuffer with changed to the screen
//
void ssd1306_UpdateScreen(I2C_TypeDef *I2Cx) 
{
	uint8_t i;
	
	for (i = 0; i < 8; i++) {
		ssd1306_WriteCommand(I2Cx, 0xB0 + i);
		ssd1306_WriteCommand(I2Cx, 0x00);
		ssd1306_WriteCommand(I2Cx, 0x10);
		I2C_MultyWriteData(I2Cx, SSD1306_I2C_ADDR, 0x40, &SSD1306_Buffer[SSD1306_WIDTH * i], SSD1306_WIDTH);
	}
}

//
//	Draw one pixel in the screenbuffer
//	X => X Coordinate
//	Y => Y Coordinate
//	color => Pixel color
//
void ssd1306_DrawPixel(uint8_t x, uint8_t y, SSD1306_COLOR color)
{
	if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) 
	{
		// Don't write outside the buffer
		return;
	}
	
	// Check if pixel should be inverted
	if (SSD1306.Inverted) 
	{
		color = (SSD1306_COLOR)!color;
	}
	
	// Draw in the right color
	if (color == White)
	{
		SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] |= 1 << (y % 8);
	} 
	else 
	{
		SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y % 8));
	}
}

//
//  Draw 1 char to the screen buffer
//	ch 		=> char om weg te schrijven
//	Font 	=> Font waarmee we gaan schrijven
//	color 	=> Black or White
//


//
//	Position the cursor
//
void ssd1306_SetCursor(uint8_t x, uint8_t y) 
{
	SSD1306.CurrentX = x;
	SSD1306.CurrentY = y;
}

void ssd1306_DrawChar(char ch, const uint8_t font[], uint8_t X, uint8_t Y, SSD1306_COLOR color)
{
    if ((ch < 31) || (ch > 127))
        return;

    uint8_t fOffset, fWidth, fHeight, fBPL;
    uint8_t* tempChar;

    fOffset = font[0];
    fWidth = font[1];
    fHeight = font[2];
    fBPL = font[3];

    tempChar = (uint8_t*)&font[((ch - 0x20) * fOffset) + 4]; /* Current Character = Meta + (Character Index * Offset) */


    for (int j = 0; j < fHeight; j++) {
        for (int i = 0; i < fWidth; i++) {
            uint8_t z = tempChar[fBPL * i + ((j & 0xF8) >> 3) + 1]; /* (j & 0xF8) >> 3, increase one by 8-bits */
            uint8_t b = 1 << (j & 0x07);
            if ((z & b) != 0x00)
            	ssd1306_DrawPixel(X + i, Y + j, (SSD1306_COLOR) color);
            else
            	ssd1306_DrawPixel(X + i, Y + j, (SSD1306_COLOR) !color);

        }
    }
}

void ssd1306_DrawText(const char* str, const uint8_t font[], uint8_t X, uint8_t Y,  SSD1306_COLOR color)
{
    uint8_t charWidth; /* Width of character */
    uint8_t fOffset = font[0]; /* Offset of character */
    uint8_t fWidth = font[1]; /* Width of font */

    while (*str) {
        ssd1306_DrawChar(*str, font,  X, Y,  color);
        /* Check character width and calculate proper position */
        uint8_t* tempChar = (uint8_t*)&font[((*str - 0x20) * fOffset) + 4];
        charWidth = tempChar[0];

        if (charWidth + 2 < fWidth) {
            /* If character width is smaller than font width */
            X += (charWidth + 2);
        }
        else {
            X += fWidth;
        }
        str++;
    }
}





static SSD1306_t SSD1306;


#define SSD1306_RIGHT_HORIZONTAL_SCROLL              0x26
#define SSD1306_LEFT_HORIZONTAL_SCROLL               0x27
#define SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL 0x29
#define SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL  0x2A
#define SSD1306_DEACTIVATE_SCROLL                    0x2E // Stop scroll
#define SSD1306_ACTIVATE_SCROLL                      0x2F // Start scroll
#define SSD1306_SET_VERTICAL_SCROLL_AREA             0xA3 // Set scroll range

#define SSD1306_NORMALDISPLAY       0xA6
#define SSD1306_INVERTDISPLAY       0xA7


void SSD1306_ScrollRight(I2C_TypeDef *I2Cx, uint8_t start_row, uint8_t end_row)
{
  ssd1306_WriteCommand(I2Cx,  SSD1306_RIGHT_HORIZONTAL_SCROLL);  // send 0x26
  ssd1306_WriteCommand(I2Cx,  0x00);  // send dummy
  ssd1306_WriteCommand(I2Cx,  start_row);  // start page address
  ssd1306_WriteCommand(I2Cx,  0X00);  // time interval 5 frames
  ssd1306_WriteCommand(I2Cx,  end_row);  // end page address
  ssd1306_WriteCommand(I2Cx,  0X00);
  ssd1306_WriteCommand(I2Cx,  0XFF);
  ssd1306_WriteCommand(I2Cx,  SSD1306_ACTIVATE_SCROLL); // start scroll
}


void SSD1306_ScrollLeft(I2C_TypeDef *I2Cx, uint8_t start_row, uint8_t end_row)
{
  ssd1306_WriteCommand(I2Cx, SSD1306_LEFT_HORIZONTAL_SCROLL);  // send 0x26
  ssd1306_WriteCommand(I2Cx, 0x00);  // send dummy
  ssd1306_WriteCommand(I2Cx, start_row);  // start page address
  ssd1306_WriteCommand(I2Cx, 0X00);  // time interval 5 frames
  ssd1306_WriteCommand(I2Cx, end_row);  // end page address
  ssd1306_WriteCommand(I2Cx, 0X00);
  ssd1306_WriteCommand(I2Cx, 0XFF);
  ssd1306_WriteCommand(I2Cx, SSD1306_ACTIVATE_SCROLL); // start scroll
}


void SSD1306_Scrolldiagright(I2C_TypeDef *I2Cx, uint8_t start_row, uint8_t end_row)
{
  ssd1306_WriteCommand(I2Cx, SSD1306_SET_VERTICAL_SCROLL_AREA);  // sect the area
  ssd1306_WriteCommand(I2Cx, 0x00);   // write dummy
  ssd1306_WriteCommand(I2Cx, SSD1306_HEIGHT);

  ssd1306_WriteCommand(I2Cx, SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL);
  ssd1306_WriteCommand(I2Cx, 0x00);
  ssd1306_WriteCommand(I2Cx, start_row);
  ssd1306_WriteCommand(I2Cx, 0X00);
  ssd1306_WriteCommand(I2Cx, end_row);
  ssd1306_WriteCommand(I2Cx, 0x01);
  ssd1306_WriteCommand(I2Cx, SSD1306_ACTIVATE_SCROLL);
}


void SSD1306_Scrolldiagleft(I2C_TypeDef *I2Cx, uint8_t start_row, uint8_t end_row)
{
  ssd1306_WriteCommand(I2Cx, SSD1306_SET_VERTICAL_SCROLL_AREA);  // sect the area
  ssd1306_WriteCommand(I2Cx, 0x00);   // write dummy
  ssd1306_WriteCommand(I2Cx, SSD1306_HEIGHT);

  ssd1306_WriteCommand(I2Cx, SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL);
  ssd1306_WriteCommand(I2Cx, 0x00);
  ssd1306_WriteCommand(I2Cx, start_row);
  ssd1306_WriteCommand(I2Cx, 0X00);
  ssd1306_WriteCommand(I2Cx, end_row);
  ssd1306_WriteCommand(I2Cx, 0x01);
  ssd1306_WriteCommand(I2Cx, SSD1306_ACTIVATE_SCROLL);
}


void SSD1306_Stopscroll(I2C_TypeDef *I2Cx)
{
	ssd1306_WriteCommand(I2Cx, SSD1306_DEACTIVATE_SCROLL);
}



void SSD1306_InvertDisplay (I2C_TypeDef *I2Cx, int i)
{
  if (i) ssd1306_WriteCommand(I2Cx, SSD1306_INVERTDISPLAY);

  else   ssd1306_WriteCommand(I2Cx, SSD1306_NORMALDISPLAY);

}

void SSD1306_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, SSD1306_COLOR c) {
	int16_t dx, dy, sx, sy, err, e2, i, tmp; 
	
	/* Check for overflow */
	if (x0 >= SSD1306_WIDTH) {
		x0 = SSD1306_WIDTH - 1;
	}
	if (x1 >= SSD1306_WIDTH) {
		x1 = SSD1306_WIDTH - 1;
	}
	if (y0 >= SSD1306_HEIGHT) {
		y0 = SSD1306_HEIGHT - 1;
	}
	if (y1 >= SSD1306_HEIGHT) {
		y1 = SSD1306_HEIGHT - 1;
	}
	
	dx = (x0 < x1) ? (x1 - x0) : (x0 - x1); 
	dy = (y0 < y1) ? (y1 - y0) : (y0 - y1); 
	sx = (x0 < x1) ? 1 : -1; 
	sy = (y0 < y1) ? 1 : -1; 
	err = ((dx > dy) ? dx : -dy) / 2; 

	if (dx == 0) {
		if (y1 < y0) {
			tmp = y1;
			y1 = y0;
			y0 = tmp;
		}
		
		if (x1 < x0) {
			tmp = x1;
			x1 = x0;
			x0 = tmp;
		}
		
		/* Vertical line */
		for (i = y0; i <= y1; i++) {
			ssd1306_DrawPixel(x0, i, c);
		}
		
		/* Return from function */
		return;
	}
	
	if (dy == 0) {
		if (y1 < y0) {
			tmp = y1;
			y1 = y0;
			y0 = tmp;
		}
		
		if (x1 < x0) {
			tmp = x1;
			x1 = x0;
			x0 = tmp;
		}
		
		/* Horizontal line */
		for (i = x0; i <= x1; i++) {
			ssd1306_DrawPixel(i, y0, c);
		}
		
		/* Return from function */
		return;
	}
	
	while (1) {
		ssd1306_DrawPixel(x0, y0, c);
		if (x0 == x1 && y0 == y1) {
			break;
		}
		e2 = err; 
		if (e2 > -dx) {
			err -= dy;
			x0 += sx;
		} 
		if (e2 < dy) {
			err += dx;
			y0 += sy;
		} 
	}
}

void SSD1306_DrawRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, SSD1306_COLOR c) {
	/* Check input parameters */
	if (
		x >= SSD1306_WIDTH ||
		y >= SSD1306_HEIGHT
	) {
		/* Return error */
		return;
	}
	
	/* Check width and height */
	if ((x + w) >= SSD1306_WIDTH) {
		w = SSD1306_WIDTH - x;
	}
	if ((y + h) >= SSD1306_HEIGHT) {
		h = SSD1306_HEIGHT - y;
	}
	
	/* Draw 4 lines */
	SSD1306_DrawLine(x, y, x + w, y, c);         /* Top line */
	SSD1306_DrawLine(x, y + h, x + w, y + h, c); /* Bottom line */
	SSD1306_DrawLine(x, y, x, y + h, c);         /* Left line */
	SSD1306_DrawLine(x + w, y, x + w, y + h, c); /* Right line */
}

void SSD1306_DrawFilledRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, SSD1306_COLOR c) {
	uint8_t i;
	
	/* Check input parameters */
	if (
		x >= SSD1306_WIDTH ||
		y >= SSD1306_HEIGHT
	) {
		/* Return error */
		return;
	}
	
	/* Check width and height */
	if ((x + w) >= SSD1306_WIDTH) {
		w = SSD1306_WIDTH - x;
	}
	if ((y + h) >= SSD1306_HEIGHT) {
		h = SSD1306_HEIGHT - y;
	}
	
	/* Draw lines */
	for (i = 0; i <= h; i++) {
		/* Draw lines */
		SSD1306_DrawLine(x, y + i, x + w, y + i, c);
	}
}

void SSD1306_DrawTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, SSD1306_COLOR color) {
	/* Draw lines */
	SSD1306_DrawLine(x1, y1, x2, y2, color);
	SSD1306_DrawLine(x2, y2, x3, y3, color);
	SSD1306_DrawLine(x3, y3, x1, y1, color);
}


void SSD1306_DrawFilledTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, SSD1306_COLOR color) {
	int16_t deltax = 0, deltay = 0, x = 0, y = 0, xinc1 = 0, xinc2 = 0, 
	yinc1 = 0, yinc2 = 0, den = 0, num = 0, numadd = 0, numpixels = 0, 
	curpixel = 0;
	
	deltax = ABS(x2 - x1);
	deltay = ABS(y2 - y1);
	x = x1;
	y = y1;

	if (x2 >= x1) {
		xinc1 = 1;
		xinc2 = 1;
	} else {
		xinc1 = -1;
		xinc2 = -1;
	}

	if (y2 >= y1) {
		yinc1 = 1;
		yinc2 = 1;
	} else {
		yinc1 = -1;
		yinc2 = -1;
	}

	if (deltax >= deltay){
		xinc1 = 0;
		yinc2 = 0;
		den = deltax;
		num = deltax / 2;
		numadd = deltay;
		numpixels = deltax;
	} else {
		xinc2 = 0;
		yinc1 = 0;
		den = deltay;
		num = deltay / 2;
		numadd = deltax;
		numpixels = deltay;
	}

	for (curpixel = 0; curpixel <= numpixels; curpixel++) {
		SSD1306_DrawLine(x, y, x3, y3, color);

		num += numadd;
		if (num >= den) {
			num -= den;
			x += xinc1;
			y += yinc1;
		}
		x += xinc2;
		y += yinc2;
	}
}

void SSD1306_DrawCircle(int16_t x0, int16_t y0, int16_t r, SSD1306_COLOR c) {
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

    ssd1306_DrawPixel(x0, y0 + r, c);
    ssd1306_DrawPixel(x0, y0 - r, c);
    ssd1306_DrawPixel(x0 + r, y0, c);
    ssd1306_DrawPixel(x0 - r, y0, c);

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        ssd1306_DrawPixel(x0 + x, y0 + y, c);
        ssd1306_DrawPixel(x0 - x, y0 + y, c);
        ssd1306_DrawPixel(x0 + x, y0 - y, c);
        ssd1306_DrawPixel(x0 - x, y0 - y, c);

        ssd1306_DrawPixel(x0 + y, y0 + x, c);
        ssd1306_DrawPixel(x0 - y, y0 + x, c);
        ssd1306_DrawPixel(x0 + y, y0 - x, c);
        ssd1306_DrawPixel(x0 - y, y0 - x, c);
    }
}

void SSD1306_DrawFilledCircle(int16_t x0, int16_t y0, int16_t r, SSD1306_COLOR c) {
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

    ssd1306_DrawPixel(x0, y0 + r, c);
    ssd1306_DrawPixel(x0, y0 - r, c);
    ssd1306_DrawPixel(x0 + r, y0, c);
    ssd1306_DrawPixel(x0 - r, y0, c);
    SSD1306_DrawLine(x0 - r, y0, x0 + r, y0, c);

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        SSD1306_DrawLine(x0 - x, y0 + y, x0 + x, y0 + y, c);
        SSD1306_DrawLine(x0 + x, y0 - y, x0 - x, y0 - y, c);

        SSD1306_DrawLine(x0 + y, y0 + x, x0 - y, y0 + x, c);
        SSD1306_DrawLine(x0 + y, y0 - x, x0 - y, y0 - x, c);
    }
}
 

void SSD1306_DrawBitmap(int16_t x, int16_t y, const unsigned char* bitmap, int16_t w, int16_t h, uint16_t color)
{

    int16_t byteWidth = (w + 7) / 8; // Bitmap scanline pad = whole byte
    uint8_t byte = 0;

    for(int16_t j=0; j<h; j++, y++)
    {
        for(int16_t i=0; i<w; i++)
        {
            if(i & 7)
            {
               byte <<= 1;
            }
            else
            {
               byte = (*(const unsigned char *)(&bitmap[j * byteWidth + i / 8]));
            }
            if(byte & 0x80) ssd1306_DrawPixel(x+i, y, color);
        }
    }
}


void SSD1306_Clear (I2C_TypeDef *I2Cx)
{
	ssd1306_Fill (0);
  ssd1306_UpdateScreen(I2Cx);
}

void SSD1306_ON(I2C_TypeDef *I2Cx) 
{
	ssd1306_WriteCommand(I2Cx, 0x8D);  
	ssd1306_WriteCommand(I2Cx, 0x14);  
	ssd1306_WriteCommand(I2Cx, 0xAF);  
}

void SSD1306_OFF(I2C_TypeDef *I2Cx) 
{
	ssd1306_WriteCommand(I2Cx, 0x8D);  
	ssd1306_WriteCommand(I2Cx, 0x10);
	ssd1306_WriteCommand(I2Cx, 0xAE);  
}


