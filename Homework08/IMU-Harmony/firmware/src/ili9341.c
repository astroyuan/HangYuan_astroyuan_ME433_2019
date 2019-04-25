#include <xc.h>
#include "ili9341.h"

void LCD_init() {
    int time = 0;
    
    CS = 0; // CS
   
    LCD_command(ILI9341_SWRESET);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 7200000) {} // 300ms

    LCD_command(0xEF);
  	LCD_data(0x03);
	LCD_data(0x80);
	LCD_data(0x02);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(0xCF);
  	LCD_data(0x00);
	LCD_data(0xC1);
	LCD_data(0x30);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(0xED);
  	LCD_data(0x64);
	LCD_data(0x03);
	LCD_data(0x12);
    LCD_data(0x81);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(0xE8);
  	LCD_data(0x85);
	LCD_data(0x00);
	LCD_data(0x78);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(0xCB);
  	LCD_data(0x39);
	LCD_data(0x2C);
	LCD_data(0x00);
    LCD_data(0x34);
    LCD_data(0x02);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(0xF7);
  	LCD_data(0x20);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(0xEA);
  	LCD_data(0x00);
	LCD_data(0x00);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(ILI9341_PWCTR1);
  	LCD_data(0x23);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(ILI9341_PWCTR2);
  	LCD_data(0x10);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(ILI9341_VMCTR1 );
  	LCD_data(0x3e);
    LCD_data(0x28);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(ILI9341_VMCTR2);
  	LCD_data(0x86);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(ILI9341_MADCTL);
  	LCD_data(0x48);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
/*    
    LCD_command(ILI9341_VSCRSADD);
  	LCD_data(0x00);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
 */   
    LCD_command(ILI9341_PIXFMT);
  	LCD_data(0x55);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(ILI9341_FRMCTR1);
  	LCD_data(0x00);
    LCD_data(0x18);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command( ILI9341_DFUNCTR);
  	LCD_data(0x08);
    LCD_data(0x82);
    LCD_data(0x27);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(0xF2);
  	LCD_data(0); // 1
    LCD_data(0x00);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(ILI9341_GAMMASET);
  	LCD_data(0x01);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(ILI9341_GMCTRP1);
  	LCD_data(0x0F);
    LCD_data(0x31);
    LCD_data(0x2B);
    LCD_data(0x0C);
    LCD_data(0x0E);
    LCD_data(0x08);
    LCD_data(0x4E);
    LCD_data(0xF1);
    LCD_data(0x37);
    LCD_data(0x07);
    LCD_data(0x10);
    LCD_data(0x03);
    LCD_data(0x0E);
    LCD_data(0x09);
    LCD_data(0x00);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(ILI9341_GMCTRN1);
  	LCD_data(0x00);
    LCD_data(0x0E);
    LCD_data(0x14);
    LCD_data(0x03);
    LCD_data(0x11);
    LCD_data(0x07);
    LCD_data(0x31);
    LCD_data(0xC1);
    LCD_data(0x48);
    LCD_data(0x08);
    LCD_data(0x0F);
    LCD_data(0x0C);
    LCD_data(0x31);
    LCD_data(0x36);
    LCD_data(0x0F);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(0xB1);
  	LCD_data(0x00);
    LCD_data(0x10);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(ILI9341_SLPOUT);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(ILI9341_DISPON);
    
    CS = 1; // CS
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    
    CS = 0; // CS
    
    LCD_command(ILI9341_MADCTL);
    LCD_data(MADCTL_MX | MADCTL_BGR); // rotation
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    CS = 1; // CS
}

void SPI1_init() {
  SDI1Rbits.SDI1R = 0b0100; // B8 is SDI1
  RPA1Rbits.RPA1R = 0b0011; // A1 is SDO1
  TRISBbits.TRISB7 = 0; // CS is B7
  CS = 1; // CS starts high

  // DC pin
  TRISBbits.TRISB15 = 0;
  DC = 1;
  
  SPI1CON = 0; // turn off the spi module and reset it
  SPI1BUF; // clear the rx buffer by reading from it
  SPI1BRG = 0; // baud rate to 12 MHz [SPI1BRG = (48000000/(2*desired))-1]
  SPI1STATbits.SPIROV = 0; // clear the overflow bit
  SPI1CONbits.CKE = 1; // data changes when clock goes from hi to lo (since CKP is 0)
  SPI1CONbits.MSTEN = 1; // master operation
  SPI1CONbits.ON = 1; // turn on spi1
}

unsigned char spi_io(unsigned char o) {
  SPI1BUF = o;
  while(!SPI1STATbits.SPIRBF) { // wait to receive the byte
    ;
  }
  return SPI1BUF;
}

void LCD_command(unsigned char com) {
    DC = 0; // DC
    spi_io(com);
    DC = 1; // DC
}

void LCD_data(unsigned char dat) {
    spi_io(dat);
}

void LCD_data16(unsigned short dat) {
    spi_io(dat>>8);
    spi_io(dat);
}

void LCD_setAddr(unsigned short x, unsigned short y, unsigned short w, unsigned short h) {
    LCD_command(ILI9341_CASET); // Column
    LCD_data16(x);
	LCD_data16(x+w-1);

	LCD_command(ILI9341_PASET); // Page
	LCD_data16(y);
	LCD_data16(y+h-1);

	LCD_command(ILI9341_RAMWR); // Into RAM
}

void LCD_drawPixel(unsigned short x, unsigned short y, unsigned short color) {
  // check boundary
    if (LCD_checkBoundary(x,y) == -1) return;
    
    CS = 0; // CS
    
    LCD_setAddr(x,y,1,1);
    LCD_data16(color);
    
    CS = 1; // CS
}

void LCD_clearScreen(unsigned short color) {
    int i;
    
    CS = 0; // CS
    
    LCD_setAddr(0,0,ILI9341_TFTWIDTH,ILI9341_TFTHEIGHT);
	for (i = 0;i < ILI9341_TFTWIDTH*ILI9341_TFTHEIGHT; i++){
		LCD_data16(color);
	}
    
    CS = 1; // CS
}

void LCD_drawCharacter(char s, unsigned short x0, unsigned short y0, unsigned short fgcolor, unsigned short bgcolor)
{
    // draw a single ASCII letter
    int offset = 0x20;
    int idx = s - offset;
    
    // bitmap 8x5 bits
    // draw pixel by pixel
    int i,j;
    for (i = 0; i < 5; i++)
        for (j = 0; j < 8; j++)
        {
            if (ASCII[idx][i] & (1<<j))
                LCD_drawPixel(x0+i, y0+j, fgcolor);
            else
                LCD_drawPixel(x0+i, y0+j, bgcolor);
        }
}

int LCD_drawString(char* s, unsigned short x0, unsigned short y0, unsigned fgcolor, unsigned short bgcolor)
{
    // draw a string of ASCII letters
    int i=0;
    while (s[i])
    {
        LCD_drawCharacter(s[i], (x0+i*5)%ILI9341_TFTWIDTH, y0+((x0+i*5)/ILI9341_TFTWIDTH)*8, fgcolor, bgcolor);
        i++;
    }
    int x_next = (x0+i*5)%ILI9341_TFTWIDTH;
    int y_next = y0+((x0+i*5)/ILI9341_TFTWIDTH)*8;
    return x_next*ILI9341_TFTWIDTH + y_next;
}

int LCD_checkBoundary(unsigned short x, unsigned short y)
{
    // boundary limit checking
    if ((x > ILI9341_TFTWIDTH) | (y > ILI9341_TFTHEIGHT))
    {
        LCD_drawString("Error: Exceed display boundary!", 0, 0, ILI9341_RED, ILI9341_BLACK);
        return -1;
    }
    else
        return 1;
}

void LCD_drawProgressBar(unsigned short x0, unsigned short y0, unsigned short width, unsigned height, unsigned short progress, unsigned short fgcolor, unsigned short bgcolor)
{
    int i,j;
    for (i=0; i<width; i++)
        for (j=0; j<height; j++)
        {
            if (i <= progress)
                LCD_drawPixel(x0+i,y0+j,fgcolor);
            else
                LCD_drawPixel(x0+i,y0+j,bgcolor);
        }
}

void LCD_drawCross(unsigned short x0, unsigned short y0, unsigned short fullscale, short Xcomp, short Ycomp, unsigned short fgcolor, unsigned short bgcolor)
{
    // visualize a 2d vector x,y components with origin defined by (x0,y0)
    unsigned short xt,yt;
    unsigned short xl,xh,yl,yh;
    unsigned short incx, incy;
    unsigned short xc,yc;
    unsigned short thickness=2;
    
    xl=x0-fullscale/2;
    xh=x0+fullscale/2;
    yl=y0-fullscale/2;
    yh=y0+fullscale/2;
    
    xt=(unsigned short)((short)x0+Xcomp);
    yt=(unsigned short)((short)y0+Ycomp);
    
    int tmp;
    // x-component
    yc=y0;
    for(xc=xl;xc<xh;xc++)
    {
        if(Xcomp<0 && xc!=x0)
        {
            if((xt <= xc && xc <= x0))
            {
                LCD_drawPixel(xc,yc,fgcolor);
                for(tmp=1;tmp<=thickness;tmp++)
                {
                    LCD_drawPixel(xc,yc+tmp,fgcolor);
                    LCD_drawPixel(xc,yc-tmp,fgcolor);
                }
            }
            else
            {
                LCD_drawPixel(xc,yc,bgcolor);
                for(tmp=1;tmp<=thickness;tmp++)
                {
                    LCD_drawPixel(xc,yc+tmp,bgcolor);
                    LCD_drawPixel(xc,yc-tmp,bgcolor);
                }
            }
        }
        else if(Xcomp>0 && xc!=x0)
        {
            if(x0 <= xc && xc <= xt)
            {
                LCD_drawPixel(xc,yc,fgcolor);
                for(tmp=1;tmp<=thickness;tmp++)
                {
                    LCD_drawPixel(xc,yc+tmp,fgcolor);
                    LCD_drawPixel(xc,yc-tmp,fgcolor);
                }
            }
            else
            {
                LCD_drawPixel(xc,yc,bgcolor);
                for(tmp=1;tmp<=thickness;tmp++)
                {
                    LCD_drawPixel(xc,yc+tmp,bgcolor);
                    LCD_drawPixel(xc,yc-tmp,bgcolor);
                }
            }
        }
    }
    // y-component
    xc=x0;
    for(yc=yl;yc<yh;yc++)
    {
        if(Ycomp<0 && yc!=y0)
        {
            if(yt <= yc && yc <= y0)
            {
                LCD_drawPixel(xc,yc,fgcolor);
                for(tmp=1;tmp<=thickness;tmp++)
                {
                    LCD_drawPixel(xc+tmp,yc,fgcolor);
                    LCD_drawPixel(xc-tmp,yc,fgcolor);
                }
            }
            else
            {
                LCD_drawPixel(xc,yc,bgcolor);
                for(tmp=1;tmp<=thickness;tmp++)
                {
                    LCD_drawPixel(xc+tmp,yc,bgcolor);
                    LCD_drawPixel(xc-tmp,yc,bgcolor);
                }
            }
        }
        else if(Ycomp>0 && yc!=y0)
        {
            if(y0 <= yc && yc <= yt)
            {
                LCD_drawPixel(xc,yc,fgcolor);
                for(tmp=1;tmp<=thickness;tmp++)
                {
                    LCD_drawPixel(xc+tmp,yc,fgcolor);
                    LCD_drawPixel(xc-tmp,yc,fgcolor);
                }
            }
            else
            {
                LCD_drawPixel(xc,yc,bgcolor);
                for(tmp=1;tmp<=thickness;tmp++)
                {
                    LCD_drawPixel(xc+tmp,yc,bgcolor);
                    LCD_drawPixel(xc-tmp,yc,bgcolor);
                }
            }
        }
    }
    // origin
    LCD_drawPixel(x0,y0,fgcolor);
    for(tmp=1;tmp<=thickness;tmp++)
        LCD_drawPixel(x0+tmp,y0,fgcolor);
    for(tmp=1;tmp<=thickness;tmp++)
        LCD_drawPixel(x0,y0+tmp,fgcolor);
}