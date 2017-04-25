// functions to operate the ILI9163C on the PIC32
// adapted from https://github.com/sumotoy/TFT_ILI9163C/blob/master/TFT_ILI9163C.cpp

// pin connections:
// VCC - 3.3V
// GND - GND
// CS - B7
// RESET - 3.3V
// A0 - B15
// SDA - A1
// SCK - B14
// LED - 3.3V

// B8 is turned into SDI1 but is not used or connected to anything

#include <xc.h>
#include "ILI9163C.h"
#define SLAVE_ADDR 0b1101011

void SPI1_init() {
	SDI1Rbits.SDI1R = 0b0100; // B8 is SDI1
    RPA1Rbits.RPA1R = 0b0011; // A1 is SDO1
    TRISBbits.TRISB7 = 0; // SS is B7
    LATBbits.LATB7 = 1; // SS starts high

    // A0 / DAT pin
    ANSELBbits.ANSB15 = 0;
    TRISBbits.TRISB15 = 0;
    LATBbits.LATB15 = 0;
	
	SPI1CON = 0; // turn off the spi module and reset it
    SPI1BUF; // clear the rx buffer by reading from it
    SPI1BRG = 1; // baud rate to 12 MHz [SPI1BRG = (48000000/(2*desired))-1]
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
    LATBbits.LATB15 = 0; // DAT
    LATBbits.LATB7 = 0; // CS
    spi_io(com);
    LATBbits.LATB7 = 1; // CS
}

void LCD_data(unsigned char dat) {
    LATBbits.LATB15 = 1; // DAT
    LATBbits.LATB7 = 0; // CS
    spi_io(dat);
    LATBbits.LATB7 = 1; // CS
}

void LCD_data16(unsigned short dat) {
    LATBbits.LATB15 = 1; // DAT
    LATBbits.LATB7 = 0; // CS
    spi_io(dat>>8);
    spi_io(dat);
    LATBbits.LATB7 = 1; // CS
}

void LCD_init() {
    int time = 0;
    LCD_command(CMD_SWRESET);//software reset
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 48000000/2/2) {} //delay(500);

	LCD_command(CMD_SLPOUT);//exit sleep
    time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/200) {} //delay(5);

	LCD_command(CMD_PIXFMT);//Set Color Format 16bit
	LCD_data(0x05);
	time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/200) {} //delay(5);

	LCD_command(CMD_GAMMASET);//default gamma curve 3
	LCD_data(0x04);//0x04
	time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {} //delay(1);

	LCD_command(CMD_GAMRSEL);//Enable Gamma adj
	LCD_data(0x01);
	time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {} //delay(1);

	LCD_command(CMD_NORML);

	LCD_command(CMD_DFUNCTR);
	LCD_data(0b11111111);
	LCD_data(0b00000110);

    int i = 0;
	LCD_command(CMD_PGAMMAC);//Positive Gamma Correction Setting
	for (i=0;i<15;i++){
		LCD_data(pGammaSet[i]);
	}

	LCD_command(CMD_NGAMMAC);//Negative Gamma Correction Setting
	for (i=0;i<15;i++){
		LCD_data(nGammaSet[i]);
	}

	LCD_command(CMD_FRMCTR1);//Frame Rate Control (In normal mode/Full colors)
	LCD_data(0x08);//0x0C//0x08
	LCD_data(0x02);//0x14//0x08
	time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {} //delay(1);

	LCD_command(CMD_DINVCTR);//display inversion
	LCD_data(0x07);
	time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {} //delay(1);

	LCD_command(CMD_PWCTR1);//Set VRH1[4:0] & VC[2:0] for VCI1 & GVDD
	LCD_data(0x0A);//4.30 - 0x0A
	LCD_data(0x02);//0x05
	time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {} //delay(1);

	LCD_command(CMD_PWCTR2);//Set BT[2:0] for AVDD & VCL & VGH & VGL
	LCD_data(0x02);
	time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {} //delay(1);

	LCD_command(CMD_VCOMCTR1);//Set VMH[6:0] & VML[6:0] for VOMH & VCOML
	LCD_data(0x50);//0x50
	LCD_data(99);//0x5b
	time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {} //delay(1);

	LCD_command(CMD_VCOMOFFS);
	LCD_data(0);//0x40
	time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {} //delay(1);

	LCD_command(CMD_CLMADRS);//Set Column Address
	LCD_data16(0x00);
    LCD_data16(_GRAMWIDTH);

	LCD_command(CMD_PGEADRS);//Set Page Address
	LCD_data16(0x00);
    LCD_data16(_GRAMHEIGH);

	LCD_command(CMD_VSCLLDEF);
	LCD_data16(0); // __OFFSET
	LCD_data16(_GRAMHEIGH); // _GRAMHEIGH - __OFFSET
	LCD_data16(0);

	LCD_command(CMD_MADCTL); // rotation
    LCD_data(0b00001000); // bit 3 0 for RGB, 1 for GBR, rotation: 0b00001000, 0b01101000, 0b11001000, 0b10101000

	LCD_command(CMD_DISPON);//display ON
	time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {} //delay(1);

	LCD_command(CMD_RAMWR);//Memory Write
}

void LCD_drawPixel(unsigned short x, unsigned short y, unsigned short color) {
    // check boundary
    LCD_setAddr(x,y,x+1,y+1);
    LCD_data16(color);
}

void LCD_setAddr(unsigned short x0, unsigned short y0, unsigned short x1, unsigned short y1) {
    LCD_command(CMD_CLMADRS); // Column
    LCD_data16(x0);
	LCD_data16(x1);

	LCD_command(CMD_PGEADRS); // Page
	LCD_data16(y0);
	LCD_data16(y1);

	LCD_command(CMD_RAMWR); //Into RAM
}

void LCD_clearScreen(unsigned short color) {
    int i;
    LCD_setAddr(0,0,_GRAMWIDTH,_GRAMHEIGH);
		for (i = 0;i < _GRAMSIZE; i++){
			LCD_data16(color);
		}
}

void i2c_setup(void) {
  i2c_master_setup();
  ANSELBbits.ANSB2 = 0;
  ANSELBbits.ANSB3 = 0;
}

void initChip(void){
    i2c_master_start();
    i2c_master_send(SLAVE_ADDR << 1);
    i2c_master_send(0x10);
    i2c_master_send(0x82); 
    i2c_master_stop();
    
    i2c_master_start();
    i2c_master_send(SLAVE_ADDR << 1);
    i2c_master_send(0x11);
    i2c_master_send(0x88); 
    i2c_master_stop();
}

void I2C_read_multiple(unsigned char reg, unsigned char * data, int length){
    int i;
    
    i2c_master_start();
    i2c_master_send(SLAVE_ADDR << 1);
    i2c_master_send(reg);
    i2c_master_restart();
    i2c_master_send((SLAVE_ADDR << 1) | 1);
    
    for (i=0;i<(length-1);i++){
        *(data+i)=i2c_master_recv();
        i2c_master_ack(0);
    }
    *(data+(length-1))=i2c_master_recv();
    i2c_master_ack(1);
    i2c_master_stop();   
}

void drawChar(unsigned short x, unsigned short y, unsigned short color, unsigned char asc){
    int i,j;
    asc=asc-0x20;
    if (x+5<=128){
        if (y+8<=128){
            for (i=0;i<5;i++){
                for (j=0;j<8;j++){
                    if ((ASCII[asc][i]>>j)&0b1){
                        LCD_drawPixel(x+i,y+j,color);  
                    } else {
                        LCD_drawPixel(x+i,y+j,MAGENTA);
                    }
                }
            }
        }
    }
}

void drawString(unsigned short x, unsigned short y, unsigned short color, unsigned char * addr){
    int i=0;
    while(addr[i]){
        drawChar(x,y,BLACK,addr[i]);
        x=x+6;
        i++;
    }
}

void constructShorts(unsigned char * data, int length, signed short * shorts){
    int i, maxi;
    maxi=length/2;
    
    for (i=0;i<maxi;i++){
        signed short store1;
        store1=data[((2*i)+1)]<<8;
        store1=store1 | (data[(2*i)]);
        shorts[i]=store1;
    }
}

void drawXBar(signed short accelX){
    int i,j,barInt;
    float barLength, accelxf;
    
    accelxf=(float) accelX;
    barLength=40*(accelxf/16384);
    barInt=(int) barLength;
    
    //barL=barInt;
    
    if (barInt>0){
        for (j=0;j<barInt;j++){
        for (i=0;i<5;i++){
        LCD_drawPixel(64+j,64+i,CYAN);
        }
        }
        for (j=0;j<64;j++){
        for (i=0;i<5;i++){
        LCD_drawPixel(64-j,64+i,MAGENTA);
        }
        }
        for (j=barInt;j<64;j++){
        for (i=0;i<5;i++){
        LCD_drawPixel(64+j,64+i,MAGENTA);
        }
        }
    }
    
    if (barInt<0){
        barInt=-barInt;
        for (j=0;j<barInt;j++){
        for (i=0;i<5;i++){
        LCD_drawPixel(64-j,64+i,CYAN);
        }
        }
        for (j=0;j<64;j++){
        for (i=0;i<5;i++){
        LCD_drawPixel(64+j,64+i,MAGENTA);
        }
        }
        for (j=barInt;j<64;j++){
        for (i=0;i<5;i++){
        LCD_drawPixel(64-j,64+i,MAGENTA);
        }
        }
    }
    
}

void drawYBar(signed short accelY){
    int i,j,barInt;
    float barLength, accelyf;
    
    accelyf=(float) accelY;
    barLength=40*(accelyf/16384);
    barInt=(int) barLength;
    
    //barL2=barInt;
    
    if (barInt>0){
        for (j=0;j<barInt;j++){
        for (i=0;i<5;i++){
        LCD_drawPixel(64+i,64+j,GREEN);
        }
        }
        for (j=0;j<64;j++){
        for (i=0;i<5;i++){
        LCD_drawPixel(64+i,64-j,MAGENTA);
        }
        }
        for (j=barInt;j<64;j++){
        for (i=0;i<5;i++){
        LCD_drawPixel(64+i,64+j,MAGENTA);
        }
        }
    }
    
    if (barInt<0){
        barInt=-barInt;
        for (j=0;j<barInt;j++){
        for (i=0;i<5;i++){
        LCD_drawPixel(64+i,64-j,GREEN);
        }
        }
        for (j=0;j<64;j++){
        for (i=0;i<5;i++){
        LCD_drawPixel(64+i,64+j,MAGENTA);
        }
        }
        for (j=barInt;j<64;j++){
        for (i=0;i<5;i++){
        LCD_drawPixel(64+i,64-j,MAGENTA);
        }
        }
    }
}