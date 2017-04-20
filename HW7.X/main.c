/* 
 * File:   main.c
 * Author: kevin
 *
 * Created on April 19, 2017, 12:59 PM
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <xc.h>
#include "i2c_master_noint.h"
#include "ILI9163C.h"

// DEVCFG0
#pragma config DEBUG = OFF // no debugging
#pragma config JTAGEN = OFF // no jtag
#pragma config ICESEL = ICS_PGx1 // use PGED1 and PGEC1
#pragma config PWP = OFF // no write protect
#pragma config BWP = OFF // no boot write protect
#pragma config CP = OFF // no code protect

// DEVCFG1
#pragma config FNOSC = PRIPLL // use primary oscillator with pll
#pragma config FSOSCEN = OFF // turn off secondary oscillator
#pragma config IESO = OFF // no switching clocks
#pragma config POSCMOD = HS // high speed crystal mode
#pragma config OSCIOFNC = OFF // free up secondary osc pins
#pragma config FPBDIV = DIV_1 // divide CPU freq by 1 for peripheral bus clock
#pragma config FCKSM = CSDCMD // do not enable clock switch
#pragma config WDTPS = PS1048576 // slowest wdt
#pragma config WINDIS = OFF // no wdt window
#pragma config FWDTEN = OFF // wdt off by default
#pragma config FWDTWINSZ = WINSZ_25 // wdt window at 25%

// DEVCFG2 - get the CPU clock to 48MHz
#pragma config FPLLIDIV = DIV_2 // divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = MUL_24 // multiply clock after FPLLIDIV
#pragma config FPLLODIV = DIV_2 // divide clock after FPLLMUL to get 48MHz
#pragma config UPLLIDIV = DIV_2 // divider for the 8MHz input clock, then multiply by 12 to get 48MHz for USB
#pragma config UPLLEN = ON // USB clock on

// DEVCFG3
#pragma config USERID = 0 // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = OFF // allow multiple reconfigurations
#pragma config IOL1WAY = OFF // allow multiple reconfigurations
#pragma config FUSBIDIO = ON // USB pins controlled by USB module
#pragma config FVBUSONIO = ON // USB BUSON controlled by USB module

#define SLAVE_ADDR 0b1101011

void i2c_setup(void);
void initChip(void);
void I2C_read_multiple(unsigned char , unsigned char *, int);
void drawChar(unsigned short, unsigned short, unsigned short, unsigned char);
void drawString(unsigned short, unsigned short, unsigned short, unsigned char *);
void constructShorts(unsigned char *, int, signed short *);
/*
 * 
 */
int main(void) {
    i2c_setup();
    initChip();
    SPI1_init();
    LCD_init();
    LCD_clearScreen(MAGENTA);
    
    unsigned char gyroData[14], msg[100];
    signed short gyroShorts[7], temp, gyroX, gyroY, gyroZ, accelX, accelY, accelZ;
    
    while(1){
        _CP0_SET_COUNT(0);
        
        I2C_read_multiple(0x20,gyroData,14);
        constructShorts(gyroData,14,gyroShorts);
        temp=gyroShorts[0];
        gyroX=gyroShorts[1];
        gyroY=gyroShorts[2];
        gyroZ=gyroShorts[3];
        accelX=gyroShorts[4];
        accelY=gyroShorts[5];
        accelZ=gyroShorts[6];
        
        sprintf(msg,"temp: %d   ",temp);
        drawString(28,32,BLACK,msg);
        sprintf(msg,"gyroX: %d   ",gyroX);
        drawString(28,42,BLACK,msg);
        sprintf(msg,"gyroY: %d   ",gyroY);
        drawString(28,52,BLACK,msg);
        sprintf(msg,"gyroZ: %d   ",gyroZ);
        drawString(28,62,BLACK,msg);
        sprintf(msg,"accelX: %d   ",accelX);
        drawString(28,72,BLACK,msg);
        sprintf(msg,"accelY: %d   ",accelY);
        drawString(28,82,BLACK,msg);
        sprintf(msg,"accelZ: %d   ",accelZ);
        drawString(28,92,BLACK,msg);
        
        while(_CP0_GET_COUNT()<4799999){};
    };
    
    return 0;
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
