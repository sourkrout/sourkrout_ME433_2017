/* 
 * File:   main.c
 * Author: kevin
 *
 * Created on April 18, 2017, 9:53 AM
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <xc.h>
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

void drawChar(unsigned short, unsigned short, unsigned short, unsigned char);
void drawString(unsigned short, unsigned short, unsigned short, unsigned char *);
void drawBar(unsigned short, unsigned short, int);
/*
 * 
 */
int main(void) {

    SPI1_init();
    LCD_init();
    
    LCD_clearScreen(MAGENTA);
    
    unsigned char msg[100];
    
    while(1){
        
        int i;
        for (i=0;i<101;i++){
        _CP0_SET_COUNT(0);
        sprintf(msg,"Hello world %d!",i);
        drawString(28,32,BLACK,msg);
        drawBar(28,42,i);
        while(_CP0_GET_COUNT()<4799999){};
        }
        LCD_clearScreen(MAGENTA);
    };
    
    return 0;
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

void drawBar(unsigned short x, unsigned short y, int barLength){
    int i;
    for (i=0;i<5;i++){
        LCD_drawPixel(x+barLength,y+i,CYAN);
    }
}