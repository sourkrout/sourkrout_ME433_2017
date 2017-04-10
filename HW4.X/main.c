/* 
 * File:   main.c
 * Author: kevin
 *
 * Created on April 9, 2017, 3:33 PM
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <xc.h>

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


#define CS LATAbits.LATA4

//static volatile unsigned char triWave[200];
static volatile float sineWave[100], triWave[200];

void makeSine(void);
void makeTri(void);
void setVoltage(unsigned char, unsigned char);
unsigned char spi_io(unsigned char);
void spi_init(void);

       

int main(void) {
    spi_init();
    makeSine();
    makeTri();
    _CP0_SET_COUNT(0);
    int i=0,j=0;

    while(1){
        if (_CP0_GET_COUNT()>=23999){
            unsigned char test2, test4;
            int test1, test3;
            test1=(int) (sineWave[i]);
            test2=(unsigned char) (test1);
            setVoltage(0,test2);
            //setVoltage(1,triWave[j]);
            test3=(int) (triWave[j]);
            test4=(unsigned char) (test3);
            setVoltage(1,test4);
            i++;
            j++;
            
            if (i==100){
                i=0;
            }
            
            if (j==200){
                j=0;
            }
            _CP0_SET_COUNT(0);
        }
    }
    return 0;
}

void makeSine (void){
    int i;
    
    for (i=0;i<100;i++){
        sineWave[i]=((1.65*sin(2*M_PI*i/100)+1.65)*255/3.3);
    }
}

void makeTri (void){
    int i;
    
    for (i=0;i<200;i++){
        triWave[i]=(i*256/200);
    }
}

void spi_init(void){

    
TRISAbits.TRISA4 = 0;
CS=1;

RPB8Rbits.RPB8R=0b0011;
SDI1Rbits.SDI1R=0b0000;

SPI1CON = 0;              
SPI1BUF;                  
SPI1BRG = 0x1;            
SPI1STATbits.SPIROV = 0;
SPI1CONbits.CKE=1;     
SPI1CONbits.MSTEN = 1;    
SPI1CONbits.ON = 1;

}

unsigned char spi_io(unsigned char o) {
  SPI1BUF = o;
  while(!SPI1STATbits.SPIRBF) { // wait to receive the byte
    ;
  }
  return SPI1BUF;
}

void setVoltage(unsigned char channel, unsigned char voltage){
    unsigned char data1, data2;
    channel = channel << 7;
    data1 = voltage >> 4;
    data2 = voltage << 4;
    data1 = channel | data1;
    data1 = data1 | 0b00110000;
    CS=0;
    spi_io(data1);
    spi_io(data2);
    CS=1;
}