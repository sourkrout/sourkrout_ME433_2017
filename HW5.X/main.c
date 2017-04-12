/* 
 * File:   main.c
 * Author: kevin
 *
 * Created on April 12, 2017, 10:32 AM
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <xc.h>
#include "i2c_master_noint.h"

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

#define SLAVE_ADDR 0x20

void initExpander(void);
void setExpander(unsigned char, unsigned char);
unsigned char getExpander(void);
/*
 * 
 */
int main(void) {
    
    ANSELBbits.ANSB2 = 0;
    ANSELBbits.ANSB3 = 0;
    i2c_master_setup();
    initExpander();
    
    while (1){
        
        if ((getExpander() & 0b10000000) == 0b00000000){
            setExpander(0,1);
        } else {
            setExpander(0,0);
        }
        
    }
    
    return 0;
}

void initExpander(void){
    i2c_master_start();
    i2c_master_send(SLAVE_ADDR << 1);
    i2c_master_send(0x00);
    i2c_master_send(0xF0); 
    i2c_master_stop();
    
    i2c_master_start();
    i2c_master_send(SLAVE_ADDR << 1);
    i2c_master_send(0x05);
    i2c_master_send(0x20);
    i2c_master_stop();
    
    i2c_master_start();
    i2c_master_send(SLAVE_ADDR << 1);
    i2c_master_send(0x06);
    i2c_master_send(0x80);
    i2c_master_stop();
}

void setExpander(unsigned char pin, unsigned char level){
    char val;
    val = level << pin;
    
    i2c_master_start();
    i2c_master_send(SLAVE_ADDR << 1);
    i2c_master_send(0x0A);
    i2c_master_send(val);
    i2c_master_stop();
}

unsigned char getExpander(void){
    unsigned char pinval;
    
    i2c_master_start();
    i2c_master_send(SLAVE_ADDR << 1);
    i2c_master_send(0x09);
    i2c_master_restart();
    i2c_master_send((SLAVE_ADDR << 1) | 1);
    pinval = i2c_master_recv();
    i2c_master_ack(1);
    i2c_master_stop();

    return pinval;
}
