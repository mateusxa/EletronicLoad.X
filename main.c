/*
 * File:   main.c
 * Author: mateu
 *
 * Created on September 7, 2021, 7:11 PM
 */


#include "xc8.h"


void MCUinit(void);
void OSCinit(void);
void IOinit(void);
void INTinit(void);

void main(void) {
    
    MCUinit();
    
    while(1){
        
    }
            
    return;
}

void MCUinit(void){
    OSCinit();
    IOinit();
    INTinit();
}

void OSCinit(void){
    OSCCONbits.SCS = 0B10;
    OSCCONbits.IRCF = 0B1101;
    while(OSCSTATbits.HFIOFS != 1);
}

void IOinit(void){
    TRISAbits.TRISA5 = 1;
    TRISAbits.TRISA4 = 1;
    
    WPUAbits.WPUA5 = 1;
    WPUAbits.WPUA4 = 1;
    
    IOCANbits.IOCAN5 = 1;
    IOCANbits.IOCAN4 = 1;
}

void INTinit(void){
    INTCONbits.IOCIE = 1;
    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;
}