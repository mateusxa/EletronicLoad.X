/*
 * File:   main.c
 * Author: mateu
 *
 * Created on September 7, 2021, 7:11 PM
 */


#include "xc8.h"

#define RT_CLK      RA5
#define RT_DT       RA4
#define RT_SW       RA3

void MCUinit(void);
void OSCinit(void);
void IOinit(void);
void INTinit(void);

void UARTinit(void);                // UART setup
void UART_write(char c);            // Write on UART
void UART_writeStr(char *data);     // Write string UART

void ADCinit(void);                 // ADC setup
unsigned short AnalogRead(void);          // 

void I2Cinit(void);
void I2C_start(void);
void I2C_stop(void);
void I2C_restart(void);
void I2C_write(uint8_t data);

void DAC_write(uint16_t data);


void main(void) {
    
    MCUinit();
    

    while(1)
    {
        
        __delay_ms(1000);

    }  
    return;
}

void __interrupt () Interruptions (void){
    if(INTCONbits.IOCIE && INTCONbits.IOCIF){
        if(!PORTAbits.RA3)
            DAC_write(0xFFF);
        
        if(!PORTAbits.RA5)
            DAC_write(0x000);
        
        INTCONbits.IOCIF = 0;       // Reset Flag
    }
}

//----------------------------------------------------------------------
// Main configuration function
void MCUinit(void){
    OSCinit();
    IOinit();
    ADCinit();
    //INTinit();
    UARTinit();
    I2Cinit();
    INTCONbits.GIE = 1;             // Enable General interrupts
}

//----------------------------------------------------------------------
// OSCILLATOR configuration function
void OSCinit(void){
    OSCCONbits.SCS = 0B10;              // 1x - Internal oscillator block
    OSCCONbits.IRCF = 0B1101;           // 1101 = 4 MHz HF
    while(OSCSTATbits.HFIOFS != 1);     // Wait for the clock to stabilize
}

//----------------------------------------------------------------------
// IO PORTS configuration function
void IOinit(void){

    // Data Direction register
    TRISAbits.TRISA3 = 1;       // Defining SW as input
    TRISAbits.TRISA4 = 1;       // Defining DT as input
    TRISAbits.TRISA5 = 1;       // Defining CLK as input
    
    // Interrupt on change
    IOCAFbits.IOCAF3 = 1;       // Enable Interrupt on SW
    IOCAFbits.IOCAF5 = 1;       // Enable Interrupt on CLK
    
    // Edge Trigger
    IOCANbits.IOCAN3 = 1;       // Set SW to trigger on positive edge
    IOCANbits.IOCAN5 = 1;       // Set CLK to trigger on positive edge
    
    // Interruption
    INTCONbits.PEIE = 1;        // Enable Peripheral Interrupt
    INTCONbits.IOCIE = 1;       // Enable Interrupt on change 
    
}

void INTinit(void){
    /*
    INTCONbits.IOCIE = 1;               
    INTCONbits.PEIE = 1;                
    INTCONbits.GIE = 1;    
     * */             
}

//----------------------------------------------------------------------
// ANALOG TO DIGITAL configuration function
void ADCinit(void){
    
    TRISAbits.TRISA4 = 1;       // Setting pin to input    
    ANSELAbits.ANSA4 = 1;       // Assigning Analog input to RA4
    ADCON0bits.CHS = 3;         // Select channel AN3
    
    FVRCONbits.FVREN = 1;       // Enable Fixed voltage reference
    FVRCONbits.ADFVR = 3;       // FVR output is 4.096V
    while(!FVRCONbits.FVRRDY);  // wait for FVR to bee ready
    
    ADCON1bits.ADPREF = 3;      // Select internal Fixed Volt Reference module
    ADCON1bits.ADCS = 4;        // Select TAD = 1 us
    ADCON0bits.ADON = 1;        // Enable ADC
}

//----------------------------------------------------------------------
// ANALOG read function
unsigned short AnalogRead(void){

    __delay_us(5);              // Needed to wait 5 us for acquisition time
    ADCON0bits.GO_nDONE = 1;    // Start conversion
    while(ADCON0bits.GO_nDONE); // wait for conversion to finish
    
    return ADRES;
}
     
void UARTinit(void){
    TRISAbits.TRISA0 = 0;   // TX set as output
    TXSTAbits.SYNC = 0;     // Choose Asynchronous mode
    TXSTAbits.BRGH = 1;     // Select High speed Baud Rate Generator
    BAUDCONbits.BRG16 = 0;  // Select 8 bit Baud Rate Generator
    SPBRG = 25;             // Baud Rate is 9600 with 0.16 error
    
    RCSTAbits.SPEN = 1;     // Enabling serial port
    TXSTAbits.TXEN = 1;     // Enabling Transmission(TX) 
}

void UART_write(char c){
    while(!TXSTAbits.TRMT);
    TXREG = c;
}

void UART_writeStr(char *data){
    while(*data){
       UART_write(*data++);
    }
}

void I2Cinit(void){
    TRISAbits.TRISA1 = 1;       // Set RA1 - SCL - as input
    TRISAbits.TRISA2 = 1;       // Set RA2 - SDA - as input
    
    SSP1CON1bits.SSPM = 0x08;   // Set I2C Master Mode, clock = Fosc/(4 * (SSP1ADD+1))
    SSP1ADD = 0x09;             // Set Baud Rate as 100 kHz
    SSP1CON1bits.SSPEN = 1;     // Enables the serial port and configure SDA and SCL    
}

void I2C_start(void){
    SSP1CON2bits.SEN = 1;       // Generate start bit
    while(!PIR1bits.SSP1IF);    // Wait for start bit to complete
    PIR1bits.SSP1IF = 0;        // Clear Flag
}

void I2C_stop(void){
    SSP1CON2bits.PEN = 1;       // Generate Stop bit
    while(!PIR1bits.SSP1IF);    // Wait for Stop bit to complete
    PIR1bits.SSP1IF = 0;        // Clear Flag
}

void I2C_restart(void){
    SSP1CON2bits.RSEN = 1;      // Generate ReStart bit
    while(!PIR1bits.SSP1IF);    // Wait for ReStart bit to complete
    PIR1bits.SSP1IF = 0;        // Clear Flag
}

void I2C_write(uint8_t data){
    SSP1BUF = data;             // Write data on Buffer to be transmitted
    while(!PIR1bits.SSP1IF);    // Wait for start bit to complete
    PIR1bits.SSP1IF = 0;        // Clear Flag
}

void DAC_write(uint16_t data){
    I2C_start();            //  Send Start bit
    I2C_write(0xC0);        //  Device Code (1100) - Address Bits (000) - Write (0)
    I2C_write(data >> 8);   //  00 - Fast Mode Command | 00 - Vcc | Last Nimble data
    I2C_write(0xFF & data); //  First and Second Nimble
    I2C_stop();             //  Send Stop bit
}