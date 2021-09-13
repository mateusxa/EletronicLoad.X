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
void I2C_Master_Init(const unsigned long c);
void I2C_Master_Wait();
void I2C_Master_Start();
void I2C_Master_RepeatedStart();
void I2C_Master_Stop();
void I2C_Master_Write(unsigned char d);
unsigned short I2C_Master_Read(unsigned short a);


void main(void) {
    
    MCUinit();
    

  while(1)
  {
    
      /*
      I2C_Master_Start();         //Start condition
    I2C_Master_Write(0x30);     //7 bit address + Write
    I2C_Master_Write(PORTB);    //Write data
    I2C_Master_Stop();          //Stop condition
    __delay_ms(200);
    I2C_Master_Start();         //Start condition
    I2C_Master_Write(0x31);     //7 bit address + Read
    PORTD = I2C_Master_Read(0); //Read + Acknowledge
    I2C_Master_Stop();          //Stop condition
    __delay_ms(200);
  */
    }  
    return;
}

void MCUinit(void){
    OSCinit();
    IOinit();
    INTinit();
    I2C_Master_Init(100000);      //Initialize I2C Master with 100KHz clock
}

void OSCinit(void){
    OSCCONbits.SCS = 0B10;              // 1x - Internal oscillator block
    OSCCONbits.IRCF = 0B1101;           // 1101 = 4 MHz HF
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

// Initialize I2C Module as Master
void I2C_Master_Init(const unsigned long c)
{
  SSPCON = 0b00101000;            //SSP Module as Master
  SSPCON2 = 0;
  SSPADD = 0x09;                    //Clock 4 MHz - 100 kHz
  SSPSTAT = 0;
  TRISA1 = 1;                   //Setting as input as given in datasheet
  TRISA2 = 1;                   //Setting as input as given in datasheet
}

// For Waiting
void I2C_Master_Wait()
{
  while ((SSPSTAT & 0x04) || (SSPCON2 & 0x1F)); //Transmit is in progress
}

// Start Condition
void I2C_Master_Start()
{
  I2C_Master_Wait();    
  SEN = 1;             //Initiate start condition
}

// Repeated Start
void I2C_Master_RepeatedStart()
{
  I2C_Master_Wait();
  RSEN = 1;           //Initiate repeated start condition
}

// Stop Condition
void I2C_Master_Stop()
{
  I2C_Master_Wait();
  PEN = 1;           //Initiate stop condition
}

// Write Data
void I2C_Master_Write(unsigned char d)
{
  I2C_Master_Wait();
  SSPBUF = d;         //Write data to SSPBUF
}

// Read Data
unsigned short I2C_Master_Read(unsigned short a)
{
  unsigned short temp;
  I2C_Master_Wait();
  RCEN = 1;
  I2C_Master_Wait();
  temp = SSPBUF;      //Read data from SSPBUF
  I2C_Master_Wait();
  ACKDT = (a)?0:1;    //Acknowledge bit
  ACKEN = 1;          //Acknowledge sequence
  return temp;
}
