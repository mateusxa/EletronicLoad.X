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

void UARTinit(void);                // UART setup
void UART_write(char c);            // Write on UART
void UART_writeStr(char *data);     // Write string UART

void ADCinit(void);                 // ADC setup
int AnalogRead(void);          // 

/*
// ------------------------------------------
void I2C_Master_Init(const unsigned long c);
void I2C_Master_Wait();
void I2C_Master_Start();
void I2C_Master_RepeatedStart();
void I2C_Master_Stop();
void I2C_Master_Write(unsigned char d);
unsigned short I2C_Master_Read(unsigned short a);
*/

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
    */
      
    
    UART_writeStr("ABCD");
    UART_write(0x0A);
    UART_write(0x0D);
    
    AnalogRead();
    __delay_ms(1000);

    }  
    return;
}

void MCUinit(void){
    OSCinit();
    IOinit();
    ADCinit();
    //INTinit();
    UARTinit();
    //I2C_Master_Init(100000);      //Initialize I2C Master with 100KHz clock
}

void OSCinit(void){
    OSCCONbits.SCS = 0B10;              // 1x - Internal oscillator block
    OSCCONbits.IRCF = 0B1101;           // 1101 = 4 MHz HF
    while(OSCSTATbits.HFIOFS != 1);     // Wait for the clock to stabilize
}

void IOinit(void){
    TRISAbits.TRISA0 = 0;
    TRISAbits.TRISA1 = 1;
    /*
    WPUAbits.WPUA5 = 1;
    WPUAbits.WPUA4 = 1;
    
    IOCANbits.IOCAN5 = 1;
    IOCANbits.IOCAN4 = 1;
    */
}

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

int AnalogRead(void){
    int temp;
    
    __delay_us(5);              // Needed to wait 5 us for acquisition time
    ADCON0bits.GO_nDONE = 1;    // Start conversion
    while(ADCON0bits.GO_nDONE); // wait for conversion to finish
    temp = ADRES;
    
    return temp;
}

void INTinit(void){
    /*
    INTCONbits.IOCIE = 1;               
    INTCONbits.PEIE = 1;                
    INTCONbits.GIE = 1;    
     * */             
}
     
void UARTinit(void){
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
/*
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
*/