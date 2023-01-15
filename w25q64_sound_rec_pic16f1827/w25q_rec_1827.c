//*  w25q64 flash memory recorder    *
// MPLAB X v3.10 XC8 v1.45. 
// This code is free to use, for more info   http://www.moty22.co.uk
// pic16f1827       
   
#include <htc.h>

#pragma config WDTE=OFF, PWRTE=OFF, MCLRE=OFF, BOREN=OFF, FOSC=HS, CP=OFF, CPD=OFF, LVP=OFF, WRT = OFF, PLLEN = OFF

#define _XTAL_FREQ 16000000
#define recLED RA4    //
#define CS   RA1 // 
#define DO   RB2   // AO
#define DI  RA0
#define CLK  RB5
#define stop  RB0
#define play  RB1
#define rec  RB4
#define erase  RA6
#define eraseLED  RA3

//prototypes
unsigned char spi(unsigned char spidata);
void rec_chip();
void playBack();
void write_en();
void isBusy();
void address(unsigned long adrs);

unsigned char c=0;

void main(void)
{
	
    // PIC I/O init
    OSCCON = 0b1111010; //internal 16Mhz
    //C1ON=0; C2ON=0;		//comparator off
    ANSELA=0b100;   //A2 analog   
    ANSELB=0;    
	TRISA = 0b11100100;		//
	TRISB = 0b11010111;   	//
        //pullup
    OPTION_REG = 0b1111111;
    WPUB = 0b1010011;
    
	//analogue init
	CCP1CON = 0B1100;	//PWM mode
	PR2 = 100;	//20KHz
	T2CON = 0B100;	//prescale 1, post scale 1, timer2 on
    ADCON1 = 0B10100000;		// right just, Fosc/32, ref=Vdd, . 
	ADCON0 = 0b1001;	//  AN2, adc ON
	recLED = 0;
    eraseLED=0;
    
        //SPI init
    //SSP2STAT = 0B11000000;	//SMP, CKE
    SSP2CON1 = 0B110001;	//SPI for TFT: full speed, enabled,clock idle=H
    SSP2STATbits.CKE=1;	//Data transmitted on rising edge of SCK
    SSP2STATbits.SMP=1;
    CS=1;
    
    __delay_ms(500);
    
	while(1){
        if(!rec){ rec_chip();__delay_ms(500);}    //record

        if(!play){playBack();__delay_ms(500);}    //play
	}
}

void rec_chip(){
  unsigned char out;  //lbyte, j, 
  unsigned long addr=0;
  unsigned int i;
        //erase chip
    eraseLED=1;
    write_en();
    CS=0;
    spi(0xC7); //erase chip
    CS=1;
    isBusy();
    eraseLED=0;
        //record
    recLED=1;
    while(stop){ //A2
        write_en();
        CS=0;
        spi(0x02); //write
        address(addr);
        for(i=0;i<256;i++){
            ADCON0bits.GO = 1;
            while(ADCON0bits.GO);
            __delay_us(75);
            out=ADRESL;
				//play while record
			CCPR1L = (out >> 1);
			spi(out);		//send analogue byte  
            
        }
        CS=1; 
        isBusy();      
        addr=addr+256;
    }
    recLED=0;
}

void playBack(){
    unsigned char adata;
    
    CS=0;
    spi(0x03); //read
    address(0);
    adata = spi(0xFF);
	CCPR1L = (adata >> 1);    
    while(stop){
        __delay_us(100);        
        adata = spi(0xFF);
        CCPR1L = (adata >> 1);
    }
    CS=1;
}

void write_en(){
    CS=0;
    spi(0x06);
    CS=1;
    isBusy();
}

void isBusy(){
  unsigned char busy=1;
    while(busy){ 
      CS=0;
      spi(0x05);  //stat1
      c=spi(0xFF);
      CS=1;
      busy = c & 1; 
    } 
}

void address(unsigned long adrs)
{
  spi((adrs & 0x00FF0000) >> 16);
  spi((adrs & 0x0000FF00) >> 8);
  spi(adrs & 0x000000FF);

}

unsigned char spi(unsigned char spidata)		// send character over SPI
{
	SSP2BUF = spidata;			// load character
	while (!SSP2STATbits.BF);		// sent
	return SSP2BUF;		// received character
}

