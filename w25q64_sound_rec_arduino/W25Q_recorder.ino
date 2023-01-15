/*
 * flash recorder
 *
 * Created: 12/01/2023 
 *  Author: moty22.co.uk
 */ 

#define cs 10          // CS
#define mosi 11
#define clk 13
#define miso 12          // miso 
#define RecLED  8          // led connected to digital pin PB0
#define eraseLED 9          // error led connected to digital pin PB1
#define Stop A2     //stop pushbutton C2
#define play A3   //play PB C3
#define rec A4      //record PB C4
#define erase A5    //
#define mic A0     //microphone

unsigned char c=0; //standard sd, n=50

void setup() {
Serial.begin(9600);
pinMode(cs, OUTPUT);
pinMode(mosi, OUTPUT);
pinMode(clk, OUTPUT);
pinMode(RecLED, OUTPUT);
pinMode(eraseLED, OUTPUT);
pinMode(Stop, INPUT_PULLUP);
pinMode(play, INPUT_PULLUP);
pinMode(rec, INPUT_PULLUP);
pinMode(erase, INPUT_PULLUP);
pinMode(miso, INPUT);          // set SPI to master
pinMode(mic, INPUT);

  //spi init
SPCR=0b01011101;  // Enable SPI, Master, MODE 3
SPSR = _BV(SPI2X);    //set clock rate fck/8, 2MHz

OCR2B = 64;
TCCR2A = _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);  //output in phase, fast PWM mode
TCCR2B = _BV(CS20); // 16MHz/256 = 64KHz
pinMode(3, OUTPUT);

   //ADC
  ADCSRA |= _BV(ADPS1) | _BV(ADPS0) | _BV(ADEN); //prescaler=8 500KHz, enabled   
  ADMUX=0b1000000;  //right justified, A0 input

delay(100);
digitalWrite(cs, HIGH);
}


void loop(){
  if(! digitalRead(A5)){          //erase
    digitalWrite(eraseLED, HIGH);
    write_en();
    digitalWrite(cs, LOW);
    spi(0xC7); //erase chip
    digitalWrite(cs, HIGH);
    isBusy();
    digitalWrite(eraseLED, LOW);
    
  }

  if(!digitalRead(A4)){ rec_chip();}    //record
 
  if(!digitalRead(A3)){playBack();}    //play

}

void rec_chip(){
  unsigned char lbyte, j, out;
  unsigned long addr=0;
  unsigned int i;
    digitalWrite(RecLED, HIGH);
    while(digitalRead(Stop)){ //A2
        write_en();
        digitalWrite(cs, LOW);
        spi(0x02); //write
        address(addr);
        for(i=0;i<256;i++){
            ADCSRA |=_BV(ADSC);   //start conversion  
            while ( !( ADCSRA & (1<<ADIF)) ){} // Wait for conversion to complete
            lbyte=ADCL;
            out=ADCH;
            OCR2B=lbyte;
            ADCSRA |=_BV(ADIF);   //reset interrupt          
            spi(lbyte);    //record analogue byte 
 
      }
        digitalWrite(cs, HIGH); 
        isBusy();      
        addr=addr+256;
    }
    digitalWrite(RecLED, LOW);
}

void playBack(){
    digitalWrite(cs, LOW);
    spi(0x3); //read
    address(0);
    OCR2B=spi(0xFF);
    while(digitalRead(Stop)){ //A2
      delayMicroseconds(100); //play speed
      OCR2B=spi(0xFF);
      //Serial.println(spi(0xFF));
    }
    digitalWrite(cs, HIGH); 
}

void write_en(){
  
    digitalWrite(cs, LOW);
    spi(0x06);
    digitalWrite(cs, HIGH);
    isBusy();
}

void isBusy(){
  unsigned char busy=1;
     while(busy){ 
      digitalWrite(cs, LOW);
      spi(0x05);  //stat1
      c=spi(0xFF);
      digitalWrite(cs, HIGH);
      busy = c & 1; 
    } 
}

unsigned char spi(unsigned char data)   // send character over spi
{
  SPDR = data;  // Start transmission
  while(!(SPSR & _BV(SPIF)));   // Wait for transmission to complete
  return SPDR;    // received byte

}

void address(unsigned long adrs)
{
  spi((adrs & 0x00FF0000) >> 16);
  spi((adrs & 0x0000FF00) >> 8);
  spi(adrs & 0x000000FF);

}
