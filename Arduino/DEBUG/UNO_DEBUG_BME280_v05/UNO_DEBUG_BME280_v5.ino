//-------------------------------------------------------------------------
//ONLY DEBUG
//Frequency Measure for a Motorcycle Testbench
//Andreas Benz andreasnbenz@gmail.com
// SEND e for BME280 data FIXED DATA
// SEND m for starting meassurement
//
//
//----------------OUTPUT over Serial--------------------------------------
//sendCycle;Messfrequenz;Frequenz Kanal1(Pin49);Frequenz Kanal2(Pin48)
//------------------------------------------------------------------------
//Debug Output Pin38
//Built 5 20.01.2017
//---------------------------------------------

#include <util/atomic.h>
#include <stdint.h>
#include "SparkFunBME280.h"
#include "SPI.h"
#include "Wire.h"
#include <util/delay.h>
//-------------------Ringbuffer SIZE---------------------------------------
boolean initial=true;
int comlevel = 1;                         
double Temp;
double P;
double Hum;

 String inputString = "";         // a string to hold incoming data
 boolean stringComplete = false;  // whether the string is complete
//-------------------------------------------------------------------

volatile uint32_t  ovlTIM0 = 0;


 //----------BME280--------------
//Global sensor object
BME280 mySensor;


int main(void) {
  cli();

  //--------------------Reset all---------------------
  //Timer0      8-bit
  TCCR0A = 0x00;
  TCCR0B = 0x00;
  TIMSK0 = 0x00;
  

  EICRA = 0x00;
  EIMSK = 0x00;
  
  DDRD = 0xff;    //all output
  PORTD = 0x00;   //all 0V;
  //--------------------SETUP-----------------------------

  // INTERUPTS
  //EIMSK |= (1 << INT0);                   //enable external interrupt 
  //EICRA |= ((1 << ISC01) | (ISC00));      //rising edge of INT0

 
  //TIMER 0
  TIMSK0 |= (1 << TOIE0);                     // Timer 0 Overflow
  TCCR0B |= ((1 << CS02) | (1 << CS00));
  TCCR0B &= ~(1 << CS01);                      //1024 prescaler, timer 0 and START Overflow every 16.384ms


  //----------DEBUG Port----------
  DDRD |= (1 << PD7) ;               //DEBUG
 


  //----------SERIAL---------
  Serial.begin(250000);


 uint32_t freqsum4;
  uint32_t freqsum5;
  uint16_t debugcount;
  double timer;
  uint32_t multi;
  uint16_t cycle=0;

  multi=comlevel*256;
  int x;

sei();
  bool     mess=false;

  while (1)  {
   
   if (Serial.available()>0) serialEvent();
   
if (stringComplete && inputString=="e" ) {
  BME280();
    Serial.print(Temp,1);
    Serial.print(";");
    Serial.print(P,0);
    Serial.print(";");
    Serial.println(Hum,1);
    
    stringComplete = false;
  }

  
  if (stringComplete && inputString=="m" ) {
    //Serial.println(mess);
    if (mess==false){
   //Serial.println("false");
    // clear the string:
    mess=true;
    inputString = "";
    }

    else{
   // Serial.println(mess);
    mess=false;
    cycle=0;
    // clear the string:
    inputString = "";
   }
    stringComplete = false;
  }
  
    if (ovlTIM0 >= comlevel && mess) {
  
      cycle++;
    

      timer= TCNT0+multi;
       timer=F_CPU/timer;
       timer=timer/1024;
      
       if(cycle/timer<10){
        debugcount=0;
        
         freqsum4=500;
        freqsum5=500;
       }
       if(cycle/timer>10 && cycle/timer<20){
         freqsum4=500;
        freqsum5=500;
        freqsum4=freqsum4+debugcount;
        freqsum5=freqsum5+debugcount;
        debugcount++;
       
       }
       if(cycle/timer>20){
        freqsum4=freqsum4-debugcount/100;
        freqsum5=freqsum5-debugcount/100;
        debugcount++;
        if(freqsum4<10){
        cycle=0;
        TCNT0=0;
        }
       }
       
      // PORTD ^= (1 << PD7);
        Serial.print(  cycle, DEC);
      Serial.print(";");
      Serial.print(  timer, 2);
      Serial.print(";");
      Serial.print(  freqsum4, DEC);
      Serial.print(";");
      Serial.print( freqsum5, DEC);
      Serial.print('\n');
 
//------------------RESET-------------------------
    
      
    ovlTIM0 = 0;
 
    TIFR0 |= (1 << TOV0);   //Clear Overflow FLAG
 
    TCNT0 = 0x00;           //Set Timer0 to 0

     
    }
  }
}



ISR(TIMER0_OVF_vect) {
  ovlTIM0++;

}


void serialEvent() {
  while (Serial.available()) {
    
    char inChar = (char)Serial.read();
    if (inChar != '\n'){
      inputString = inChar;
      stringComplete = true;
    }
  }
}

void BME280() {
  Serial.begin(57600);
    if (initial){
        _delay_ms(10);  //Make sure sensor had enough time to turn on. BME280 requires 2ms to start up.
  mySensor.begin();

 


initial=false;
    }
 
mySensor.readRegister(BME280_CHIP_ID_REG);
 // Serial.print("Reset register(0xE0): 0x");
mySensor.readRegister(BME280_RST_REG);
  //Serial.print("ctrl_meas(0xF4): 0x");
mySensor.readRegister(BME280_CTRL_MEAS_REG);
  //Serial.print("ctrl_hum(0xF2): 0x");
mySensor.readRegister(BME280_CTRL_HUMIDITY_REG);
  
//  Temp=mySensor.readTempC();
//  
// P=mySensor.readFloatPressure();
// 
// Hum=mySensor.readFloatHumidity();
//   

 Temp=23.3;
 P=101345;
 Hum=43.8;
  Serial.begin(250000);
}


