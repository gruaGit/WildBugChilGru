
//-------------------------------------------------------------------------
// Frequency Measure for a Motorcycle Testbench
// ICP4 and ICP5 are used for external signals. Based on Arduino Mega
// Rising edges will be detected
// Send commands with Newline \n
// Andreas Benz andreasnbenz@gmail.com
// SEND e for BME280 data
// SEND m for starting meassurement
// MAX31855 libary integrated
// Set thermo = true for max31855 data transfer
//----------------OUTPUT over Serial--------------------------------------
//sendCycle;Messfrequenz;Frequenz Kanal1(Pin49);Frequenz Kanal2(Pin48);Max31855 Thermocouple read
//------------------------------------------------------------------------
//Debug Output Pin38   PORTD ^= (1 << PD7);
//Built 1.01 02.03.2017
//---------------------------------------------

//-------------------SETUP---------------------------------------
uint32_t baud = 115200;
int comlevel = 1;                    	// increase for lower data transfer frequenz 1 approx. 45Hz
int ringsize = 50;                 		// size of Ringbuffer
uint16_t thermocycle = 25;	
bool thermo = false;                   	// MAX31855 Data (Thermoelement) send yes or no
int p_cor= 500;							// Correction for BME280 Pressure Measurement in Pa
int t_cor= 0;  
int h_cor= 0;  
//----------MAX31855--------------

#define MAXCS 53						// CS Pin fpr MAX31855



volatile uint32_t  StartTime4[50];        // ICR4-Wert bei 1.High-Flanke speichern
volatile uint32_t  EndTime4[50];          // ICR4-Wert bei 2.High-Flanke speicher
volatile int ovlCAPTst4[50];
volatile int ovlCAPTend4[50];
volatile int Messung4[50];
volatile int ErsteFlanke4 = 0;
volatile int ovlTIM4 = 0;

volatile uint32_t  StartTime5[50];        // ICR5-Wert bei 1.High-Flanke speichern
volatile uint32_t  EndTime5[50];          // ICR5-Wert bei 2.High-Flanke speicher
volatile int ovlCAPTst5[50];
volatile int ovlCAPTend5[50];
volatile int Messung5[50];
volatile int ErsteFlanke5 = 0;
volatile int ovlTIM5 = 0;


uint32_t freqsum4;
uint32_t freqsum5;
uint32_t pulse4[50];
uint32_t freq4[50];
uint32_t pulse5[50];
uint32_t freq5[50];
uint32_t shift4[50];
uint32_t shift5[50];

uint16_t debugcount;
double timer;
uint32_t multi;
uint16_t cycle = 0;
uint16_t lastcycle =0;
bool    mess = false;
bool    initial = true;
double  c = 0;
//-------------------------------------------------------------------
int i = 0;
int j = 0;
uint32_t h4;
uint32_t h5;
volatile uint32_t  ovl4 = 0;
volatile uint32_t  ovl5 = 0;
volatile uint32_t  ovlTIM2 = 0;
double Temp;
double P;
double Hum;



String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete



#include <util/atomic.h>
#include <stdint.h>
#include <SparkFunBME280.h>
#include <Adafruit_MAX31855.h>
#include <SPI.h>
#include <Wire.h>
#include <util/delay.h>

//////////////////////////////////////////////////////////////////
//------------------Global sensor object------------------------//
//////////////////////////////////////////////////////////////////


//----------MAX31855--------------
Adafruit_MAX31855 thermocouple(MAXCS);

//----------BME280--------------
BME280 mySensor;

//----------Serial Commands--------------
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar != '\n') {
      inputString = inChar;
      stringComplete = true;
    }
  }
}

//----------BME280--------------
void BME280() {
  Serial.begin(57600);	//BME280 communicates on 57600 115200
  if (initial) {
   _delay_ms(10);  
    mySensor.begin();
   _delay_ms(100);  
    initial = false;
  }

  mySensor.readRegister(BME280_CHIP_ID_REG);
  mySensor.readRegister(BME280_RST_REG);
  mySensor.readRegister(BME280_CTRL_MEAS_REG);
  mySensor.readRegister(BME280_CTRL_HUMIDITY_REG);

  Temp = mySensor.readTempC();
  Temp=Temp+t_cor;
  P = mySensor.readFloatPressure();
  P= P+p_cor;
  Hum = mySensor.readFloatHumidity();
  Hum=Hum+h_cor;
  Serial.begin(baud);
}


//////////////////////////////////////////////////////////////////
//------------------AVR SETUP-----------------------------------//
//////////////////////////////////////////////////////////////////

void setup() {
  cli();
  //--------------------Reset all Timer and Register---------------------

  //Timer2      8-bit
  TCCR2A = 0x00;
  TCCR2B = 0x00;
  TIMSK2 = 0x00;

  //Timer4      16-bit
  TCCR4A = 0x00;
  TCCR4B = 0x00;
  TCCR4C = 0x00;
  TIMSK4 = 0x00;

  //Timer5      16-bit
  TCCR5A = 0x00;
  TCCR5B = 0x00;
  TCCR5C = 0x00;
  TIMSK5 = 0x00;

  EICRA = 0x00;
  EIMSK = 0x00;

  DDRD = 0xff;    //all output
  PORTD = 0x00;   //all 0V;
  //--------------------SETUP-----------------------------

  // INTERUPTS TIMER 4
  TCCR4B |= (1 << ICNC4);             //Noise Cancelling
  TIMSK4 |= (1 << TOIE4);             //Overflow enable
  TIMSK4 |= (1 << ICIE4);             //ICP on

  // INTERUPTS TIMER 5
  TCCR5B |= (1 << ICNC5);             //Noise Cancelling
  TIMSK5 |= (1 << TOIE5);             //Overflow enable
  TIMSK5 |= (1 << ICIE5);             //ICP on

  // TIMER 4
  TCCR4B |= (1 << CS40);
  TCCR4B &= ~((1 << CS42) | (1 << CS41));     // no prescaler, timer 1 and START*/

  // TIMER 5
  TCCR5B |= (1 << CS50);
  TCCR5B &= ~((1 << CS52) | (1 << CS51));     // no prescaler, timer 1 and START*/

  //TIMER 2
  TIMSK2 |= (1 << TOIE2);                     // Timer 2 Overflow
  TCCR2B |= (1 << CS22) ;
  TCCR2B |= ((1 << CS21) | (1 << CS20));     //1024 prescaler, timer 2 and START 

  //----------Port definition----------

  DDRD &= ~(1 << PL0) ;               //INPUT for INT0
  PORTD |= (1 << PL0);

  DDRB &= ~(1 << PL1) ;               //INPUT for ICP
  PORTB |= (1 << PL1);

  //----------DEBUG Port----------
  DDRD |= (1 << PD7) ;               //DEBUG



  //----------SERIAL---------
  Serial.begin(baud);
  
  inputString.reserve(5);
  
  //----------BME280--------------

  mySensor.settings.commInterface = I2C_MODE;
  mySensor.settings.I2CAddress = 0x77;
  mySensor.settings.runMode = 3; //Normal mode

  mySensor.settings.tStandby = 0;
  mySensor.settings.tempOverSample = 1;
  mySensor.settings.pressOverSample = 1;
  mySensor.settings.humidOverSample = 1;
  
  multi = comlevel * 256;

//----------------Clear all and GO!!-----------------------------
  TIFR4 |= (1 << ICF4);   //Clear ICP4 FLAG
  TIFR5 |= (1 << ICF5);   //Clear ICP5 FLAG

  sei();
//----------------INIT Ringbuffer = 0-----------------------------
  for (int x = 0; x <= ringsize - 1 ; x++) {
    pulse4[x] = 0;
    pulse5[x] = 0;
    freq4[x] = 0;
    freq5[x] = 0;
    shift4[x] = 0;
    shift5[x] = 0;
  }
}


 

///////////////////////////////////////////////////////////////
//-----------------MAIN--------------------------------------//
///////////////////////////////////////////////////////////////


void loop() {


  if (Serial.available() > 0) serialEvent();

  if (stringComplete && inputString == "e" ) {
    BME280();
    Serial.print(Temp, 1);
    Serial.print(";");
    Serial.print(P, 0);
    Serial.print(";");
    Serial.println(Hum, 1);

    stringComplete = false;
  }


  if (stringComplete && inputString == "m" ) {

    if (mess == false) {
      mess = true;
      inputString = "";
    }

    else {
      mess = false;
      cycle = 0;
      inputString = "";
    }
    stringComplete = false;
  }

  if (ovlTIM2 >= comlevel && mess ) {
   PORTD ^= (1 << PD7);
    cycle++;
     h4 = 1;
     h5 = 1;

    for (int x = 0; x <= ringsize - 1 ; x++) {
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if (Messung4[x] == 1) {
          h4++;
          shift4[x] = (ovlCAPTend4[x] - ovlCAPTst4[x]);
          shift4[x] = shift4[x] << 16;
          pulse4[x] = EndTime4[x] - StartTime4[x] + shift4[x] ;
          freq4[x] = F_CPU / pulse4[x];
          freqsum4 = freqsum4 + freq4[x];
        }
      }
    }

    for (int x = 0; x <= ringsize - 1 ; x++) {
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if (Messung5[x] == 1) {
          h5++;
          shift5[x] = (ovlCAPTend5[x] - ovlCAPTst5[x]);
          shift5[x] = shift5[x] << 16;                  //mal 2^16
          pulse5[x] = EndTime5[x] - StartTime5[x] + shift5[x] ;
          freq5[x] = F_CPU / pulse5[x];
          freqsum5 = freqsum5 + freq5[x];
        }
      }
    }

    //--------------------RESULT of moving average-------------------
    freqsum4 = freqsum4 / h4;
    freqsum5 = freqsum5 / h5;

    //------------------OUTPUT-------------------------

    timer = TCNT2 + multi;
    timer = F_CPU / timer;
    timer = timer / 1024;
    
    // PORTD ^= (1 << PD7);
    Serial.print(  cycle, DEC);
    Serial.print(";");
    Serial.print(  timer, 2);
    Serial.print(";");
    Serial.print(  freqsum4, DEC);
    Serial.print(";");
    Serial.print( freqsum5, DEC);
    if (thermo&&cycle>lastcycle+thermocycle){
     c = thermocouple.readCelsius();
      lastcycle=cycle;
    }
    Serial.print(";");
    Serial.print( c, 2);
    Serial.print('\n');

    //------------------RESET-------------------------


    ovlTIM2 = 0;

    TIFR2 |= (1 << TOV2);   //Clear Overflow FLAG

    TCNT2 = 0x00;           //Set Timer0 to 0

  }
}






ISR(TIMER2_OVF_vect) {
  ovlTIM2++;
}

ISR(TIMER4_OVF_vect) {
  ovlTIM4++;
}

ISR(TIMER5_OVF_vect) {
  ovlTIM5++;
}



ISR(TIMER4_CAPT_vect) {

  if ( ErsteFlanke4 == 1) {

    Messung4[i] = 0;
    StartTime4[i] = ICR4;
    ovlCAPTst4[i] = ovlTIM4;
    if ((TIFR4 & 1 << TOV4) && (StartTime4[i] < 0x8000))      // evtl. Ueberlauf T4 noch offen?
    {
      ovlCAPTst4[i]++;                                         // nur, wenn capture-int + overflow-int gleichzeitig !
    }
    ErsteFlanke4 = 0;
  }
  else {

    EndTime4[i] = ICR4;
    ovlCAPTend4[i] = ovlTIM4;
    if ((TIFR4 & 1 << TOV4) && (EndTime4[i] < 0x8000))      // evtl. Ueberlauf T4 noch offen?
    {
      ovlCAPTend4[i]++;                                      // nur, wenn capture-int + overflow-int gleichzeitig !
    }


    Messung4[i] = 1;
    ErsteFlanke4 = 1;

    TIFR4 |= (1 << ICF4);
    i++;
    //-------End of ringsize
    if (i == ringsize) {
      i = 0;
    }
  }
}

ISR(TIMER5_CAPT_vect) {

  if ( ErsteFlanke5 == 1) {

    Messung5[j] = 0;
    StartTime5[j] = ICR5;
    ovlCAPTst5[j] = ovlTIM5;
    if ((TIFR5 & 1 << TOV5) && (StartTime5[j] < 0x8000))      // evtl. Ueberlauf T5 noch offen?
    {
      ovlCAPTst5[j]++;                                         // nur, wenn capture-int + overflow-int gleichzeitig !
    }
    ErsteFlanke5 = 0;
  }
  else {

    EndTime5[j] = ICR5;
    ovlCAPTend5[j] = ovlTIM5;
    if ((TIFR5 & 1 << TOV5) && (EndTime5[j] < 0x8000))      // evtl. Ueberlauf T5 noch offen?
    {
      ovlCAPTend5[j]++;                                      // nur, wenn capture-int + overflow-int gleichzeitig !
    }

    Messung5[j] = 1;
    ErsteFlanke5 = 1;

    TIFR5 |= (1 << ICF5);
    j++;
    //-------End of ringsize
    if (j == ringsize) {
      j = 0;
    }
  }
}


