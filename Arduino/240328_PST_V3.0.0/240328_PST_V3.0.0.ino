/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                               #Open Source Prüfstandssoftware auf Basis von Arduino Mega und LabVIEW                            //
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// benoetigt die Bibiotheken SparkFunBME280 Adafruit_MAX31855
// I2C Adresse für BME280 muss an das jeweilige Breakoutboard angepasst werden in Zeile #define BMEADDRESS 0x76
// Wenn ein Max31855 Breakoutboard für EGT vorhanden ist muss bool thermo auf true gesetzt werden
//
//
#define VERSION "Version 3.0.0 vom 28.03.24"
//
//-------------------SETUP---------------------------------------
uint32_t baud = 115200;
int comlevel = 1;                      // increase for lower data transfer frequenz 1 approx. 45Hz

#define RINGSIZE4 5   //Groesse des Ringspeichers ZÃ¼ndsignal
#define RINGSIZE5 50  //Groesse des Ringspeichers Rollensignal, einzustellten auf 30-50 je nach Signalte/Umdrehung
#define AFRADDRESS 1        // I2C Adresse Spartan2, default=1
#define MAXCS 53            // CS Pin für MAX31855

bool thermo = false;                    // MAX31855 Data (Thermoelement) set true oder false
#define BMEADDRESS 0x76     //Adresse des BME280 Boards, 0x76 oder 0x77

int p_cor = 0;   // Korrekturoffset für die Werte vom BME280
int t_cor = 0;
int h_cor = 0;


//AFR
#define Pump_Current16_to_Lambda_Start 0 // To get index for Pump_Current16_to_Lambda Array for we take Pump_Current16 and subtract Pump_Current16_to_Lambda_Start
#define Pump_Current16_to_Lambda_Size 409 //size of Pump_Current16_to_Lambda Array
#define Ri_to_Temperature_C_Start 113 // To get index for Ri_to_Temperature_C Array for we take Ri and subtract Ri_to_Temperature_C_Start
#define Ri_to_Temperature_C_Size 75 //size of Ri_to_Temperature_C Array
//lookup table to convert Pump_Current16 to Lambda
const PROGMEM unsigned int Pump_Current16_to_Lambda[]={376,379,381,384,387,390,393,395,398,401,403,406,409,411,414,417,419,422,424,427,430,432,435,437,440,442,445,447,450,452,455,457,460,462,464,467,469,472,474,476,479,481,483,486,488,490,493,495,497,499,502,504,506,508,511,513,515,517,520,522,524,526,528,530,533,535,537,539,541,543,546,548,550,552,554,556,558,560,562,564,567,569,571,573,575,577,579,581,583,585,587,589,591,593,595,598,600,602,604,606,608,610,612,614,616,618,620,622,624,626,628,630,632,634,636,638,640,642,644,646,648,650,653,655,657,659,661,663,665,667,669,671,673,675,677,679,681,684,686,688,690,692,694,696,698,700,703,705,707,709,711,713,715,718,720,722,724,726,729,731,733,735,737,740,742,744,746,749,751,753,756,758,760,763,765,767,770,772,774,777,779,781,784,786,789,791,794,796,799,801,804,806,809,811,814,816,819,821,824,827,829,832,834,837,840,842,845,848,851,853,856,859,862,864,867,870,873,876,879,881,884,887,890,893,896,899,902,905,908,911,914,917,920,923,927,930,933,936,939,942,946,949,952,955,959,962,965,969,972,976,979,982,986,989,993,996,1000,1007,1015,1023,1031,1039,1047,1056,1064,1073,1082,1090,1099,1108,1118,1127,1136,1146,1156,1166,1176,1186,1196,1207,1217,1228,1239,1250,1262,1273,1285,1297,1309,1321,1334,1346,1359,1372,1386,1399,1413,1427,1442,1456,1471,1486,1502,1517,1533,1550,1566,1583,1600,1618,1636,1654,1673,1692,1711,1731,1752,1772,1794,1815,1837,1860,1883,1907,1931,1956,1981,2007,2033,2061,2088,2117,2146,2176,2207,2239,2271,2305,2339,2374,2411,2448,2486,2526,2567,2608,2652,2696,2742,2790,2839,2889,2942,2996,3052,3110,3171,3233,3298,3365,3435,3508,3584,3663,3745,3831,3920,4014,4112,4215,4323,4436,4555,4680,4812,4952,5099,5255,5420,5596,5783,5983,6196,6425,6670,6935,7220,7530,7866,8232,8634,9076,9564,10106,10713,11394,12167,13050,14068,15256,16659,18342,20399,22967,26268,30664,36811,46012,61299};
//lookup table to convert Ri to Sensor Temperature[C]
const PROGMEM unsigned int Ri_to_Temperature_C[]={820,818,817,815,814,813,811,810,808,807,805,804,803,801,800,799,798,796,795,794,792,791,790,789,788,786,785,784,783,782,781,780,779,777,776,775,774,773,772,771,770,769,768,767,766,765,764,763,762,761,760,759,758,758,757,756,755,754,753,752,751,751,750,749,748,747,746,745,745,744,743,742,741,741,740};
byte Val_Array[10];
unsigned int iafr=0;
byte No_I2C_Response_Flag=1;
byte I2C_Address = 1;
byte temp;
struct I2C_Struct{
  byte I2C_Addr;
  byte ID8;
  unsigned int Pump_Current16;
  byte Ri;
  byte Status8;
};
double Afr;
I2C_Struct I2C_Data; //This Struct holds the I2C data in the same form that it is on Spartan 2
struct Processed_Data_Struct{
  byte I2C_Addr;
  byte ID;
  float Lambda;
  unsigned int Temperature_C;
  byte Status;
};

Processed_Data_Struct Processed_Data; //This Struct hold the final Processed Data
volatile uint32_t  StartTime4[RINGSIZE4];        // ICR4-Wert bei 1.High-Flanke speichern
volatile uint32_t  EndTime4[RINGSIZE4];          // ICR4-Wert bei 2.High-Flanke speicher
volatile int ovlCAPTst4[RINGSIZE4];
volatile int ovlCAPTend4[RINGSIZE4];
volatile int Messung4[RINGSIZE4];
volatile int ErsteFlanke4 = 0;
volatile int ovlTIM4 = 0;

volatile uint32_t  StartTime5[RINGSIZE5];        // ICR5-Wert bei 1.High-Flanke speichern
volatile uint32_t  EndTime5[RINGSIZE5];          // ICR5-Wert bei 2.High-Flanke speicher
volatile int ovlCAPTst5[RINGSIZE5];
volatile int ovlCAPTend5[RINGSIZE5];
volatile int Messung5[RINGSIZE5];
volatile int ErsteFlanke5 = 0;
volatile int ovlTIM5 = 0;

uint32_t freqsum4;
uint32_t freqsum5;
uint32_t pulse4[RINGSIZE4];
uint32_t freq4[RINGSIZE4];
uint32_t pulse5[RINGSIZE5];
uint32_t freq5[RINGSIZE5];
uint32_t shift4[RINGSIZE4];
uint32_t shift5[RINGSIZE5];

double timer;
uint32_t multi;
uint16_t cycle = 0;
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
#include <avr/pgmspace.h>
#include <util/delay.h>

Adafruit_MAX31855 thermocouple(MAXCS);

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
void BME280read() {
  if (initial) {
    _delay_ms(10);
    mySensor.begin();
    _delay_ms(100);
    initial = false;
  }
  Temp = mySensor.readTempC();
  Temp = Temp + t_cor;
  P = mySensor.readFloatPressure();
  P = P + p_cor;
  Hum = mySensor.readFloatHumidity();
  Hum = Hum + h_cor;
}

///////////////////////////////////////////////////////////////
//-----------------Setup-------------------------------------//
///////////////////////////////////////////////////////////////
void setup() {
  pinMode(2, OUTPUT);
  Wire.begin(); // Initialize I2C

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

  //----------SERIAL---------
  Serial.begin(baud);

  inputString.reserve(5);

  //----------BME280--------------

  mySensor.settings.commInterface = I2C_MODE;
  mySensor.settings.I2CAddress = BMEADDRESS;
  mySensor.settings.runMode = 3; //Normal mode

  mySensor.settings.tStandby = 0;
  mySensor.settings.tempOverSample = 1;
  mySensor.settings.pressOverSample = 1;
  mySensor.settings.humidOverSample = 1;

 
  double timer;
  multi = comlevel * 256;

  //----------------Clear all and GO!!-----------------------------
  TIFR4 |= (1 << ICF4);   //Clear ICP4 FLAG
  TIFR5 |= (1 << ICF5);   //Clear ICP5 FLAG

  sei();
  //----------------INIT Ringbuffer = 0-----------------------------
  for (int x = 0; x <= RINGSIZE4 - 1 ; x++) {
    pulse4[x] = 0;

    freq4[x] = 0;

    shift4[x] = 0;
  }
    for (int x = 0; x <= RINGSIZE5 - 1 ; x++) {
  
    pulse5[x] = 0;
   
    freq5[x] = 0;
 
    shift5[x] = 0;
  }
}

///////////////////////////////////////////////////////////////
//-----------------AFR---------------------------------------//
///////////////////////////////////////////////////////////////

void afr() {

      Wire.beginTransmission(AFRADDRESS); //Setup communication with device @ I2C Address = 1 
      Wire.write(0); //This will tell Spartan that OEM that we want to start a read from Memory Address 0
      Wire.endTransmission();     // end transmission
      Wire.requestFrom(AFRADDRESS, 6); //Read 6 Bytes from Device with I2C Address = I2C_Address
      iafr=0;
      No_I2C_Response_Flag=1;
      while (Wire.available()) //Dump the I2C data into an Array
      {
        Val_Array[iafr] = Wire.read();
        No_I2C_Response_Flag=0;
        iafr++;
      }
      
      I2C_Data.I2C_Addr=Val_Array[0];
      I2C_Data.ID8=Val_Array[1];
      I2C_Data.Pump_Current16=(Val_Array[2]<<8) + Val_Array[3];
      I2C_Data.Ri=Val_Array[4];
      I2C_Data.Status8=Val_Array[5];
     
      Processed_Data.I2C_Addr=I2C_Data.I2C_Addr;
      Processed_Data.ID=I2C_Data.ID8;
      iafr=I2C_Data.Pump_Current16-Pump_Current16_to_Lambda_Start;
      // make sure array index is within array bounds
      if (iafr<0)
      {
          iafr=0;
      }
      if (iafr>(Pump_Current16_to_Lambda_Size-1))
      {
          iafr=Pump_Current16_to_Lambda_Size-1;
      }
      Processed_Data.Lambda=((float)pgm_read_word_near(Pump_Current16_to_Lambda+iafr))/1000; //use Pump_Current16 as index to Pump_Current16_to_Lambda lookup table and then Divide by 1000 to get actual Lambda
     
      iafr=I2C_Data.Ri-Ri_to_Temperature_C_Start;// Index has to be justified to the right by Ri_to_Temperature_C_Start
      // make sure array index is within array bounds
      if (iafr<0)
      {
          iafr=0;
      }
      if (iafr>(Ri_to_Temperature_C_Size-1))
      {
          iafr=Ri_to_Temperature_C_Size-1;
      }
      Processed_Data.Temperature_C=pgm_read_word_near(Ri_to_Temperature_C+iafr); //Use the Justified Ri as index to Ri_to_Temaperature_C lookup table to get get Sensor Temperature[C]
      Processed_Data.Status=I2C_Data.Status8;

      if (No_I2C_Response_Flag==1)
      {
      Afr = 2000;  
      }
      if (No_I2C_Response_Flag==0)
      {
      Afr = Processed_Data.Lambda*1470;  
      }
}

///////////////////////////////////////////////////////////////
//-----------------EGT---------------------------------------//
///////////////////////////////////////////////////////////////

void egt() {
      if (thermo) {
      c = thermocouple.readCelsius();
    }
}




///////////////////////////////////////////////////////////////
//-----------------MAIN--------------------------------------//
///////////////////////////////////////////////////////////////


void loop() {

  if (Serial.available() > 0) serialEvent();

  if (stringComplete && inputString == "e" ) {
    BME280read(); 
    Serial.print(Temp, 1);
    Serial.print(";");
    Serial.print(P, 0);
    Serial.print(";");
    Serial.println(Hum, 1);

    stringComplete = false;
  }

  if (stringComplete && inputString == "v" ) {
    Serial.print(VERSION);
    Serial.println(";");
    Serial.print("Ringspeicher Zuendsignal:");
    Serial.print(RINGSIZE4);
    Serial.println(";");
    Serial.print("Ringspeicher Rolle:");
    Serial.print(RINGSIZE5);
    Serial.println(";");
    Serial.print("Adresse BME280:");
    Serial.print(BMEADDRESS,HEX);
    Serial.println(";");
    Serial.print("Thermoelement (1=ja 0=nein):");
    Serial.print(thermo);
    Serial.println(";");


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
        digitalWrite(2, HIGH);
    afr();
    egt();
    
    PORTD ^= (1 << PD7);
    cycle++;
    h4 = 1;
    h5 = 1;

    for (int x = 0; x <= RINGSIZE4 - 1 ; x++) {
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

    for (int x = 0; x <= RINGSIZE5 - 1 ; x++) {
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

    //------------------TIMER-OUTPUT-------------------------

    timer = TCNT2 + multi;
    timer = F_CPU / timer;
    timer = timer / 1024;
    //------------------TIMER-RESET-------------------------
    ovlTIM2 = 0;
    TIFR2 |= (1 << TOV2);   //Clear Overflow FLAG
    TCNT2 = 0x00;           //Set Timer0 to 0

    Serial.print(  cycle, DEC);
    Serial.print(";");
    Serial.print(  timer, 2);
    Serial.print(";");
    Serial.print(  freqsum4, DEC);
    Serial.print(";");
    Serial.print( freqsum5, DEC);
    Serial.print(";");
    if (thermo) {

    Serial.print(c, 0);
    
    Serial.print(";");
    }
    else {
    Serial.print(Processed_Data.Temperature_C);
    Serial.print(";");
    }
    Serial.print((Afr)/100);
    if (thermo ) {
    Serial.print(";");
    Serial.print(Processed_Data.Temperature_C);
    }
    
    Serial.print('\n');

    //------------------RESET-------------------------
    //ovlTIM2 = 0;
    //TIFR2 |= (1 << TOV2);   //Clear Overflow FLAG
    //TCNT2 = 0x00;           //Set Timer0 to 0

    digitalWrite(2, LOW);
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
    if (i == RINGSIZE4) {
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
    if (j == RINGSIZE5) {
      j = 0;
    }
  }
}
