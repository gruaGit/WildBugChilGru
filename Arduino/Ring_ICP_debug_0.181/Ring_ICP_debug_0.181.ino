//-------------------------------------------------------------------------
//Frequency Measure for a Motorcycle Testbench
//ICP4 and ICP5 are used for external signals. Based on Arduino Mega
//Rising edges will be detected
//Andreas Benz andreasnbenz@gmail.com
//----------------OUTPUT over Serial--------------------------------------
//sendCycle;Messfrequenz;Frequenz Kanal1(Pin49);Frequenz Kanal2(Pin48)
//------------------------------------------------------------------------
//Debug Output Pin38
//Built 4.4 21.12.2016
//---------------------------------------------

//-------------------Ringbuffer SIZE---------------------------------------
int comlevel = 1;                         
int ringsize = 50;
bool debug = false;

volatile uint32_t  StartTime4[50];        // ICR1-Wert bei 1.High-Flanke speichern
volatile uint32_t  EndTime4[50];          // ICR1-Wert bei 2.High-Flanke speicher
volatile int ovlCAPTst4[50];
volatile int ovlCAPTend4[50];
volatile int Messung4[50];
volatile int ErsteFlanke4 = 0;
volatile int ovlTIM4 = 0;

volatile uint32_t  StartTime5[50];        // ICR1-Wert bei 1.High-Flanke speichern
volatile uint32_t  EndTime5[50];          // ICR1-Wert bei 2.High-Flanke speicher
volatile int ovlCAPTst5[50];
volatile int ovlCAPTend5[50];
volatile int Messung5[50];
volatile int ErsteFlanke5 = 0;
volatile int ovlTIM5 = 0;
//-------------------------------------------------------------------
int i = 0;
int j = 0;
volatile uint32_t  ovl4 = 0;
volatile uint32_t  ovl5 = 0;
volatile uint32_t  ovlTIM0 = 0;





#include <util/atomic.h>

int main(void) {
  cli();

  //--------------------Reset all---------------------
  //Timer0      8-bit
  TCCR0A = 0x00;
  TCCR0B = 0x00;
  TIMSK0 = 0x00;
  
  //Timer1      16-bit
  TCCR1A = 0x00;
  TCCR1B = 0x00;
  TCCR1C = 0x00;
  TIMSK1 = 0x00;
  
  //Timer2      8-bit
  TCCR2A = 0x00;
  TCCR2B = 0x00;
  TIMSK2 = 0x00;
  
  //Timer3      16-bit
  TCCR3A = 0x00;
  TCCR3B = 0x00;
  TCCR3C = 0x00;
  TIMSK3 = 0x00;
  
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

  // INTERUPTS
  //EIMSK |= (1 << INT0);                   //enable external interrupt 
  //EICRA |= ((1 << ISC01) | (ISC00));      //rising edge of INT0

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

  //TIMER 0
  TIMSK0 |= (1 << TOIE0);                     // Timer 0 Overflow
  TCCR0B |= ((1 << CS02) | (1 << CS00));
  TCCR0B &= ~(1 << CS01);                      //1024 prescaler, timer 0 and START Overflow every 16.384ms

  //----------Port definition----------

  
  DDRD &= ~(1 << PL0) ;               //INPUT for INT0
  PORTD |= (1 << PL0);

  DDRB &= ~(1 << PL1) ;               //INPUT for ICP
  PORTB |= (1 << PL1);

  //----------DEBUG Port----------
  DDRD |= (1 << PD7) ;               //DEBUG
 


  //----------SERIAL---------
  Serial.begin(250000);



  //Clear all Interrupt Flags
 // EIFR |= (1 << INTF0);
  TIFR4 |= (1 << ICF4);   //Clear ICP4 FLAG
  TIFR5 |= (1 << ICF5);   //Clear ICP5 FLAG
 


  //------enables global Interupts--------------
  sei();


  uint32_t freqsum4;
  uint32_t freqsum5;
  uint32_t pulse4[ringsize];
  uint32_t freq4[ringsize];
  uint32_t pulse5[ringsize];
  uint32_t freq5[ringsize]; 
  uint32_t shift4[ringsize];
  uint32_t shift5[ringsize];

  uint16_t debugcount;
  double timer;
  uint32_t multi;
  uint16_t cycle=0;

  multi=comlevel*256;
  int x;

for (x = 0; x <= ringsize - 1 ; x++) {
  pulse4[x]=0;
  pulse5[x]=0;
  freq4[x]=0;
  freq5[x]=0;
  shift4[x]=0;
  shift5[x]=0;
}


  while (1)  {
    

    if (ovlTIM0 >= comlevel) {
  
      cycle++;
      uint32_t h4 = 1;
    uint32_t h5 = 1;

      for (x = 0; x <= ringsize - 1 ; x++) {
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
    
    for (x = 0; x <= ringsize - 1 ; x++) {
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
    freqsum4 = freqsum4/h4;
    freqsum5 = freqsum5/h5;





//------------------OUTPUT-------------------------

       timer= TCNT0+multi;
       timer=F_CPU/timer;
       timer=timer/1024;
       
       if(debug==true){
       
      
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
    if ((TIFR4 & 1 << TOV4) && (StartTime4[i] < 0x8000))      // evtl. Ueberlauf T1 noch offen?
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
    if ((TIFR5 & 1 << TOV5) && (StartTime5[j] < 0x8000))      // evtl. Ueberlauf T1 noch offen?
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
