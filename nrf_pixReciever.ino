#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
//////////////////////CONFIGURATION///////////////////////////////
#define CHANNEL_NUMBER 8  //set the number of chanels
#define CHANNEL_DEFAULT_VALUE 1500  //set the default servo value
#define FRAME_LENGTH 22500  //set the PPM frame length in microseconds (1ms = 1000µs)
#define PULSE_LENGTH 300  //set the pulse length
#define onState 1  //set polarity of the pulses: 1 is positive, 0 is negative
#define sigPin 10  //set PPM signal output pin on the arduino

/*this array holds the servo values for the ppm signal
 change theese values in your code (usually servo values move between 1000 and 2000)*/
int ppm[CHANNEL_NUMBER];
RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00001";

void setup(){  
  
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
  
  //initiallize default ppm values
  for(int i=0; i<CHANNEL_NUMBER; i++){
      ppm[i]= CHANNEL_DEFAULT_VALUE;
  }
  pinMode(7, INPUT);
  pinMode(sigPin, OUTPUT);
  digitalWrite(sigPin, !onState);  //set the PPM signal pin to the default state (off)
  
  cli();
  TCCR1A = 0; // set entire TCCR1 register to 0
  TCCR1B = 0;
  
  OCR1A = 100;  // compare match register, change this
  TCCR1B |= (1 << WGM12);  // turn on CTC mode
  TCCR1B |= (1 << CS11);  // 8 prescaler: 0,5 microseconds at 16mhz
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
  sei();

}

  int throttle = 1500;
  int yaw      = 1500;
  int pitch    = 1500;
  int roll     = 1500;
  int aux1     = 1500;
  int aux2     = 1500;
  int camroll  = 1500;
  int campitch = 1500;

void loop(){
   
   if(radio.available())
   {
     char message[20] = "";
     radio.read(&message, sizeof(message));
     String throttlestr = message[0]  + message[1] + message[2]   + message[3];
     String yawstr      = message[4]  + message[5] + message[6]   + message[7];
     String pitchstr    = message[8]  + message[9] + message[10]  + message[11];
     String rollstr     = message[12] + message[13] + message[14] + message[15];
     String armstr      = message[16] + message[17] + message[18] + message[19];

     throttle = throttlestr.toInt();
     yaw      = yawstr.toInt();
     pitch    = pitchstr.toInt();
     roll     = rollstr.toInt();
     aux1     = armstr.toInt();
     
   }
     
}

ISR(TIMER1_COMPA_vect){  //leave this alone
  static boolean state = true;
  
  TCNT1 = 0;
  
  if (state) {  //start pulse
    digitalWrite(sigPin, onState);
    OCR1A = PULSE_LENGTH * 2;
    state = false;
  } else{  //end pulse and calculate when to start the next pulse
    static byte cur_chan_numb;
    static unsigned int calc_rest;
  
    digitalWrite(sigPin, !onState);
    state = true;

    if(cur_chan_numb >= CHANNEL_NUMBER){
      cur_chan_numb = 0;
      calc_rest = calc_rest + PULSE_LENGTH;// 
      OCR1A = (FRAME_LENGTH - calc_rest) * 2;
      calc_rest = 0;
    }
    else{
      OCR1A = (ppm[cur_chan_numb] - PULSE_LENGTH) * 2;
      calc_rest = calc_rest + ppm[cur_chan_numb];
      cur_chan_numb++;
    }     
  }
}
