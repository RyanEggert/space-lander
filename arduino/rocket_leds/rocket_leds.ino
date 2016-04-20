#include <Adafruit_NeoPixel.h>
#include <TimerOne.h>

#define IDLE_RESET 0b00
#define FLYING 0b01
#define LOSE 0b10
#define WIN 0b11

#define LED_PIN 6
#define ST_PIN1 1
#define ST_PIN2 2

typedef void (*STATE_HANDLER_T)(void);
STATE_HANDLER_T state, last_state;
void win(void);
void lose(void);
void flying(void);
void idle(void);


volatile int state_var = 0b00;


Adafruit_NeoPixel barge_strip = Adafruit_NeoPixel(60, LED_PIN, NEO_GRB + NEO_KHZ800);
//Adafruit_NeoPixel lives_strip = Adafruit_NeoPixel(3, LED_PIN, NEO_GRB + NEO_KHZ800);


void win(){
   for(int i=0;i<4;i++){
    // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
    barge_strip.setPixelColor(i, barge_strip.Color(0,255,0)); // Moderately bright green color.
    barge_strip.show(); // This sends the updated pixel color to the hardware.
  }  
};

void lose(){
    for(int i=0;i<4;i++){
    // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
    barge_strip.setPixelColor(i, barge_strip.Color(255,0,0)); // Moderately bright red color.
    barge_strip.show(); // This sends the updated pixel color to the hardware.
  }  
};

void flying(){
  for(int i=0;i<4;i++){
    // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
    barge_strip.setPixelColor(i, barge_strip.Color(255,255,255)); // white
    barge_strip.show(); // This sends the updated pixel color to the hardware.
  }   
};

void idle(){
  for(int i=0;i<4;i++){
    // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
    barge_strip.setPixelColor(i, barge_strip.Color(255,255,255)); // white
    barge_strip.show(); // This sends the updated pixel color to the hardware.
  }  
};

void setup(){
  Timer1.initialize(10000); //toggles every .01 seconds
  Timer1.attachInterrupt(callback); //attaches as timer overflow interrupt
  pinMode(ST_PIN1, INPUT); //state_var pin 1
  pinMode(ST_PIN2, INPUT); //sate pin 2  
  
  barge_strip.begin();
  barge_strip.show();
  
  state = idle;
  last_state = (STATE_HANDLER_T)NULL;
  
  Serial.begin(9600);
  
  //lives_strip.begin();
  //lives_strip.show();
}

void callback()
{
  state_var = 0b00;
  state_var = (state_var | digitalRead(ST_PIN2) | (digitalRead(ST_PIN1) << 1));
  Serial.println(state_var, BIN);
  switch(state_var){
    case IDLE_RESET:
      state = idle;
      break;
    case FLYING:
      state = flying;
      break;
    case WIN:
      state = win;
      break;
    case LOSE:
      state = lose;
      break;
  }
}


void loop(){
  state();
  //Serial.println(state_var, BIN);
}
