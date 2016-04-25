#include <Adafruit_NeoPixel.h>
#include <TimerOne.h>

#define IDLE_RESET 0b00
#define FLYING 0b01
#define LOSE 0b10
#define WIN 0b11

#define BARGE_PIN 6
#define LIVES_PIN 7
#define FIRE_PIN 8

#define ST_PIN1 2
#define ST_PIN2 3
#define THROTTLE 10

typedef void (*STATE_HANDLER_T)(void);
STATE_HANDLER_T state, last_state;

void win(void);
void lose(void);
void flying(void);
void idle(void);

volatile int state_var = 0b00;
volatile int brightness = 0;
int lives = 3;

Adafruit_NeoPixel barge_strip = Adafruit_NeoPixel(4, BARGE_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel lives_strip = Adafruit_NeoPixel(3, LIVES_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel fire_strip = Adafruit_NeoPixel(1, FIRE_PIN, NEO_GRB + NEO_KHZ800);


void win(){
   if (state != last_state) {
     for(int i=0;i<4;i++){
      // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
      barge_strip.setPixelColor(i, barge_strip.Color(0,255,0)); // bright green color.
      barge_strip.show(); // This sends the updated pixel color to the hardware.
      }
      lives = 3;
   }
};

void lose(){
   if (state != last_state) {
     for(int i=0;i<4;i++){
      // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
      barge_strip.setPixelColor(i, barge_strip.Color(255,0,0)); // Moderately bright red color.
      barge_strip.show(); // This sends the updated pixel color to the hardware.
      }
     lives --;
     if (lives == 0){
       lives = 3;
     }
     lives_strip.setPixelColor(lives, 0); // Turn it off
     lives_strip.show(); // This sends the updated pixel color to the hardware.
      }  
};

void flying(){
  if (state != last_state) {
    for(int i=0;i<4;i++){
      // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
      barge_strip.setPixelColor(i, barge_strip.Color(255,255,255)); // white
      barge_strip.show(); // This sends the updated pixel color to the hardware.
    }
    fire_strip.setPixelColor(0,0); //initialize to off
    fire_strip.show();
  }
 
//  brightness++;
//  barge_strip.setBrightness(brightness%255);
//  delay(100);
//  
  
//  if (digitalRead(THROTTLE)==1){
//    fire_strip.setPixelColor(0, fire_strip.Color(255,128,0));
//    fire_strip.show();
//  }
//  else{
//    fire_strip.setPixelColor(0,0);
//    fire_strip.show();
//  }  
};

void idle(){
  if (state != last_state) {
    for(int i=0;i<4;i++){
      // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
      barge_strip.setPixelColor(i, barge_strip.Color(255,255,255)); // white
      barge_strip.show(); // This sends the updated pixel color to the hardware.
    }
    for(int i=0;i<lives;i++){
    // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
    lives_strip.setPixelColor(i, barge_strip.Color(0,255,0)); // bright green color.
    lives_strip.show(); // This sends the updated pixel color to the hardware.
    }  
  }  
};

void setup(){
  Timer1.initialize(10000); //toggles every .1 seconds
  Timer1.attachInterrupt(callback); //attaches as timer overflow interrupt
  pinMode(ST_PIN1, INPUT); //state_var pin 1
  pinMode(ST_PIN2, INPUT); //sate pin 2  
  pinMode(THROTTLE, INPUT); //button input  
  
  barge_strip.begin();
  barge_strip.show();
  
  lives_strip.begin();
  lives_strip.show();
  
  state = idle;
  last_state = (STATE_HANDLER_T)NULL;
  
  Serial.begin(9600);
  
  lives_strip.begin();
  lives_strip.show();
  
  fire_strip.begin();
  fire_strip.show();
}



void callback()
{
  state_var = 0b00;
  last_state = state;
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
  if (state == flying){
    if (digitalRead(THROTTLE)==1){
      fire_strip.setPixelColor(0, fire_strip.Color(255,128,0));
      fire_strip.show();
    }
    else{
      fire_strip.setPixelColor(0,0);
      fire_strip.show();
    } 
  }
}


void loop(){
  state();
  //Serial.println(state_var, BIN);
}
