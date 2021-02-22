#include <Arduino.h>
#include <RotaryEncoder.h>
#include <U8g2lib.h>

// Encoder input pins
#define PIN_IN1 A0
#define PIN_IN2 A2

// Push button input pin
#define PIN_BUTTON A1

// PWM led outputs
#define PWM_RED 10
#define PWM_GREEN 11
#define PWM_BLUE 9

// Initialize display
// Adjust for the correct display model
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0);

// Setup a RotaryEncoder with 2 steps per latch for the 2 signal input pins:
RotaryEncoder encoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::TWO03);

enum{
  RED,
  GREEN,
  BLUE
}currentColor=RED;

unsigned short red=0, green=0, blue=0;
int encPos;

// Interrupt Service Routine for Pin Change Interrupt 1
// This routine will only be called on any signal change on A2 and A3.
ISR(PCINT1_vect) {
   encoder.tick();
}

void setup()
{
  // Activate the ISR PCINT1.
  // Modify the next 2 lines if using other pins than A2 and A3
  PCICR |= (1 << PCIE1);    // This enables Pin Change Interrupt 1 that covers the Analog input pins or Port C.
  PCMSK1 |= (1 << PCINT8) | (1 << PCINT10);  // This enables the interrupt for pin 0 and 2 of Port C.
  
  u8g2.begin();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  
  Serial.begin(115200);
  while (! Serial);
  Serial.println("RGB color mixer");
  
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  pinMode(PWM_RED, OUTPUT);
  pinMode(PWM_GREEN, OUTPUT);
  pinMode(PWM_BLUE, OUTPUT);
  
  currentColor=RED;

  displayDraw();
}

void loop()
{
  analogWrite(PWM_RED, red);
  analogWrite(PWM_GREEN, green);
  analogWrite(PWM_BLUE, blue);
  
  if(digitalRead(PIN_BUTTON) == 0){
    switch(currentColor){
      case RED:
        currentColor=GREEN;
        break;
      case GREEN:
        currentColor=BLUE;
        break;
      case BLUE:
        currentColor=RED;
        break;
    }
    displayDraw();
    while(digitalRead(PIN_BUTTON) == 0)
      ;
    delay(100);
  }
  
  encoder.tick();
  encPos=encoder.getPosition();
  
  if(encPos != 0){
    adjColor();
    encoder.setPosition(0);
    displayDraw();
  }
}

void adjColor(){
  switch(currentColor){
      case RED:
        red += encPos*2;
        if(red > 255)
          encPos > 0 ? red=255 : red=0;
        break;
      case GREEN:
        green += encPos*2;
        if(green > 255)
          encPos > 0 ? green=255 : green=0;
        break;
      case BLUE:
        blue += encPos*2;
        if(blue > 255)
          encPos > 0 ? blue=255 : blue=0;
        break;
    }
}

void displayDraw(){
  u8g2.clearBuffer();
  u8g2.drawFrame(1, 1, 126, 14);
  u8g2.drawFrame(1, 16, 126, 14);
  u8g2.drawFrame(1, 31, 126, 14);
  u8g2.drawBox(3, 3, (float(red)/255.0)*122.0, 10);
  u8g2.drawBox(3, 18, (float(green)/255.0)*122.0, 10);
  u8g2.drawBox(3, 33, (float(blue)/255.0)*122.0, 10);
  drawSelFrame();
  u8g2.setCursor(0, 58);
  u8g2.print("R: ");
  u8g2.print(red);
  u8g2.setCursor(40, 58);
  u8g2.print("G: ");
  u8g2.print(green);
  u8g2.setCursor(80, 58);
  u8g2.print("B: ");
  u8g2.print(blue);
  u8g2.sendBuffer();
  
  return;
}

void drawSelFrame(){
  switch(currentColor){
      case RED:
        u8g2.drawFrame(13, 48, 21, 12);
        break;
      case GREEN:
        u8g2.drawFrame(52, 48, 21, 12);
        break;
      case BLUE:
        u8g2.drawFrame(92, 48, 21, 12);
        break;
    }
}
