#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <AccelStepper.h>

AccelStepper stepper(AccelStepper::DRIVER, 3, 2);

LiquidCrystal_I2C lcd(0x27, 16, 2);

const int powerButtonPin = 7;
const int limitSwitchPin = 13;
const int red_LED = 11;
const int green_LED = 9;
const int blue_LED = 10;
const int potPin = A0;

float step2lin = 0.04;  // 0.04 mm per step
float syringeDiameter = 18.75;  // 18.75 mm for 20ml 14.65 mm for 10 mL
int flowRate = 20;  // in mL/min
int maxVolume = 20;  // mL

int oldPotValue = 0;
int potValue;

float diaCm = syringeDiameter / 10.0; // cm
float area = 0.25 * pow(diaCm, 2) * 3.14159;  // cm^2
float volume = area * (step2lin / 10);  // mL per Step

//calculate steps per sec for given flowrate and syringe diameter
float flow2step(int flowRate, float diameter) {
  float diaCm = diameter / 10.0;
  float area = 0.25 * pow(diaCm, 2) * 3.14159;
  float volume = area * (step2lin / 10);
  return (flowRate / 60.0) / volume;
}

// calculates the mL/min for given steps per sec
float step2flow(float step) {
  return step * volume * 60;
}

float distance = maxVolume / volume; // Steps to remove maxVolume
float totalDuration = distance / flow2step(flowRate,syringeDiameter); // the time to remove all fluid

//set up LCD display
void lcdDisplay(int flowRate, float duration) {
  lcd.setCursor(0, 0);
  lcd.print("Flow Rate:   ");
  lcd.setCursor(10, 0);
  lcd.print(flowRate);
  lcd.setCursor(0, 1);
  lcd.print("Time:");
  lcd.setCursor(5, 1);
  lcd.print(duration);
}


void setup() {
  Serial.begin(9600);

  lcd.init();
  lcd.backlight();

  pinMode(powerButtonPin, INPUT_PULLUP);
  pinMode(limitSwitchPin, INPUT);
  pinMode(potPin, INPUT);

  pinMode(red_LED, OUTPUT);
  pinMode(green_LED, OUTPUT);
  pinMode(blue_LED, OUTPUT);

  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(0);

  float stepPersec = flow2step(flowRate, syringeDiameter);

  stepper.setSpeed(stepPersec);

  lcdDisplay(flowRate, totalDuration);
}

/*
setColor - allows for the RGB values of the to be set easier for the LED
*/
void setColor(int red, int green, int blue) {
  #ifdef COMMON_ANODE
    red = 255 - red;
    green = 255 - green;
    blue = 255 - blue;
  #endif
  analogWrite(red_LED, red);
  analogWrite(green_LED, green);
  analogWrite(blue_LED, blue);
}

unsigned long interval = 1000; // 1 sec
unsigned long curTime = 0;
float remainingTime = totalDuration; // Initialize remaining time

unsigned long lcdUpdateTime = 0;  // Variable to track LCD update time
const unsigned long lcdInterval = 2000;  // LCD update interval in milliseconds


void loop() {
  static unsigned long prevTime = millis(); // Initialize prevTime

  potValue = analogRead(potPin);

  if (potValue != oldPotValue) {
    oldPotValue = potValue;
    flowRate = map(potValue, 0, 1023, 0, 695);
    //stepper.setSpeed(flow2step(flowRate,syringeDiameter));
    Serial.print("Flow Rate (ml/min): ");
    Serial.println(flowRate);
    //Serial.println(stepper.speed());
    totalDuration = distance / flow2step(flowRate,syringeDiameter);
    remainingTime = totalDuration; // Reset remaining time when flow rate changes
    prevTime = millis(); // Reset the timer
  }
  

  /*
  unsigned long currentMillis = millis();
  // Update LCD display at intervals without delay
  if (currentMillis - lcdUpdateTime >= lcdInterval) {
    lcdUpdateTime = currentMillis;  // Reset the LCD update time
    lcdDisplay(flowRate, remainingTime);
    Serial.println("update");
  }
  */
  

  if (digitalRead(powerButtonPin) == LOW){
    setColor(255, 125, 0); // Yellow when paused
    remainingTime = 0; // Reset remaining time when not in operation
  }
  else if(digitalRead(limitSwitchPin) == HIGH){
    setColor(255, 0, 0); // Red when stopped
    remainingTime = 0; // Reset remaining time when not in operation
  }
  else if (flowRate == 0){
    setColor(255, 125, 0); // Yellow when paused
    remainingTime = 0; // Reset remaining time when flow rate is 0
  }
  else{
    stepper.runSpeed();
    setColor(0,255,0);  // Green when running

    /*
    curTime = millis(); // Get the current time
    unsigned long elapsedTime = curTime - prevTime;

    if (elapsedTime >= interval && remainingTime > 0) {
        remainingTime -= (elapsedTime / 1000.0); // Decrement remaining time
        prevTime = curTime; // Update previous time
    }
    else{
      remainingTime = 0;
    }
    */
  }
}




