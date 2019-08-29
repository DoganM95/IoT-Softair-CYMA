// Libraries
#include "./libraries/newlib/pthread.h"

// Credentials

// GPIO
// input only
const int shotSensorPin = 34;  // Read: 1 = Free, 0 = Interrupted
const int triggerPin = 35;     // Read: 1 = Shoot, 0 = Stop
// output only
const int firePin = 33;  // Write 1 = enable relay = shoot, else stop

// setting PWM properties
const int freq = 5000;
const int shotChannel = 0;  // Shot Sensor
const int resolution = 8;   // 8 Bit results in 0-255 as Range for Duty Cycle

int shotDetection = 0;
int shotNumber = 0;

void setup() {
  // Serial Setup
  Serial.begin(115200);

  // GPIO Setup
  ledcSetup(shotChannel, freq, resolution);
  ledcAttachPin(shotSensorPin, shotChannel);
  pinMode(shotSensorPin, INPUT);
}

void loop() {
  Serial.println("entering sensor loop now");
  shotDetection = digitalRead(34);
  while (true) {
    if (digitalRead(34) == 0) {
      Serial.print("Shot FIred!!! - ");
      Serial.println(++shotNumber);
      while (digitalRead(34) == 0) {
      }
    }
  }
}

// Todo:

// pthread: Run loops threaded (like shot detection loop)

// declare volatile, global vars to be shared between threads

// build in a trigger sensor to shoot when trigger is pulled and implement semi
// automatic shooting afterwards

// 3D Print barrel cover

// implement deepsleep mode (sleep when no shot is detected for 1 minute) and
// connect led's to gpios, so glock lights up a red status led to inidcate
// deepsleep and green when awake

// mechanically integrate a metal (flat side of a screw?) into trigger which
// gets touched wen touching the trigger to wake the esp from deepsleep mode