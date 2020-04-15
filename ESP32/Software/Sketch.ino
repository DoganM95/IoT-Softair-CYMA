// Librararies
#include <Adafruit_NeoPixel.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <Update.h>
#include <WebServer.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
#include <WiFiUdp.h>
#include <pthread.h>
#include <stdbool.h>
#include <time.h>
// #include <Adafruit_NeoPixel>

// #include <queue.h>
// #include <semphr.h>

// Credentials
#include "Credentials/OtaLogin.h"
#include "Credentials/Wifi.h"

// using namespace std;

class Softair {
 public:
  char* manufacturer;
  char* brand;
  char* model;

  // GPIO Pins
  static const unsigned short int pistonSensorReadPin = 34;  // Read: 1 = infrared barrier free, 0 = IR interrupted
  static const unsigned short int triggerPullReadPin = 32;   // Read: 1 = Shoot, 0 = Stop
  static const unsigned short int triggerTouchReadPin = 4;
  static const unsigned short int motorControlPin = 33;  // HIGH = shoot
  static const unsigned short int neoPixelPin = 23;

  // PWM channels and settings
  static const int freq = 5000;
  static const int shotChannel = 0;  // Shot Sensor
  static const int resolution = 8;   // 8 Bits resulting in 0-255 as Range for Duty Cycle

  // Counters
  static unsigned long int shotsFired;  // each counter only since boot, for permanent storage use a database or eeprom
  // Setting debounce delay for mechanical trigger switch/button
  static const int triggerDebounceDelay = 1;

  // Configurations
  static const unsigned short int burstShootCount = 3;                // Defines how many BB's to shoot in burst mode with one trigger pull
  static const unsigned int touchDetectionThreshold = 18;             // Adjustment of touch pin sensitivity
  static const unsigned short int debounceStableTimeUntilShoot = 10;  // Time for a button state to persist until it is considered as settled (not bouncing anymore)
  static const unsigned short int debounceTimeout = 100;              // Time in ms, after which the current trigger process is cancelled
  static char* fireMode;

  // States / vars
  static bool triggerEnabledByTouch;
  static bool triggerPulledByFinger;
  static unsigned int triggerTouchValue;

  // Thread Adjustments
  static const short int threadSetTriggerTouchedStateRoutineSleepDuration = 100;  // in ms (alters responsiveness)
  static const short int threadSetTriggerPulledStateRoutineSleepDuration = 100;   // in ms (alters responsiveness)
  static const short int threadShootOnTouchAndTriggerRoutineSleepDuration = 100;  // in ms (alters responsiveness)

  // TODO: Rename Thread handlers according to thread names
  pthread_t setTriggerTouchedStateRoutineThreadHandle;
  pthread_t setTriggerPulledStateRoutineThreadHandle;
  pthread_t shootOnTouchAndTriggerRoutineThreadHandle;
  pthread_t countShotsRoutineThreadHandle;
  pthread_t setSystemSleepStateRoutineThreadHandle;

  SemaphoreHandle_t exampleSemaphoreForPthreads;
  pthread_mutex_t exampleMutexForThreads;

  // Constructor
  Softair(const char* manufacturer, const char* brand, const char* model);

  // Actions

  static void shoot(bool state, char* shootMode);  // sets ShootPin's output HUGH, shootMode is optional

  // Threaded reactive attributes / routines

  static void* setTriggerTouchedStateRoutine(void* param);  // Trigger's capacitive touch area listener
  static void* setTriggerPulledStateRoutine(void* param);   // Trigger's pull listener
  static void* shootOnTouchAndTriggerRoutine(void* param);  // Motor GPIO state setter
  static void* countShotsRoutine(void* param);              // counts the interruptions between IR transmitter and receiver
  static void* setSystemSleepStateRoutine(void* param);     // sleeping if gun idle for spec. seconds, wake on touch pin touched
};

// SETUP
bool Softair::triggerEnabledByTouch = false;
bool Softair::triggerPulledByFinger = false;
unsigned long int Softair::shotsFired = 0;
char* Softair::fireMode = "semi-automatic";
unsigned int Softair::triggerTouchValue = 0;
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(4, Softair::neoPixelPin, NEO_GRB + NEO_KHZ800);

// ----------------------------------------------------------------------------
// Softair Constructors
// ----------------------------------------------------------------------------

Softair::Softair(const char* manufacturer, const char* brand, const char* model) {
  manufacturer = manufacturer;
  brand = brand;
  model = model;
}

// ----------------------------------------------------------------------------
// Softair Methods (outside of class to keep the class clean and readable)
// ----------------------------------------------------------------------------
void Softair::shoot(bool state, char* shootMode = "") {
  unsigned int duration = 0;
  unsigned int shots = 0;

  if (state) {
    if (shootMode == "semi-automatic") {
      digitalWrite(motorControlPin, HIGH);
      while (digitalRead(pistonSensorReadPin) == 1) {  // while infrared barrier is not blocked, wait
        delay(1);                                      // FIXME: needs to be replaced by a time check
        duration++;
        if (duration >= 3000) {
          goto skipCounterCausedByBarrierError;
          break;  // security mechanism to stop shooting after x ms (in case of sensor failure)}
        }
      }

      shotsFired++;
    skipCounterCausedByBarrierError:
      digitalWrite(motorControlPin, LOW);  // on infrared barrier blocked, cut motor electricity
    }

    else if (shootMode == "burst") {
      digitalWrite(motorControlPin, HIGH);
      while (digitalRead(pistonSensorReadPin) == 1) {
        delay(1);
      }
    }

    else if (shootMode == "full-automatic") {  // shoots until either trigger is released or finger stopped touching trigger
      while (digitalRead(triggerPullReadPin) == HIGH && touchRead(4) <= touchDetectionThreshold) {
      continueShooting:
        digitalWrite(motorControlPin, HIGH);
      }
      delay(triggerDebounceDelay / 4);
      if (digitalRead(triggerPullReadPin) == LOW || touchRead(4) >= touchDetectionThreshold) {
        digitalWrite(motorControlPin, LOW);
      } else {
        goto continueShooting;
      }
    }

  } else {
    digitalWrite(motorControlPin, LOW);
  }
}
// ----------------------------------------------------------------------------
// Threaded Routines
// ----------------------------------------------------------------------------

void* Softair::setTriggerTouchedStateRoutine(void* param) {  // Working
  // Flags to prevent writing the value of setTriggerTouchedStateRoutine over and over again while it is already enabled
  bool setEnabledFlag = false;
  bool setDisabledFlag = false;
  short int tempTouchValue;

  while (true) {
    tempTouchValue = touchRead(triggerTouchReadPin);
    triggerTouchValue = tempTouchValue;
    if (tempTouchValue <= touchDetectionThreshold && setEnabledFlag == false) {
      triggerEnabledByTouch = true;
      setEnabledFlag = true;
      setDisabledFlag = false;
    } else if (tempTouchValue > touchDetectionThreshold && setDisabledFlag == false) {
      triggerEnabledByTouch = false;
      setEnabledFlag = false;
      setDisabledFlag = true;
    }
    delay(threadSetTriggerTouchedStateRoutineSleepDuration);  // time to sleep between each iteration
  }
}

void* Softair::setTriggerPulledStateRoutine(void* param) {
  // Flags to prevent writing the value of setTriggerTouchedStateRoutine over and over again while it is already enabled
  bool setEnabledFlag = false;
  bool setDisabledFlag = false;
  bool touchFlag = false;

  unsigned int timerLow = 0;
  unsigned int timerHigh = 0;
  unsigned int timeoutTimer = 0;

  while (true) {
    timerHigh = 0;
    timerLow = 0;

    if (triggerEnabledByTouch) {
      touchFlag = true;
      Serial.printf("-----------------------------------------------------\n");
      Serial.printf("pullThread - Trigger Touching: %d\n", triggerTouchValue);

      while (digitalRead(triggerPullReadPin) == HIGH && setEnabledFlag == false) {
        timerHigh++;
        Serial.printf("pullThread - Time High: %d\n", timerHigh);
        if (timerHigh >= 10) {
          Serial.printf("pullThread - pulling trigger and touching, timerHigh = %d\n", timerHigh);
          triggerPulledByFinger = true;
          setEnabledFlag = true;
          setDisabledFlag = false;
        }
        delay(1);
      }

      while (digitalRead(triggerPullReadPin) == LOW && setDisabledFlag == false) {
        timerLow++;
        Serial.printf("pullThread - Time Low: %d\n", timerLow);
        if (timerLow >= 10) {
          Serial.printf("pullThread - released trigger or not touching, timerLow = %d\n", timerLow);
          triggerPulledByFinger = false;
          setEnabledFlag = false;
          setDisabledFlag = true;
        }
        delay(1);
      }

    } else if (!triggerEnabledByTouch) {
      // setEnabledFlag = false;
      // setDisabledFlag = false;
      touchFlag = false;
      triggerPulledByFinger = false;
      Serial.printf("pullThread - stopped Touching Trigger: %d\n", triggerTouchValue);
    }

    delay(Softair::threadSetTriggerPulledStateRoutineSleepDuration);
  }
}

void* Softair::shootOnTouchAndTriggerRoutine(void* param) {
  while (true) {
    if (Softair::triggerEnabledByTouch && Softair::triggerPulledByFinger) {
      shoot(true, fireMode);
    }
    delay(threadShootOnTouchAndTriggerRoutineSleepDuration);  // time to sleep between each iteration
  }
}

// void* Softair::countShotsRoutine(void* param) {
//   bool incrementedCountFlag = false;
//   while (true) {
//     if (digitalRead(pistonSensorReadPin) == 0 && incrementedCountFlag == false) {
//       // shotsFired++;
//       while (digitalRead(pistonSensorReadPin) == 0) {
//         delay(1);  // wait until piston leaves the barrier to prevent multiple increasements in one shot
//       }
//     }
//   }
// }

void* Softair::setSystemSleepStateRoutine(void* param) {}

// ----------------------------------------------------------------------------
// GLOBAL SETTINGS & ADJUSTMENTS
// ----------------------------------------------------------------------------

// TODO: Migrate all these vars into gun class to keep the pre-setup clean
const unsigned int otaPort = 80;

// WebServer
// Allows setting the realm of authentication Default:"Login Required"
const char* www_realm = "Custom Auth Realm";
// the Content of the HTML response in case of Unautherized Access Default:empty
const char* authFailResponse = "Authentication Failed";

WebServer server(otaPort);
const char* serverIndex =
    "<form method='POST' action='/update' enctype='multipart/form-data'><input "
    "type='file' name='update'><input type='submit' value='Update'></form>";

// ----------------------------------------------------------------------------
// THREADS
// ----------------------------------------------------------------------------
// TODO: clean up or use old threading methods (xTaskCreatePinnedToCore)
pthread_t webserverThreadHandle;
SemaphoreHandle_t webServerSemaphore;

// ----------------------------------------------------------------------------
// SETUP
// ----------------------------------------------------------------------------
void setup() {
  // Serial SETUP
  Serial.begin(115200);

  // Creating the Softair Object
  Softair gun("Cyma", "Glock", "18C");  // creation first, so gpio setup can access its attributes

  // Shot sensor
  ledcSetup(Softair::shotChannel, Softair::freq, Softair::resolution);
  ledcAttachPin(Softair::pistonSensorReadPin, Softair::shotChannel);
  pinMode(Softair::pistonSensorReadPin, INPUT_PULLDOWN);
  pinMode(Softair::triggerPullReadPin, INPUT_PULLDOWN);
  pinMode(Softair::motorControlPin, OUTPUT);

  Serial.printf("initial touch: %d, initial pull: %d\n", touchRead(Softair::triggerTouchReadPin), digitalRead(Softair::triggerPullReadPin));

  pixels.begin();
  pixels.setPixelColor(3, pixels.Color(100, 0, 0));
  pixels.setPixelColor(2, pixels.Color(0, 100, 0));
  pixels.setPixelColor(1, pixels.Color(0, 0, 100));
  pixels.setPixelColor(0, pixels.Color(100, 100, 100));
  pixels.show();

  // WIFI SETUP
  unsigned short int counter = 0;
  if (WiFi.status() != WL_CONNECTED) {
    Serial.printf("Connecting to Wifi: %s\n", WIFI_SSID);
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSORD);
    while (WiFi.status() != WL_CONNECTED && counter <= 5) {
      Serial.printf("Wifi Connection Try: %d\n", counter++);
      delay(1000);
    }
    if (counter > 5 && WiFi.status() != WL_CONNECTED) {
      Serial.printf("Failed to connect to Wifi: %s\n", WIFI_SSID);
    } else {
      Serial.printf("Connected to Wifi: %s\n", WIFI_SSID);
    }
  }

  // THREAD SETUP
  disableCore0WDT();  // Disable WatchDogTimeout, so threads can run as long as they want
  disableCore1WDT();  // Same goes for the second core

  delay(5000);
  Serial.printf("%s\n", time());

  gettimeofday time asctime clock ctime difftime gmtime localtime mktime strftime

      // Thread Creations
      if (pthread_create(&gun.setTriggerTouchedStateRoutineThreadHandle, NULL, gun.setTriggerTouchedStateRoutine, NULL) == 0)
          Serial.println("Thread setTriggerTouchedStateRoutine successfully started\n");
  else Serial.println("Thread setTriggerTouchedStateRoutine Failed\n");

  if (pthread_create(&gun.setTriggerPulledStateRoutineThreadHandle, NULL, gun.setTriggerPulledStateRoutine, NULL) == 0)
    Serial.println("Thread setTriggerPulledStateRoutine successfully started\n");
  else
    Serial.println("Thread setTriggerPulledStateRoutine failed\n");

  // if (pthread_create(&gun.countShotsRoutineThreadHandle, NULL, gun.countShotsRoutine, NULL) == 0)
  //   Serial.println("Thread countShotsRoutine successfully started");
  // else
  //   Serial.println("Thread countShotsRoutine failed");

  if (pthread_create(&gun.shootOnTouchAndTriggerRoutineThreadHandle, NULL, gun.shootOnTouchAndTriggerRoutine, NULL) == 0)
    Serial.println("Thread shootOnTouchAndTriggerRoutine successfully started");
  else
    Serial.println("Thread shootOnTouchAndTriggerRoutine failed");

  // if (pthread_create(&gun.setSystemSleepStateRoutineThreadHandle, NULL, gun.setSystemSleepStateRoutine, NULL) == 0)
  //   Serial.println("Thread setSystemSleepStateRoutine successfully started");
  // else
  //   Serial.println("Thread setSystemSleepStateRoutine failed");

  // if (pthread_create(&webserverThreadHandle, NULL, webServerThreadFunction, NULL) == 0)
  //   Serial.println("Thread WebServer successfully started");
  // else
  //   Serial.println("Thread WebServer failed");
}

// ----------------------------------------------------------------------------
// MAIN LOOP
// ----------------------------------------------------------------------------
void loop() {}

// ----------------------------------------------------------------------------
// Functions running as threads
// ---
// pthread_create(&pthread_t, NULL, callback_function, type * input_argument)
// ----------------------------------------------------------------------------

void* webServerThreadFunction(void* param) {
  // Webserver for OTA Updates
  MDNS.begin(WIFI_HOSTNAME);
  ArduinoOTA.begin();
  server.on("/", []() {
    if (!server.authenticate(www_username, www_password)) {
      // Basic Auth Method with Custom realm and Failure Response
      // return server.requestAuthentication(BASIC_AUTH, www_realm,
      // authFailResponse); Digest Auth Method with realm="Login Required" and
      // empty Failure Response return server.requestAuthentication(DIGEST_AUTH);
      // Digest Auth Method with Custom realm and empty Failure Response
      // return server.requestAuthentication(DIGEST_AUTH, www_realm);
      // Digest Auth Method with Custom realm and Failure Response
      return server.requestAuthentication(DIGEST_AUTH, www_realm, authFailResponse);
    }
    server.sendHeader("Connection", "lel", "close");
    server.send(200, "text/html", serverIndex);
    server.on("/", HTTP_GET, []() {
      server.sendHeader("Connection", "close");
      server.send(200, "text/html", serverIndex);
    });
    server.on(
        "/update", HTTP_POST,
        []() {
          server.sendHeader("Connection", "close");
          server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
          ESP.restart();
        },
        []() {
          HTTPUpload& upload = server.upload();
          if (upload.status == UPLOAD_FILE_START) {
            Serial.setDebugOutput(true);
            Serial.printf("Update: %s\n", upload.filename.c_str());
            if (!Update.begin()) {  // start with max available size
              Update.printError(Serial);
            }
          } else if (upload.status == UPLOAD_FILE_WRITE) {
            if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
              Update.printError(Serial);
            }
          } else if (upload.status == UPLOAD_FILE_END) {
            if (Update.end(true)) {  // true to set the size to the current progress
              Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
            } else {
              Update.printError(Serial);
            }
            Serial.setDebugOutput(false);
          } else {
            Serial.printf(
                "Update Failed Unexpectedly (likely broken connection): "
                "status=%d\n",
                upload.status);
          }
        });
  });
  server.begin();
  MDNS.addService("http", "tcp", otaPort);
  Serial.printf("Open http://%s:%i\n", WiFi.localIP(), otaPort);
  while (true) {
    server.handleClient();
    ArduinoOTA.handle();
  }
}

// TODO: 3D Print barrel cover

// TODO: implement deepsleep mode (sleep when no shot is detected for 1 minute) and

// TODO: add led's to look like a tritium sight (with configurable brightness?
// using photoresistor?)

// TODO: aim sight dots should light up red, as long as no finger is touching the tpuch pin,
// green while touch pin is being touched by human skin and low blue, when in deepsleep mode

// TODO: when touching the trigger, wake the esp from deepsleep mode

// TODO: export gun clas / methods etc and include in Sektch.ino to keep the sketch clean, header file should contain declerations only and cpp file the implementation
// => header file gives an idea of how the system works

// TODO: rewrite wifi connector to be threaded and non-blocking (offline data should be collected and pushed to connected platform as soon as connection is available again)

// TODO: Switch to ssl secured webserver and create self signed certificates to secure identity

// --------------------------------------------------------------------------------------------------------------------------------------------------------
// CONCEPTS to check
// --------------------------------------------------------------------------------------------------------------------------------------------------------

// pthread_mutex_t mutex;

// pthread_mutex_lock(&mutex);
// Serial.println("mutex locked");
// pthread_mutex_unlock(&mutex);
// Serial.println("mutex unlocked");

// xTaskHandle Task1;
// xTaskCreatePinnedToCore(&triggerThreadTask, "triggerThread", 10000, NULL, 1, NULL, 0);
