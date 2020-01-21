// Libraries
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
// #include <queue.h>
// #include "task.h"
// #include <FreeRTOS.h>
// #include <semphr.h>

// Credentials
#include "Credentials/OtaLogin.h"
#include "Credentials/Wifi.h"

using namespace softair;
class Softair {
 public:
  String manufacturer = "";
  String brand = "";
  String model = "";

  const unsigned short int pistonSensorReadPin = 34;  // Read: 1 = infrared barrier free, 0 = IR interrupted
  const unsigned short int triggerPullReadPin = 35;   // Read: 1 = Shoot, 0 = Stop
  const unsigned short int triggerTouchReadPin = 4;
  const unsigned short int motorControlPin = 33;  // HIGH = shoot

  bool triggerEnabledByTouch = false;
  bool triggerPulledByFinger = false;

  unsigned long int shotsFired = 0;
  const unsigned short int burstShootCount = 3;
  const unsigned int touchDetectionThreshold = 18;
  const unsigned int debounceCount = 10;
  const unsigned short int debounceStableTimeUntilShoot = 10;  // Time for a button state to persist until it is considered as settled (not bouncing anymore)

  // Thread Adjustments
  const short int threadSetTriggerTouchedStateReactiveSleepDuration = 1;  // in ms
  const short int threadSetTriggerPulledStateReactiveSleepDuration = 1;   // in ms
  const short int threadShootOnTouchAndTriggerSleepDuration = 1;          // in ms

  // Constructor
  Softair(string manufacturer, string brand, string model);

  // Actions

  void shoot(boolean state, String shootMode = "");  // sets ShootPin's output HUGH, shootMode is optional

  // Threaded reactive attributes / routines

  void* setTriggerTouchedStateReactive(void* param);  // Trigger capacitive touch listener

  void* setTriggerPulledStateReactive(void* param);  // Trigger pull listener

  void* shootOnTouchAndTrigger(void* param);  // Motor GPIO state setter

  void* countShots(void* param);  // counts the interruptions between IR transmitter and receiver

  void* setSystmSleepState(void* param);  // sleeping if softair idle for spec. seconds, wake on touch pin touched
};

// ----------------------------------------------------------------------------
/**
 * Implementation of declared Methods & Constructors:
 * Softair should be completely defined before Setup() kicks in, as pins and functions declared in Softair Object are used by functions in Setup()
 */
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// Softair Constructors
// ----------------------------------------------------------------------------

Softair::Softair(string manufacturer, string brand, string model) {
  manufacturer = manufacturer;
  brand = brand;
  model = model;
}

// ----------------------------------------------------------------------------
// Softair Methods (outside of class to keep the class clean and readable)
// ----------------------------------------------------------------------------
void Softair::shoot(boolean state, String shootMode = "") {
  if (state) {
    delay(triggerDebounceDelay / 2);      // debouncing here enables a simple if-else-conditoin before shooting
    if (shootMode == "semi-automatic") {  // registers one shot using speed sensor, delays to complete shot and stops
      while (digitalRead(pistonSensorReadPin) == HIGH) {
        digitalWrite(motorControlPin, HIGH);
      }
      delay(10);
      digitalWrite(motorControlPin, LOW);
      Serial.printf("Shot once - %d\n", ++shotNumber);
    }

    else if (shootMode == "full-automatic") {  // shoots until either trigger is released or finger stopped touching trigger
      while (digitalRead(triggerPullReadPin) == HIGH && touchRead(4) <= touchDetectionThreshold) {
      continueShooting:
        digitalWrite(motorControlPin, HIGH);
      }
      delay(triggerDebounceDelay / 4);
      if (digitalRead(triggerPullReadPin) == LOW || touchRead(4) >= touchDetectionThreshold) {
        digitalWrite(motorControlPin, LOW);
      }
    }

    else if (shootMode == "burst") {
      // TODO: implement burst mode

    }

    else {
      goto continueShooting;
    }
  }
}
// ----------------------------------------------------------------------------
// Threaded Stuff
// ----------------------------------------------------------------------------

void* Softair::setTriggerTouchedStateReactive(void* param) {
  // Flags to prevent writing the value of setTriggerTouchedStateReactive over and over again while it is already enabled
  bool setEnabledFlag = false;
  bool setDisabledFlag = false;

  while (true) {
    if (touchRead(triggerTouchReadPin) >= touchDetectionThreshold && setEnabledFlag == false) {
      triggerEnabledByTouch = true;
      setEnabledFlag = true;
      setDisabledFlag = false;
      Serial.printf("Trigger Touching");
    } else if (touchRead(triggerTouchReadPin) < touchDetectionThreshold && setDisabledFlag == false) {
      triggerEnabledByTouch = false;
      setEnabledFlag = false;
      setDisabledFlag = true;
      Serial.printf("Trigger Touching Stopped");
    }
    sleep(threadSetTriggerTouchedStateReactiveSleepDuration);  // time to sleep between each iteration
  }
}

void* Softair::setTriggerPulledStateReactive(void* param) {
  // Flags to prevent writing the value of setTriggerTouchedStateReactive over and over again while it is already
  // enabled
  bool setEnabledFlag = false;
  bool setDisabledFlag = false;
  unsigned int currentCpuClockspeed = rtc_clk_cpu_freq_get();

  clock_t startHigh;
  clock_t endHigh;
  clock_t startLow;
  clock_t endLow;

  while (true) {
    if (triggerEnabledByTouch) {
      startHigh = clock();
      while (digitalRead(triggerPullReadPin) == HIGH) {                                               // Debouncing begins from here
        if (((clock() - startHigh) * 1000 / currentCpuClockspeed) >= debounceStableTimeUntilShoot) {  // check if elapsed time == debounceStableTimeUntilShoot
        }
      }
      endHigh
    }
  }

  delay(triggerDebounceDelay / 2);      // debouncing here enables a simple if-else-conditoin before shooting
  if (shootMode == "semi-automatic") {  // registers one shot using speed sensor, delays to complete shot and stops
    while (digitalRead(pistonSensorReadPin) == HIGH) {
      digitalWrite(motorControlPin, HIGH);
    }
    delay(10);
    digitalWrite(motorControlPin, LOW);
    Serial.printf("Shot once - %d\n", ++shotNumber);
  }

  else if (shootMode == "full-automatic") {  // shoots until either trigger is released or finger stopped touching trigger
    while (digitalRead(triggerPullReadPin) == HIGH && touchRead(4) <= touchDetectionThreshold) {
    continueShooting:
      digitalWrite(motorControlPin, HIGH);
    }
    delay(triggerDebounceDelay / 4);
    if (digitalRead(triggerPullReadPin) == LOW || touchRead(4) >= touchDetectionThreshold) {
      digitalWrite(motorControlPin, LOW);
    }
  }
}

void* Softair::shootOnTouchAndTrigger(void* param) {
  while (true) {
    if (touchRead(4) <= touchDetectionThreshold) {  // Trigger touched by skin

      // TODO read shootMode first with digitalRead(buttonsResponsibleForShootMode)

      if (digitalRead(triggerPullReadPin) == HIGH) {
        Serial.printf("touched and triggered - %d\n", i++);
        shoot(true, "semi-automatic");
      }
    }
    sleep(threadShootOnTouchAndTriggerSleepDuration);  // time to sleep between each iteration
  }
}

void* Softair::countShots(void* param) {
  bool incrementedCountFlag = false;
  while (true) {
    if (digitalRead(pistonSensorReadPin) == 0 && incrementedCountFlag == false) {
      shotsFired++;
      while (digitalRead(pistonSensorReadPin) == 0) {
        sleep(0.1);  // wait until piston leaves the barrier to prevent multiple increasements in one shot
      }
    }
  }
}

// ----------------------------------------------------------------------------
// SETTINGS & ADJUSTMENTS
// ----------------------------------------------------------------------------

// TODO: Migrate all these vars into softair class to keep the pre-setup clean
const unsigned int otaPort = 80;

unsigned int i = 0;

// setting PWM properties
const int freq = 5000;
const int shotChannel = 0;  // Shot Sensor
const int resolution = 8;   // 8 Bits resulting in 0-255 as Range for Duty Cycle

// Setting debounce delay for mechanical trigger switch/button
int triggerDebounceDelay = 120;

int shotNumber = 0;  // TODO: DELETE (replace by shotsFired)
int triggerNumber = 0;

// WebServer
// allows you to set the realm of authentication Default:"Login Required"
const char* www_realm = "Custom Auth Realm";
// the Content of the HTML response in case of Unautherized Access Default:empty
String authFailResponse = "Authentication Failed";

WebServer server(otaPort);
const char* serverIndex =
    "<form method='POST' action='/update' enctype='multipart/form-data'><input "
    "type='file' name='update'><input type='submit' value='Update'></form>";

// ----------------------------------------------------------------------------
// THREADS
// ----------------------------------------------------------------------------
pthread_t triggerThreadHandle;
pthread_t shotdetectionThreadHandle;
pthread_t shootThreadHandle;
pthread_t webserverThreadHandle;
pthread_t setTriggerTouchedStateReactiveThreadHandle;

SemaphoreHandle_t triggerSemaphore;
SemaphoreHandle_t shotdetectionSemaphore;
SemaphoreHandle_t shootSemaphore;
SemaphoreHandle_t webServerSemaphore;

// pthread_mutex_t mutex;

// pthread_mutex_lock(&mutex);
// Serial.println("mutex locked");
// pthread_mutex_unlock(&mutex);
// Serial.println("mutex unlocked");

xTaskHandle Task1;
int threadCreationResult;
// xTaskHandle Task2;

// xTaskCreatePinnedToCore(&triggerThreadTask, "triggerThread", 10000, NULL, 1,
//                         NULL, 0);

// ----------------------------------------------------------------------------
// SETUP
// ----------------------------------------------------------------------------
void setup() {
  // Serial SETUP
  Serial.begin(115200);

  // Creating the Softair Object
  Softair softair();  // creation first, so gpio setup can access its attributes

  // GPIO SETUP

  // shotsensor
  ledcSetup(shotChannel, freq, resolution);
  ledcAttachPin(softair.pistonSensorReadPin, shotChannel);
  pinMode(softair.pistonSensorReadPin, INPUT_PULLDOWN);
  pinMode(softair.triggerPullReadPin, INPUT_PULLDOWN);
  pinMode(softair.motorControlPin, OUTPUT);

  // WIFI SETUP
  Serial.println("Waiting for Wifi.");
  WiFi.begin(WIFI_SSID, WIFI_PASSORD);
  while (WiFi.status() != WL_CONNECTED) {  // Wait for Wifi connection
  }
  Serial.printf("Connected to Wifi: %s\n", WIFI_SSID);

  // THREADS SETUP

  // Thread configurations
  disableCore0WDT();
  disableCore1WDT();

  Serial.print("shotSensor state: normally");
  Serial.println(digitalRead(pistonSensorReadPin));

  // Thread Creations
  if (pthread_create(&setTriggerTouchedStateReactiveThreadHandle, NULL, softair.setTriggerTouchedStateReactive, NULL) == 0) {
    Serial.println("Thread setTriggerTouchedStateReactive successfully started");
  }
  if (pthread_create(&shootThreadHandle, NULL, shootOnTouchAndTrigger, NULL) == 0) {
    Serial.println("Thread shootFunctionality successfully started");
  }
  if (pthread_create(&webserverThreadHandle, NULL, webServerThreadFunction, NULL) == 0) {
    Serial.println("Thread WebServer successfully started");
  }
}

// ----------------------------------------------------------------------------
// MAIN LOOP
// ----------------------------------------------------------------------------
void loop() {}

// ----------------------------------------------------------------------------
// Functions running as threads
// ---
// pthread_create(&pthread_t, NULL, function_to_be_executed, void* input_argument)
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
    server.sendHeader("Connection, lel", "close");
    server.send(200, "text/html", serverIndex);

    server.on("/", HTTP_GET, []() {
      server.sendHeader("Connection", "close");
      server.send(200, "text/html", serverIndex);
    });
    server.on("/update", HTTP_POST,
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

// Todo:
// Fix String & string, boolean & bool

// 3D Print barrel cover

// implement deepsleep mode (sleep when no shot is detected for 1 minute) and
// connect led's to gpios, so glock lights up a red status led to inidcate
// deepsleep and green when awake

// when touching the trigger, wake the esp from deepsleep mode

// integrate led's to look like a tritium sight (with configurable brightness?
// using photoresistor?)