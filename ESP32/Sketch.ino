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

// Credentials
#include "Credentials/OtaLogin.h"
#include "Credentials/Wifi.h"

// ----------------------------------------------------------------------------
// SETTINGS
// ----------------------------------------------------------------------------

int otaPort = 80;

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

// Setting debounce delay for mechanical trigger switch/button
int triggerDebounceDelay = 120;

int shotNumber = 0;
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
//  FUNCTION DECLARATIONS
// ----------------------------------------------------------------------------
void shoot();

// ----------------------------------------------------------------------------
// THREADS
// ----------------------------------------------------------------------------

// xTaskHandle Task1;
// xTaskHandle Task2;

void triggerThreadTask(void* p) {
  while (true) {
  }
}

// xTaskCreateUniversal(&triggerThreadTask, "triggerThread", 10000, NULL, 1,
// NULL,0);

void triggerThreadTask();
void shotdetectionThreadTask();
void shootThreadTask();

void webserverThreadTask();

// ----------------------------------------------------------------------------
// SETUP
// ----------------------------------------------------------------------------
void setup() {
  // Serial SETUP
  Serial.begin(115200);
  Serial.println(xPortGetCoreID());
  // GPIO SETUP

  // shotsensor
  ledcSetup(shotChannel, freq, resolution);
  ledcAttachPin(shotSensorPin, shotChannel);
  pinMode(shotSensorPin, INPUT_PULLDOWN);
  pinMode(triggerPin, INPUT_PULLDOWN);
  pinMode(firePin, OUTPUT);

  // Wifi Setup
  Serial.println("Waiting for Wifi.");
  WiFi.begin(WIFI_SSID, WIFI_PASSORD);
  while (WiFi.status() != WL_CONNECTED) {  // Wait for Wifi connection
  }
  Serial.printf("Connected to Wifi: %s", WIFI_SSID);

  // Webserver for OTA Updates
  MDNS.begin(WIFI_HOSTNAME);
  ArduinoOTA.begin();

  server.on("/", []() {
    if (!server.authenticate(www_username, www_password))
    // Basic Auth Method with Custom realm and Failure Response
    // return server.requestAuthentication(BASIC_AUTH, www_realm,
    // authFailResponse); Digest Auth Method with realm="Login Required" and
    // empty Failure Response return server.requestAuthentication(DIGEST_AUTH);
    // Digest Auth Method with Custom realm and empty Failure Response
    // return server.requestAuthentication(DIGEST_AUTH, www_realm);
    // Digest Auth Method with Custom realm and Failure Response
    {
      return server.requestAuthentication(DIGEST_AUTH, www_realm,
                                          authFailResponse);
    }
    server.sendHeader("Connection, lel", "close");
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
            if (Update.write(upload.buf, upload.currentSize) !=
                upload.currentSize) {
              Update.printError(Serial);
            }
          } else if (upload.status == UPLOAD_FILE_END) {
            if (Update.end(
                    true)) {  // true to set the size to the current progress
              Serial.printf("Update Success: %u\nRebooting...\n",
                            upload.totalSize);
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
  Serial.printf("Open http://%s:%i", WiFi.localIP(), otaPort);
}

// ----------------------------------------------------------------------------
// MAIN LOOP
// ----------------------------------------------------------------------------
void loop() {
  Serial.println("entering sensor loop now");
  while (true) {
    ArduinoOTA.handle();
    server.handleClient();
    if (digitalRead(triggerPin) == 1) {
      delay(triggerDebounceDelay / 2);

      if (digitalRead(triggerPin) == 1) {
        delay(triggerDebounceDelay / 2);
        Serial.printf("Trigger Pulled - %d\n", ++triggerNumber);
        if (digitalRead(triggerPin) == 1) {
          Serial.println("Shooting now..");
        }
        while (digitalRead(triggerPin) == 1) {
          shoot(true, "semi");
        }
        delay(triggerDebounceDelay / 3.5);
        if (digitalRead(triggerPin) == 0) {
          shoot(false, "semi");
          Serial.println("Stopped shooting.");
          continue;
        }
      }
    }
  }
}

// ----------------------------------------------------------------------------
// Actions
// ----------------------------------------------------------------------------
void shoot(boolean state, String automaticMode) {
  if (state) {
    // if (digitalRead(shotSensorPin) == 1) {
    digitalWrite(firePin, 1);
  } else {
    digitalWrite(firePin, 0);
  }
  // }
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

// integrate led's to look like a tritium sight (with configurable brightness?
// using photoresistor?)