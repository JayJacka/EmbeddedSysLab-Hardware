// /*
// Copyright (c) 2021 Jakub Mandula

// Example of using one PZEM module with Hardware Serial interface.
// ================================================================

// If desired, a HardwareSerial handle can be passed to the constructor
// which will then be used for the communication with the module.

// Note that ESP32 HardwareSerial must also be provided with the RX and TX
// pins.

// */

#include <PZEM004Tv30.h>


#if !defined(PZEM_RX_PIN) && !defined(PZEM_TX_PIN)
#define PZEM_RX_PIN 32
#define PZEM_TX_PIN 33
#endif

#if !defined(PZEM_SERIAL)
#define PZEM_SERIAL Serial2
#endif


#if defined(ESP32)
/*************************
 *  ESP32 initialization
 * ---------------------
 * 
 * The ESP32 HW Serial interface can be routed to any GPIO pin 
 * Here we initialize the PZEM on Serial2 with RX/TX pins 16 and 17
 */
PZEM004Tv30 pzem(PZEM_SERIAL, PZEM_RX_PIN, PZEM_TX_PIN);
#elif defined(ESP8266)
/*************************
 *  ESP8266 initialization
 * ---------------------
 * 
 * Not all Arduino boards come with multiple HW Serial ports.
 * Serial2 is for example available on the Arduino MEGA 2560 but not Arduino Uno!
 * The ESP32 HW Serial interface can be routed to any GPIO pin 
 * Here we initialize the PZEM on Serial2 with default pins
 */
//PZEM004Tv30 pzem(Serial1);
#else
/*************************
 *  Arduino initialization
 * ---------------------
 * 
 * Not all Arduino boards come with multiple HW Serial ports.
 * Serial2 is for example available on the Arduino MEGA 2560 but not Arduino Uno!
 * The ESP32 HW Serial interface can be routed to any GPIO pin 
 * Here we initialize the PZEM on Serial2 with default pins
 */
PZEM004Tv30 pzem(PZEM_SERIAL);
#endif

// void setup() {
//     // Debugging Serial port
//     Serial.begin(9600);

//     // Uncomment in order to reset the internal energy counter
//     // pzem.resetEnergy()
// }

// void loop() {
//     // Print the custom address of the PZEM
//     Serial.print("Custom Address:");
//     Serial.println(pzem.readAddress(), HEX);

//     // Read the data from the sensor
//     float voltage = pzem.voltage();
//     float current = pzem.current();
//     float power = pzem.power();
//     float energy = pzem.energy();
//     float frequency = pzem.frequency();
//     float pf = pzem.pf();

//         // Print the values to the Serial console
//         Serial.print("Voltage: ");      Serial.print(voltage);      Serial.println("V");
//         Serial.print("Current: ");      Serial.print(current);      Serial.println("A");
//         Serial.print("Power: ");        Serial.print(power);        Serial.println("W");
//         Serial.print("Energy: ");       Serial.print(energy,3);     Serial.println("kWh");
//         Serial.print("Frequency: ");    Serial.print(frequency, 1); Serial.println("Hz");
//         Serial.print("PF: ");           Serial.println(pf);

    
//     Serial.println();
//     delay(2000);
// }

#include <SoftwareSerial.h>
#include <WiFiManager.h>
#include <ArduinoHttpClient.h>
#include <Firebase_ESP_Client.h>
WiFiManager wifiManager;

SoftwareSerial STM(16, 17);

#define API_KEY "AIzaSyC5rXutcbLKOy_6muSPZ2pkR7oQHLqDQJE";

#define DATABASE_URL "https://embeddedlab-a82f7-default-rtdb.firebaseio.com/";

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
bool signupOk = false;
long last = 0;
long sendlast = 0;
long interval = 30000;
void setup() {
  Serial.begin(9600);
  STM.begin(9600);

  bool res;
  Serial.println("waiting for connecting wifi...");
  res = wifiManager.autoConnect("AntiPunPun","embedded123");
  Serial.println("connecting connecting wifi...");
  if(!res) {
      Serial.println("Failed to connect");
  } 
  else {
      Serial.println("connected...yeey :)");
  }
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;

  if (Firebase.signUp(&config, &auth, "", "")) {
    Serial.println("firebase ok");
    signupOk = true;
  } else {
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  }

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  Serial.println("TEST");


}

unsigned int rightRotate(unsigned int n, unsigned int d) {
  return (n >> d) | (n << (32 - d));
}

int readInt() {
  unsigned int value = 0;
  for (int i = 0; i < 4; i++) {
    while (!STM.available())
      ;  // poll input until we get a byte
    // value <<= 8;
    value |= STM.read();            // OR in one byte.
    // value = rightRotate(value, 8);  // shift 8 bits
  }
  return (int) value;
}


const int BUFFER_SIZE = 10;
char buf[BUFFER_SIZE];

void loop() {
  long now = millis();
  // Serial.println(now);
  // Serial.println(last);
  if (now - last >= 5000) {
        // Print the custom address of the PZEM
    // Serial.print("Custom Address:");
    // Serial.println(pzem.readAddress(), HEX);

    // Read the data from the sensor
    float voltage = pzem.voltage();
    float current = pzem.current();
    float power = pzem.power();
    float energy = pzem.energy();
    float frequency = pzem.frequency();
    float pf = pzem.pf();

        // Print the values to the Serial console
        Serial.print("Voltage: ");      Serial.print(voltage);      Serial.println("V");
        Serial.print("Current: ");      Serial.print(current);      Serial.println("A");
        Serial.print("Power: ");        Serial.print(power);        Serial.println("W");
        Serial.print("Energy: ");       Serial.print(energy,3);     Serial.println("kWh");
        Serial.print("Frequency: ");    Serial.print(frequency, 1); Serial.println("Hz");
        Serial.print("PF: ");           Serial.println(pf);

    
    Serial.println();
    Serial.println("get");
    STM.print("x");
    if (STM.available() > 0) {
      int temp = readInt();
      int tds = readInt();
      Serial.print("TEMP: ");
      Serial.println(temp);
      Serial.print("TDS: ");
      Serial.println(tds);
      if (temp <= 10 || temp >= 50) ESP.restart();
      if (Firebase.ready() && signupOk && now-sendlast > interval) {
        if (Firebase.RTDB.setInt(&fbdo, "test/temp", temp)) {
          Serial.println("PASSED");
          Serial.println("PATH: " + fbdo.dataPath());
          Serial.println("TYPE: " + fbdo.dataType());
        } else {
        Serial.println("FAILED");
        Serial.println("REASON: " + fbdo.errorReason());
        } 
        if (tds != 0) {
          if (Firebase.RTDB.pushInt(&fbdo, "test/TDS", tds)) {
            Serial.println("PASSED");
            Serial.println("PATH: " + fbdo.dataPath());
            Serial.println("TYPE: " + fbdo.dataType());
          } else {
          Serial.println("FAILED");
          Serial.println("REASON: " + fbdo.errorReason());
          } 
        }
        if (Firebase.RTDB.pushInt(&fbdo, "test/energy", power)) {
            Serial.println("PASSED");
            Serial.println("PATH: " + fbdo.dataPath());
            Serial.println("TYPE: " + fbdo.dataType());
          } else {
          Serial.println("FAILED");
          Serial.println("REASON: " + fbdo.errorReason());
          } 
        sendlast = now;
      }
    }
    
    last = now;
  }
  // delay(3000);
}
