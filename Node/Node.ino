#include <WiFi.h>
#include <WiFiManager.h> 
#include <FirebaseESP32.h>
#include <DHT.h>

#define DATABASE_URL "https://webcr7-ed8b1-default-rtdb.firebaseio.com"
#define DATABASE_SECRET "AIzaSyDLBnMV4FkvaujtNfcZDCSCUg_wrDkjNag"

#define LED_CONNECTED   22
#define LED_NOCONNECTED 21
#define RESET           23

#define FLAME_SENSOR 27
#define DHT_PIN 33
#define DHT_TYPE DHT11
#define MQ2_SENSOR 34
DHT dht(DHT_PIN, DHT_TYPE);
bool NewState_ResetButton, LastState_ResetButton, ResetState;

unsigned long CurrentCountAfterPressResetButton, StartToCountAfterPressResetButton;
FirebaseData fbdo;
WiFiManager wm;
bool res;
float TempValue;
uint8_t GasValue;
bool FireState, FireOffApproval;
unsigned long LastTimetoSendData, LastTimetoCheckWifi, FireLastTime;
/* -------------------- END function 4 config WIFI -------------------- */


/*==================================== SETUP ======================================= */
void setup(){
  Serial.begin(115200);
  /* for button */
  pinMode(RESET,INPUT_PULLUP);

  pinMode(FLAME_SENSOR, INPUT);
  pinMode(MQ2_SENSOR, INPUT);
  pinMode(DHT_PIN, INPUT);
  dht.begin();  

  pinMode(LED_CONNECTED, OUTPUT);
  pinMode(LED_NOCONNECTED, OUTPUT);
 
  
  Serial.println("NO WIFI");
  /* running config WIFI function */
  digitalWrite(LED_CONNECTED, LOW);
  digitalWrite(LED_NOCONNECTED, HIGH);
  Func_ConfigWifi();

  /* connect to Firebase after successful connect wifi */
  Firebase.begin(DATABASE_URL, DATABASE_SECRET);
}
/*==================================== END SETUP ====================================== */


/*==================================== MAIN =========================================== */
void loop() {

    ReadData();
    if(millis() - LastTimetoSendData > 3000){
      LastTimetoSendData = millis();
      SetDataROOM2();
    }
    
    if(millis() - LastTimetoCheckWifi > 10000){
      LastTimetoCheckWifi = millis();
        if (WiFi.status() != WL_CONNECTED) {
          Serial.println("Reconnecting to the Internet");
          WiFi.reconnect();
          digitalWrite(LED_CONNECTED, LOW);
          digitalWrite(LED_NOCONNECTED, HIGH);
        }
        if (WiFi.status() == WL_CONNECTED) {
          Serial.println("Reconnected to the Internet");
          digitalWrite(LED_CONNECTED, HIGH);
          digitalWrite(LED_NOCONNECTED, LOW);
        }
    }
       

    /* Reset to config wifi */
    LastState_ResetButton = NewState_ResetButton;   // save old state before get from button
    NewState_ResetButton = digitalRead(RESET);
    if (NewState_ResetButton == LOW) {
      if (LastState_ResetButton == HIGH && NewState_ResetButton == LOW) {
        StartToCountAfterPressResetButton = millis();
        ResetState = 1;
      }

      if (LastState_ResetButton == LOW && NewState_ResetButton == LOW) {    // Keep pressing Reset button in 3000 ms
        CurrentCountAfterPressResetButton = millis();
        if (CurrentCountAfterPressResetButton - StartToCountAfterPressResetButton > 3000) {
          if (ResetState) {
            WiFiManager wm;
            wm.resetSettings();
            ESP.restart();
            //Func_ConfigWifi();
            ResetState = 0;
          }
        }
      }
    }
}
/*==================================== END MAIN ====================================== */

void ReadData(){
  TempValue = dht.readTemperature();
  GasValue = map(analogRead(MQ2_SENSOR), 0, 4095, 0, 100);
  if(digitalRead(FLAME_SENSOR) == 1){
    FireState = 0;
    if((FireOffApproval) && (millis() - FireLastTime > 30000)){
      Firebase.setBool(fbdo, "/ROOM2/SENSORS/Fire", FireState);
      FireOffApproval = 0;
    }
  } 
  else
  {
    FireState = 1;
    Firebase.setBool(fbdo, "/ROOM2/SENSORS/Fire", FireState);
    FireOffApproval = 1;
    FireLastTime= millis();
  }
}
void SetDataROOM2(){
  Firebase.setInt(fbdo, "/ROOM2/SENSORS/Gas", GasValue); // SET gas value 
  Firebase.setFloat(fbdo, "/ROOM2/SENSORS/Temperature", TempValue );//set temp threshold  on firebase
  
}
void Func_ConfigWifi(){
  res = wm.autoConnect("NODE_VT3","79797979");
  if(!res) {
    /**/
  } 
  else {
    Serial.println("connected...yeey ");

  }
}














