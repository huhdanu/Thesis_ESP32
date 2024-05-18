#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <WiFiManager.h> 
#include <FirebaseESP32.h>
// #include <SoftwareSerial.h>
#include <DHT.h>
#include <HardwareSerial.h>

#define simSerial               Serial2
#define MCU_SIM_BAUDRATE        115200
#define MCU_SIM_TX_PIN              17
#define MCU_SIM_RX_PIN              16


#define DATABASE_URL "https://webcr7-ed8b1-default-rtdb.firebaseio.com"
#define DATABASE_SECRET "AIzaSyDLBnMV4FkvaujtNfcZDCSCUg_wrDkjNag"

// #define DATABASE_URL "https://final-pro-97449-default-rtdb.firebaseio.com/"
// #define DATABASE_SECRET "w46zvJ2MFTUyHnt4TwUW08aqNq9X1NCIEkVGllKL"

/*--------------------OUTPUT DEFINITION OF DEVICES---------------------------------------------------*/
#define PUMP         19
#define LAMP         18
#define ALARM        5

#define LAMP_ROOM2    4
#define ALARM_ROOM2   2
/*--------------------FEEDBACK DEFINITION OF DEVICES---------------------------------------------------*/
#define ACS_PUMP        34 //ok
#define ACS_ALARMROOM1  35 //ok
#define ACS_ALARMROOM2  32 //ok
/*---------------------------*/
#define ACS_LAMPROOM2   25  //???
#define ACS_LAMPROOM1   33 //ok


#define ADCTHRESHOLD  2907 /*threshold to detect whether the LOAD is active*/
/*---------------------------------------------------------------------------------------------------*/
#define FLAME_SENSOR 39
#define DHT_PIN 0
#define DHT_TYPE DHT11
#define MQ2_SENSOR 36
DHT dht(DHT_PIN, DHT_TYPE);
/*---------------------------------------------------------------------------------------------------*/
#define lcdColumns  20
#define lcdRows      4

#define OPTION              13
#define UP                  26
#define DOWN                14
#define BACK                27
#define OK                  12
#define RESET               15// tro ve ban dau chu ko phai reset wifi

#define TRUE                1
#define FALSE               0

/* --------------------------- MACRO ------------------------- */
#define MANUAL              0
#define AUTO                1

#define DIV_ROOM        0
#define LAYER1_MANUAL   1           // default menu in manual mode
#define LAYER2_MANUAL   2           // for select device Pump, ALARM, LAMP
#define LAYER3_MANUAL   3           // for control device ON, OFF

#define LINE_1          1           // select line
#define LINE_2          2           // 
#define LINE_3          3           //

#define ROOM1                     0
#define ROOM2                     1

#define SENSORS         0
#define ACTUATORS       1
#define SETTING         2

#define GASTHRESHOLD     3
#define TEMPTHRESHOLD    4      


/* global variable ------------------------------------------------------------------- */
char g_layerMenu = DIV_ROOM;               // layer menu: main, select device, control device
char g_mode = MANUAL;                           // initial AUTO mode
char g_line = LINE_1;                           // initial LINE 17500
char g_SaveSelLAYER1;                 // save previous menu in layer 1
char g_SaveSelLAYER2;                     // save previous menu in layer 2
char g_SaveSelROOM = ROOM1;

char g_flag = 0;                                // flag for clear 2 under line in setting threshold
bool flag_PumpOFFapproval = 0;
bool flag_AlarmOFFapproval = 0;
bool flag_AlarmRoom2OFFapproval = 0;
bool flag_LampOFFapproval = 0;
bool flag_LampRoom2OFFapproval = 0;
bool OptionMenu = 1; // Manual | Auto
bool Option_pre =1;
bool Option_tmp;

/*  */
unsigned long LastimeToCheckWifi = 0;
unsigned long LastimeToGetData = 0;
unsigned long LastimeToSetDevice = 0;
unsigned long LastimeToGetDataRoom1 = 0;
unsigned long LastimeToSetDeviceState = 0;
unsigned long LastimeToSetAlarmStateRoom1 = 0;
unsigned long LastimeToSetAlarmStateRoom2 = 0;
unsigned long LastimeToSetLampStateRoom1 = 0;
unsigned long LastimeToSetLampStateRoom2 = 0;
unsigned long LastimeChangeState = 0;
unsigned long LastCall = 0;
unsigned long lastTime = 0;
/*-----------------------*/
// uint16_t ADC_Pump = 0;
// uint16_t ADC_Pump_Average = 0;

/*  */
String phoneNumber = "0772488647";
bool PhoneCallState;
String response;
bool ConfigWifi;

bool NewState_ResetButton = 1;
bool LastState_ResetButton = 1;
bool ResetState;
bool FirstimeToDisplay;

/* flag_data for ?? */
bool flag_data = 1;
bool flag_ShowConfirmThreshold = FALSE; 
bool flag_SentData2Firebase = FALSE;

bool SelectRoom;                /* which ROOM be selected */

/*------------------------------------ variable in ROOM1*/ 
float TempValue, TempValue_tmp;

uint8_t GasValue, GasValue_tmp;
uint8_t GasThreshold, TempThreshold ;        // variable for big number
uint8_t GasThreshold_tmp, TempThreshold_tmp ;

uint8_t pre_Value = 0;          // variable for save data threshold before be changed
bool FireState;
// uint8_t g_ValueGAS;
// uint8_t g_ValueTEMP;
// g_ValueGAS = GasThreshold;
// g_ValueTEMP = TempThreshold;
/*------------------------------------------------*/


/*----------------------------------- variable in ROOM2*/
float TempValueRoom2, TempValue_tmpRoom2;
uint8_t GasValueRoom2, GasValue_tmpRoom2;

uint8_t GasThresholdRoom2, TempThresholdRoom2 ;
uint8_t GasThreshold_tmpRoom2, TempThreshold_tmpRoom2 ;

bool FireStateRoom2;
/*------------------------------------------------*/


bool AutoMenuTransitionState; // Menu Room1 -> MenuRoom2

unsigned long StartToCountAfterPressResetButton;
unsigned long CurrentCountAfterPressResetButton;
unsigned long LastTimetoDetectFire;
bool PumpState, LampState, AlarmState;
bool PumpState_tmp, LampState_tmp, AlarmState_tmp;               // firebase state
bool PumpStateRoom2, LampStateRoom2, AlarmStateRoom2;
bool PumpStateRoom2_tmp, LampStateRoom2_tmp, AlarmStateRoom2_tmp;

bool Pump_PreState = 0;
bool Lamp_PreState = 0;
bool Alarm_PreState = 0;
bool LampRoom2_PreState = 0;
bool AlarmRoom2_PreState = 0;


/* this flag use for controlling the value of "PhoneState", if PhoneState is TRUE-> has really detected fire and notify user AND
just in case THE user decline to anser AND
This flag will be equal TRUE in NORMAL state (no fire, no gas leak,...) */ 
bool f_EnablePhone = 1; 


unsigned long lastDebounceTime_UP = 0;
unsigned long lastDebounceTime_DW = 0;
unsigned long lastDebounceTime_OK = 0;
unsigned long lastDebounceTime_BACK = 0;
FirebaseData fbdo;
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);
WiFiManager wm;
bool res;

/* ---------------------- function 4 config WIFI ----------------------- */
void Func_ConfigWifi(){
  res = wm.autoConnect("TUHAOVT3","68686868");
  if(!res) {
    /* Serial.println("Failed to connect");
    lcd.setCursor(0, 0);
    lcd.print("FAILED TO CONNECT   ");
    lcd.setCursor(0, 1);
    lcd.print("PLEASE CONFIG YOUR  ");
    lcd.setCursor(0, 2);
    lcd.print("ACCESS POIN         ");
    Serial.println("RES =0 ");
    //ESP.restart();
    */
  } 
  else {
    Serial.println("connected...yeey ");
    lcd.setCursor(0, 0); lcd.print("                    ");
    lcd.setCursor(0, 0); lcd.print("WAITING.............");
    lcd.setCursor(0, 1); lcd.print("                    ");  
    lcd.setCursor(0, 1); lcd.print("THE SYSTEM IS       ");
    lcd.setCursor(0, 2);
    lcd.print("CONNECTING TO WIFI  ");
    lcd.setCursor(0, 3);
    lcd.print("                    ");
    delay(5000);
  }
}
/* -------------------- END function 4 config WIFI -------------------- */


/*==================================== SETUP ======================================= */
void setup(){
  Serial.begin(115200);
  simSerial.begin(MCU_SIM_BAUDRATE, SERIAL_8N1, MCU_SIM_RX_PIN, MCU_SIM_TX_PIN);
  lcd.init();                
  lcd.backlight();

  pinMode(FLAME_SENSOR, INPUT);
  pinMode(MQ2_SENSOR, INPUT);
  pinMode(DHT_PIN, INPUT);
  dht.begin();

  /* for button */
  pinMode(OPTION, INPUT_PULLUP);
  pinMode(UP,INPUT_PULLUP);
  pinMode(DOWN,INPUT_PULLUP);
  pinMode(BACK,INPUT_PULLUP);
  pinMode(OK,INPUT_PULLUP);
  pinMode(RESET,INPUT_PULLUP);
  /* for detected current */
  pinMode(ACS_LAMPROOM2, INPUT);
  pinMode(ACS_PUMP, INPUT);
  pinMode(ACS_ALARMROOM1, INPUT);
  pinMode(ACS_LAMPROOM1, INPUT);
  pinMode(ACS_ALARMROOM2, INPUT);

  /* for devices */
  pinMode(ALARM, OUTPUT);
  pinMode(LAMP, OUTPUT);

  pinMode(ALARM_ROOM2, OUTPUT);
  pinMode(LAMP_ROOM2, OUTPUT);

  pinMode(PUMP, OUTPUT);

  // Initial state devices state in room1
  digitalWrite(ALARM, LOW);
  digitalWrite(LAMP, LOW);

  // Initial devices state in room2
  digitalWrite(ALARM_ROOM2, LOW);
  digitalWrite(LAMP_ROOM2, LOW);

  digitalWrite(PUMP, LOW);

  /* <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<-- INTERRUPT DECLARE -->>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> */

  attachInterrupt(digitalPinToInterrupt(OPTION), pressOPTION, FALLING);
  attachInterrupt(digitalPinToInterrupt(UP), pressUP, FALLING);
  attachInterrupt(digitalPinToInterrupt(DOWN), pressDOWN, FALLING);
  attachInterrupt(digitalPinToInterrupt(BACK), pressBACK, FALLING);
  attachInterrupt(digitalPinToInterrupt(OK), pressOK, FALLING);
  attachInterrupt(digitalPinToInterrupt(RESET), pressRESET, FALLING);

  //Config WIFI from User, [if SSID and PASSWORD are False OR The system have NO SSID and PASSWORD before -> res = 0]
  Serial.println("NO WIFI");
  lcd.setCursor(0, 0);
  lcd.print("    NO CONNECTION   ");
  lcd.setCursor(0, 1);
  lcd.print("PLEASE CONFIG YOUR  ");
  lcd.setCursor(0, 2);
  lcd.print("    ACCESS POINT    ");
    
  /* running config WIFI function */
  Func_ConfigWifi();

  /* connect to Firebase after successful connect wifi */
  Firebase.begin(DATABASE_URL, DATABASE_SECRET);

  /* initialize for variable */
  TempValue = 31;
  GasValue = 31;
  GasThreshold = 70;
  TempThreshold = 50;

  TempValueRoom2 = 31;
  GasValueRoom2 = 31;
  GasThresholdRoom2 = 70;
  TempThresholdRoom2 = 50;
}
/*==================================== END SETUP ====================================== */


/*==================================== MAIN =========================================== */
void loop() {

    /* check auto or manual */
    
    /* Auto Option: OptionMenu == 0 */
    if(!OptionMenu){   

      if (!FirstimeToDisplay){
          MainMenu_Auto();
          FirstimeToDisplay = 1;
          AutoMenuTransitionState = 1;
      }
      /* process after 5 seconds */
      if((millis() - LastimeChangeState) > 6500){
        // LastimeChangeState = millis();
        if(!AutoMenuTransitionState){
          ReadDataRoom1();
          MainMenu_Auto();
          GetThresholdOnlyRoom1();
          SetDataRoom1();
          GetPhoneNumber();

          if(AlarmState){
            SetAlarmStateRoom1();
          }
          else if((!AlarmState) && flag_AlarmOFFapproval){
            SetAlarmStateRoom1();
            flag_AlarmOFFapproval= 0;
          }
          else if(LampState){
            SetLampStateRoom1();
          }
          else if((!LampState) && flag_LampOFFapproval){
            SetLampStateRoom1();
            flag_LampOFFapproval = 0;
          }
          ReCall();          
        }       
        else{ /* AutoMenuTransitionState == 1 */
          MainMenu_AutoRoom2();
          
          GetDataRoom2();
          GetThresholdOnlyRoom2();
          if(AlarmStateRoom2){
            SetAlarmStateRoom2();
          }
          else if((!AlarmStateRoom2) && flag_AlarmRoom2OFFapproval){
            SetAlarmStateRoom2();
            flag_AlarmRoom2OFFapproval = 0;
          }

          if(LampStateRoom2){
            SetLampStateRoom2();
          }
          else if((!LampStateRoom2) && flag_LampRoom2OFFapproval){
            SetLampStateRoom2();
            flag_LampRoom2OFFapproval = 0;
          }
          ReCall();
          
        }
                
        if(PumpState || PumpStateRoom2){
          SetPumpState();
        }
        else if((!PumpState) && (!PumpStateRoom2) && flag_PumpOFFapproval){
          SetPumpState();
          flag_PumpOFFapproval = 0;
        }

        ReCall();
        
        AutoMenuTransitionState = !AutoMenuTransitionState;
        LastimeChangeState = millis();
        ReadDataRoom1();
      }
      ReadDataRoom1();
      GetDeviceStateRoom1();
      GetDeviceStateRoom2();
      GetPumpState();

      /*  */
      if((FireState || FireStateRoom2) && ((GasValue > GasThreshold)||(GasValueRoom2 > GasThresholdRoom2) ||(TempValue >  TempThreshold) ||(TempValueRoom2 > TempThresholdRoom2))){
          PhoneCallState = 1;
      }
      else{
        f_EnablePhone = 1; // in normal state
        PhoneCallState = 0;
      }

      if(PhoneCallState && f_EnablePhone){        
        LastCall = millis();
        callPhone(phoneNumber);
        delay(18);
        f_EnablePhone = 0;       
      }
      ReCall();   
      Serial.print("PhoneCallState is: ");
      Serial.println(PhoneCallState);
      Serial.print("f_EnablePhone is: ");
      Serial.println(f_EnablePhone);
      /*---------------------------------------*/
      Serial.print("AlarmStateRoom1 is: ");
      Serial.println(AlarmState);

      Serial.print("LampStateRoom1 is: ");
      Serial.println(LampState);

      Serial.print("PumpStateRoom1 is: ");
      Serial.println(PumpState);
      /*---------------------------------------*/

      Serial.print("AlarmStateRoom2 is: ");
      Serial.println(AlarmStateRoom2);

      Serial.print("LampStateRoom2 is: ");
      Serial.println(LampStateRoom2);

      Serial.print("PumpStateRoom2 is: ");
      Serial.println(PumpStateRoom2);

    }

    /* MANUAL MODE ----------------------------------->! */
    else{   
        switch (g_layerMenu)
        {

        /* select room */
        case DIV_ROOM:
          DivideRoom();
          f_EnablePhone = 1;
          break;

        /* select option: SENSOR - ACTUATORS - SETTING */
        case LAYER1_MANUAL:                             // default display
            // display menu list in layer 1
            MainMenu_Manual();
            //LCD_SendLine(g_line);
            break;

        case LAYER2_MANUAL:                             // select device Pump, ALARM, LAMP
            switch (g_SaveSelLAYER1)
            {

            /* SENSORS display */
            case SENSORS:                            
                // display menu list SENSORS
                if(SelectRoom == ROOM2){
                  SensorMenuRoom2();
                }
                else SensorMenu();

                break;

            /* ACTUATORS display and select */
            case ACTUATORS:                             
                // display menu list ACTUATORS
                ActuatorMenu();
                break;                                  
                
            /* SETTING display and select modify THRESHOLD of GAS and TEMP */
            case SETTING:                               
                // display menu list SETTING
                SettingMenu();
                break;                                  
            }           
            /* break in case LAYER2_MANUAL */
            break;                                      
        case LAYER3_MANUAL:                // ON - OFF device layer
            switch (g_SaveSelLAYER2)
            {
            case PUMP:
                // display menu Pump
                PumpControlMenu();
                  digitalWrite(PUMP, PumpState | PumpState_tmp | PumpStateRoom2);
                  if(PumpState_tmp || PumpStateRoom2){
                    PumpState_tmp = 0;
                    PumpStateRoom2 = 0;
                    PumpState = 1;
                  }
                  if(Pump_PreState != PumpState) {                
                    /* send data to Firebase function */
                    SetPumpState();
                    Pump_PreState = PumpState;
                  }
                  IsPumpActive();
                break;            
            case ALARM:
                // display menu ALARM
                AlarmControlMenu();
                if(SelectRoom == ROOM1){
                  digitalWrite(ALARM, AlarmState | AlarmState_tmp);
                  if(AlarmState_tmp){
                    AlarmState = AlarmState_tmp;
                    AlarmState_tmp = 0;
                  }
                  if(Alarm_PreState != AlarmState) {
                    SetAlarmStateRoom1();
                    Alarm_PreState = AlarmState;
                  }
                  IsAlarmActiveRoom1();
                }
                /* case in ROOM 2 */
                else{
                  digitalWrite(ALARM_ROOM2, AlarmStateRoom2 | AlarmStateRoom2_tmp);
                  if(AlarmStateRoom2_tmp){
                    AlarmStateRoom2 = AlarmStateRoom2_tmp;
                    AlarmStateRoom2_tmp = 0;
                  }
                  if(AlarmRoom2_PreState != AlarmStateRoom2) {            
                    SetAlarmStateRoom2();
                    AlarmRoom2_PreState = AlarmStateRoom2;
                  }
                  IsAlarmActiveRoom2();                 
                }             
                break;

            case LAMP:
                // display menu LAMP
                LampControlMenu();
                if(SelectRoom == ROOM1){
                  digitalWrite(LAMP, LampState | LampState_tmp);
                  if(LampState_tmp){
                    LampState = LampState_tmp;
                    LampState_tmp = 0;
                  }
                  if(Lamp_PreState != LampState) {
                    SetLampStateRoom1();
                    Lamp_PreState = LampState;
                  }
                  IsLampActiveRoom1();
                }
                /* case in ROOM 2 */
                else{
                  digitalWrite(LAMP_ROOM2, LampStateRoom2 | LampStateRoom2_tmp);
                  if(LampStateRoom2_tmp){
                    LampStateRoom2 = LampStateRoom2_tmp;
                    LampStateRoom2_tmp = 0;
                  }
                  if(LampRoom2_PreState != LampStateRoom2) {
                    SetLampStateRoom2();
                    LampRoom2_PreState = LampStateRoom2;
                  }
                  IsLampActiveRoom2();
                } 
                break;

            case GASTHRESHOLD:
                /* check flag show table confirm */
                if (millis() - LastimeToGetData > 500)
                {
                  LastimeToGetData = millis();
                  (g_SaveSelROOM == ROOM1)?(GetThresholdOnlyRoom1()):(GetThresholdOnlyRoom2());
                }

                if(flag_ShowConfirmThreshold){    // true
                  /* check flag to admit set data to Firebase */
                  ConfirmDataThreshold(); 
                  if(flag_SentData2Firebase){                 // true
                    /* off flag for show table confirm and flag set data to Firebase */
                    flag_SentData2Firebase = FALSE;
                    flag_ShowConfirmThreshold = FALSE;

                    /* send data to Firebase */
                    (g_SaveSelROOM == ROOM1)?(SetThresholdOnlyRoom1()):(SetThresholdOnlyRoom2());
                    /* save data be changed */
                    (g_SaveSelROOM == ROOM1)?(pre_Value = GasThreshold):(pre_Value = GasThresholdRoom2);

                    ShowSendDataSuccesful();
                  }else{                                      // cannot set data
                    /* show table confirm for use select */
                    ConfirmDataThreshold();
                  }
                }else{
                  (g_SaveSelROOM == ROOM1)?(GasThresholdMenu()):(GasThresholdMenuRoom2());
                }
                break;

            case TEMPTHRESHOLD:
                if (millis() - LastimeToGetData > 500)
                {
                  LastimeToGetData = millis();
                  (g_SaveSelROOM == ROOM1)?(GetThresholdOnlyRoom1()):(GetThresholdOnlyRoom2());
                }
                if(flag_ShowConfirmThreshold){
                  /* show table confirm for use select */
                  ConfirmDataThreshold(); 
                  if(flag_SentData2Firebase){
                    /* off flag for show table confirm and flag set data to Firebase */
                    flag_SentData2Firebase = FALSE;
                    flag_ShowConfirmThreshold = FALSE;

                    /* send data to Firebase */
                    (SelectRoom == ROOM1)?(SetThresholdOnlyRoom1()):(SetThresholdOnlyRoom2());
                    (g_SaveSelROOM == ROOM1)?(pre_Value = TempThreshold):(pre_Value = TempThresholdRoom2);
                    ShowSendDataSuccesful();
                  }
                }else{
                  (g_SaveSelROOM == ROOM1)?(TempThresholdMenu()):(TempThresholdMenuRoom2());
                }
                break;
            }

          // break case LAYER3_MANUAL
          break;                                          
        }
    }          
    GetOptionMenu();
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
        if (CurrentCountAfterPressResetButton - StartToCountAfterPressResetButton > 1000) {
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
  SetOptionMenu();
  Option_pre = OptionMenu;
}
/*==================================== END MAIN ====================================== */



/* ========================================== define function =========================================== */
uint8_t custChar[8][8] = {
  {31, 31, 31, 0, 0, 0, 0, 0},      // Small top line - 0
  {0, 0, 0, 0, 0, 31, 31, 31},      // Small bottom line - 1
  { B11111,
    B00000,
    B00000,
    B00000,                         // This shows an alternative
    B00000,                         // way of defining a custome character,
    B00000,                         // a bit more 'visually' perhaps?
    B00000,
    B11111,
  },
  //{31, 0, 0, 0, 0, 0, 0, 31},     // Small lines top and bottom -2
  {0, 0, 0, 0, 0, 0,  0, 31},       // Thin bottom line - 3
  {31, 31, 31, 31, 31, 31, 15, 7},  // Left bottom chamfer full - 4
  {28, 30, 31, 31, 31, 31, 31, 31}, // Right top chamfer full -5
  {31, 31, 31, 31, 31, 31, 30, 28}, // Right bottom chamfer full -6
  {7, 15, 31, 31, 31, 31, 31, 31},  // Left top chamfer full -7
};
/* ------------------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------------------- */
// Define our numbers 0 thru 9
// 254 is blank and 255 is the "Full Block"
uint8_t bigNums[10][6] = {
  {7, 0, 5, 4, 1, 6},         //0
  {0, 5, 254, 1, 255, 1},     //1
  {0, 2, 5, 7, 3, 1},         //2
  {0, 2, 5, 1, 3, 6},         //3
  {7, 3, 255, 254, 254, 255}, //4
  {7, 2, 0, 1, 3, 6},         //5
  {7, 2, 0, 4, 3, 6},         //6
  {0, 0, 5, 254, 7, 254},   //7
  {7, 2, 5, 4, 3, 6},         //8
  {7, 2, 5, 1, 3, 6},         //9
};
/* ------------------------------------------------------------------------------------- */


/* ------------------------------------------------------------------------------------- */
void printBigNum(int number, int startCol, int startRow) {
  // Position cursor to requested position (each char takes 3 cols plus a space col)
  lcd.setCursor(startCol, startRow);

  // Each number split over two lines, 3 chars per line. Retrieve character
  // from the main array to make working with it here a bit easier.
  uint8_t thisNumber[6];
  for (int cnt = 0; cnt < 6; cnt++) {
    thisNumber[cnt] = bigNums[number][cnt];
  }

  // First line (top half) of digit
  for (int cnt = 0; cnt < 3; cnt++) {
    lcd.print((char)thisNumber[cnt]);
  }

  // Now position cursor to next line at same start column for digit
  lcd.setCursor(startCol, startRow + 1);

  // 2nd line (bottom half)
  for (int cnt = 3; cnt < 6; cnt++) {
    lcd.print((char)thisNumber[cnt]);
  }
}
/* ------------------------------------------------------------------------------------- */



/* ------------------------------------------------------------------------------------- */
/* void LCD_SendLine(int row){
  lcd.setCursor(0,row);
  lcd.print("->"); 
} */
/* ------------------------------------------------------------------------------------- */


/* ------------------------------------------------------------------------------------- */
void MainMenu_Manual(){
  lcd.setCursor(0, 0);
  if(!g_SaveSelROOM){
    lcd.print("Manual BED ROOM     "); /* Manual   [STATE]     */
  }
  else lcd.print("Manual KITCHEN ROOM "); /* Manual   [STATE]     */

  switch(g_line){
    case LINE_1:
      lcd.setCursor(0, 1);
      lcd.print("->SENSORS           "); 
      lcd.setCursor(0, 2);
      lcd.print("  ACTUATORS         ");
      lcd.setCursor(0, 3);
      lcd.print("  SETTINGS          ");
      break;
    case LINE_2:
      lcd.setCursor(0, 1);
      lcd.print("  SENSORS           "); 
      lcd.setCursor(0, 2);
      lcd.print("->ACTUATORS         ");
      lcd.setCursor(0, 3);
      lcd.print("  SETTINGS          ");
      break;
    case LINE_3:
      lcd.setCursor(0, 1);
      lcd.print("  SENSORS           "); 
      lcd.setCursor(0, 2);
      lcd.print("  ACTUATORS         ");
      lcd.setCursor(0, 3);
      lcd.print("->SETTINGS          ");
      break;
  }
}

/*-------------------------------------------------------------------------------------
        use to control Setting in menu display
-------------------------------------------------------------------------------------*/
void DivideRoom(){
  lcd.setCursor(0, 0);  lcd.print("Select ROOM         "); 
  switch(g_line){
    case LINE_1:
      lcd.setCursor(0, 1);  lcd.print("->BED ROOM        ");
      lcd.setCursor(0, 2);  lcd.print("  KITCHEN ROOM      ");
      lcd.setCursor(0, 3);  lcd.print("                    ");
      break;
    case LINE_2:
      lcd.setCursor(0, 1);  lcd.print("  BED ROOM        ");
      lcd.setCursor(0, 2);  lcd.print("->KITCHEN ROOM      ");
      lcd.setCursor(0, 3);  lcd.print("                    ");
      break;
  }
}

/*-------------------------------------------------------------------------------------
        use to control Setting in menu display
-------------------------------------------------------------------------------------*/
void SensorMenu(){

  ReadDataRoom1();   
  lcd.setCursor(0, 0);  lcd.print("M       SENSORS   R1"); 
  lcd.setCursor(0, 1);  lcd.print(" GAS                ");/* GAS          [VALUE]  */
  lcd.setCursor(0, 2);  lcd.print(" TEM                ");/* TEMP         [VALUE]  */
  lcd.setCursor(0, 3);  lcd.print(" FIRE               ");/* FIRE         [STATE]  */
  lcd.setCursor(14, 1);  lcd.print(GasValue);
  lcd.setCursor(14, 2);  lcd.print(TempValue);
  if(FireState){
    lcd.setCursor(12, 3);  lcd.print("Detected");
  }
  else{
    lcd.setCursor(12, 3);  lcd.print("No-Fire ");
  }
  if((millis() - LastimeToGetDataRoom1) > 4000){
    SetDataRoom1();
    LastimeToGetDataRoom1 = millis();
  } 
}

/* ------------------------------------------------------------------------------------- */
void SensorMenuRoom2(){

  /* display in LCD */
  lcd.setCursor(0, 0);  lcd.print("M     SENSORS     R2"); 
  lcd.setCursor(0, 1);  lcd.print(" GAS                ");/* GAS          [VALUE]  */
  lcd.setCursor(0, 2);  lcd.print(" TEMP               ");/* TEMP         [VALUE]  */
  lcd.setCursor(0, 3);  lcd.print(" FIRE               ");/* FIRE         [STATE]  */
  lcd.setCursor(14, 1);  lcd.print(GasValueRoom2);
  lcd.setCursor(14, 2);  lcd.print(TempValueRoom2);

  if(FireStateRoom2){
    lcd.setCursor(12, 3);  lcd.print("Detected");
  }
  else{
    lcd.setCursor(12, 3);  lcd.print("No-Fire ");
  }
  if((millis() - LastimeToGetDataRoom1) > 4000){
    /* get data from Firebase */
    GetDataRoom2();
    LastimeToGetDataRoom1 = millis();
  }
}

/*-------------------------------------------------------------------------------------*/
void ActuatorMenu(){
  lcd.setCursor(0, 0);  lcd.print("Manual  ACTUATORS   "); 
  switch(g_line){
    case LINE_1:
      lcd.setCursor(0, 1);  lcd.print("->PUMP               "); /* Pump          [STATE] */
      lcd.setCursor(0, 2);  lcd.print("  ALARM             "); /* ALARM        [STATE] */
      lcd.setCursor(0, 3);  lcd.print("  LAMP              "); /* LAMP         [STATE] */
      break;
    case LINE_2:
      lcd.setCursor(0, 1);  lcd.print("  PUMP               "); /* Pump          [STATE] */
      lcd.setCursor(0, 2);  lcd.print("->ALARM             "); /* ALARM        [STATE] */
      lcd.setCursor(0, 3);  lcd.print("  LAMP              "); /* LAMP         [STATE] */
      break;
    case LINE_3:
      lcd.setCursor(0, 1);  lcd.print("  PUMP               "); /* Pump          [STATE] */
      lcd.setCursor(0, 2);  lcd.print("  ALARM             "); /* ALARM        [STATE] */
      lcd.setCursor(0, 3);  lcd.print("->LAMP              "); /* LAMP         [STATE] */
      break;
  }
}

/*-------------------------------------------------------------------------------------*/
void PumpControlMenu(){
  lcd.setCursor(0, 0);  lcd.print("Manual  PUMP         "); 
  switch(g_line){
    case LINE_1:
      lcd.setCursor(0, 1);  lcd.print("->ON                ");
      lcd.setCursor(0, 2);  lcd.print("  OFF               ");
      lcd.setCursor(0, 3);  lcd.print("                    ");
      break;
    case LINE_2:
      lcd.setCursor(0, 1);  lcd.print("  ON                ");
      lcd.setCursor(0, 2);  lcd.print("->OFF               ");
      lcd.setCursor(0, 3);  lcd.print("                    ");
      break;
  }
}

/*-------------------------------------------------------------------------------------*/
void AlarmControlMenu(){
  lcd.setCursor(0, 0);  lcd.print("Manual  ALARM       "); 
  switch(g_line){
    case LINE_1:
      lcd.setCursor(0, 1);  lcd.print("->ON                ");
      lcd.setCursor(0, 2);  lcd.print("  OFF               ");
      lcd.setCursor(0, 3);  lcd.print("                    ");
      break;
    case LINE_2:
      lcd.setCursor(0, 1);  lcd.print("  ON                ");
      lcd.setCursor(0, 2);  lcd.print("->OFF               ");
      lcd.setCursor(0, 3);  lcd.print("                    ");
      break;
  }
}

/*-------------------------------------------------------------------------------------*/
void LampControlMenu(){
  lcd.setCursor(0, 0);  lcd.print("Manual  LAMP        "); 
  switch(g_line){
    case LINE_1:
      lcd.setCursor(0, 1);  lcd.print("->ON                ");
      lcd.setCursor(0, 2);  lcd.print("  OFF               ");
      lcd.setCursor(0, 3);  lcd.print("                    ");
      break;
    case LINE_2:
      lcd.setCursor(0, 1);  lcd.print("  ON                ");
      lcd.setCursor(0, 2);  lcd.print("->OFF               ");
      lcd.setCursor(0, 3);  lcd.print("                    ");
      break;
  }
}

/*-------------------------------------------------------------------------------------*/
void SettingMenu(){
  lcd.setCursor(0, 0);  lcd.print("Manual  SETTINGS    "); 
  switch(g_line){
    case LINE_1:
      lcd.setCursor(0, 1);  lcd.print("->GAS THRESHOLD     ");
      lcd.setCursor(0, 2);  lcd.print("  TEMP THRESHOLD    ");
      lcd.setCursor(0, 3);  lcd.print("------------------- ");
      break;
    case LINE_2:
      lcd.setCursor(0, 1);  lcd.print("  GAS THRESHOLD     ");
      lcd.setCursor(0, 2);  lcd.print("->TEMP THRESHOLD    ");
      lcd.setCursor(0, 3);  lcd.print("------------------- ");
      break;
  }
}

/*-------------------------------------------------------------------------------------*/
void GasThresholdMenu(){
  lcd.setCursor(0, 0);  lcd.print("    GAS THRESHOLD   "); 
  lcd.setCursor(0, 1);  lcd.print("                    ");  /* Print Big-number Value */
  if(g_flag){
    g_flag = 0;
    lcd.setCursor(0, 2); lcd.print("                    ");
    lcd.setCursor(0, 3); lcd.print("                    ");
  }
  
  for (int cnt = 0; cnt < sizeof(custChar) / 8; cnt++) lcd.createChar(cnt, custChar[cnt]);

  printBigNum(GasThreshold / 10, 6, 2);
  printBigNum(GasThreshold % 10, 10, 2);
}

/*-------------------------------------------------------------------------------------*/
void TempThresholdMenu(){
  lcd.setCursor(0, 0);  lcd.print("   TEMP THRESHOLD   "); 
  lcd.setCursor(0, 1);  lcd.print("                    ");/* Print Big-number Value */
  if(g_flag){
    g_flag = 0;
    lcd.setCursor(0, 2);  lcd.print("                    ");
    lcd.setCursor(0, 3);  lcd.print("                    ");
  }

  for (int cnt = 0; cnt < sizeof(custChar) / 8; cnt++) lcd.createChar(cnt, custChar[cnt]);

  printBigNum(TempThreshold / 10, 6, 2);
  printBigNum(TempThreshold % 10, 10, 2);
}

/*-------------------------------------------------------------------------------------*/
void GasThresholdMenuRoom2(){
  lcd.setCursor(0, 0);  lcd.print("    GAS THRESHOLD   "); 
  lcd.setCursor(0, 1);  lcd.print("                    ");/* Print Big-number Value */
  if(g_flag){
    g_flag = 0;
    lcd.setCursor(0, 2); lcd.print("                    ");
    lcd.setCursor(0, 3); lcd.print("                    ");
  }
  
  for (int cnt = 0; cnt < sizeof(custChar) / 8; cnt++) lcd.createChar(cnt, custChar[cnt]);

  printBigNum(GasThresholdRoom2 / 10, 6, 2);
  printBigNum(GasThresholdRoom2 % 10, 10, 2);
}

/*-------------------------------------------------------------------------------------*/
void TempThresholdMenuRoom2(){
  lcd.setCursor(0, 0);  lcd.print("   TEMP THRESHOLD   "); 
  lcd.setCursor(0, 1);  lcd.print("                    ");/* Print Big-number Value */
  if(g_flag){
    g_flag = 0;
    lcd.setCursor(0, 2);  lcd.print("                    ");
    lcd.setCursor(0, 3);  lcd.print("                    ");
  } 

  for (int cnt = 0; cnt < sizeof(custChar) / 8; cnt++) lcd.createChar(cnt, custChar[cnt]);

  printBigNum(TempThresholdRoom2 / 10, 6, 2);
  printBigNum(TempThresholdRoom2 % 10, 10, 2);
}
/*-------------------------------------------------------------------------------------*/


void ConfirmDataThreshold(){
  switch(g_SaveSelLAYER2){
    case GASTHRESHOLD:
        if (g_SaveSelROOM == ROOM1) {
          lcd.setCursor(0, 0);  lcd.print("SET GasThreshold    ");
          lcd.setCursor(18, 0);  lcd.print(GasThreshold);
        }else{
          lcd.setCursor(0, 0);  lcd.print("SET GasThreshold    ");
          lcd.setCursor(18, 0);  lcd.print(GasThresholdRoom2);
        }
      break;

    case TEMPTHRESHOLD:
        if (g_SaveSelROOM == ROOM1) {
          lcd.setCursor(0, 0);  lcd.print("SET TempThreshold   ");
          lcd.setCursor(18, 0); lcd.print(TempThreshold);
        }else{
          lcd.setCursor(0, 0);  lcd.print("SET TempThreshold   ");
          lcd.setCursor(18, 0); lcd.print(TempThresholdRoom2);
        }
      
      
      break;
  }

  /* show cursor */
  if (g_line == LINE_1){
    lcd.setCursor(0, 1);  lcd.print("->Yes, sure         ");
    lcd.setCursor(0, 2);  lcd.print("  No                ");
    lcd.setCursor(0, 3);  lcd.print("                    ");
  }else{
    lcd.setCursor(0, 1);  lcd.print("  Yes, sure         ");
    lcd.setCursor(0, 2);  lcd.print("->No                ");
    lcd.setCursor(0, 3);  lcd.print("                    ");
  }
}
/*-------------------------------------------------------------------------------------*/

void ShowSendDataSuccesful(){
  /* clear screen */
  lcd.setCursor(0, 1);  lcd.print("                    ");
  lcd.setCursor(0, 2);  lcd.print("                    ");
  lcd.setCursor(0, 3);  lcd.print("                    ");

  switch(g_SaveSelLAYER2){
    case GASTHRESHOLD:
      lcd.setCursor(0, 0);  lcd.print("   SET GasThreshold  ");
      lcd.setCursor(0, 1);
      if(g_SaveSelROOM == ROOM1) {
        lcd.print(" in ROOM 1");
        /* display points */
        for (int i = 10; i < 20; i++){
          lcd.setCursor(i, 1);
          lcd.print(".");
          while(millis() - lastTime <= 200);
          lastTime = millis();
        }
      }  
      else {
        lcd.print(" in ROOM 2");
        /* display points */
        for (int i = 10; i < 20; i++){
          lcd.setCursor(i, 1);
          lcd.print(".");
          while(millis() - lastTime <= 200);
          lastTime = millis();
        }
      }                        
      lcd.setCursor(0, 2);  lcd.print("      succesful      ");

      /* hold for user see all table */
      while(millis() - lastTime <= 1000);
      lastTime = millis();
      break;

    case TEMPTHRESHOLD:
      lcd.setCursor(0, 0);  lcd.print("  SET TempThreshold ");
      lcd.setCursor(0, 1);
      if(g_SaveSelROOM == ROOM1)   {
        lcd.print(" in ROOM 1");
        /* display points */
        for (int i = 10; i < 20; i++){
          lcd.setCursor(i, 1);
          lcd.print(".");
          while(millis() - lastTime <= 200);
          lastTime = millis();
        }
      }  
      else  {
        lcd.print(" in ROOM 2");
        /* display points */
        for (int i = 10; i < 20; i++){
          lcd.setCursor(i, 1);
          lcd.print(".");
          while(millis() - lastTime <= 200);
          lastTime = millis();
        }
      }      
      lcd.setCursor(0, 2);  lcd.print("      succesful      ");

      /* hold for user see all table */
      while(millis() - lastTime <= 1000);
      lastTime = millis();
      break;
  }

  g_flag = TRUE; // clear 2 underline in big number
}
/*-------------------------------------------------------------------------------------*/

void MainMenu_Auto(){

  //   if(g_flag) {
  //   g_flag = 0;
  //   lcd.setCursor(0, 0);  lcd.print("                    ");/* GAS          [VALUE]  */
  //   lcd.setCursor(0, 1);  lcd.print("                    ");
  //   lcd.setCursor(0, 2);  lcd.print("                    ");/* GAS          [VALUE]  */
  //   lcd.setCursor(0, 3);  lcd.print("                    ");
  //   delay(100);                                                            // wait clear display succesfully
  // }

  lcd.setCursor(0, 0);  lcd.print("A      NORMAL     R1");/*Auto    [STATE]        */
  lcd.setCursor(0, 1);  lcd.print(" GAS                ");/* GAS          [VALUE]  */
  lcd.setCursor(0, 2);  lcd.print(" TEMP               ");/* TEMP         [VALUE]  */
  lcd.setCursor(0, 3);  lcd.print(" FIRE               ");/* FIRE         [STATE]  */

  lcd.setCursor(14, 1);  lcd.print(GasValue);
  lcd.setCursor(14, 2);  lcd.print(TempValue);
  /*
  Descrtiption about "truth table lol"
  1: detect fire, gasvalue > gas threshold, temp > temp threshold
  0: no fire, gasvalue < gas threshold, temp < temp threshold
  FIRE   |   GAS   |   TEMP
     1   |    1    |     1   -> DANGEROUS,            ALARM-ON, LAMP-ON        .
     1   |    1    |     0   -> FIRE-GAS LEAK,        ALARM-ON, LAMP-ON            .
     1   |    0    |     1   -> FIRE-HIGH TEMP,       ALARM-ON, LAMP-ON         .
     1   |    0    |     0   -> DETECTED FIRE,        ALARM-ON [MAYBE cháy giả nên chỉ báo còi]
     0   |    1    |     1   -> GAS LEAK & HIGH TEMP, Pump- ON, ALARM-ON, LAMP-ON        .
     0   |    1    |     0   -> GAS LEAK            , Pump- ON, ALARM-ON       .
     0   |    0    |     1   -> HIGH TEMP           , ALARM-ON      .
     0   |    0    |     0   -> NORMAL              , OF OF OF       

  */
  switch(FireState){
    case 1: if( (GasValue > GasThreshold) && (TempValue > TempThreshold) ){
              lcd.setCursor(0, 0);  lcd.print("A      DANGEROUS  R1");
              // AlarmState = 1;
              LampState = 1;
              PumpState = 1;
            }
            else if((GasValue > GasThreshold) && (TempValue < TempThreshold)){
              lcd.setCursor(0, 0);  lcd.print("A      GAS LEAK   R1");
              // AlarmState = 1;
              LampState = 1;
              PumpState = 1;      
            }
            else if((GasValue < GasThreshold) && (TempValue > TempThreshold)){
              lcd.setCursor(0, 0);  lcd.print("A      HIGH TEMP  R1");
              // AlarmState = 1;
              LampState = 1;
              PumpState = 1;      
            }
            else if((GasValue < GasThreshold) && (TempValue < TempThreshold)){
              lcd.setCursor(0, 0);  lcd.print("A   DETECTED FIRE R1");
              // AlarmState = 1;
              LampState = 0;
              PumpState = 0;      
            }
              AlarmState = 1;
              lcd.setCursor(12, 3);  lcd.print("Detected");
            break;
    case 0: if( (GasValue > GasThreshold) && (TempValue > TempThreshold) ){
              lcd.setCursor(0, 0);  lcd.print("GAS LEAK & HIGH TEMP");
              AlarmState = 1;
              LampState = 0;
              PumpState = 0;
            }
            else if((GasValue > GasThreshold) && (TempValue < TempThreshold)){
              lcd.setCursor(0, 0);  lcd.print("A     GAS LEAK    R1");
              AlarmState = 1;
              LampState = 0;
              PumpState = 0;      
            }
            else if((GasValue < GasThreshold) && (TempValue > TempThreshold)){
              lcd.setCursor(0, 0);  lcd.print("A      HIGH TEMP  R1");
              AlarmState = 1;
              LampState = 0;
              PumpState = 0;      
            }
            else if((GasValue < GasThreshold) && (TempValue < TempThreshold)){
              lcd.setCursor(0, 0);  lcd.print("A      NORMAL     R1");
              AlarmState = 0;
              LampState = 0;
              PumpState = 0;  
            }
            lcd.setCursor(12, 3);  lcd.print("NO-FIRE");
            break;        
  }

  digitalWrite(ALARM, AlarmState | AlarmState_tmp);
  
  digitalWrite(LAMP, LampState | LampState_tmp);
  
  digitalWrite(PUMP, PumpState | PumpState_tmp | PumpStateRoom2);
  IsAlarmActiveRoom1();
  IsPumpActive();
  IsLampActiveRoom1();

}
/*----------------------*/
void MainMenu_AutoRoom2(){

  lcd.setCursor(0, 0);  lcd.print("A      NORMAL     R2");/*Auto    [STATE]        */
  lcd.setCursor(0, 1);  lcd.print(" GAS                ");/* GAS          [VALUE]  */
  lcd.setCursor(0, 2);  lcd.print(" TEMP               ");/* TEMP         [VALUE]  */
  lcd.setCursor(0, 3);  lcd.print(" FIRE               ");/* FIRE         [STATE]  */

  lcd.setCursor(14, 1);  lcd.print(GasValueRoom2);
  lcd.setCursor(14, 2);  lcd.print(TempValueRoom2);
  /*
  Descrtiption about "truth table lol"
  1: detect fire, gasvalue > gas threshold, temp > temp threshold
  0: no fire, gasvalue < gas threshold, temp < temp threshold
  FIRE   |   GAS   |   TEMP
     1   |    1    |     1   -> DANGEROUS,            ALARM-ON, LAMP-ON        .
     1   |    1    |     0   -> FIRE-GAS LEAK,        ALARM-ON, LAMP-ON            .
     1   |    0    |     1   -> FIRE-HIGH TEMP,       ALARM-ON, LAMP-ON         .
     1   |    0    |     0   -> DETECTED FIRE,        ALARM-ON [MAYBE cháy giả nên chỉ báo còi]
     0   |    1    |     1   -> GAS LEAK & HIGH TEMP, Pump- ON, ALARM-ON, LAMP-ON        .
     0   |    1    |     0   -> GAS LEAK            , Pump- ON, ALARM-ON       .
     0   |    0    |     1   -> HIGH TEMP           , ALARM-ON      .
     0   |    0    |     0   -> NORMAL              , OF OF OF       



       FIRE   |   GAS   |   TEMP
     1   |    1    |     1   -> DANGEROUS,            ALARM-ON, LAMP-ON        .
     1   |    1    |     0   -> FIRE-GAS LEAK,        ALARM-ON, LAMP-ON            .
     1   |    0    |     1   -> FIRE-HIGH TEMP,       ALARM-ON, LAMP-ON         .
     1   |    0    |     0   -> DETECTED FIRE,        ALARM-ON [MAYBE cháy giả nên chỉ báo còi]
     0   |    1    |     1   -> GAS LEAK & HIGH TEMP, Pump- ON, ALARM-ON, LAMP-ON        .
     0   |    1    |     0   -> GAS LEAK            , Pump- ON, ALARM-ON       .
     0   |    0    |     1   -> HIGH TEMP           , ALARM-ON      .
     0   |    0    |     0   -> NORMAL              , OF OF OF       

  */
  switch(FireStateRoom2){
    case 1: if( (GasValueRoom2 > GasThresholdRoom2) && (TempValueRoom2 > TempThresholdRoom2) ){
              lcd.setCursor(0, 0);  lcd.print("A    DANGEROUS    R2");
              AlarmStateRoom2 = 1;
              LampStateRoom2 = 1;
              PumpStateRoom2 = 1;
            }
            else if((GasValueRoom2 > GasThresholdRoom2) && (TempValueRoom2 < TempThresholdRoom2)){
              lcd.setCursor(0, 0);  lcd.print("A    GAS LEAK     R2");
              AlarmStateRoom2 = 1;
              LampStateRoom2 = 1;
              PumpStateRoom2 = 1;      
            }
            else if((GasValueRoom2 < GasThresholdRoom2) && (TempValueRoom2 > TempThresholdRoom2)){
              lcd.setCursor(0, 0);  lcd.print("A    HIGH TEMP    R2");
              AlarmStateRoom2 = 1;
              LampStateRoom2 = 1;
              PumpStateRoom2 = 1;      
            }
            else if((GasValueRoom2 < GasThresholdRoom2) && (TempValueRoom2 < TempThresholdRoom2)){
              lcd.setCursor(0, 0);  lcd.print("A  DETECTED FIRE  R2");
              AlarmStateRoom2 = 1;
              LampStateRoom2 = 0;
              PumpStateRoom2 = 0 ;      
            }
            else{
              // do nothing
            }
            lcd.setCursor(12, 3);  lcd.print("Detected");
            break;
    case 0: if( (GasValueRoom2 > GasThresholdRoom2) && (TempValueRoom2 > TempThresholdRoom2) ){
              lcd.setCursor(0, 0);  lcd.print("GASLEAK-HIGHTEMP  R2");
              AlarmStateRoom2 = 1;
              LampStateRoom2 = 0;
              PumpStateRoom2 = 0;
            }
            else if((GasValueRoom2 > GasThresholdRoom2) && (TempValueRoom2 < TempThresholdRoom2)){
              lcd.setCursor(0, 0);  lcd.print("A    GAS LEAK     R2");
              AlarmStateRoom2 = 1;
              LampStateRoom2 = 0;
              PumpStateRoom2 = 0;      
            }
            else if((GasValueRoom2 < GasThresholdRoom2) && (TempValueRoom2 > TempThresholdRoom2)){
              lcd.setCursor(0, 0);  lcd.print("A    HIGH TEMP    R2");
              AlarmStateRoom2 = 1;
              LampStateRoom2 = 0;
              PumpStateRoom2 = 0;      
            }
            else if((GasValueRoom2 < GasThresholdRoom2) && (TempValueRoom2 < TempThresholdRoom2)){
              lcd.setCursor(0, 0);  lcd.print("A      NORMAL     R2");
              AlarmStateRoom2 = 0;
              LampStateRoom2 = 0;
              PumpStateRoom2 = 0;      
            }
            else{
              //do nothing
            }
            lcd.setCursor(12, 3);  lcd.print("NO-FIRE");
            break;        
  }
    digitalWrite(ALARM_ROOM2, AlarmStateRoom2 | AlarmStateRoom2_tmp);
    
    digitalWrite(LAMP_ROOM2, LampStateRoom2 | LampStateRoom2_tmp);
    
    digitalWrite(PUMP, PumpState | PumpState_tmp | PumpStateRoom2);
    IsPumpActive();
    IsAlarmActiveRoom2();
    IsLampActiveRoom2();
   
}

/* exception function handler --------------------------------------------------- */
/* func for BACK */
/*******************************
 *  previous layer menu
********************************/
void pressBACK(){
    if (millis() - lastDebounceTime_BACK > 200) {
      lastDebounceTime_BACK = millis();
      (g_layerMenu==DIV_ROOM)?(g_layerMenu=DIV_ROOM):(g_layerMenu--);
      g_line = LINE_1;

      /* set flag confirm set Threshold data to Firebase be wrong */
      /* 2 case should be processed is:
        - LCD is showing table confirm, and user want to back previous to continous modify value threshold (up/down)
        =====> data value of threshold which be showing will not be changed
        EX: before go to table confirm, value threshold which be up/down is 45, and user want to set it in FIREBASE,
        but at the same time, user dont wanna set it, user press BACK button, data is showing is still 45.
        and if user wanna quit in big number interface, that is the second case below will happen.

        - LCD is showing big number of threshold, and user want to back previous interface, value threshold will not be 
        changed after modify and cannot set new threshold on Firebase.
        Ex: before go to see big number of threshold, value threshold is 30 and after see, user can modify up 
        or down and also want to quit it, after press BACK button, data value until is 30.
       */
      if (g_SaveSelLAYER2 == GASTHRESHOLD || g_SaveSelLAYER2 == TEMPTHRESHOLD)
      {
        
        
        /* case 1: LCD is showing table confirm */
        if (flag_ShowConfirmThreshold){
          /* off flag to set data to Firebase */
          //flag_SentData2Firebase = FALSE;
          flag_ShowConfirmThreshold = FALSE;
          g_flag = TRUE;
          g_line = LINE_1;

          /* still on SETTING Threshold value after press BACK button */
          g_layerMenu++;
        }

        /* case 2: LCD is showing big Threshold value */
        else{
          if(g_SaveSelLAYER2 == GASTHRESHOLD){
            (g_SaveSelROOM == ROOM1)?(GasThreshold = pre_Value):(GasThresholdRoom2 = pre_Value);
          }else{
            (g_SaveSelROOM == ROOM1)?(TempThreshold = pre_Value):(TempThresholdRoom2 = pre_Value);
          }
        }
      }
    }
}


/*-------------------------------------------------------------------------------------*/
void pressOK(){
  if (millis() - lastDebounceTime_OK > 175) {
    lastDebounceTime_OK = millis();

    switch (g_layerMenu)                            
    {
    case DIV_ROOM:
        switch (g_line)
        {
        case LINE_1:                                    // jump to SENSOR menu
            SelectRoom = ROOM1;                         // select Room1
            g_layerMenu = LAYER1_MANUAL;
            g_SaveSelROOM = ROOM1;
            g_line = LINE_1;                            // reset ">" to line 1
                                    
            break;
        
        case LINE_2:                                    // jump to ACTUATORS menu
            SelectRoom = ROOM2;
            g_layerMenu = LAYER1_MANUAL;
            // g_layerMenu = LAYER1_MANUAL;
            g_SaveSelROOM = ROOM2;
            g_line = LINE_1;                            // reset ">" to line 1
            
            break;
        }
        break;

    case LAYER1_MANUAL:                                 // in manual mode default display      
        switch (g_line)
        {
          case LINE_1:                                    // jump to SENSOR menu
              g_SaveSelLAYER1 = SENSORS;
              g_line = LINE_1;                            // reset ">" to line 1
              break;
          
          case LINE_2:                                    // jump to ACTUATORS menu
              g_SaveSelLAYER1 = ACTUATORS;
              g_line = LINE_1;                            // reset ">" to line 1
              break;

          case LINE_3:                                    // jump to SETTING menu
              g_SaveSelLAYER1 = SETTING;              
              g_line = LINE_1;                            // reset ">" to line 1
              break;
        }
            g_layerMenu = LAYER2_MANUAL;
        // g_layerMenu = LAYER2_MANUAL;
        break;                                          // break LAYER 1 MANUAL menu
    
    case LAYER2_MANUAL:                                 // menu display in SENSOR, ACTUATOR, SETTING
        switch (g_SaveSelLAYER1)
        {
        case SENSORS:
            g_layerMenu = LAYER2_MANUAL;                // stay in LAYER2_MANUAL
            break;

        case ACTUATORS:                                 // in ACTUATORS mode
            switch (g_line)
            {
            case LINE_1:                                // select Pump
                g_SaveSelLAYER2 = PUMP;
                g_line = LINE_1;                        // reset ">" to line 1
                break;
            
            case LINE_2:                                // select ALARM
                g_SaveSelLAYER2 = ALARM;
                g_line = LINE_1;                        // reset ">" to line 1
                break;
            
            case LINE_3:                                // select LAMP
                g_SaveSelLAYER2 = LAMP;
                g_line = LINE_1;                        // reset ">" to line 1
                break;
            }
              g_layerMenu = LAYER3_MANUAL;
            // g_layerMenu = LAYER3_MANUAL;
          break;                                      // break ACTUATORS          
        
        case SETTING:                                   
            if(g_line == LINE_1){
              g_SaveSelLAYER2=GASTHRESHOLD;

              /* save value GasThreshold beford be changed */
              (g_SaveSelROOM == ROOM1)?(pre_Value = GasThreshold):(pre_Value = GasThresholdRoom2);
              Serial.println(pre_Value);
            }
            if(g_line == LINE_2){
              g_SaveSelLAYER2=TEMPTHRESHOLD;

              /* save value TempThreshold beford be changed */
              (g_SaveSelROOM == ROOM1)?(pre_Value = TempThreshold):(pre_Value = TempThresholdRoom2);
              Serial.println(pre_Value);
            }

            g_flag = 1;
            g_line = LINE_1;
            g_layerMenu = LAYER3_MANUAL;
                
            break;
        }
        //g_layerMenu = LAYER3_MANUAL; 
        break;                                          // break LAYER 2 MENU

    case LAYER3_MANUAL:                                 // ON-OFF device or setting GAS, TEMP Threshold
        switch (g_SaveSelLAYER2)
        {
        case PUMP:                                       // in Pump device
            /* maybe use soft flag to know device be ON or OFF when return while(1) jump to on-0ff device.
            to avoid active device in interrupt handler 
            OR can send command to device to turn on or off device, but send data to Firebase must use soft flag
            -> it maybe be delay in device and data in firebase 
            
            --> best way is set flag to return while(1) to do both nad off flag: set device and send data to firebase */
            // g_line 1 -> on device
            // g_line 2 -> off device
            //(g_line)?():();
            // (g_line==1)?(Serial.println("Pump ON")):((Serial.println("Pump OFF")));
            // if(SelectRoom == ROOM1){
            //   (g_line==1)?( PumpState = HIGH ):(PumpState = LOW );
            // }
            // else{
            //   (g_line==1)?( PumpStateRoom2 = HIGH ):(PumpStateRoom2 = LOW );    
            // }
            (g_line==1)?( PumpState = HIGH ):(PumpState = LOW );
            break;
        
        case ALARM:                                       // in Pump device
            // same above
            //(g_line)?():();
            // (g_line==1)?(Serial.println("ALARM ON")):((Serial.println("ALARM OFF")));
            if(SelectRoom == ROOM1){
              (g_line==1)?( AlarmState = HIGH ):(AlarmState = LOW );
            }
            else{
              (g_line==1)?( AlarmStateRoom2 = HIGH ):(AlarmStateRoom2 = LOW );    
            }
            break;
        
        case LAMP:                                       // in Pump device
            /* maybe use soft flag to know device be ON or OFF when return jump to active.
            to avoid active device in interrupt handler  */
            // g_line 1 -> on device
            // g_line 2 -> off device
            //(g_line)?():();
            // (g_line==1)?(Serial.println("LAMP ON")):((Serial.println("LAMP OFF")));
            if(SelectRoom == ROOM1){
              (g_line==1)?( LampState = HIGH ):(LampState = LOW );
            }
            else{
              (g_line==1)?( LampStateRoom2 = HIGH ):(LampStateRoom2 = LOW );    
            }
            break;

        /* for confirm data threshold GAS and TEMP */
        default:
            /* set flag for confirm threshold */

            if (!flag_ShowConfirmThreshold){                // LCD dont show table confirm data 
              /* press OK will be set flag ON */
              flag_ShowConfirmThreshold = TRUE;       
            } else{                                         // LCD is showing table confirm data 
              /* process data Threshold */ 
              if (g_line == LINE_1){                        // in line 1, accept to send data
                /* on flag admit to send data to Firebase */
                flag_SentData2Firebase = TRUE;
              }else{                                        // in line 2, dont accept confirm to set data 
                /* off flag show table confirm */
                flag_ShowConfirmThreshold = FALSE;

                if(g_SaveSelLAYER2 == GASTHRESHOLD){
                  (g_SaveSelROOM == ROOM1)?(GasThreshold = pre_Value):(GasThresholdRoom2 = pre_Value);
                }else{
                  (g_SaveSelROOM == ROOM1)?(TempThreshold = pre_Value):(TempThresholdRoom2 = pre_Value);
                }
              }
              g_flag = TRUE;      // to clear 2 underline Setting threshold
              g_line = LINE_1;
            }
          break;        // break default
      }
    }
  }
}

// /* func for UP */
void pressUP(){
  if (millis() - lastDebounceTime_UP > 250) {
    lastDebounceTime_UP = millis();

    switch (g_layerMenu)
    {
    case DIV_ROOM:
        (g_line==LINE_1)?(g_line=LINE_1):(g_line--);
        break;

    case LAYER1_MANUAL:                                 // in default menu manual
        
        (g_line==LINE_1)?(g_line=LINE_1):(g_line--);
        break;
    
    case LAYER2_MANUAL:                                 // in SENSOR, ACTUATOR, SETTING
        (g_line==LINE_1)?(g_line=LINE_1):(g_line--);
        break;

    case LAYER3_MANUAL:                                 // in control device or set value for GAS or TEMP
        switch (g_SaveSelLAYER2)
        {
          case GASTHRESHOLD:
              /* add new feature for confirm data */

              if(flag_ShowConfirmThreshold == TRUE){            // flag confirm data is turn ON
                  (g_line==LINE_1)?(g_line=LINE_1):(g_line--);
              }
              else{                                             // flag confirm data is turn OFF
                if(SelectRoom == ROOM1){
                    GasThreshold = GasThreshold + 1;
                    if(GasThreshold > 99) GasThreshold = 99;                      
                 
                }
                else{                                          // in ROOM 2
                    GasThresholdRoom2 = GasThresholdRoom2 + 1; 
                    if(GasThresholdRoom2 > 99) GasThresholdRoom2 = 99;                   
                         
                } 
              // can limit value GAS here
              }
              
              break;
          case TEMPTHRESHOLD:
              /* add new feature for confirm data */

              if(flag_ShowConfirmThreshold == TRUE){            // flag confirm data is turn ON
                  (g_line==LINE_1)?(g_line=LINE_1):(g_line--);

              }else{
                if(SelectRoom == ROOM1){
                    TempThreshold = TempThreshold + 1;
                    if(TempThreshold > 99) TempThreshold = 99;
                }else{
                    TempThresholdRoom2 = TempThresholdRoom2 + 1;
                    if(TempThresholdRoom2 > 99) TempThresholdRoom2 = 99;
                }             
              }
              break;
          default:                    // for Pump, ALARM, LAMP
              (g_line==LINE_1)?(g_line=LINE_1):(g_line--);
              break;
        }
        //sendata

        break;
    }
  }
}

// /* func for DOWN */
void pressDOWN(){
  if (millis() - lastDebounceTime_UP > 250) {
    lastDebounceTime_UP = millis();

    switch (g_layerMenu)
    {
    case DIV_ROOM:
      (g_line==LINE_2)?(g_line=LINE_2):(g_line++);
      break;

    case LAYER1_MANUAL:
        //   if (millis() - lastDebounceTime_DW > 250) {
        //   lastDebounceTime_DW = millis();
        //   (g_line==LINE_3)?(g_line=LINE_3):(g_line++);
        // }
        (g_line==LINE_3)?(g_line=LINE_3):(g_line++);
        break;
    
    case LAYER2_MANUAL:
          // if (millis() - lastDebounceTime_DW > 250) {
          //   lastDebounceTime_DW = millis();
          //   (g_line==LINE_1)?(g_line=LINE_2):(g_line++);
          // }
          (g_line==LINE_3)?(g_line=LINE_3):(g_line++);
        break;
    case LAYER3_MANUAL:
        switch (g_SaveSelLAYER2)
        {
          case GASTHRESHOLD:
              /* add new feature for confirm data */

              if(flag_ShowConfirmThreshold == TRUE){            // flag confirm data is turn ON
                  (g_line==LINE_2)?(g_line=LINE_2):(g_line++);
              }
              else{
                if(SelectRoom == ROOM1){                       // in ROOM 1
                    GasThreshold = GasThreshold - 1;
                    if(GasThreshold < 1) GasThreshold = 1;
                } 
                else{                                         // in ROOM 2
                    GasThresholdRoom2 = GasThresholdRoom2 - 1;
                    if(GasThresholdRoom2 < 1) GasThresholdRoom2 = 1;
                } 
              }
              break;
          case TEMPTHRESHOLD:
              /* add new feature for confirm data */

              if(flag_ShowConfirmThreshold == TRUE){            // flag confirm data is turn ON
                (g_line==LINE_2)?(g_line=LINE_2):(g_line++);
              }
              else{                                             // flag confirm data is turn OFF
                if(SelectRoom == ROOM1){                        // in ROOM 1
                  TempThreshold = TempThreshold - 1;
                  if(TempThreshold <1 ) TempThreshold = 1;
                }
                else{                                           // in ROOM 2
                  TempThresholdRoom2 = TempThresholdRoom2 - 1;
                  if(TempThresholdRoom2 < 1) TempThresholdRoom2 = 1;
                }
              }
              break;
          default:                    // for Pump, ALARM, LAMP
              (g_line==LINE_2)?(g_line=LINE_2):(g_line++);
              break;

  
        }

        break;
    }
  }
}

/*-------------------------------------------------------------------------------------*/
void pressRESET(){
  if(g_mode == MANUAL) {
    g_mode = AUTO;
    g_flag = 1;
  }
  else{
    g_mode = MANUAL;
    g_layerMenu = DIV_ROOM;               
    g_mode = MANUAL;                          
    g_line = LINE_1;                           
    // g_SaveSelLAYER1 = SENSORS;                 
    // g_SaveSelLAYER2 = Pump;                     
  }
  g_layerMenu = DIV_ROOM;   
}

/*-------------------------------------------------------------------------------------*/
void pressOPTION(){
  if (millis() - lastDebounceTime_BACK > 200) {
    lastDebounceTime_BACK = millis();
    OptionMenu = !OptionMenu;
    FirstimeToDisplay = OptionMenu;
    if(OptionMenu == 1){
      // PhoneCallState = 0;
      // f_EnablePhone = 1;
      g_layerMenu = DIV_ROOM;
    }

    // Serial.print("OptionMenu is: ");
    // Serial.println(OptionMenu);


  }
}



/*-------------------------------------------------------------------------------------*/
void ReadDataRoom1(){
  TempValue = dht.readTemperature();
  GasValue = map(analogRead(MQ2_SENSOR), 0, 4095, 0, 100);
  if(digitalRead(FLAME_SENSOR) == 1){
    if(millis() - LastTimetoDetectFire > 20000){
      Firebase.setBool(fbdo, "/Room1/SENSORS/Fire", FireState);
      FireState = 0;
    }
  } 
  else
  {
    FireState = 1;
    Firebase.setBool(fbdo, "/Room1/SENSORS/Fire", FireState);
    LastTimetoDetectFire = millis();
  }
}

/*-------------------------------------------------------------------------------------*/
void SetDataRoom1(){
  Firebase.setInt(fbdo, "/Room1/SENSORS/Gas", GasValue); // SET gas value 
  Firebase.setFloat(fbdo, "/Room1/SENSORS/Temperature", TempValue );//set temp threshold  on firebase
}

/*-------------------------------------------------------------------------------------*/
void SetPumpState(){
    Firebase.setInt(fbdo, "/Pump", PumpState | PumpStateRoom2);
}

/*-------------------------------------------------------------------------------------*/
void GetPumpState(){
  Firebase.RTDB.getInt(&fbdo,"/Pump");
  PumpState_tmp = fbdo.intData();
  if((PumpState || PumpStateRoom2) && PumpState_tmp)
  {
    PumpState_tmp = 0;
    flag_PumpOFFapproval = 1;
  }

}

void ShowDataBeChanged(){
  /* clear screen */
  lcd.setCursor(0, 0);  lcd.print("                    ");
  lcd.setCursor(0, 1);  lcd.print("                    ");
  lcd.setCursor(0, 2);  lcd.print("                    ");
  lcd.setCursor(0, 3);  lcd.print("                    ");

  /* display note */
  if (g_SaveSelLAYER2 == GASTHRESHOLD)
  {
    lcd.setCursor(0, 1);  lcd.print("    Gas threshold   ");
    lcd.setCursor(0, 2);  lcd.print(" be changed to      ");
    if(g_SaveSelROOM == ROOM1){lcd.setCursor(16, 2);  lcd.print(GasThreshold);}
    else                      {lcd.setCursor(16, 2);  lcd.print(GasThresholdRoom2);}
    lcd.setCursor(0, 3);  lcd.print("    in APP          ");
  }
  else{
    lcd.setCursor(0, 1);  lcd.print("   Temp threshold   ");
    lcd.setCursor(0, 2);  lcd.print(" be changed to      ");
    if(g_SaveSelROOM == ROOM1){lcd.setCursor(16, 2);  lcd.print(TempThreshold);}
    else                      {lcd.setCursor(16, 2);  lcd.print(TempThresholdRoom2);}
    lcd.setCursor(0, 3);  lcd.print("    in APP          ");
  }

  /* delay for user see table */
  while(millis() - lastTime <= 8000);
  lastTime = millis();
}
/*-------------------------------------------------------------------------------------*/


void GetThresholdOnlyRoom1(){
  if (!OptionMenu){
    /*-------------------------------------------------------------------------------------------*/
    Firebase.RTDB.getInt(&fbdo,"/Room1/SETTINGS/Gas Threshold"); // get gas threshold  on firebase
    GasThreshold_tmp = fbdo.intData();
    if(GasThreshold_tmp < 1)
    {
      GasThreshold = 1;
    }
    else if(GasThreshold_tmp > 99)
    {
      GasThreshold = 99;
    }
    else if (GasThreshold !=  GasThreshold_tmp){
      GasThreshold = GasThreshold_tmp;
    }
    /*-------------------------------------------------------------------------------------------*/
    Firebase.RTDB.getInt(&fbdo,"/Room1/SETTINGS/Temperature Threshold"); // get temp value of NODE on firebase
    TempThreshold_tmp = fbdo.intData();
    if(TempThreshold_tmp < 1)
    {
      TempThreshold = 1;
    }
    else if(TempThreshold_tmp > 99)
    {
      TempThreshold = 100;
    }
    else TempThreshold =  TempThreshold_tmp;
    /*-------------------------------------------------------------------------------------------*/
  }
  else{
    switch (g_SaveSelLAYER2)
      {
      case GASTHRESHOLD:
        /*-------------------------------------------------------------------------------------------*/
          Firebase.RTDB.getInt(&fbdo,"/Room1/SETTINGS/Gas Threshold"); // get gas threshold  on firebase
          GasThreshold_tmp = fbdo.intData();
          Serial.println(GasThreshold_tmp);
          if(GasThreshold_tmp != pre_Value){
            /* assign new value for Gas Threshold */
            GasThreshold = GasThreshold_tmp;
            pre_Value = GasThreshold;

            /* display in LCD */
            ShowDataBeChanged();

            /* off flag show table confirm */
            flag_ShowConfirmThreshold = FALSE;
            g_flag = TRUE;
          }       
        break;

      case TEMPTHRESHOLD:
        /*-------------------------------------------------------------------------------------------*/
        Firebase.RTDB.getInt(&fbdo,"/Room1/SETTINGS/Temperature Threshold"); // get temp value of NODE on firebase
          TempThreshold_tmp = fbdo.intData();
          Serial.println(TempThreshold_tmp);
          if(TempThreshold_tmp != pre_Value){
            /* assign new value for Gas Threshold */
            TempThreshold = TempThreshold_tmp;
            pre_Value = TempThreshold;

            /* display in LCD */
            ShowDataBeChanged();

            /* off flag show table confirm */
            flag_ShowConfirmThreshold = FALSE;
            g_flag = TRUE;
          }        
        break;
      }
  }
}

/*-------------------------------------------------------------------------------------*/
void SetThresholdOnlyRoom1(){
  //GasThreshold = fbdo.intData();
  /* if(GasThreshold != GasThreshold_tmp){
    Firebase.setInt(fbdo, "/Room1/SETTINGS/Gas Threshold", GasThreshold);
  }

  //TempThreshold = fbdo.intData();
  if(TempThreshold != TempThreshold_tmp){
    Firebase.setInt(fbdo, "/Room1/SETTINGS/Temperature Threshold", TempThreshold);
  } */

  if (g_SaveSelLAYER2 == GASTHRESHOLD) {                // gas threshold
    Firebase.setInt(fbdo, "/Room1/SETTINGS/Gas Threshold", GasThreshold);
  }else{                                                // temp threshold  
    Firebase.setInt(fbdo, "/Room1/SETTINGS/Temperature Threshold", TempThreshold);
  }
}

/*-------------------------------------------------------------------------------------*/
void SetAlarmStateRoom1(){
  Firebase.setInt(fbdo, "/Room1/DEVICES/Alarm", AlarmState);
}

/*-------------------------------------------------------------------------------------*/
void SetLampStateRoom1(){
  Firebase.setInt(fbdo, "/Room1/DEVICES/Lamp", LampState);
}

/*-------------------------------------------------------------------------------------*/
void GetDeviceStateRoom1(){
  Firebase.RTDB.getInt(&fbdo,"/Room1/DEVICES/Alarm");
  AlarmState_tmp = fbdo.intData();
  if(AlarmState && AlarmState_tmp){
    AlarmState_tmp = 0;
    flag_AlarmOFFapproval = 1;
  }
  Firebase.RTDB.getInt(&fbdo,"/Room1/DEVICES/Lamp");
  LampState_tmp = fbdo.intData();
  if(LampState && LampState_tmp ){
    LampState_tmp = 0;
    flag_LampOFFapproval = 1;
  }
}

/*set data to Firebase --------------------------------------------------------------*/
void SetDeviceStateRoom1(){
  SetAlarmStateRoom1();
  SetLampStateRoom1();
}

/*------------------------------------------------------------------------------------------------------------------*/
void GetDataRoom2(){
  Firebase.RTDB.getInt(&fbdo,"/ROOM2/SENSORS/Gas"); // get gas value of NODE on firebase
  GasValue_tmpRoom2 = fbdo.intData();
  if(GasValue_tmpRoom2 != 0)
  {
    GasValueRoom2 = GasValue_tmpRoom2;
  }

  Firebase.RTDB.getInt(&fbdo,"/ROOM2/SENSORS/Temperature");//get temp threshold  on firebase
  TempValue_tmpRoom2 = fbdo.floatData();
  if(TempValue_tmpRoom2 != 0)
  {
    TempValueRoom2 = TempValue_tmpRoom2;
  }
  Firebase.RTDB.getBool(&fbdo,"/ROOM2/SENSORS/Fire");
  FireStateRoom2=fbdo.boolData();
}

/*-------------------------------------------------------------------------------------*/
void GetThresholdOnlyRoom2(){
  if(!OptionMenu){
    /*-------------------------------------------------------------------------------------------*/
    Firebase.RTDB.getInt(&fbdo,"/ROOM2/SETTINGS/Gas Threshold"); // get gas threshold  on firebase
    GasThreshold_tmpRoom2 = fbdo.intData();
    if(GasThreshold_tmpRoom2 < 1)
    {
      GasThresholdRoom2 = 1;
    }
    else if(GasThreshold_tmpRoom2 > 99)
    {
      GasThresholdRoom2 = 99;
    }
    else if(GasThresholdRoom2 !=  GasThreshold_tmpRoom2){
      GasThresholdRoom2 = GasThreshold_tmpRoom2;
    }
    /*-------------------------------------------------------------------------------------------*/
    Firebase.RTDB.getInt(&fbdo,"/ROOM2/SETTINGS/Temperature Threshold"); // get temp value of NODE on firebase
    TempThreshold_tmpRoom2 = fbdo.intData();
    if(TempThreshold_tmpRoom2 < 1)
    {
      TempThresholdRoom2 = 1;
    }
    else if(TempThreshold_tmpRoom2 > 99)
    {
      TempThresholdRoom2 = 99;
    }
    else if (TempThresholdRoom2 !=  TempThreshold_tmpRoom2){
      TempThresholdRoom2 = TempThreshold_tmpRoom2;
    }
    /*-------------------------------------------------------------------------------------------*/
  } 
  else{
    switch (g_SaveSelLAYER2){
      case GASTHRESHOLD:
        /*-------------------------------------------------------------------------------------------*/
        Firebase.RTDB.getInt(&fbdo,"/ROOM2/SETTINGS/Gas Threshold"); // get gas threshold  on firebase
          GasThreshold_tmpRoom2 = fbdo.intData();
          Serial.println(GasThreshold_tmpRoom2);
          if(GasThreshold_tmpRoom2 != pre_Value){
            /* assign new value for Gas Threshold */
            GasThresholdRoom2 = GasThreshold_tmpRoom2;
            pre_Value = GasThresholdRoom2;

            /* display in LCD */
            ShowDataBeChanged();

            /* off flag show table confirm */
            flag_ShowConfirmThreshold = FALSE;
            g_flag = TRUE;
          }
        
        break;
      
      case TEMPTHRESHOLD:
        /*-------------------------------------------------------------------------------------------*/
        Firebase.RTDB.getInt(&fbdo,"/ROOM2/SETTINGS/Temperature Threshold"); // get temp value of NODE on firebase
          TempThreshold_tmpRoom2 = fbdo.intData();
          Serial.println(TempThreshold_tmpRoom2);
          if(TempThreshold_tmpRoom2 != pre_Value){
            /* assign new value for Gas Threshold */
            TempThresholdRoom2 = TempThreshold_tmpRoom2;
            pre_Value = TempThresholdRoom2;

            /* display in LCD */
            ShowDataBeChanged();

            /* off flag show table confirm */
            flag_ShowConfirmThreshold = FALSE;
            g_flag = TRUE;
          }       
        break;
    }
  }
}

/*-------------------------------------------------------------------------------------*/
void SetThresholdOnlyRoom2(){
  /* //GasThreshold = fbdo.intData();
  if(GasThresholdRoom2 != GasThreshold_tmpRoom2){
     Firebase.setInt(fbdo, "/ROOM2/SETTINGS/Gas Threshold", GasThresholdRoom2);
  }
  else if(TempThresholdRoom2 != TempThreshold_tmpRoom2){
    Firebase.setInt(fbdo, "/ROOM2/SETTINGS/Temperature Threshold", TempThresholdRoom2);
  } */
  
  if (g_SaveSelLAYER2 == GASTHRESHOLD) {
    Firebase.setInt(fbdo, "/ROOM2/SETTINGS/Gas Threshold", GasThresholdRoom2);
  }else{                                    // TEMP THRESHOLD PLAYER
    Firebase.setInt(fbdo, "/ROOM2/SETTINGS/Temperature Threshold", TempThresholdRoom2);
  }
}
  
/*-------------------------------------------------------------------------------------*/
void SetAlarmStateRoom2(){
  Firebase.setInt(fbdo, "/ROOM2/DEVICES/Alarm", AlarmStateRoom2);
}

/*-------------------------------------------------------------------------------------*/
void SetLampStateRoom2(){
  Firebase.setInt(fbdo, "/ROOM2/DEVICES/Lamp", LampStateRoom2);
}

/*-------------------------------------------------------------------------------------*/
void GetDeviceStateRoom2(){
  Firebase.RTDB.getInt(&fbdo,"/ROOM2/DEVICES/Alarm");
  AlarmStateRoom2_tmp = fbdo.intData();
  if(AlarmStateRoom2 && AlarmStateRoom2_tmp){
    AlarmStateRoom2_tmp = 0;
    flag_AlarmRoom2OFFapproval = 1;
  }

  Firebase.RTDB.getInt(&fbdo,"/ROOM2/DEVICES/Lamp");
  LampStateRoom2_tmp = fbdo.intData();
  if(LampStateRoom2_tmp && LampStateRoom2){
    LampStateRoom2_tmp = 0;
    flag_LampRoom2OFFapproval = 1;
  }
}

/*-------------------------------------------------------------------------------------*/
void SetDeviceStateRoom2(){
  SetAlarmStateRoom2();
  SetLampStateRoom2();
}

/*-------------------------------------------------------------------------------------*/
void callPhone(String phone) {
  simSerial.println("ATD"+phone+";");
}

/*-------------------------------------------------------------------------------------*/
/*Recall user after timing out*/
void ReCall(){
  if(PhoneCallState){
    response = simSerial.readString();
    Serial.println(response); 
  
    if(((millis() - LastCall > 44000) && (response.indexOf("NO CARRIER") != -1))
    ||((response.indexOf("ERROR") != -1) || (response.indexOf("+CME ERROR: memory failure") != -1))) {
      LastCall = millis();
      callPhone(phoneNumber);
      delay(5);
    }   
  }
}

/*-------------------------------------------------------------------------------------*/
void GetPhoneNumber(){
  if((millis() - LastimeToCheckWifi > 60000) && !PhoneCallState){
    Firebase.RTDB.getString(&fbdo, "/PhoneNumber");
    phoneNumber = fbdo.stringData();
    LastimeToCheckWifi = millis();
  }
}
/*-----------------------------ARE DEVICES REALLY AVITVE?-------------------------------*/


void IsPumpActive(){
  if (millis() - LastimeToSetDeviceState > 3000) {
    LastimeToSetDeviceState = millis();
    if(PumpState || PumpState_tmp || PumpStateRoom2){
      uint32_t ADC_Pump_Average = 0;
        for(uint8_t i = 0; i < 255; i++){
          uint16_t ADC_Pump = analogRead(ACS_PUMP);
          ADC_Pump_Average += ADC_Pump;
        }
      ADC_Pump_Average /= 255;

      if(ADC_Pump_Average > ADCTHRESHOLD){
        Firebase.setInt(fbdo, "/ACTIVE/IsPumpActive", 1); 
      }
      else{
        Firebase.setInt(fbdo, "/ACTIVE/IsPumpActive", 0); 
      }
      Serial.print("ADC_Pump_Average");
      Serial.println(ADC_Pump_Average); 
    }
  }
}
/*-----------------------------ROOM1-------------------------------*/

void IsAlarmActiveRoom1(){
  if (millis() - LastimeToSetAlarmStateRoom1 > 3000) {
  LastimeToSetAlarmStateRoom1 = millis();
    if(AlarmState || AlarmState_tmp){
      uint32_t ADC_AlarmRoom1_Average = 0;
        for(uint8_t i = 0; i < 255; i++){
          uint16_t ADC_AlarmRoom1 = analogRead(ACS_ALARMROOM1);
          ADC_AlarmRoom1_Average += ADC_AlarmRoom1;
        }
      ADC_AlarmRoom1_Average /= 255;

      if(ADC_AlarmRoom1_Average > ADCTHRESHOLD){
        Firebase.setInt(fbdo, "/ACTIVE/IsAlarmActiveRoom1", 1); 
      }
      else{
        Firebase.setInt(fbdo, "/ACTIVE/IsAlarmActiveRoom1", 0); 
      }
      Serial.print("ADC_AlarmRoom1_Average");
      Serial.println(ADC_AlarmRoom1_Average);  
    }
  }
}
void IsLampActiveRoom1(){
  if (millis() - LastimeToSetLampStateRoom1 > 3000) {
  LastimeToSetLampStateRoom1 = millis();
    if(LampState || LampState_tmp){
    uint32_t ADC_LampRoom1_Average = 0;
      for(uint8_t i = 0; i < 255; i++){
        uint16_t ADC_LampRoom1 = analogRead(ACS_LAMPROOM1);
        ADC_LampRoom1_Average += ADC_LampRoom1;
      }
      ADC_LampRoom1_Average /= 255;
      if(ADC_LampRoom1_Average > ADCTHRESHOLD){
        Firebase.setInt(fbdo, "/ACTIVE/IsLampActiveRoom1", 1); 
      }
      else{
        Firebase.setInt(fbdo, "/ACTIVE/IsLampActiveRoom1", 0); 
      } 
      Serial.print("ADC_LampRoom1_Average");
      Serial.println(ADC_LampRoom1_Average);  
    }
  }
}
/*-----------------------------ROOM2-------------------------------*/

void IsAlarmActiveRoom2(){
  if (millis() - LastimeToSetAlarmStateRoom2 > 3000) {
  LastimeToSetAlarmStateRoom2 = millis();
    if(AlarmStateRoom2 || AlarmStateRoom2_tmp){
      uint32_t ADC_AlarmRoom2_Average = 0;
        for(uint8_t i = 0; i < 255; i++){
          uint16_t ADC_AlarmRoom2 = analogRead(ACS_ALARMROOM2);
          ADC_AlarmRoom2_Average += ADC_AlarmRoom2;
        }
      ADC_AlarmRoom2_Average /= 255;

      if(ADC_AlarmRoom2_Average > ADCTHRESHOLD){
        Firebase.setInt(fbdo, "/ACTIVE/IsAlarmActiveRoom2", 1); 
      }
      else{
        Firebase.setInt(fbdo, "/ACTIVE/IsAlarmActiveRoom2", 0); 
      } 
      Serial.print("ADC_AlarmRoom2_Average");
      Serial.println(ADC_AlarmRoom2_Average); 
    }
  }
}
void IsLampActiveRoom2(){
  if (millis() - LastimeToSetLampStateRoom2 > 3000) {
  LastimeToSetLampStateRoom2 = millis();
    if(LampStateRoom2 || LampStateRoom2_tmp){
    uint32_t ADC_LampRoom2_Average = 0;
      for(uint8_t i = 0; i < 255; i++){
        uint16_t ADC_LampRoom2 = analogRead(ACS_LAMPROOM2);
        ADC_LampRoom2_Average += ADC_LampRoom2;
      }
      ADC_LampRoom2_Average /= 255;
      if(ADC_LampRoom2_Average > ADCTHRESHOLD){
        Firebase.setInt(fbdo, "/ACTIVE/IsLampActiveRoom2", 1); 
      }
      else{
        Firebase.setInt(fbdo, "/ACTIVE/IsLampActiveRoom2", 0); 
      } 
      Serial.print("ADC_LampRoom2_Average");
      Serial.println(ADC_LampRoom2_Average);  
    }
  }
}
void GetOptionMenu(){
   Firebase.RTDB.getInt(&fbdo, "/OPTION");
    Option_tmp = fbdo.intData();
    if((OptionMenu != Option_tmp) && (OptionMenu == Option_pre){
      OptionMenu = Option_tmp;
    }
}
void SetOptionMenu(){
  if((OptionMenu != Option_pre) && (OptionMenu != Option_tmp)){
    Firebase.setInt(fbdo, "/OPTION", OptionMenu);
  }
}

/*======================================= END =====================================*/




















