/*
Title: Acrobot Legs v1
Author: Daniel Simu
Date: February 2023
Description: This is the Acrobot firmware

Status light: Blue = connected wireless, Red = disconnected wireless

*/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include "AS5600.h"
#include "Wire.h"
#include <PID_v1.h> // https://github.com/br3ttb/
#include <RunningMedian.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h>
#include <LiquidCrystal_I2C.h>
#include "PCF8574.h"

#define R_F_PWM_PIN  16  
#define R_B_PWM_PIN  17   
#define L_F_PWM_PIN  19  
#define L_B_PWM_PIN  18 
#define BATTERY_V_PIN 32

#define BUZZER_PIN 26
#define IR_PIN 27
#define LED_R_PIN 14
#define LED_G_PIN 12
#define LED_B_PIN 13

#define BOOT_SW_PIN 23

#define ENCODER_L_PWM 15

// ---------------
// MARK: - FORWARD DECLARATIONS in order of document 

// BATTERY

int8_t batteryPercent;
RunningMedian batterySamples = RunningMedian(64); 
uint32_t batteryAlarmTimer = 0;

void updateBattery();

// BUZZER
uint32_t buzzerTimer = 0;

void setBuzzer(uint16_t time=100);
void updateBuzzer();

// DATA espnow

uint32_t lastReceiveTime = 0;

uint8_t remoteAddress[] = {0x34, 0x94, 0x54, 0xBE, 0xDB, 0x6C};

uint32_t dataTimer = 0;

bool connectionStatus = false;

// data out:
typedef struct struct_data_out {

  double rP;
  double rI;
  double rD;

  double rInput;
  double lInput;

} struct_data_out;

struct_data_out dataOut;

// data in:

typedef struct struct_data_in {
  int16_t joystickLX;
  int16_t joystickLY;
  int16_t joystickRX;
  int16_t joystickRY;

  int16_t sliderLL;
  int16_t sliderLA;
  int16_t sliderRL;
  int16_t sliderRA;

  int16_t encoderPos;
  bool encoderSwDown;

  char key;

  int8_t batteryPercent;

  double rP;
  double rI;
  double rD;

  double lP;
  double lI;
  double lD;

  uint16_t rTargetPositionDegrees;
  uint16_t lTargetPositionDegrees;

} struct_data_in;

struct_data_in dataIn;

esp_now_peer_info_t peerInfo;

void resetReceiveTimeout();
void checkReceiveTimeout();
void sendData();
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len);

String success; // TODO: remove this

// ENCODERS

AS5600 rAs5600; // encoder
AS5600 lAs5600; // encoder

void captureLPWM();

volatile uint32_t lPWMduration = 0;

uint16_t positionRLegRaw;
uint16_t positionLLegRaw;
double positionRLegDegrees;
double positionLLegDegrees;
const uint16_t NEUTRAL_R_LEG = 3107; // 4096 - position at very top, raw
const uint16_t NEUTRAL_L_LEG = 4004; // 4096 - position at very top, raw

void updatePositions();

// EXPANDER

PCF8574 Expander(0x38);

void updateButtons();
void expanderInit();

bool buttonUpL;
bool buttonUpR;
bool buttonDownL;
bool buttonDownR;
bool yellowSwitch;


// I2C MULTIPLEXER

QWIICMUX myMux;

void muxInit();
uint16_t getRAngleThroughMux();
uint16_t getLAngleThroughMux();

// LCD

LiquidCrystal_I2C lcd(0x27, 20, 4);
uint32_t lcdTimer = 0;

void lcdInit();
void setLCD();
void updateLCD();

bool lcdInfo = true;
void lcdSetInfo();

bool lcdPID = true;
void lcdSetPID();
void lcdUpdatePID();

bool lcdTargetPosition = true;
void lcdSetTargetPosition();
void lcdUpdateTargetPosition();

// LED

void ledRed(uint8_t);
void ledGreen(uint8_t);
void ledBlue(uint8_t);
void ledYellow(uint8_t);
void ledWhite(uint8_t);
void updateLED();

// PID

uint16_t rTargetPositionDegrees = 180;
uint16_t lTargetPositionDegrees = 180;

const uint8_t RDEADBAND = 44;
const uint8_t LDEADBAND = 46;

double rSetpoint, rInput, rOutput; //used by PID lib
double lSetpoint, lInput, lOutput;
//setpoint= nb Rotation of the motor shaft,
//input = current rotation,
//output is pwmSpeed of the motor

//Specify the links and initial tuning parameters
double rP = 1., rI = 0, rD = 0.; 
double lP = 1., lI = 0, lD = 0.; 

PID rPID(&rInput, &rOutput, &rSetpoint, rP, rI, rD, DIRECT); 
PID lPID(&lInput, &lOutput, &lSetpoint, lP, lI, lD, DIRECT); 

void pidInit();
void controlMotorPID();
void updatePID();

// PRINT

uint32_t printTimer = 0;
void printAll();

// PROCESS DATA

// make joystick value 

int16_t joystickRX;
int16_t joystickRY;
int16_t joystickLX;
int16_t joystickLY;

const uint8_t JOYSTICK_TRESHOLD = 30;

void processJoystick();
void zeroJoystick();


// PWM

const uint8_t R_F_PWM_CHAN = 0;
const uint8_t R_B_PWM_CHAN = 1;
const uint8_t L_F_PWM_CHAN = 2;
const uint8_t L_B_PWM_CHAN = 3;
const uint16_t PWM_FREQ = 24000; // TODO can be experimented with. https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/ledc.html?highlight=pwm#supported-range-of-frequency-and-duty-resolutions
const uint8_t PWM_RES = 8; // will receive error on serial if set too high.
const uint16_t PWM_RANGE = pow(2, PWM_RES) -1;
// const uint16_t PWM_RANGE = 128;

void pwmInit();

//REMOTE CONTROL

// 
uint16_t slideTestForward;
uint16_t slideTestBackward;

void joystickOrButtonsControlLegs();
void sliderPWMtest();


// END FORWARD DECLARATIONS
// **********************************

// MARK: -Setup


void setup() {
  pinMode(BOOT_SW_PIN, INPUT_PULLUP);
  pinMode(L_F_PWM_PIN, OUTPUT);
  digitalWrite(L_F_PWM_PIN, LOW);


  Serial.begin(115200);
  WiFi.mode(WIFI_MODE_STA);
  Serial.println(WiFi.macAddress());

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // data sent&receive callback
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  // register peer
  memcpy(peerInfo.peer_addr, remoteAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  pwmInit();
  pidInit();
  muxInit();
  lcdInit();
  expanderInit();

}

//**********************************
// MARK: -Loop


void loop() {
  checkReceiveTimeout();
  updateLED();
  updatePositions();
  updateButtons();
  updateLCD();
  updateBattery();
  updatePID();

  dataOut.rInput = rInput;
  dataOut.lInput = lInput;

  controlMotorPID();
  // sliderPWMtest();

  sendData();

  // joystickOrButtonsControlLegs();

  printAll();

}


//**********************************
// MARK: -FUNCTIONS


// -------------------------------
// MARK: - Battery

void updateBattery(){
  batterySamples.add(analogRead(BATTERY_V_PIN));

  batteryPercent = map(batterySamples.getAverage(), 2060, 2370, 0, 100); // 2060 =~ 3.65v, 2370 =~ 4.2v
  // TODO: Update for LIPO battery robot!!!

  // 3577 = 15.0v
  // 4045 = 16.3
  // 4095 = 16.7, oops... add voltage drop diode?

  // 14.9v = 20% (critical), 16.8v = 100%

  // low battery alarm, turn on again when updated for Robin

  // if (batteryPercent < 10 && batteryAlarmTimer < millis()){
  //   setBuzzer(300);
  //   batteryAlarmTimer = millis() + 600;
  // }
}

// -------------------------------
// MARK: - Buzzer

void setBuzzer(uint16_t time){
  buzzerTimer = millis() + time;
}
void updateBuzzer(){
  if (buzzerTimer > millis()){
    digitalWrite(BUZZER_PIN, HIGH);
  } else {
    digitalWrite(BUZZER_PIN, LOW);
  }
}


// -------------------------------
// MARK: - Data espnow


void resetReceiveTimeout(){
  lastReceiveTime = millis();
  connectionStatus = true;
}

void checkReceiveTimeout(){
  if (millis() - lastReceiveTime > 200){
    connectionStatus = false;
    zeroJoystick();
  }
}

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Serial.print("\r\nLast Packet Send Status:\t");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }

  //Note that too short interval between sending two ESP-NOW data may lead to disorder of sending callback function. So, it is recommended that sending the next ESP-NOW data after the sending callback function of the previous sending has returned.
  // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_now.html
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&dataIn, incomingData, sizeof(dataIn));
  // Serial.print("Bytes received: ");
  // Serial.println(len);
  processJoystick();
  resetReceiveTimeout();

  rP = dataIn.rP;
  rI = dataIn.rI;
  rD = dataIn.rD;

  lP = dataIn.lP;
  lI = dataIn.lI;
  lD = dataIn.lD;

  rTargetPositionDegrees = dataIn.rTargetPositionDegrees;
  lTargetPositionDegrees = dataIn.lTargetPositionDegrees;
}

void sendData(){
  
  if (dataTimer < millis()){
    esp_now_send(remoteAddress, (uint8_t *) &dataOut, sizeof(dataOut));

    dataTimer = millis() + 5;
  }
  
};


// -------------------------------
// MARK: - Encoders


void updatePositions(){
  positionLLegRaw = getLAngleThroughMux();
  positionRLegRaw = getRAngleThroughMux();

  positionLLegDegrees = 360 - fmod(((positionLLegRaw + NEUTRAL_L_LEG) / 4096.) * 360, 360);
  positionRLegDegrees = fmod(((positionRLegRaw + NEUTRAL_R_LEG) / 4096.) * 360, 360);
}

void captureLPWM()
{
  static uint32_t lastTime  = 0;
  uint32_t now = micros();
  if (digitalRead(ENCODER_L_PWM) == HIGH)
  {
    lPWMduration = now - lastTime;
  }

  Serial.print("*");
}

// -------------------------------
// MARK: - Expander


void expanderInit(){
  if (Expander.begin(255)){
    Serial.println("Expander connected");
  } else {
    Serial.println("Expander not found");
  };
}

void updateButtons(){

  uint8_t readings = Expander.read8();

  buttonUpL = !digitalRead(BOOT_SW_PIN);
  buttonUpR = !(readings & (1 << 0));
  buttonDownR = !(readings & (1 << 1));
  yellowSwitch = !(readings & (1 << 2));
  buttonDownL = !(readings & (1 << 3));

}

// -------------------------------
// MARK: - I2C multiplexer

void muxInit(){
  Wire.begin();
  if (myMux.begin() == false) {
    Serial.println("Mux not detected.");
  }

  myMux.setPort(0);
  rAs5600.begin();
  myMux.setPort(1);
  lAs5600.begin();

  if (!lAs5600.isConnected()){
    Serial.println("Left encoder not connected.");
  }

  myMux.setPort(0);
  if (!rAs5600.isConnected()){
    Serial.println("Right encoder not connected.");
  }

}

uint16_t getRAngleThroughMux(){
  myMux.setPort(0);
  return rAs5600.readAngle();
}

uint16_t getLAngleThroughMux(){
  myMux.setPort(1);
  return lAs5600.readAngle();
}

// -------------------------------
// MARK: - Lcd

void lcdInit(){
  lcd.init();
  lcd.clear();
  lcd.backlight();

  setLCD();
}

void setLCD()
{
  lcd.clear();

  if (lcdInfo){
    lcdSetInfo();
  }

  if (lcdPID){
    lcdSetPID();
  }

  if (lcdTargetPosition){
    lcdSetTargetPosition();
  }

}

void updateLCD()
{

  if (lcdTimer > millis())
  {
    return;
  }
  lcdTimer = millis() + 50; // update every 200 milliseconds

  if (lcdPID){
    lcdUpdatePID();
  }

  if (lcdTargetPosition){
    lcdUpdateTargetPosition();
  }

  lcd.setCursor(16, 0);
  lcd.print(batterySamples.getAverage());
  lcd.print(" ");
  // char batPerc[3];
  // sprintf(batPerc, "%02d", batteryPercent);
  // lcd.print(batPerc);
}

void lcdSetInfo(){
  lcd.setCursor(0, 0);
  lcd.print("Robin is awake!");
}

void lcdSetPID(){
  lcd.setCursor(0, 3);
  lcd.print("p:     i:     d:");
}

void lcdUpdatePID(){
  lcd.setCursor(2,3);
  lcd.print(rP, 1);
  lcd.setCursor(9,3);
  lcd.print(rI, 1);
  lcd.setCursor(16,3);
  lcd.print(rD, 1);
}

void lcdSetTargetPosition(){
  lcd.setCursor(0,1);
  lcd.print("tR:     tL:");
  lcd.setCursor(0,2);
  lcd.print("pR:     pL:");
}

void lcdUpdateTargetPosition(){
  lcd.setCursor(3,1);
  lcd.print(rTargetPositionDegrees);
  lcd.print(" ");
  lcd.setCursor(3,2);
  lcd.print(rInput, 0);
  lcd.print(" ");

  lcd.setCursor(11,1);
  lcd.print(lTargetPositionDegrees);
  lcd.print(" ");
  lcd.setCursor(11,2);
  lcd.print(lInput, 0);
  lcd.print(" ");
}


// -------------------------------
// MARK: - Led


void ledRed(uint8_t brightness = 10){
  analogWrite(LED_R_PIN, brightness);
  analogWrite(LED_G_PIN, 0);
  analogWrite(LED_B_PIN, 0);
}

void ledGreen(uint8_t brightness = 10){
  analogWrite(LED_R_PIN, 0);
  analogWrite(LED_G_PIN, brightness);
  analogWrite(LED_B_PIN, 0);
}

void ledBlue(uint8_t brightness = 10){
  analogWrite(LED_R_PIN, 0);
  analogWrite(LED_G_PIN, 0);
  analogWrite(LED_B_PIN, brightness);
}

void ledYellow(uint8_t brightness = 10){
  analogWrite(LED_R_PIN, brightness);
  analogWrite(LED_G_PIN, brightness * 0.4);
  analogWrite(LED_B_PIN, 0);
}

void ledWhite(uint8_t brightness = 10){
  analogWrite(LED_R_PIN, brightness);
  analogWrite(LED_G_PIN, brightness);
  analogWrite(LED_B_PIN, brightness);
}

void ledOff(){
  analogWrite(LED_R_PIN, 0);
  analogWrite(LED_G_PIN, 0);
  analogWrite(LED_B_PIN, 0);
}

void updateLED(){
  if (connectionStatus){
    ledBlue();
  } else {
    ledRed();
  }
}


// -------------------------------
// MARK: - PID

void pidInit(){
  rPID.SetMode(AUTOMATIC);
  rPID.SetOutputLimits(-PWM_RANGE, PWM_RANGE);
  rPID.SetSampleTime(1);

  lPID.SetMode(AUTOMATIC);
  lPID.SetOutputLimits(-PWM_RANGE, PWM_RANGE);
  lPID.SetSampleTime(1);
}

void updatePID(){
  rInput = positionRLegDegrees;
  rSetpoint = rTargetPositionDegrees;
  rPID.SetTunings(rP, rI, rD);
  rPID.Compute();

  lInput = positionLLegDegrees;
  lSetpoint = lTargetPositionDegrees;
  lPID.SetTunings(rP, rI, rD); // still R incoming
  lPID.Compute();
}

void controlMotorPID(){
  // right
  if (rOutput > 1){
    ledcWrite(R_B_PWM_CHAN, map(rOutput, 0, PWM_RANGE, RDEADBAND, PWM_RANGE));
    ledcWrite(R_F_PWM_CHAN, 0);
  }
  if (rOutput < -1){
    ledcWrite(R_F_PWM_CHAN, -map(rOutput, 0, -PWM_RANGE, -RDEADBAND, -PWM_RANGE));
    ledcWrite(R_B_PWM_CHAN, 0);
  }
  if (rOutput > -1 && rOutput < 1){
    ledcWrite(R_F_PWM_CHAN, 0);
    ledcWrite(R_B_PWM_CHAN, 0);
  }

  // left
  if (lOutput > 1){
    ledcWrite(L_B_PWM_CHAN, map(lOutput, 0, PWM_RANGE, LDEADBAND, PWM_RANGE));
    ledcWrite(L_F_PWM_CHAN, 0);
  }
  if (lOutput < -1){
    ledcWrite(L_F_PWM_CHAN, -map(lOutput, 0, -PWM_RANGE, -LDEADBAND, -PWM_RANGE));
    ledcWrite(L_B_PWM_CHAN, 0);
  }
  if (lOutput > -1 && lOutput < 1){
    ledcWrite(L_F_PWM_CHAN, 0);
    ledcWrite(L_B_PWM_CHAN, 0);
  }
}

// -------------------------------
// MARK: - Print

void printAll(){
  if (printTimer < millis()){
    printTimer = millis() + 300;
    Serial.print("R: ");
    Serial.print(positionRLegDegrees);
    Serial.print("\t");
    Serial.print(getRAngleThroughMux());
    Serial.print("L: ");
    Serial.print(positionLLegDegrees);
    Serial.print("\t");
    Serial.println(getLAngleThroughMux());

    Serial.print(buttonUpL);
    Serial.print(buttonDownL);
    Serial.print(yellowSwitch);
    Serial.print(buttonUpR);
    Serial.println(buttonDownR);

  }
}

// -------------------------------
// MARK: - Process data

void processJoystick(){
  // maps value to -PWM_RANGE to PWM_RANGE, if above/below JOYSTICK_THRESHOLD

  joystickRX = map(dataIn.joystickRX, 0, 4095, -PWM_RANGE, PWM_RANGE);
  joystickRY = map(dataIn.joystickRY, 0, 4095, -PWM_RANGE, PWM_RANGE);
  joystickLX = map(dataIn.joystickLX, 0, 4095, -PWM_RANGE, PWM_RANGE);
  joystickLY = map(dataIn.joystickLY, 0, 4095, -PWM_RANGE, PWM_RANGE);

  if (joystickRX < JOYSTICK_TRESHOLD && joystickRX > -JOYSTICK_TRESHOLD){
    joystickRX = 0;
  }
  if (joystickRY < JOYSTICK_TRESHOLD && joystickRY > -JOYSTICK_TRESHOLD){
    joystickRY = 0;
  }
  if (joystickLX < JOYSTICK_TRESHOLD && joystickLX > -JOYSTICK_TRESHOLD){
    joystickLX = 0;
  }
  if (joystickLY < JOYSTICK_TRESHOLD && joystickLY > -JOYSTICK_TRESHOLD){
    joystickLY = 0;
  }

}

void zeroJoystick(){
  joystickRX = 0;
  joystickRY = 0;
  joystickLX = 0;
  joystickLY = 0;
}

// -------------------------------
// MARK: - PWM

void pwmInit(){
  ledcSetup(R_F_PWM_CHAN, PWM_FREQ, PWM_RES);
  ledcAttachPin(R_F_PWM_PIN, R_F_PWM_CHAN);

  ledcSetup(R_B_PWM_CHAN, PWM_FREQ, PWM_RES);
  ledcAttachPin(R_B_PWM_PIN, R_B_PWM_CHAN);

  ledcSetup(L_F_PWM_CHAN, PWM_FREQ, PWM_RES);
  ledcAttachPin(L_F_PWM_PIN, L_F_PWM_CHAN);

  ledcSetup(L_B_PWM_CHAN, PWM_FREQ, PWM_RES);
  ledcAttachPin(L_B_PWM_PIN, L_B_PWM_CHAN);
}



// --------------------------------
// MARK: - Remote control

void joystickOrButtonsControlLegs(){

  uint8_t speed = 80;

  // right
  if (joystickRY > JOYSTICK_TRESHOLD){
    ledcWrite(R_F_PWM_CHAN, joystickRY /2);
  } else if (buttonUpR){
    ledcWrite(R_F_PWM_CHAN, speed);
  } else {
    ledcWrite(R_F_PWM_CHAN, 0);
  }

  if (joystickRY < -JOYSTICK_TRESHOLD){
    ledcWrite(R_B_PWM_CHAN, -joystickRY /2);
  } else if (buttonDownR){
    ledcWrite(R_B_PWM_CHAN, speed);
  }  else {
    ledcWrite(R_B_PWM_CHAN, 0);
  }

  // left
  if (joystickLY > JOYSTICK_TRESHOLD){
    ledcWrite(L_F_PWM_CHAN, joystickLY /2);
  } else if (buttonUpL){
    ledcWrite(L_F_PWM_CHAN, speed);
  } else {
    ledcWrite(L_F_PWM_CHAN, 0);
  }

  if (joystickLY < -JOYSTICK_TRESHOLD){
    ledcWrite(L_B_PWM_CHAN, -joystickLY /2);
  } else if (buttonDownL){
    ledcWrite(L_B_PWM_CHAN, speed);
  } else {
    ledcWrite(L_B_PWM_CHAN, 0);
  }

}

void sliderPWMtest(){
  slideTestForward = map(dataIn.sliderLL, 0, 17620, 0, 255);
  slideTestBackward = map(dataIn.sliderRL , 0, 17620, 0, 255);

  if (slideTestForward > 5){
    ledcWrite(R_F_PWM_CHAN, slideTestForward);
    ledcWrite(L_F_PWM_CHAN, slideTestForward);
  } else {
    ledcWrite(R_F_PWM_CHAN, 0);
    ledcWrite(L_F_PWM_CHAN, 0);
  }

  if (slideTestBackward > 5){
    ledcWrite(R_B_PWM_CHAN, slideTestBackward);
    ledcWrite(L_B_PWM_CHAN, slideTestBackward);
  } else {
    ledcWrite(R_B_PWM_CHAN, 0);
    ledcWrite(L_B_PWM_CHAN, 0);
  }

}



