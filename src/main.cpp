#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include "AS5600.h"
#include "Wire.h"
#include <PID_v1.h> //https://github.com/br3ttb/
#include <RunningMedian.h>

#define R_F_PWM_PIN  16  
#define R_B_PWM_PIN  17   
#define L_F_PWM_PIN  5  
#define L_B_PWM_PIN  18 
#define BATTERY_V_PIN 32

#define BUZZER_PIN 26
#define IR_PIN 27
#define LED_R_PIN 12
#define LED_G_PIN 13
#define LED_B_PIN 14

#define BOOT_SW_PIN 23


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
  char hi[6];
  uint16_t pot1;
  uint16_t pot2;
} struct_data_out;

// testing with 2 slide pot values
uint16_t pot1;
uint16_t pot2;

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
AS5600 lAs5600;

uint16_t positionRLegRaw;
uint16_t positionLLegRaw;
double positionRLegDegrees;
double positionLLegDegrees;
const uint16_t NEUTRAL_R_LEG = 0; // 4096 - position at very top, raw
const uint16_t NEUTRAL_L_LEG = 0; // 4096 - position at very top, raw

void updatePositions();


// LCD
// LED

void ledRed(uint8_t);
void ledGreen(uint8_t);
void ledBlue(uint8_t);
void ledYellow(uint8_t);
void ledWhite(uint8_t);
void updateLED();

// PID

double setpoint, input, output; //used by PID lib
//setpoint= nb Rotation of the motor shaft,
//input = current rotation,
//output is pwmSpeed of the motor

//Specify the links and initial tuning parameters
double Kp = 60., Ki = 1.2, Kd = 0.; 
PID rPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT); 

void pidInit();

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
// const uint16_t PWM_RANGE = pow(2, PWM_RES) -1;
const uint16_t PWM_RANGE = 128;

void pwmInit();

//REMOTE CONTROL

void joystickControlsLegs();


// END FORWARD DECLARATIONS
// **********************************

// MARK: -Setup


void setup() {
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

  rAs5600.begin();

}

//**********************************
// MARK: -Loop


void loop() {

  checkReceiveTimeout();
  updateLED();

  // TEST DATA
  strcpy(dataOut.hi, "hello"); // easiest way to replace string
  dataOut.pot1 = pot1;
  dataOut.pot2 = 555;

  sendData();

  joystickControlsLegs();
  


}


//**********************************
// MARK: -FUNCTIONS


// -------------------------------
// MARK: - Battery

void updateBattery(){
  batterySamples.add(analogRead(BATTERY_V_PIN));

  batteryPercent = map(batterySamples.getAverage(), 2060, 2370, 0, 100); // 2060 =~ 3.65v, 2370 =~ 4.2v
  // TODO: Update for LIPO battery robot

  // low battery alarm
  if (batteryPercent < 10 && batteryAlarmTimer < millis()){
    setBuzzer(300);
    batteryAlarmTimer = millis() + 600;
  }
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
}

void sendData(){
  
  if (dataTimer < millis()){
    esp_now_send(remoteAddress, (uint8_t *) &dataOut, sizeof(dataOut));

    dataTimer = millis() + 2;
  }
  
};


// -------------------------------
// MARK: - Encoders


void updatePositions(){
  positionRLegRaw = rAs5600.readAngle();
  // positionLLegRaw = lAs5600.readAngle();

  positionRLegDegrees = fmod(((positionRLegRaw + NEUTRAL_R_LEG) / 4096.) * 360, 360);
  positionLLegDegrees = fmod(((positionLLegRaw + NEUTRAL_L_LEG) / 4096.) * 360, 360);
}

// -------------------------------
// MARK: - Lcd

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
}

// -------------------------------
// MARK: - Print

void printAll(){
  if (printTimer < millis()){
    printTimer = millis() + 200;
    Serial.print(fmod(positionRLegDegrees, 360));
    Serial.print("\t");
    Serial.println(rAs5600.readStatus(),BIN);
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

void joystickControlsLegs(){

  // right
  if (joystickRY > JOYSTICK_TRESHOLD){
    ledcWrite(R_F_PWM_CHAN, joystickRY);
  } else {
    ledcWrite(R_F_PWM_CHAN, 0);
  }

  if (joystickRY < -JOYSTICK_TRESHOLD){
    ledcWrite(R_B_PWM_CHAN, -joystickRY);
  } else {
    ledcWrite(R_B_PWM_CHAN, 0);
  }

  // left
  if (joystickLY > JOYSTICK_TRESHOLD){
    ledcWrite(L_F_PWM_CHAN, joystickLY);
  } else {
    ledcWrite(L_F_PWM_CHAN, 0);
  }

  if (joystickLY < -JOYSTICK_TRESHOLD){
    ledcWrite(L_B_PWM_CHAN, -joystickLY);
  } else {
    ledcWrite(L_B_PWM_CHAN, 0);
  }

}




