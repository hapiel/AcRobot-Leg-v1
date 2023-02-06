#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include "AS5600.h"
#include "Wire.h"
#include <PID_v1.h> //https://github.com/br3ttb/

#define R_F_PWM_PIN  5  
#define R_B_PWM_PIN  18   

uint8_t remoteAddress[] = {0x34, 0x94, 0x54, 0xBE, 0xDB, 0x6C}; 

AS5600 rAs5600; // encoder

//PID
double setpoint, input, output; //used by PID lib
//setpoint= nb Rotation of the motor shaft,
//input = current rotation,
//output is pwmSpeed of the motor

//Specify the links and initial tuning parameters
double Kp = 60., Ki = 1.2, Kd = 0.; 
PID rPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT); 

//PWM
const uint8_t R_F_PWM_CHAN = 0;
const uint8_t R_B_PWM_CHAN = 1;
const uint16_t PWM_FREQ = 24000; // TODO can be experimented with. https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/ledc.html?highlight=pwm#supported-range-of-frequency-and-duty-resolutions
const uint8_t PWM_RES = 8; // will receive error on serial if set too high.
const uint16_t PWM_RANGE = pow(2, PWM_RES) -1;

void motorInit(){
  ledcSetup(R_F_PWM_CHAN, PWM_FREQ, PWM_RES);
  ledcAttachPin(R_F_PWM_PIN, R_F_PWM_CHAN);

  ledcSetup(R_B_PWM_CHAN, PWM_FREQ, PWM_RES);
  ledcAttachPin(R_B_PWM_PIN, R_B_PWM_CHAN);

  rPID.SetMode(AUTOMATIC);
  rPID.SetOutputLimits(-PWM_RANGE, PWM_RANGE);
  rPID.SetSampleTime(1);

}


// DATA ESP-NOW
typedef struct struct_data_out {
  char hi[6];
  uint16_t pot1;
  uint16_t pot2;
} struct_data_out;

// testing with 2 slide pot values
uint16_t pot1;
uint16_t pot2;

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

struct_data_out dataOut;
struct_data_in dataIn;

esp_now_peer_info_t peerInfo;

String success;

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
}

uint32_t dataTimer = 0;

void sendData(){
  
  if (dataTimer < millis()){
    esp_now_send(remoteAddress, (uint8_t *) &dataOut, sizeof(dataOut));

    dataTimer = millis() + 2;
  }
  
};

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

  motorInit();
  rAs5600.begin();
  Wire.setTimeout(5);

}

boolean dirForward = true;
int16_t rEncoderRaw, rPrevEncoderRaw = 0;
uint16_t rEncoderRotations = 36000 ; // difficult to work with when below 0
const uint8_t GEAR_RATIO = 1;

long printTimer = 0;

const uint8_t JOYSTICK_TRESHOLD = 30;

long prevMillis = 0;

void loop() {

  if (millis() - prevMillis > 1){
    Serial.print("out of loop time: ");
    Serial.println(millis() - prevMillis);
  }

  long startLoop = micros();

  long sendDataTime = millis();

  // TEST DATA
  strcpy(dataOut.hi, "hello"); // easiest way to replace string
  dataOut.pot1 = pot1;
  dataOut.pot2 = 555;

  sendData();

  if (millis() - sendDataTime > 1){
    Serial.print("data time: ");
    Serial.println(millis() - sendDataTime);
  }

  

  uint32_t start = micros();

  rEncoderRaw = rAs5600.readAngle();
  
  uint32_t duration = micros() - start;
  if (duration > 1000)
  {
    Serial.print(millis());
    Serial.print("\t");
    Serial.print(duration);
    Serial.print("\t");
    Serial.println(rEncoderRaw);
  }
  
  long calculationsTime = millis();

  if ((rEncoderRaw - rPrevEncoderRaw) < -999) // big jump, must have turned full circle
  {
    rEncoderRotations++ ;
    dirForward = true;
  }
  if ((rEncoderRaw - rPrevEncoderRaw) > 999)
  {
    rEncoderRotations-- ;
    dirForward = false;
  }

  double motorAngle = (rEncoderRotations + rEncoderRaw/4096.) / GEAR_RATIO * 360;

  if (printTimer < millis()){
    printTimer = millis() + 200;
    Serial.print(fmod(motorAngle, 360));
    Serial.print("\t");
    Serial.println(rAs5600.readStatus(),BIN);
  }

  rPrevEncoderRaw = rEncoderRaw;

  if (millis() - calculationsTime > 1){
    Serial.print("calc time: ");
    Serial.println(millis() - calculationsTime);
  }

  long pwmTime = millis();

  int16_t joystickRY = map(dataIn.joystickRY, 0, 4095, -PWM_RANGE, PWM_RANGE);
  
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

  if (millis() - pwmTime > 1){
    Serial.print("pwm time: ");
    Serial.println(millis() - pwmTime);
  }

  // TODO: If not receiving data for too long, reduce speed to 0.

  if (micros() - startLoop > 800){
    // Serial.print("loop time: ");
    // Serial.println(micros() - startLoop);
  }

  prevMillis = millis();

}