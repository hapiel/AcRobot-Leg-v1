#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

uint8_t remoteAddress[] = {0x34, 0x94, 0x54, 0xBE, 0xDB, 0x6C}; 

typedef struct struct_data_out {
  char hi[6];
  uint16_t pot1;
  uint16_t pot2;
} struct_data_out;

// testing with 2 slide pot values
uint16_t pot1;
uint16_t pot2;

typedef struct struct_data_in {
  uint16_t pot1;
  uint16_t pot2;
} struct_data_in;

struct_data_out dataOut;
struct_data_in dataIn;

esp_now_peer_info_t peerInfo;

String success;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
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
  Serial.print("Bytes received: ");
  Serial.println(len);
  pot1 = dataIn.pot1;
  pot2 = dataIn.pot2;
}

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
}

void loop() {

  strcpy(dataOut.hi, "hello"); // easiest way to replace string
  dataOut.pot1 = pot1;
  dataOut.pot2 = 555;

  esp_now_send(remoteAddress, (uint8_t *) &dataOut, sizeof(dataOut));

  delay(100);


}