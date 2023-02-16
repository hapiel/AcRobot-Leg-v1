#include "AS5600.h"
#include "Wire.h"
#include <Arduino.h>

AS5600 as5600;   //  use default Wire

const uint8_t irqPin = 23;

volatile uint32_t duration = 0;

void capturePWM()
{
  static uint32_t lastTime  = 0;
  uint32_t now = micros();
  if (digitalRead(irqPin) == HIGH)
  {
    duration = now - lastTime;
  }
}


void setup()
{
  Serial.begin(115200);
  Serial.println(__FILE__);
  Serial.print("AS5600_LIB_VERSION: ");
  Serial.println(AS5600_LIB_VERSION);

  Wire.begin();

  attachInterrupt(digitalPinToInterrupt(irqPin), capturePWM, CHANGE);

  as5600.begin(4);
  as5600.setOutputMode(AS5600_OUTMODE_PWM);
  as5600.setPWMFrequency(AS5600_PWM_115);
}


void loop()
{
  Serial.print(millis());
  Serial.print("\t");
  Serial.print(as5600.readAngle());
  Serial.print("\t");
  Serial.print(as5600.rawAngle() * AS5600_RAW_TO_DEGREES);
  Serial.print("\t");
  Serial.println(duration);

  delay(100);
}

