#include <Arduino.h>
#include "GyverStepper-main/src/GyverStepper.h"
// #include "Servo-master/src/Servo.h"

GStepper<STEPPER4WIRE> stepper(255, 5, 3, 4, 2);
// 4076
void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  stepper.setRunMode(KEEP_SPEED);
  stepper.setMaxSpeed(600); // скорость движения к цели
  // stepper.setMaxSpeedDeg((int32_t)600);
  // stepper.setAcceleration(700); // ускорение
  stepper.setSpeed(600);
  // stepper.setSpeedDeg(600);
  // stepper.setTargetDeg(180);

  // stepper.setTarget(1800);
}
bool dir = 1;
void loop()
{
  // put your main code here, to run repeatedly:

  // мотор асинхронно крутится тут
  // если приехали
  static uint32_t tmr;
  if (millis() - tmr >= 30)
  {
    tmr = millis();
    Serial.println(stepper.getCurrentDeg());
    // Serial.print(stepper.getCurrent());
    // Serial.print(stepper.getPeriod());
  }
  return;
  if (!stepper.tick())
  {
    dir = !dir;                      // разворачиваем
    stepper.setTargetDeg(dir * 180); // едем в другую сторону
  }
}