#include <Arduino.h>
#include "GyverStepper-main/src/GyverStepper.h"
// #include "Vector-master/src/Vector.h"

GStepper<STEPPER4WIRE> stepper(1, 5, 3, 4, 2);

class Point
{
public:
  int deg;
  int step;
  Point(int deg = 0, int step = 0)
  {
    this->deg = deg;
    this->step = step;
  };
};

// 4076
// steps for round = 2038
// 1 deg = 5.66111111 steps
void setup()
{
  Serial.begin(9600);
  stepper.setRunMode(KEEP_SPEED);
  stepper.setMaxSpeed(500); // скорость движения
  // stepper.setMaxSpeedDeg((int32_t)600);
  stepper.setSpeed(500);
  // stepper.setSpeedDeg(1);
}

bool dir = true;
int counter = 0;
int currentDeg = 0;
const int arrLen = 1000;
Point res[arrLen];
int arrI = 0;
// Vector<Point> res(storage);
void loop()
{
  if (!dir)
    return;
  stepper.tick();
  static uint32_t tmr = 0;
  if (millis() - tmr > 15)
  {
    tmr = millis();
    int cur = stepper.getCurrent();
    if (currentDeg != int(cur / 5.64) % 360)
    {
      currentDeg = int(cur / 5.64) % 360;
      // int currentSteps = stepper.getCurrent();
      Serial.print(counter);
      Serial.print("     ");
      // Serial.print(stepper.getCurrentDeg());
      Serial.print("     ");
      Serial.println(currentDeg);
      Point p(currentDeg, cur);
      res[arrI] = p;
      if (arrI < arrLen)
        arrI++;

      if (currentDeg == 0)
      {
        // stepper.setCurrent(0);
        counter += 1;
      }
      if (counter == 20)
      {
        dir = false;
        for (Point p : res)
          Serial.print(p.deg, p.step); // print out
      }
    }
  }
}
// void loop()
// {
//   stepper.tick();
//   static uint32_t tmr;
//   if (millis() - tmr >= 30)
//   {
//     tmr = millis();
//     int currentStep = stepper.getCurrent();
//     // Serial.println(currentStep);

//     // Вычисление текущего градуса поворота
//     float currentDegree = ((float)currentStep / 64.0) * 360.0; // 64 шага на один оборот

//     while (currentDegree > 360)
//       currentDegree -= 360;
//     Serial.println(currentDegree);
//   }
// }