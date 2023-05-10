#include <Arduino.h>
#include "GyverStepper-main/src/StepperCore.h"
#include "tfmini-master/src/TFMini.h"
#include <SoftwareSerial.h>

Stepper<STEPPER4WIRE> stepper(5, 3, 4, 2);

class Point
{
public:
  int deg;
  int step;
  int distance;
  int strength;
  Point(int deg = 0, int step = 0, int distance = 0, int strength = 0)
  {
    this->deg = deg;
    this->step = step;
    this->distance = distance;
    this->strength = strength;
  };
};

TFMini tfmini;

SoftwareSerial SerialTFMini(9, 8); // The only value that matters here is the first one, Rx

void getTFminiData(int *distance, int *strength)
{
  static char i = 0;
  char j = 0;
  int checksum = 0;
  static int rx[9];
  if (SerialTFMini.available())
  {
    rx[i] = SerialTFMini.read();
    if (rx[0] != 0x59)
    {
      i = 0;
    }
    else if (i == 1 && rx[1] != 0x59)
    {
      i = 0;
    }
    else if (i == 8)
    {
      for (j = 0; j < 8; j++)
      {
        checksum += rx[j];
      }
      if (rx[8] == (checksum % 256))
      {
        *distance = rx[2] + rx[3] * 256;
        *strength = rx[4] + rx[5] * 256;
      }
      i = 0;
    }
    else
    {
      i++;
    }
  }
}

void setup()
{
  Serial.begin(9600);
  // while (!Serial)
  //   ;
  Serial.println("Initializing...");
  stepper.dir = 1;                     // или -1
  stepper.pos = 0;                     // доступ к позиции
  SerialTFMini.begin(TFMINI_BAUDRATE); // 115200 Initialize the data rate for the SoftwareSerial port
  tfmini.begin(&SerialTFMini);         // Initialize the TF Mini sensor
}

bool dir = true;
int counter = 0;
int currentDeg = 0;
const int arrLen = 360;
uint32_t tmr = 0;
Point res[arrLen];
int arrI = 0;
// Vector<Point> res(storage);
void loop()
{
  if (!dir)
    return;
  stepper.step();
  int newDeg = int(stepper.pos / 5.64) % 360;
  if (currentDeg != newDeg)
  {
    currentDeg = newDeg;
    // Serial.print(counter);
    // Serial.print("     ");
    // Serial.println(currentDeg);

    int distance = 0;
    int strength = 0;

    getTFminiData(&distance, &strength);
    while (!distance)
    {
      getTFminiData(&distance, &strength);
      if (distance)
        // {
        Serial.println(distance);
      //   Serial.print("cm\t");
      //   Serial.print("strength: ");
      //   Serial.println(strength);
    }
    // }
    // return;

    Point p(currentDeg, stepper.pos, distance, strength);
    // res[arrI] = p;
    if (arrI < arrLen)
      arrI++;
    else
      arrI = 0;

    if (currentDeg == 0)
    {
      // stepper.setCurrent(0);
      counter += 1;
    }
    if (counter == 1)
    {
      dir = false;
      for (Point p : res)
      {
        Serial.println("");
        Serial.print("deg: ");       // print out
        Serial.print(p.deg);         // print out
        Serial.print(" step: ");     // print out
        Serial.print(p.step);        // print out
        Serial.print(" distance: "); // print out
        Serial.print(p.distance);    // print out
      }
    }
  }
  delay(5);
}