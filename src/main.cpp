#include <Arduino.h>
#include <SoftwareSerial.h>
#include "tfmini-master\src\TFMini.h"
// #include "TFLidar.h"
// #include "GyverStepper-main\src\GyverStepper.h"
// создаем экземпляр AccelStepper
#include "AccelStepper-master\src\AccelStepper.h"

#define IN1 4
#define IN2 5
#define IN3 6
#define IN4 7
AccelStepper stepper2(8, IN4, IN2, IN3, IN1);
// GStepper<STEPPER4WIRE> stepper(2048, 7, 5, 6, 4);
// мотор с драйвером ULN2003 подключается по порядку пинов, но крайние нужно поменять местами
// то есть у меня подключено D2-IN1, D3-IN2, D4-IN3, D5-IN4, но в программе поменял 5 и 2
// создание объекта
// steps - шагов на один оборот вала (для расчётов с градусами)
// step, dir, pin1, pin2, pin3, pin4 - любые GPIO
// en - пин отключения драйвера, любой GPIO
// GStepper<STEPPER4WIRE> stepper(steps, pin1, pin2, pin3, pin4);      // драйвер 4 пин

TFMini tfmini;

SoftwareSerial SerialTFMini(2, 3); // The only value that matters here is the first one, 2, Rx

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
  Serial.begin(9600); // 9600 Initialize hardware serial port (serial debug port)
  Serial.println("Initializing0...");
  while (!Serial)
    ; // wait for serial port to connect. Needed for native USB port only
  Serial.println("Initializing...");
  SerialTFMini.begin(TFMINI_BAUDRATE); // 115200 Initialize the data rate for the SoftwareSerial port
  tfmini.begin(&SerialTFMini);         // Initialize the TF Mini sensor
  stepper2.setMaxSpeed(900.0);
  stepper2.setAcceleration(100.0);
  stepper2.setSpeed(200);
  stepper2.moveTo(2000);

  // stepper.setRunMode(KEEP_SPEED);
  // // можно установить скорость
  // stepper.setSpeed(120); // в шагах/сек
  // stepper.setSpeedDeg(90); // в градусах/сек
  //  режим следования к целевй позиции
  //  можно установить позицию
  // stepper.setTarget(-2024); // в шагах
  //  stepper.setTargetDeg(-360); // в градусах
  //  установка макс. скорости в градусах/сек
  //  stepper.setMaxSpeedDeg(400);

  // установка макс. скорости в шагах/сек
  // stepper.setMaxSpeed(1800);
  // установка ускорения в градусах/сек/сек
  // stepper.setAccelerationDeg(300);
  // установка ускорения в шагах/сек/сек
  // stepper.setAcceleration(300);
  // отключать мотор при достижении цели
  // stepper.autoPower(true);
}
bool dir = false;
void loop()
{
  int distance = 0;
  int strength = 0;

  getTFminiData(&distance, &strength);
  while (!distance)
  {
    getTFminiData(&distance, &strength);
    if (distance)
    {
      Serial.print(distance);
      Serial.print("cm\t");
      Serial.print("strength: ");
      Serial.println(strength);
    }
  }

  // if (!stepper.tick())
  // {
  //   dir = !dir;
  //   stepper.setTarget(dir ? -1024 : 1024);
  // }

  delay(100);
}