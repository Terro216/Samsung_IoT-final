#include <Arduino.h>
#include <SoftwareSerial.h>
#include "AmperkaServo/src/AmperkaServo.h"
#include "tfmini-master/src/TFMini.h"
// #include "TFLidar.h"
// #include "GyverStepper-main/src/GyverStepper2.h"
// #include "CustomStepper/CustomStepper.h"
// #include "Stepper_28BYJ/ssss.h"

// const int stepsPerRevolution = 4076;
// CustomStepper stepper(8, 9, 10, 11);
// Stepper_28BYJ stepper(stepsPerRevolution, 8, 9, 10, 11);
// GStepper2<STEPPER4WIRE> stepper(2048, 8, 9, 10, 11);

constexpr uint8_t SERVO_PIN = 9;
// Speed values in the range [0, 180]. 0 is full speed in reverse,
// 90 is neutral, and 180 is full speed forward.
int FORWARD = 180;
int NEUTRAL = 90;
int REVERSE = 0;
AmperkaServo myservo;

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
    // while (!Serial)
    //     ; // wait for serial port to connect. Needed for native USB port only
    Serial.println("Initialized...");

    // SerialTFMini.begin(TFMINI_BAUDRATE); // 115200 Initialize the data rate for the SoftwareSerial port
    // tfmini.begin(&SerialTFMini);         // Initialize the TF Mini sensor
    // stepper.setSpeed(5);
    myservo.attach(SERVO_PIN, 544, 2400, 0, 180);
}

int stepCount = 0;

void loop()
{
    int distance = 0;
    int strength = 0;

    // getTFminiData(&distance, &strength);
    // while (!distance)
    // {
    //   getTFminiData(&distance, &strength);
    //   if (distance)
    //   {
    //     Serial.print(distance);
    //     Serial.print("cm\t");
    //     Serial.print("strength: ");
    //     Serial.println(strength);
    //   }
    // }

    // stepper.tick(); // мотор асинхронно крутится тут

    // // если приехали
    // if (stepper.ready())
    // {
    //   dir = !dir; // разворачиваем
    //   stepper.setTargetDeg(dir ? (int32_t)200 : -200);
    // }

    // // асинхронный вывод в порт
    // static uint32_t tmr;
    // if (millis() - tmr >= 30)
    // {
    //   tmr = millis();
    //   Serial.println(stepper.pos);
    // }
    Serial.println("loop");
    myservo.writeSpeed(255);
    // Ждём от 1 до 5 секунд
    delay(random(1000, 5000));
    // Останавливаем сервопривод
    myservo.writeSpeed(0);
    // Ждём одну секунду для полной остановки мотора
    delay(1000);
    // Считываем и выводим текущий угол сервопривода
    // Serial.println(myservo.readAngleFB());
    // Ждём одну секунду для просмотра результата
}

/*

#include <Arduino.h>
#include <SoftwareSerial.h>
#include "tfmini-master/src/TFMini.h"
// #include "TFLidar.h"
// #include "GyverStepper-main/src/GyverStepper2.h"
// создаем экземпляр AccelStepper
#include "AccelStepper-master\src\AccelStepper.h"
// #include "CustomStepper/CustomStepper.h"
// #include "Stepper_28BYJ/ssss.h"

AccelStepper stepper(AccelStepper::FULL4WIRE, 8, 9, 10, 11);
// CustomStepper stepper(4, 5, 6, 7);
// Stepper_28BYJ stepper(4078 * 2, 8, 9, 10, 11);
// GStepper2<STEPPER4WIRE> stepper(2048, 8, 9, 10, 11);

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
  Serial.println("Initialized...");
  // SerialTFMini.begin(TFMINI_BAUDRATE); // 115200 Initialize the data rate for the SoftwareSerial port
  // tfmini.begin(&SerialTFMini);         // Initialize the TF Mini sensor
  // stepper.setMaxSpeedDeg((int32_t)360);
  // // stepper.setMaxSpeed((int32_t)4000);

  // stepper.setTargetDeg((int32_t)200);
  stepper.setMaxSpeed(1000);    // in steps per second
  stepper.setAcceleration(500); // in steps per second^2
}

bool dir = 1;
void loop()
{
  int distance = 0;
  int strength = 0;

  // getTFminiData(&distance, &strength);
  // while (!distance)
  // {
  //   getTFminiData(&distance, &strength);
  //   if (distance)
  //   {
  //     Serial.print(distance);
  //     Serial.print("cm\t");
  //     Serial.print("strength: ");
  //     Serial.println(strength);
  //   }
  // }

  // stepper.tick(); // мотор асинхронно крутится тут

  // // если приехали
  // if (stepper.ready())
  // {
  //   dir = !dir; // разворачиваем
  //   stepper.setTargetDeg(dir ? (int32_t)200 : -200);
  // }

  // // асинхронный вывод в порт
  // static uint32_t tmr;
  // if (millis() - tmr >= 30)
  // {
  //   tmr = millis();
  //   Serial.println(stepper.pos);
  // }
  stepper.runSpeed();

  // Wait for 1 second before repeating the cycle
  delay(100);
}


*/