/* Ноябрь 2018
   roboPlatform - мобильная робоплатформа, выполняющая несколько функций
   Левый поворот
   Правый поворот
   Разворот
   Движение по прямой заданное количество времени
   Движение по линии (1, 2 или 3 датчика)
   Объезд препятствий
   Bluetooth-робот
*/
// -------------------- подключение библиотек ---------------------------------------
#include "PinChangeInterrupt.h"
// -------------------- Ультразвуковой датчик расстояния ----------------------------
#include <NewPing.h>
#define TRIGGER_PIN  10  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     11  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 20 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
// sonar.ping_cm() - Send ping, get distance in cm and print result (0 = outside set distance range)
//--------------------- EEPROM ------------------------------------------------------
#include <EEPROMex.h>
// все настраиваемые параметры будем хранить как float
byte addressFilter = 0;                    // занимает 4Б
byte addressBrightness = 5;                // занимает 4Б
byte addressDistance = 10;                 // занимает 4Б
byte addressMotorSpeed = 15;               // занимает 4Б
byte addressForwardSec = 20;               // занимает 4Б
byte addressLineSensorThresholdValue = 25; // занимает 4Б
//--------------------- EEPROM ------------------------------------------------------
//--------------------- Simple lowpass ----------------------------------------------
#include <MeanFilter.h>

float k = 0.7;                       // коэффициент фильтрации 0-1 через 0.01
MeanFilter leftFilter(k);
MeanFilter centreFilter(k);
MeanFilter rightFilter(k);

/*
   myF.filterAVG(sample);           вычислить следующее значение
   myF.setLast(sampleFiltered);     обновить прошлое значение
*/
// -------------------- Simple lowpass ----------------------------------------------
// -------------------- LCD1602 -----------------------------------------------------
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x3f, 16, 2);
#define printByte(args) write(args);

// ------------------------------------- СИМВОЛЫ -------------------------------------
byte pacman1[8] = {B00000, B01110, B10100, B11000, B11100, B01110, B00000, B00000};  // pacman1
// ----------------------- полоски ---------------------------------------------------
/*byte v1[8] = {B00000, B00000, B00000, B00000, B00000, B00000, B00000, B11111};
  byte v2[8] = {B00000, B00000, B00000, B00000, B00000, B00000, B11111, B00000};
  byte v3[8] = {B00000, B00000, B00000, B00000, B00000, B11111, B00000, B00000};
  byte v4[8] = {B00000, B00000, B00000, B00000, B11111, B00000, B00000, B00000};
  byte v5[8] = {B00000, B00000, B00000, B11111, B00000, B00000, B00000, B00000};
  byte v6[8] = {B00000, B00000, B11111, B00000, B00000, B00000, B00000, B00000};
  byte v7[8] = {B00000, B11111, B00000, B00000, B00000, B00000, B00000, B00000};
  byte v8[8] = {B11111, B00000, B00000, B00000, B00000, B00000, B00000, B00000};*/
// ------------------------ снежинка -------------------------------------------------
byte s1[8] = {0b00100, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000};
byte s2[8] = {0b10101, 0b00100, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000};
byte s3[8] = {0b01110, 0b10101, 0b00100, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000};
byte s4[8] = {0b10101, 0b01110, 0b10101, 0b00100, 0b00000, 0b00000, 0b00000, 0b00000};
byte s5[8] = {0b00100, 0b10101, 0b01110, 0b10101, 0b00100, 0b00000, 0b00000, 0b00000};
byte s6[8] = {0b00000, 0b00100, 0b10101, 0b01110, 0b10101, 0b00100, 0b00000, 0b00000};
byte s7[8] = {0b00000, 0b00000, 0b00100, 0b10101, 0b01110, 0b10101, 0b00100, 0b00000};
byte s8[8] = {0b00000, 0b00000, 0b00000, 0b00100, 0b10101, 0b01110, 0b10101, 0b00100};
// ------------------------------------- СИМВОЛЫ -------------------------------------
// -------------------- LCD1602 ------------------------------------------------------
byte butCentrePin = 4;                      // пин подключения центральной кнопки
byte butLeftPin = 2;                        // пин подключения левой кнопки
byte butRightPin = 3;                       // пин подключения правой кнопки
/*
   из-за быстрого опроса кнопки не может нормально работать "Движение по линии"
*/
unsigned long startButPressTimer = 0;       // стартовое значение таймера удержания
bool butIsLong = false;                     // флаг длинного нажатия кнопки
bool butIsSingle = false;                   // флаг однократного нажатия центральной кнопки
bool butIsDouble = false;                   // флаг двойного нажатия центральной кнопки
bool butIsTriple = false;                   // флаг тройного нажатия кнопки
volatile bool butLeftIsPress = false;       // флаг нажатия левой кнопки
volatile bool butRightIsPress = false;      // флаг нажатия правой кнопки
unsigned int butCentreHoldTime = 1500;      // время удержания для длинного нажатия
unsigned int shortButCount = 0;             // счетчик коротких нажатий
unsigned int longButCount = 0;              // счетчик длинных нажатий
byte lcdBackLightPin = 5;                   // пин управления подсветкой экрана
byte lcdBrightness = 50;                    // яркость lcd дисплея в процентах 0-100%
byte setDistance = 10;                      // переменная для настройки расстояния до препятствия
byte setMotorSpeed = 50;                    // переменная для настройки мощности мотора 0-100% - регулируется из меню робота
byte startValuePwmMotor = 50;               // начальная мощность моторов для П-регулятора
float koefProp = 0.15;                      // коэффициент пропорциональности для П-регулятора
float koefDif = 1.2;                        // дифференциальный коэффициент ПИД-регулятора
float koefInteg = 1.0;                      // интегральный коэффициент ПИД-регулятора
float koef[3] = {koefProp, koefDif, koefInteg};
String koefNames[3] = {"kp", "kd", "ki"};
byte sizeOfKoef = sizeof(koef) / sizeof(float) - 1;
float errOld = 0;                           // предыдущее значение ошибки для ПД-регулятора
float integralOld = 0;                      // предыдущее значение интегральной составляющей для ПИД-регулятора
byte delayBetweenActionLineSensor = 10;     // интервал измерения датчиков линии
byte setForwardSec = 5;                     // переменная для настройка времени движения робота по прямой
unsigned long startForwardRobot = 0;        // время старта робота "Движение по прямой"
bool robotStartFlag = false;                // флаг запуска какого-либо робота

/*
   Подкючение моторов через щилд на L293
   На каждый мотор 3 управляющих сигнала:
   2 направление вращения/остановка и 1 ШИМ
   Так как два первых сигнала в нормальном режиме работы
   всегда инверсны, то подключаем их черех транзисторный ключ
   Один сигнал берем со входа, другой с коллектора
   А к управляющему сигналу ШИМ подключаем порт с ШИМ
   В результате имеем 2 упр сигнала для каждого мотора
*/
byte leftMotorDirPin = 7;                                     // порт управления направлением вращения левого мотора
byte leftMotorPwmPin = 6;                                     // порт управления скоростью вращения левого мотора
byte rightMotorDirPin = 8;                                    // порт управления направлением вращения правого мотора
byte rightMotorPwmPin = 9;                                    // порт управления скоростью вращения правого мотора

byte leftLineSensor = A0;                                     // порт подключения левого ИК датчика линии
byte centreLineSensor = A1;                                   // порт подключения центрального ИК датчика линии
byte rightLineSensor = A2;                                    // порт подключения правого ИК датчика линии
byte sensorLabels[] = {0b01111111, 0b01011110, 0b01111110};   // значки для указания датчика - левый/центр/правый
int lineSensorNum = 0;                                        // переменная для выбора датчика в режиме "Linetracer 1"
//----------------------- описание меню ----------------------------------------------
String mainMenuItems[] = {"Left turn", "Right turn", "Circle turn", \
                          "Forward(sec)", "Linetracer 1", "Linetracer 2", \
                          "Linetracer 3", "Around wall", "Bluetooth robot", \
                          "Cal.lineSensor", "Set M speed", "Set distance", "Set brightness", \
                          "Set k filter"
                         };                                                       // пункты основного меню
byte mainMenuSize = sizeof(mainMenuItems) / sizeof(String) - 1;                   // размер (кол-во строк) основного меню (-1 так как счет с 0)
bool mainMenuFlag = true;                                                         // флаг основного меню
bool lcdDrawFlag = true;                                                          // флаг отрисовки дисплея

unsigned int refreshLcdTime = 500;                                                // период обновления дисплея
unsigned int lastRefreshLcd = 0;                                                  // время предыдущего обновления
int mainMenuCounter = 0;                                                          // счетчик позиции основного меню
// ------------------------- Подменю Calibration lineSensor --------------------------
String lineSensorMenuItems[] = {"Auto mode", "Manual mode", "Delay"};             // пункты меню Calibration lineSensor
byte lineSensorMenuSize = sizeof(lineSensorMenuItems) / sizeof(String) - 1;       // размер (кол-во строк) меню Calibration lineSensor (-1 так как счет с 0)
// counter = 0 -> auto mode, counter = 1 -> (manual mode)
int lineSensorMenuCounter = 0;                                                    // счетчик позиций подменю "Calibration lineSensor"
//настройку уровней белого и черного будем производить по левому сенсору
unsigned long whiteValue = 900;                                                   // уровень отраженного света от белой поверхности
unsigned long blackValue = 125;                                                   // уровень отраженного света от черной поверхности
byte lineSensorOffset = 25;                                                       // добавка к уровням белого и черного
long lineSensorThresholdValue = 0;                                                // порог принятия решения белый/черный для движения по линии
byte minlineSensorThresholdValue = 0;                                             // нижняя граница диапазона "серого"
byte maxlineSensorThresholdValue = 0;                                             // верхняя граница диапазона "серого"
byte thresholdPwmValue = 75;                                                      // пороговое значение ШИМ сигнала для пропорционального регулятора с одним датчиком
bool manualModeLineSensorMenuFlag = false;                                        // флаг ручной настройки порога белый/черный
bool delayBetweenActionLineSensorFlag = false;                                    // флаг настройки интервала опроса датчиков
byte lineSensorMask = 0b00000000;                                                 // маска значений датчиков линии
// ------------------------- Подменю Calibration lineSensor --------------------------
// ------------------------- Подменю Linetracer1 -------------------------------------
String line1MenuItems[] = {"Set sensor", "Relay", "Cubic", "PID"};          // пункты меню Linetracer1
byte line1MenuSize = sizeof(line1MenuItems) / sizeof(String) - 1;     // размер (кол-во строк) меню Linetracer1 (-1 так как счет с 0)
// counter = 0 -> auto mode, counter = 1 -> (manual mode)
int line1MenuCounter = 0;                                                   // счетчик позиций подменю "Linetracer1"
bool setSensorLine1MenuFlag = false;
bool relayLine1MenuFlag = false;
bool cubicLine1MenuFlag = false;
bool pidLine1MenuFlag = false;
int pidLine1MenuCounter = 0;
// ------------------------- Подменю Linetracer1 -------------------------------------

// флаги для всех подпунктов меню корневого уровня
bool leftTurnMenuFlag = false;
bool rightTurnMenuFlag = false;
bool circleTurnMenuFlag = false;
bool forwardMenuFlag = false;
bool line1MenuFlag = false;
bool line2MenuFlag = false;
bool line3MenuFlag = false;
bool wallMenuFlag = false;
bool bluetoothMenuFlag = false;
bool setLineSensorMenuFlag = false;
bool setSpeedMenuFlag = false;
bool setDistMenuFlag = false;
bool setBrightnessMenuFlag = false;         // флаг меню установки яркости дисплея
bool setKFilterMenuFlag = false;
//----------------------- описание меню -----------------------------------------------
//----------------------- заставки дисплея --------------------------------------------
int stringSnow[16];                         // массив снежинок
int SNOW_SPEED = 150;                       // скорость снегопада для заставки
unsigned long lastActionMillis = 0;         // время последнего действия для запуска заставки
//----------------------- заставки дисплея --------------------------------------------

void setup() {
  // запись в EEPROM
  /*EEPROM.writeFloat(addressFilter, k);
    EEPROM.writeFloat(addressBrightness, lcdBrightness);
    EEPROM.writeFloat(addressDistance, setDistance);
    EEPROM.writeFloat(addressMotorSpeed, setMotorSpeed);
    EEPROM.writeFloat(addressForwardSec, setForwardSec);*/
  k = EEPROM.readFloat(addressFilter);
  lcdBrightness = int(EEPROM.readFloat(addressBrightness));
  setDistance = int(EEPROM.readFloat(addressDistance));
  setMotorSpeed = int(EEPROM.readFloat(addressMotorSpeed));
  setForwardSec = int(EEPROM.readFloat(addressForwardSec));
  lineSensorThresholdValue = int(EEPROM.readFloat(addressLineSensorThresholdValue));
  pinMode(butCentrePin, INPUT);
  pinMode(butLeftPin, INPUT);
  pinMode(butRightPin, INPUT);
  pinMode(lcdBackLightPin, OUTPUT);
  pinMode(leftMotorDirPin, OUTPUT);
  pinMode(leftMotorPwmPin, OUTPUT);
  pinMode(rightMotorDirPin, OUTPUT);
  pinMode(rightMotorPwmPin, OUTPUT);
  analogWrite(leftMotorPwmPin, 0);            // останавливаем левый мотор
  analogWrite(rightMotorPwmPin, 0);           // останавливаем правый мотор
  Serial.begin(115200);
  lcd.init();
  lcdChars();
  lcd.backlight();
  analogWrite(lcdBackLightPin, map(lcdBrightness, 0, 100, 0, 255));
  lcd.setCursor(2, 0);
  lcd.print("roboPlatform");
  lcd.setCursor(10, 1);
  lcd.print("v1.1");
  attachInterrupt(0, butLeftRead, FALLING);
  attachInterrupt(1, butRightRead, FALLING);
  attachPCINT(digitalPinToPCINT(butCentrePin), butCentreRead, CHANGE);
  delay(2000);
  // заполняем массив снежинок случайным образом
  for (int i = 0; i < sizeof(stringSnow) / sizeof(int); i++) {
    stringSnow[i] = random(0, 16);
  }
}


void loop() {
  while (1) {
    //------------------ проверка нажатий кнопки --------------------
    unsigned long butCountTimer = millis() - startButPressTimer;
    if (butCountTimer > 650 and shortButCount != 0) {
      switch (shortButCount) {
        case 1:
          butIsSingle = true;
          Serial.println("butIsSingle");
          shortButCount = 0;
          break;
        case 2:
          butIsDouble = true;
          Serial.println("butIsDouble");
          shortButCount = 0;
          break;
        case 3:
          butIsTriple = true;
          Serial.println("butIsTriple");
          shortButCount = 0;
          break;
        default:
          butIsSingle = false;
          butIsDouble = false;
          butIsTriple = false;
          shortButCount = 0;
      }
    }
    //------------------ проверка нажатий кнопки --------------------
    //------------------ заставка -----------------------------------
    if (millis() - lastActionMillis > 10000) {
      screenSaver();
    }
    //------------------ заставка -----------------------------------
    // отрисовка основного меню
    if (mainMenuFlag and lcdDrawFlag) {
      mainMenuDraw();
      lcdDrawFlag = false;
    }

    // кнопки лево/право = движение вверх/вних по меню или настройка параметра меньше/больше
    //------------ КНОПКА ЛЕВО/МЕНЬШЕ/ВНИЗ -------------------------------
    if (butLeftIsPress) {
      lastActionMillis = millis();
      lcdDrawFlag = true;
      butLeftIsPress = false;
      // действия в основном меню
      if (mainMenuFlag) {
        lcdDrawFlag = true;
        if (mainMenuCounter == 0) {
          mainMenuCounter = mainMenuSize;
        } else {
          mainMenuCounter--;
        }
      }

      // действия в меню "Linetracer1"
      if (line1MenuFlag and !setSensorLine1MenuFlag and !relayLine1MenuFlag and !cubicLine1MenuFlag and !pidLine1MenuFlag) {
        if (line1MenuCounter == 0) {
          line1MenuCounter = line1MenuSize;
        } else {
          line1MenuCounter--;
        }
        lcdDrawFlag = true;
      }

      // действия в подменю "Linetracer 1: Set sensor"
      if (setSensorLine1MenuFlag) {
        lineSensorNum -= 1;
        lineSensorNum = constrain(lineSensorNum, 0, 2);
        lcdDrawFlag = true;
      }

      // действия в подменю "Linetracer 1: PID"
      if (pidLine1MenuFlag) {
        switch (pidLine1MenuCounter) {
          case 0:
            if (koefProp < 1)koefProp -= 0.01;
            if (koefProp >= 1 and koefProp < 10)koefProp -= 0.1;
            if (koefProp >= 10)koefProp -= 1;
            break;
          case 1:
            if (koefDif < 1)koefDif -= 0.01;
            if (koefDif >= 1 and koefDif < 10)koefDif -= 0.1;
            if (koefDif >= 10)koefDif -= 1;
            break;
          case 2:
            if (koefInteg < 1)koefInteg -= 0.01;
            if (koefInteg >= 1 and koefInteg < 10)koefInteg -= 0.1;
            if (koefInteg >= 10)koefInteg -= 1;
            break;
        }
        koef[0] = koefProp;
        koef[1] = koefDif;
        koef[2] = koefInteg;
        lcdDrawFlag = true;
      }

      // действия в меню "Calibration lineSensor"
      if (setLineSensorMenuFlag and !manualModeLineSensorMenuFlag and !delayBetweenActionLineSensorFlag) {
        if (lineSensorMenuCounter == 0) {
          lineSensorMenuCounter = lineSensorMenuSize;
        } else {
          lineSensorMenuCounter--;
        }
        lcdDrawFlag = true;
      }

      // действия в подменю "Calibration lineSensor: Manual mode"
      if (setLineSensorMenuFlag and manualModeLineSensorMenuFlag) {
        lineSensorThresholdValue -= 5;
        lcdDrawFlag = true;
      }

      // действия в подменю "Calibration lineSensor: Delay"
      if (setLineSensorMenuFlag and delayBetweenActionLineSensorFlag) {
        delayBetweenActionLineSensor -= 1;
        lcdDrawFlag = true;
      }

      // действия в меню "Установка яркости дисплея"
      if (setBrightnessMenuFlag) {
        lcdBrightness -= 5;
        lcdBrightness = constrain(lcdBrightness, 0, 100);
        analogWrite(lcdBackLightPin, map(lcdBrightness, 0, 100, 0, 255));
        lcdDrawFlag = true;
      }

      // действия в меню "Настройка скорости моторов"
      if (setSpeedMenuFlag) {
        setMotorSpeed -= 5;
        setMotorSpeed = constrain(setMotorSpeed, 0, 100);
        lcdDrawFlag = true;
      }

      // действия в меню "Настройка расстояния до препятствий"
      if (setDistMenuFlag) {
        setDistance -= 1;
        lcdDrawFlag = true;
      }

      // действия в меню "Forward (sec)"
      if (forwardMenuFlag) {
        setForwardSec -= 1;
        lcdDrawFlag = true;
      }

      // действия в меню "Set k filter"
      if (setKFilterMenuFlag) {
        k -= 0.01;
        k = constrain(k, 0.0, 1.0);
        lcdDrawFlag = true;
      }

    }

    //------------ КНОПКА ВПРАВО/БОЛЬШЕ/ВВЕРХ -------------------------------
    if (butRightIsPress) {
      lastActionMillis = millis();
      lcdDrawFlag = true;
      butRightIsPress = false;
      // действия в основном меню
      if (mainMenuFlag) {
        lcdDrawFlag = true;
        if (mainMenuCounter == mainMenuSize) {
          mainMenuCounter = 0;
        } else {
          mainMenuCounter++;
        }
      }

      // действия в меню "Linetracer1"
      if (line1MenuFlag and !setSensorLine1MenuFlag and !relayLine1MenuFlag and !cubicLine1MenuFlag and !pidLine1MenuFlag) {
        if (line1MenuCounter == line1MenuSize) {
          line1MenuCounter = 0;
        } else {
          line1MenuCounter++;
        }
        lcdDrawFlag = true;
      }

      // действия в подменю "Linetracer 1: Set sensor"
      if (setSensorLine1MenuFlag) {
        lineSensorNum += 1;
        lineSensorNum = constrain(lineSensorNum, 0, 2);
        lcdDrawFlag = true;
      }

      // действия в подменю "Linetracer 1: PID"
      if (pidLine1MenuFlag) {
        switch (pidLine1MenuCounter) {
          case 0:
            if (koefProp < 1)koefProp += 0.01;
            if (koefProp >= 1 and koefProp < 10)koefProp += 0.1;
            if (koefProp >= 10)koefProp += 1;
            break;
          case 1:
            if (koefDif < 1)koefDif += 0.01;
            if (koefDif >= 1 and koefDif < 10)koefDif += 0.1;
            if (koefDif >= 10)koefDif += 1;
            break;
          case 2:
            if (koefInteg < 1)koefInteg += 0.01;
            if (koefInteg >= 1 and koefInteg < 10)koefInteg += 0.1;
            if (koefInteg >= 10)koefInteg += 1;
            break;
        }
        koef[0] = koefProp;
        koef[1] = koefDif;
        koef[2] = koefInteg;
        lcdDrawFlag = true;
      }

      // действия в меню "Calibration lineSensor"
      if (setLineSensorMenuFlag and !manualModeLineSensorMenuFlag and !delayBetweenActionLineSensorFlag) {
        if (lineSensorMenuCounter == lineSensorMenuSize) {
          lineSensorMenuCounter = 0;
        } else {
          lineSensorMenuCounter++;
        }
        lcdDrawFlag = true;
      }

      // действия в подменю "Calibration lineSensor: Manual mode"
      if (setLineSensorMenuFlag and manualModeLineSensorMenuFlag) {
        lineSensorThresholdValue += 5;
        lcdDrawFlag = true;
      }

      // действия в подменю "Calibration lineSensor: Delay"
      if (setLineSensorMenuFlag and delayBetweenActionLineSensorFlag) {
        delayBetweenActionLineSensor += 1;
        lcdDrawFlag = true;
      }

      // действия в меню "Установка яркости дисплея"
      if (setBrightnessMenuFlag) {
        lcdBrightness += 5;
        lcdBrightness = constrain(lcdBrightness, 0, 100);
        analogWrite(lcdBackLightPin, map(lcdBrightness, 0, 100, 0, 255));
        lcdDrawFlag = true;
      }

      // действия в меню "Настройка скорости моторов"
      if (setSpeedMenuFlag) {
        setMotorSpeed += 5;
        setMotorSpeed = constrain(setMotorSpeed, 0, 100);
        lcdDrawFlag = true;
      }

      // действия в меню "Настройка расстояния до препятствий"
      if (setDistMenuFlag) {
        setDistance += 1;
        lcdDrawFlag = true;
      }

      // действия в меню "Forward (sec)"
      if (forwardMenuFlag) {
        setForwardSec += 1;
        lcdDrawFlag = true;
      }

      // действия в меню "Set k filter"
      if (setKFilterMenuFlag) {
        k += 0.01;
        k = constrain(k, 0.0, 1.0);
        lcdDrawFlag = true;
      }

    }

    //------------------------ SINGLE PRESS CENTRAL BUTTON -------------------------------------
    if (butIsSingle) {
      lastActionMillis = millis();
      // действия основного меню
      if (mainMenuFlag) {
        takeSubMenu();
        mainMenuFlag = false;
        lcdDrawFlag = true;
        goto END_SINGLE;
      }

      // действия в меню "Linetracer1"
      if (line1MenuFlag and !setSensorLine1MenuFlag and !relayLine1MenuFlag and !cubicLine1MenuFlag and !pidLine1MenuFlag) {
        switch (line1MenuCounter) {
          case 0:
            line1MenuFlag = false;
            setSensorLine1MenuFlag = true;
            relayLine1MenuFlag = false;
            cubicLine1MenuFlag = false;
            pidLine1MenuFlag = false;
            break;
          case 1:
            line1MenuFlag = false;
            setSensorLine1MenuFlag = false;
            relayLine1MenuFlag = true;
            cubicLine1MenuFlag = false;
            pidLine1MenuFlag = false;
            break;
          case 2:
            line1MenuFlag = false;
            setSensorLine1MenuFlag = false;
            relayLine1MenuFlag = false;
            cubicLine1MenuFlag = true;
            pidLine1MenuFlag = false;
            break;
          case 3:
            line1MenuFlag = false;
            setSensorLine1MenuFlag = false;
            relayLine1MenuFlag = false;
            cubicLine1MenuFlag = false;
            pidLine1MenuFlag = true;
            break;
        }
        lcdDrawFlag = true;
        goto END_SINGLE;
      }

      // действия в подменю "Linetracer 1: PID"
      if (pidLine1MenuFlag) {
        pidLine1MenuCounter += 1;
        if (pidLine1MenuCounter == 3) {
          pidLine1MenuCounter = 0;
        }
        //pidLine1MenuCounter = constrain(pidLine1MenuCounter, 0, 2);
        lcdDrawFlag = true;
      }

      // действия в меню "Calibration lineSensor"
      if (setLineSensorMenuFlag and !manualModeLineSensorMenuFlag) {
        switch (lineSensorMenuCounter) {
          case 0:
            autoModeLineSensor();
            break;
          case 1:
            manualModeLineSensorMenuFlag = true;
            //manualModeLineSensor();
            break;
          case 2:
            delayBetweenActionLineSensorFlag = true;
            //delayBetweenAction();
            break;
        }
        lcdDrawFlag = true;
        goto END_SINGLE;
      }

      // действия в подменю "Calibration lineSensor: Manual mode"
      if (setLineSensorMenuFlag and manualModeLineSensorMenuFlag) {
        saveSettings(addressLineSensorThresholdValue, lineSensorThresholdValue);
        lcdDrawFlag = true;
        goto END_SINGLE;
      }

      // действия в меню "Настройки яркости дисплея"
      if (setBrightnessMenuFlag) {
        saveSettings(addressBrightness, lcdBrightness);
        lcdDrawFlag = true;
        goto END_SINGLE;
      }

      // действия в меню "Настройка скорости моторов"
      if (setSpeedMenuFlag) {
        saveSettings(addressMotorSpeed, setMotorSpeed);
        lcdDrawFlag = true;
        goto END_SINGLE;
      }

      // действия в меню "Настройка расстояния до препятствий"
      if (setDistMenuFlag) {
        saveSettings(addressDistance, setDistance);
        lcdDrawFlag = true;
        goto END_SINGLE;
      }

      // действия в меня "Set k filter"
      if (setKFilterMenuFlag) {
        saveSettings(addressFilter, k);
        lcdDrawFlag = true;
        goto END_SINGLE;
      }

END_SINGLE:
      butIsSingle = false;
    }
    //------------------------ SINGLE PRESS CENTRAL BUTTON -------------------------------------
    //------------------------ DOUBLE PRESS CENTRAL BUTTON -------------------------------------
    if (butIsDouble) {
      if (pidLine1MenuFlag or leftTurnMenuFlag or rightTurnMenuFlag or circleTurnMenuFlag or forwardMenuFlag or relayLine1MenuFlag or cubicLine1MenuFlag or line2MenuFlag or line3MenuFlag or wallMenuFlag or bluetoothMenuFlag) {
        lcd.setCursor(0, 1);
        lcd.print("    working     ");
        delay(2000);                    // задержка, чтобы отойти от робота
        startForwardRobot = millis();
        robotStartFlag = true;
        lcdDrawFlag = false;
        goto END_DOUBLE;
      }
END_DOUBLE:
      butIsDouble = false;
    }
    //------------------------ DOUBLE PRESS CENTRAL BUTTON -------------------------------------
    // отрисовка общего меню для элементов Left, Right, Circle, LinetracerN, Around wall and Bluetooth robot
    if ((leftTurnMenuFlag or rightTurnMenuFlag or circleTurnMenuFlag or line2MenuFlag or line3MenuFlag or wallMenuFlag or bluetoothMenuFlag) and lcdDrawFlag) {
      commonMenu();
      lcdDrawFlag = false;
    }

    // запуск определенной программы робота
    if (robotStartFlag) {
      if (leftTurnMenuFlag) {
        leftTurnRobot();
      }

      if (rightTurnMenuFlag) {
        rightTurnRobot();
      }

      if (circleTurnMenuFlag) {
        circleTurnRobot();
      }

      if (forwardMenuFlag) {
        forwardRobot();
      }

      if (relayLine1MenuFlag) {
        line1RobotRelay();
      }

      if (cubicLine1MenuFlag) {
        line1RobotCub();
      }

      if (pidLine1MenuFlag) {
        line1RobotPID();
        }

      if (line2MenuFlag) {
        line2Robot();
      }

      if (line3MenuFlag) {
        line3Robot();
      }

      if (wallMenuFlag) {
        wallRobot();
      }

      if (bluetoothMenuFlag) {
        bluetoothRobot();
      }
    }

    // отрисовка меню "Linetracer 1"
    if (line1MenuFlag and lcdDrawFlag) {
      line1Menu();
      lcdDrawFlag = false;
    }

    // отрисовка меню "Linetracer 1: Set sensor"
    if (setSensorLine1MenuFlag and lcdDrawFlag) {
      setSensorLine1Menu();
      lcdDrawFlag = false;
    }

    // отрисовка меню "Linetracer 1: Relay"
    if (relayLine1MenuFlag and lcdDrawFlag) {
      relayLine1Menu();
      lcdDrawFlag = false;
    }

    // отрисовка меню "Linetracer 1: Cubic"
    if (cubicLine1MenuFlag and lcdDrawFlag) {
      cubicLine1Menu();
      lcdDrawFlag = false;
    }

    // отрисовка меню "Linetracer 1: PID"
    if (pidLine1MenuFlag and lcdDrawFlag) {
      pidLine1Menu();
      lcdDrawFlag = false;
    }

    // отрисовка меню "Calibration lineSensor"
    if (setLineSensorMenuFlag and lcdDrawFlag and !manualModeLineSensorMenuFlag and !delayBetweenActionLineSensorFlag) {
      setLineSensorMenu();
      lcdDrawFlag = false;
    }

    // отрисовка меню "Calibration lineSensor: Manual mode"
    if (setLineSensorMenuFlag and lcdDrawFlag and manualModeLineSensorMenuFlag) {
      manualModeLineSensor();
      lcdDrawFlag = false;
    }

    // отрисовка меню "Calibration lineSensor: Delay"
    if (setLineSensorMenuFlag and lcdDrawFlag and delayBetweenActionLineSensorFlag) {
      delayBetweenAction();
      lcdDrawFlag = false;
    }

    // отрисовка меню "Настройка яркости дисплея"
    if (setBrightnessMenuFlag and lcdDrawFlag) {
      setBrightnessMenu();
      lcdDrawFlag = false;
    }

    // отрисовка меню "Настройка скорости моторов"
    if (setSpeedMenuFlag and lcdDrawFlag) {
      setSpeedMenu();
      lcdDrawFlag = false;
    }

    //отрисовка меню "Настройка расстояния до препятствий"
    if (setDistMenuFlag and lcdDrawFlag) {
      setDistMenu();
      lcdDrawFlag = false;
    }

    //отрисовка меню "Настройка времени движения вперед"
    if (forwardMenuFlag and lcdDrawFlag) {
      forwardMenu();
      lcdDrawFlag = false;
    }

    //отрисовка меню "Настройка коэффициента фильтрации"
    if (setKFilterMenuFlag and lcdDrawFlag) {
      kFilterMenu();
      lcdDrawFlag = false;
    }

    //------------------------ LONG PRESS CENTRAL BUTTON -------------------------------------
    if (butIsLong) {
      lastActionMillis = millis();  // обновляем таймер действия, чтобы не уйти в заставку
      butIsLong = false;
      robotStartFlag = false;       // выход в основное меню = остановка работы робота (надо не забыть остановить все моторы)
      lcdDrawFlag = true;
      mainMenuFlag = true;
      leftTurnMenuFlag = false;
      rightTurnMenuFlag = false;
      circleTurnMenuFlag = false;
      forwardMenuFlag = false;
      if (line1MenuFlag) {
        line1MenuFlag = false;
      }
      line2MenuFlag = false;
      line3MenuFlag = false;
      wallMenuFlag = false;
      bluetoothMenuFlag = false;
      setLineSensorMenuFlag = false;
      setSpeedMenuFlag = false;
      setDistMenuFlag = false;
      setBrightnessMenuFlag = false;
      setKFilterMenuFlag = false;
      manualModeLineSensorMenuFlag = false;
      delayBetweenActionLineSensorFlag = false;
      if ((setSensorLine1MenuFlag or relayLine1MenuFlag or cubicLine1MenuFlag or pidLine1MenuFlag) and !line1MenuFlag) {
        line1MenuFlag = true;
        setSensorLine1MenuFlag = false;
        relayLine1MenuFlag = false;
        cubicLine1MenuFlag = false;
        pidLine1MenuFlag = false;
        mainMenuFlag = false;
      }
      analogWrite(leftMotorPwmPin, 0);
      analogWrite(rightMotorPwmPin, 0);
    }
    //------------------------ LONG PRESS CENTRAL BUTTON -------------------------------------
  }
}


// функция обработки нажатий центральной кнопки
void butCentreRead() {
  bool butState = !digitalRead(butCentrePin);

  // кнопка нажата и была нажата = удерживаем = "count time"
  if (butState) {
    startButPressTimer = millis();
  }

  // кнопка не нажата и была не нажата = не нажимали = "update startTimer"
  if (!butState) {
    unsigned long butPressTimer = millis() - startButPressTimer;
    if (butPressTimer < 250) {
      //butIsShort = true;
      //butIsLong = false;
      shortButCount += 1;
      //Serial.println("shortButCounter: " + String(shortButCount));
    }

    if (butPressTimer > butCentreHoldTime) {
      //butIsShort = false;
      butIsLong = true;
      //Serial.println("butIsLong");
    }
  }

}

// функция обработки левой кнопки
void butLeftRead() {
  if (!butLeftIsPress) {
    butLeftIsPress = true;
  }
}

// функция обработки правой кнопки
void butRightRead() {
  if (!butRightIsPress) {
    butRightIsPress = true;
  }
}

// функция отрисовки элементов основного меню
void mainMenuDraw() {
  lcd.clear();
  lcd.setCursor(0, 0);
  //lcd.printByte(0);                         // если указатель меню в виде кастомного символа
  lcd.printByte(0b01111110);
  byte nextItem = 0;
  if (mainMenuCounter == mainMenuSize) {
    nextItem = 0;
  } else {
    nextItem = mainMenuCounter + 1;
  }
  lcd.setCursor(1, 0);
  lcd.print(mainMenuItems[mainMenuCounter]);
  lcd.setCursor(1, 1);
  lcd.print(mainMenuItems[nextItem]);
}

// функция автоматической настройки порога белый/черный
void autoModeLineSensor() {
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print(lineSensorMenuItems[lineSensorMenuCounter]);
  lcd.setCursor(1, 1);
  lcd.print("Set white");                             // начинаем калибровку по белому цвету, необходимо в течении 5 секунд поднести датчики к белому
  lcd.setCursor(12, 1);
  unsigned long timer0 = millis();
  byte cnt = 0;
  while (millis() - timer0 < 6000) {
    lcd.setCursor(12, 1);
    lcd.print(cnt++);
    delay(1000);
  }
  lcd.setCursor(0, 1);
  lcd.print("                ");
  unsigned long meanValue[] = {0, 0, 0};              // начинаем процесс калибровки, десять раз считываем значение с каждого датчика и усредняем
  for (int sensor = 0; sensor < 3; sensor++) {
    lcd.setCursor(sensor * 5 + 1, 1);
    lcd.printByte(sensorLabels[sensor]);
    for (int sample = 0; sample < 10; sample++) {
      meanValue[sensor] += analogRead(sensor);
      delay(200);
    }
    lcd.setCursor(sensor * 5 + 2, 1);                // выводим позицию датчика
    lcd.print(int(meanValue[sensor] / 10.0));        // среднее значение уровня для данного датчика
  }
  delay(2000);
  // ищем максимальное значение
  int maxValueSensor = 0;
  for (int i = 0; i < 3; i++) {
    meanValue[i] = int(meanValue[i] / 10.0);
    if (meanValue[i] > maxValueSensor) {
      maxValueSensor = meanValue[i];
    }
  }

  whiteValue = maxValueSensor + lineSensorOffset;  // добавляем к уровню белого небольшой запас
  lcd.setCursor(0, 1);
  lcd.print("                ");
  lcd.setCursor(1, 1);
  lcd.print("whiteValue");
  lcd.setCursor(12, 1);
  lcd.print(whiteValue);
  delay(2000);

  // калибровка по черному
  lcd.setCursor(0, 1);
  lcd.print("                ");
  lcd.setCursor(1, 1);
  lcd.print("Set black");
  lcd.setCursor(12, 1);
  timer0 = millis();
  cnt = 0;
  while (millis() - timer0 < 6000) {
    lcd.setCursor(12, 1);
    lcd.print(cnt++);
    delay(1000);
  }
  lcd.setCursor(0, 1);
  lcd.print("                ");
  for (int i = 0; i < 3; i++) {
    meanValue[i] = 0;
  }
  for (int sensor = 0; sensor < 3; sensor++) {
    lcd.setCursor(sensor * 5 + 1, 1);
    lcd.printByte(sensorLabels[sensor]);
    for (int sample = 0; sample < 10; sample++) {
      meanValue[sensor] += analogRead(sensor);
      delay(200);
    }
    lcd.setCursor(sensor * 5 + 2, 1);
    lcd.print(int(meanValue[sensor] / 10.0));
  }
  delay(2000);
  // ищем максимальное значение
  int minValueSensor = 1023;
  for (int i = 0; i < 3; i++) {
    meanValue[i] = int(meanValue[i] / 10.0);
    if (meanValue[i] < minValueSensor) {
      minValueSensor = meanValue[i];
    }
  }

  blackValue = minValueSensor - lineSensorOffset;  // вычитаем небольшой запас из уровня черного
  lcd.setCursor(0, 1);
  lcd.print("                ");
  lcd.setCursor(1, 1);
  lcd.print("blackValue");
  lcd.setCursor(12, 1);
  lcd.print(blackValue);
  delay(2000);

  // вычисляем порог принятия решения белый/черный
  lineSensorThresholdValue = int(whiteValue + (blackValue - whiteValue) / 2.0);
  lcd.setCursor(0, 1);
  lcd.print("                ");
  lcd.setCursor(1, 1);
  lcd.print("Threshold");
  lcd.setCursor(12, 1);
  lcd.print(lineSensorThresholdValue);
  EEPROM.writeFloat(addressLineSensorThresholdValue, lineSensorThresholdValue);
  delay(2000);
  // вычисляем пропорциональный коэффициент для П - регулятора
  koefProp = 2.0 * (100 - startValuePwmMotor) / (blackValue - whiteValue);
  koef[0] = koefProp;
  lcd.setCursor(0, 1);
  lcd.print("                ");
  lcd.setCursor(1, 1);
  lcd.print("Kp");
  lcd.setCursor(12, 1);
  lcd.print(koefProp, 2);
  delay(2000);
  lastActionMillis = millis();
  setLineSensorMenuFlag = true;
  lcdDrawFlag = true;
}

// функция ручной настройки порога белый/черный
void manualModeLineSensor() {
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print(lineSensorMenuItems[lineSensorMenuCounter]);
  lcd.setCursor(1, 1);
  lcd.print("Threshold");
  lcd.setCursor(12, 1);
  lcd.print(lineSensorThresholdValue);
}

/*
    функция "Left turn", "Right turn", "Circle turn", "Linetracer1"
    "Linetracer2", "Linetracer3", "Around wall", "Bluetooth robot"
*/
void commonMenu() {
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print(mainMenuItems[mainMenuCounter]);
}

// робот "Левый поворот"
void leftTurnRobot() {
  digitalWrite(rightMotorDirPin, HIGH);
  // так как скорость задаем в процентах - необходимо перевести ее в биты
  analogWrite(rightMotorPwmPin, map(setMotorSpeed, 0, 100, 0, 255));
}

// робот "Правый поворот"
void rightTurnRobot() {
  digitalWrite(leftMotorDirPin, HIGH);
  // так как скорость задаем в процентах - необходимо перевести ее в биты
  analogWrite(leftMotorPwmPin, map(setMotorSpeed, 0, 100, 0, 255));
}

// робот "Разворот"
void circleTurnRobot() {
  digitalWrite(leftMotorDirPin, HIGH);
  analogWrite(leftMotorPwmPin, map(setMotorSpeed, 0, 100, 0, 255));
  digitalWrite(rightMotorDirPin, LOW);
  analogWrite(rightMotorPwmPin, map(setMotorSpeed, 0, 100, 0, 255));
}

// робот "Движение вперед по времени"
void forwardRobot() {
  while (millis() - startForwardRobot < setForwardSec * 1000) {
    digitalWrite(leftMotorDirPin, HIGH);
    analogWrite(leftMotorPwmPin, map(setMotorSpeed, 0, 100, 0, 255));
    digitalWrite(rightMotorDirPin, HIGH);
    analogWrite(rightMotorPwmPin, map(setMotorSpeed, 0, 100, 0, 255));
  }
  butIsLong = true;
}

// робот "Линия с 1 датчиком - ПИД-регулятор"
void line1RobotPID() {
  lastActionMillis = millis();
  int sample = analogRead(lineSensorNum);
  float sampleFiltered = 0;
  if (k == 0.0) {
    sampleFiltered = sample;
  } else {
    sampleFiltered = centreFilter.filterAVG(sample);
    centreFilter.setLast(sampleFiltered);
  }

  float kp = koefProp;                                               // пропорциональный коэффициент равен рассчитанному на этапа автокалибровки датчиков
  float kd = koefDif;
  float ki = koefInteg;

  float integralMax = 10;
  float integralMin = -1.0 * integralMax;

  float err = sampleFiltered - lineSensorThresholdValue;             // вычисляем ошибку = разность между текущим значениемс датчика и требуемым
  float errErr = err - errOld;                                       // вычисляем изменение ошибки
  errOld = err;                                                      // обновили значение предыдущей ошибки
  float  integral = integralOld + ki * err;                          // вычисляем интегральную составляющую

  // ограничение интегральной составляющей
  if (integral > integralMax) {
    integral = integralMax;
  }
  if (integral < integralMin) {
    integral = integralMin;
  }
  integralOld = integral;                                            // обновляем предыдущее значение интегральной составляющей

  float upid = kp * err + kd * errErr + integral;                    // рассчитываем пропорционально-дифференциальный регулятор
  float leftM  = constrain(startValuePwmMotor + upid, 0, 100);
  float rightM  = constrain(startValuePwmMotor - upid, 0, 100);

  digitalWrite(leftMotorDirPin, HIGH);
  digitalWrite(rightMotorDirPin, HIGH);

  int maxPwm = map(setMotorSpeed, 0, 100, 0, 255);                   // расчет максимального значения PWM для моторов в зависимости от выбранной мощности

  switch (lineSensorNum) {
    case 0:
      analogWrite(leftMotorPwmPin, map(rightM, 0, 100, 0, maxPwm));
      analogWrite(rightMotorPwmPin, map(leftM, 0, 100, 0, maxPwm));
      break;
    case 1:
      analogWrite(leftMotorPwmPin, map(rightM, 0, 100, 0, maxPwm));
      analogWrite(rightMotorPwmPin, map(leftM, 0, 100, 0, maxPwm));
      break;
    case 2:
      analogWrite(leftMotorPwmPin, map(leftM, 0, 100, 0, maxPwm));
      analogWrite(rightMotorPwmPin, map(rightM, 0, 100, 0, maxPwm));
      break;
  }
  delay(delayBetweenActionLineSensor);
}

// робот "Линия с 1 датчиком - Кубический регулятор"
void line1RobotCub() {
  lastActionMillis = millis();
  int sample = analogRead(lineSensorNum);
  float sampleFiltered = 0.0;
  if (k == 0.0) {
    sampleFiltered = sample;
  } else {
    sampleFiltered = centreFilter.filterAVG(sample);
    centreFilter.setLast(sampleFiltered);
  }

  float err = sampleFiltered - lineSensorThresholdValue;      // вычисляем ошибку = разность между текущим значениемс датчика и требуемым
  float upk = koefProp * err + 1.5e-7 * pow(err, 3);            // рассчитываем пропорционально-кубический коэфициент

  float leftM  = constrain(startValuePwmMotor + upk, 0, 100);
  float rightM  = constrain(startValuePwmMotor - upk, 0, 100);

  digitalWrite(leftMotorDirPin, HIGH);
  digitalWrite(rightMotorDirPin, HIGH);

  int maxPwm = map(setMotorSpeed, 0, 100, 0, 255);                   // расчет максимального значения PWM для моторов в зависимости от выбранной мощности
  analogWrite(leftMotorPwmPin, map(leftM, 0, 100, 0, maxPwm));
  analogWrite(rightMotorPwmPin, map(rightM, 0, 100, 0, maxPwm));
  delay(delayBetweenActionLineSensor);
}

// робот "Линия с 1 датчиком - Релейный регулятор"
void line1RobotRelay() {
  lastActionMillis = millis();
  int sample = analogRead(lineSensorNum);
  int sampleFiltered = 0;
  if (k == 0.0) {
    sampleFiltered = sample;
  } else {
    sampleFiltered = centreFilter.filterAVG(sample);
    centreFilter.setLast(sampleFiltered);
  }

  if (sampleFiltered < lineSensorThresholdValue) {
    switch (lineSensorNum) {
      case 0:
        analogWrite(rightMotorPwmPin, 0);
        digitalWrite(leftMotorDirPin, HIGH);
        analogWrite(leftMotorPwmPin, map(setMotorSpeed, 0, 100, 0, 255));
        break;
      case 1:
        analogWrite(rightMotorPwmPin, 0);
        digitalWrite(leftMotorDirPin, HIGH);
        analogWrite(leftMotorPwmPin, map(setMotorSpeed, 0, 100, 0, 255));

        break;
      case 2:
        analogWrite(leftMotorPwmPin, 0);
        digitalWrite(rightMotorDirPin, HIGH);
        analogWrite(rightMotorPwmPin, map(setMotorSpeed, 0, 100, 0, 255));
        break;
    }
  } else {
    switch (lineSensorNum) {
      case 0:
        analogWrite(leftMotorPwmPin, 0);
        digitalWrite(rightMotorDirPin, HIGH);
        analogWrite(rightMotorPwmPin, map(setMotorSpeed, 0, 100, 0, 255));
        break;
      case 1:
        analogWrite(leftMotorPwmPin, 0);
        digitalWrite(rightMotorDirPin, HIGH);
        analogWrite(rightMotorPwmPin, map(setMotorSpeed, 0, 100, 0, 255));
        break;
      case 2:
        analogWrite(rightMotorPwmPin, 0);
        digitalWrite(leftMotorDirPin, HIGH);
        analogWrite(leftMotorPwmPin, map(setMotorSpeed, 0, 100, 0, 255));
        break;
    }
  }
  delay(delayBetweenActionLineSensor);
}

// робот "Линия с 2 датчиками"
void line2Robot() {
  lastActionMillis = millis();
  int sampleLeft = analogRead(0);
  int sampleLeftFiltered = leftFilter.filterAVG(sampleLeft);
  leftFilter.setLast(sampleLeftFiltered);
  int sampleRight = analogRead(2);
  int sampleRightFiltered = rightFilter.filterAVG(sampleRight);
  rightFilter.setLast(sampleRightFiltered);

  // оба датчика на черном - останавливаемся
  if (sampleLeftFiltered >= lineSensorThresholdValue and sampleRightFiltered >= lineSensorThresholdValue) {
    analogWrite(leftMotorPwmPin, 0);
    analogWrite(rightMotorPwmPin, 0);
  }

  // левый датчик на черном/правый на белом - работает правый мотор
  if (sampleLeftFiltered >= lineSensorThresholdValue and sampleRightFiltered < lineSensorThresholdValue) {
    analogWrite(leftMotorPwmPin, 0);
    digitalWrite(rightMotorDirPin, HIGH);
    analogWrite(rightMotorPwmPin, map(setMotorSpeed, 0, 100, 0, 255));
  }

  // левый датчик на белом/правый на черном - работает левый мотор
  if (sampleLeftFiltered < lineSensorThresholdValue and sampleRightFiltered >= lineSensorThresholdValue) {
    analogWrite(rightMotorPwmPin, 0);
    digitalWrite(leftMotorDirPin, HIGH);
    analogWrite(leftMotorPwmPin, map(setMotorSpeed, 0, 100, 0, 255));
  }

  // оба датчика на белом - оба мотора работают
  if (sampleLeftFiltered < lineSensorThresholdValue and sampleRightFiltered < lineSensorThresholdValue) {
    digitalWrite(rightMotorDirPin, HIGH);
    analogWrite(rightMotorPwmPin, map(setMotorSpeed, 0, 100, 0, 255));
    digitalWrite(leftMotorDirPin, HIGH);
    analogWrite(leftMotorPwmPin, map(setMotorSpeed, 0, 100, 0, 255));
  }
  delay(delayBetweenActionLineSensor);
}

// робот "Линия с 3 датчиками"
void line3Robot() {
  lastActionMillis = millis();
  int sampleLeft = analogRead(0);
  int sampleLeftFiltered = leftFilter.filterAVG(sampleLeft);
  leftFilter.setLast(sampleLeftFiltered);
  int sampleRight = analogRead(2);
  int sampleRightFiltered = rightFilter.filterAVG(sampleRight);
  rightFilter.setLast(sampleRightFiltered);
  int sampleCentre = analogRead(1);
  int sampleCentreFiltered = centreFilter.filterAVG(sampleCentre);
  centreFilter.setLast(sampleCentreFiltered);
  byte controlByte = 0b000;

  if (sampleLeftFiltered > lineSensorThresholdValue) {
    bitSet(controlByte, 2);
  } else {
    bitClear(controlByte, 2);
  }
  if (sampleCentreFiltered > lineSensorThresholdValue) {
    bitSet(controlByte, 1);
  } else {
    bitClear(controlByte, 1);
  }
  if (sampleRightFiltered > lineSensorThresholdValue) {
    bitSet(controlByte, 0);
  } else {
    bitClear(controlByte, 0);
  }

  switch (controlByte) {
    case 0b000:   // все датчики на белом
      break;

    case 0b001:   // линия справа - левый мотор работает
      analogWrite(rightMotorPwmPin, 0);
      digitalWrite(leftMotorDirPin, HIGH);
      analogWrite(leftMotorPwmPin, map(setMotorSpeed, 0, 100, 0, 255));
      break;

    case 0b010:   // линия по центру
      digitalWrite(rightMotorDirPin, HIGH);
      analogWrite(rightMotorPwmPin, map(setMotorSpeed, 0, 100, 0, 255));
      digitalWrite(leftMotorDirPin, HIGH);
      analogWrite(leftMotorPwmPin, map(setMotorSpeed, 0, 100, 0, 255));
      break;

    case 0b011:   // линия ушла немного вправо
      digitalWrite(rightMotorDirPin, HIGH);
      analogWrite(rightMotorPwmPin, map(setMotorSpeed, 0, 100, 0, 200));
      digitalWrite(leftMotorDirPin, HIGH);
      analogWrite(leftMotorPwmPin, map(setMotorSpeed, 0, 100, 0, 255));
      break;

    case 0b100:   // линия слева - работает правый мотор
      analogWrite(leftMotorPwmPin, 0);
      digitalWrite(rightMotorDirPin, HIGH);
      analogWrite(rightMotorPwmPin, map(setMotorSpeed, 0, 100, 0, 255));
      break;

    case 0b101:   // не используется
      break;

    case 0b110:   // линия ушла немного влево
      digitalWrite(rightMotorDirPin, HIGH);
      analogWrite(rightMotorPwmPin, map(setMotorSpeed, 0, 100, 0, 255));
      digitalWrite(leftMotorDirPin, HIGH);
      analogWrite(leftMotorPwmPin, map(setMotorSpeed, 0, 100, 0, 200));

      break;

    case 0b111:   // не используется
      break;
  }
}

// робот "Объезд препятствий"
void wallRobot() {

}

// робот "Робот с управлением по Bluetooth"
void bluetoothRobot() {

}

// функция "Forward"
void forwardMenu() {
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print(mainMenuItems[mainMenuCounter]);
  lcd.setCursor(1, 1);
  lcd.print("Seconds");
  lcd.setCursor(13, 1);
  lcd.print(setForwardSec);
}

// функция "Linetracer 1"
// выбор датчика и выбор алгоритма движения 1.Релейный 2.Кубический 3.ПИД с настройкой коэффициентов
void line1Menu() {
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print(mainMenuItems[mainMenuCounter]);
  lcd.setCursor(1, 1);
  lcd.print(line1MenuItems[line1MenuCounter]);
}

// функция отрисовки подменю "Linetracer1: Set sensor"
void setSensorLine1Menu() {
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print(line1MenuItems[line1MenuCounter]);
  switch (lineSensorNum) {
    case 0:
      lcd.setCursor(1, 1);
      break;
    case 1:
      lcd.setCursor(8, 1);
      break;
    case 2:
      lcd.setCursor(15, 1);
      break;
  }
  lcd.printByte(sensorLabels[lineSensorNum]);
}

// функция отрисовки подменю "Linetracer1: Relay"
void relayLine1Menu() {
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print(line1MenuItems[line1MenuCounter]);
}

// функция отрисовки подменю "Linetracer1: Cubic"
void cubicLine1Menu() {
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print(line1MenuItems[line1MenuCounter]);
}

// функция отрисовки подменю "Linetracer1: PID"
void pidLine1Menu() {
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print(line1MenuItems[line1MenuCounter]);
  koef[0] = koefProp;
  koef[1] = koefDif;
  koef[2] = koefInteg;
  for (int i = 0; i <= sizeOfKoef; i++) {
    lcd.setCursor(4 * i + 5, 0);
    lcd.print(koefNames[i]);
  }

  for (int i = 0; i <= sizeOfKoef; i++) {
    if (koef[i] < 1) {
      lcd.setCursor(4 * i + 3, 1);
      lcd.print(koef[i], 2);
    }
    if (koef[i] >= 1 and koef[i] < 10) {
      lcd.setCursor(4 * i + 4, 1);
      lcd.print(koef[i], 1);
    }
    if (koef[i] >= 10) {
      lcd.setCursor(4 * i + 5, 1);
      lcd.print(koef[i], 0);
    }
  }

  lcd.setCursor(pidLine1MenuCounter * 4 + 3, 1);
  lcd.printByte(0b01111110);                        // отрисовываем стрелку указатель на текущий элемент
}

// функция отрисовки "Настройка порога белый/черный для датчиков линии"
void setLineSensorMenu() {
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print(mainMenuItems[mainMenuCounter]);
  lcd.setCursor(1, 1);
  lcd.print(lineSensorMenuItems[lineSensorMenuCounter]);
}

void delayBetweenAction() {
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print(mainMenuItems[mainMenuCounter]);
  lcd.setCursor(1, 1);
  lcd.print(lineSensorMenuItems[lineSensorMenuCounter]);
  lcd.setCursor(12, 1);
  lcd.print(delayBetweenActionLineSensor);
}

// функция отрисовки "Настройка яркости дисплея"
void setBrightnessMenu() {
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print(mainMenuItems[mainMenuCounter]);
  lcd.setCursor(1, 1);
  lcd.print("Brightness");
  lcd.setCursor(12, 1);
  lcd.print(lcdBrightness);
  lcd.setCursor(15, 1);
  lcd.print("%");
}

// функция отрисовки "Настройка скорости моторов"
void setSpeedMenu() {
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print(mainMenuItems[mainMenuCounter]);
  lcd.setCursor(1, 1);
  lcd.print("Speed");
  lcd.setCursor(12, 1);
  lcd.print(setMotorSpeed);
  lcd.setCursor(15, 1);
  lcd.print("%");
}

// функция отрисовки "Настройка расстояния до препятствий"
void setDistMenu() {
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print(mainMenuItems[mainMenuCounter]);
  lcd.setCursor(1, 1);
  lcd.print("Distance");
  lcd.setCursor(13, 1);
  lcd.print(setDistance);
}

// функция "Set k filter"
void kFilterMenu() {
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print(mainMenuItems[mainMenuCounter]);
  lcd.setCursor(5, 1);
  lcd.print("k:");
  lcd.setCursor(7, 1);
  if (k == 0.0) {
    lcd.print("off ");
  } else {
    lcd.print(k);
  }
}

// функция отрисовки "Сохранения настроек"
// передаем в функцию адрес в памяти EEPROM и значение которое нужно записать
void saveSettings(byte address, float value) {
  EEPROM.writeFloat(address, value);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Saving settings");
  for (int i = 0; i < 16; i++) {
    lcd.setCursor(i, 1);
    lcd.print(">");
    delay(125);
  }
}

// функция заставки
void screenSaver() {
  // отрисовываем снегопад
  for (int x = 0; x < sizeof(stringSnow) / sizeof(int); x++) {
    int num = 0;
    if (stringSnow[x] <= 7) {
      lcd.setCursor(x, 1);
      lcd.print(" ");
      num = stringSnow[x];
      lcd.setCursor(x, 0);
    } else {
      num = stringSnow[x] - 8;
      lcd.setCursor(x, 0);
      lcd.print(" ");
      lcd.setCursor(x, 1);
    }
    lcd.printByte(num);
  }
  // сдвигаем снегопад вниз, если элемент вышел за границы, то генерируем новую снежинку случайно
  for (int i = 0; i < sizeof(stringSnow) / sizeof(int); i++) {
    int newValue = stringSnow[i] + 1;
    if (newValue > 15) {
      stringSnow[i] = random(0, 8);
    } else {
      stringSnow[i] = newValue;
    }
  }
  delay(SNOW_SPEED);
}

// создание кастомных символов для дисплея
void lcdChars() {
  lcd.createChar(0, s1);
  lcd.createChar(1, s2);
  lcd.createChar(2, s3);
  lcd.createChar(3, s4);
  lcd.createChar(4, s5);
  lcd.createChar(5, s6);
  lcd.createChar(6, s7);
  lcd.createChar(7, s8);
}

// функция выбора подменю
void takeSubMenu() {
  switch (mainMenuCounter) {
    case 0:
      leftTurnMenuFlag = true;
      rightTurnMenuFlag = false;
      circleTurnMenuFlag = false;
      forwardMenuFlag = false;
      line1MenuFlag = false;
      line2MenuFlag = false;
      line3MenuFlag = false;
      wallMenuFlag = false;
      bluetoothMenuFlag = false;
      setLineSensorMenuFlag = false;
      setSpeedMenuFlag = false;
      setDistMenuFlag = false;
      setBrightnessMenuFlag = false;
      setKFilterMenuFlag = false;
      break;
    case 1:
      leftTurnMenuFlag = false;
      rightTurnMenuFlag = true;
      circleTurnMenuFlag = false;
      forwardMenuFlag = false;
      line1MenuFlag = false;
      line2MenuFlag = false;
      line3MenuFlag = false;
      wallMenuFlag = false;
      bluetoothMenuFlag = false;
      setLineSensorMenuFlag = false;
      setSpeedMenuFlag = false;
      setDistMenuFlag = false;
      setBrightnessMenuFlag = false;
      setKFilterMenuFlag = false;
      break;
    case 2:
      leftTurnMenuFlag = false;
      rightTurnMenuFlag = false;
      circleTurnMenuFlag = true;
      forwardMenuFlag = false;
      line1MenuFlag = false;
      line2MenuFlag = false;
      line3MenuFlag = false;
      wallMenuFlag = false;
      bluetoothMenuFlag = false;
      setLineSensorMenuFlag = false;
      setSpeedMenuFlag = false;
      setDistMenuFlag = false;
      setBrightnessMenuFlag = false;
      setKFilterMenuFlag = false;
      break;
    case 3:
      leftTurnMenuFlag = false;
      rightTurnMenuFlag = false;
      circleTurnMenuFlag = false;
      forwardMenuFlag = true;
      line1MenuFlag = false;
      line2MenuFlag = false;
      line3MenuFlag = false;
      wallMenuFlag = false;
      bluetoothMenuFlag = false;
      setLineSensorMenuFlag = false;
      setSpeedMenuFlag = false;
      setDistMenuFlag = false;
      setBrightnessMenuFlag = false;
      setKFilterMenuFlag = false;
      break;
    case 4:
      leftTurnMenuFlag = false;
      rightTurnMenuFlag = false;
      circleTurnMenuFlag = false;
      forwardMenuFlag = false;
      line1MenuFlag = true;
      line2MenuFlag = false;
      line3MenuFlag = false;
      wallMenuFlag = false;
      bluetoothMenuFlag = false;
      setLineSensorMenuFlag = false;
      setSpeedMenuFlag = false;
      setDistMenuFlag = false;
      setBrightnessMenuFlag = false;
      setKFilterMenuFlag = false;
      break;
    case 5:
      leftTurnMenuFlag = false;
      rightTurnMenuFlag = false;
      circleTurnMenuFlag = false;
      forwardMenuFlag = false;
      line1MenuFlag = false;
      line2MenuFlag = true;
      line3MenuFlag = false;
      wallMenuFlag = false;
      bluetoothMenuFlag = false;
      setLineSensorMenuFlag = false;
      setSpeedMenuFlag = false;
      setDistMenuFlag = false;
      setBrightnessMenuFlag = false;
      setKFilterMenuFlag = false;
      break;
    case 6:
      leftTurnMenuFlag = false;
      rightTurnMenuFlag = false;
      circleTurnMenuFlag = false;
      forwardMenuFlag = false;
      line1MenuFlag = false;
      line2MenuFlag = false;
      line3MenuFlag = true;
      wallMenuFlag = false;
      bluetoothMenuFlag = false;
      setLineSensorMenuFlag = false;
      setSpeedMenuFlag = false;
      setDistMenuFlag = false;
      setBrightnessMenuFlag = false;
      setKFilterMenuFlag = false;
      break;
    case 7:
      leftTurnMenuFlag = false;
      rightTurnMenuFlag = false;
      circleTurnMenuFlag = false;
      forwardMenuFlag = false;
      line1MenuFlag = false;
      line2MenuFlag = false;
      line3MenuFlag = false;
      wallMenuFlag = true;
      bluetoothMenuFlag = false;
      setLineSensorMenuFlag = false;
      setSpeedMenuFlag = false;
      setDistMenuFlag = false;
      setBrightnessMenuFlag = false;
      setKFilterMenuFlag = false;
      break;
    case 8:
      leftTurnMenuFlag = false;
      rightTurnMenuFlag = false;
      circleTurnMenuFlag = false;
      forwardMenuFlag = false;
      line1MenuFlag = false;
      line2MenuFlag = false;
      line3MenuFlag = false;
      wallMenuFlag = false;
      bluetoothMenuFlag = true;
      setLineSensorMenuFlag = false;
      setSpeedMenuFlag = false;
      setDistMenuFlag = false;
      setBrightnessMenuFlag = false;
      setKFilterMenuFlag = false;
      break;
    case 9:
      leftTurnMenuFlag = false;
      rightTurnMenuFlag = false;
      circleTurnMenuFlag = false;
      forwardMenuFlag = false;
      line1MenuFlag = false;
      line2MenuFlag = false;
      line3MenuFlag = false;
      wallMenuFlag = false;
      bluetoothMenuFlag = false;
      setLineSensorMenuFlag = true;
      setSpeedMenuFlag = false;
      setDistMenuFlag = false;
      setBrightnessMenuFlag = false;
      setKFilterMenuFlag = false;
      break;
    case 10:
      leftTurnMenuFlag = false;
      rightTurnMenuFlag = false;
      circleTurnMenuFlag = false;
      forwardMenuFlag = false;
      line1MenuFlag = false;
      line2MenuFlag = false;
      line3MenuFlag = false;
      wallMenuFlag = false;
      bluetoothMenuFlag = false;
      setLineSensorMenuFlag = false;
      setSpeedMenuFlag = true;
      setDistMenuFlag = false;
      setBrightnessMenuFlag = false;
      setKFilterMenuFlag = false;
      break;
    case 11:
      leftTurnMenuFlag = false;
      rightTurnMenuFlag = false;
      circleTurnMenuFlag = false;
      forwardMenuFlag = false;
      line1MenuFlag = false;
      line2MenuFlag = false;
      line3MenuFlag = false;
      wallMenuFlag = false;
      bluetoothMenuFlag = false;
      setLineSensorMenuFlag = false;
      setSpeedMenuFlag = false;
      setDistMenuFlag = true;
      setBrightnessMenuFlag = false;
      setKFilterMenuFlag = false;
      break;
    case 12:
      leftTurnMenuFlag = false;
      rightTurnMenuFlag = false;
      circleTurnMenuFlag = false;
      forwardMenuFlag = false;
      line1MenuFlag = false;
      line2MenuFlag = false;
      line3MenuFlag = false;
      wallMenuFlag = false;
      bluetoothMenuFlag = false;
      setLineSensorMenuFlag = false;
      setSpeedMenuFlag = false;
      setDistMenuFlag = false;
      setBrightnessMenuFlag = true;
      setKFilterMenuFlag = false;
      break;

    case 13:
      leftTurnMenuFlag = false;
      rightTurnMenuFlag = false;
      circleTurnMenuFlag = false;
      forwardMenuFlag = false;
      line1MenuFlag = false;
      line2MenuFlag = false;
      line3MenuFlag = false;
      wallMenuFlag = false;
      bluetoothMenuFlag = false;
      setLineSensorMenuFlag = false;
      setSpeedMenuFlag = false;
      setDistMenuFlag = false;
      setBrightnessMenuFlag = false;
      setKFilterMenuFlag = true;
      break;
  }
}

