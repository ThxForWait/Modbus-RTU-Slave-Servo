#include <Servo.h>
#include "ModbusSlave.h"

// Конфигурация
#define SERVO_PIN 9
#define SLAVE_ID 1
#define BAUD_RATE 9600

// Карта регистров
enum {
  REG_TARGET_ANGLE = 0, // Целевой угол (запись)
  REG_SPEED_DELAY,      // Задержка в мс между шагами (чем больше, тем медленнее)
  REG_CURRENT_ANGLE,    // Текущий угол (чтение)
  REG_COUNT             // Общее количество
};

uint16_t mbRegisters[REG_COUNT];

Servo myServo;
ModbusSlave modbus(Serial, SLAVE_ID);

// Переменные для плавного движения
unsigned long lastMoveTime = 0;
int currentServoAngle = 90;

void setup() {
  Serial.begin(BAUD_RATE);
  myServo.attach(SERVO_PIN);
  
  // Начальные значения
  mbRegisters[REG_TARGET_ANGLE] = 90;
  mbRegisters[REG_SPEED_DELAY] = 15; // Средняя скорость по умолчанию
  mbRegisters[REG_CURRENT_ANGLE] = 90;
  
  myServo.write(currentServoAngle);
  
  modbus.setRegisters(mbRegisters, REG_COUNT);
}

void loop() {
  //  Modbus
  modbus.poll();

  // управление сервоприводом
  updateServo();
}

void updateServo() {
  int target = mbRegisters[REG_TARGET_ANGLE];
  int speedDelay = mbRegisters[REG_SPEED_DELAY];

  // Ограничение углов
  if (target > 180) target = 180;
  if (target < 0) target = 0;
  mbRegisters[REG_TARGET_ANGLE] = target; 

  if (currentServoAngle == target) {
    mbRegisters[REG_CURRENT_ANGLE] = currentServoAngle;
    return;
  }

  if (speedDelay == 0) {
    currentServoAngle = target;
    myServo.write(currentServoAngle);
    mbRegisters[REG_CURRENT_ANGLE] = currentServoAngle;
    return;
  }

  // Плавное движение
  if (millis() - lastMoveTime >= speedDelay) {
    lastMoveTime = millis();
    
    if (currentServoAngle < target) {
      currentServoAngle++;
    } else {
      currentServoAngle--;
    }
    
    myServo.write(currentServoAngle);
    
    // Обновляем статус 
    mbRegisters[REG_CURRENT_ANGLE] = currentServoAngle;
  }
}