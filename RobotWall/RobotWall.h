#ifndef RobotWall_H
#define RobotWall_H

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <EEPROM.h>

// OLED configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SCREEN_ADDRESS 0x3C
#define OLED_RESET -1

// Motor pin definitions
#define pin_dir_motor_L 9
#define pin_pwm_motor_L 5
#define pin_dir_motor_R 10
#define pin_pwm_motor_R 6

// Button pin definitions
#define pin_button_OK A0
#define pin_button_DOWNL 8
#define pin_button_UPR 11
#define pin_button_DOWNR 12

#define button_OK digitalRead(pin_button_OK)
#define button_DOWNL digitalRead(pin_button_DOWNL)
#define button_UPR digitalRead(pin_button_UPR)
#define button_DOWNR digitalRead(pin_button_DOWNR)

// Ultrasonic sensor pin definitions
#define pin_trigger 7
#define pin_echo_L 2
#define pin_echo_C 3
#define pin_echo_R 4

extern double distance1, duration1;
extern double distance2, duration2;
extern double distance3, duration3;

extern byte Kp, Ki, Kd;
extern bool robotRunning;
extern bool follow_left;

// PID control parameters
struct dataSetting {
  int speed;
  byte kp;
  byte ki;
  byte kd;
};

// Robot control parameters

// PID variables
extern double P;
extern double D;
extern double I;
extern double error;
extern double lastError;
extern double sumOut;
extern double lastProcess;
extern int run_speed;

int setSpeed();
void Wallsetup();
void Wallloop();
void saveSetting();
void loadSetting();
void setPID(byte p, byte i, byte d);
void begin();
void navigateMenu();
void choice();
void setPIDParameters();
void setMotor(int LL, int RR);
void followWall(double targetDistance, const int threshold_distance, const int turn_speed, const int turn_duration);
void RobotLoop();
void displaySensor();
#endif
