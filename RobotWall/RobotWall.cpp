#include "RobotWall.h"
Adafruit_SSD1306 lcd(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

int run_speed = 150;
double distance1, duration1;
double distance2, duration2;
double distance3, duration3;

double P = 0;
double D = 0;
double I = 0;
double error = 0;
double lastError = 0;
double sumOut = 0;

byte Kp = 0;
byte Ki = 0;
byte Kd = 0;

bool robotRunning = false;
bool follow_left = false;

double lastProcess = 0;
dataSetting setting;

double readUltrasonic1() {
  digitalWrite(pin_trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(pin_trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(pin_trigger, LOW);
  duration1 = pulseIn(pin_echo_L, HIGH);
  // Calculating the distance
  distance1 = duration1 * 0.034 / 2;
  delay(12);
}
double readUltrasonic2() {
  digitalWrite(pin_trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(pin_trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(pin_trigger, LOW);
  duration2 = pulseIn(pin_echo_C, HIGH);
  // Calculating the distance
  distance2 = duration2 * 0.034 / 2;
  delay(12);
}
double readUltrasonic3() {
  digitalWrite(pin_trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(pin_trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(pin_trigger, LOW);
  duration3 = pulseIn(pin_echo_R, HIGH);
  // Calculating the distance
  distance3 = duration3 * 0.034 / 2;
  delay(12);
}


void Wallsetup() {
  begin();
}

void Wallloop() {
  if (robotRunning) {
    RobotLoop();
  } else {
  	 setMotor(0,0);
     navigateMenu();
  };
}

void saveSetting() {
  EEPROM.put(0, setting);
}

void loadSetting() {
  EEPROM.get(0, setting);
  run_speed = setting.speed;
  Kp = setting.kp;
  Ki = setting.ki;
  Kd = setting.kd;
}

void setPID(byte p, byte i, byte d) {
  Kp = p;
  Ki = i;
  Kd = d;
  setting.kp = p;
  setting.ki = i;
  setting.kd = d;
  saveSetting();
}

void begin() {
  Serial.begin(9600);

  // Pin setup
  pinMode(pin_button_OK, INPUT_PULLUP);
  pinMode(pin_button_DOWNL, INPUT_PULLUP);
  pinMode(pin_button_UPR, INPUT_PULLUP);
  pinMode(pin_button_DOWNR, INPUT_PULLUP);

  pinMode(pin_dir_motor_L, OUTPUT);
  pinMode(pin_pwm_motor_L, OUTPUT);
  pinMode(pin_dir_motor_R, OUTPUT);
  pinMode(pin_pwm_motor_R, OUTPUT);

  pinMode(pin_trigger, OUTPUT);
  pinMode(pin_echo_L, INPUT);
  pinMode(pin_echo_C, INPUT);
  pinMode(pin_echo_R, INPUT);

  // OLED setup
  if (!lcd.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }
  lcd.clearDisplay();
  lcd.setTextSize(1);
  lcd.setTextColor(SSD1306_WHITE);
  lcd.setCursor(34, 15);
  lcd.print(F("HELLO WORLD"));
  lcd.setCursor(0, 30);
  lcd.print(F("ROBOTOO WALL FOLLOWER"));
  lcd.setCursor(25, 45);
  lcd.print(F("WWW.KEBAKAR.ID"));
  lcd.display();
  delay(600);
  lcd.setTextSize(1);
  lcd.clearDisplay();
  lcd.setCursor(0, 0);

  loadSetting();
}

void navigateMenu() {
  static int menuIndex = 0;
  if (!button_UPR) {
    menuIndex++;
    delay(200);
  }
  if (!button_DOWNL) {
    menuIndex--;
    delay(200);
  }
  if (menuIndex < 0) {
    menuIndex = 4;
  }
  if (menuIndex > 4) {
    menuIndex = 0;
  }

  if (!button_OK) {
    delay(200);
    switch (menuIndex) {
    case 0:
      run_speed = setSpeed();
      break;
    case 1:
      robotRunning = true; // Start running the robot
      break;
    case 2:
      setPIDParameters();
      break;
    case 3:
      displaySensor();
      break;
    case 4:
      choice();
    }
  }

  lcd.clearDisplay();
  lcd.setCursor(0, 0);
  lcd.print("Menu:");
  lcd.setCursor(0, 8);
  lcd.print(menuIndex == 0 ? "> Set Speed" : "  Set Speed");
  lcd.setCursor(0, 16);
  lcd.print(menuIndex == 1 ? "> Run Robot" : "  Run Robot");
  lcd.setCursor(0, 24);
  lcd.print(menuIndex == 2 ? "> Set PID" : "  Set PID");
  lcd.setCursor(0, 32);
  lcd.print(menuIndex == 3 ? "> Display Sensor" : "  Display Sensor");
  lcd.setCursor(0, 40);
  lcd.print(menuIndex == 4 ? "> Select Mode" : "  Select Mode");
  lcd.display();
}

void choice(){
  while(true){
  lcd.clearDisplay();
  lcd.setCursor(0, 0);
  lcd.print("Select Mode:");
  if (follow_left == true){
    lcd.setCursor(0, 10);
    lcd.print("Follow Left");
  }
  if (follow_left == false){
    lcd.setCursor(0, 10);
    lcd.print("Follow Right");
  }
  if (!button_UPR){
    follow_left = true;
    delay(200);
  }
  if (!button_DOWNL){
    follow_left = false;
    delay(200);
  }
  lcd.display();
  if (!button_OK) {
    delay(200);
    return;
    }}}

int setSpeed() {
  int speed = run_speed;
  while (true) {
    lcd.clearDisplay();
    lcd.setCursor(0, 0);
    lcd.print("Set Speed:");
    lcd.setCursor(0, 10);
    lcd.print("Speed: ");
    lcd.print(speed);
    lcd.display();
    if (!button_UPR) {
      speed += 10;
      if (speed > 255) {
        speed = 255;
      }
      delay(200);
    }
    if (!button_DOWNL) {
      speed -= 10;
      if (speed < 0) {
        speed = 0;
      }
      delay(200);
    }
    if (!button_OK) {
      delay(200);
      setting.speed = speed;
      saveSetting();
      return speed;
    }
  }
}

void setPIDParameters() {
  while (true) {
    lcd.clearDisplay();
    lcd.setCursor(0, 0);
    lcd.print("Set PID:");
    lcd.setCursor(0, 10);
    lcd.print("Kp: ");
    lcd.print(Kp);
    lcd.setCursor(0, 20);
    lcd.print("Ki: ");
    lcd.print(Ki);
    lcd.setCursor(0, 30);
    lcd.print("Kd: ");
    lcd.print(Kd);
    lcd.display();
    if (!button_UPR) {
      Kp++;
      delay(200);
    }
    if (!button_DOWNL) {
      Kp--;
      delay(200);
    }
    if (!button_DOWNR) {
      Kd++;
      if(Kd == 5){
        Kd = 0;
      }
      delay(200);
    }
    if (!button_OK) {
      delay(200);
      setPID(Kp, Ki, Kd);
      setting.kp = Kp;
      setting.ki = Ki;
      setting.kd = Kd;
      saveSetting();
      return;
    }
  }
}

void setMotor(int LL, int RR) {
  if (RR < 0) {
    digitalWrite(pin_dir_motor_R, 1);
    analogWrite(pin_pwm_motor_R, -RR);
  } else {
    digitalWrite(pin_dir_motor_R, 0);
    analogWrite(pin_pwm_motor_R, RR);
  }
  if (LL < 0) {
    digitalWrite(pin_dir_motor_L, 1);
    analogWrite(pin_pwm_motor_L, -LL);
  } else {
    digitalWrite(pin_dir_motor_L, 0);
    analogWrite(pin_pwm_motor_L, LL);
  }
}
void followWall(double targetDistance, const int threshold_distance, const int turn_speed, const int turn_duration) {
  readUltrasonic1();
  readUltrasonic2();
  readUltrasonic3();
  double distance_L = distance1;
  double distance_C = distance2;
  double distance_R = distance3;

  // Check if there is a wall in front of the robot
  if (distance_C < threshold_distance) {
    // Perform a turn
    if (follow_left == true) {
      setMotor(turn_speed, 0); // Turn right
    } else {
      setMotor(0, turn_speed); // Turn left
    }
    delay(turn_duration); // Adjust the turn duration based on your requirements
  } else {
    // Continue wall following
    double error;
    if (follow_left == true) {
      error = targetDistance - distance_L; // Adjusted for wall following on the left
    } else {
      error = targetDistance - distance_R; // Adjusted for wall following on the right
    }
    double currentTime = millis();
    double deltaTime = (currentTime - lastProcess) / 1000.0;

    double P = error * Kp;
    double D = ((error - lastError) * Kd) / deltaTime;
    sumOut += error * deltaTime;
    double I = sumOut * Ki;

    double PIDValue = P + I + D;

    double leftSpeed, rightSpeed;
    if (follow_left == true) {
      leftSpeed = run_speed + PIDValue;
      rightSpeed = run_speed - PIDValue;
    } else {
      leftSpeed = run_speed - PIDValue;
      rightSpeed = run_speed + PIDValue;
    }

    if (leftSpeed > 255) leftSpeed = 255;
    if (leftSpeed < -255) leftSpeed = -255;
    if (rightSpeed > 255) rightSpeed = 255;
    if (rightSpeed < -255) rightSpeed = -255;

    setMotor(leftSpeed, rightSpeed);

    lastError = error;
    lastProcess = currentTime;
  }
}

void RobotLoop() {
  lcd.clearDisplay();
  lcd.setCursor(0, 0);
  lcd.print(F("ROBOTOO WALL FOLLOWER"));
  lcd.setCursor(12, 12);
  lcd.print("L: ");
  lcd.print(distance1);
  lcd.print("; C: ");
  lcd.print(distance2);
  lcd.setCursor(40, 21);
  lcd.print("R: ");
  lcd.print(distance3);
  lcd.setCursor(25, 33);
  lcd.print(F("+++RUNNING+++"));
  lcd.setCursor(0, 45);
  lcd.print(F("~~~~~~~~~~~~~~~~~~~~~"));
  lcd.setCursor(25, 55);
  lcd.print(F("KELOMPOK SATU"));
  lcd.display();
  
  loadSetting();
  followWall(8.0, 10, 50, 150);

  if (!button_OK) {
    robotRunning = false; // Stop running the robot
    setMotor(0, 0);
    delay(200);
  }
}

void displaySensor() {
  while (true) {
    readUltrasonic1();
    readUltrasonic2();
    readUltrasonic3();
    lcd.clearDisplay();
    lcd.setCursor(0, 0);
    lcd.print("Sensor Readings:");
    lcd.setCursor(0, 10);
    lcd.print("Left: ");
    lcd.print(distance1);
    lcd.print(" cm");
    lcd.setCursor(0, 20);
    lcd.print("Center: ");
    lcd.print(distance2);
    lcd.print(" cm");
    lcd.setCursor(0, 30);
    lcd.print("Right: ");
    lcd.print(distance3);
    lcd.print(" cm");
    lcd.display();

    if (!button_OK) {
      delay(200);
      return;
    }
  }
}
