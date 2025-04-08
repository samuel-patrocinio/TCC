#include <Wire.h>
#include <EEPROM.h>
#include "BluetoothSerial.h"
#include "Arduino.h"

#define BUZZER      2
#define VBAT        2
#define INT_LED     2

#define BRAKE       14

#define PWM1        27
#define DIR1        16
#define PWM1_CH     0

#define PWM2        19
#define DIR2        23
#define PWM2_CH     1

#define PWM3        33
#define DIR3        32
#define PWM3_CH     2

#define TIMER_BIT  8
#define BASE_FREQ  20000

#define ACCEL_CONFIG 0x1C      // Accelerometer configuration address
#define GYRO_CONFIG 0x1B       // Gyro configuration address
#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C

//Sensor output scaling
#define accSens 0              // 0 = 2g, 1 = 4g, 2 = 8g, 3 = 16g
#define gyroSens 1             // 0 = 250rad/s, 1 = 500rad/s, 2 1000rad/s, 3 = 2000rad/s

#define EEPROM_SIZE 32

float Gyro_amount = 0.1;    

bool vertical = false;
bool calibrating = false;
bool calibrated = false;

float K1 = 10.0;
float K2 = 0.0;
float K3 = 0.0;
int loop_time = 10;

struct OffsetsObj {
  int ID;
  float X;
  float Y;
};

OffsetsObj offsets;

int16_t  AcX, AcY, AcZ, GyY, gyroY, GyX, gyroX;

int16_t  AcX_offset = -940; 
int16_t  AcY_offset = -200;      
int16_t  AcZ_offset = 1800;
int16_t  GyX_offset = 0;
int16_t  GyY_offset = 0;
int32_t  GyX_offset_sum = 0;
int32_t  GyY_offset_sum = 0;

float robot_angleX, robot_angleY, angleX, angleY;
float Acc_angleX, Acc_angleY;      
int32_t motor_speed_X; 
int32_t motor_speed_Y;   

long currentT, previousT_1, previousT_2 = 0;

const uint8_t MPU6050 = 0x68;

BluetoothSerial SerialBT;

void writeTo(byte device, byte address, byte value) {
    Wire.beginTransmission(device);
    Wire.write(address);
    Wire.write(value);
    Wire.endTransmission(true);
  }

  void angle_calc() {
  
    Wire.beginTransmission(MPU6050);
    Wire.write(0x3B);                       // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050, (uint8_t)6);  
    AcX = Wire.read() << 8 | Wire.read();   // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    AcY = Wire.read() << 8 | Wire.read();   // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ = Wire.read() << 8 | Wire.read();   // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    
    Wire.beginTransmission(MPU6050);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050, (uint8_t)4);  
    GyX = Wire.read() << 8 | Wire.read();   // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY = Wire.read() << 8 | Wire.read();   // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  
    AcX += AcX_offset;
    AcY += AcY_offset;
    AcZ += AcZ_offset;
    GyX -= GyX_offset;
    GyY -= GyY_offset;
  
    robot_angleY += GyY * loop_time / 1000 / 65.536;  
    Acc_angleY = -atan2(AcX, AcZ) * 57.2958;      // angle from acc. values * 57.2958 (deg/rad)
    robot_angleY = robot_angleY * Gyro_amount + Acc_angleY * (1.0 - Gyro_amount);
  
    robot_angleX += GyX * loop_time / 1000 / 65.536;  
    Acc_angleX = atan2(AcY, AcZ) * 57.2958;      // angle from acc. values * 57.2958 (deg/rad)
    robot_angleX = robot_angleX * Gyro_amount + Acc_angleX * (1.0 - Gyro_amount);
  
    angleX = robot_angleX - offsets.X;
    angleY = robot_angleY - offsets.Y;
    
    //SerialBT.print(angleX); SerialBT.print(" "); SerialBT.println(angleY); // plot
    SerialBT.print("AngleX: "); SerialBT.print(angleX); SerialBT.print(" AngleY: "); SerialBT.println(angleY);
    if (abs(angleX) > 30 || abs(angleY) > 30) vertical = false;
    if (abs(angleX) < 20 && abs(angleY) < 20) vertical = true;
  }
  
  void angle_setup() {
    Wire.begin();
    delay(100);
    writeTo(MPU6050, PWR_MGMT_1, 0);
    writeTo(MPU6050, ACCEL_CONFIG, accSens << 3); // Specifying output scaling of accelerometer
    writeTo(MPU6050, GYRO_CONFIG, gyroSens << 3); // Specifying output scaling of gyroscope
    delay(100);
    Serial.println("Calibrating gyros...");
  
    for (int i = 0; i < 1024; i++) {
      angle_calc();
      GyX_offset_sum += GyX;
      GyY_offset_sum += GyY;
      delay(3);
    }
    GyX_offset = GyX_offset_sum >> 10;
    GyY_offset = GyY_offset_sum >> 10;
    Serial.print("GyX offset value = "); Serial.println(GyX_offset);
    Serial.print("GyY offset value = "); Serial.println(GyY_offset);
  
    digitalWrite(BUZZER, HIGH);
    delay(70);
    digitalWrite(BUZZER, LOW);
    delay(80);
    digitalWrite(BUZZER, HIGH);
    delay(70);
    digitalWrite(BUZZER, LOW);
  }

  void pwmSet(uint8_t channel, uint32_t value) {
    ledcWrite(channel, value);
  }

  void Motor1_control(int sp) {
    if (sp < 0) {
      digitalWrite(DIR1, LOW);
      sp = -sp;
    } else {
      digitalWrite(DIR1, HIGH);
    }
    pwmSet(PWM1_CH, sp > 255 ? 255 : 255 - sp);
  }
  
  void Motor2_control(int sp) {
    if (sp < 0) {
      digitalWrite(DIR2, LOW);
      sp = -sp;
    } else {
      digitalWrite(DIR2, HIGH);
    }
    pwmSet(PWM2_CH, sp > 255 ? 255 : 255 - sp);
  }
  
  void Motor3_control(int sp) {
    if (sp < 0) {
      digitalWrite(DIR3, LOW);
      sp = -sp;
    } else {
      digitalWrite(DIR3, HIGH);
    }
    pwmSet(PWM3_CH, sp > 255 ? 255 : 255 - sp);
  }
  
  void XY_to_threeWay(float pwm_X, float pwm_Y) {   
    int16_t m1 = round(0.5 * pwm_X - 0.75 * pwm_Y); 
    int16_t m2 = round(0.5 * pwm_X + 0.75 * pwm_Y);
    int16_t m3 = -pwm_X;
    Motor1_control(m1);
    Motor2_control(m2);
    Motor3_control(m3);
  }
  
  void battVoltage(double voltage) {
    // Serial.print("batt: "); Serial.println(voltage);
    if (voltage > 8 && voltage <= 9.5) {
      digitalWrite(BUZZER, HIGH);
    } else {
      digitalWrite(BUZZER, LOW);
    }
  }

  void printValues() {
    SerialBT.print("K1: "); SerialBT.print(K1);
    SerialBT.print(" K2: "); SerialBT.print(K2);
    SerialBT.print(" K3: "); SerialBT.println(K3,4);
  }
  
  int Tuning() {
    if (!SerialBT.available()) return 0;
    //delay(2);
    char param = SerialBT.read();               // get parameter byte
    if (!SerialBT.available()) return 0;
    char cmd = SerialBT.read();                 // get command byte
    //SerialBT.flush();
    switch (param) {
      
      case 'p':
        if (cmd == '+')    K1 += 1;
        if (cmd == '-')    K1 -= 1;
        printValues();
        break;
        case 'i':
        if (cmd == '+')    K2 += 0.05;
        if (cmd == '-')    K2 -= 0.05;
        printValues();
        break;  
      case 's':
        if (cmd == '+')    K3 += 0.005;
        if (cmd == '-')    K3 -= 0.005;
        printValues();
        break;  
      case 'c':
        if (cmd == '+' && !calibrating) {
          calibrating = true;
           SerialBT.println("calibrating on");
        }
        if (cmd == '-' && calibrating)  {
          SerialBT.print("X: "); SerialBT.print(robot_angleX); SerialBT.print(" Y: "); SerialBT.println(robot_angleY);
          if (abs(robot_angleX) < 15 && abs(robot_angleY) < 15) {
            offsets.X = robot_angleX;
            offsets.Y = robot_angleY;
            offsets.ID = 34;
            EEPROM.put(0, offsets);
            EEPROM.commit();
            calibrated = true;
            calibrating = false;
            SerialBT.println("calibrating off");
            digitalWrite(BUZZER, HIGH);
            delay(70);
            digitalWrite(BUZZER, LOW);
          } else {
            SerialBT.println("The angles are wrong!!!");
            digitalWrite(BUZZER, HIGH);
            delay(50);
            digitalWrite(BUZZER, LOW);
            delay(70);
            digitalWrite(BUZZER, HIGH);
            delay(50);
            digitalWrite(BUZZER, LOW);
          }
        }
        break;        
     }
     return 1;
  }

void setup() {
  Serial.begin(115200);
  SerialBT.begin("3-Wheel-Robot"); // Bluetooth device name
  EEPROM.begin(EEPROM_SIZE);
  
  pinMode(BUZZER, OUTPUT);
  pinMode(BRAKE, OUTPUT);
  digitalWrite(BRAKE, HIGH);
  
  pinMode(DIR1, OUTPUT);
  ledcSetup(PWM1_CH, BASE_FREQ, TIMER_BIT);
  ledcAttachPin(PWM1, PWM1_CH);
  Motor1_control(0);
  
  pinMode(DIR2, OUTPUT);
  ledcSetup(PWM2_CH, BASE_FREQ, TIMER_BIT);
  ledcAttachPin(PWM2, PWM2_CH);
  Motor2_control(0);
  
  pinMode(DIR3, OUTPUT);
  ledcSetup(PWM3_CH, BASE_FREQ, TIMER_BIT);
  ledcAttachPin(PWM3, PWM3_CH);
  Motor3_control(0);
  EEPROM.get(0, offsets);
  if (offsets.ID == 34) calibrated = true;
    else calibrated = false;
  delay(3000);
  digitalWrite(BUZZER, HIGH);
  delay(70);
  digitalWrite(BUZZER, LOW);
  angle_setup();
}

void loop() {

  SerialBT.print("vertical:");
  SerialBT.println(vertical);

  currentT = millis();

  if (currentT - previousT_1 >= loop_time) {
    Tuning(); 
    angle_calc();
    if (abs(robot_angleX) < 8 || abs(robot_angleY) < 8) {  
      Gyro_amount = 0.996; 
    } else 
      Gyro_amount = 0.1;
    
	if (vertical && calibrated && !calibrating) {
      digitalWrite(BRAKE, HIGH);
      gyroX = GyX / 131.0; // Convert to deg/s
      gyroY = GyY / 131.0; // Convert to deg/s
      
      int pwm_X = constrain(K1 * angleX + K2 * gyroX + K3 * motor_speed_X, -255, 255);
      int pwm_Y = constrain(K1 * angleY + K2 * gyroY + K3 * motor_speed_Y, -255, 255);
      motor_speed_X += pwm_X; 
      motor_speed_Y += pwm_Y;
      XY_to_threeWay(-pwm_X, pwm_Y);
    } else {
      XY_to_threeWay(0, 0);
      digitalWrite(BRAKE, LOW);
      motor_speed_X = 0;
      motor_speed_Y = 0;
    }
    previousT_1 = currentT;
  }
  
  if (currentT - previousT_2 >= 2000) {    
    battVoltage((double)analogRead(VBAT) / 300); // need to measure
    if (!calibrated && !calibrating) {
      SerialBT.println("first you need to calibrate the balancing point...");
    }
    previousT_2 = currentT;
  }
}