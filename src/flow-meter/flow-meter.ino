#include <MPU9250.h>
#include <SPI.h>
#include <SD.h>
#include <RTClib.h>
#include <avr/sleep.h>

MPU9250 IMU(Wire,0x69); // Because AD0 = 1
RTC_DS3231 rtc;

int status;
unsigned int ReadCycle = 1;
const int alarmPin = 2; // The number of the pin for monitoring alarm status on DS3231
const int SleepMin = 5;
const int SleepSec = 0; 

void setup() {
  pinMode(LED_BUILTIN, OUTPUT); //Status Led
  pinMode(alarmPin, INPUT_PULLUP); // Set alarm pin as pullup
  
  SD.begin(9);
  
  status = IMU.begin(); // Start communication with MPU9250
  if (status < 0) {
    digitalWrite(LED_BUILTIN, HIGH);
    while(1) {} // Interrupt program execution
  }

  SetMPU9250MagCal();
  
  if (!rtc.begin()) {
    digitalWrite(LED_BUILTIN, HIGH);
    while(1) {}
  }
  
  // Disable and clear both alarms
  rtc.disableAlarm(1);
  rtc.disableAlarm(2);
  rtc.clearAlarm(1);
  rtc.clearAlarm(2);

  rtc.writeSqwPinMode(DS3231_OFF); // Place SQW pin into alarm interrupt mode
}

void loop() {
  IMU.readSensor(); // Read the sensor
  
  // Get current time and set alarm to a time to wake
  DateTime now = rtc.now();  // Get current time
  rtc.setAlarm1(now + TimeSpan(0, 0, SleepMin, SleepSec), DS3231_A1_Minute); // In 5 minutes

  WriteSensorData(now);
  ReadCycle++;
    
  EnterSleep();  // Go to sleep
}

void SetMPU9250MagCal() {
  IMU.setMagCalX(4.461882,0.977149);
  IMU.setMagCalY(-12.536654,0.997031);
  IMU.setMagCalZ(-29.446018,1.027077);
}

void WriteSensorData(DateTime now) {
  File IMUData = SD.open("rawdata.txt", FILE_WRITE); // Open file for writing
      
  // Save sensor data
  IMUData.print(ReadCycle);
  IMUData.print(",");
  IMUData.print(now.year());
  IMUData.print("/");
  IMUData.print(now.month());
  IMUData.print("/");
  IMUData.print(now.day());
  IMUData.print(" ");
  IMUData.print(now.hour());
  IMUData.print(":");
  IMUData.print(now.minute());
  IMUData.print(":");
  IMUData.print(now.second());
  IMUData.print(",");
  IMUData.print(IMU.getAccelX_mss(),6);
  IMUData.print(",");
  IMUData.print(IMU.getAccelY_mss(),6);
  IMUData.print(",");
  IMUData.print(IMU.getAccelZ_mss(),6);
  IMUData.print(",");
  IMUData.print(IMU.getGyroX_rads(),6);
  IMUData.print(",");
  IMUData.print(IMU.getGyroY_rads(),6);
  IMUData.print(",");
  IMUData.print(IMU.getGyroZ_rads(),6);
  IMUData.print(",");
  IMUData.print(IMU.getMagX_uT(),6);
  IMUData.print(",");
  IMUData.print(IMU.getMagY_uT(),6);
  IMUData.print(",");
  IMUData.print(IMU.getMagZ_uT(),6);
  IMUData.print(",");
  IMUData.println(IMU.getTemperature_C(),6);
  IMUData.close();

  IMUData = SD.open("data.txt", FILE_WRITE); // Open file for writing
  IMUData.print(ReadCycle);
  IMUData.print(",");
  IMUData.print(TiltCalculation(),6);
  IMUData.print(",");
  IMUData.println(HeadingCalculation(),6);
  IMUData.close();
}

float TiltCalculation() {
  float AccelX = IMU.getAccelY_mss();
  float AccelY = IMU.getAccelX_mss();
  float AccelZ = IMU.getAccelZ_mss();
  float G;
  float ZX;
  float Tilt;
  
  G = sqrt(pow(AccelX,2)+pow(AccelY,2)+pow(AccelZ,2));
  ZX = sqrt(pow(AccelX,2)+pow(AccelZ,2));
  Tilt = acos(ZX/G) * (180/M_PI);
  return Tilt;
}

float HeadingCalculation() {
    float G, ax, ay, az, H, hx, hy, hz, pitch, roll, yaw, Heading;

    ax = IMU.getAccelX_mss();
    ay = IMU.getAccelY_mss();
    az = IMU.getAccelZ_mss();
    hx = IMU.getMagX_uT();
    hy = IMU.getMagY_uT();
    hz = IMU.getMagZ_uT();
    
    /* Normalize accelerometer and magnetometer data */
    G = sqrtf(ax * ax + ay * ay + az * az);
    ax /= G;
    ay /= G;
    az /= G;
    H = sqrtf(hx * hx + hy * hy + hz * hz);
    hx /= H;
    hy /= H;
    hz /= H;

    pitch = asinf(ax);
    roll = asinf(-ay / cosf(pitch));
    yaw = atan2f(hz * sinf(roll) - hy * cosf(roll), hx * cosf(pitch) + hy * sinf(pitch) * sinf(roll) + hz * sinf(pitch) * cosf(roll));
    Heading = AngleCalculation(yaw) * (180/PI);
    return Heading;
}

float AngleCalculation(float yaw) {
  yaw = fmod(yaw, 2.0 * PI);
  if (yaw < 0.0)
    yaw += 2.0 * PI;
  return yaw;
}

void EnterSleep() {
  sleep_enable();                       // Enabling sleep mode
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  // Setting the sleep mode, in this case full sleep
  
  noInterrupts();                       // Disable interrupts
  attachInterrupt(digitalPinToInterrupt(alarmPin), Alarm_ISR, LOW);
    
  interrupts();                         // Allow interrupts again
  sleep_cpu();                          // Enter sleep mode

  /* The program will continue from here when it wakes */
  
  // Disable and clear alarm
  rtc.disableAlarm(1);
  rtc.clearAlarm(1);
 }

void Alarm_ISR() {
  // This runs when SQW pin is low. It will wake up the µController
  
  sleep_disable(); // Disable sleep mode
  detachInterrupt(digitalPinToInterrupt(alarmPin)); // Detach the interrupt to stop it firing
}
