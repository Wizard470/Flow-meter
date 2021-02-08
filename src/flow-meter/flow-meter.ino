#include <MPU9250.h>
#include <SPI.h>
#include <SD.h>
#include <DS3232RTC.h>
#include <avr/sleep.h>

MPU9250 IMU(Wire,0x69); // Because AD0 = 1

int status;
unsigned int ReadCycle = 1;
const int alarmPin = 2; // The number of the pin for monitoring alarm status on DS3231
time_t MEASUREMENT_INTERVAL = 300; // Measurement cycle in seconds
int statusLedok = 5;
int statusLedng = 4;

void setup() {
  pinMode(statusLedok, OUTPUT); //Status Led
  pinMode(statusLedng, OUTPUT); //Status Led
  pinMode(alarmPin, INPUT_PULLUP); // Set alarm pin as pullup
  
  if (!SD.begin(9)) {
    while(1) {
      digitalWrite(statusLedng, HIGH);
      delay(800);
      digitalWrite(statusLedng, LOW);
      delay(200);
    } // Interrupt program execution
  }
  
  status = IMU.begin(); // Start communication with MPU9250
  if (status < 0) {
    while(1) {
      digitalWrite(statusLedng, HIGH);
      delay(200);
      digitalWrite(statusLedng, LOW);
      delay(800);
    } // Interrupt program execution
  }

  SetMPU9250MagCal();
  
    
  // Disable and clear both alarms
  RTC.setAlarm(ALM1_MATCH_DATE, 0, 0, 0, 1);
  RTC.setAlarm(ALM2_MATCH_DATE, 0, 0, 0, 1);
  RTC.alarm(ALARM_1);
  RTC.alarm(ALARM_2);
  RTC.alarmInterrupt(ALARM_1, true);
  RTC.alarmInterrupt(ALARM_2, false);
  RTC.squareWave(SQWAVE_NONE);
  
  digitalWrite(statusLedok, HIGH);
  delay(1000);
  digitalWrite(statusLedok, LOW);
}

void loop() {
  IMU.readSensor(); // Read the sensor

  // Get current time and set alarm to a time to wake
  time_t t = RTC.get();
  time_t alarmTime = t + MEASUREMENT_INTERVAL;
  RTC.setAlarm(ALM1_MATCH_HOURS, second(alarmTime), minute(alarmTime), hour(alarmTime), 0); 
  RTC.alarm(ALARM_1);
    
  WriteSensorData(t);
  ReadCycle++;
    
  EnterSleep();  // Go to sleep
}

void SetMPU9250MagCal() {
  IMU.setMagCalX(4.461882,0.977149);
  IMU.setMagCalY(-12.536654,0.997031);
  IMU.setMagCalZ(-29.446018,1.027077);
}

void WriteSensorData(time_t t) {
  File IMUData = SD.open("rawdata.txt", FILE_WRITE); // Open file for writing
      
  // Save sensor data
  IMUData.print(ReadCycle);
  IMUData.print(",");
  IMUData.print(year(t));
  IMUData.print("/");
  IMUData.print(month(t));
  IMUData.print("/");
  IMUData.print(day(t));
  IMUData.print(" ");
  IMUData.print(hour(t));
  IMUData.print(":");
  IMUData.print(minute(t));
  IMUData.print(":");
  IMUData.print(second(t));
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
  IMUData.print(year(t));
  IMUData.print("/");
  IMUData.print(month(t));
  IMUData.print("/");
  IMUData.print(day(t));
  IMUData.print(" ");
  IMUData.print(hour(t));
  IMUData.print(":");
  IMUData.print(minute(t));
  IMUData.print(":");
  IMUData.print(second(t));
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
  
  RTC.alarm(ALARM_1); // clear the alarm flag
 }

void Alarm_ISR() {
  // This runs when SQW pin is low. It will wake up the µController
  
  sleep_disable(); // Disable sleep mode
  detachInterrupt(digitalPinToInterrupt(alarmPin)); // Detach the interrupt to stop it firing
}
