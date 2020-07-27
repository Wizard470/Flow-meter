#include <MPU9250.h>
#include <SPI.h>
#include <SD.h>
#include <RTClib.h>
#include <avr/sleep.h>

MPU9250 IMU(Wire,0x69); // Because AD0 = 1 
int status;
File myFile;
int ReadCycle = 1;
const int alarmPin = 2; // The number of the pin for monitoring alarm status on DS3231
RTC_DS3231 rtc;

void setup() {
  pinMode(alarmPin, INPUT_PULLUP); // Set alarm pin as pullup
  
  SD.begin(9);
  myFile = SD.open("sensor.txt", FILE_WRITE); // Open file for writing
  
  status = IMU.begin(); // Start communication with MPU9250
  if (status < 0) {
    myFile.println("IMU initialization unsuccessful");
    myFile.print("Status: ");
    myFile.println(status);
    myFile.close();
    while(1) {} // Interrupt program execution
  }

  // Setting the accelerometer full scale range to +/-4G 
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_4G);
  // Setting the gyroscope full scale range to +/-250 deg/s
  IMU.setGyroRange(MPU9250::GYRO_RANGE_250DPS);
  // Setting DLPF bandwidth to 5 Hz
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_5HZ);
  // Setting SRD to 49 for a 20 Hz update rate
  IMU.setSrd(49);

  if (!rtc.begin()) {
    myFile.println("Couldn't find RTC");
    myFile.close();
    while(1) {}
  }
  
  // Disable and clear both alarms
  rtc.disableAlarm(1);
  rtc.disableAlarm(2);
  rtc.clearAlarm(1);
  rtc.clearAlarm(2);

  rtc.writeSqwPinMode(DS3231_OFF); // Place SQW pin into alarm interrupt mode
  
  myFile.close();
}

void loop() {
  // Get current time and set alarm to a time to wake
  DateTime now = rtc.now();  // Get current time
  rtc.setAlarm1(now + TimeSpan(0, 0, 5, 0), DS3231_A1_Minute); // In 5 minutes

  ReadAndWriteSensorData(now);
  ReadCycle++;
  
  EnterSleep();  // Go to sleep
}

void ReadAndWriteSensorData(DateTime now) {
  myFile = SD.open("sensor.txt", FILE_WRITE); // Open file for writing
  IMU.readSensor(); // Read the sensor
    
  // Save sensor data
  myFile.print(ReadCycle);
  myFile.print(",");
  myFile.print(now.year());
  myFile.print("/");
  myFile.print(now.month());
  myFile.print("/");
  myFile.print(now.day());
  myFile.print(" ");
  myFile.print(now.hour());
  myFile.print(":");
  myFile.print(now.minute());
  myFile.print(":");
  myFile.print(now.second());
  myFile.print(",");
  myFile.print(IMU.getAccelX_mss(),6);
  myFile.print(",");
  myFile.print(IMU.getAccelY_mss(),6);
  myFile.print(",");
  myFile.print(IMU.getAccelZ_mss(),6);
  myFile.print(",");
  myFile.print(IMU.getGyroX_rads(),6);
  myFile.print(",");
  myFile.print(IMU.getGyroY_rads(),6);
  myFile.print(",");
  myFile.print(IMU.getGyroZ_rads(),6);
  myFile.print(",");
  myFile.print(IMU.getMagX_uT(),6);
  myFile.print(",");
  myFile.print(IMU.getMagY_uT(),6);
  myFile.print(",");
  myFile.print(IMU.getMagZ_uT(),6);
  myFile.print(",");
  myFile.println(IMU.getTemperature_C(),6);
    
  myFile.close();
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
