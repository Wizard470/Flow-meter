#include <MPU9250.h>
#include <SPI.h>
#include <SD.h>

MPU9250 IMU(Wire,0x69); //Because AD0 = 1 
int status;
File myFile;
int ReadCycle = 1;

void setup() {
  SD.begin(9);
  //myFile = SD.open("sensor.txt", FILE_WRITE); // Open file for writing
  
  status = IMU.begin(); // Start communication with MPU9250
  if (status < 0) {
    myFile.println("IMU initialization unsuccessful");
    myFile.print("Status: ");
    myFile.println(status);
    while(1) {} // Interrupt program execution
  }
  
  // Setting the accelerometer full scale range to +/-8G 
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_4G);
  // Setting the gyroscope full scale range to +/-500 deg/s
  IMU.setGyroRange(MPU9250::GYRO_RANGE_250DPS);
  // Setting DLPF bandwidth to 5 Hz
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_5HZ);
  // Setting SRD to 49 for a 20 Hz update rate
  IMU.setSrd(49);
  
  myFile.close();
  
}

void loop() {
  ReadAndWriteSensorData();
  ReadCycle++;
}

void ReadAndWriteSensorData() {
  myFile = SD.open("sensor.txt", FILE_WRITE); // Open file for writing
  IMU.readSensor(); // Read the sensor
    
  // Save sensor data
  myFile.print(ReadCycle);
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
