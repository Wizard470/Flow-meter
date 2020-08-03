// Setting Date and time of DS3231 RTC
#include <RTClib.h>
#include <MPU9250.h>

MPU9250 IMU(Wire,0x69);
RTC_DS3231 rtc;

int status;

void setup () {
  Serial.begin(115200);

  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    while(1) {}
  }
  
  // Sets the RTC for the date and time the sketch was compiled
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  //rtc.adjust(DateTime(2020, 7, 28, 10, 0, 0)); // Or explicitly

  ReadDateTime();
  
  status = IMU.begin();
  
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.print("Status: ");
    Serial.println(status);
    while(1){}
  }

  MPU9250MagCal();
  ReadMPU9250MagCal();
}

void loop () {

}
void ReadDateTime() {
    DateTime now = rtc.now();
    Serial.print("The date is set to: ");
    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print(" ");
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.println();  
}

void MPU9250MagCal(){
  Serial.println("Start MPU9250 magnetic sensor calibration...");
  IMU.calibrateMag();
  Serial.println("Done! You can power off the device.");
}

void ReadMPU9250MagCal() {
  Serial.println("MPU9250 calibration parameters:");

  Serial.print("hxb: ");
  Serial.print(IMU.getMagBiasX_uT(),6);
  Serial.print("\t");
  Serial.print("hyb: ");
  Serial.print(IMU.getMagBiasY_uT(),6);
  Serial.print("\t");
  Serial.print("hzb: ");
  Serial.print(IMU.getMagBiasZ_uT(),6);
  Serial.print("\t");
  Serial.print("hxs: ");
  Serial.print(IMU.getMagScaleFactorX(),6);
  Serial.print("\t");
  Serial.print("hys: ");
  Serial.print(IMU.getMagScaleFactorY(),6);
  Serial.print("\t");
  Serial.print("hzs: ");
  Serial.println(IMU.getMagScaleFactorZ(),6);
  Serial.println("---------------------------------------------------------------------------------------------------------------------------------------------------");
  Serial.flush();
}
