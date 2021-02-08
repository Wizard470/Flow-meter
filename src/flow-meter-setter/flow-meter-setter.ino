// Setting Date and time of DS3231 RTC
#include <DS3232RTC.h>
#include <MPU9250.h>

MPU9250 IMU(Wire,0x69);

int status;

void setup () {
  Serial.begin(115200);
   
  // Sets the RTC for the date and time the sketch was compiled
  RTC.set(compileTime());

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
    time_t t = RTC.get();
    Serial.print("The date is set to: ");
    Serial.print(year(t), DEC);
    Serial.print('/');
    Serial.print(month(t), DEC);
    Serial.print('/');
    Serial.print(day(t), DEC);
    Serial.print(" ");
    Serial.print(hour(t), DEC);
    Serial.print(':');
    Serial.print(minute(t), DEC);
    Serial.print(':');
    Serial.print(second(t), DEC);
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

time_t compileTime()
{
    const time_t FUDGE(10);    //fudge factor to allow for upload time, etc. (seconds, YMMV)
    const char *compDate = __DATE__, *compTime = __TIME__, *months = "JanFebMarAprMayJunJulAugSepOctNovDec";
    char compMon[4], *m;

    strncpy(compMon, compDate, 3);
    compMon[3] = '\0';
    m = strstr(months, compMon);

    tmElements_t tm;
    tm.Month = ((m - months) / 3 + 1);
    tm.Day = atoi(compDate + 4);
    tm.Year = atoi(compDate + 7) - 1970;
    tm.Hour = atoi(compTime);
    tm.Minute = atoi(compTime + 3);
    tm.Second = atoi(compTime + 6);

    time_t t = makeTime(tm);
    return t + FUDGE;        //add fudge factor to allow for compile time
}
