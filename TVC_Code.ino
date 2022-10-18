#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050_tockn.h>
#include <Adafruit_BMP280.h>
#include <Servo.h>
#include <SD.h>

//Sensor Start
MPU6050 mpu6050(Wire);
Adafruit_BMP280 bmp;

//Timer
unsigned long FlightStage0Timer, FlightStage1Timer;
unsigned long SDwriteTimer, lastWrite = 0;
unsigned long StartupTimer;
unsigned long MeasurementTimer;

//SD-Card 
const int chipSelect = BUILTIN_SDCARD;
char datalog[] = "LOGGER00.CSV";
char infolog[] = "INFO00.txt";
File logfile;
File info;

//Flightconfig
int FlightStage = 0;

//Variables
int xAngle;
int yAngle;
float LastHeightReached = 0.0;
float CurrenHeightReached = 0.0;
int ApogeeCount = 0;
float Apogee = 0.0;
int Buzzer = 5;
int LEDred = 28;
int LEDgrn = 27; 


//Servo-Config
Servo servoX;
Servo servoY;


void setup() {
  Wire.begin();
  Serial.begin(9600);

  //Pin-Setup
  pinMode(LEDred, OUTPUT);
  pinMode(LEDgrn, OUTPUT);
  pinMode(Buzzer, OUTPUT);

  //Buzzer-Test
  tone(Buzzer, 700);
  delay(500);
  tone(Buzzer, 1000);
  delay(500);
  noTone(Buzzer);
   
  //SD-Card Start
  Serial.println("========================================");
  Serial.print("SD-Card initialisation ...");
  if (!SD.begin(chipSelect)) {
    Serial.println(" failed.");
    ClrRed();
    return;
  }
  
  Serial.println(" done.");
  
  
  Serial.print("Files creating ...");

  for (uint8_t i = 0; i <= 100; i++) {
    infolog[4] = i/10 + '0';
    infolog[5] = i%10 + '0';
    if (! SD.exists(infolog)) {
      info = SD.open(infolog, FILE_WRITE);
      break;
    }
  }
  
  for (uint8_t i = 0; i <= 100; i++) {
    datalog[6] = i/10 + '0';
    datalog[7] = i%10 + '0';
    if (! SD.exists(datalog)) {
      logfile = SD.open(datalog, FILE_WRITE); 
      break;  
    }
  }
  
  if (! logfile) {
    Serial.println(" failed.");
    ClrRed();
    return;
  }
  if (! infolog) {
    Serial.println(" failed.");
    ClrRed();
    return;
  }

  else {
    Serial.println(" done.");
    Serial.print("Logging to: ");
    Serial.print(datalog);
    Serial.print(" / ");
    Serial.println(infolog);
  }
  
  File logfile = SD.open(datalog, FILE_WRITE);
  logfile.println("MeasurementTimer,TemperatureBMP,TemperatureMPU,PressureBMP,AlltitudeBMP,Flightstage,xAngleMPU,yAngleMPU,zAccMPU");



  //Sensor Check
  Serial.println("========================================");
  unsigned status;
  Serial.print("BMP280 initialisation ...");
  status = bmp.begin(0x76);
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  delay(1000);
  if (!status) {
    Serial.println(" failed.");
    ClrRed();
    delay(500);
    Clr0();
    delay(500);
    ClrRed();
    delay(500);
    Clr0();
    return; 
  }
  else {
    Serial.println(" done.");
  }
  
  Serial.print("MPU6050 initialisation ... ");
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  //Test-Data
  Serial.println("BMP280 testdata:");

  Serial.print(F("Temperature = "));
  Serial.print(bmp.readTemperature());
  Serial.println(" *C");

  Serial.print(F("Pressure = "));
  Serial.print(bmp.readPressure());
  Serial.println(" Pa");

  Serial.print(F("Approx altitude = "));
  Serial.print(bmp.readAltitude(1013.25));
  Serial.println(" m");
  Serial.println("========================================");
  Serial.println("MPU6050 testdata:");
  mpu6050.update();

  Serial.print("Temperature: ");
  Serial.print(mpu6050.getTemp());
  Serial.println(" *C");

  Serial.print("xAngleMPU: ");
  Serial.println(mpu6050.getAngleX());

  Serial.print("yAngleMPU: ");
  Serial.println(mpu6050.getAngleY());

  Serial.print("zAcc: ");
  Serial.println(mpu6050.getAccZ());
  Serial.println("========================================");
  

  servoX.attach(20);
  servoY.attach(21);
  servoX.write(90);
  servoY.write(90);

  ClrGrn();
  delay(1000);
  StartupTimer = millis();
}

void loop() {
  MeasurementTimer = millis() - StartupTimer;
  mpu6050.update(); 
  
  if (FlightStage == 0){
    if (mpu6050.getAccZ() >= 1.8){
      FlightStage0Timer = millis() - StartupTimer;
      FlightStage = 1;
      Clr0();
      
    }
  }
  TVCgimble();
  SDlog();
  

}
void TVCgimble() {
  if (FlightStage == 1){
    CurrenHeightReached = bmp.readAltitude(1013.25);
    xAngle = mpu6050.getGyroAngleX() + 90;
    yAngle = mpu6050.getGyroAngleY() + 90;
    servoX.write(xAngle);
    servoY.write(yAngle);
    ApogeeDetect();
    LastHeightReached = CurrenHeightReached;
  }
}

void ApogeeDetect() {
  if (CurrenHeightReached < LastHeightReached) {
    ApogeeCount =  ApogeeCount + 1;
  }
  else {
    ApogeeCount = 0;
  }

  if (ApogeeCount > 3) {
    Apogee = bmp.readAltitude(1013.25);
    FlightStage = 2;
    FlightStage1Timer = millis() - FlightStage0Timer;
    servoX.write(90);
    servoY.write(90);
    Clr0();
    ClrGrn();
    ClrRed();
  }
  
}


void SDlog() {
  //saves data from Sensors on SD-Card
  File logfile = SD.open(datalog, FILE_WRITE);
  logfile.print(MeasurementTimer);
  logfile.print(",");
  logfile.print(bmp.readTemperature());
  logfile.print(",");
  logfile.print(mpu6050.getTemp());
  logfile.print(",");
  logfile.print(bmp.readPressure());
  logfile.print(",");
  logfile.print(bmp.readAltitude(1013.25));
  logfile.print(",");
  logfile.print(FlightStage);
  logfile.print(",");
  logfile.print(mpu6050.getAngleX());
  logfile.print(",");
  logfile.print(mpu6050.getAngleY());
  logfile.print(",");
  logfile.print(mpu6050.getAccZ());
  logfile.println();
}

//LED-Functions
void Clr0() {
  digitalWrite(LEDred, LOW);
  digitalWrite(LEDgrn, LOW);
  Serial.println("LED off");
}
void ClrRed() {
  digitalWrite(LEDred, HIGH);
  Serial.println("LED red");
}
void ClrGrn() {
  digitalWrite(LEDgrn, HIGH);
  Serial.println("LED green");
}
