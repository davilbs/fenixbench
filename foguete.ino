//SW1=Pin94=dig57=A3-middle (Won't be used)
//SW2=Pin93=dig58=A4-beside of volt meter
//SW3=Pin92=dig59=A5-beside of antenna
#include <Wire.h>
#include <CircularBuffer.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <MS5611.h>
#include <Servo.h>
#define NOTE_G4  392
#define NOTE_C5  523
#define NOTE_E5  659
#define NOTE_G5  784
#define REST  0

//Iicialize song
int tempo = 200;
int buzzer = 60;
int melody[] = {
  NOTE_E5, 8, NOTE_E5, 8, REST, 8, NOTE_E5, 8, REST, 8, NOTE_C5, 8, NOTE_E5, 8, //1
  NOTE_G5, 4, REST, 4, NOTE_G4, 8, REST, 4
};
int notes = sizeof(melody) / sizeof(melody[0]) / 2;
int wholenote = (60000 * 4) / tempo;
int divider = 0, noteDuration = 0;
//******

int GPSBaud = 9600; //Comunication speed of GPS module
unsigned long tim = 0;
double Fbar = 0, Fimu = 0, FLoc = 0, reserva = 2, Accx1 = 0;

Servo myservo;
TinyGPSPlus gps;
SoftwareSerial gpsSerial(11, 12);  //GPS PINs
CircularBuffer<int, 5> buffer;  //Number of points to calculate a average height
File myFile;
MS5611 ms5611;
Adafruit_MPU6050 mpu;

void setup() {
  //Song **
  for (int thisNote = 0; thisNote < notes * 2; thisNote = thisNote + 2) {
    divider = melody[thisNote + 1];
    if (divider > 0) {
      noteDuration = (wholenote) / divider;
    } else if (divider < 0) {
      noteDuration = (wholenote) / abs(divider);
      noteDuration *= 1.5;
    }
    tone(buzzer, melody[thisNote], noteDuration * 0.9);
    delay(noteDuration);
    noTone(buzzer);
  }
  //**

  Serial.begin(57600);  //Comunication speed of telemetry
  myservo.attach(45);
  pinMode(30, OUTPUT);  //LED 1
  pinMode(33, OUTPUT);  //LED 2
  pinMode(32, OUTPUT);  //LED 3
  pinMode(31, OUTPUT);  //LED 4
  pinMode(46, OUTPUT);  //Servo 1
  pinMode(45, OUTPUT);  //Servo 2
  pinMode(44, OUTPUT);  //Servo 3
  pinMode(22, INPUT);  //Altimeter 1 e o que esta do lado do furo de prender cabo
  pinMode(69, INPUT);  //Altimeter 2 e o mais perto do sdcard
  pinMode(24, INPUT);  //Door 1
  pinMode(23, INPUT);  //Door 2
  pinMode(57, OUTPUT);  //Parachute 1
  pinMode(58, OUTPUT);  //Parachute 2
  pinMode(59, OUTPUT);  //Parachute 3
  pinMode(60, OUTPUT);  //Buzzer
  //digitalWrite(57, LOW); //Parachute 1 nao esta sendo usado, mas pode.
  digitalWrite(58, LOW); //Parachute 2 e o que esta perto do meditor de tensao
  digitalWrite(59, LOW); //Parachute 3 e o que esta do lado da antena
  myservo.write(0); //Place servo to 0 degrees


  SD.begin(10);  // Port to data of Sdcard
  while (!ms5611.begin(MS5611_HIGH_RES)) {
    Fbar = 0;
    delay(500);
  }
  Fbar = 1;

  while (!Serial) {
    delay(10); // waiting
  }

  if (!mpu.begin()) {
    Fimu = 0;
  }

  else Fimu = 1;

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G); // Max acceleration
  mpu.setGyroRange(MPU6050_RANGE_500_DEG); //Max angle range
  mpu.setFilterBandwidth(MPU6050_BAND_184_HZ); //Max bandwidth
  delay(10);

  gpsSerial.begin(GPSBaud);
 
  /////////***HEADER****/////////
  //Serial.print("Acc X   ");
  //  Serial.print("Acc Y   ");
  //  Serial.print("Acc Z   ");
  //  Serial.print("Rot X   ");
  //  Serial.print("Rot y   ");
  //  Serial.print("Rot z   ");
  //  Serial.print("Latitude    ");
  //  Serial.print("Longitude   ");
  //  Serial.print("Avhei  ");
  //  Serial.print("VerSpe   ");
  //  Serial.println("Time  ");
  //****AINDA TEM QUE ADEQUAR*****
}

//___________LOOP_____________//

void loop() {
  static double lati, lon, A[] = {0.00, 0.00, 0.00, 0.00, 0.00, 0.00}, InstantaneousHeight, VertSpeed, ExpectedHeight, Avh, Accx, timer = 0, volt, current;
  static unsigned long tim1 = 0, tim2 = 0;
  static bool door1 = false, door2 = false, FlagImu=true;

  if (millis() - tim1 >= 10) {
    tim1 = millis();
    while (gpsSerial.available() > 0)
      if (gps.encode(gpsSerial.read()))
        LocationInfo(&lati, &lon);

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
//    if(FlagImu==true){
//      Accx1 = -a.acceleration.x;
//    }
//    FlagImu=false;
    //Serial.println("");
    //Serial.print("Alt1=");
    //Serial.println(digitalRead(22));
    //Serial.print("ALT2=");
    //Serial.println(digitalRead(69));

    //
    //  if (!mpu.begin())
    //    Fimu=0;
    //  else Fimu=1;
    //
    //  if(!ms5611.begin(MS5611_HIGH_RES))
    //    Fbar=0;
    //  else Fbar=1;

    //*BUZZER CHECK**///
    if (Fbar == 0 || Fimu == 0) {
      if (millis() - tim2 >= 1000) {
        tim2 = millis();
        BuzError();
      }
    }

    if (Fbar == 1 && Fimu == 1) {
      if (millis() - tim2 >= 3000) {
        tim2 = millis();
        BuzOk();
      }
    }

    //*THE END BUZZER CHECK**//

    //Serial.print("Floc=");
    //Serial.println(FLoc);
    //Serial.print("Fbar=");
    //Serial.println(Fbar);
    //Serial.print("Fimu=");
    //Serial.println(Fimu);

    if (Fbar == 0)
      digitalWrite(33, HIGH);
    else
      digitalWrite(33, LOW);

    if (FLoc == 0)
      digitalWrite(31, HIGH);
    else
      digitalWrite(31, LOW);

    if (Fimu == 0)
      digitalWrite(30, HIGH);
    else
      digitalWrite(30, LOW);
    //reserva = reserva * cos(3.14);
    //    volt = Volt();
    //    current = volt / .2;
    //    Serial.print("volt");
    //    Serial.println(volt);
    //    Serial.print(a.acceleration.x);
    //    Serial.print("   ");
    //    Serial.print(a.acceleration.y);
    //    Serial.print("    ");
    //    Serial.print(a.acceleration.z);
    //    Serial.print("   ");
    //    //Serial.println(" m/s^2");
    //    Serial.print(g.gyro.x * 180 / 3.14);
    //    Serial.print("   ");
    //    Serial.print(g.gyro.y * 180 / 3.14);
    //    Serial.print("   ");
    //    Serial.print(g.gyro.z * 180 / 3.14);
    //    Serial.print("    ");

    InstantaneousHeight = readPressure();
    Avh = averHeight(InstantaneousHeight);

    //** Shift position *//
    A[5] = A[4];
    A[4] = A[3];
    A[3] = A[2];
    A[2] = A[1];
    A[1] = A[0];
    A[0] = Avh;

    VertSpeed = AverVertSpeed(A[0], A[1]);
    ExpectedHeight = HeiEst(VertSpeed, Avh);
//        Serial.print(lati, 6);
//        Serial.print("    ");
//        Serial.print(lon, 6);
//        Serial.print("    ");

//    Serial.print(Avh);
//    Serial.print(",");
//    Serial.print(InstantaneousHeight);
//    Serial.print(",");
//    Serial.println(VertSpeed);
//    Serial.print(",");
//    Serial.print(A[0]);
//    Serial.print(",");
//    Serial.print(A[1]);
//    Serial.print(",");
//    Serial.println(millis());


    static int Parac = Parachutes(A[0], A[4]);
    CheckAlt1(Parac);
    CheckAlt2(Parac);


    //////////////VERIFICAER////////////////
    storage(a.acceleration.x, a.acceleration.y, a.acceleration.z, g.gyro.x, g.gyro.y, g.gyro.z, Avh, VertSpeed, ExpectedHeight, millis());
    double tim2 = tim1;
    Accx = a.acceleration.y;
    //    Serial.print("Accz=");
    //    Serial.println(Accz);
    timer = millis();
    //Serial.println(Accx);
    //SendToPython(&Accx, &Avh, &VertSpeed, &ExpectedHeight, &timer);//,&ExpectedHeight);

    Serial.print(Accx);
    Serial.print(" , ");
    Serial.print(Avh);
    Serial.print(" , ");
    Serial.print(VertSpeed);
    Serial.print(" , ");
    Serial.println(ExpectedHeight);
//    Serial.print(" , ");
//    Serial.println(timer);
    //////////*******//////////////////////
  }
  
}

//______ FUNCTION RESPONSIBLE TO SEND DATAS TO GROUND STATION _______//

void SendToPython(double* data1, double* data2, double* data3, double* data4, double* data5) //double* data6, double* data7, double* data8, double* data9, double* data10, double* data11)//, double* data12)
{
  byte* byteData1 = (byte*)(data1);
  byte* byteData2 = (byte*)(data2);
  byte* byteData3 = (byte*)(data3);
  byte* byteData4 = (byte*)(data4);
  byte* byteData5 = (byte*)(data5);
//  byte* byteData6 = (byte*)(data6);
//  byte* byteData7 = (byte*)(data7);
//  byte* byteData8 = (byte*)(data8);
//  byte* byteData9 = (byte*)(data9);
//  byte* byteData10 = (byte*)(data10);
//  byte* byteData11 = (byte*)(data11);
  //byte* byteData12 = (byte*)(data12);
  //  byte* byteData13 = (byte*)(data13);


  byte buf[20] = {byteData1[0], byteData1[1], byteData1[2], byteData1[3],
                  byteData2[0], byteData2[1], byteData2[2], byteData2[3],
                  byteData3[0], byteData3[1], byteData3[2], byteData3[3],
                  byteData4[0], byteData4[1], byteData4[2], byteData4[3],
                  byteData5[0], byteData5[1], byteData5[2], byteData5[3]
//                  byteData6[0], byteData6[1], byteData6[2], byteData6[3],
//                  byteData7[0], byteData7[1], byteData7[2], byteData7[3],
//                  byteData8[0], byteData8[1], byteData8[2], byteData8[3],
//                  byteData9[0], byteData9[1], byteData9[2], byteData9[3],
//                  byteData10[0], byteData10[1], byteData10[2], byteData10[3],
//                  byteData11[0], byteData11[1], byteData11[2], byteData11[3]
                  //byteData12[0], byteData12[1], byteData12[2], byteData12[3]
                 };
  //  byteData13[0], byteData13[1], byteData13[2], byteData13[3]};
  Serial.write(buf, 20);
}

//___________PARACHUTES_____________//

int Parachutes(double Pos0, double Pos5) {
  static int Count = 0;
  if ((Pos5 < (Pos0 + 1)) && (Pos0 > 500)) {
    Count++;
    if (Count > 10 && Count < 29)return (2);
    if (Count > 30)return (1);
  }
  else return (0);
}
//__________AVERAGE SPEED____________//

double AverVertSpeed(double Pos0, double Pos1) {
  static double Deltime;
  int Speed = (Pos0 - Pos1) * 10 / ( (millis() - Deltime)); //m/s
  Deltime = millis();
  return (Speed);

}
//__________averHeight___________//

double averHeight(double InstantaneousHeight) {
  double a = InstantaneousHeight;
  //if(a>.20){
  buffer.push((int)InstantaneousHeight);
  if (millis() - tim >= 20) {
    //Serial.print("time=");
    //Serial.println(millis() - tim);
    tim = millis();

    float avgHeigt = 0.00;
    using index_t = decltype(buffer)::index_t;
    for (index_t i = 0; i < buffer.size(); i++) {
      avgHeigt += buffer[i] / (float)buffer.size();
      // if(i>1 && (buffer[i] / (float)buffer.size()<buffer[i-1] / (float)buffer.size())){

    }
    return (avgHeigt);
  }

}
//__________SD CARD STORAGE___________//

double storage(double Accx, double Accy, double Accz, double Rotx, double Roty,
               double Rotz, double AveHei,  double Speed, double expectedHeigh, double Time) {
  String txt = ".txt";

  static int b;
  static bool count = true;
  if (count) {
    for (int a = 0; a < 100; a++) {
      if (SD.exists(String(a) + txt)) {
        b = a;
      }
    }
  }
  myFile = SD.open(String(b + 1) + txt, FILE_WRITE);
  if (myFile) {
    if (count) {
      myFile.println("Acc X   Acc Y   Acc Z   Rot X   Rot Y   Rot Z   Avhei   Speed    Time  ExpectedHeight");
        
      count = false;
    }
    myFile.print(Accx);
    myFile.print("   ");
    myFile.print(Accy);
    myFile.print("   ");
    myFile.print(Accz);
    myFile.print("   ");
    myFile.print(Rotx);
    myFile.print("   ");
    myFile.print(Roty);
    myFile.print("   ");
    myFile.print(Rotz);
    myFile.print("     ");
    myFile.print(AveHei);
    myFile.print("     ");
    myFile.print(Speed);
    myFile.print("     ");
    myFile.print(expectedHeigh);
    myFile.print("     ");
    myFile.println(millis());
    myFile.close();
  }
}
//____________PRESSURE CALCULATOR____________//

double readPressure()
{ static int i = 0;
  int NumIte = 50; 
  static double AveHeig = 0, baseline = 0, SetPoint = 0;
  if (i < NumIte) {
    for (i = 0; i < NumIte; i++) {
      //double realTemperature = ms5611.readTemperature(true);
      double realPressure = ms5611.readPressure();
      double realAltitude = ms5611.getAltitude(realPressure);
      AveHeig = AveHeig + realAltitude;
      //      Serial.print("AveHeig=");
      //      Serial.println(AveHeig);
    }
    baseline = AveHeig / NumIte;
    //  Serial.print("baseline");
    //  Serial.println(baseline);
  }
  SetPoint = ms5611.getAltitude(ms5611.readPressure(true)) - baseline;
  //FilterHeight(SetPoint);
  return (SetPoint);//-FilterHeight(SetPoint));
}
//FilterHeight

//____________LOCATION____________//

void LocationInfo(double *LAT, double *LON )//FUNÇÃO RESPONSAVEL PELA LEITURA DOS DADOS
{
  if (gps.location.isValid())//SE A LOCALIZAÇÃO DO SINAL ENCONTRADO É VÁLIDA, ENTÃO
  {
    *LAT = gps.location.lat();
    *LON = gps.location.lng();
    FLoc = 1;
    // Serial.println(" location identified");
  }
  else
  {
    FLoc = 0;
    //Serial.println("No location identified yet");//No identified location
  }
}

//_________ BEEP FOR PROBLEMS CONDITIONS ___________//

void BuzError() {
  static int count = 0;
  //if(count==0)
  tone(60, 500, 30);

  if (count == 1)
    tone(60, 1500, 30);

  count++;
  if (count == 2)
    count = 0;
}

//_________ BEEP FOR REGULARS CONDITIONS __________//

void BuzOk() {
  tone(60, 2000, 30);
}

//___________ VOLT METER _____________//

double Volt() {
  double  volt1, volt2, volt;
  volt1 = analogRead(A1);
  volt2 = analogRead(A0);
  volt = volt1 - volt2;
  return (volt);
}

//___________ALTIMETER 1 INPUT CHECK _______//

void CheckAlt1(int parac) {

  static double starTime, starTime2;
  static bool Flag = true;
  if ((digitalRead(22) == HIGH) || parac == 1)
    Flag = false;
  //Serial.print("flag=");
  //Serial.println(Flag);
  if (Flag == true)
    starTime = millis();
  //Serial.print("startime=");
  //Serial.println(starTime);
  if (Flag == false && ((millis() - starTime) < 4000) ) {
    digitalWrite(58, HIGH);
    starTime2 = millis();
  }
  else {
    digitalWrite(58, LOW);
    if (((millis() - starTime2) < 4000) ) {
      digitalWrite(59, HIGH);
    }
    else
      digitalWrite(59, LOW);
  }
}
//________ ALTIMETER 2 INPUT CHECK SERVO CUBESAT ______//

void CheckAlt2(int parac) {

  static double starTime;
  static bool Flag = true;
  if (parac == 2)
    Flag = false;

  if (Flag == true)
    starTime = millis();

  if ((Flag == false && (millis() - starTime < 10000)) || digitalRead(24) == HIGH) {
    //digitalWrite(59, HIGH);
    myservo.write(40);//Place servo  to 180 degrees.
  }
  else {
    //digitalWrite(59, LOW);
    myservo.write(0);
  }
}

//__________ HEIGHT ESTIMATOR ________//

double HeiEst(double VertSpeed, double Avh) {
  static double starTime, ExpHei = 0;
  static bool Flag = true;
  if (Avh > 100)
    Flag = false;

  if (Flag == true)
    starTime = millis();

  if (Flag == false && (millis() - starTime > 2000) ) {
    ExpHei = (VertSpeed * VertSpeed) / (19.62) + Avh;
    return (ExpHei);
  }
  else {
    return (0);
  }
}


//______ ANT-DEVIATION FILTER FOR HEIGHT CALCULATOR ______//

//double FilterHeight (double SetPoint) {
//  static double FilterTime = 0, HoldValue = 0, dif = 0;
//
//  if ((millis() - FilterTime) > 40000) {
//    if ((SetPoint > HoldValue + .20) || (SetPoint < HoldValue - .20)) {
//    HoldValue = SetPoint;
//    dif = 0;
//    return (dif);
//    }
//    else {
//      dif = SetPoint - HoldValue;
//      return (dif);
//    }
//  }
//  else {
//    return(0);
//  }
//}