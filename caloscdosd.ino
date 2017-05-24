#include <SPI.h>
#include <SoftwareSerial.h>
#include <SD.h>
#include <TinyGPS.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU9250.h>

SoftwareSerial gpsBoard (8, 9);
TinyGPS gps;
MPU9250 accelgyro;
I2Cdev   I2C_M;

int buttonState = HIGH;
int ledState = -1;
long lastDebounceTime = 0;
long debounceDelay = 1000;

const int chipSelect = 4;
char filename[16];

uint8_t buffer_m[6];
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;

float Axyz[3];


const float alpha = 0.5;
double fXg = 0;
double fYg = 0;
double fZg = 0;

static void print_float(float val, float invalid, int len, int prec);
static void print_int(unsigned long val, unsigned long invalid, int len);
static void print_date(TinyGPS &gps);
bool newData = false;

bool start = 1;

File dataFile;

void setup() {

  pinMode(6, INPUT_PULLUP);
  pinMode(13, OUTPUT);
  pinMode(5, OUTPUT);

  //  Serial.begin(9600);
  //  while (!Serial);
  delay(25);
  gpsBoard.begin(9600);
  delay(25);
  Wire.begin();
  delay(25);
  accelgyro.initialize();
  delay(25);

  if (!SD.begin(chipSelect)) {
    // Serial.println("Card failed, or not present");
    // don't do anything more:
    digitalWrite(13, LOW);
    return;
  }

  int n = 0;
  snprintf(filename, sizeof(filename), "zapis%03d.txt", n); // includes a three-digit sequence number in the file name
  while (SD.exists(filename)) {
    n++;
    snprintf(filename, sizeof(filename), "zapis%03d.txt", n);
  }
  dataFile = SD.open(filename, FILE_READ);
  //  Serial.println(n);
  //  Serial.println(filename);
  dataFile.close();
  delay(1000);


}

void loop()
{
  getAccel_Data();

  for (unsigned long start = millis(); millis() - start < 300;)
  {
    while (gpsBoard.available())
    {
      char c = gpsBoard.read();
      if (gps.encode(c)) // Did a new valid sentence come in?
      {
        newData = true;
        digitalWrite(13, HIGH);
      }
    }
  }

  if (newData == true && start == 1)
  {
    dataFile = SD.open(filename, FILE_WRITE);
    if (dataFile)
    {
      //      Serial.println("REJESTRATOR v1.0");
      //      Serial.println("by BOGON");
      //      Serial.println();
      //      Serial.println();
      //      Serial.println("Data       Godzina      Szerokosc Dlugosc    Wysokosc SATS Przyspiezenie   Pochylenie");
      //      Serial.println("                        (deg)     (deg)      (m)           X, Y, Z                  Przechylenie");
      //      Serial.println("----------------------------------------------------------------------------------------------------------------");


      dataFile.println("REJESTRATOR v1.0");
      dataFile.println("by BOGON");
      dataFile.println();
      dataFile.println();
      dataFile.println("Data       Godzina      Szerokosc Dlugosc    Wysokosc SATS Przyspiezenie   Pochylenie");
      dataFile.println("                        (deg)     (deg)      (m)           X, Y, Z                  Przechylenie");
      dataFile.println("----------------------------------------------------------------------------------------------------------------");
      dataFile.close();
    }
    start = 0;

  }

  if (newData == true)
  {
    digitalWrite(5, HIGH);
    dataFile = SD.open(filename, FILE_WRITE);
    if (dataFile)
    {

      float flat, flon;
      unsigned long date, time;

      print_date(gps);
      dataFile.print("\t");
      dataFile.print('\t');
      //      Serial.print("\t");
      gps.f_get_position(&flat, &flon);

      print_float(flat, TinyGPS::GPS_INVALID_F_ANGLE, 10, 6);
      print_float(flon, TinyGPS::GPS_INVALID_F_ANGLE, 11, 6);
      print_float(gps.f_altitude(), TinyGPS::GPS_INVALID_F_ALTITUDE, 7, 2);
      dataFile.print("  ");
      //      Serial.print("  ");
      print_int(gps.satellites(), TinyGPS::GPS_INVALID_SATELLITES, 5);

      dataFile.print(Axyz[0]);
      dataFile.print(",");
      dataFile.print(Axyz[1]);
      dataFile.print(",");
      dataFile.print(Axyz[2]);
      //      Serial.print(Axyz[0]);
      //      Serial.print(",");
      //      Serial.print(Axyz[1]);
      //      Serial.print(",");
      //      Serial.print(Axyz[2]);
      dataFile.print("  ");
      //      Serial.print("  ");
      printRollPitch();

      dataFile.print('\n');
      dataFile.println();
      //      Serial.println();
      dataFile.close();
    }
  }
}




static void print_float(float val, float invalid, int len, int prec)
{
  if (val == invalid)
  {
    while (len-- > 1)
    {
      dataFile = SD.open(filename, FILE_WRITE);
      dataFile.print('*');
      dataFile.print(' ');
      dataFile.close();
    }

  }
  else
  {
    //    Serial.print(val, prec);
    dataFile = SD.open(filename, FILE_WRITE);
    dataFile.print(val, prec);
    dataFile.close();
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i = flen; i < len; ++i)
    {
      //      Serial.print(' ');
      dataFile = SD.open(filename, FILE_WRITE);
      dataFile.print(' ');
      dataFile.close();
    }

  }
}

static void print_int(unsigned long val, unsigned long invalid, int len)
{
  char sz[32];
  if (val == invalid)
    strcpy(sz, "*******");
  else
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i = strlen(sz); i < len; ++i)
    sz[i] = ' ';
  if (len > 0)
  {
    sz[len - 1] = ' ';
    //    Serial.print(sz);
    dataFile = SD.open(filename, FILE_WRITE);
    dataFile.print(sz);
    dataFile.close();
  }

}

static void print_date(TinyGPS &gps)
{
  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned long age;
  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
  if (age == TinyGPS::GPS_INVALID_AGE)
  {
    dataFile = SD.open(filename, FILE_WRITE);
    dataFile.print("********** ******** ");
    dataFile.close();

    //   Serial.print("********** ******** ");
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d %02d:%02d:%02d ",
            day, month, year, hour + 2, minute, second);
    //    Serial.print(sz);
    dataFile = SD.open(filename, FILE_WRITE);
    dataFile.print(sz);
    dataFile.close();

  }

}

void getAccel_Data(void)
{
  accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  Axyz[0] = ((double) ax / 16384);
  Axyz[1] = ((double) ay / 16384);
  Axyz[2] = ((double) az / 16384);
}

void printRollPitch(void)
{
  double ppp, rrr;
  //Low Pass Filter
  fXg = Axyz[0] * alpha + (fXg * (1.0 - alpha));
  fYg = Axyz[1] * alpha + (fYg * (1.0 - alpha));
  fZg = Axyz[2] * alpha + (fZg * (1.0 - alpha));

  //Roll & Pitch Equations
  rrr  = (atan2(-fYg, fZg) * 180.0) / M_PI;
  ppp = (atan2(fXg, sqrt(fYg * fYg + fZg * fZg)) * 180.0) / M_PI;

  //  Serial.print(ppp = ppp + 4.5);
  //  Serial.print("  ");
  //  Serial.print(rrr = rrr + 0.6);
  dataFile = SD.open(filename, FILE_WRITE);
  dataFile.print(ppp = ppp + 4.5);
  dataFile.print("  ");
  dataFile.print(rrr = rrr + 0.6);
  dataFile.close();

}
