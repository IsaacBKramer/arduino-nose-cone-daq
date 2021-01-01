#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <HX711.h>

#define FS_BYTE_8G          0b00010000 // byte for +/- 8g full scale
#define FS_SENSITIVITY_8G   4096       // LSB/g

#define AVG_COUNTER         1000        // number of accelerometer data points to average as reference
#define G_THRESHOLD         1          // threshold in g's to start logging load data  

#define INIT                0
#define READY               1          // waiting for launch acceleration
#define LOGGING             2          // recording load cell data
#define DONE                3          // done, do nothing 

typedef struct {
  int16_t x;
  int16_t y;
  int16_t z;
} Accel_t; // acceleration data struct

typedef struct {
  int16_t stag;
  int16_t stat;
} Transducer_t; // pressure transducer data struct

typedef struct {
  long x; 
  long y;
  long z;
} Bias;  // accelerometer bias struct

typedef struct {
  Accel_t accel;
  Transducer_t ducer;
  float load;
} data_t; // flight data struct for accelerometer and ducers

Bias bias;
data_t flightData;
File dataFile;

const int sdVCC = 2;
const int chipSelect = 10; // SPI chip select pin for SD card

const int cellVCC = A0; // load cell power pin
const int cellSCL = A1; // load cell clock pin
const int cellSDA = A2; // load cell data pin
const int calibrationFactor = 20000; // load cell calibration factor for HX711 library

int state = 0; // state tracking for flight

float startTime; 

HX711 cell; // HX711 load cell object

void setup() {

  Serial.begin(38400);

  pinMode(cellVCC, OUTPUT);
  digitalWrite(cellVCC, HIGH); // power for load cell board

  Wire.beginTransmission(0b1101000);
  Wire.write(0x6B); // PWR_MGMT_1
  Wire.write(0b00000000); 
  Wire.endTransmission();

  Wire.beginTransmission(0b1101000);
  Wire.write(0x6C); // PWR_MGMT_2
  Wire.write(0b00000111); // gyroscope axes in standby
  Wire.endTransmission();

  Wire.beginTransmission(0b1101000);
  Wire.write(0x1C); // ACCEL_CONFIG
  Wire.write(FS_BYTE_8G); // set full scale accel range
  Wire.endTransmission();

  cell.begin(cellSDA, cellSCL);
  cell.set_scale(calibrationFactor);
  cell.tare();

  if (!SD.begin(chipSelect)) { // SD card init failure management
    Serial.println("SD init failure."); //
    while(true);
  } else {
    Serial.println("Successful SD init");
  }

  bias.x = 0; 
  bias.y = 0;
  bias.z = 0;
  
}

void loop() {

  switch(state) {
    case INIT: { 
      
      Serial.println("Calibrating bias values...");
      for(int counter=0; counter<AVG_COUNTER; counter++) { 
        Wire.beginTransmission(0b1101000);
        Wire.write(0x3B);
        Wire.requestFrom(0b1101000, 6); // pull bytes from six accelerometer registers
        while (Wire.available() < 6);
        
        bias.x += (Wire.read() << 8) | Wire.read(); // read accelerometer axes
        bias.y += (Wire.read() << 8) | Wire.read();
        bias.z += (Wire.read() << 8) | Wire.read();
        Wire.endTransmission();
        
        Serial.print(bias.x); Serial.print("\t");
        Serial.print(bias.y); Serial.print("\t");
        Serial.println(bias.z);
        
      }
      bias.x = bias.x/AVG_COUNTER; // divide sum of accelerometer readings by number of readings
      bias.y = bias.y/AVG_COUNTER;
      bias.z = bias.z/AVG_COUNTER;
      Serial.println("Bias values ready.");
      Serial.print(bias.x); Serial.print("\t");
      Serial.print(bias.y); Serial.print("\t");
      Serial.println(bias.z);
      delay(2000);
      Serial.println("Ready for launch");
      state++;
    }
    case READY: { // launch detection
      
      while(true) {
        Wire.beginTransmission(0b1101000);
        Wire.write(0x3B); // first accelerometer data register
        Wire.requestFrom(0b1101000, 6); // pull bytes from six accelerometer registers
        
        while (Wire.available() < 6);
        
        flightData.accel.x = ((Wire.read() << 8) | Wire.read()) - int16_t(bias.x);
        flightData.accel.y = ((Wire.read() << 8) | Wire.read()) - int16_t(bias.y);
        flightData.accel.z = ((Wire.read() << 8) | Wire.read()) - int16_t(bias.z);
      
        Wire.endTransmission();
    
        Serial.print(flightData.accel.x); Serial.print("\t");
        Serial.print(flightData.accel.y); Serial.print("\t");
        Serial.println(flightData.accel.z);

        if (abs(flightData.accel.x) > (G_THRESHOLD * FS_SENSITIVITY_8G)) {
          long launchDetection = 0;
          for(int counter=0; counter<10; counter++) {
            Wire.beginTransmission(0b1101000);
            Wire.write(0x3B);
            Wire.requestFrom(0b1101000, 2);
            while(Wire.available() < 2);
            launchDetection += ((Wire.read() << 8) | Wire.read()) - int16_t(bias.x);
            Wire.endTransmission();
          }
          launchDetection = launchDetection/10;
          if(launchDetection > (G_THRESHOLD * FS_SENSITIVITY_8G)) {
            Serial.println("Launch detected!");
            startTime = millis();
            state++;
            break;
          }
        } else if (abs(flightData.accel.y) > (G_THRESHOLD * FS_SENSITIVITY_8G)) {
          long launchDetection = 0;
          for(int counter=0; counter<10; counter++) {
            Wire.beginTransmission(0b1101000);
            Wire.write(0x3B);
            Wire.requestFrom(0b1101000, 2);
            while(Wire.available() < 2);
            launchDetection += ((Wire.read() << 8) | Wire.read()) - int16_t(bias.x);
            Wire.endTransmission();
          }
          launchDetection = launchDetection/10;
          if(launchDetection > (G_THRESHOLD * FS_SENSITIVITY_8G)) {
            Serial.println("Launch detected!");
            startTime = millis();
            state++;
            break;
          }
        } else if (abs(flightData.accel.z) > (G_THRESHOLD * FS_SENSITIVITY_8G)) {
          long launchDetection = 0;
          for(int counter=0; counter<10; counter++) {
            Wire.beginTransmission(0b1101000);
            Wire.write(0x3B);
            Wire.requestFrom(0b1101000, 2);
            while(Wire.available() < 2);
            launchDetection += ((Wire.read() << 8) | Wire.read()) - int16_t(bias.x);
            Wire.endTransmission();
          }
          launchDetection = launchDetection/10;
          if(launchDetection > (G_THRESHOLD * FS_SENSITIVITY_8G)) {
            Serial.println("Launch detected!");
            startTime = millis();
            state++;
            break;
          }
        }
        
        delay(10);
      }
      
    }
    case LOGGING: {
      for(int counter=0; counter<100; counter++) {
        Wire.beginTransmission(0b1101000);
        Wire.write(0x3B); // first accelerometer data register
        Wire.requestFrom(0b1101000, 6); // pull bytes from six accelerometer registers
        while (Wire.available() < 6);
        
        flightData.accel.x = ((Wire.read() << 8) | Wire.read()) - int16_t(bias.x);
        flightData.accel.y = ((Wire.read() << 8) | Wire.read()) - int16_t(bias.y);
        flightData.accel.z = ((Wire.read() << 8) | Wire.read()) - int16_t(bias.z);
        Wire.endTransmission();
  
        flightData.load = cell.get_units();
  
        dataFile = SD.open("data.txt", FILE_WRITE);
        if (dataFile) {
          dataFile.print(millis()); dataFile.print("\t");
          dataFile.print(flightData.accel.x); dataFile.print("\t");
          dataFile.print(flightData.accel.y); dataFile.print("\t");
          dataFile.print(flightData.accel.z); dataFile.print("\t");
          dataFile.println(flightData.load);
          dataFile.close();  
        }
      }
      state++;  
    }  
    case DONE: {
      while(true); //nothing
    }
  }
}
