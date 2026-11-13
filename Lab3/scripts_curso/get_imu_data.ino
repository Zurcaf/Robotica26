// sketch para adquirir sinais para o lab 3 de Robotica 2026
//
// the sensor returns  data in units mg, dps, uT


#include <I2Cdev.h>
#include <ICM20600.h>
#include <Wire.h>
#include <RunningMedian.h>

#include <Timer.h>

// Send T1357041600  through the terminal to start the clock in reasonable date/time

#define MAX_RECORDS 20000
#define MAX_REC_TIME_MILLIS 20000

#include <SPI.h>
#include <SD.h>

//Timer timer(MILLIS);
Timer timer(MICROS);

RunningMedian samples = RunningMedian(5);

ICM20600 icm20600(true);

float Z_filtered;
float X, Y, Z;
float wx, wx_filtered, wx_median, wy, wz;
char str1[30], str2[30], str3[30], str4[30], str5[30], str6[30], str123[50], str_time[30];
int counter = 0;

/*
// Check:  https://github.com/Seeed-Studio/Grove_6Axis_Accelerometer_And_Gyroscope_BMI088/blob/master/BMI088.h

enum acc_scale_type_t { // measurement rage
    RANGE_3G = 0x00, //
    RANGE_6G = 0x01, //
    RANGE_12G = 0x02, //
    RANGE_24G = 0x03, //
};

enum gyro_scale_type_t { // measurement rage
    RANGE_2000 = 0x00, //
    RANGE_1000 = 0x01, //
    RANGE_500 = 0x02, //
    RANGE_250 = 0x03, //
    RANGE_125 = 0x04, //
};
*/

File myFile;

void setup() {

  Wire.begin();
  Serial.begin(9600);
  icm20600.initialize();
    // icm20600.reset();


  icm20600.setAccScaleRange(0x00);
  //icm20600.setGyroRange(0x04);
  
  // Open serial communications and wait for port to open:
  // Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

/*
  Serial.print("Initializing SD card...");


  if (!SD.begin(4)) {
    Serial.println("failed!");
    while (1);
  }
  Serial.println("done.");

  if( SD.exists("test.txt") ) {
    Serial.println("file exists");
    if( SD.remove( "test.txt" ) ) {
      Serial.println("file deleted");
    }
    else {
      Serial.println("failed to delete file");
    }
  }

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  myFile = SD.open("test.txt", FILE_WRITE);
*/
  pinMode( LED_BUILTIN, OUTPUT );
  digitalWrite( LED_BUILTIN, LOW );

  Serial.println("Setup completed");
  Serial.println("-----------------------");
//  myFile.println("-----------------------");
 // myFile.close();

  // este delay é critico para o SD funcionar
  delay(1000);

  timer.start();
  if(timer.state() == RUNNING) Serial.println("timer running");
}

void loop() { 

  X = icm20600.getAccelerationX();
  Y = icm20600.getAccelerationY();
  Z = icm20600.getAccelerationZ();

  samples.add(Z);
  Z_filtered = samples.getMedian();
  
  wx = icm20600.getGyroscopeX();
  wy = icm20600.getGyroscopeY();
  wz = icm20600.getGyroscopeZ();

  dtostrf(X,8,1,str1);
  dtostrf(Y,8,1,str2);
  dtostrf(Z,8,1,str3);
  dtostrf(wx,8,1,str4);
  dtostrf(wy,8,1,str5);
  dtostrf(wz,8,1,str6);

  dtostrf(timer.read(), 8,1,str_time);

  strcpy(str123, str_time);
  strcat(str123, " ");
  
  strcat(str123, str1);
  strcat(str123, " ");
  strcat(str123, str2);
  strcat(str123, " ");
  strcat(str123, str3);
  strcat(str123, " ");
  strcat(str123, str4);
  strcat(str123, " ");
  strcat(str123, str5);
  strcat(str123, " ");
  strcat(str123, str6);

  Serial.println(str123);
  
/*
  if( counter<MAX_RECORDS && timer.read()<MAX_REC_TIME_MILLIS ) {
    if (myFile) {
      //Serial.print("Writing to test.txt...");
      myFile.println(str123);
      Serial.println(str123);
      
      delay(100);
    } 
    else {
      // if the file didn't open, print an error:
      Serial.println("error opening test.txt");
    }
    counter += 1;

    if( (counter % 100)==0 ) {
      Serial.println( counter );
    }
  }
  else if( counter==MAX_RECORDS || timer.read()>=MAX_REC_TIME_MILLIS ) {
    timer.stop();
    myFile.close();
    Serial.print("done, elapsed time: "); 
    Serial.println( timer.read() );
  
    counter += 1;

    // isto do led aqui nao funciona - nao percebo pq
    //Serial.println( "" ); // sem isto o led nao acende (???); delay, flush, etc nao resulta ...
                          // este println faz o clear do terminal
    //digitalWrite(LED_BUILTIN, HIGH );
  }
  else {
    Serial.println( "" ); // sem isto o led nao acende (???); delay, flush, etc nao resulta ...
                          // este println faz o clear do terminal
    digitalWrite(LED_BUILTIN, HIGH );
  }
  */
}
