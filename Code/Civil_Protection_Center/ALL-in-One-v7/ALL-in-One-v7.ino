/*********************************/
/*Gyro
/* https://lastminuteengineers.com/mpu6050-accel-gyro-arduino-tutorial/
/* Include Librarys : Adafruit MPU6050
/*  Adafruit Unified Sensor Driver and 
/ *   
/*Adafruit Bus IO Library 
/* Adafruit Bus IO
/*
/* LCD
/* https://lastminuteengineers.com/i2c-lcd-arduino-tutorial/
/*  LiquidCrystalI2C library by Frank de Brabander.
 *   
* Gas Sensor
* https://circuitdigest.com/microcontroller-projects/interfacing-mq2-gas-sensor-with-arduino
*  https://lastminuteengineers.com/mq2-gas-senser-arduino-tutorial/
*
 * Rain Sensor
 * https://lastminuteengineers.com/rain-sensor-arduino-tutorial/
 * 
 *  temperature and humidity sensor
 * https://lastminuteengineers.com/dht11-module-arduino-tutorial/
 * include dhtlib library
 * 
 * Flame Sensor
 * https://projecthub.arduino.cc/SURYATEJA/arduino-modules-flame-sensor-e48e97
 * 
 * 
 * Relay 4 module
 * https://lastminuteengineers.com/two-channel-relay-module-arduino-tutorial/
 * 
 * buzzer module
 * https://arduinointro.com/projects/adding-sounds-to-arduino-using-the-mh-fmd-piezo-buzzer-module
 * https://www.circuitbasics.com/how-to-use-active-and-passive-buzzers-on-the-arduino/
 * 
 * Tsounami module
 * https://arduinogetstarted.com/tutorials/arduino-switch
 * ezButton library
 * 
 * Interupts
 * https://reference.arduino.cc/reference/tr/language/functions/external-interrupts/attachinterrupt/
 * 
*/
/*
 * Libary's to include
 * Adafruit MPU6050
 * Adafruit Unified Sensor Driver
 * Adafruit Bus IO
 * LiquidCrystalI2C  library by Frank de Brabander
 * dhtlib
 * ezButton
 * 
 */
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
//LCD Setup
LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 20 chars and 4 line display
//Gyro Sensor
Adafruit_MPU6050 mpu;
//Rain Sensor
#define RainSensorPin 6 //Rain Sensor Analog Port 6 
//Gas Sensor
#define Threshold 400
#define sensorPower 7 //GAS Sensor digital port 7
#define MQ2pin 7 //GAS Sensor analog port 7
float sensorValue;  //variable to store sensor value
//  temperature and humidity sensor
#include <dht.h>        // Include library
#define outPin 6        // temperature Sensor Digital Port 6  
dht DHT;                // Creates a DHT object
// Flame Sensor
// lowest and highest sensor readings:
const int sensorMin = 0;     //  sensor minimum
const int sensorMax = 1024;  // sensor maximum
//Relay Module
int RelayPin1 = 39; //Relay Digital Pin 39
int RelayPin2 = 37; //Relay Digital Pin 37
int RelayPin3 = 35; //Relay Digital Pin 35
int RelayPin4 = 33; //Relay Digital Pin 33
//Buzzer Module
#define buzzer 8 // Buzzer Digital port 8
//int buzzer = 8;
//Tsounami Module

//#define TsounamiPin 2
#include <ezButton.h>
//const byte interruptPin = 2;
const byte interruptPin2 = 2 ;  //Interrupt Port Push Button
//ezButton TsounamiPin(interruptPin);
 
int TsounamiPin = 3; //Tsounami Sensor Digital Port 3
int TsounamiStatus = 0;
//Earthqueke
int AccelSpeed =0;
int AlarmStatus =0;
/*
 * Interupt Pin 2
 */
/*
 * Flame 
 */
 int FlameReading0  = 0; 
 int FlameReading1  = 0;
 int FlameReading2  = 0;
 int FlameReading3  = 0;
 int FlameReading4  = 0; 
 char FireDiraction0[10] = "WEST";
 char FireDiraction1[10] = "NORTH-WEST";
 char FireDiraction2[10] = "NORTH";
 char FireDiraction3[10] = "NORTH-EAST";
 char FireDiraction4[10] = "EAST";
/*
 * *******************************************
 * 
 * *******************************************
 */
void setup() {
 InterruptSetup();
 Serial.begin(9600);
 GasSensorSetup;
 GyroSetup();
 RelaySetup();
 // delay(500);
 LCD_i2c_Setup();
 BuzzerSetup();
 TsounamiSetup();
 RainSetup();
delay(2000);
//lcd.clear();
//digitalWrite(buzzer, LOW);
}



/*
 * ------------------------------------------
 * Setup Functions
 * --------------------------------------------
 */

/*
  * Interupt setup
  * 
  */
  void InterruptSetup(){
     pinMode(interruptPin2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin2), ChangeAlarmStatus, LOW);
  }
void ChangeAlarmStatus(){
  
  AlarmStatus =0;
  //Serial.println("Älarm off");
  
}
  
 /*
  * Gyro Sensor Setup
  */
  void GyroSetup(){
   // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      //delay(10);
      delayMicroseconds(100);
    }
  }
  Serial.println("MPU6050 Found!");

  // set accelerometer range to +-8G
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  // set gyro range to +- 500 deg/s
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  // set filter bandwidth to 21 Hz
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  //delay(100);
  
}
/*
 * LCD I2C Sutup
 */
void LCD_i2c_Setup(){
  lcd.init();
  lcd.clear();         
  lcd.backlight();      // Make sure backlight is on
  
  // Print a message on both lines of the LCD.
  lcd.setCursor(0,0);   //Set cursor to character 2 on line 0
  lcd.print("2o Elementary School!");
  lcd.setCursor(3,1);   //Move cursor to character 2 on line 1
  lcd.print("Chania Crete");
  lcd.setCursor(2,2);   //Move cursor to character 2 on line 1
  lcd.print("Open ELLAK 2023");
  lcd.setCursor(0,3);   //Move cursor to character 2 on line 1
  lcd.print("The little Rescuers!");
     // lcd.setCursor(0,3);   //Move cursor to character 2 on line 1
  //lcd.print("");

//delay(1000);
//lcd.clear();    
}

void FlameSetup(){
pinMode(A0, INPUT);
pinMode(A1, INPUT);
pinMode(A2, INPUT);
pinMode(A3, INPUT);
pinMode(A4, INPUT);

}
void RainSetup(){
  pinMode(RainSensorPin, INPUT);
}
/*
 * Relay Setup
 */
void RelaySetup() {
  // Set RelayPin as an output pin
  pinMode(RelayPin1, OUTPUT);
  pinMode(RelayPin2, OUTPUT);
  pinMode(RelayPin3, OUTPUT);
  pinMode(RelayPin4, OUTPUT);
}

/*
 * Buzzer Setup
 */
void BuzzerSetup(){
  pinMode(buzzer, OUTPUT);
 //digitalWrite(buzzer, HIGH);
 //analogWrite(buzzer,0);
  tone(buzzer, 1000,10); // Send 1KHz sound signal...
   noTone(buzzer);
 // noTone(buzzer);
}
/*
 * Tsounami Setup
 */
 
void TsounamiSetup(){
 // pinMode(TsounamiPin, INPUT);           // set pin to input
 //pinMode(interruptPin2, INPUT_PULLUP);
 pinMode(TsounamiPin, INPUT_PULLUP);
//  digitalWrite(Tsounamipin, HIGH);
//TsounamiPin.setDebounceTime(50);
}
/*
 * Gass Setup
 */
 void GasSensorSetup(){
   // pinMode(sensorPower, OUTPUT);
  // Initially keep the sensor OFF
  //digitalWrite(sensorPower, LOW);
}



/*
 * ----------------------------------------
 * Main Program
 * ----------------------------------------
 */
/*
 * main Loop
 */
void loop(){
 // Serial.print("interupt =");
 //Serial.println(digitalRead(interruptPin2));
 if (AlarmStatus ==0 ){
  LCD_i2c_GeneralMessage();
  RelayGeneralStatus();
 }
  TsounamiStatus = digitalRead(TsounamiPin); //read Tsounami sensor
  if  (TsounamiStatus == 0){
    AlarmStatus =1;
    TsounamiAlarm();
  }
  // Read Flame Sensorss
 FlameReading0  = analogRead(A0);
 FlameReading1  = analogRead(A1);
 FlameReading2  = analogRead(A2);
 FlameReading3  = analogRead(A3);
 FlameReading4  = analogRead(A4); 

 if ((FlameReading0 > 300 )  || (FlameReading1 > 300 )||
     (FlameReading1 > 300 )  || (FlameReading2 > 300 )|| 
     (FlameReading3 > 300 )  || (FlameReading4 > 300 )){
  AlarmStatus =1;
   digitalWrite(RelayPin3, HIGH);
  LCDFireArarm();
    }

    //Earthquake
    sensors_event_t a, g, temp;
     mpu.getEvent(&a, &g, &temp);
    AccelSpeed = abs(a.acceleration.x + a.acceleration.y); //check Earthqueke
   // Serial.println (AccelSpeed);
if ( AccelSpeed >7){
  AlarmStatus =1;
  digitalWrite(RelayPin1, HIGH);
  digitalWrite(RelayPin2, HIGH);
  digitalWrite(RelayPin3, HIGH);
  digitalWrite(RelayPin4, HIGH);
  LCDEarthquake();
}

 //Rain
   int RainVal = analogRead(RainSensorPin); ;
   if (RainVal < 100){
    AlarmStatus =1;
    LCDRain();
   }
  //Temperature
   int readData = DHT.read11(outPin);
   float t = DHT.temperature; 
  if (t <20){
    LCDRain();
  }
//GAS
sensorValue = analogRead(MQ2pin);
if(sensorValue > Threshold)
  {
      AlarmStatus =1; 
        digitalWrite(RelayPin1, HIGH);
    digitalWrite(RelayPin2, HIGH);
    digitalWrite(RelayPin3, HIGH);
      LCDGAS();
   // Serial.print(" | Smoke detected!");
   
  }

}


/*
 * ----------------------------------------
 * Functions
 * ----------------------------------------
 */
 void LCDGAS(){
     lcd.init();
 // lcd.clear();         
  //lcd.backlight();      // Make sure backlight is on
  
  // Print a message on both lines of the LCD.
  lcd.setCursor(1,0);   //Set cursor to character 2 on line 0
  lcd.print("--- Attetion ---");
  lcd.setCursor(3,1);   //Move cursor to character 2 on line 1
  lcd.print("GAS Leak");
  lcd.setCursor(3,2);   //Move cursor to character 2 on line 1
  lcd.print("");
  lcd.setCursor(3,3);   //Move cursor to character 2 on line 1
  lcd.print("");

     // lcd.setCursor(0,3);   //Move cursor to character 2 on line 1
  //lcd.print("");
 }
 void LCDRain(){
    lcd.init();
 // lcd.clear();         
  //lcd.backlight();      // Make sure backlight is on
  
  // Print a message on both lines of the LCD.
  lcd.setCursor(1,0);   //Set cursor to character 2 on line 0
  lcd.print("--- Attetion ---");
  lcd.setCursor(3,1);   //Move cursor to character 2 on line 1
  lcd.print("Please Procced to");
  lcd.setCursor(3,2);   //Move cursor to character 2 on line 1
  lcd.print("near station to  ");
  lcd.setCursor(3,3);   //Move cursor to character 2 on line 1
  lcd.print("received tents");
     // lcd.setCursor(0,3);   //Move cursor to character 2 on line 1
  //lcd.print("");
 }
 void LCDEarthquake(){
  lcd.init();
 // lcd.clear();         
  //lcd.backlight();      // Make sure backlight is on
  tone(buzzer, 1000, 500);
  // Print a message on both lines of the LCD.
  lcd.setCursor(1,0);   //Set cursor to character 2 on line 0
  lcd.print("--- Attetion ---");
  lcd.setCursor(3,1);   //Move cursor to character 2 on line 1
  lcd.print("Earthquake");
  lcd.setCursor(3,2);   //Move cursor to character 2 on line 1
  lcd.print(" in progress");
  lcd.setCursor(3,3);   //Move cursor to character 2 on line 1
  lcd.print("");
     // lcd.setCursor(0,3);   //Move cursor to character 2 on line 1
  //lcd.print("");
 }
  void LCDFireArarm(){
   
    // LCD Message
   lcd.init();
 lcd.clear();         
  //lcd.backlight();      // Make sure backlight is on
   // Print a message on both lines of the LCD.
  lcd.setCursor(3,0);   //Set cursor to character 2 on line 0
  lcd.print("-- Attetion --");
  lcd.setCursor(4,1);   //Move cursor to character 2 on line 1
  lcd.print("Fire ALarm");
  lcd.setCursor(4,2);   //Move cursor to character 2 on line 1
  lcd.print("Go way form");
  //lcd.setCursor(4,3);   //Move cursor to character 2 on line 1
 // lcd.print(FireDiraction);
if (FlameReading0 > 300 ){
  lcd.setCursor(7,3);
  lcd.print(FireDiraction0);
  }
if (FlameReading1 > 300 ){
  lcd.setCursor(6,3);
  lcd.print(FireDiraction1);
  }    
  if (FlameReading2 > 300 ){
    lcd.setCursor(4,3);
  lcd.print(FireDiraction2);
  }
  if (FlameReading3 > 300 ){
    lcd.setCursor(4,3);
  lcd.print(FireDiraction3);
  }
 
 if (FlameReading4 > 300 ){
  lcd.setCursor(4,3);
  lcd.print(FireDiraction4);
  }
  
  }
 /*
 * LCD I2C General Message
 */
void LCD_i2c_GeneralMessage(){
  lcd.init();
 // lcd.clear();         
//  lcd.backlight();      // Make sure backlight is on
  
  // Print a message on both lines of the LCD.
  lcd.setCursor(1,0);   //Set cursor to character 2 on line 0
  lcd.print("Go to the nearest");
  lcd.setCursor(3,1);   //Move cursor to character 2 on line 1
  lcd.print("assembly point");
  lcd.setCursor(3,2);   //Move cursor to character 2 on line 1
  lcd.print("You are near");
  lcd.setCursor(3,3);   //Move cursor to character 2 on line 1
  lcd.print("-- West Wall --");
     // lcd.setCursor(0,3);   //Move cursor to character 2 on line 1
  //lcd.print("");

//delay(2000);
//lcd.clear();    
}
 
void RelayGeneralStatus(){
  digitalWrite(RelayPin1, LOW);
  //delay(500);
  digitalWrite(RelayPin2, LOW);
  //delay(500);
  digitalWrite(RelayPin3, LOW);
  //delay(500);
 digitalWrite(RelayPin4, LOW);
}

 /*
  * Tsounami Alarm
  */

  void TsounamiAlarm(){
   
 // LCD Message
  digitalWrite(RelayPin1, HIGH);
  //digitalWrite(buzzer, HIGH);
  tone(buzzer, 1000, 500);
 //delay(1000);
//digitalWrite(buzzer, LOW);
LCDTsounamiArarm();
TsounamiStatus = 1;

 Serial.println("Tsounami Alarm");
       
  }

  void LCDTsounamiArarm(){
   
    // LCD Message
   lcd.init();
 lcd.clear();         
  //lcd.backlight();      // Make sure backlight is on
   // Print a message on both lines of the LCD.
  lcd.setCursor(3,0);   //Set cursor to character 2 on line 0
  lcd.print("-- Attetion --");
  lcd.setCursor(4,1);   //Move cursor to character 2 on line 1
  lcd.print("Tsounami ALarm");
  lcd.setCursor(7,2);   //Move cursor to character 2 on line 1
  lcd.print("Go to");
  lcd.setCursor(4,3);   //Move cursor to character 2 on line 1
  lcd.print("Higher Groud");
  //
  
  }
  
