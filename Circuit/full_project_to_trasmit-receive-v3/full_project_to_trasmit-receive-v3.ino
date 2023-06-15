/*
* https://randomnerdtutorials.com/rf-433mhz-transmitter-receiver-module-with-arduino/
* Include RadioHead Library
* Include LiquidCrystal Library
*/
#include <RH_ASK.h> // Include RadioHead Library
#include <SPI.h> // Not actually used but needed to compile
#include <LiquidCrystal.h> // include LiquidCrystal Library for LCD 

//RH_ASK driver;
RH_ASK driver(2000, 4, 5); // TX pin5, RX pin 4
// RH_ASK driver(2000, 4, 5, 0); // ESP8266 or ESP32: do not use pin 11 or 2
// RH_ASK driver(2000, 3, 4, 0); // ATTiny, RX on D3 (pin 2 on attiny85) TX on D4 (pin 3 on attiny85), 
// RH_ASK driver(2000, PD14, PD13, 0); STM32F4 Discovery: see tx and rx on Orange and Red LEDS

// Creates an LCD object. Parameters: (rs, enable, d4, d5, d6, d7)

LiquidCrystal lcd(12, 11, 9, 8, 7, 6);
 
void setup()
{
    Serial.begin(9600);    // Debugging only
    if (!driver.init())
         Serial.println("init failed");
//lcd setup
// set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  // Clears the LCD screen
  lcd.clear();
  delay(1000);

}


void loop()
{
//Enable "trasmit" when you want to use module trasmit
// trasmit(); 
//Enable "receive"  when you want to use moduel receive
receive(); 

}

void trasmit()
{
    const char *msg = "Kastrinakis Manolis";
    driver.send((uint8_t *)msg, strlen(msg));
    driver.waitPacketSent();
    Serial.print("Message: "); //use only to debug 
    Serial.print(msg); 
    delay(1000);
}

void receive()
{
  //buffer size 16 char as LCD dispaly
    uint8_t buf[25];
    
   uint8_t buflen = sizeof(buf);
    if (driver.recv(buf, &buflen)) // Non-blocking
    {
      int i;
      // Message with a good checksum received, dump it.
      Serial.print("Message: ");
      Serial.println((char*)buf); 
    lcd.setCursor(0,0);
   lcd.print((char*)buf);    
    // LCD_Message() ;
  //    lcd.clear();
  delay(500);

    }
}
void LCD_Message(){
    uint8_t buf[20];
    uint8_t buflen = sizeof(buf);
   // Print a message to the LCD.
  lcd.setCursor(0, 0);
  // lcd.print((char*)buf);
  lcd.print(" Hello world!");

  // set the cursor to column 0, line 1
  // (note: line 1 is the second row, since counting begins with 0):
  lcd.setCursor(0, 1);
  // Print a message to the LCD.
  lcd.print(" LCD Tutorial");
    delay(5000);
   lcd.clear();
}
