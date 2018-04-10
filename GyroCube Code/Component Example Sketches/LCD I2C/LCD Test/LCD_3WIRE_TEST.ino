#include <LiquidCrystal_SR_LCD3.h>

const int PIN_LCD_STROBE = 14;   // Out: LCD IC4094 shift-register strobe
const int PIN_LCD_DATA = 15;     // Out: LCD IC4094 shift-register data
const int PIN_LCD_CLOCK = 16;    // Out: LCD IC4094 shift-register clock

LiquidCrystal_SR_LCD3 LCD(PIN_LCD_DATA, PIN_LCD_CLOCK, PIN_LCD_STROBE);
String String_Print = ""; 

void setup()
{
    Serial.begin(9600);
    LCD.begin(20,4);
    LCD.home();
    delay(2000);
    Serial.println("SERIAL OPEN GOING TO MAIN NOW");
}

void loop()
{
    LCD.home();
    LCD.clear();
    delay(2000);
    String_Print = String("LCD TEST");
    LCD.print(String_Print.c_str());
    delay(2000);
}