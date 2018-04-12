// include the library code:
#include <LiquidCrystal.h>

// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = 0, en = 1, d4 = 14, d5 = 15, d6 = 16, d7 = 17;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

void setup()
{
    delay(2000);
    // set up the LCD's number of columns and rows:
    lcd.begin(20, 4);
    // Print a message to the LCD.
    lcd.print("LCD INIT WORKED! L1");
    lcd.setCursor(0,1);
    lcd.print("LCD INIT WORKED! L2");
    lcd.setCursor(0,2);
    lcd.print("LCD INIT WORKED! L3");
    lcd.setCursor(0,3);
    lcd.print("LCD INIT WORKED! L4");
    delay(2000);
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Time since last loop");
}

void loop()
{
    // set the cursor to column 0, line 1
    // (note: line 1 is the second row, since counting begins with 0):
    lcd.setCursor(0, 1);
    // print the number of seconds since reset:
    lcd.print(millis() / 1000);

}
