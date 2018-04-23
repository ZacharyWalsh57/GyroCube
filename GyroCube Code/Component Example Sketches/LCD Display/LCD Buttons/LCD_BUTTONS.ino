#include <LiquidCrystal.h>
#include <Button.h>

#define PULLUP true
#define INVERT true
#define DB_MS 20

const int RS = 0, EN = 1, D4 = 14, D5 = 15, D6 = 16, D7 = 17;
const int UP_PIN = 4, DOWN_PIN = 7, LEFT_PIN = 8, RIGHT_PIN = 12, MIDDLE_PIN = 13;

LiquidCrystal LCD(RS, EN, D4, D5, D6, D7);

Button UP(UP_PIN, PULLUP, INVERT, DB_MS);
Button DOWN(DOWN_PIN, PULLUP, INVERT, DB_MS);
Button LEFT(LEFT_PIN, PULLUP, INVERT, DB_MS);
Button RIGHT(RIGHT_PIN, PULLUP, INVERT, DB_MS);
Button MIDDLE(MIDDLE_PIN, PULLUP, INVERT, DB_MS);
boolean BUTTON_STATE;

void setup()
{
    delay(2000);
    LCD.begin(20, 4);
    LCD.print(F("BUTTON PRESSED:"));
}

void loop()
{
    LCD.setCursor(0,1);
    UP.read();
    DOWN.read();
    LEFT.read();
    RIGHT.read();
    MIDDLE.read();

    if (UP.wasReleased())
    {
        BUTTON_STATE = !BUTTON_STATE;
        LCD.print(F("BUTTON UP!"));
    }
    if (DOWN.wasReleased())
    {
        BUTTON_STATE = !BUTTON_STATE;
        LCD.print(F("BUTTON DOWN!"));
    }
    if (LEFT.wasReleased())
    {
        BUTTON_STATE = !BUTTON_STATE;
        LCD.print(F("BUTTON LEFT!"));
    }
    if (RIGHT.wasReleased())
    {
        BUTTON_STATE = !BUTTON_STATE;
        LCD.print(F("BUTTON RIGHT!"));
    }
    if (MIDDLE.wasReleased())
    {
        BUTTON_STATE = !BUTTON_STATE;
        LCD.print(F("BUTTON MIDDLE!"));
    }
}
