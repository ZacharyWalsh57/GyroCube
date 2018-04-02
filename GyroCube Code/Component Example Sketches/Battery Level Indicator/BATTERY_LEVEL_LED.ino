#include <Wire.h>

int BATT_LED_GREEN = 12;
int BATT_LED_RED = 4;

void setup() 
{
    pinMode(BATT_LED_GREEN, OUTPUT);
    pinMode(BATT_LED_RED, OUTPUT);
}

void loop() 
{
    //Turns on the resistive voltage for the green pin and off for the red one
    digitalWrite(BATT_LED_GREEN, LOW);
    digitalWrite(BATT_LED_RED, HIGH); 
    delay(2000);

    //Turns off the resistive voltage on the red pin and on for the green one    
    digitalWrite(BATT_LED_RED, LOW);
    digitalWrite(BATT_LED_GREEN, HIGH);
    delay(2000);
}