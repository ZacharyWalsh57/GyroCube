#include <Wire.h>

int redLeft = 11;
int greenLeft = 10;
int redRight = 9;
int greenRight = 6;
int redBrightness = 0;
int greenBrightness = 0;

void setup() {
    Serial.begin(9600);
    pinMode(redLeft, OUTPUT);
    pinMode(greenLeft, OUTPUT);
    pinMode(redRight, OUTPUT);
    pinMode(greenRight, OUTPUT);
}

void loop() {
    Serial.println("::Enter Color Brightness::");
    
    Serial.print("RED  :  ");
    while(Serial.available()==0){};
    redBrightness = 225 - Serial.parseInt();

    Serial.print("GREEN  :  ");
    while(Serial.available()==0){}
    greenBrightness = 225 - Serial.parseInt();

    analogWrite(redLeft, redBrightness);
    analogWrite(greenLeft, greenBrightness);
    analogWrite(redRight, redBrightness);
    analogWrite(greenRight, greenBrightness);
    delay(3000);
}