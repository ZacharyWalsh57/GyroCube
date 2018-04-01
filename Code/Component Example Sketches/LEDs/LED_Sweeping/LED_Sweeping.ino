/* First LED wiring program which controls the two BiColor R/G LEDS
on the cube frame on the front and back of the T Junution mount.

This is the sweeping script where the LEDs will sweep from red to green
to yellow to off and repeat.  Good for a warmup script while ESCs are warming and
the MPU6050 is calibrating. 

LOGIC FLOW::
-> STARTUP CODE
    -> Set LED Pins for OUTPUT 
        -> LED Right - Red=9, Green=6, Yellow=9+6
        -> LED Left - Red=11, Green=10, Yellow=11+10
    -> Setup Brightness For Left and Right LEDS for each color
        -> LEFT_RED=0
        -> LEFT_GRN=0
        -> RIGHT_RED=0
        -> RIGHT_GRN=0

-> Setup Loop
    -> Serial Conection Opening: 115200
    -> pinMode - LED_LEFT, LED_RIGHT

-> Void Run Loop
    -> Set the colors for both LEDs to 0 (no light if possible)
        -> TODO - Setup the LED Circuitry so that the lights are off.
        -> TODO - I guess just swapping the pins to not used would work too?
        -> FIXED - LED Power off is read as 255 on all four pins. So backwards??
    -> Setup four variables - LEFT_GRN, LEFT_RED, RIGHT_GRN, RIGHT_RED
        -> Make it so that the red is on for 3, then the green
           for 3 seconds, then both for 3 seconds.
        -> Stop, set pins to nothing, reset and run again.

*/

#include <Wire.h>

int LED_LEFT_RED = 11; //LEFT RED
int LED_LEFT_GRN = 10; //LEFT GREEN
int LED_RIGHT_RED = 9; //RIGHT RED
int LED_RIGHT_GRN = 6; //RIGHT GREEN

void setup() {
    Serial.begin(9600); //Establish new Serial Connection to host
    pinMode(LED_LEFT_RED, OUTPUT); //LEFT RED PIN OUT
    pinMode(LED_LEFT_GRN, OUTPUT); //LEFT GREEN PIN OUT
    pinMode(LED_RIGHT_RED, OUTPUT); //RIGHT RED PIN OUT
    pinMode(LED_RIGHT_GRN, OUTPUT); //RIGHT GREEN PIN OUT
}

void loop() {
    Serial.println("SETTING TO RED..."); //Set RED
    analogWrite(LED_LEFT_RED, 0);
    analogWrite(LED_LEFT_GRN, 255);
    analogWrite(LED_RIGHT_RED, 0);
    analogWrite(LED_RIGHT_GRN, 255);

    delay(3000); //Sleep to display color long enough for visible change

    Serial.println("SETTING TO GREEN..."); //Set GREEN
    analogWrite(LED_LEFT_RED, 255);
    analogWrite(LED_LEFT_GRN, 0);
    analogWrite(LED_RIGHT_RED, 255);
    analogWrite(LED_RIGHT_GRN, 0);

    delay(3000); //Check for color swap

    Serial.println("SETTING TO YELLOW..."); //Set YELLOW
    analogWrite(LED_LEFT_RED, 0);
    analogWrite(LED_LEFT_GRN, 0);
    analogWrite(LED_RIGHT_RED, 0);
    analogWrite(LED_RIGHT_GRN, 0);

    delay(3000); //Check for color swap

    Serial.println("ATTEMPTING POWER OFF..."); //TRY POWER OFF HERE
    analogWrite(LED_LEFT_RED, 255);
    analogWrite(LED_LEFT_GRN, 255);
    analogWrite(LED_RIGHT_RED, 255);
    analogWrite(LED_RIGHT_GRN, 255);

    delay(3000); //Check for power down now
}
