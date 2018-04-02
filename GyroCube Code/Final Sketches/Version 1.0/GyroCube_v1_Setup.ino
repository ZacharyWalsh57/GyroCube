/* This is the code which we will be using to begin the whole process.
It includes libs, sets up pins for motors and LEDS, and sets up the MPU6050.
When that's done with, it moves into the void loop for the main program.
More info including a logic flow will come once the program is written.
*/

//Pre Setup Loop here:
//This is nothing more than making a whole bunch of Variables
//and assigning pins

#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Servo.h>

//Simple LED Pin assignments go here:
int LED_LEFT_RED = 11; //LEFT RED
int LED_LEFT_GRN = 10; //LEFT GREEN
int LED_RIGHT_RED = 9; //RIGHT RED
int LED_RIGHT_GRN = 6; //RIGHT GREEN

//Setup the motor pins here:
Servo LEFT_MOTOR;
Servo RIGHT_MOTOR;
int RIGHT_ESC = 5;
int LEFT_ESC = 3;
int MIN_PULSE = 1000;
int MAX_PULSE = 2000;

//MPU6050 Init Begins here:
//Basic precautions incase something we use is left out
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE //Make sure the Wire and I2Cdev are combined
#include "Wire.h"
#endif

//MPU6050 Object creation
MPU6050 MPU6050_CHIP;

//MPU Variables and Status Indications here we come! (Also end github dude's contrubution. You OG)
bool DMP_READY = false;  //Set to TRUE if the init went off right
uint8_t MPU_INT_STATUS;  //Holds our acrual interupt signal and status byte
uint8_t DEV_STATUS;      //Return status value for each packet/operation complete
uint8_t PACKET_SIZE;     //The EXPECTED DMP packet size (42 bytes according to datasheet)
uint8_t FIFO_COUNT;      //The old FIFO counter for bytes within the FIFO buffer itself
uint8_t FIFO_BUFFER[64]; //The actual FIFO Buffer which we use to move data from I2C to wire

Quaternion QUAT;          // [W , X, Y, Z] - Quaternion list/container
VectorInt16 ACCEL_ACTUAL; // [X, Y, Z] - The raw accel values read
VectorInt16 ACCEL_REAL;   // [X, Y, Z] - Gravity based accel values read
VectorInt16 ACCEL_WORLD;  // [X, Y, Z] - World based accel values read
VectorFloat GRAVITY;      // [X, Y, Z] - Gravity force on each reading
float EULER_LIST[3];      // [PSI, THETA, PHI] - Euler Angle list/container
float YAW_PITCH_ROLL[3];  // [YAW, PITCH, ROLL] - YPR list/container

//Interupt Detection HERE:
volatile bool MPU_INTERUPT = false; //Sets if MPU interupt pin is low or high currently
void DMP_DATA_READY()
{
    MPU_INTERUPT = true;
}

//-----------------------------------------------------------------------------------------------
//Setup Loop:
void setup()
{
    //LEDS Out of the way first:
    pinMode(LED_LEFT_RED, OUTPUT);  //LEFT RED PIN OUT
    pinMode(LED_LEFT_GRN, OUTPUT);  //LEFT GREEN PIN OUT
    pinMode(LED_RIGHT_RED, OUTPUT); //RIGHT RED PIN OUT
    pinMode(LED_RIGHT_GRN, OUTPUT); //RIGHT GREEN PIN OUT

    //Set warmup color for LEDS
    Serial.println("GYROCUBE V1.0 SETUP");
    Serial.println("LEDs ARE NOW YELLOW AND WILL REMAIN TILL SETUP IS DONE\n\n");
    LEDS_YELLOW();

    //Ensure I2C is properly included
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24;
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    //Serial Line time:
    Serial.begin(115200);
    while(!Serial);

    //Make our Gyro connections and boot it up
    Serial.println("CONNECTING TO MPU6050:");
    MPU6050_CHIP.initialize();
    Serial.println(F("Testing device connections..."));
    Serial.println(MPU6050_CHIP.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed")); 
    Serial.println("ATTEMPTING INIT OF DMP NOW....");
    DEV_STATUS = MPU6050_CHIP.dmpInitialize();

    //Offsets (WOO):
    MPU6050_CHIP.setXGyroOffset(114);
    MPU6050_CHIP.setYGyroOffset(-101);
    MPU6050_CHIP.setZGyroOffset(-3);
    MPU6050_CHIP.setZAccelOffset(1928);

    if (DEV_STATUS == 0)
    {
        //Set the DMP to ON since its working by the grace of god...
        Serial.println("DMP ALL GOOD, LETS BURN THIS BITCH....");
        MPU6050_CHIP.setDMPEnabled(true);

        Serial.println("CHECKING FOR INTERUP AND ENABLING NOW....");
        attachInterrupt(0, DMP_DATA_READY, RISING);
        MPU_INT_STATUS = MPU6050_CHIP.getIntStatus();

        DMP_READY = true; //ITS TOO LITTY

        //Dump our first packet and read the size of it.
        PACKET_SIZE = MPU6050_CHIP.dmpGetFIFOPacketSize();

        //Set the LEDS green once and then back to yellow
        LEDS_GREEN();
        //Delay so it looks like there's a blink
        delay(1500);
        //Back to yellow:
        LEDS_YELLOW();
    }

    else
    {
        //SHIT HIT THE FAN ABORT NOW
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(DEV_STATUS);
        Serial.println(F(")"));
        LEDS_RED();
    }

    Serial.println("MPU6050 CONNECTED!! LEDS SHOULD BLINK GREEN THEN SOLID YELLOW");
    Serial.println("IF THEY ARE NOT YELLOW, REBOOT AND CHECK CONNECTIONS\n\n");
    Serial.println("ARMING THE ESCs AND SPEEDING UP TO IDLE FOR 3 SECONDS");

    //Connect the ESC Pins to the Servos:
    LEFT_MOTOR.attach(LEFT_ESC, MIN_PULSE, MAX_PULSE);
    RIGHT_MOTOR.attach(RIGHT_ESC, MIN_PULSE, MAX_PULSE);
    
    //ARM THE ESCs
    LEFT_MOTOR.write(0);
    RIGHT_MOTOR.write(0);

    Serial.println("IF THE ESCs DID NOT MAKE THE 1356 NOISE, REBOOT!!!!");
    Serial.println("THE ESCs ARE NOW ARMED.  DO NOT DISCONNECT POWER");
    Serial.println("WAITING 3 SECONDS THEN REVVING TO IDLE FOR 3 SECONDS.");
    Serial.print("3.....");
    delay(1000);
    Serial.print("2.....");
    delay(1000);
    Serial.print("1.....");
    delay(1000);
    Serial.println("\nESC IDLE STARTING NOW");

    //Idle motor functions
    Serial.println("MOTORS SHOULD BE IDLED NOW. WAITING 5 SECONDS THEN STOPPING");
    IDLE_MOTORS(35,50,1);
    delay(5000);
    Serial.println("SPINNING DOWN NOW....\n\n");
    IDLE_MOTORS(0,50,1);

    Serial.println("AT THIS POINT ALL COMPONENTS ARE WORKING");
    LEDS_GREEN();
    Serial.println("WAITING 15 SECONDS THEN MOVING TO VOID LOOP");

    delay(15000);
}
//-----------------------------------------------------------------------------------
//Extra functions defined here for use in the setup script ONLY!!!
//Trying to call these in loop will not work!

void LEDS_YELLOW() 
{   //Set LEDS YELLOW
    analogWrite(LED_LEFT_RED, 0);
    analogWrite(LED_LEFT_GRN, 0);
    analogWrite(LED_RIGHT_RED, 0);
    analogWrite(LED_RIGHT_GRN, 0);
}

void LEDS_GREEN() 
{   //SET LETS GREEN
    analogWrite(LED_LEFT_RED, 255);
    analogWrite(LED_LEFT_GRN, 0);
    analogWrite(LED_RIGHT_RED, 255);
    analogWrite(LED_RIGHT_GRN, 0);
} 

void LEDS_RED() 
{   //SET LEDS RED
    analogWrite(LED_LEFT_RED, 0);
    analogWrite(LED_LEFT_GRN, 255);
    analogWrite(LED_RIGHT_RED, 0);
    analogWrite(LED_RIGHT_GRN, 255);
}

void IDLE_MOTORS(int IDLE_SPEED, int STEP_DELAY, int THROTTLE_STEP) 
{   //Set the motors to an idle speed.
    int CURRENT_THROTTLE = CHECK_THROTTLE();
    while (CURRENT_THROTTLE != IDLE_SPEED) 
    {
        if (CURRENT_THROTTLE < IDLE_SPEED)
        {
            LEFT_MOTOR.write(CURRENT_THROTTLE + THROTTLE_STEP);
            RIGHT_MOTOR.write(CURRENT_THROTTLE + THROTTLE_STEP);
            CURRENT_THROTTLE = CHECK_THROTTLE();
            delay(STEP_DELAY);
        }
        if (CURRENT_THROTTLE > IDLE_SPEED) 
        {
            LEFT_MOTOR.write(CURRENT_THROTTLE - THROTTLE_STEP);
            RIGHT_MOTOR.write(CURRENT_THROTTLE - THROTTLE_STEP);
            CURRENT_THROTTLE = CHECK_THROTTLE();
            delay(STEP_DELAY);
        }
    }
}

int CHECK_THROTTLE() 
{   //Read current throttle:
    int CURRENT_THROTTLE = LEFT_MOTOR.read();
    return CURRENT_THROTTLE;
}

//------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------

//Void Loop goes here where the PID control is added in.
void loop() {
    Serial.println("SETUP DONE");
}