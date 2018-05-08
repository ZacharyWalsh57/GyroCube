#include <Wire.h>
#include <Servo.h>
#include <Button.h>
#include <LiquidCrystal.h>
#include <LiquidMenu.h>
#include "quaternionFilters.h"
#include "MPU9250.h"

//LEDs:
#define LED_LEFT_RED 10
#define LED_LEFT_GREEN 11
#define LED_RIGHT_RED 6
#define LED_RIGHT_GREEN 9

//Motors:
Servo LEFT_MOTOR;
Servo RIGHT_MOTOR;
#define LEFT_MOTOR_PIN 3
#define RIGHT_MOTOR_PIN 5

//LCD - Screen
//RED YELLOW GREEN BLK
#define RS 0
#define EN 1
#define D4 14
#define D5 15
#define D6 16
#define D7 17

//LCD Creation:
LiquidCrystal GYROCUBE_LCD(RS,EN,D4,D5,D6,D7);

//LCD - Menu Text in PROGMEM:
const char BOOT_WELCOME_CHARS[] PROGMEM = "GyroCube 3.0";
const char BOOT_ZACK_CHARS[] PROGMEM = "Zack Walsh";
const char BOOT_BRYCE_CHARS[] PROGMEM = "Bryce Hadden";
const char MAIN_TITLE_CHARS[] PROGMEM = "GyroCube 3.0";
const char MAIN_RUN_PID_CHARS[] PROGMEM = "Press Any Button";
const char MONITOR_TITLE_CHARS[] PROGMEM = "Real-time Values";

LiquidLine BOOT_WELCOME(4,0,BOOT_WELCOME_CHARS);
LiquidLine BOOT_ZACK(5,2,BOOT_ZACK_CHARS);
LiquidLine BOOT_BRYCE(4,3,BOOT_BRYCE_CHARS);
LiquidLine MAIN_TITLE(4,0,MAIN_TITLE_CHARS);
LiquidLine MAIN_RUN_PID(2,2,MAIN_RUN_PID_CHARS);
LiquidLine MONITOR_TITLE(2,0,MONITOR_TITLE_CHARS);

//LCD - Screens:
LiquidScreen BOOT_SCREEN(BOOT_WELCOME, BOOT_ZACK, BOOT_BRYCE);
LiquidScreen MAIN_SCREEN(MAIN_TITLE, MAIN_RUN_PID);
LiquidScreen MONITOR_SCREEN(MONITOR_TITLE);

//LCD - Menus:
LiquidMenu BOOT_MENU(GYROCUBE_LCD, BOOT_SCREEN);
LiquidMenu MAIN_MENU(GYROCUBE_LCD, MAIN_SCREEN);
LiquidMenu MONITOR_MENU(GYROCUBE_LCD, MONITOR_SCREEN);

//LCD - System Menu:
LiquidSystem GYROCUBE_SYSTEM(BOOT_MENU, MAIN_MENU, MONITOR_MENU, 1);

//LCD - Button Pins
#define PULLUP true
#define INVERT true
#define DEBOUNCE 20
#define UP_PIN 4
#define DOWN_PIN 7
#define LEFT_PIN 8
#define RIGHT_PIN 12
#define MIDDLE_PIN 13

//LCD - Button Construction
Button UP(UP_PIN,PULLUP,INVERT,DEBOUNCE);
Button DOWN(DOWN_PIN,PULLUP,INVERT,DEBOUNCE);
Button LEFT(LEFT_PIN,PULLUP,INVERT,DEBOUNCE);
Button RIGHT(RIGHT_PIN,PULLUP,INVERT,DEBOUNCE);
Button MIDDLE(MIDDLE_PIN,PULLUP,INVERT,DEBOUNCE);

//MPU9255
#define INT_PIN 2
MPU9250 MPU_GYRO;
double ROLL_ANGLE;

//PID Constants:
unsigned long TIME_LAST;
double INPUT_VALUE, OUTPUT_VALUE, ERROR;
double SETPOINT = 45.0;
double I_TERM, LAST_INPUT;
double KP, KI, KD;
double OUT_MIN, OUT_MAX;
bool RUN_ALGO = true;

//---------------------------------------------------------------------------------------
//HELPER FUNCTIONS: BUTTONS, LCD, MPU, LEDS, PID, MENU ETC

//Button Sweeper for the void loop.  One run, the rest is automatic
void CHECK_BUTTONS() {
    UP.read(); 
    DOWN.read();
    LEFT.read();
    RIGHT.read();
    MIDDLE.read();
    if (UP.wasReleased())
    {
        GYROCUBE_LCD.clear();
        GYROCUBE_LCD.setCursor(5,0);
        GYROCUBE_LCD.print("SETTING UP");
        delay(500);
        INIT_PARTS();
    }
    if (DOWN.wasReleased())
    {
        GYROCUBE_LCD.clear();
        GYROCUBE_LCD.setCursor(5,0);
        GYROCUBE_LCD.print("SETTING UP");
        delay(500);
        INIT_PARTS();
    }
    if (LEFT.wasReleased())
    {
        GYROCUBE_LCD.clear();
        GYROCUBE_LCD.setCursor(5,0);
        GYROCUBE_LCD.print("SETTING UP");
        delay(500);
        INIT_PARTS();
    }
    if (RIGHT.wasReleased())
    {
        GYROCUBE_LCD.clear();
        GYROCUBE_LCD.setCursor(5,0);
        GYROCUBE_LCD.print("SETTING UP");
        delay(500);
        INIT_PARTS();
    }
    if (MIDDLE.wasReleased())
    {
        GYROCUBE_LCD.clear();
        GYROCUBE_LCD.setCursor(5,0);
        GYROCUBE_LCD.print("SETTING UP");
        delay(500);
        INIT_PARTS();
    }
}

//LED Functions to reduce code size.  RED, YELLOW, GREEN are defined
//as LEDS_*color*.  Cycle LEDS is for flashing through RGY. LEDS_OFF 
//just turns them off.  Used to make a flashing LED.
void LEDS_RED() {
    analogWrite(LED_LEFT_RED, 0);
    analogWrite(LED_LEFT_GREEN, 255);
    analogWrite(LED_RIGHT_RED, 0);
    analogWrite(LED_RIGHT_GREEN, 255);
}
void LEDS_GREEN() {
    analogWrite(LED_LEFT_RED, 255);
    analogWrite(LED_LEFT_GREEN, 0);
    analogWrite(LED_RIGHT_RED, 255);
    analogWrite(LED_RIGHT_GREEN, 0);
}
void LEDS_YELLOW() {
    analogWrite(LED_LEFT_RED, 0);
    analogWrite(LED_LEFT_GREEN, 0);
    analogWrite(LED_RIGHT_RED, 0);
    analogWrite(LED_RIGHT_GREEN, 0);
}
void CYCLE_LEDS(int DELAY) {
    LEDS_RED();
    delay(DELAY);
    LEDS_GREEN();
    delay(DELAY);
    LEDS_YELLOW();
}
void LEDS_OFF() {
    analogWrite(LED_LEFT_RED, 255);
    analogWrite(LED_LEFT_GREEN, 255);
    analogWrite(LED_RIGHT_RED, 255);
    analogWrite(LED_RIGHT_GREEN, 255);
}

//Motor throttle changes and ESC Arming. 
//NOTE: THIS SPEED SET IS ONLY FOR THE SETUP:
//Arming ESC functions
void ESC_ARMING() {
    //Arm with old stuff.  PID with the new.
    SET_SPEED(1000);
    delay(1000);
    LEDS_GREEN();
    delay(250);
    CYCLE_LEDS(375);
    LEDS_YELLOW();
    SET_SPEED(1450);
    delay(2000);
    SET_SPEED(1000);
    delay(1500);
    CYCLE_LEDS(275);
    LEDS_GREEN();   
}
void SET_SPEED(int MICROS) {
    LEFT_MOTOR.writeMicroseconds(MICROS);
    RIGHT_MOTOR.writeMicroseconds(MICROS);
}

//MPU92655 Init functions and updating RTVs when running.
void MPU9255_INIT() {
    //Setup code here I guess
    MPU_GYRO.calibrateMPU9250(MPU_GYRO.gyroBias, MPU_GYRO.accelBias);
    MPU_GYRO.initMPU9250();
    MPU_GYRO.initAK8963(MPU_GYRO.factoryMagCalibration);

    MPU_GYRO.getAres();
    MPU_GYRO.getGres();
    MPU_GYRO.getMres();

    if (MPU_GYRO.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {
        CYCLE_LEDS(750);
        digitalWrite(INT_PIN, LOW);
    }
    else {
        GYROCUBE_LCD.clear();
        GYROCUBE_LCD.setCursor(0,0);
        GYROCUBE_LCD.print("MPU INIT FAILED! KILLING");
        LEDS_RED();
        abort();
    }
}
void MPU9255_UPDATE() {
    if (MPU_GYRO.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {
        
        MPU_GYRO.readAccelData(MPU_GYRO.accelCount);  // Read the x/y/z adc values
        MPU_GYRO.ax = (float)MPU_GYRO.accelCount[0] * MPU_GYRO.aRes;
        MPU_GYRO.ay = (float)MPU_GYRO.accelCount[1] * MPU_GYRO.aRes;
        MPU_GYRO.az = (float)MPU_GYRO.accelCount[2] * MPU_GYRO.aRes;
        
        MPU_GYRO.readGyroData(MPU_GYRO.gyroCount);  // Read the x/y/z adc values
        MPU_GYRO.gx = (float)MPU_GYRO.gyroCount[0] * MPU_GYRO.gRes;
        MPU_GYRO.gy = (float)MPU_GYRO.gyroCount[1] * MPU_GYRO.gRes;
        MPU_GYRO.gz = (float)MPU_GYRO.gyroCount[2] * MPU_GYRO.gRes;
        
        MPU_GYRO.readMagData(MPU_GYRO.magCount);  // Read the x/y/z adc values
        MPU_GYRO.mx = (float)MPU_GYRO.magCount[0] * MPU_GYRO.mRes
               * MPU_GYRO.factoryMagCalibration[0] - MPU_GYRO.magBias[0];
        MPU_GYRO.my = (float)MPU_GYRO.magCount[1] * MPU_GYRO.mRes
               * MPU_GYRO.factoryMagCalibration[1] - MPU_GYRO.magBias[1];
        MPU_GYRO.mz = (float)MPU_GYRO.magCount[2] * MPU_GYRO.mRes
               * MPU_GYRO.factoryMagCalibration[2] - MPU_GYRO.magBias[2];
               
        MPU_GYRO.updateTime();
        
        MahonyQuaternionUpdate(MPU_GYRO.ax, MPU_GYRO.ay, MPU_GYRO.az, MPU_GYRO.gx * DEG_TO_RAD,
                         MPU_GYRO.gy * DEG_TO_RAD, MPU_GYRO.gz * DEG_TO_RAD, MPU_GYRO.my,
                         MPU_GYRO.mx, MPU_GYRO.mz, MPU_GYRO.deltat);
        
        MPU_GYRO.delt_t = millis() - MPU_GYRO.count;

        MPU_GYRO.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2)
                    * *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1)
                    * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) + *(getQ()+3)
                    * *(getQ()+3));
        MPU_GYRO.roll *= RAD_TO_DEG;
        ROLL_ANGLE = abs(MPU_GYRO.roll);
        //Serial.println(ROLL_ANGLE);
    }
}

//Finally make sure all the parts are working right in one big init function.
void INIT_PARTS() {
    CYCLE_LEDS(750);
    STRING_ON_LCD(0,1,"LEDS SETUP",false);
    //GYROCUBE_LCD.setCursor(0, 1);
    //GYROCUBE_LCD.print("LEDS SETUP");
    
    MPU9255_INIT();
    MPU9255_UPDATE();
    //GYROCUBE_LCD.setCursor(0, 2);
    //GYROCUBE_LCD.print("MPU9255 DMP OK");
    STRING_ON_LCD(0,2,"MPU9255 DMP OK",false);

    ESC_ARMING();
    //GYROCUBE_LCD.setCursor(0, 3);
    //GYROCUBE_LCD.print("ESCs ARMED");
    STRING_ON_LCD(0,3,"ESCs ARMED",false);
    delay(750);
    
    GYROCUBE_LCD.clear();
    CYCLE_LEDS(250);
    PID_ALGO();
    //Then finally, throw this bitch on autopilot lol
}

//PID FUNCTIONS: These functions are pretty fuckin dope if you ask me. Credit to Brett Beauregard for some basic help/explaining. (Spelling lol)
void PID_ALGO() {
    CYCLE_LEDS(750);
    SET_OUTPUT_LIMITS(2000, 1250);
    SET_TUNINGS(5.43, 0.42, 4.43);
    CYCLE_LEDS(1000);
    delay(500);
    SET_SPEED(1100);
    
    GYROCUBE_SYSTEM.change_menu(MONITOR_MENU);
    STRING_ON_LCD(0,1,"Kp",false);
    STRING_ON_LCD(9,1,"SP",false);

    STRING_ON_LCD(0,2,"Ki",false);
    STRING_ON_LCD(9,2,"RT",false);

    STRING_ON_LCD(0,3,"Kd",false);
    STRING_ON_LCD(9,3,"EV",false);
    
    delay(2000);
    MPU9255_UPDATE();
    while(RUN_ALGO) {
        PID_COMPUTE();
    }
}
void SET_OUTPUT_LIMITS(double MAX, double MIN) {
    if(MIN > MAX)
        return;
    OUT_MAX = MAX;
    OUT_MIN = MIN;

    if(OUTPUT_VALUE > OUT_MAX)
        OUTPUT_VALUE = OUT_MAX;
    else if(OUTPUT_VALUE < OUT_MIN)
        OUTPUT_VALUE = OUT_MIN;

    if(I_TERM > OUT_MAX) 
        I_TERM = OUT_MAX;
    else if(I_TERM < OUT_MIN)
        I_TERM = OUT_MIN;
}
void SET_TUNINGS(double SKP, double SKI, double SKD) {
    if (SKP < 0 || SKI < 0 || SKD < 0) {
        return;
    }
    KP = SKP;
    KI = SKI;
    KD = SKD;
}
void FETCH_NEW_DATA() {
    //Fetch the latest angle values
    MPU9255_UPDATE();
    INPUT_VALUE = ROLL_ANGLE;
}
void PID_COMPUTE() {
    CHECK_HALT();
    FETCH_NEW_DATA();
    //PID computation in here:
    unsigned long NOW = millis();
    int ELAPSED = (NOW - TIME_LAST);

    //Make sure its time for new samples.
    //Compute Working Variables now:
    ERROR = SETPOINT - INPUT_VALUE;
    I_TERM += (KI * ERROR);
    if(I_TERM > OUT_MAX)
        I_TERM = OUT_MAX;
    else if(I_TERM < OUT_MIN)
        I_TERM = OUT_MIN;
    double D_INPUT = (INPUT_VALUE - LAST_INPUT);

    //Convert this over to PID Components here:
    OUTPUT_VALUE = KP * ERROR + I_TERM - KD * D_INPUT;
    if (OUTPUT_VALUE > OUT_MAX)
        OUTPUT_VALUE = OUT_MAX;
    else if (OUTPUT_VALUE < OUT_MIN)
        OUTPUT_VALUE = OUT_MIN;

    //Take note of some important values:
    LAST_INPUT = INPUT_VALUE;
    TIME_LAST = NOW;

    //Write the new value to the motors:
    PID_MOTORS();

    DOUBLE_ON_LCD(12,1,SETPOINT,false);
    DOUBLE_ON_LCD(12,2,INPUT_VALUE,false);
    DOUBLE_ON_LCD(12,3,ERROR,false);
    DOUBLE_ON_LCD(3,1,KP,false);
    DOUBLE_ON_LCD(3,2,KI,false);
    DOUBLE_ON_LCD(3,3,KD,false);
}
unsigned int SPEED_CONVERSION(float DEGREE) {
    return 1000 + (DEGREE * 150 + 13) / 27;
}
void PID_MOTORS() {
    //Motor write values. Take care of this later on. 
    //Double check all the new code/make sure it compiles.
    //Code compiles.  This function is gonna be tough since they spin
    //against each other. So we have to make it so that if we add to one 
    //and remove from another.  Not hard.  Just annoying.
    if(ERROR > 0) {
        double RIGHT_SPEED = SPEED_CONVERSION(RIGHT_MOTOR.read());
        double DIFFERENCE_RIGHT = (OUTPUT_VALUE - RIGHT_SPEED);
        RIGHT_MOTOR.writeMicroseconds(OUTPUT_VALUE - DIFFERENCE_RIGHT);
        LEFT_MOTOR.writeMicroseconds(OUTPUT_VALUE);
    }
    if(ERROR < 0) {
        double LEFT_SPEED = SPEED_CONVERSION(LEFT_MOTOR.read());
        double DIFFERENCE_LEFT = (OUTPUT_VALUE - LEFT_SPEED);
        RIGHT_MOTOR.writeMicroseconds(OUTPUT_VALUE);
        LEFT_MOTOR.writeMicroseconds(OUTPUT_VALUE - DIFFERENCE_LEFT);
    }
}
void CHECK_HALT() {
    UP.read(); 
    DOWN.read();
    LEFT.read();
    RIGHT.read();
    MIDDLE.read();
    if (UP.wasReleased())
    {
        PID_HALTED();
    }
    if (DOWN.wasReleased())
    {
        PID_HALTED();
    }
    if (LEFT.wasReleased())
    {
        PID_HALTED();
    }
    if (RIGHT.wasReleased())
    {
        PID_HALTED();
    }
    if (MIDDLE.wasReleased())
    {
        PID_HALTED();
    }

}
void PID_HALTED() {
    RUN_ALGO = !RUN_ALGO;
    LEDS_OFF();
    SET_SPEED(1300);
    delay(1000);
    SET_SPEED(1000);
    STRING_ON_LCD(0,0, "PID LOOP HALTED", true);
    while(!RUN_ALGO) {
        LEDS_RED();
        delay(750);
        LEDS_OFF();
        delay(750);
    }
}

//LCD UPDATE Function.  Give two ints, a string or double and a boolean for clear or not.
//Might help cut down on code size. 
void STRING_ON_LCD(int COLL, int ROW, String TEXT, bool CLEAR) {
    if(CLEAR) {
        GYROCUBE_LCD.clear();
    }
    GYROCUBE_LCD.setCursor(COLL, ROW);
    GYROCUBE_LCD.print(TEXT);
}
void DOUBLE_ON_LCD(int COLL, int ROW, double VALUE, bool CLEAR) {
    if(CLEAR) {
        GYROCUBE_LCD.clear();
    }
    GYROCUBE_LCD.setCursor(COLL, ROW);
    GYROCUBE_LCD.print(VALUE);
}


//--------------------------------------------------------------------------------------------------------------------------------------------------------------
//RUNTIME CODE:

void setup() {
    //SERIAL MONITOR AND LCD DONT WORK AT THE SAME TIME.  DAMN IT.
    //Debugging purposes.
    //Serial.begin(38400);

    Wire.begin(); //For the MPU9255

    //Fire up LEDs:
    pinMode(LED_LEFT_RED, OUTPUT);  //LEFT RED PIN OUT
    pinMode(LED_LEFT_GREEN, OUTPUT);  //LEFT GREEN PIN OUT
    pinMode(LED_RIGHT_RED, OUTPUT); //RIGHT RED PIN OUT
    pinMode(LED_RIGHT_GREEN, OUTPUT); //RIGHT GREEN PIN OUT
    delay(350);

    //Motor Connections and arming
    LEFT_MOTOR.attach(LEFT_MOTOR_PIN);
    RIGHT_MOTOR.attach(RIGHT_MOTOR_PIN);
    SET_SPEED(1000);
    delay(250);

    //Setup the MPU9255:
    pinMode(INT_PIN, INPUT);
    digitalWrite(INT_PIN, LOW);
    delay(350);

    //LCD Start:
    delay(1500);
    GYROCUBE_LCD.begin(20,4);

    //Setup PROGMEM Char sets to Strings:
    BOOT_WELCOME.set_asProgmem(1);
    BOOT_ZACK.set_asProgmem(1);
    BOOT_BRYCE.set_asProgmem(1);
    MAIN_TITLE.set_asProgmem(1);
    MAIN_RUN_PID.set_asProgmem(1);
    MONITOR_TITLE.set_asProgmem(1);

    //LCD Show Splash and Pause, then update:
    GYROCUBE_SYSTEM.change_menu(BOOT_MENU);
    delay(5000);
    GYROCUBE_SYSTEM.change_menu(MAIN_MENU);

    //System update:
    GYROCUBE_SYSTEM.update();
    CYCLE_LEDS(750);

}
//Simple ass void loop lol
void loop() {
    //Check ya buttons
    CHECK_BUTTONS();
}