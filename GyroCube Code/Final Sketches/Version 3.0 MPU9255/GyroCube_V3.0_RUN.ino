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
#define MIN_MOTOR_PULSE 1000
#define MAX_MOTOR_PULSE 2000

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
/*
LiquidLine MONITOR_KP(0,1, "Kp: SETP | SpA: XX.X");
LiquidLine MONITOR_KI(0,2, "Ki: SETI | RtA: XX.X"); //TODO: Change these into chars
LiquidLine MONITOR_KD(0,3, "Kd: SETD | PID: MOVE"); //FIXED: Just remove them overall. Just reprint the LCD every 2/3 seconds
*/

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
double INPUT_VALUE, OUTPUT_VALUE, SETPOINT;
double I_TERM, LAST_INPUT;
double KP, KI, KD;
int SAMPLE_TIME = 250; // .25 seconds
double OUT_MIN, OUT_MAX;
bool IN_AUTO = false;

#define MANUAL 0
#define AUTOMATIC 1 

#define DIRECT 0
#define REVERSE 1
int CONTROLLER_DIRECTION = DIRECT;

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
//as LEDS_*color*.  Cycle LEDS is for flashing through RGY
//RED
void LEDS_RED() {
    analogWrite(LED_LEFT_RED, 0);
    analogWrite(LED_LEFT_GREEN, 255);
    analogWrite(LED_RIGHT_RED, 0);
    analogWrite(LED_RIGHT_GREEN, 255);
}
//GREEN
void LEDS_GREEN() {
    analogWrite(LED_LEFT_RED, 255);
    analogWrite(LED_LEFT_GREEN, 0);
    analogWrite(LED_RIGHT_RED, 255);
    analogWrite(LED_RIGHT_GREEN, 0);
}
//YELLOW
void LEDS_YELLOW() {
    analogWrite(LED_LEFT_RED, 0);
    analogWrite(LED_LEFT_GREEN, 0);
    analogWrite(LED_RIGHT_RED, 0);
    analogWrite(LED_RIGHT_GREEN, 0);
}
//Flash through the LEDS - RED, GREEN, YELLOW
void CYCLE_LEDS(int DELAY) {
    LEDS_RED();
    delay(DELAY);
    LEDS_GREEN();
    delay(DELAY);
    LEDS_YELLOW();
}

//Motor throttle changes. THIS IS ONLY FOR SETUP. NOT THE LOOP:
void SWEEP_THROTTLE(int THROTTLE) {
    // Read the current throttle value
    int CURRENT_THROTTLE = READ_THROTTLE();
    // Are we going up or down?
    int STEP = 1;
    if (THROTTLE < CURRENT_THROTTLE)
        STEP = -1;
    // Slowly move to the new throttle value
    while (CURRENT_THROTTLE != THROTTLE)
    {
        LEFT_MOTOR.write(CURRENT_THROTTLE + STEP);
        RIGHT_MOTOR.write(CURRENT_THROTTLE + STEP);
        CURRENT_THROTTLE = READ_THROTTLE();
        delay(100);
    }
}
int READ_THROTTLE() {
    int THROTTLE_LEFT = LEFT_MOTOR.read();
    int THROTTLE_RIGHT = RIGHT_MOTOR.read();
    int TOTAL_THROTTLE = THROTTLE_LEFT;
    return TOTAL_THROTTLE;
}

//Arming ESC functions
void ESC_ARMING() {
    //Arm with old stuff.  PID with the new.
    LEFT_MOTOR.write(0);
    RIGHT_MOTOR.write(0);
    delay(1000);
    LEDS_GREEN();
    delay(250);
    CYCLE_LEDS(375);
    LEDS_YELLOW();
    SWEEP_THROTTLE(65);
    delay(2000);
    SWEEP_THROTTLE(40);
    delay(1500);
    CYCLE_LEDS(275);
    LEDS_GREEN();
    
}

//MPU92655 Init functions and updating RTVs
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
        ROLL_ANGLE = MPU_GYRO.roll;
        //Serial.println(ROLL_ANGLE);
    }
}

//Finally make sure all the parts are working right in one big init function.
void INIT_PARTS() {
    CYCLE_LEDS(750);
    GYROCUBE_LCD.setCursor(0, 1);
    GYROCUBE_LCD.print("LEDS SETUP");
    MPU9255_INIT();
    MPU9255_UPDATE();
    GYROCUBE_LCD.setCursor(0, 2);
    GYROCUBE_LCD.print("MPU9255 DMP OK");
    ESC_ARMING();
    GYROCUBE_LCD.setCursor(0, 3);
    GYROCUBE_LCD.print("ESCs ARMED");
    delay(2000);
    GYROCUBE_LCD.clear();
    GYROCUBE_SYSTEM.change_menu(MONITOR_MENU);
    delay(1500);
    CYCLE_LEDS(250);
    PID_ALGO();
    //Then finally, throw this bitch on autopilot lol
}

//PID FUNCTIONS: These functions are pretty fuckin dope if you ask me.  Took a good amount of time to
//figure it all out right.
void INIT_PID_ALGO() {
    LAST_INPUT = INPUT_VALUE;
    I_TERM = OUTPUT_VALUE;
    if(I_TERM > OUT_MAX) 
        I_TERM = OUT_MAX;
    else if(I_TERM < OUT_MIN)
        I_TERM = OUT_MIN;
}
void SET_DIRECTION(int DIRECTION) {
    CONTROLLER_DIRECTION = DIRECTION;
}
void FETCH_NEW_DATA() {
    //Fetch the latest angle values
    MPU9255_UPDATE();
    INPUT_VALUE = ROLL_ANGLE;
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
void SET_MODE(int MODE) {
    bool NEW_AUTO = (MODE == AUTOMATIC);
    if(NEW_AUTO == !IN_AUTO) {
        INIT_PID_ALGO();
    }
    IN_AUTO = NEW_AUTO;
}
void SET_TUNINGS(double SKP, double SKI, double SKD) {
    if (SKP < 0 || SKI < 0 || SKD < 0)
        return;
    
    double SAMPLE_TIME_SECONDS = ((double)SAMPLE_TIME) / 1000;
    KP = SKP;
    KI = SKI * SAMPLE_TIME_SECONDS;
    KD = SKD / SAMPLE_TIME_SECONDS;
}
void SET_SAMPLE_TIME(int NEW_TIME) {
    if(NEW_TIME > 0) {
        double RATIO = (double)NEW_TIME / double(SAMPLE_TIME);
        KI *= RATIO;
        KD /= RATIO;
        SAMPLE_TIME = (unsigned long)NEW_TIME;
    }
}
void PID_MOTORS(double VALUE) {
    //Motor write values. Take care of this later on. 
    //Double check all the new code/make sure it compiles.
}
void PID_COMPUTE() {
    //PID computation in here:
    if(!IN_AUTO)
        return;
    unsigned long NOW = millis();
    int ELAPSED = (NOW - TIME_LAST);
    if(ELAPSED >= SAMPLE_TIME) {
        //Compute Working Variables now:
        double ERROR = SETPOINT - INPUT_VALUE;
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
        else if (OUTPUT_VALUE < OUT_MAX)
            OUTPUT_VALUE = OUT_MIN;

        //Take note of some important values:
        LAST_INPUT = INPUT_VALUE;
        TIME_LAST = NOW;

        //Write the new value to the motors:
        PID_MOTORS(OUTPUT_VALUE);
    }
}
void PID_HALTED() {
    RUN_ALGO = false;
    LEDS_RED();
    GYROCUBE_LCD.clear();
    GYROCUBE_LCD.setCursor(0,0);
    GYROCUBE_LCD.print("PID LOOP HALTED");
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
void PID_ALGO() {
    CYCLE_LEDS(750);
    SET_DIRECTION(0);
    SET_OUTPUT_LIMITS(2000, 1000);
    SET_MODE(1);
    SET_TUNINGS(5.026, 0.22, 3.6);
    LEDS_GREEN();
    delay(150);
    while(RUN_ALGO) {
        CHECK_HALT();
        FETCH_NEW_DATA();
        PID_COMPUTE();
    } 
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
    LEFT_MOTOR.write(0);
    RIGHT_MOTOR.write(0);
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