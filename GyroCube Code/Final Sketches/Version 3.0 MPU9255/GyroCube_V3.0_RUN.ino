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
Servo MOTOR_LEFT;
Servo MOTOR_RIGHT;
#define LEFT_MOTOR 3
#define RIGHT_MOTOR 5
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
LiquidLine MONITOR_KP(0,1, "Kp: SETP | SpA: XX.X");
LiquidLine MONITOR_KI(0,2, "Ki: SETI | RtA: XX.X"); //TODO: Change these into chars
LiquidLine MONITOR_KD(0,3, "Kd: SETD | PID: MOVE");

//LCD - Screens:
LiquidScreen BOOT_SCREEN(BOOT_WELCOME, BOOT_ZACK, BOOT_BRYCE);
LiquidScreen MAIN_SCREEN(MAIN_TITLE, MAIN_RUN_PID);
LiquidScreen MONITOR_SCREEN(MONITOR_TITLE, MONITOR_KP, MONITOR_KI, MONITOR_KD);

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

//PID Constants:
#define Kp 1.00
#define Ki 0.12
#define Kd 2.45
#define SETPOINT 45

//MPU9255
#define INT_PIN 2
MPU9250 MPU_GYRO;
float ROLL_ANGLE;


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
        RUN_PID_ALGO();
    }
    if (DOWN.wasReleased())
    {
        GYROCUBE_LCD.clear();
        GYROCUBE_LCD.setCursor(5,0);
        GYROCUBE_LCD.print("SETTING UP");
        delay(500);
        RUN_PID_ALGO();
    }
    if (LEFT.wasReleased())
    {
        GYROCUBE_LCD.clear();
        GYROCUBE_LCD.setCursor(5,0);
        GYROCUBE_LCD.print("SETTING UP");
        delay(500);
        RUN_PID_ALGO();
    }
    if (RIGHT.wasReleased())
    {
        GYROCUBE_LCD.clear();
        GYROCUBE_LCD.setCursor(5,0);
        GYROCUBE_LCD.print("SETTING UP");
        delay(500);
        RUN_PID_ALGO();
    }
    if (MIDDLE.wasReleased())
    {
        GYROCUBE_LCD.clear();
        GYROCUBE_LCD.setCursor(5,0);
        GYROCUBE_LCD.print("SETTING UP");
        delay(500);
        RUN_PID_ALGO();
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
    //Small failsafe so it doesnt freak tf out.
    if (DELAY = 0) {
        DELAY = 500;
    }
    LEDS_RED();
    delay(DELAY);
    LEDS_GREEN();
    delay(DELAY);
    LEDS_YELLOW();
}

void ESC_ARMING() {
    //Arm new version with writeMillis here:
}

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

void RUN_PID_ALGO() {
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
    //Then finally, throw this bitch on autopilot lol

}

//--------------------------------------------------------------------------------------------------------------------------------------------------------------
//RUNTIME CODE:

void setup() {
    //Debugging purposes.
    //Serial.begin(38400);

    //Setup the MPU9255:
    Wire.begin();
    pinMode(INT_PIN, INPUT);
    digitalWrite(INT_PIN, LOW);

    //Link motors:
    MOTOR_LEFT.attach(LEFT_MOTOR);
    MOTOR_RIGHT.attach(RIGHT_MOTOR);
    
    //Fire up LEDs:
    pinMode(LED_LEFT_RED, OUTPUT);  //LEFT RED PIN OUT
    pinMode(LED_LEFT_GREEN, OUTPUT);  //LEFT GREEN PIN OUT
    pinMode(LED_RIGHT_RED, OUTPUT); //RIGHT RED PIN OUT
    pinMode(LED_RIGHT_GREEN, OUTPUT); //RIGHT GREEN PIN OUT

    //LCD Start:

    delay(2000);
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

}

void loop() {
    //Check ya buttons
    CHECK_BUTTONS();
}