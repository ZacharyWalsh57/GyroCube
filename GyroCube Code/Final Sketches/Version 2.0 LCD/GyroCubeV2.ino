#include <Wire.h>
#include <Servo.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Button.h>
#include <LiquidCrystal.h>
#include <LiquidMenu.h>

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

//PID Constants:
#define Kp 1.00
#define Ki 0.12
#define Kd 2.45
#define SETPOINT 45

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
const char BOOT_WELCOME_CHARS[] PROGMEM = "GyroCube 2.0";
const char BOOT_ZACK_CHARS[] PROGMEM = "Zack Walsh";
const char BOOT_BRYCE_CHARS[] PROGMEM = "Bryce Hadden";
const char MAIN_TITLE_CHARS[] PROGMEM = "GyroCube 2.0";
const char MAIN_RUN_PID_CHARS[] PROGMEM = "Press Any Button";
const char MONITOR_TITLE_CHARS[] PROGMEM = "Real-time Values";

LiquidLine BOOT_WELCOME(4,0,BOOT_WELCOME_CHARS);
LiquidLine BOOT_ZACK(5,2,BOOT_ZACK_CHARS);
LiquidLine BOOT_BRYCE(4,3,BOOT_BRYCE_CHARS);
LiquidLine MAIN_TITLE(4,0,MAIN_TITLE_CHARS);
LiquidLine MAIN_RUN_PID(2,2,MAIN_RUN_PID_CHARS);
LiquidLine MONITOR_TITLE(2,0,MONITOR_TITLE_CHARS);
LiquidLine MONITOR_KP(0,1, "Kp: SETP | SpY: XX.X");
LiquidLine MONITOR_KI(0,2, "Ki: SETI | RtY: XX.X"); //TODO: Change these into chars
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

//MPU6050 Creation:
MPU6050 MPU6050_CHIP;

//MPU6050 Variables:
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
volatile bool MPU_INTERUPT = false; //Sets if MPU interupt pin is low or high currently
void DMP_DATA_READY() {MPU_INTERUPT = true;}


//SETUP HERE:
void setup() 
{
    //Turn on the LEDs
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

    GYROCUBE_SYSTEM.update();

}
//HELPER FUNCTIONS: BUTTONS, LCD, MPU, LEDS, PID, MENU ETC
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

void MPU6050_INIT() {
    MPU6050_CHIP.initialize();
    DEV_STATUS = MPU6050_CHIP.dmpInitialize();

    MPU6050_CHIP.setXGyroOffset(114);
    MPU6050_CHIP.setYGyroOffset(-101);
    MPU6050_CHIP.setZGyroOffset(-3);
    MPU6050_CHIP.setZAccelOffset(1928);
    
    if (DEV_STATUS == 0){
        MPU6050_CHIP.setDMPEnabled(true);
        attachInterrupt(0, DMP_DATA_READY, RISING);
        MPU_INT_STATUS = MPU6050_CHIP.getIntStatus();
        DMP_READY = true;
        PACKET_SIZE = MPU6050_CHIP.dmpGetFIFOPacketSize();
        CYCLE_LEDS(200);}

    else {
        GYROCUBE_LCD.clear();
        GYROCUBE_LCD.setCursor(0,0);
        GYROCUBE_LCD.print("MPU INIT FAILED! KILLING");
        LEDS_RED();
        return;}
}

void ESC_ARMING() {

}

void RUN_PID_ALGO() {
    CYCLE_LEDS(750);
    GYROCUBE_LCD.setCursor(0,1);
    GYROCUBE_LCD.print("LEDS SETUP");
    MPU6050_INIT();
    GYROCUBE_LCD.setCursor(0,2);
    GYROCUBE_LCD.print("MPU6050 DMP OK");
    ESC_ARMING();
    GYROCUBE_LCD.setCursor(0,3);
    GYROCUBE_LCD.print("ESCs ARMED");
    delay(2000);
    GYROCUBE_LCD.clear();
    GYROCUBE_SYSTEM.change_menu(MONITOR_MENU);
}

void loop() {
    CHECK_BUTTONS();
}