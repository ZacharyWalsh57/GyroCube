#include <LiquidCrystal.h>
#include <LiquidMenu.h>
#include <Button.h>
#include <Wire.h>

//LCD Pins
const byte RS=0;
const byte EN=1;
const byte D4=14;
const byte D5=15;
const byte D6=16;
const byte D7=17;

//Button Pins
const bool PULLUP=true;
const bool INVERT=true;
const byte DB_MS=20;

const byte UP_BTN=4;
const byte DN_BTN=7;
const byte LT_BTN=8;
const byte RT_BTN=12;
const byte MD_BTN=13;

//Setup the pin assignments here:
LiquidCrystal GYROCUBE_LCD(RS,EN,D4,D5,D6,D7);
Button UP(UP_BTN,PULLUP,INVERT,DB_MS);
Button DOWN(DN_BTN,PULLUP,INVERT,DB_MS);
Button LEFT(LT_BTN,PULLUP,INVERT,DB_MS);
Button RIGHT(RT_BTN,PULLUP,INVERT,DB_MS);
Button MIDDLE(MD_BTN,PULLUP,INVERT,DB_MS);

//Make the Menu Here:
//Functions for menu stepping:
enum FunctionTypes {
    INCREASE = 1,
    DECREASE = 2,
};

//Start with a boot screen just to get the hang of making this work:
LiquidLine BOOT_WELCOME(4,0, "GyroCube 2.0");
LiquidLine BOOT_ZACK(5,2, "Zack Walsh");
LiquidLine BOOT_BRYCE(4,3, "Bryce Hadden");
LiquidScreen BOOT_SCREEN(BOOT_WELCOME, BOOT_ZACK, BOOT_BRYCE);

//Make a main Screen now which direct to other menus:
LiquidLine MAIN_TITLE(4,0, "GyroCube 2.0");
LiquidLine MAIN_RUN_PID(0,1, "DO THE THING");
LiquidLine MAIN_MONITOR(0,2, "Data Monitor");
LiquidLine MAIN_SETUP(0,3, "Setup Components");
LiquidScreen MAIN_SCREEN(MAIN_TITLE, MAIN_MONITOR, MAIN_SETUP, MAIN_RUN_PID);

//Make the Monitor Screen here:
LiquidLine MONITOR_TITLE(2,0, "Real-time Values");
LiquidLine MONITOR_KP(0,1, "Kp: SETP | SpY: XX.X");
LiquidLine MONITOR_KI(0,2, "Ki: SETI | RtY: XX.X");
LiquidLine MONITOR_KD(0,3, "Kd: SETD | PID: MOVE");
LiquidScreen MONITOR_SCREEN(MONITOR_TITLE, MONITOR_KP, MONITOR_KI, MONITOR_KD);

//Make a Components Menu now:
LiquidLine COMPONENT_TITLE(2,0, "SETUP COMPONENTS");
LiquidLine MPU6050_TEST(0,1, "Connect MPU6050");
LiquidLine MOTOR_TESTS(0,2, "Arm ESCs");
LiquidLine EEPROM_TESTS(0,3, "DUMP EEPOROM");
LiquidScreen COMPONENT_SCREEN(COMPONENT_TITLE, MPU6050_TEST, MOTOR_TESTS, EEPROM_TESTS);

//Now wrap it all inside of a System Menu for navigation:
LiquidMenu BOOT_MENU(GYROCUBE_LCD, BOOT_SCREEN);
LiquidMenu MAIN_MENU(GYROCUBE_LCD, MAIN_SCREEN);
LiquidMenu MONITOR_MENU(GYROCUBE_LCD, MONITOR_SCREEN);
LiquidMenu COMPONENT_MENU(GYROCUBE_LCD, COMPONENT_SCREEN);
LiquidSystem GYROCUBE_SYSTEM(BOOT_MENU, MAIN_MENU, MONITOR_MENU, COMPONENT_MENU);


//NOTE: THESE FUNCTIONS ARE NOT DONE YET.  THIS IS JUST POC.
//WHEN THE FINAL PROGRAM IS READY, THEY WILL BE USED IN THE 
//ACTUAL LOOPS!
void CHECK_BUTTONS() {
    UP.read();
    DOWN.read();
    MIDDLE.read();
    LEFT.read();
    RIGHT.read();

    if(UP.wasReleased()) {
    }
    if(DOWN.wasReleased()) {
    }
    if(MIDDLE.wasReleased()) {
    }
    if(LEFT.wasReleased()) {
    }
    if(RIGHT.wasReleased()) {
    }
}

//Simple menu swap functions.  They get latched onto other lines.
void RETURN_HOME() {
    GYROCUBE_SYSTEM.change_menu(MAIN_MENU);
}
void VIEW_MONITOR() {
    GYROCUBE_SYSTEM.change_menu(MONITOR_MENU);
}
void VIEW_COMPONENTS() {
    GYROCUBE_SYSTEM.change_menu(COMPONENT_MENU);
}

void setup() {
    //LCD Startup
    delay(2000);
    GYROCUBE_LCD.begin(20,4);
    
    //Throw up the splash screen
    GYROCUBE_SYSTEM.change_menu(BOOT_MENU);
    delay(5000);

    //Force the main menu now and move into the main loop
    GYROCUBE_SYSTEM.change_menu(MAIN_MENU);

}

void loop() {
    //NOTHING TO DO HERE YET
}









