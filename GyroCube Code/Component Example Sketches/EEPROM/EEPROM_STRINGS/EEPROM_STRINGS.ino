#include <EEPROM.h>
/* All the char arrays/Lines we need on this damn eeprom

const char BOOT_WELCOME_CHARS[] PROGMEM = "GyroCube 2.0";
const char BOOT_ZACK_CHARS[] PROGMEM = "Zack Walsh";
const char BOOT_BRYCE_CHARS[] PROGMEM = "Bryce Hadden";
const char MAIN_TITLE_CHARS[] PROGMEM = "GyroCube 2.0";
const char MAIN_RUN_PID_CHARS[] PROGMEM = "DO THE THING";
const char MAIN_MONITOR_CHARS[] PROGMEM = "Data Monitor";
const char MAIN_SETUP_CHARS[] PROGMEM = "Setup Components";
const char MONITOR_TITLE_CHARS[] PROGMEM = "Real-time Values";
const char COMPONENT_TITLE_CHARS[] PROGMEM = "SETUP COMPONENTS";
const char MPU6050_TEST_CHARS[] PROGMEM = "Connect MPU6050";
const char MOTOR_TESTS_CHARS[] PROGMEM = "Arm ESCs";
const char EEPROM_CHARS[] PROGMEM = "BURN EEPROM";

LiquidLine BOOT_WELCOME(4,0,BOOT_WELCOME_CHARS);
LiquidLine BOOT_ZACK(5,2,BOOT_ZACK_CHARS);
LiquidLine BOOT_BRYCE(4,3,BOOT_BRYCE_CHARS);
LiquidLine MAIN_TITLE(4,0,MAIN_TITLE_CHARS);
LiquidLine MAIN_RUN_PID(0,1,MAIN_RUN_PID_CHARS);
LiquidLine MAIN_MONITOR(0,2,MAIN_MONITOR_CHARS);
LiquidLine MAIN_SETUP(0,3,MAIN_SETUP_CHARS);
LiquidLine MONITOR_TITLE(2,0,MONITOR_TITLE_CHARS);
LiquidLine MONITOR_KP(0,1, "Kp: SETP | SpY: XX.X");
LiquidLine MONITOR_KI(0,2, "Ki: SETI | RtY: XX.X"); //TODO: Change these into chars
LiquidLine MONITOR_KD(0,3, "Kd: SETD | PID: MOVE");
LiquidLine COMPONENT_TITLE(2,0,COMPONENT_TITLE_CHARS);
LiquidLine MPU6050_TEST(0,1,MPU6050_TEST_CHARS);
LiquidLine MOTOR_TESTS(0,2,MOTOR_TESTS_CHARS);
LiquidLine EEPROM_BURN(0,3,EEPROM_CHARS);

*/ 

//One choice:
#define BOOT_WELCOME "GyroCube 2.0"
#define BOOT_ZACK "Zack Walsh"
#define BOOT_BRYCE "Bryce Hadden"
#define MAIN_TITLE "GyroCube 2.0"
#define MAIN_RUN_PID "DO THE THING"
#define MAIN_MONITOR "Data Monitor"
#define MAIN_SETUP "Setup Components"
#define MONITOR_TITLE "REAL-TIME VALUES"
#define COMPONENT_TITLE "SETUP COMPONENTS"
#define MPU6050_TEST "Connect MPU6050"
#define MOTOR_TESTS "Arm ESCs"
#define EEPROM "BURN EEPROM"



/*
//Then make chars globally for the EEPROM - Stupid fuckin way.
//Better Method:
#define char BOOT_WELCOME_CHARS[] = "GyroCube 2.0"
#define char BOOT_ZACK_CHARS[] = "Zack Walsh"
#define char BOOT_BRYCE_CHARS[] = "Bryce Hadden"
#define char MAIN_TITLE_CHARS[] = "GyroCube 2.0"
#define char MAIN_RUN_PID_CHARS[] = "DO THE THING"
#define char MAIN_MONITOR_CHARS[] = "Data Monitor"
#define char MAIN_SETUP_CHARS[] = "Setup Components"
#define char MONITOR_TITLE_CHARS[] = "Real-time Values"
#define char COMPONENT_TITLE_CHARS[] = "SETUP COMPONENTS"
#define char MPU6050_TEST_CHARS[] = "Connect MPU6050"
#define char MOTOR_TESTS_CHARS[] = "Arm ESCs"
#define char EEPROM_CHARS[] = "BURN EEPROM"
*/

//Insert the Lines into the EEPROM now:
void setup() { 
    EEPROM.put(0,BOOT_WELCOME_CHARS);
    EEPROM.put(64,BOOT_ZACK_CHARS);
    
    Serial.begin(115200);
    Serial.println("EEPROM STORED, DELAY AND READ");
    
    delay(2000);
}
void loop() {}
