/*GYROCUBE V2.0 Setup
Not a whole lot to explain here but it can't hurt to just throw the basics down here:
The best way to go about it is just explaining by blocks of code/lines.  So here it goes

--------------------------------------------------------------------------------------------------

Begins at #include <... and ends at menu.h>:
These are the included Library files for the program to run correctly.  Each one IS critical.
---- Wire:          Used for the LEDs, the MPU Serial comms, and the LCD Pinnout.  Probably the
                    most important but most simple Library we used here.
---- Servo:         Used for the motor ESCs.  They use PPM so Servo is used instead of
                    a traditional setup of using digital write to send PWM (More on the Wiki)
---- I2CDev:        Used for the MPU connections and the LCD (Somehow. No idea how lol)
                    MPU runs serial data on SDA and SCL with pin D2 (P3) as an interupt.
                    FIFO buffer is in place to keep things orderly and read the serial data
                        Since the angle readings are sent via I2C, they come in 2 packets of 8
                        bits. This means we have to parse them somehow. Thats where FIFO comes into
                        the equiation.
---- MPU6050:       Not gonna lie, this is complex as fuck so stay with me here. This Library does
                    all the heavy lifting behind the scenes. Converting packets to floats,
                    initializing the DMP (Digital Motion Processor), watching for FIFO overflow
                    and more. As always credit is given where it's due, Thanks to jrowberg on GitHub
                    for his work on this Library which makes this project possible honestly.
---- Button:        Library I wrote in my spare time (which I have wayyy too much of) that allows/
                    easy creation of a hardware button on the Arduino.  Library uses a class of Button
                    and on construction and object type of button is given a pin, a boolean for if 
                    we are using pullup resistors and an int for a debounce time in MS to stop us
                    from picking up noise on the signal.
---- LqdCryst/Menu: Both of these fall under the same category so I'll explain them both 
                    here.  LiquidCrystal is responsible for making an object of LCD which is assigned
                    various pins for TX,RX, Data lines, and so on.
                    The LCD works something like this:
                        We used an old LCD/SD/Button combo off of a 3D printer that I bricked (whoops lol) so 
                        contrast and backlight were taken care of by the board that the LCD was on.
                        Interestingly, there's both a 7409 SR which would have potentially given us I2C 
                        connections and only used 2 pins, but it seems like it's big time burned out. The 
                        LiquidCrystal Library reads the pins we give the object of LCD and then is able to convert 
                        our ASCII characters (fancy for text) into digital bytes/encoded chars for the LCD to 
                        display.  The RS and EN lines are for realtime display so we can update the display in 
                        real time.

---------------------------------------------------------------------------------------------------------------

Begins with #define LED_LEFT... and ends at MIDDLE_PIN 13:
These lines are used for us to define some values we need throughout the program.
----  LEDS:         Pins needed for the LEDs to change color.  They are common annode meaning both colors
                    inside the LED share a common 5v+ input and use seperate grounds. So even though it says
                    that I'm giving 5V on the red pin, it's actually the green LED getting power. As some of you 
                    may or may not know already, when you feed a voltage into and LED thats in reverse bias, 
                    you actually create a resistance large enough that it seems like an open has been created 
                    and thus we end up turning off that color led.  And by using PWM signals, we can even create a small 
                    resistance that lets us dim an LED so we can make any combination of hues from Red and Green.

                        __________________________________________________________
                        |                   COMMON ANNODE LED                    |  If we set PIN_10 to 5v and PIN_11 to 0v,
                        |               PIN_10 = RED, PIN_11 = GREEN             |  then we allow 5v through PIN_11 and
                        |                                                        |  and block 5v on PIN_10 so we only have
                        |             GND       ================                 |  a complete circuit to ground on PIN_10
                        |     <| --- PIN_11 --- |-- GRN_WIRE ++|                 |  therefore only the green LED is powered
                        |                       |              | +++ 5v +++ ||   | 
                        |     <| --- PIN_10 --- |-- RED_WIRE ++|                 |  The same applies in for turning on the red
                        |             GND       |==============|                 |  LED except PIN_11 shows +5v and PIN_10 is 0v
                        |________________________________________________________|

---- MOTORS:        Code used to setup the ESCs for the 2312 motors inside the cube itself. Normally a motor is driven
                    by using either a variable current input (Pot to change how much resistance is shown to the motor)
                    or by using PWM which pulses a digital signal of 5v highs and 0v lows which the motor reads as an RPM.
                    In our case, the ESCs I am using are from an old drone that I built a while ago and didn't need to fly.
                    These are DJI 420s 30A ESCs which are made to run either a 2312 or 2212 motor depending on how they are
                    programmed.  These ESCs do way more than I would need them to do (active braking, velocity control, etc)
                    
---- PID Values:    Kp Ki Kd values used in the PID Loop for angle correction and rotation of the cube. Since I havent
                    written the PID loop yet, this is going to be left kinda blank. Kp represents the amount of correction
                    to output in comparison to a simple SET - CURRENT value. Ki represents the intergral of the data logged
                    over the course of a single repetition of  the loop.  Kd is the derivative of the data logged.  Each one 
                    is combined in a formula seen as:

                                u(t) = Kp(e(t)) + Ki((INT)e(TAU))*d*Tau) + Kd(d/dt(e(t))))

                    Where:
                        Kp = Proportion const
                        Ki = Intergral Const
                        Kd = Derivative const
                        e  = the error value or the difference from the last loop
                        TAU = the tau time constant to sample all values for a full cycle
                        d = derivation of values for current loop
                        t = time elapsed since last correction.

---- LCD Lines:     Lines to define pins for the LCD.  RS and EN are used for live updates, D4-D7 are used for the input to
                    the LCD itself.  The screen runs in 4 bit mode so we ignore D1-3 and D8-12 which would be used if the LCD 
                    was setup to run in 8 bit mode.  Using the Liquid Crystal Menu Library we can automaticaly assign the LCD
                    lines to a screen and then compact them into a menu which is packed into an object known as a liquid system.
                    Since these lines require massive amounts of SRAM space, they get stored in PROGMEM which is 32kb instead of 
                    the 2kb that SRAM is limited to.  Since we can't even defrag this piece of garbage RAM setup, it's just easier
                    to throw all the strings into the PROGMEM and call them as progmem later on in the setup function.
                
---- Button Pins:   Pins used to define the inputs for the 5 buttons on the LCD driver board. Each one is etup to read the digital
                    input on the pins by using pullup resistors and checking for a "change of state" essentially trigering a boolean
                    of wasPressed or wasReleased to true if the input level changes.  To prevent problems with noise and possible
                    signal latency, I have implimented a 20ms debounce timer for the signal input to wait for it to settle before 
                    checking for another change of state.

                    Under that, the buttons are constructed as objects with all the predefined pins and other flags needed for the 
                    program to read changes of 5v logic on the input pins for each button.


---- MPU6060 Vars:  Values defined/containers used for MPU data reading and storage.  Each one IS required unfortunately and they 
                    take up a fuck ton of global storage unfortunately.  Might consider trying to throw these in the EEPROM but I 
                    have a feeling it might burn out the write lifepan on the EEPORM.  All of the variable uses are explained 
                    next to their creations so I'm not writing that all out again.

---------------------------------------------------------------------------------------------------------------------------------------

As far as setup loop goes and the remaining functions, theyre all simple enough.

The setup loop does nothing other than pinmode the LED pins, Motor pins, and call the LCD lines from PROGMEM so they can be 
shown properly. From there, the LCD is setup and shows a splash screen/cycles the LEDS.

void loop is used to just look for the button inputs using the check buttons function which does just that.

For the remaining functions, the only one which needs explaining is really NORMALIZE_THROTTLE().  It reads a value that is 
supposedly going to be written onto the ESC pins to spin the motors. But since we're dealing with servos and they only go from
0-180, we cant send over 180 or less than 0. And since these motors wont idle befire 40, we take any value given and make
sure it's either less than 180 or over 40.  Anything outside that range is set to the closest limit
    - Values < 40 = 40
    - Values > 180 = 180

The rest of the functions are self explanitory or they are explained in detail within their example test folder
up two steps in the repo.  The motors, MPU, LCD, LEDs, and the buttons are all used in their own respective sketches
alone there for both testing, and to understand what's going on.

---------------------------------------------------------------------------------------------------------------------------------------
*/


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


//SETUP HERE. Fix up the LCD PROGMEM lines and assign some pins for LEDS, LCD,
//motors, throwing up the menu/splash screen and so on. 
void setup() {
    //Turn on the LEDs
    pinMode(LED_LEFT_RED, OUTPUT);  //LEFT RED PIN OUT
    pinMode(LED_LEFT_GREEN, OUTPUT);  //LEFT GREEN PIN OUT
    pinMode(LED_RIGHT_RED, OUTPUT); //RIGHT RED PIN OUT
    pinMode(LED_RIGHT_GREEN, OUTPUT); //RIGHT GREEN PIN OUT

    MOTOR_LEFT.attach(LEFT_MOTOR, MIN_MOTOR_PULSE, MAX_MOTOR_PULSE); //Left Motors
    MOTOR_RIGHT.attach(RIGHT_MOTOR, MIN_MOTOR_PULSE, MAX_MOTOR_PULSE); 

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

//Init the MPU6050 (Which of course I broke like an idiot.  New one in mail)
//Keeps things short and simple allowing us to cut down on the code size and
//complexity
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
        MOTOR_LEFT.write(CURRENT_THROTTLE + STEP);
        MOTOR_RIGHT.write(CURRENT_THROTTLE + STEP);
        CURRENT_THROTTLE = READ_THROTTLE();
        delay(100);
    }
}

void READ_THROTTLE() {
    int THROTTLE_LEFT = LEFT_MOTOR.read();
    int THROTTLE_RIGHT = RIGHT_MOTOR.read();
    int TOTAL_THROTTLE = (THROTTLE_LEFT + THROTTLE_RIGHT) / 2;
    return TOTAL_THROTTLE;
}

void NORMALIZE_THROTTLE(int THROTTLE_INPUT) {
    if (THROTTLE_INPUT < 40) {
        return 40;
    }
    if (THROTTLE_INPUT > 180) {
        return 180;
    }
    return THROTTLE_INPUT;
}

void ESC_ARMING() {
    MOTOR_LEFT.write(0);
    MOTOR_RIGHT.write(0);
    LEDS_GREEN();
    delay(250);
    LEDS_CYCLE(375);
    LEDS_YELLOW();
    SWEEP_THROTTLE(45);
    delay(2000);
    SWEEP_THROTTLE(0);
    delay(1500);
    LEDS_CYCLE(275);
    LEDS_GREEN();
}

void RUN_PID_ALGO() {
    CYCLE_LEDS(750);
    GYROCUBE_LCD.setCursor(0, 1);
    GYROCUBE_LCD.print("LEDS SETUP");
    MPU6050_INIT();
    GYROCUBE_LCD.setCursor(0, 2);
    GYROCUBE_LCD.print("MPU6050 DMP OK");
    ESC_ARMING();
    GYROCUBE_LCD.setCursor(0, 3);
    GYROCUBE_LCD.print("ESCs ARMED");
    delay(2000);
    GYROCUBE_LCD.clear();
    GYROCUBE_SYSTEM.change_menu(MONITOR_MENU);
}

void loop() {
    CHECK_BUTTONS();
}