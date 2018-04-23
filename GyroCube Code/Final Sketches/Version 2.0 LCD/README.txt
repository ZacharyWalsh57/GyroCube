V2 Setup for the Cube. Going to be doing some stuff differently since
I just found out that were out of local variable space.  Anything that 
changes, thats going into the normal flash.  Anything else is staying
away. 

Wire - LEDs and I2C Mapping
Servo - Used for the ESCs.  
I2CDev - Used for addressing the MPU
MPU6050... - Used to setup the MPU.
LiquidCrystal - LCD wiring
LiquidMenu - LCD Menus (Gonna take a lot of space....)
Button - Simple button library.  Keeps code neat.
EEPROM - Storing something eventually...

------------------------------------------------------------------------
Variables:
Define all the NEVER CHANGING PINS:
LEDs pins dont move
MOTORS dont move
MOTOR pulse rates are constant.
MPU Pins dont move (Don't need to worry)
LCD Pins dont move
Button Pins do not move
MPU Variables

Then use all the menu text and write it in one shot. 
Might consider moving it into progspace if things get tight.
Really would rather not. EDIT: I did lol

Create LCD Screens and menus and the System
Create the button pins and the button objects
Create the Servos
Create the MPU
Create the LED pins
------------------------------------------------------------------------
SETUP:
Start LCD
Start Servos and attach pins
pinMode LEDs
At least init the MPU maybe?
Open Serial monitor
-print out raw debug level things
Setup Telem output (later on)
Set the LCD focus points for each display.
------------------------------------------------------------------------

HELPER FUNCTIONS:


