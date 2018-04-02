/* MPU6050 Angle Reading and Saving.  Basic for now, read an angle and when there,
light up the LEDS GREEN ON BOTH SIDES. 

MAIN TODO: - DONE:      FIX THE DAMN FIFO PROBLEMS
           - DONE:      FIX MANAGING NEGATIVES
           - DONE:      FIX OFFSETS (WOO)

This is just a demo version of the final MPU6050 angle saving program which will
HOPEFULLY be able to send angles back, process them, and ligth up the LEDS in 
accordance with the distance from the desired angle.  Thats gonna eat so much ass
to program....

Some basic research lead me to find out some pretty strange stuff about how this works.
    -> The IMU needs the gyro to compensate for vibrations caused by the motors
    -> This can be done by just combining the angle with a quarter of the angle measured
       from the MPU6050 at that given point in time. 
    -> This works because of A) the method of data readback on the MPU6050 and B) because
       SCIENCE BITCH. (Really because of Control Theory laws and becasue we're using PID
       control methods in this project for our end goal)
    -> The best way to put the Accel and Gyro in sync is to just set their angles equal
       on startup and then begin taking measurements. Since in our case, the gyro is in 
       the same place all the time, we can set these calibration values once and walk away
       (After some stupid fucking tweaking)
    -> Somewhere inside this folder of all the project documentation is going to be a 
       spreadsheet which contains a graph showing what I mean about dividing and angle 
       matching.

LOGIC FLOW FOR THIS INO SCRIPT:

PRE SETUP
    -> LEDS
        -> Establish pins in use for the LEDS like before
            -> LED Right - Red=9, Green=6, Yellow=9+6
            -> LED Left - Red=11, Green=10, Yellow=11+10
        -> Setup Brightness For Left and Right LEDS for each color
            -> LEFT_RED=0
            -> LEFT_GRN=0
            -> RIGHT_RED=0
            -> RIGHT_GRN=0

        -> MPU6050 Pin Setup and Comms (Credit to someone on GitHub for this life
        saving block of code to come now....)
            -> Include Libs and other supporting dynamic links
            -> Establish a set of uint8s (or just ints. Oh and FUCK C++) for the
            interupt, dev, packet size, fifo count? (Look into this), and fifoBuffer
            (????)
            -> FIXED - FIFO: First in first out.  Used as a data sharing method across the
               I2C protocol to real the first bit in and the next consecuitive bit out.
               Really handy for this exact type of application.
            -> Setup orientation and Motion Variables for storage
                -> Quaternion
                -> Vector for Accel
                -> Vector for Accel without gravity
                -> Vector for World based Accel
                -> Vector for gravity itself
                -> Euler angle container (indexed list) - 3 (XYZ, or PSI, THETA, PHI)
                -> yaw, pitch, and roll variables as well (indexed list) - 3 (XYZ measurements)
            -> Setup an volitale function for the interupt pin on the MPU (DIGITAL 2)

SETUP Loop
    -> MPU6050 Comms Setup
        -> Connect the wire and I2Cdev libs together and set the CPU clock speed.
        -> Begin a high Baud Serial comm between the MPU and host.
        -> Init the MPU650 chip with the MPU6050 libs
        -> To begin comms (DMP Program mode) we're gonna use delays instead of serial input.
            -> HOWEVER, For the sake of this demo, we will be inputing an angle to match
        -> Open the comm between the MPU and the Arduino now.
        -> Set some offsets (WOO, IF YOUNG METRO DONT TRUST YOU, YOU KNOW WHATS GOOD)
        -> Check the Dev Status again (DMP really....) and make sure all is working
        -> Enable the interupt on pin 2 finally.....
        -> Wait for DMP interup and sniff packets back from the chip.
        -> In the case we hit some errors, the LEDs will go red, the error codes will
            go inside the serial monitor (Eventially the LCD)
        -> Then last and CERTIANLY LEAST, pinmode the LEDs.

VOID Loop
    -> Before we do a single thing, make sure the connection and Init went off ok.
    -> After it doesnt and I jump out a window realizing I have a lose connection,
       open the flood gates and grab some data finally....
    -> Reset the interup and kick this stupid chip out of program mode
    -> Get the current FIFO count (FIFO Buffer size and the number of physical packets)
    -> If we track the number of packets, we can end up reading more data without waiting
       for the next interupt. (AKA, it's lit)
    -> From there, we will print out the values to the serial console.
    -> Using an If statement, when the ROLL value matches the value we picked, the LEDs 
       go green.  When it's not matched, they dont light up.
    -> This is kinda shitty because the gyro doesnt keep the same desired value EXACTLY
       long enough for us to really see if it's triggering.
    -> TODO - test the LED activation at a "flat" angle first then see how it reacts
       if/when the LEDs go on at a flat angle, then we know it works.  If not, well idk lol


*/
//Include all the libs and dynamic links we need for this program to fire on all cylinders
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

//Simple LED Pin assignments go here:
int LED_LEFT_RED = 11; //LEFT RED
int LED_LEFT_GRN = 10; //LEFT GREEN
int LED_RIGHT_RED = 9; //RIGHT RED
int LED_RIGHT_GRN = 6; //RIGHT GREEN

//Begin a METRIC FUCK TON of code which I honestly dont understand nor will I probably ever
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE //Make sure the Wire and I2Cdev are combined
#include "Wire.h"
#endif

//Setup a mutable object which is properly addressable.
//Again, this is not possible without that dude on GitHub
MPU6050 MPU6050_CHIP;

//Set the onboard LED to solid, just so we can keep some more speed going
//(even though this poor excuese for code might be pretty damn quick)
//That little bool at the end just acts as a quick toggle for later on.
#define LED_PIN_ONBOARD 13
bool BLINK_STATE = false;

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

//Teapot Packet
uint8_t teapotPacket[14] = {'$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n'};

//Interupt Detection HERE:
volatile bool MPU_INTERUPT = false; //Sets if MPU interupt pin is low or high currently
void DMP_DATA_READY()
{
    MPU_INTERUPT = true;
}

//FINALLY Setup the comms for the MPU6050 and the LEDs
void setup()
{

    //LEDS Out of the way first:
    pinMode(LED_LEFT_RED, OUTPUT);  //LEFT RED PIN OUT
    pinMode(LED_LEFT_GRN, OUTPUT);  //LEFT GREEN PIN OUT
    pinMode(LED_RIGHT_RED, OUTPUT); //RIGHT RED PIN OUT
    pinMode(LED_RIGHT_GRN, OUTPUT); //RIGHT GREEN PIN OUT

//Now onto the fun shit. MPU6050 setup time:
//Join I2c Bus if not done already so we can talk between boards
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    TWBR = 24;
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif

    //Setup a serial comm line
    Serial.begin(115200);
    while (!Serial)
        ;

    //Initialization
    Serial.println("CONNECTING TO MPU6050");
    MPU6050_CHIP.initialize();

    delay(5000); //Give it time to warm up....

    //Check everything went off ok...
    Serial.println(F("Testing device connections..."));
    Serial.println(MPU6050_CHIP.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    //Wait for a ready signal on the interup pin (We dont care honestly, but we do want an angle saved)
    Serial.println("SETTING IDEAL BALANCING ANGLE NOW....");
    Serial.println("ANGLE OF 45 DEG YAW SAVED");

    //Try to init the DMP on the chip here
    Serial.println("ATTEMPTING INIT OF DMP NOW....");
    DEV_STATUS = MPU6050_CHIP.dmpInitialize();

    //And now finally, initalize the offsets (WOO) These will need some messing around with
    MPU6050_CHIP.setXGyroOffset(114);
    MPU6050_CHIP.setYGyroOffset(-101);
    MPU6050_CHIP.setZGyroOffset(-3);
    MPU6050_CHIP.setZAccelOffset(1928); //Factory is around 1688 or 1668.  Idk who cares its 230am rn

    if (DEV_STATUS == 0)
    {
        //Set the DMP to ON since its working by the grace of god...
        Serial.println("DMP ALL GOOD, LETS BURN THIS BITCH....");
        MPU6050_CHIP.setDMPEnabled(true);

        //Setup the interup here and set the arduino to look for it
        Serial.println("CHECKING FOR INTERUP AND ENABLING NOW....");
        attachInterrupt(0, DMP_DATA_READY, RISING);
        MPU_INT_STATUS = MPU6050_CHIP.getIntStatus();

        //Set the DMP flag to READY for the main loop to run right:
        DMP_READY = true; //ITS TOO LITTY

        //Get packet to compare for later
        PACKET_SIZE = MPU6050_CHIP.dmpGetFIFOPacketSize();
    }
    else
    {
        //SHIT HIT THE FAN ABORT NOW
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(DEV_STATUS);
        Serial.println(F(")"));
        analogWrite(LED_LEFT_RED, 0);
        analogWrite(LED_LEFT_GRN, 255);
        analogWrite(LED_RIGHT_RED, 0);
        analogWrite(LED_RIGHT_GRN, 255);
    }
}
//END SETUP LOOP NOW

void loop()
{
    //If the DMP init broke, stop now.
    if (!DMP_READY)
        return;

    //Wait for the MPU to get extra packets for realtime logging
    //Reset the interupt flag to make sure it looks again (And FIFO)
    MPU_INTERUPT = false;
    MPU_INT_STATUS = MPU6050_CHIP.getIntStatus();
    FIFO_COUNT = MPU6050_CHIP.getFIFOCount();

    //Check for overflow now..... Should really never happen but hey never know
    if ((MPU_INT_STATUS & 0x10) || FIFO_COUNT == 1024)
    {
        MPU6050_CHIP.resetFIFO();
        Serial.println("FIFO FUCKED GOOD. Resetting");
    }
    else if (MPU_INT_STATUS & 0x02)
    {
        while (FIFO_COUNT < PACKET_SIZE)
            FIFO_COUNT = MPU6050_CHIP.getFIFOCount();
        MPU6050_CHIP.getFIFOBytes(FIFO_BUFFER, PACKET_SIZE);

        //Track the FIFO Count this way to prevent shit from going bad if theres +1 packets
        //in line to be processed.
        FIFO_COUNT -= PACKET_SIZE;

        //Now grab the data once it has the right length from the FIFO
        //Display Euler angles in degrees
        MPU6050_CHIP.dmpGetQuaternion(&QUAT, FIFO_BUFFER);
        MPU6050_CHIP.dmpGetGravity(&GRAVITY, &QUAT);
        MPU6050_CHIP.dmpGetYawPitchRoll(YAW_PITCH_ROLL, &QUAT, &GRAVITY);

        float ROLL_ANGLE = ((YAW_PITCH_ROLL[2] * 180/M_PI));
        Serial.print("ROLL ANGLE IS:\t");
        Serial.print(ROLL_ANGLE);

        if(ROLL_ANGLE > 0) {
            if ((ROLL_ANGLE > 50) && (ROLL_ANGLE < 55)) 
            {   //Away from MPU wiring
                Serial.println("\t\tWITHIN THE RANGE: SETTING GREEN");
                analogWrite(LED_LEFT_RED, 255);
                analogWrite(LED_LEFT_GRN, 0);
                analogWrite(LED_RIGHT_RED, 255);
                analogWrite(LED_RIGHT_GRN, 0);
            }

            if ((ROLL_ANGLE < 50) || (ROLL_ANGLE > 55)) 
            {   //Away from MPU wiring
                Serial.println("\t\tOUTSIDE THE RANGE: SETTING RED");
                analogWrite(LED_LEFT_RED, 0);
                analogWrite(LED_LEFT_GRN, 255);
                analogWrite(LED_RIGHT_RED, 0);
                analogWrite(LED_RIGHT_GRN, 255);
            }
        }

        if(ROLL_ANGLE < 0) {
            if((ROLL_ANGLE < -40) && (ROLL_ANGLE > -45)) 
            {   //Towards MPU Wiring
                Serial.println("\t\tWITHIN THE RANGE: SETTING GREEN");
                analogWrite(LED_LEFT_RED, 255);
                analogWrite(LED_LEFT_GRN, 0);
                analogWrite(LED_RIGHT_RED, 255);
                analogWrite(LED_RIGHT_GRN, 0);
            }
            if((ROLL_ANGLE > -40) || (ROLL_ANGLE < -45)) 
            {   //Towards MPU Wiring       
                Serial.println("\t\tOUTSIDE THE RANGE: SETTING RED");
                analogWrite(LED_LEFT_RED, 0);
                analogWrite(LED_LEFT_GRN, 255);
                analogWrite(LED_RIGHT_RED, 0);
                analogWrite(LED_RIGHT_GRN, 255);
            }
        }

        /* First attempt and shitty method of setting up when to turn the LEDs red/green
        Had issues with omitting Ifs and not even changing them sometimes.  Delays cause FIFO
        problems.  Really just not good.

        //If we're in the green:
        if ((YAW_PITCH_ROLL[2] * 180 / M_PI) > 40 || (YAW_PITCH_ROLL[2] * 180 / M_PI) < 45)
        {
            analogWrite(LED_LEFT_RED, 255);
            analogWrite(LED_LEFT_GRN, 0);
            analogWrite(LED_RIGHT_RED, 255);
            analogWrite(LED_RIGHT_GRN, 0);
        }

        //If were not there, then make the lights red.
        else if ((YAW_PITCH_ROLL[2] * 180 / M_PI) < 40 || (YAW_PITCH_ROLL[2] * 180 / M_PI) > 45)
        {
            analogWrite(LED_LEFT_RED, 0);
            analogWrite(LED_LEFT_GRN, 255);
            analogWrite(LED_RIGHT_RED, 0);
            analogWrite(LED_RIGHT_GRN, 255);
        }


        //If we're in the green:
        else if ((YAW_PITCH_ROLL[2] * 180 / M_PI) < -40 || (YAW_PITCH_ROLL[2] * 180 / M_PI) > -45)
        {
            analogWrite(LED_LEFT_RED, 255);
            analogWrite(LED_LEFT_GRN, 0);
            analogWrite(LED_RIGHT_RED, 255);
            analogWrite(LED_RIGHT_GRN, 0);
        }


        //If were not there, then make the lights red.
        else if ((YAW_PITCH_ROLL[2] * 180 / M_PI) > -40 || (YAW_PITCH_ROLL[2] * 180 / M_PI) < -45)
        {
            analogWrite(LED_LEFT_RED, 0);
            analogWrite(LED_LEFT_GRN, 255);
            analogWrite(LED_RIGHT_RED, 0);
            analogWrite(LED_RIGHT_GRN, 255);
        }
        */

    }
}
