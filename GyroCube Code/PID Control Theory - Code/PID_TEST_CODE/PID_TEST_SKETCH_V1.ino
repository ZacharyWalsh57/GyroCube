//NOTE: THIS IS NOT MEANT TO COMPILE.
//IF GITHUB THROWS BUILD FAILING IT IS NOT.

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
    while(ROLL_ANGLE > 40) {
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

//LCD UPDAT