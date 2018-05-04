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
void CHECK_HALT() {
    UP.read(); 
    DOWN.read();
    LEFT.read();
    RIGHT.read();
    MIDDLE.read();
    if (UP.wasReleased())
    {
        RUN_ALGO = false;
        LEDS_RED();
    }
    if (DOWN.wasReleased())
    {
        RUN_ALGO = false;
        LEDS_RED();
    }
    if (LEFT.wasReleased())
    {
        RUN_ALGO = false;
        LEDS_RED();
    }
    if (RIGHT.wasReleased())
    {
        RUN_ALGO = false;
        LEDS_RED();
    }
    if (MIDDLE.wasReleased())
    {
        RUN_ALGO = false;
        LEDS_RED();
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