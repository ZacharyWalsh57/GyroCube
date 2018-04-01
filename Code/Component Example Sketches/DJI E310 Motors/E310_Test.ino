/*E310 System setup and calibrations. Since DJI ESCs dont really need
to be armed, this is kinda overkill but hey who cares.  

Since ESCs are the same thing as a servo (0-180 PWM inputs)
we can just call the two controllers Servos and issue values from 0
to 180 (one half turn since thats a normal Servo rotation)

These 420s ESCs arm at 0 but wont spin till 25 DEG is issued
Even at 25 DEG the motors are jittery, so the best answer is 
to just send off 30-35 for an idle/startup once they get power 

That means we only have 30-150 for our range of throttle. 
There fore, 30 is 1% and 150 is 100% throttle.  Aside from that
RPM can be found by working out the PWM pulse being sent and then
using the 960rpm/V rating on the motors, but thats a different story. 

LOGIC FLOW:
Pin Assignments:
  -> Setup two servos (Left and Right controllers)
  -> Pins 3/5 respectively
  -> Setup a min and max pulse rate for the PWM output.
     -> For these motors, the min is 1000 and max is 2000
     -> So we can try to stop from getting spikes in current, 
        a delay of .1 seconds is used when changing throttle.
        The final version is gonnna have to deal with it.

SETUP:
  -> Start Serial (9600 BAUD)
  -> Attach servos to pins
  -> Set the pulse rates to the servos (May wanna modify them a little)
  -> Write 0 to both of them for an arm command and hear the glorious 1356 beep

LOOP:
  -> Wait for input on console side
  -> Call two functions lower in the code for throttle changeThrottle
  -> Repeat

normalizeThrottle:
  -> Make sure the values are just between 0 and 180 thats all it does
  -> Really just to prevent the servo lib from bugging the fuck out on us.

changeThrottle:
  -> Read current throttle and then print it.
  -> Then from there add one to the throttle, print it 
     bump it up one step.  Repeat till throttle = set 
  -> When we hit the desired throttle chill.

*/

#include <Servo.h> 

Servo escLeft;
Servo escRight;

int escPinLeft = 3;
int escPinRight = 5;
int minPulseRate = 1000;
int maxPulseRate = 2000;
int throttleChangeDelay = 100;

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(500);
  // Attach the the servos to the correct pin and set the pulse range
  escLeft.attach(escPinLeft, minPulseRate, maxPulseRate); 
  escRight.attach(escPinRight, minPulseRate, maxPulseRate);
  // Write a minimum value (most ESCs require this correct startup)
  escLeft.write(0);
  escRight.write(0);
}

void loop() {
  // Wait for some input
  if (Serial.available() > 0) {
    // Read the new throttle value
    int throttle = normalizeThrottle( Serial.parseInt() );
    // Print it out
    Serial.print("Setting throttle to: ");
    Serial.println(throttle);
    // Change throttle to the new value
    changeThrottle(throttle);
  }
}

void changeThrottle(int throttle) {
  // Read the current throttle value
  int currentThrottle = readThrottle();
  // Are we going up or down?
  int step = 1;
  if( throttle < currentThrottle )
    step = -1;
  // Slowly move to the new throttle value 
  while( currentThrottle != throttle ) {
    escLeft.write(currentThrottle + step);
    escRight.write(currentThrottle + step);
    currentThrottle = readThrottle();
    delay(throttleChangeDelay);
  }
}

int readThrottle() {
  int throttle = escLeft.read();
  Serial.print("Current throttle is: ");
  Serial.println(throttle);
  return throttle;
}

// Ensure the throttle value is between 0 - 180
int normalizeThrottle(int value) {
  if( value < 0 )
    return 0;
  if( value > 180 )
    return 180;
  return value;
}