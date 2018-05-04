# GyroCube

The PID (Or maybe even PD) based, arduino powered, self balancing cube!

5/3/18:
After finally getting a new IMU and some other parts, development is back on track.  First round of progress updates is on 5/8/18.
Hopefully by then we'll have:
- A working setup loop which proves all the parts work.
- A 99% functional PID loop
- No more memory issues (Please. Running with 79% SRAM usage is painful)
- And finally, a prety good part of our presentation done. If all goes well, this might somehow work

Whats new?
- New IMU: Waveshare 10DOF IMU with an MPU9255 sensor on it (so more new libraries)
- Fully functional LCD
- Soon to be rewritten code to store more things in EEPROM and use writeMillis instead of analog write for motor speeds.
------------------------------------------------------------------------------------------------------------------------------------------

4/7/18:
It somehow didn't take that long to do a full rebuild.  The second version is up and running with stronger parts and better wiring.

Some things added now:
- PLA Soldered the parts together.  Looks bad but works wonders.
- Fixed motor vibrations
- Added in new sketch that lets us find the zero point of the MPU since we cant seem to fix the calibrations any other way.
- Also threw up diagrams for the control box and the three voltage dividers we will use for battery life monitoring.  

------------------------------------------------------------------------------------------------------------------------------------------

4/6/18:
- Well unfortunately thing took a turn for the worst.  We're left with no choice but to rebuild the whole cube from the ground up.  The repo will remain the same but will probably branch into version 1.5 leaving the main one behind. Code will be the same for basic component setup, but maybe some pins will change.  


What went wrong you might ask? Basically anything we could have imagined.
- Motor flywheels were unbalanced and caused incredible amounts of vibration.
- Hardware used to hold the cube together ripped the sides apart and caused instability.
- The MPU6050 wiring wasn't done 100% perfectly and the leads ripped out of the breakout board.
- When doing a 90% throttle test with the flywheels on, one of the flywheels heated up and expanded off of the motor completly ripping apart the entire cube.  That's the biggest problem we had.  


In the next version, as mentioned before code will remain the same for the most part, but we will likley either move to the Processing IDE or hook the system up to MATLAB (ugh) for GUI based data and tuning.  
