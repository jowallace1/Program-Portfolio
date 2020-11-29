# Program Portfolio
A repository containing programs I developed for my thrust-vector-controlled rocket project and my research work. Some research-related programs have not been included because they will be published in an upcoming paper.

## Arduino
**HoldDown.ino** - This is the flight computer code used for the hold down tests. It includes updated orientation and pid control algorithms. Prior to the test, a button can be pressed to calibrate the orientation based on the gravity vector read by the flight computer's accelerometer. During the test, the orientation is determined based on gyroscope rates.

**StateMachine.ino** - This the unfinished flight computer code, which runs a number of states based on the portion of flight the rocket is in (e.g. pad, powered ascent, unpowered ascent, descent, landed). This control system allows orientation to be calibrated based on the accelerometer on the launchpad, activate thrust-vector-control during powered ascent, eject the parachute at apogee, and download telemetry data from the flash chip to the sd card upon landing. I am currently writing code for many of these states.

**TVCTest.ino** - This is the old flight computer code. It collects data from the inertial measurement unit, altimeter and converts that to angles that are commanded to the motor mount servos in order to achieve a desired motor angle.

**fairingTest.ino** - This is the code that I use to test the third design of my fairing ejection mechanism. It simply alternates the fairing servo between 90 degrees and 0 degrees so the fairing can be loaded and subsequently ejected.

**servoCalibration.ino** - This is the code I use to zero the rocket's servos. It reads the potentiometer on the flight computer and converts that to a PWM signal that commands a servo to a corresponding angle.

**servoTest.ino** - This is the code I use to test the parameters for the motor mount servo's limits and zeros.


## Matlab & Simulink
**F15TC.xlsx** - This is a table representing the thrust curve for the F15-0 motor I intend to use in my rocket. The data was retrieved from [thrustcurve.org](https://www.thrustcurve.org/motors/Estes/F15/) and is used in my Simulink simulation.

**ThrustCurve.m** - This program reads **F15TC.xlsx** so that it can be used in the simulation.

**Simulation.slx** - This is a Simulink simulation that simulates my rocket's flight in 2 dimensions. It implements a PID controller that commands a motor mount angle in one axis.

**RHInterp.m** - This program interpolates 5 relative humidity (RH) readings at various points in a humidity hood to fill in an RH map of the rest of the hood. This program was used to determine the effectiveness of the hood that provides a high-humidity reservoir to my lab's evaporation engines.

**motionTracking.m** - This program tracks the rotation of a red dot around a blue dot. By placing a disk with such dots on the axle connected to my lab's evaporation engines, I was able to collect detailed rotation data of our engines.

**maskTest.m** - This prorgram is for calibrating the color mask used in **motionTracking.m** to locate the red and blue dots.


## Python
**catchData.py** - This program reads a serial port that my flight computer is connected to, printing the results. It was used to ensure data was being read properly.

**liveQuat.py** - This program reads attitude data in the form of Euler angles from my flight computer over a serial port, displaying a model that rotates depending on the flight computer's attitude. This was used to test if the flight computer was converting between quaternion orientation data and Euler angles properly.

