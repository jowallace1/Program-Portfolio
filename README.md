# MITMakerPortfolio
A repository containing any programs I developed for my thrust-vector-controlled rocket project.

## Arduino
**TVCTest.ino** - This is the current flight computer code. It collects data from the inertial measurement unit, altimeter and converts that to angles that are commanded to the motor mount servos in order to achieve a desired motor angle.

**fairingTest.ino** - This is the code that I use to test the third design of my fairing ejection mechanism. It simply alternates the fairing servo between 90 degrees and 0 degrees so the fairing can be loaded and subsequently ejected.

**servoCalibration.ino** - This is the code I use to zero the rocket's servos. It reads the potentiometer on the flight computer and converts that to a PWM signal that commands a servo to a corresponding angle.

**servoTest.ino** - This is the code I use to test the parameters for the motor mount servo's limits and zeros.


## Matlab & Simulink
**F15TC.xlsx** - This is a table representing the thrust curve for the F15-0 motor I intend to use in my rocket. The data was retrieved from [thrustcurve.org](https://www.thrustcurve.org/motors/Estes/F15/) and is used in my Simulink simulation.

**ThrustCurve.m** - This program reads **F15TC.xlsx** so that it can be used in the simulation.

**Simulation.slx** - This is a Simulink simulation that simulates my rocket's flight in 2 dimensions. It implements a PID controller that commands a motor mount angle in one axis.


## Python
**catchData.py** - This program reads a serial port that my flight computer is connected to, printing the results. It was used to ensure data was being read properly.

**liveQuat.py** - This program reads attitude data in the form of Euler angles from my flight computer over a serial port, displaying a model that rotates depending on the flight computer's attitude. This was used to test if the flight computer was converting between quaternion orientation data and Euler angles properly.

