## Hindbrain

The Edwin hindbrain deals with the robot's motor and sensor commands at the lowest level. For most applications, this will mostly concern serial protocols and communication with micro-controller devices. Following the Sense/Think/Act design scheme, the modules in this folder are as follows:

#### Sense
Modules in this folder deal with transforming raw data from serial sensors like IR sensor, e-stops, encoders, or battery monitors into a form usable for analysis in other scrips. Sensor information retrieved from this level are in coordinates tied to the robot directly.

#### Think
The Think protocol at the hindbrian level should be extremely fast binary decisions, as in "e-stop or no" or "slow down or no". There is no additional computation happening at this level, as these are the equivalent of "knee-jerk" reactions for a robot.

#### Act
Code executing commands issued from the Act protocol of the midbrain level resides here. Code at this level is only aware of the immediate state of the robot as additional information has been abstracted out. Scripts that execute motor and sensor commands are examples of modules belonging here.
