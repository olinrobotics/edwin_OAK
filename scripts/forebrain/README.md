## Forebrain

The Edwin forebrain deals with the robot's interaction with the world frame.
This is the highest level of computation, and deals with the machine learning levels of behavior. Following the Sense/Think/Act design scheme, the modules in this folder are as follows:

#### Sense
Modules in this folder deal with transforming raw data from the midbrain into the world frame. For all intents and purposes in the Edwin context, the world frame is the same as the base of the STR17 arm.

#### Think
Deliberation of future steps occurs at the think level. Neural nets, clustering algorithms and cloud based learning code should be placed in this folder.

#### Act
Code detailing the desired global state of the robot to be passed down to the Act level of the midbrain. Inputs are passed in from the Think module, and constants that refer to the current global state of the robot are also published from code in this folder.
