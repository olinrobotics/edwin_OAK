## Midbrain

The Edwin midbrain deals with the robot's motions in its local frame.
On this level, the robot interprets abstract commands from the forebrain into commands for the hindbrain to execute. An example of an interaction at this level would be wall following or obstacle avoidance. Following the Sense/Think/Act design scheme, the modules in this folder are as follows:

#### Sense
Modules in this folder deal with receiving data from the sense layer of the hindbrain and from more complex sensors. Camera, sonar, or lidar code belongs in this folder. At this level, sensor information is still in the frame of the robot.

#### Think
Reactive robotic behavior occurs at this level. Obstacle avoidance, target following, or complex route actions belong in this folder. Modules in at this level deal with a single goal only, such as "picking up a pen" or "drawing a number" and do not have insight into future planned behaviors or any machine learning protocols.

#### Act
Code detailing the immediate to near-immediate local state of the robot exists at this level. Code that allows the robot to execute rote actions like greeting behaviors or object manipulation code belongs here. Information passed from the Act module at the midbrain level to the Act at the hindbrain level should be very granular, i.e: "rotate write 90 degrees" or "run behavior greet_human"
