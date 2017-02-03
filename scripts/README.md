#Edwin OAK - Code Architecture

This repository is the second version of the original code, located [here](https://github.com/olinrobotics/edwin).

##Architecture
Code architecture for Edwin OAK follows the OAK architecture developed by Dave Barret at Olin College of Engineering. It follows a multi-layered decision scheme with three vertical levels and three horizontal levels.

The vertical levels are modeled after the human brain, divided into the forebrain, midbrain, and hindbrain.
The horizontal levels quantize the steps a robot must take to get from sensor data to motor commands and are divided into sense, think, and act.

Please refer to the READMEs found the corresponding folders for a more detailed breakdown of the expected modules found in each level of computation.

`brain.py` found in the topmost level of this folder is the central code that ties all the modules together. Please refer to this code when attempting to initially understand the dataflow that Edwin OAK utilizes between layers and modules.
