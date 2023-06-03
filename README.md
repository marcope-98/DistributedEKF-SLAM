# DSMA-Assignment
Assignment for the course "140472 - Distributed Systems for Measurement and Automation"

## Resources
- ***Probabilistic Robotics (Intelligent Robotics and Autonomous Agents)***. Sebastian Thrun, Wolfram Burgard, and Dieter Fox. 2005. 
- ***Distributed Cooperative SLAM using Information Consensus Filter***. Rajnikant Sharma, Clark N. Taylor, David W. Casbeer
- ***Decentralised SLAM with Low-Bandwidth Communication for Teams of Vehicles***. Eric Nettleton et al.
- [UTIAS Multi-Robot Cooperative Localization and Mapping Dataset](http://asrl.utias.utoronto.ca/datasets/mrclam/index.html)

## Repository Structure
- `./assets/` folder contains 9 datasets for the assignment
- `./include/` folder contains the implementation of the classes used for the simulation
- `./utils/`   folder contains miscellaneous function for loading, resampling and animating the dataset

The root folder contains three main scripts: `./main.m`, `./init.m` and `./config.m`
- `./config.m` contains a Matlab structure with the user defined simulation parameters
- `./init.m`   loads the dataset, it resamples it and initializes the Agents in the simulation
- `./main.m`   is the file to execute to perform the simulation. It contains the main simulation loop and a call to the animation function

## Documentation:

### Classes

- [Actuator.m](doc/classes.md#Actuator)
- [Agent.m](doc/classes.md#Agent)
- [ekfSLAM.m](doc/classes.md#ekfSLAM)
- [Message.m](doc/classes.md#Message)
- [Sensor.m](doc/classes.md#Sensor)
- [Server.m](doc/classes.md#Server)

### Helper Functions
- [cvt_rb_to_xy.m](doc/functions.md#cvt_rb_to_xy)
- [cvt_xy_to_rb.m](doc/functions.md#cvt_xy_to_rb)
- [find_transformation.m](doc/functions.md#find_transformation)
- [load_config.m](doc/functions.md#load_config)
