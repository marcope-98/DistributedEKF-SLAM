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
- [Actuator.m](include/documentation.md#Actuator)
- [Agent.m](include/documentation.md#Agent)
- [ekfSLAM.m](include/documentation.md#ekfSLAM)
- [Message.m](include/documentation.md#Message)
- [Sensor.m](include/documentation.md#Sensor)
- [Server.m](include/documentation.md#Server)