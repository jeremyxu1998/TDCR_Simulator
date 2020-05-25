## TACR Simulator

This is a simulation program for Tendon Actuated Continuum Robots (TACR) in CRL. 
It provides a visualization of forward kinematics of one or multiple robots given their configuration and input parameters, following constant curvature model.

The GUI input of program support maximum 3 robots, 3 segments and 4 tendons for each robot.

### Software Requirement

- C++11 or later
- Qt 5.12 or later
- VTK 8.2.0
- Eigen 3

### Usage

- Open tendon_robot_simulator.pro (using Qt Creator or Qt Visual Studio Tools)
- Compile the program
- Set the robot(s) cofiguration in an xml file and put it under root folder (use test_robot.xml or test_multiple_robots.xml as example)
- Run the program
    - Select robot configuration file in the poped up window
    - Enter the tendon length input for each robot
    - Click "Calculate" button or hit Enter key to start the forward kinematics animation