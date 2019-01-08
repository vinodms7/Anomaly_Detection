# Anomaly_Detection

#Synopsis

Anomaly_Detection application detects the anomaly siganals with respect to Door's position and Vechicle speed at Particlular timestamp. This package consist of four node 
1. Input Node[publishes the CAN signal ]
2. Detection Node[subscribe the CAN signal and identify the valid and Anomaly signals ]
3. Result Node[Displays the Anomaly signals at particular timestamp]
4. Test Node [Subscribs the Anomaly signals and test the working of Detection node]


C++ is used as the development language. The following are considered in designing this application. 
 - Modularity
 - Configurability
 - Re-useability

#Installation

 - Copy the "src" folder to your catkin workspace
 - Open terminal and go to "src" folder. Enter "catkin_init_workspace" command.
 - Once CMakeList.txt is created in src, go back to workspace folder in terminal 
 - Enter "catkin_make install" command in terminal. 
 - After build completes, Enter "source devel/setup.bash" command in terminal
 - Enter "roslaunch ros_number_generator ros_number_generator_multiplier.launch" in terminal. This will create all the nodes and rqt_graph,   rqt_plot, ros console. 

## Requirements

- ROS kinetic (Ubuntu 16.04)
- Catkin minimum VERSION 2.8.3
- C++11


#Test

 - Open a new terminal and go to the application workspace
 - Enter "source devel/setup.bash"
 - Enter catkin_make run_tests

