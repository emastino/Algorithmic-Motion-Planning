Map.cpp

To run Map.cpp you must have the Eigen library for matrix algebra: 
#include "Eigen/Dense" //using Dense inside Eigen

When compiling Map.cpp  you must specify where the Eigen library lives
on your system. The command line command looks like:

g++ -I "C:\Users\person1" Map.cpp -o test

You must have an obstacle txt file to run this code. The obstacle files are ordered 
such that the each line has a set of points that define a polygon.

When the code is run it will out put a text file witht he location of each vertex 
for each polygon and the angle of the robot for that polygon. The text file is
then used in the cSpace_Ex1.m MATLAB file to vusualize the results. To run this code,
make sure that the apporpiate file is in the current directory of the MATLAB script.

---------------------------------------------------------------------------------------
arm_cSpace.cpp

To run arm_cSpace.cpp you must have the Eigen library for matrix algebra: 
#include "Eigen/Dense" //using Dense inside Eigen

When compiling arm_cSpace.cpp  you must specify where the Eigen library lives
on your system. The command line command looks like:

g++ -I "C:\Users\person1" arm_cSpace.cpp -o arm

You must have an obstacle txt file to run this code. The obstacle files are ordered 
such that the each line has a set of points that define a polygon.

You can uncomment blocks of code to either run Workspace a), b), or c). arm_cSpace.cpp
outputs a text file, cSpace_Ex_3_X.txt. Each line of this txt file has the value of 
theta_1, theta_2, and whether a collision occured (0) or not (1). 

The text file is then used in the cSpace_Ex3.m MATLAB file to vusualize the results. To run 
this code, make sure that the apporpiate files are in the current directory of the 
MATLAB script.This script expects you to have all three C-Space file in the working directory 
at the same time.


---------------------------------------------------------------------------------------
Ex_2_Arm.m

Ex_2_Arm.m has two ways it can run, you can either run it in foward kinematics mode or 
inverse kinematics mode. To run it as forward kinemtaics, set the variable "FK =1". You 
can specify the bar lengths with the "a" variable. For inverse kinemtaics mode, set "FK=0"
and change the "goal" variable to whatever you wish. The code will tell you if the point 
is reachable or not. In the inverse kinematics function calls the IK.m function which 
figures out all of the inverse kinemtaics based on the goal location arm geometry. 

