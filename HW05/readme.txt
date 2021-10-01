gradient.cpp

To run gradient.cpp you must have the Eigen library for matrix algebra: 
#include "Eigen/Dense" //using Dense inside Eigen

When compiling Map.cpp  you must specify where the Eigen library lives
on your system. The command line command looks like:

g++ -I "C:\Users\person1" gradient.cpp -o grad

You must have an obstacle txt file to run this code. The obstacle files are ordered 
such that the each line has a set of points that define a polygon.

When the code is run it will out put a text file witht he location of each vertex 
for each polygon and the angle of the robot for that polygon. The text file is
then used in the Gradient_Path.m MATLAB file to vusualize the results. To run this code,
make sure that the apporpiate file is in the current directory of the MATLAB script.

You need to uncomment the different blocks of code to run Exercise 2 parts a) and the 
two sections of part b). Each section uses different obstacle files "obstacles_x.txt" 
where x can either be a, b, or c for Exercise 2.a, Exercise 2.b Workspace 1, and 
Exercise 2.b Workspace 2. 

The output of the script is a txt file of the form "gradientPath_x.txt" with the path 
of the robot, where x corresponds to the a,b, and c from above. The file name for this 
txt file can be changed with in the Map.h file in the function "gradientDecentPath". 

once you have compiled and set up your workspace, in the command line type "grad" and 
press enter.

---------------------------------------------------------------------------------------
Gradient_Path.m

Once you run your workspace in gradient.cpp, transfer the ouput files to the working 
directive of this file and all of the figures will be plotted. 

WARNING: The vector field code is SUPER sensitive to changes in dimensions and plots of 
the vector field must be opened full screen to appear properly. 

