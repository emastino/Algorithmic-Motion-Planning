paths.cpp

To run gradient.cpp you must have the Eigen library for matrix algebra: 
#include "Eigen/Dense" //using Dense inside Eigen

When compiling Map.cpp  you must specify where the Eigen library lives
on your system. The command line command looks like:

g++ -I "C:\Users\person1" paths.cpp -o paths

RUNNING EXERCISE 1
You must have an obstacle txt file to run this code. The obstacle files are ordered 
such that the each line has a set of points that define a polygon. 
You need to uncomment the different blocks of code to run Exercise 1 Workspace 1 and the 
uncomment the "Exercise 1 Workspace 1" section. To run Exercise 1 Workspace 2 and the 
uncomment the "Exercise 1 Workspace 2" section. Each section uses different obstacle files "obstacles_x.txt" 
where x can either be b and c for  Workspace 1 and Workspace 2, respectively.


When the code is run it will out put a text file witht he location of each vertex 
for each polygon and the angle of the robot for that polygon. The text file is
then used in the Gradient_Path.m MATLAB file to vusualize the results. To run this code,
make sure that the apporpiate file is in the current directory of the MATLAB script.


The output of the script is a txt file of the form "wavefrontPath_x.txt" with the path 
of the robot, where x corresponds to the b and c from above. 

Once you have compiled and set up your workspace, in the command line type "path" and 
press enter.

RUNNING EXERCISE 2
You must have an obstacle txt file to run this code. The obstacle files are ordered 
such that the each line has a set of points that define a polygon. 
You need to uncomment the Exercise 2 Manipulator section. Each section uses different 
obstacle files "obstacles_Ex_x.txt" where x can either be a,b, and c for  Workspace 1, 
Workspace 2, and Workspace 3, respectively.

You need to name the C-space file name "cSpace_Ex_3_x.txt" accordingly (or anything you
want really) and you can name the wavefront path file name anything you want, but I named
it "wavefrontPath_cSpace_Ex2_x.txt". For both x can either be a,b, and c for  Workspace 1, 
Workspace 2, and Workspace 3, respectively.


RUNNING EXERCISE 3
As it it stand, this section, Exercise 3 A*/Dijkstra's Search,  is uncommented. This
section adds all of the nodes, weights, and connections by hand. At the end both search algorithms are run. 

---------------------------------------------------------------------------------------
Gradient_Path.m

Once you run Exercise 1 in paths.cpp, transfer the ouput files, "wavefrontPath_x.txt" to the working 
directive of this file and all of the figures will be plotted. 

---------------------------------------------------------------------------------------
cSpace_Ex2.m

Once Exercise 2 has been run, transfer yor obstacle, C-space obstacle files, and wavefront
path files to the MATLAB workspace and run them ensuring that the name smatch up with
thos ein the script. 
