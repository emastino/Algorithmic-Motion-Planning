
REQUIRES: Map.h header file

Main script
multiAgentTree.cpp

To run multiAgentTree.cpp you must have the Eigen library for matrix algebra: 
#include "Eigen/Dense" //using Dense inside Eigen

When compiling Map.cpp  you must specify where the Eigen library lives
on your system. The command line command looks like:

g++ -I "C:\Users\person1" multiAgentTree.cpp -o mat


Once you have compiled and set up your workspace, in the command line type "mat" and 
press enter. The main script can be ran in one run to get all data sets for this lab. 

Needed Files:
obstacles.txt - stores data for the obstacles
----------------------------------------------------------------------------------------------
MATLAB Files for Plotting 

The main script outputs several files with data on the paths and benchmarks. Move these
files to the apporpiate matlab directory and run the following MATLAB files:

	1) Ex_1.m - Exercise 1
	2) Ex_2.m - Exercise 2
These two files will generate all of the appropriate plots and save the images automatically. 
The naming convention makes clear what question the plot belongs to: 
"Ex_x_y...txt/jpeg"
	x - exercise number
	y - which sub question 
	


