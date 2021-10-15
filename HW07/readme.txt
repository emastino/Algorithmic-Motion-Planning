prm.cpp

To run prm.cpp you must have the Eigen library for matrix algebra: 
#include "Eigen/Dense" //using Dense inside Eigen

When compiling Map.cpp  you must specify where the Eigen library lives
on your system. The command line command looks like:

g++ -I "C:\Users\person1" paths.cpp -o prm


Once you have compiled and set up your workspace, in the command line type "path" and 
press enter.

----------------------------------------------------------------------------------------------
RUNNING EXERCISE 1
Uncomment the blocks of code labled "Exercise 1 a)" or "Exercise 1 b) Worspace X" 
where X can be either 1 or 2. 

The PRM() function is called here as:
	PRM(n,r, bool1,bool2, bool3);

where n is the number of attempted nodes, r is the neighbor radius, bool1 if 'true'
outputs the path if one is found as "PRM_Path.txt", bool2 if 'true' outputs the 
entire PRM as "PRM_Graph.txt". If Bool3 is true, the path gets smoothed.

Additionally, the benchmar code for both the regular and smothed paths are run here and the 
reults are output as these 6 txt files:
	"PRM_Benchmark_validSol.txt"
	"PRM_Benchmark_pathLen.txt"
	"PRM_Benchmark_compTime.txt"
	"PRM_Benchmark_validSol_Smooth.txt"
	"PRM_Benchmark_pathLen_Smooth.txt"
	"PRM_Benchmark_compTime_Smooth.txt"); 

These are then passed into the Ex_1.m MATLAB file to plot the path, PRM, map, and boxplots

Requires file:
"obstacles_a.txt"
"obstacles_b.txt"
"obstacles_c.txt"

----------------------------------------------------------------------------------------------
RUNNING EXERCISE 2
Uncomment the blocks of code labled "Exercise 2 a) Worksapce 1" or "Exercise 2 a) Worksapce 2".
These sections also run part 

The GoalBiasRRT() function is called here as:
	GoalBiasRRT(n, r ,p,e, bool1, bool2);

where n is the number of attempted nodes, r is the max distance from q_near, bool1 if 'true'
outputs the path if one is found as "Tree_Path.txt", bool2 if 'true' outputs the 
entire Tree as "Tree.txt". 

Additionally, the benchmar code for both the regular and smothed paths are run here and the 
reults are output as these 6 txt files:
	"Tree_Benchmark_validSol.txt"
	"Tree_Benchmark_pathLen.txt"
	"Tree_Benchmark_compTime.txt"
	"Tree_Benchmark_validSol_Smooth.txt"
	"Tree_Benchmark_pathLen_Smooth.txt"
	"Tree_Benchmark_compTime_Smooth.txt"); 

These are then passed into the Ex_2.m MATLAB file to plot the path, tree, map, and boxplots

Requires file:
"obstacles_b.txt"
"obstacles_c.txt"



