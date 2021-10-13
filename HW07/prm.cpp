// Discrete Map.cpp : Defines the entry point for the application.
//

#include <iostream>
#include <algorithm>
#include <list>
#include <vector>
#include <iterator>
#include <tuple>
#include <cmath>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <chrono>
#include <windows.h>

// dense is part of the Eigen matrix library, when using it, make sure to Include
// the path to it in the compile command:
//
// g++ -I "D:\Users\emast\Personal\Personal Cpp" Map.cpp -o test
//
#include "Eigen/Dense" //using Dense inside Eigen

#include "Map.h"

using namespace Eigen;
using namespace std;
 using namespace std::chrono;

// make obstacle function
std::vector<polygon> makeObstacles(string fileName){
	// read points in from obstacles.txt and make a vector of obstacles
	// This will be a vector of polygons, which in turn a vector of positions
	std::vector<polygon> obstacles;

	string line; // whole line of text file
	int count = 1;
	vertex ver;
	// read in obstacle file
	ifstream myfile(fileName);
	while ( getline (myfile,line) ){

		std::istringstream ss( line);

		// create new polygon object to be added
		polygon poly;

		while (ss){
      string s; // string that is seperated by comma

      if (!getline( ss, s, ',' )) break; // if no more commas, break while

			if(count ==1){ // get x value
				// add x value of point
				ver.x = std::stof(s);
				count = 2;
			}
			else{ // get y value
				// add y value of point
				ver.y = std::stof(s);
				// add vertex to the polygon in obstacles
				poly.vertices.push_back(ver);
				count =1;
			}



    }
		obstacles.push_back(poly); // push the polygon into obstacles vector
	}
	// close file
	myfile.close();

	return obstacles;
}





int main()
{


	//////////////////////////////////////////////////////////////////////////////
	// Exercise 1 a)
	//
	//////////////////////////////////////////////////////////////////////////////
	// map limits
	std::tuple<double, double> xLim = std::make_tuple(-1,11);
	std::tuple<double, double> yLim = std::make_tuple(-3,3);
	// map start and goal
	std::tuple<double, double> start = std::make_tuple(0.0,0.0);
	std::tuple<double, double> goal = std::make_tuple(10.0,0.0);


	std::vector<polygon> obstacles = makeObstacles("obstacles_a.txt");

	// make a map object
	map mapa(xLim, yLim, start, goal, obstacles);


	// file names
	ofstream PRMBenchmarkFile_VS("PRM_Benchmark_validSol.txt"); // valid solutions
	ofstream PRMBenchmarkFile_PL("PRM_Benchmark_pathLen.txt");	// path lengths
	ofstream PRMBenchmarkFile_CT("PRM_Benchmark_compTime.txt"); // compile time

	int numNode[8] = {200,200,200,200,500,500,500,500};
	double rad[8] = {0.5,1,1.5,2,0.5,1,1.5,2};


	for(int j = 0; j<8 ;j++){
		// cout << "n = " << numNode[j] << endl;
		// cout << "r = " << rad[j] << endl;
		for (int i = 0 ; i <100 ; i++){

			PRM_Benchmark pm = mapa.PRM(numNode[j],rad[j], false,false);

			PRMBenchmarkFile_VS << pm.validSolution << ", " ;
			PRMBenchmarkFile_PL << pm.pathLength << ", " ;
			PRMBenchmarkFile_CT << pm.compTime << "," ;
			Sleep(1000);
		}

		PRMBenchmarkFile_VS << endl;
		PRMBenchmarkFile_PL << endl;
		PRMBenchmarkFile_CT << endl;
	}

	PRMBenchmarkFile_VS.close();
	PRMBenchmarkFile_PL.close();
	PRMBenchmarkFile_CT.close();

	// PRM_Benchmark pm = mapa.PRM(1000,0.5, true,true);



	return 0;
}
