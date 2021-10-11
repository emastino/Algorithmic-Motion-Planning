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

	mapa.PRM(200, 1);
	// Sleep(1000);
	// mapa.PRM(200, 0.5);

	// use map object to check obstacle clollisions to build PRM

	// go back and update the connections in the graph (find neighbors for each node)


	// Use Graph class to do a graph search and find paths



	// for loop that runs each imulation 100 times and collects benchmark data






	return 0;
}
