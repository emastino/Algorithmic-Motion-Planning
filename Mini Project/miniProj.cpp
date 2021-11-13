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
#include<stdio.h>

// dense is part of the Eigen matrix library, when using it, make sure to Include
// the path to it in the compile command:
//
// g++ -I "D:\Users\emast\Personal\Personal Cpp" miniProj.cpp -o MP
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

// constants
#define PI 3.14159265
const char* directory = "D:/";
const char* fileName_1 = "Exercise_1_m_";
const char* fileName_2 = "Exercise_2_m_";
const char* fileType = ".txt";

char name_buffer[512];
FILE* f = NULL;

int main()
{

	// start sound
	Beep(500,100);
	Beep(600,100);
	Beep(700,100);
  // Make Map
	std::tuple<double, double> xLim = std::make_tuple(0,40);
	std::tuple<double, double> yLim = std::make_tuple(0,40);
	// map start and goal. These don't matter now
	std::tuple<double, double> start = std::make_tuple(0.0,0.0);
	std::tuple<double, double> goal = std::make_tuple(0.0,0.0);

  // make obstacles
	std::vector<polygon> obstacles = makeObstacles("obstacles.txt");

  // // no obstacles
  // std::vector<polygon> obstacles;

	// make map object
	map mapa(xLim, yLim, start, goal, obstacles);

	// define state bounds
	MatrixXd stateBounds(8,2);
	stateBounds<< 0.0,					40.0,
								0.0,					40.0,
								0.0, 					2*PI,
								-1.0/6.0, 		1.0/2.0,
								-PI/6.0, 			PI/6.0,
								0.0, 					2*PI,
								0.0, 					2*PI,
								0.0, 					2*PI,


	cout << "State Bounds: \n" << stateBounds << endl;


	// define start position for the robotsTree
	Matrix<double,8,1> startConfig;
	startConfig << 11.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
	cout << "Start Config: \n" << startConfig << endl;

	// define the goal regiosn (low,high)
	MatrixXd goalRegion(8,2);
	goalRegion << 0.0,					6.0,
								34.0,					38.0,
								8.0*PI/10.0, 	10.0*PI/9.0,
								-1.0/40.0, 		1.0/40.0,
								-PI/6.0, 			PI/6.0,
								8.0*PI/10.0, 	10.0*PI/9.0,
								8.0*PI/10.0, 	10.0*PI/9.0,
								8.0*PI/10.0, 	10.0*PI/9.0;

	cout << "Goal Region: \n" << goalRegion << endl;


	mapa.GoalBiasRRT_Trailer(25000, 0.05, startConfig, goalRegion, stateBounds, true);

	// //test collision
	//
	// vertex temp1,temp2,temp3,temp4; // this variable will be abused :(
	// polygon tempPoly;
	// tempPoly.vertices.resize(4);
	//
	// // hitch lengths
	// double d = 2.5;
	//
	//
	//
	// temp1 = {2,2};
	// tempPoly.vertices[0] = temp1;
	// temp2 = {5,2};
	// tempPoly.vertices[1] = temp2;
	// temp3 = {5,12};
	// tempPoly.vertices[2] = temp3;
	// temp4 = {2,12};
	// tempPoly.vertices[3] = temp4;
	//
	// if(mapa.polygonObstacleCollision(tempPoly)){cout << "bing" << endl; }
	//
	// cout << "hi main" << endl;

	// end sound
	Beep(700,100);
	Beep(600,100);
	Beep(500,100);


	return 0;
}
