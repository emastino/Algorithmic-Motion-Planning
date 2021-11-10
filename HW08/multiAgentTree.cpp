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
// g++ -I "D:\Users\emast\Personal\Personal Cpp" multiAgentTree.cpp -o mat
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



const char* directory = "D:/";
const char* fileName_1 = "Exercise_1_m_";
const char* fileName_2 = "Exercise_2_m_";
const char* fileType = ".txt";

char name_buffer[512];
FILE* f = NULL;

int main()
{



  // Make Map
	std::tuple<double, double> xLim = std::make_tuple(-1,17);
	std::tuple<double, double> yLim = std::make_tuple(-1,17);
	// map start and goal. These don't matter now
	std::tuple<double, double> start = std::make_tuple(0.0,0.0);
	std::tuple<double, double> goal = std::make_tuple(0.0,0.0);

  // make obstacles
	// std::vector<polygon> obstacles = makeObstacles("obstacles.txt");

  // no obstacles
  std::vector<polygon> obstacles;

  // make robots
  int activeBots = 16;
  vertex startArray[16] = { {-1,2}, {-1,17}, {2,-1}, {5,-1}, {15,17}, {17,6} ,{-1,5}, {8,-1}, {10,-1}, {-1,5}, {3,17}, {5,17}, {12,-1}, {-1,10}, {9,17}, {17,4}};
  vertex goalArray[16] = { {6,9.5}, {6,8.75}, {6,7.25}, {6,6.5}, {6.75,8}, {6,8},{7.5,8}, {8.25,8},{8.25,9.5}, {8.25,8.75}, {8.25,7.25}, {8.25,6.5}, {9.75,9.5}, {9.75,8}, {9.75,7.25}, {9.75,6.5}};

  std::vector<robot> robotVector;

  for (int i = 0; i < activeBots ; i++){

    robot tempRob;
    tempRob.robotNumber = i;
    tempRob.start = startArray[i];
    tempRob.goal = goalArray[i];
    tempRob.R = 0.25;

    // cout << startArray[i].x << ", " << startArray[i].y << endl;

    robotVector.push_back(tempRob);
  }
	//////////////////////////////////////////////////////////////////////////////
	// Exercise 1
	//
	//////////////////////////////////////////////////////////////////////////////


	// make a map object
	// map mapa(xLim, yLim, start, goal, obstacles);




  // mapa.addRobots(robotVector);

  // PRM_Benchmark pm = mapa.GoalBiasRRT_Centralized(7500,0.5,0.05,0.25,true,false, 6); //(int n,double r,double p,double epsilon, bool outputPath, bool outPutTree, int numOfRobs)
  // int from = 1;
  // int to = 6;
	// for(int j = from; j <= to ; j++){
	// 	// cout << "n = " << numNode[j] << endl;
	// 	// cout << "r = " << rad[j] << endl;
  //   // file names
  //   sprintf(name_buffer,"%s%d%s",fileName_1,j,fileType);
  // 	ofstream benchmark(name_buffer); // valid solutions
  //
  //
	// 	for (int i = 0 ; i <100 ; i++){
  //     mapa.addRobots(robotVector);
	// 		pm = mapa.GoalBiasRRT_Centralized(7500, 0.5, 0.05, 0.25, false, false, j);
	// 		benchmark << pm.validSolution <<  ", " <<  pm.compTime  << endl ;
  //     // Sleep(1000);
	// 	}
  //   cout << "Finished Benchmarking For " << j << " Robot(s)" << endl;
  //
  //   benchmark.close();
	// }

  //////////////////////////////////////////////////////////////////////////////
	// Exercise 2
	//
	//////////////////////////////////////////////////////////////////////////////


  // make a map object
	map mapa2(xLim, yLim, start, goal, obstacles);
  mapa2.addRobots(robotVector);

  PRM_Benchmark pm2 = mapa2.GoalBiasRRT_Decentralized(10000000,0.3,0.05,0.25,true,false, 16); // path file for two robots

  // // PRM_Benchmark pm;
	// for(int j = from; j <= to ; j++){
	// 	// cout << "n = " << numNode[j] << endl;
	// 	// cout << "r = " << rad[j] << endl;
  //   // file names
  //   sprintf(name_buffer,"%s%d%s",fileName_2,j,fileType);
  // 	ofstream benchmark(name_buffer); // valid solutions
  //
  //
	// 	for (int i = 0 ; i <100 ; i++){
  //     // mapa2.addRobots(robotVector);
	// 		pm2 = mapa2.GoalBiasRRT_Decentralized(7500, 0.5, 0.05, 0.25, false, false, j);
	// 		benchmark <<  pm2.compTime << ", "  << endl ;
  //     // Sleep(1000);
	// 	}
  //   cout << "Finished Benchmarking For " << j << " Robot(s)" << endl;
  //
  //   benchmark.close();
	// }




	return 0;
}
