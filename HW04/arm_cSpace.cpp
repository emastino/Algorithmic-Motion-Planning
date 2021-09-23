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

// dense is part of the Eigen matrix library, when using it, make sure to Include
// the path to it in the compile command:
//
// g++ -I "D:\Users\emast\Personal\Personal Cpp" Map.cpp -o test
//
#include "Eigen/Dense" //using Dense inside Eigen
#include "Map.h"


using namespace Eigen;
using namespace std;

int main()
{

	// map limits
	std::tuple<double, double> xLim = std::make_tuple(-20,20);
	std::tuple<double, double> yLim = std::make_tuple(-30,30);
	// map start and goal
	std::tuple<double, double> start = std::make_tuple(0.0,0.0);
	std::tuple<double, double> goal = std::make_tuple(10.0,10.0);

  // //////////////////////////////////////////////////////////////////////////////
  // //////////////////////////////////////////////////////////////////////////////
  //
  // // Exercise 3 part a
  //
  // //////////////////////////////////////////////////////////////////////////////
	// // read points in from obstacles.txt and make a vector of obstacles
	// // This will be a vector of polygons, which in turn a vector of positions
	// std::vector<polygon> obstacles_1;
  //
	// string line; // whole line of text file
	// int count = 1; // helps get pairs of 2
	// vertex ver;
	// // read in obstacle file
	// ifstream myfile("obstacle_Ex_3_a.txt");
	// while ( getline (myfile,line) ){
  //
	// 	std::istringstream ss( line);
  //
	// 	// create new polygon object to be added
	// 	polygon poly;
  //
	// 	while (ss){
  //     string s; // string that is seperated by comma
  //
  //     if (!getline( ss, s, ',' )) break; // if no more commas, break while
  //
	// 		if(count ==1){ // get x value
	// 			// add x value of point
	// 			ver.x = std::stof(s);
	// 			count = 2;
	// 		}
	// 		else{ // get y value
	// 			// add y value of point
	// 			ver.y = std::stof(s);
	// 			// add vertex to the polygon in obstacles
	// 			poly.vertices.push_back(ver);
	// 			count =1;
	// 		}
  //
  //
  //   }
  //
	// 	obstacles_1.push_back(poly); // push the polygon into obstacles vector
	// }
	// myfile.close();
  //
  //
	// // make a map object
	// map map_1(xLim, yLim, start, goal, obstacles_1);
  //
  // double delta_theta = 0.5; // number of points from 0-360 degrees
  //
  // double arm_1_length = 1;
  // double arm_2_length = 1;
  //
  // double theta_1 = 0;
  // double theta_2 = 0;
  //
  // vertex origin, arm1,arm2;
  // origin.x = 0; origin.y =0;
  //
  // int collisionStatus = 1; // collision 1 means no collision occured
  // // map_1.printPolygons();
  //
  //
  // ofstream cSpaceFile ("cSpace_Ex_3_a.txt");
  //
  // while(theta_1 <= 360){
  //
  //   while(theta_2 <= 360){
  //
  //     // make function to get position of points
  //     arm1.x = arm_1_length*cos(theta_1*PI/180.0);
  //     arm1.y = arm_1_length*sin(theta_1*PI/180.0);
  //
  //     // check collision of arm 1
  //     if(map_1.lineCollision(origin, arm1)){
  //       collisionStatus = 0; // collision occured
  //     }
  //
  //     arm2.x = arm1.x + arm_2_length*cos((theta_1+theta_2)*PI/180.0);
  //     arm2.y = arm1.y + arm_2_length*sin((theta_1+theta_2)*PI/180.0);
  //
  //     // check collision of arm 2
  //     if(map_1.lineCollision(arm1, arm2)){
  //       collisionStatus = 0; // collision occured
  //     }
  //
  //     cSpaceFile << theta_1 << "," << theta_2 << "," << collisionStatus << endl;
  //
  //     collisionStatus  = 1 ;
  //     theta_2 = theta_2 + delta_theta;
  //
  //   }
  //
  //   theta_2 = 0;
  //   theta_1 = theta_1 + delta_theta;
  // }
  //
  // cSpaceFile.close();



  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////

  // Exercise 3 part b

  //////////////////////////////////////////////////////////////////////////////
	// read points in from obstacles.txt and make a vector of obstacles
	// This will be a vector of polygons, which in turn a vector of positions
	std::vector<polygon> obstacles_2;

	string line; // whole line of text file
	int count = 1; // helps get pairs of 2
	vertex ver;
	// read in obstacle file
	ifstream myfile("obstacle_Ex_3_b.txt");
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

		obstacles_2.push_back(poly); // push the polygon into obstacles vector
	}
	myfile.close();


	// make a map object
	map map_2(xLim, yLim, start, goal, obstacles_2);

  double delta_theta = 0.5; // number of points from 0-360 degrees

  double arm_1_length = 1;
  double arm_2_length = 1;

  double theta_1 = 0;
  double theta_2 = 0;

  vertex origin, arm1,arm2;
  origin.x = 0; origin.y =0;

  int collisionStatus = 1; // collision 1 means no collision occured
  // map_1.printPolygons();


  ofstream cSpaceFile ("cSpace_Ex_3_b.txt");

  while(theta_1 <= 360){

    while(theta_2 <= 360){

      // make function to get position of points
      arm1.x = arm_1_length*cos(theta_1*PI/180.0);
      arm1.y = arm_1_length*sin(theta_1*PI/180.0);

      // check collision of arm 1
      if(map_2.lineCollision(origin, arm1)){
        collisionStatus = 0; // collision occured
      }

      arm2.x = arm1.x + arm_2_length*cos((theta_1+theta_2)*PI/180.0);
      arm2.y = arm1.y + arm_2_length*sin((theta_1+theta_2)*PI/180.0);

      // check collision of arm 2
      if(map_2.lineCollision(arm1, arm2)){
        collisionStatus = 0; // collision occured
      }

      cSpaceFile << theta_1 << "," << theta_2 << "," << collisionStatus << endl;

      collisionStatus  = 1 ;
      theta_2 = theta_2 + delta_theta;

    }

    theta_2 = 0;
    theta_1 = theta_1 + delta_theta;
  }

  cSpaceFile.close();



	// // save C-Space obstacles to txt file
	// ofstream cSpaceFile ("cSpace_Ex_3_a.txt");
	// int cnt = 0;
	// for(const auto& poly_it: map_1.c_space_obs){
  //
	// 	for(const auto& vert_it: poly_it.vertices){
	// 		cSpaceFile << vert_it.x << "," << vert_it.y << ", " << angle[cnt] << endl;
	// 	}
  //
	// 	cnt++;
	// }
  //
	// cSpaceFile.close();
}
