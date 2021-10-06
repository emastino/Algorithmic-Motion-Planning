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


	// //////////////////////////////////////////////////////////////////////////////
	// // Exercise 1 Workspace 1
	// //
	// //////////////////////////////////////////////////////////////////////////////
	// // map limits
	// std::tuple<double, double> xLim = std::make_tuple(-1,14);
	// std::tuple<double, double> yLim = std::make_tuple(-1,14);
	// // map start and goal
	// std::tuple<double, double> start = std::make_tuple(0.0,0.0);
	// std::tuple<double, double> goal = std::make_tuple(10.0,10.0);
	//
	//
	// std::vector<polygon> obstacles = makeObstacles("obstacles_b.txt");
	//
	// // make a map object
	// map mapa(xLim, yLim, start, goal, obstacles);
	//
	// // initGrid
	// string filename = "wavefrontPath_b.txt";
	// double epsilon = 0.25;
	// mapa.initGrid(epsilon);
	// mapa.wavefront();
	// mapa.wavefrontPath(filename);

	// //////////////////////////////////////////////////////////////////////////////
	// // Exercise 1 Workspace 2
	// //
	// //////////////////////////////////////////////////////////////////////////////
	// // map limits
	// std::tuple<double, double> xLim = std::make_tuple(-7,36);
	// std::tuple<double, double> yLim = std::make_tuple(-7,7);
	// // map start and goal
	// std::tuple<double, double> start = std::make_tuple(0.0,0.0);
	// std::tuple<double, double> goal = std::make_tuple(35.0,0.0);
	//
	//
	// std::vector<polygon> obstacles = makeObstacles("obstacles_c.txt");
	//
	// // make a map object
	// map mapa(xLim, yLim, start, goal, obstacles);
	//
	// // initGrid
	// string filename = "wavefrontPath_c.txt";
	// double epsilon = 0.25;
	// mapa.initGrid(epsilon);
	// mapa.wavefront();
	// mapa.wavefrontPath(filename);

	// print the obstacles
	// mapa.printPolygons();


	// //////////////////////////////////////////////////////////////////////////////
	// // Exercise 2 Manipulator
	// //
	// //////////////////////////////////////////////////////////////////////////////
	// map limits
	std::tuple<double, double> xLim = std::make_tuple(-20,20);
	std::tuple<double, double> yLim = std::make_tuple(-30,30);
	// map start and goal
	std::tuple<double, double> start_t = std::make_tuple(0.0,0.0);
	std::tuple<double, double> goal_t = std::make_tuple(10.0,10.0);

	// read points in from obstacles.txt and make a vector of obstacles
	// This will be a vector of polygons, which in turn a vector of positions

	std::vector<polygon> obstacles_1 = makeObstacles("obstacle_Ex_2_c.txt");

	// make a map object
	map map_1(xLim, yLim, start_t, goal_t, obstacles_1);

	// manipulator parameters
  double epsilon = 0.5; // number of points from 0-360 degrees
  double arm_1_length = 1;
  double arm_2_length = 1;
	// theta initials
  double theta_1 = 0;
  double theta_2 = 0;

	// define points for manipulators
  vertex origin, arm1,arm2;
  origin.x = 0; origin.y =0;

  int collisionStatus = 1; // collision 1 means no collision occured
  // map_1.printPolygons();

	// v[4][2]

	// define all of the stuff for the grid
	std::vector<std::vector<cell>> c_grid;

	double xNumCells =  (360.0-0.0)/epsilon;	// number of cells along x
	double yNumCells =  (360.0-0.0)/epsilon;	// number of cells along y
	cout << "xNumCells: " << xNumCells << endl;
	cout << "yNumCells: " << yNumCells << endl;
	double xMapMin = 0;
	double yMapMin = 0;

	c_grid.resize(xNumCells, vector<cell>(yNumCells));
	vertex v1,v2,v3,v4,c,goal,start;
	goal.x = 360;
	goal.y = 0;
	start.x = 180;
	start.y = 0;
	bool goalFound = false;
	bool startFound = false;

	int gGX, gGY, sGX, sGY;

  ofstream cSpaceFile ("cSpace_Ex_3_c.txt");

  for(int i = 0; i < xNumCells ; i++){

    for(int j = 0; j < yNumCells ; j++){
			theta_1 = i*epsilon ;
			theta_2 = j*epsilon ;
			// cout << "Theta 1: " << theta_1 << endl;
			// cout << "Theta 2: " << theta_2 << endl;
      // make function to get position of points
      arm1.x = arm_1_length*cos(theta_1*PI/180.0);
      arm1.y = arm_1_length*sin(theta_1*PI/180.0);

      // check collision of arm 1
      if(map_1.lineCollision(origin, arm1)){
        collisionStatus = 0; // collision occured
      }

      arm2.x = arm1.x + arm_2_length*cos((theta_1+theta_2)*PI/180.0);
      arm2.y = arm1.y + arm_2_length*sin((theta_1+theta_2)*PI/180.0);

      // check collision of arm 2
      if(map_1.lineCollision(arm1, arm2)){
        collisionStatus = 0; // collision occured
      }


			cell temp;	// temp cell
			// define vertices for cell polygon
			v1.x = xMapMin + epsilon*i;
			v1.y = yMapMin + epsilon*j;
			temp.cellPoly.vertices.push_back(v1);

			v2.x = xMapMin + epsilon*(i+1);
			v2.y = yMapMin + epsilon*j;
			temp.cellPoly.vertices.push_back(v2);

			v3.x = xMapMin + epsilon*(i+1);
			v3.y = yMapMin + epsilon*(j+1);
			temp.cellPoly.vertices.push_back(v3);

			v4.x = xMapMin + epsilon*i;
			v4.y = yMapMin + epsilon*(j+1);
			temp.cellPoly.vertices.push_back(v4);

			// cell center
			c.x = v1.x + epsilon/2;
			c.y = v1.y + epsilon/2;
			temp.center = c;


			// check if we are at goal
			if (map_1.pointPolyCollision(goal, temp.cellPoly) && !goalFound){
				temp.value = 2;
				gGX = i;
				gGY = j;
				goalFound = true;

				// debug message
				cout << "---- Found Goal ----" << endl;
				cout << "Grid Location: (" << gGX << ", " << gGY << ")" << endl;
				cout << "Center Location: (" << c.x << ", " << c.y << ")" << endl;
				cout << "Polygon Vertices:" << endl;
				map_1.printPolygon(temp.cellPoly);
			}


			// check if we are at start
			if (map_1.pointPolyCollision(start, temp.cellPoly) && !startFound){
				sGX = i;
				sGY = j;
				startFound = true;

				// debug message
				cout << "---- Found Start ----" << endl;
				cout << "Grid Location: (" << sGX << ", " << sGY << ")" << endl;
				cout << "Center Location: (" << c.x << ", " << c.y << ")" << endl;
				cout << "Polygon Vertices:" << endl;
				map_1.printPolygon(temp.cellPoly);
			}



			if(collisionStatus == 0){
				temp.value = 1;
			}
			// store the index of the cell
			temp.x = i;
			temp.y = j;

			// add the cell to the grid
			c_grid[i][j] = temp;

			// cout << "---- GRID ----" << endl;
			// cout << "Grid Location: (" << i << ", " << j << ")" << endl;
			// cout << "Center Location: (" << c_grid[i][j].center.x << ", " << c_grid[i][j].center.y << ")" << endl;



      cSpaceFile << theta_1 << "," << theta_2 << "," << collisionStatus << endl;

      collisionStatus  = 1;

    }

  }

  cSpaceFile.close();


	// pass c_grid to make a wavefront and a path
	map_1.wavefrontCSpace(&c_grid, gGX, gGY, xNumCells,yNumCells);
	string filename = "wavefrontPath_cSpace_Ex2_c.txt";
	map_1.wavefrontPathCSpace(filename,c_grid,gGX, gGY,sGX, sGY,xNumCells, yNumCells);





	return 0;
}
