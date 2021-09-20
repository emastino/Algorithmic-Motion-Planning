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


	// read points in from obstacles.txt and make a vector of obstacles
	// This will be a vector of polygons, which in turn a vector of positions
	std::vector<polygon> obstacles;

	string line; // whole line of text file
	int count = 1;
	vertex ver;
	// read in obstacle file
	ifstream myfile("obstacles_Ex_1.txt");
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
	myfile.close();


// // Debugging
// // Display what is stored in obstacles
// int polyCnt = 1;
// for (const auto& it: obstacles){
// 	cout << "Polygon " << polyCnt << endl;
// 	for (const auto& it1: it.vertices){
// 		cout << "pos = [" << it1.x << "," << it1.y << "]" << endl;
// 	}
// 	polyCnt++;
// }

// // Debugging
// test out polygon struct functionality
// polygon triangle;
// for(int i = 0; i <3; i++){
// 	pos p;
// 	p.x = 2.0+i;
// 	p.y = i*5.0;
// 	triangle.points.push_back(p);
// }
// cout << "Triangle Points" << endl;
// for(const auto& it : triangle.points){
// 	cout << it.x << ", " << it.y << endl;
// }


	// make a map object
	map mapa(xLim, yLim, start, goal, obstacles);

	// // test point intersection
	// vertex v;
	// v.x = 6;
	// v.y = 8;
	// mapa.pointCollision(v);

	// Robot polygon
	polygon robot;
	vertex bot1;
	bot1.x = 0;
	bot1.y = 0;
	vertex bot2;
	bot2.x = 1;
	bot2.y = 2;
	vertex bot3;
	bot3.x = 0;
	bot3.y = 2;
	// there is a better way to do this but I am too lazy atm
	robot.vertices.push_back(bot1);
	robot.vertices.push_back(bot2);
	robot.vertices.push_back(bot3);

	// obstacle
	polygon test_obstacle = mapa.obstacles.back();


	// // see if collision occurs
	// mapa.polygonCollision(robot, test_obstacle);

	// // test inversion
	// mapa.invert_polygon(robot);

	// // test rotation
	// vertex rotVert;
	// rotVert.x = 0.0;
	// rotVert.y = 0.0;
	//
	// mapa.rotate_polygon(robot, rotVert, 90);
	double angle[12];
	angle[0] = -2.5;
	// test makeCSpaceObstacles
	for(int i = 0; i<12; i++){

		mapa.makeCSpaceObstacle(test_obstacle, robot, angle[i]);
		angle[i+1] = angle[i]+0.5;
	}

	// save C-Space obstacles to txt file

	ofstream cSpaceFile ("cSpaceFile.txt");
	int cnt = 0;

	for(const auto& poly_it: mapa.c_space_obs){

		for(const auto& vert_it: poly_it.vertices){
			cSpaceFile << vert_it.x << "," << vert_it.y << ", " << angle[cnt] << endl;
		}

		cnt++;
	}

	cSpaceFile.close();



	// mapa.printObs();

	return 0;
}
