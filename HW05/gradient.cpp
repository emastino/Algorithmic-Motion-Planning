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

	// // Exercise 2.a ////////////////////////////////////////////////////////////
	//
	//
	// // map limits
	// std::tuple<double, double> xLim = std::make_tuple(-1,14);
	// std::tuple<double, double> yLim = std::make_tuple(-1,14);
	// // map start and goal
	// std::tuple<double, double> start = std::make_tuple(0.0,0.0);
	// std::tuple<double, double> goal = std::make_tuple(10.0,0.0);
	//
	//
	//
	// std::vector<polygon> obstacles_a = makeObstacles("obstacles_a.txt");
	//
	// // make a map object
	// map mapa_a(xLim, yLim, start, goal, obstacles_a);
	//
	// // get gradient
	// double Qstar[obstacles_a.size()] = {11, 2.5};
	//
	// mapa_a.gradientDecentPath(Qstar);
	//
	// vertex point, arrow;
	//
	// ofstream vectorField ("vectorField.txt");
	//
	// for(double x = -1; x <= 14; x=x+0.5){
	//
	// 	for (double y = -4; y <= 4; y=y+0.5){
	// 		point.x = x;
	// 		point.y =y;
	// 		if(!mapa_a.pointCollision(point)){
	// 			arrow = mapa_a.gradient(point,Qstar);
	// 			vectorField << point.x << ", " << point.y << ", " << -arrow.x << ", " << -arrow.y << endl;
	// 		}
	// 		// else{
	// 		// 	vectorField << point.x << ", " << point.y << ", " << 0 << ", " << 0 << endl;
	// 		// }
	// 	}
	// }
	//
	//
	// vectorField.close();
	//
	// // print the obstacles
	// mapa_a.printPolygons();



	// // Exercise 2.b_1 ////////////////////////////////////////////////////////////
	//
	//
	// // map limits
	// std::tuple<double, double> xLim = std::make_tuple(-1,14);
	// std::tuple<double, double> yLim = std::make_tuple(-1,14);
	// // map start and goal
	// std::tuple<double, double> start = std::make_tuple(0.0,0.0);
	// std::tuple<double, double> goal = std::make_tuple(10.0,10.0);
	//
	//
	// std::vector<polygon> obstacles_b = makeObstacles("obstacles_b.txt");
	//
	// // make a map object
	// map mapa_b(xLim, yLim, start, goal, obstacles_b);
	//
	// // get gradient
	// double Qstar[obstacles_b.size()] = {1, 0.3 , 13.0 , 7.0 , 2.0 };
	//
	// mapa_b.gradientDecentPath(Qstar);
	//
	// // print the obstacles
	// mapa_b.printPolygons();
	//
	//
	// mapa_b.printObs();




	// Exercise 2.b_2 ////////////////////////////////////////////////////////////


	// map limits
	std::tuple<double, double> xLim = std::make_tuple(-7,36);
	std::tuple<double, double> yLim = std::make_tuple(-7,7);
	// map start and goal
	std::tuple<double, double> start = std::make_tuple(0.0,0.0);
	std::tuple<double, double> goal = std::make_tuple(35.0,0.0);


	std::vector<polygon> obstacles_c = makeObstacles("obstacles_c.txt");

	// make a map object
	map mapa_c(xLim, yLim, start, goal, obstacles_c);

	// get gradient
	double Qstar[obstacles_c.size()] = {8.5, 3 , 25 , 0.25 , 0.25, 0.25 , 0.25 , 0.25 , 0.25 };

	mapa_c.gradientDecentPath(Qstar);

	// print the obstacles
	mapa_c.printPolygons();



	return 0;
}
