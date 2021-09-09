// Discrete Map.cpp : Defines the entry point for the application.
//

#include <iostream>
#include <algorithm>
#include <list>
#include <vector>
#include <tuple>
#include <cmath>
#include <iostream>
#include <fstream>


#include "Map.h"

using namespace std;

int main()
{
	//
	// cout << "Hello World" << endl;
	//
	////////////////////////////// Workspace 2 //////////////////////////////////
	// Workspace W1
	std::tuple<double, double> xLims = std::make_tuple(-1.0,14.0);
	std::tuple<double, double> yLims = std::make_tuple(-1.0,14.0);

	// // start and goal tuples
	std::tuple<double, double> start = std::make_tuple(0.0,0.0); // (x,y)
	std::tuple<double, double> goal = std::make_tuple(10.0, 10.0); // (x,y)

	// Obstacles
	const int numObs = 5;
	// 8 integers to define the x,y positions of 4 corners of a rectangle
	tuple<double, double, double, double, double, double, double, double> obstacles[numObs] = {
		//
		std::make_tuple(1.0,1.0,2.0,1.0,2.0,5.0,1.0,5.0),
		std::make_tuple(3.0,3.0,4.0,3.0,4.0,12.0,3.0,12.0),
		std::make_tuple(3.0, 12.0, 12.0, 12.0, 12.0, 13.0, 3.0, 13.0),
		std::make_tuple(12.0, 5.0, 13.0, 5.0, 13.0, 13.0,12.0,13.0),
		std::make_tuple(6.0,5.0,12.0,5.0,12.0,6.0,6.0,6.0),

	};



	Map mapa(xLims, yLims,start, goal, obstacles, numObs);

	mapa.Bug1();
	mapa.Bug2();
	mapa.saveRoute();



	// ////////////////////////////// Workspace 2 //////////////////////////////////
	// // Workspace W2
	// std::tuple<double, double> xLims = std::make_tuple(-7.0,36.0);
	// std::tuple<double, double> yLims = std::make_tuple(-7.0,7.0);
	//
	// // mapa.displayMap();
	// // // start and goal tuples
	// std::tuple<double, double> start = std::make_tuple(0.0,0.0); // (x,y)
	// std::tuple<double, double> goal = std::make_tuple(35.0, 0.0); // (x,y)
	//
	// // Obstacles
	// const int numObs = 9;
	// // 8 integers to define the x,y positions of 4 corners of a rectangle
	// tuple<double, double, double, double, double, double, double, double> obstacles[numObs] = {
	// 	//
	// 	std::make_tuple(-6.0,-6.0,  25.0,-6.0,  25.0,-5.0, -6.0,-5.0),
	// 	std::make_tuple(-6.0,5.0, 30.0,5.0, 30.0,6.0, -6.0,6.0),
	// 	std::make_tuple(-6.0,-5.0,  -5.0,-5.0,  -5.0, 5.0,  -6.0, 5.0),
	// 	std::make_tuple(4.0,-5.0,  5.0,-5.0,  5.0,1.0,  4.0,1.0),
	// 	std::make_tuple(9.0,0.0,  10.0,0.0,  10.0,5.0,  9.0,5.0),
	//
	// 	std::make_tuple(14.0,-5.0, 15.0,-5.0, 15.0,1.0, 14.0,1.0),
	// 	std::make_tuple(19.0,0.0,  20.0,0.0,  20.0,5.0,  19.0, 5.0),
	// 	std::make_tuple(24.0,-5.0,  25.0,-5.0,  25.0,1.0,  24.0,1.0),
	// 	std::make_tuple(29.0,0.0,  30.0,0.0,  30.0,5.0,  29.0,5.0)
	//
	// };
	//
	//
	//
	// Map mapa(xLims, yLims,start, goal, obstacles, numObs);
	//
	// mapa.Bug1();
	// mapa.Bug2();
	// mapa.saveRoute();


	return 0;
}
