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
#include <sstream>


#include "Map.h"

using namespace std;

int main()
{

	// map limits
	std::tuple<float, float> xLim = std::make_tuple(0.0,13.2);
	std::tuple<float, float> yLim = std::make_tuple(0.0,17.8);
	// map start and goal
	std::tuple<float, float> start = std::make_tuple(0.0,0.0);
	std::tuple<float, float> goal = std::make_tuple(10.0,10.0);


	// read points in from obstacles.txt and make a vector of obstacles
	// This will be a vector of polygons, which in turn a vector of positions
	std::vector<polygon> obstacles;

	string line; // whole line of text file
	int count = 1;
	vertex vertex;

	ifstream myfile("obstacles.txt");
	while ( getline (myfile,line) ){

		std::istringstream ss( line);

		// create new polygon object to be added
		polygon poly;

		while (ss){
      string s; // string seperated by comma

      if (!getline( ss, s, ',' )) break;

			if(count ==1){
				// add x value of point
				vertex.x = std::stof(s);
				count = 2;
			}
			else{
				// add y value of point
				vertex.y = std::stof(s);
				// add vertex to the polygon in obstacles
				poly.vertices.push_back(vertex);
				count =1;
			}


    }
		obstacles.push_back(poly);
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
// test out polygon struct funstionality
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



	Map mapa(xLim, yLim, start, goal);




	return 0;
}
