// Discrete Map.h : Include file for standard system include files,
// or project specific include files.

#pragma once
#include <iostream>
#include <algorithm>
#include <list>
#include <vector>
#include <tuple>
#include <cmath>
#include <iostream>
#include <fstream>
#include <sstream>


using namespace std;

struct vertex{
	float x,y;
};

// Polygon struct
struct polygon{
	std::vector<vertex> vertices;
};



class Map {
	float xMapMin, xMapMax, yMapMin, yMapMax, sx, sy,gx,gy;
	std::vector<polygon> obstacles;
	public:
	Map( std::tuple<float,float>, std::tuple<float,float>, std::tuple<float,float>, std::tuple<float,float>);
	bool pointCollision(vertex); // find out if a point collides with any of the obstacles
	bool polygonCollision(); // find out if a polygon collides with another polygon

};

////////////////////////////////////////////////////////////////////////////////
// Map Constructor
////////////////////////////////////////////////////////////////////////////////
Map::Map( std::tuple<float,float> xLim,  std::tuple<float,float> yLim, std::tuple<float,float> start, std::tuple<float,float> goal){
	std::tie(xMapMin, xMapMax) = xLim;
	std::tie(yMapMin, yMapMax) = yLim;

	std::tie(sx,sy) = start;
	std::tie(gx,gy) = goal;

	// Debugging
	// cout << "xLim = [" << xMapMin << "," << xMapMax << "]" << endl;
	// cout << "yLim = [" << yMapMin << "," << yMapMax << "]" << endl;
	// cout << "start = (" << sx << "," << sy << ")" << endl;
	// cout << "goal = (" << gx << "," << gy << ")" << endl;
}

////////////////////////////////////////////////////////////////////////////////
// pointCollision
////////////////////////////////////////////////////////////////////////////////

bool Map::pointCollision(vertex point){

	return false;
}


////////////////////////////////////////////////////////////////////////////////
// polygonCollision
////////////////////////////////////////////////////////////////////////////////

bool Map::polygonCollision(){

	return false;
}
