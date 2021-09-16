// Discrete Map.h : Include file for standard system include files,
// or project specific include files.

#pragma once
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
#include "Eigen/Dense"


using namespace Eigen;
using namespace std;

struct vertex{
	float x,y;
};

// Polygon struct
struct polygon{
	std::vector<vertex> vertices;
};


// Map class
class map {
	float xMapMin, xMapMax, yMapMin, yMapMax, sx, sy,gx,gy;
	std::vector<polygon> obstacles;
	public:
	map( std::tuple<float,float>, std::tuple<float,float>, std::tuple<float,float>, std::tuple<float,float>, std::vector<polygon>);
	bool pointCollision(vertex); // find out if a point collides with any of the obstacles
	bool polygonCollision(); // find out if a polygon collides with another polygon

};

////////////////////////////////////////////////////////////////////////////////
// Map Constructor
////////////////////////////////////////////////////////////////////////////////
map::map( std::tuple<float,float> xLim,  std::tuple<float,float> yLim, std::tuple<float,float> start, std::tuple<float,float> goal, std::vector<polygon> obs){
	// limits of the map
	std::tie(xMapMin, xMapMax) = xLim;
	std::tie(yMapMin, yMapMax) = yLim;
	// goal and start locations
	std::tie(sx,sy) = start;
	std::tie(gx,gy) = goal;
	// obstacles on map
	obstacles = obs;

	// Debugging
	// cout << "xLim = [" << xMapMin << "," << xMapMax << "]" << endl;
	// cout << "yLim = [" << yMapMin << "," << yMapMax << "]" << endl;
	// cout << "start = (" << sx << "," << sy << ")" << endl;
	// cout << "goal = (" << gx << "," << gy << ")" << endl;
}

////////////////////////////////////////////////////////////////////////////////
// pointCollision
////////////////////////////////////////////////////////////////////////////////

bool map::pointCollision(vertex point){

	// iterate through points of all polygons on map and  test of the point is
	// inside or touching the polygon
		cout << "POINT: " << point.x << "," << point.y  << endl;
	// iterate through obstacles
	vertex current, first, next, end_v;
	int polyCnt = 1;
	int edgeCnt = 0;
	int zCnt = 0;


	for (vector<polygon>::iterator poly_it = obstacles.begin(); poly_it != obstacles.end(); ++poly_it){
		// define vectors
		Vector3d v_vertex;
		Vector3d v_point;
		Vector3d v_result;

		// cout << "Polygon " << polyCnt << endl;

		// vector of vertices for polygon in question
		std::vector<vertex> polygon_vertices = poly_it->vertices;

		first = polygon_vertices.front();
		end_v = polygon_vertices.back();
		// auto end_v = *polygon_vertices.end();
		// cout << "first: " << first.x <<","<<first.y <<endl;
		// cout << "end_v: " << end_v.x <<","<<end_v.y <<endl;


		for (vector<vertex>::iterator vertex_it = polygon_vertices.begin(); vertex_it != polygon_vertices.end(); vertex_it++){
				// cout << "pos = [" << vert_it.x << "," << vert_it.y << "]" << endl;
				// set current vertext as iterator
				current = *vertex_it;

				if(current.x == end_v.x && current.y == end_v.y){
					// cout << "Entered IF" << endl;
					// current = *vertex_it;
					// vector from current vertex to next
					v_vertex << (first.x - current.x), (first.y - current.y), 1;
					v_point << (point.x - current.x), (point.y - current.y), 1;

					v_result = v_vertex.cross(v_point);
					//
					// cout << "Result Vector: \n" << v_result << endl;
					float zvalue = v_result(2);

					// cout << "zvalue: \n" << zvalue << endl;

					// cout << "Zval: " << zvalue << endl;
					// cout << "v_vertex: " << (first.x - current.x) << "," << (first.y - current.y) << "," << 1 << endl;
					// cout << "v_point: " << (point.x - current.x) << "," << (point.y - current.y) << "," << 1 << endl;

					edgeCnt = edgeCnt +1;
					if (zvalue >0){
						zCnt = zCnt +1;
					}
				}
				else{
					// next vertex in the vector
					next = *std::next(vertex_it,1);

					// vector from current vertex to next
					v_vertex << (next.x - current.x), (next.y - current.y), 1;
					// vector from current vertext to point of interest
					v_point << (point.x - current.x), (point.y - current.y), 1;

					// debugging print statements
					// cout << "v_vertex: " << (next.x - current.x) << "," << (next.y - current.y) << "," << 1 << endl;
					// cout << "v_point: " << (point.x - current.x) << "," << (point.y - current.y) << "," << 1 << endl;
					//
					v_result = v_vertex.cross(v_point);
					//
					// cout << "Result Vector: \n" << v_result << endl;
					float zvalue = v_result(2);

					edgeCnt = edgeCnt +1;
					if (zvalue >0){
						zCnt = zCnt +1;
					}
			}

		}

		if(zCnt == edgeCnt){
			cout << "collision with polygon " << polyCnt << " has occured" << endl;
			return true;
		}

		polyCnt++;
		zCnt = 0;
		edgeCnt = 0;
	}

	cout << "No collision has occured" << endl;
	return false;
}


////////////////////////////////////////////////////////////////////////////////
// polygonCollision
////////////////////////////////////////////////////////////////////////////////

bool map::polygonCollision(){

	return false;
}
