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

// namespaces
using namespace Eigen;
using namespace std;

// constants
#define PI 3.14159265


// vertex structure
struct vertex{
	double x,y;
};

// Polygon struct
struct polygon{
	std::vector<vertex> vertices;
};

// grid cell structure
struct cell{
	polygon cellPoly; // polygon defining cell
	vertex 	center; // center of polygon
	double 	value = 0; // velue used in the wavefront. Default 0, 1 for obstacles

	int x,y;	// position in the vectors of the cell

};


// Map class
class map {
	double xMapMin, xMapMax, yMapMin, yMapMax, sx, sy,gx,gy;

	public:
	std::vector<polygon> obstacles; // obstacles in the workspace
	std::vector<polygon> c_space_obs;	// c-space representation of the obstacles
	std::vector<std::vector<cell>> grid; // grid representation of the map
	int startGridX, startGridY, goalGridX, goalGridY, xNumCells, yNumCells;
	// v[4][2]
	std::vector<vertex> manipulator_points;
	int numberOfObstacles;

	// constructor
	map( std::tuple<double,double>, std::tuple<double,double>, std::tuple<double,double>, std::tuple<double,double>, std::vector<polygon>);

	// collisions
	bool 		pointCollision(vertex); // find out if a point collides with any of the obstacles
	bool 		polygonObstacleCollision(polygon); // check if a polygon collides with any obstacle
	bool 		pointPolyCollision(vertex, polygon); // check if a point and polygon collide
	bool 		polygonCollision(polygon, polygon); // find out if a polygon collides with another polygon
	bool 		lineCollision(vertex ,vertex );

	// polygon manipulation
	polygon rotate_polygon(polygon, vertex, double); // rotate a polygon about a point
	polygon invert_polygon(polygon); // invert the points of a polygon
	polygon orderVertices(polygon p); // order the vertices of a polygon

	// construction of c-space
	polygon makeCSpaceObstacle(polygon, polygon, double);

	// gradient map
	void 		gradientDecentPath(double *);
	vertex 	gradient(vertex, double *);
	vertex 	minDistanceToObs(polygon, vertex);

	// wavefront planner
	void initGrid(double);
	void wavefront();
	void wavefrontPath(string);
	void wavefrontCSpace(std::vector<std::vector<cell>>*, int, int,int,int);
	void wavefrontPathCSpace(string filename,std::vector<std::vector<cell>>, int,int,int,int,int,int);

	// helpers
	double 	dist(vertex, vertex);

	// debugging
	void 		printObs();
	void 		printPolygon(polygon); // print a selected polygon
	void 		printPolygons(); // print all polygons

};

////////////////////////////////////////////////////////////////////////////////
// Map Constructor
////////////////////////////////////////////////////////////////////////////////
map::map( std::tuple<double,double> xLim,  std::tuple<double,double> yLim, std::tuple<double,double> start, std::tuple<double,double> goal, std::vector<polygon> obs){
	// limits of the map
	std::tie(xMapMin, xMapMax) = xLim;
	std::tie(yMapMin, yMapMax) = yLim;
	// goal and start locations
	std::tie(sx,sy) = start;
	std::tie(gx,gy) = goal;
	// obstacles on map
	obstacles = obs;

	numberOfObstacles = obstacles.size();
	// Debugging
	// cout << "xLim = [" << xMapMin << "," << xMapMax << "]" << endl;
	// cout << "yLim = [" << yMapMin << "," << yMapMax << "]" << endl;
	// cout << "start = (" << sx << "," << sy << ")" << endl;
	// cout << "goal = (" << gx << "," << gy << ")" << endl;
}

////////////////////////////////////////////////////////////////////////////////
// pointCollision
////////////////////////////////////////////////////////////////////////////////
// Test if a point collides with any obsyacle in the map
// False if no collision occurs
// True if collsion occurs
bool map::pointCollision(vertex point){

	// iterate through points of all polygons on map and  test of the point is
	// inside or touching the polygon
		// cout << "POINT: " << point.x << "," << point.y  << endl;
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
					double zvalue = v_result(2);

					// cout << "zvalue: \n" << zvalue << endl;

					// cout << "Zval: " << zvalue << endl;
					// cout << "v_vertex: " << (first.x - current.x) << "," << (first.y - current.y) << "," << 1 << endl;
					// cout << "v_point: " << (point.x - current.x) << "," << (point.y - current.y) << "," << 1 << endl;

					// collition with a vertex
					if((point.x - current.x)== 0 && (point.y - current.y)==0){
						// cout << "Collision with polygon " << polyCnt << " has occured" << endl;
						return true;
					}

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
					double zvalue = v_result(2);

					// collision with a vertex
					if((point.x - current.x)== 0 && (point.y - current.y)==0){
						// cout << "Collision with polygon " << polyCnt << " has occured" << endl;
						return true;
					}

					edgeCnt = edgeCnt +1;
					if (zvalue >0){
						zCnt = zCnt +1;
					}

			}

		}

		if(zCnt == edgeCnt){
			// cout << "Collision with polygon " << polyCnt << " has occured" << endl;
			return true;
		}

		polyCnt++;
		zCnt = 0;
		edgeCnt = 0;
	}

	// cout << "No point collision has occured" << endl;
	return false;
}


////////////////////////////////////////////////////////////////////////////////
// pointPolyCollision
////////////////////////////////////////////////////////////////////////////////
// Pass in a point and a polygon and see if they collide
bool map::pointPolyCollision(vertex point, polygon poly){


	// iterate through points of all polygons on map and  test of the point is
	// inside or touching the polygon
		// cout << "POINT: " << point.x << "," << point.y  << endl;
	// iterate through obstacles
	vertex current, first, next, end_v;
	int edgeCnt = 0;
	int zCnt = 0;

	// define vectors
	Vector3d v_vertex;
	Vector3d v_point;
	Vector3d v_result;

	// vector of vertices for polygon in question
	std::vector<vertex> polygon_vertices = poly.vertices;

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
				double zvalue = v_result(2);

				// cout << "zvalue: \n" << zvalue << endl;

				// cout << "Zval: " << zvalue << endl;
				// cout << "v_vertex: " << (first.x - current.x) << "," << (first.y - current.y) << "," << 1 << endl;
				// cout << "v_point: " << (point.x - current.x) << "," << (point.y - current.y) << "," << 1 << endl;

				// collition with a vertex
				if((point.x - current.x)== 0 && (point.y - current.y)==0){
					// cout << "Collision with polygon " << polyCnt << " has occured" << endl;
					return true;
				}

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
				double zvalue = v_result(2);

				// collision with a vertex
				if((point.x - current.x)== 0 && (point.y - current.y)==0){
					// cout << "Collision with polygon " << polyCnt << " has occured" << endl;
					return true;
				}

				edgeCnt = edgeCnt +1;
				if (zvalue >0){
					zCnt = zCnt +1;
				}

			}

		}

		if(zCnt == edgeCnt){
			// cout << "Collision with polygon " << polyCnt << " has occured" << endl;
			return true;
		}


	// cout << "No point collision has occured" << endl;
	return false;

}


////////////////////////////////////////////////////////////////////////////////
// polygonCollision
////////////////////////////////////////////////////////////////////////////////
// Test if two polygons collide
bool map::polygonCollision(polygon robot, polygon obstacle){
	Vector3d edge_vec, robot_vec, normal, inThePlane;
	vertex current, next;
	inThePlane << 0, 0 , 1;

	// cout << "------------------ Check Robot Edges ------------------" << endl;
	// firt iterate through the sides of the robot
	vertex robot_first_vertex = (robot.vertices).front();
	vertex robot_last_vertex = (robot.vertices).back();

	for(const auto& vert_it: robot.vertices){

		current = vert_it;

		if(vert_it.x == robot_last_vertex.x && vert_it.x == robot_last_vertex.x){
			// next vertex in the vector

			next = robot_first_vertex;
			// vector from current vertex to next
			robot_vec << (next.x - current.x), (next.y - current.y), 1;
			// cout << robot_vec << endl;

			// 	// claculate normal vector to the robot edge
			normal = robot_vec.cross(inThePlane).normalized();
			// cout << normal << endl;

			// // vector from current vertext to point of interest
			// v_point << (point.x - current.x), (point.y - current.y), 1;

		}
		else{
			next = *std::next(&vert_it,1);

			// vector from current vertex to next
			robot_vec << (next.x - current.x), (next.y - current.y), 1;
			// cout << robot_vec << endl;

			// claculate normal vector to the robot edge
			normal = robot_vec.cross(inThePlane).normalized();
			// cout << normal << endl;
		}


		// now using that normal find the max and min points of the robot's and the
		// obstacles' projection on the normal line

		double robotProjectedMax = 0;
		double robotProjectedMin = 0;
		double projection;
		int count = 1;

		for(const auto& vert_it_2: robot.vertices){
			Vector3d temp;
			temp << vert_it_2.x , vert_it_2.y , 1;

			projection = normal.dot(temp);

			if(count == 1){
				robotProjectedMax = projection;
				robotProjectedMin = projection;
			}

			else{
				if(projection > robotProjectedMax){
					robotProjectedMax = projection;
				}
				if(projection < robotProjectedMin){
					robotProjectedMin = projection;
				}
			}

			count++;
		}
		// cout << "Robot Min " << robotProjectedMin << endl;
		// cout << "Robot Max " << robotProjectedMax << endl;
	// 	// now using that normal find the max and min points of the obstacles'
	//
		double obsProjectedMax = 0;
		double obsProjectedMin = 0;
		count = 1;
		for(const auto& obs_it: obstacle.vertices){
			Vector3d temp;
			temp << obs_it.x , obs_it.y , 1;

			projection = normal.dot(temp);

			if(count == 1){
				obsProjectedMax = projection;
				obsProjectedMin = projection;
			}

			else{
				if(projection > obsProjectedMax){
					obsProjectedMax = projection;
				}
				if(projection < obsProjectedMin){
					obsProjectedMin = projection;
				}
			}

			count++;
		}
		//
		// cout << "Obstacle Min " << obsProjectedMin << endl;
		// cout << "Obstacle Max " << obsProjectedMax << endl;

		if(robotProjectedMin >obsProjectedMax || robotProjectedMax < obsProjectedMin){
			// cout << "No collsion with a robot" << endl;
			return false;
		}

	}

	// cout << "------------------ Check Obstacle Edges ------------------" << endl;
	// first iterate through the sides of the obstacle
	vertex obs_first_vertex = (obstacle.vertices).front();
	vertex obs_last_vertex = (obstacle.vertices).back();

	for(const auto& vert_it: obstacle.vertices){

		current = vert_it;

		if(vert_it.x == obs_last_vertex.x && vert_it.x == obs_last_vertex.x){
			// next vertex in the vector

			next = obs_first_vertex;
			// vector from current vertex to next
			edge_vec << (next.x - current.x), (next.y - current.y), 1;
			// cout << robot_vec << endl;

			// 	// claculate normal vector to the robot edge
			normal = edge_vec.cross(inThePlane).normalized();
			// cout << normal << endl;

			// // vector from current vertext to point of interest
			// v_point << (point.x - current.x), (point.y - current.y), 1;

		}
		else{
			next = *std::next(&vert_it,1);

			// vector from current vertex to next
			edge_vec << (next.x - current.x), (next.y - current.y), 1;
			// cout << robot_vec << endl;

			// claculate normal vector to the robot edge
			normal = edge_vec.cross(inThePlane).normalized();
			// cout << normal << endl;
		}


		// now using that normal find the max and min points of the robot's and the
		// obstacles' projection on the normal line

		double robotProjectedMax = 0;
		double robotProjectedMin = 0;
		double projection;
		int count = 1;

		for(const auto& vert_it_2: robot.vertices){
			Vector3d temp;
			temp << vert_it_2.x , vert_it_2.y , 1;

			projection = normal.dot(temp);

			if(count == 1){
				robotProjectedMax = projection;
				robotProjectedMin = projection;
			}

			else{
				if(projection > robotProjectedMax){
					robotProjectedMax = projection;
				}
				if(projection < robotProjectedMin){
					robotProjectedMin = projection;
				}
			}

			count++;
		}
		// cout << "Robot Min " << robotProjectedMin << endl;
		// cout << "Robot Max " << robotProjectedMax << endl;
	// 	// now using that normal find the max and min points of the obstacles'
	//
		double obsProjectedMax = 0;
		double obsProjectedMin = 0;
		count = 1;
		for(const auto& obs_it: obstacle.vertices){
			Vector3d temp;
			temp << obs_it.x , obs_it.y , 1;

			projection = normal.dot(temp);

			if(count == 1){
				obsProjectedMax = projection;
				obsProjectedMin = projection;
			}

			else{
				if(projection > obsProjectedMax){
					obsProjectedMax = projection;
				}
				if(projection < obsProjectedMin){
					obsProjectedMin = projection;
				}
			}

			count++;
		}

		// cout << "Obstacle Min " << obsProjectedMin << endl;
		// cout << "Obstacle Max " << obsProjectedMax << endl;

		if(robotProjectedMin >obsProjectedMax || robotProjectedMax < obsProjectedMin){
			// cout << "No collsion with a robot" << endl;
			return false;
		}

	}


	// cout << "COLLISION OCCURED" <<endl;
	return true;
}

////////////////////////////////////////////////////////////////////////////////
// polygonObstacleCollision
////////////////////////////////////////////////////////////////////////////////
// check if a polygon collides with any obstacle
bool map::polygonObstacleCollision(polygon poly){

	for(const auto& poly_it:obstacles){
		if(polygonCollision(poly, poly_it)){return true;}
	}

	return false;
}


////////////////////////////////////////////////////////////////////////////////
//  lineCollision
////////////////////////////////////////////////////////////////////////////////
// see if the line connecting two points intersects an obstacle
bool map::lineCollision(vertex v1,vertex v2 ){

	vertex temp;

	double lambda = 0;
	double delta_lambda = 0.01;


	while (lambda <= 1){

		temp.x = v1.x*(1-lambda) + v2.x*lambda;
		temp.y = v1.y*(1-lambda) + v2.y*lambda;

		if(pointCollision(temp)) {return true;}
		else {lambda = lambda + delta_lambda;}

	}


	return false;

}


////////////////////////////////////////////////////////////////////////////////
//  rotate_polygon
////////////////////////////////////////////////////////////////////////////////
// rottate a polygon about a point return by theta degrees
polygon map::rotate_polygon(polygon p, vertex v, double theta){

	polygon rotated_poly;

	vertex first_vertex = p.vertices.front();

	Matrix3d T;
	T << cos(PI*theta/180.0), -sin(PI*theta/180.0), -v.x*cos(PI*theta/180.0) +v.y*sin(PI*theta/180.0) +v.x,
			sin(PI*theta/180.0), cos(PI*theta/180.0), -v.x*sin(PI*theta/180.0) -v.y*cos(PI*theta/180.0) +v.y,
			0,0,1.0f;
	// cout << "T: " << T << endl;


	for(const auto& vert_it: p.vertices){
		vertex vp;
		Vector3d ogV = Vector3d(vert_it.x, vert_it.y, 1.0);
		Vector3d newV = T*ogV;

		// cout << "OG Vector: " << ogV << endl;
		//
		// cout << "New Vector: " << newV << endl;

		vp.x = newV(0);
		vp.y = newV(1);
		//
		// cout << "Before: " << vert_it.x << ", " << vert_it.y << endl;
		// cout << "After: " << vp.x << ", " << vp.y << endl;
		rotated_poly.vertices.push_back(vp);

	}

	return rotated_poly;

}


////////////////////////////////////////////////////////////////////////////////
//  invert polygon
////////////////////////////////////////////////////////////////////////////////
// rottate a polygon about a point return by theta degrees
polygon map::invert_polygon(polygon p){

	polygon inv_poly;
	vertex first_vertex = p.vertices.front();

	for(const auto& vert_it: p.vertices){
		vertex v;
		v.x = -vert_it.x + first_vertex.x;
		v.y = -vert_it.y + first_vertex.y;
		// cout << "Before: " << vert_it.x << ", " << vert_it.y << endl;
		// cout << "After: " << v.x << ", " << v.y << endl;
		inv_poly.vertices.push_back(v);

	}

	return inv_poly;
}

////////////////////////////////////////////////////////////////////////////////
//  makeCSpaceObstacle
////////////////////////////////////////////////////////////////////////////////
// rottate a polygon about a point return by theta degrees
polygon map::makeCSpaceObstacle(polygon obs, polygon robot, double angle){

	// to return
	polygon cSpaceObs;

	// number of vertices
	int numVertObs = obs.vertices.size();
	int numVertRobot = robot.vertices.size();
	int totalNewPoints =numVertObs + numVertRobot;

	// center of rotation
	vertex CentOfRot;
	CentOfRot.x = 0.0;
	CentOfRot.y = 0.0;

	// cout << "Obstacle" << endl;
	// printPolygon(obs);

	// trnasform robot
	polygon transRob = rotate_polygon(robot,CentOfRot,angle);

	// cout << "Original Robot" << endl;
	// printPolygon(robot);

	// cout << "Rotated Robot" << endl;
	// printPolygon(transRob);

	transRob = invert_polygon(transRob);

	// cout << "Inverted Robot" << endl;
	// printPolygon(transRob);

	transRob = orderVertices(transRob);

	// cout << "Inverted + Ordered Robot" << endl;
	// printPolygon(transRob);


	// make array for ostacles
	vertex obsArray[numVertObs+1];
	double obsAngleArray[numVertObs];
	int obsInd = 0;
	int obsAngInd = 0; //obstacle angle index

	// populate obsArray
	for (const auto& obs_it: obs.vertices){
		obsArray[obsInd] = obs_it;
		obsInd++;
	}
	obsArray[obsInd] = obs.vertices.front();
	obsInd = 0; //reset obstacle index

	// populate angles
		// cout << "Obstacle Angles: ";
	for (int i = 0; i < numVertObs; i++ ){
		obsAngleArray[i] = atan2(obsArray[i+1].y-obsArray[i].y, obsArray[i+1].x-obsArray[i].x);
		if(obsAngleArray[i] < 0){obsAngleArray[i] = 2*PI+obsAngleArray[i];}
		// cout << obsAngleArray[i]*180.0/PI << ", ";
	}
	// cout << endl;


	// make array for transformed robot
	vertex robArray[numVertRobot+1];
	double robAngleArray[numVertObs];
	int robInd = 0;
	int robAngInd = 0;

	// populate robArray
	for (const auto& rob_it: transRob.vertices){
		robArray[robInd] = rob_it;
		robInd++;
	}
	robArray[robInd] = transRob.vertices.front();
	robInd = 0; //reset obstacle index

	// populate angles
	// cout << "Robot Angles: " ;
	for (int i = 0; i < numVertRobot; i++ ){
		robAngleArray[i] = atan2(robArray[i+1].y-robArray[i].y, robArray[i+1].x-robArray[i].x);

		if(robAngleArray[i] < 0){robAngleArray[i] = 2*PI+robAngleArray[i];}
		// cout << robAngleArray[i]*180.0/PI<< ", ";
	}
	// cout << endl;
	// number of vertices of new polygon
	int vertCnt =0;

	vertex newPoints[totalNewPoints];
	vertex sum;
	sum.x=robArray[robInd].x + obsArray[obsInd].x;
	sum.y=robArray[robInd].y + obsArray[obsInd].y;
	cSpaceObs.vertices.push_back(sum);
	vertCnt++;

	while(vertCnt < totalNewPoints){

		if(robAngleArray[robAngInd] <= obsAngleArray[obsAngInd]){
			if(robInd < numVertRobot+1){
				robInd++;
			}
			if(robInd < numVertRobot){
				robAngInd++;
			}

			// cout << "RobInd and RobAng: " << robInd << ", " << robAngInd << endl;

			sum.x=robArray[robInd].x + obsArray[obsInd].x;
			sum.y=robArray[robInd].y + obsArray[obsInd].y;
			cSpaceObs.vertices.push_back(sum);

			vertCnt++;
		}
		else{
			if(obsInd < numVertObs+1){
				obsInd++;
			}
			if(obsInd < numVertObs){
				obsAngInd++;
			}

			// cout << "ObsInd and ObsAng: " << obsInd << ", " << obsAngInd << endl;
			sum.x=robArray[robInd].x + obsArray[obsInd].x;
			sum.y=robArray[robInd].y + obsArray[obsInd].y;
			cSpaceObs.vertices.push_back(sum);
			vertCnt++;
		}


	}

	// printPolygon(cSpaceObs);
	c_space_obs.push_back(cSpaceObs);

	return cSpaceObs;



}

////////////////////////////////////////////////////////////////////////////////
//  orderVertices
////////////////////////////////////////////////////////////////////////////////
// orders the vertices of a convex polygon.
polygon map::orderVertices(polygon p){

	polygon orderedPoly;
	// Before
	// cout << "before " << endl;
	//
	// printPolygon(p);


	// find point 1
	vertex firstVertex;
	firstVertex = p.vertices.front();
	int count =0;
	int minIndex =0;
	for (const auto& vert_it: p.vertices){

		if(vert_it.y < firstVertex.y){
				firstVertex = vert_it;
				minIndex = count;
		}
		count++;
	}

	//
	orderedPoly.vertices.push_back(firstVertex);

	// erase the  min element from the list
	p.vertices.erase(p.vertices.begin()+minIndex);

	// number of remaining points
	int remainingPoints = p.vertices.size();

	list<double> angles; // list of angles

	for(const auto& vert_it: p.vertices){
		double angle = atan2(vert_it.y - firstVertex.y, vert_it.x - firstVertex.x);
		angles.push_back(angle);
	}

	// sort angles
	angles.sort();

	for(const auto& list_it:angles){

		for (const auto& vert_it: p.vertices ){
			double temp = atan2(vert_it.y - firstVertex.y, vert_it.x - firstVertex.x);
			if (temp == list_it){
				orderedPoly.vertices.push_back(vert_it);
			}
		}

	}


	// cout << "after " << endl;
	// printPolygon(orderedPoly);
	// order the points based on angle from point 1
	return orderedPoly;

}


////////////////////////////////////////////////////////////////////////////////
//  gradientDecent()
////////////////////////////////////////////////////////////////////////////////
// make a potential map
void map::gradientDecentPath(double *Qstar){

	// goal bounds
	double epsilon = 0.25;

	// temporary vertex to store path
	vertex temp, newGradient;
	temp.x = sx;
	temp.y = sy;

	// vector storing path points
	std::vector<vertex> gradDecPath;

	// add start to gradDecPath
	gradDecPath.push_back(temp);

	// set reached goal flag
	bool reachedGoal = false;

	// step size in direction of gradient
	double alpha = 0.001;


	ofstream gradientPath ("gradientPath_c.txt");
	int count = 0;
	while(!reachedGoal){

		// cout << "(x,y): " << temp.x << ", " << temp.y << endl;

		gradientPath << temp.x << ", " << temp.y << endl;
		newGradient = gradient(temp,Qstar); // bad syntax, need another var probs

		temp.x = temp.x + alpha*newGradient.x;
		temp.y = temp.y + alpha*newGradient.y;

		// add new point to gradDecPath
		gradDecPath.push_back(temp);

		if((pow(temp.x-gx,2.0) +  pow(temp.y-gy,2.0))<=pow(epsilon,2.0)){
			cout << "You reached goal!" << endl;
			reachedGoal = true;
		}



	}

	gradientPath.close();

}


////////////////////////////////////////////////////////////////////////////////
//  gradient()
////////////////////////////////////////////////////////////////////////////////
// return the gradient direction
vertex map::gradient(vertex current, double *Qstar){

	// dStarGoal
	double dStarGoal = 5;

	// start location
	vertex start;
	start.x = sx;
	start.y = sy;

	// goal location
	vertex goal;
	goal.x = gx;
	goal.y = gy;


	// gradients
	vertex gradient, attractive, repulsive;

	// attractive gradient //////////////////////////////////////////////////////
	// scalar
	double xi = 1;

	if(dist(current,goal) <= dStarGoal){
		attractive.x = xi*(current.x - goal.x);
		attractive.y = xi*(current.y - goal.y);
	}
	else{
		attractive.x = dStarGoal*xi*(current.x - goal.x)/dist(current,goal);
		attractive.y = dStarGoal*xi*(current.y - goal.y)/dist(current,goal);
	}


	// repulsive gradient ///////////////////////////////////////////////////////
	double d[numberOfObstacles];
	double eta = 1;		// repulsive scalar
	int count = 0;		// count to populate Qstar and d arrays

	//initialize repulsive function
	repulsive.x = 0;
	repulsive.y = 0;


	for (const auto& poly_it: obstacles){
		vertex c = minDistanceToObs(poly_it, current);
		vertex del_d;

		d[count] = dist(current,c);

		// gradient of d_i
		if(d[count] <= Qstar[count]){

			del_d.x = (current.x-c.x)/d[count];
			del_d.y =(current.y-c.y)/d[count];
			repulsive.x = repulsive.x + del_d.x*eta*(1/Qstar[count] - 1/d[count])/pow(d[count],2);
			repulsive.y = repulsive.y + del_d.y*eta*(1/Qstar[count] - 1/d[count])/pow(d[count],2);
		}
		else{
			repulsive.x = repulsive.x + 0.0;
			repulsive.y = repulsive.y + 0.0;
		}

		count ++; 	// increment counter

	}

	// complete gradient
	gradient.x = -attractive.x - repulsive.x;
	gradient.y = -attractive.y - repulsive.y;

	return gradient;

}


////////////////////////////////////////////////////////////////////////////////
//  minDistanceToObs
////////////////////////////////////////////////////////////////////////////////
// return minimum distance to obstacles. Will only work with rectangles
// for now

vertex map::minDistanceToObs(polygon poly, vertex current){
	int count = 1;
	double minDistance, xmin,xmax, ymin,ymax;
	vertex c;

	for (const auto& vert_it : poly.vertices){
		// set the first
		if(count == 1) {
			xmin = vert_it.x;
			ymin = vert_it.y;

			xmax= vert_it.x;
			ymax = vert_it.y;

			minDistance = dist(vert_it,current);
			c = vert_it;
		}

		// x min
		if(vert_it.x < xmin){
			xmin = vert_it.x;
		}
		//x max
		if(vert_it.x >xmax){
			xmax = vert_it.x;
		}


		// y min
		if(vert_it.y < ymin){
			ymin = vert_it.y;
		}
		//y max
		if(vert_it.y > ymax){
			ymax = vert_it.y;
		}

		if(minDistance > dist(vert_it,current)){
			minDistance = dist(vert_it,current);
			c = vert_it;
		}


		count++; //increment counter
	}

	// now we have the max and min values of the shape for X
	if(current.x >= xmin && current.x < xmax){
		if(current.y > ymax){
			minDistance = current.y-ymax;
			c.x = current.x;
			c.y = ymax;
		}
		else{
			minDistance = ymin-current.y;
			c.x = current.x;
			c.y = ymin;
		}
	}

	// now we have the max and min values of the shape for Y
	if(current.y >= ymin && current.y < ymax){
		if(current.x > xmax){
			minDistance = current.x-xmax;
			c.x = xmax;
			c.y = current.y;

		}
		else{
			minDistance = xmin-current.x;
			c.x = xmin;
			c.y = current.y;
		}
	}


	return c;
}


////////////////////////////////////////////////////////////////////////////////
//  initGrid
////////////////////////////////////////////////////////////////////////////////
// make a grid out of the map based on obstacles
void map::initGrid(double epsilon){

	// v[4][2]
	xNumCells =  (xMapMax-xMapMin)/epsilon;	// number of cells along x
	yNumCells =  (yMapMax-yMapMin)/epsilon;	// number of cells along y
	cout << "xNumCells: " << xNumCells << endl;
	cout << "yNumCells: " << yNumCells << endl;

	grid.resize(xNumCells, vector<cell>(yNumCells));


	vertex v1,v2,v3,v4,c,goal,start;
	goal.x = gx; goal.y = gy;
	start.x = sx; start.y = sy;

	bool goalFound = false;
	bool startFound = false;

	for(unsigned i = 0; i < xNumCells; i++){

		for(unsigned j = 0; j < yNumCells; j++){
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
			if (pointPolyCollision(goal, temp.cellPoly) && !goalFound){
				temp.value = 2;
				goalGridX = i;
				goalGridY = j;
				goalFound = true;

				// debug message
				cout << "---- Found Goal ----" << endl;
				cout << "Grid Location: (" << goalGridX << ", " << goalGridY << ")" << endl;
				cout << "Center Location: (" << c.x << ", " << c.y << ")" << endl;
				cout << "Polygon Vertices:" << endl;
				printPolygon(temp.cellPoly);
			}


			// check if we are at start
			if (pointPolyCollision(start, temp.cellPoly) && !startFound){
				startGridX = i;
				startGridY = j;
				startFound = true;

				// debug message
				cout << "---- Found Start ----" << endl;
				cout << "Grid Location: (" << startGridX << ", " << startGridY << ")" << endl;
				cout << "Center Location: (" << c.x << ", " << c.y << ")" << endl;
				cout << "Polygon Vertices:" << endl;
				printPolygon(temp.cellPoly);
			}



			if(polygonObstacleCollision(temp.cellPoly)){
				temp.value = 1;
			}
			// store the index of the cell
			temp.x = i;
			temp.y = j;

			// add the cell to the grid
			grid[i][j] = temp;

		}

	}

	if(!startFound){
		cout << "Start was not found" << endl;
	}
}


////////////////////////////////////////////////////////////////////////////////
//  wavefront
////////////////////////////////////////////////////////////////////////////////
// make the wavefront for the grid map based on obstacles
void map::wavefront(){

	std::list<cell> cellList;

	// cout << "Size of cellList: " << cellList.size() << endl;
	// add goal to cellList
	cell currentCell = grid[goalGridX][goalGridY];
	cellList.push_back(currentCell);

	// int n,e,s,w; // cardinal directions

	while (cellList.size() != 0){
		// debugging
		// cout << currentCell.x << ", " << currentCell.y << endl;
		currentCell = cellList.front();

		// cout << "Current Cell Value: " << currentCell.value << endl;

		// on the left wall and takes care of the corners
		if(currentCell.x == 0){

			if(currentCell.y == 0){ // only north and east

				// add north
				if(grid[currentCell.x][currentCell.y+1].value == 0 && grid[currentCell.x][currentCell.y+1].value !=1){
					grid[currentCell.x][currentCell.y+1].value = currentCell.value + 1;
					cellList.push_back(grid[currentCell.x][currentCell.y+1]);
				}

				// add east
				if(grid[currentCell.x+1][currentCell.y].value == 0 && grid[currentCell.x+1][currentCell.y].value !=1){
					grid[currentCell.x+1][currentCell.y].value = currentCell.value + 1;
					cellList.push_back(grid[currentCell.x+1][currentCell.y]);
				}


			}
			else if (currentCell.y == yNumCells-1 ){ // only south and east

				// add south
				if(grid[currentCell.x][currentCell.y-1].value == 0 && grid[currentCell.x][currentCell.y-1].value !=1){
					grid[currentCell.x][currentCell.y-1].value = currentCell.value + 1;
					cellList.push_back(grid[currentCell.x][currentCell.y-1]);
				}

				// add east
				if(grid[currentCell.x+1][currentCell.y].value == 0 && grid[currentCell.x+1][currentCell.y].value !=1){
					grid[currentCell.x+1][currentCell.y].value = currentCell.value + 1;
					cellList.push_back(grid[currentCell.x+1][currentCell.y]);
				}

			}
			else{ // only north,east, and south

				// add north
				if(grid[currentCell.x][currentCell.y+1].value == 0 && grid[currentCell.x][currentCell.y+1].value !=1){
					grid[currentCell.x][currentCell.y+1].value = currentCell.value + 1;
					cellList.push_back(grid[currentCell.x][currentCell.y+1]);
				}

				// add south
				if(grid[currentCell.x][currentCell.y-1].value == 0 && grid[currentCell.x][currentCell.y-1].value !=1){
					grid[currentCell.x][currentCell.y-1].value = currentCell.value + 1;
					cellList.push_back(grid[currentCell.x][currentCell.y-1]);
				}

				// add east
				if(grid[currentCell.x+1][currentCell.y].value == 0 && grid[currentCell.x+1][currentCell.y].value !=1){
					grid[currentCell.x+1][currentCell.y].value = currentCell.value + 1;
					cellList.push_back(grid[currentCell.x+1][currentCell.y]);
				}


			}

		}

		// on the right wall and takes care of the corners
		else if(currentCell.x == xNumCells-1){
			if(currentCell.y == 0){ // only north and west

				// add north
				if(grid[currentCell.x][currentCell.y+1].value == 0 && grid[currentCell.x][currentCell.y+1].value !=1){
					grid[currentCell.x][currentCell.y+1].value = currentCell.value + 1;
					cellList.push_back(grid[currentCell.x][currentCell.y+1]);
				}

				// add west
				if(grid[currentCell.x-1][currentCell.y].value == 0 && grid[currentCell.x-1][currentCell.y].value !=1){
					grid[currentCell.x-1][currentCell.y].value = currentCell.value + 1;
					cellList.push_back(grid[currentCell.x-1][currentCell.y]);
				}


			}
			else if (currentCell.y == yNumCells-1 ){ // only south and west

				// add south
				if(grid[currentCell.x][currentCell.y-1].value == 0 && grid[currentCell.x][currentCell.y-1].value !=1){
					grid[currentCell.x][currentCell.y-1].value = currentCell.value + 1;
					cellList.push_back(grid[currentCell.x][currentCell.y-1]);
				}

				// add west
				if(grid[currentCell.x-1][currentCell.y].value == 0 && grid[currentCell.x-1][currentCell.y].value !=1){
					grid[currentCell.x-1][currentCell.y].value = currentCell.value + 1;
					cellList.push_back(grid[currentCell.x-1][currentCell.y]);
				}

			}
			else{ // only north,east, and south

				// add south
				if(grid[currentCell.x][currentCell.y-1].value == 0 && grid[currentCell.x][currentCell.y-1].value !=1){
					grid[currentCell.x][currentCell.y-1].value = currentCell.value + 1;
					cellList.push_back(grid[currentCell.x][currentCell.y-1]);
				}

				// add north
				if(grid[currentCell.x][currentCell.y+1].value == 0 && grid[currentCell.x][currentCell.y+1].value !=1){
					grid[currentCell.x][currentCell.y+1].value = currentCell.value + 1;
					cellList.push_back(grid[currentCell.x][currentCell.y+1]);
				}

				// add west
				if(grid[currentCell.x-1][currentCell.y].value == 0 && grid[currentCell.x-1][currentCell.y].value !=1){
					grid[currentCell.x-1][currentCell.y].value = currentCell.value + 1;
					cellList.push_back(grid[currentCell.x-1][currentCell.y]);
				}


			}
		}

		// on the bottom of the map
		else if(currentCell.y == 0){ // west,north, and east

			// add east
			if(grid[currentCell.x+1][currentCell.y].value == 0 && grid[currentCell.x+1][currentCell.y].value !=1){
				grid[currentCell.x+1][currentCell.y].value = currentCell.value + 1;
				cellList.push_back(grid[currentCell.x+1][currentCell.y]);
			}

			// add north
			if(grid[currentCell.x][currentCell.y+1].value == 0 && grid[currentCell.x][currentCell.y+1].value !=1){
				grid[currentCell.x][currentCell.y+1].value = currentCell.value + 1;
				cellList.push_back(grid[currentCell.x][currentCell.y+1]);
			}

			// add west
			if(grid[currentCell.x-1][currentCell.y].value == 0 && grid[currentCell.x-1][currentCell.y].value !=1){
				grid[currentCell.x-1][currentCell.y].value = currentCell.value + 1;
				cellList.push_back(grid[currentCell.x-1][currentCell.y]);
			}

		}

		// on the top of the map
		else if(currentCell.y == yNumCells-1){ // west, south, and east

			// add east
			if(grid[currentCell.x+1][currentCell.y].value == 0 && grid[currentCell.x+1][currentCell.y].value !=1){
				grid[currentCell.x+1][currentCell.y].value = currentCell.value + 1;
				cellList.push_back(grid[currentCell.x+1][currentCell.y]);
			}

			// add south
			if(grid[currentCell.x][currentCell.y-1].value == 0 && grid[currentCell.x][currentCell.y-1].value !=1){
				grid[currentCell.x][currentCell.y-1].value = currentCell.value + 1;
				cellList.push_back(grid[currentCell.x][currentCell.y-1]);
			}

			// add west
			if(grid[currentCell.x-1][currentCell.y].value == 0 && grid[currentCell.x-1][currentCell.y].value !=1){
				grid[currentCell.x-1][currentCell.y].value = currentCell.value + 1;
				cellList.push_back(grid[currentCell.x-1][currentCell.y]);
			}

		}

		else{ // n,e,s,w neighbors are not outside of the map

			// add north
			if(grid[currentCell.x][currentCell.y+1].value == 0 && grid[currentCell.x][currentCell.y+1].value !=1){
				grid[currentCell.x][currentCell.y+1].value = currentCell.value + 1;
				cellList.push_back(grid[currentCell.x][currentCell.y+1]);
			}

			// add east
			if(grid[currentCell.x+1][currentCell.y].value == 0 && grid[currentCell.x+1][currentCell.y].value !=1){
				grid[currentCell.x+1][currentCell.y].value = currentCell.value + 1;
				cellList.push_back(grid[currentCell.x+1][currentCell.y]);
			}

			// add south
			if(grid[currentCell.x][currentCell.y-1].value == 0 && grid[currentCell.x][currentCell.y-1].value !=1){
				grid[currentCell.x][currentCell.y-1].value = currentCell.value + 1;
				cellList.push_back(grid[currentCell.x][currentCell.y-1]);
			}

			// add west
			if(grid[currentCell.x-1][currentCell.y].value == 0 && grid[currentCell.x-1][currentCell.y].value !=1){
				grid[currentCell.x-1][currentCell.y].value = currentCell.value + 1;
				cellList.push_back(grid[currentCell.x-1][currentCell.y]);
			}

		}

		// update Grid
		// cout << "Size of cellList: " << cellList.size() << endl;
		// take off the back element
		cellList.pop_front();



	}

	// cout << "Size of cellList: " << cellList.size() << endl;

}


////////////////////////////////////////////////////////////////////////////////
//  wavefrontPath
////////////////////////////////////////////////////////////////////////////////
// make the wavefront for the grid map based on obstacles
void map::wavefrontPath(string filename){


	cell currentCell = grid[startGridX][startGridY];

	cell minValCell; // cell with min value

	int n,e,s,w; // cardinal directions


	bool notAtGoal = true;

	ofstream wavefrontFile(filename.c_str());

	while(notAtGoal){

		minValCell = grid[currentCell.x][currentCell.y];

		// define cardinal directions
		n = currentCell.y + 1;
		e = currentCell.x + 1;
		s = currentCell.y - 1;
		w = currentCell.x - 1;

		// north cell
		if(n <= yNumCells-1){
			if(grid[currentCell.x][n].value != 1 && grid[currentCell.x][n].value < minValCell.value ){
				minValCell = grid[currentCell.x][n];
			}

			// cout << "North Value: " << grid[currentCell.x][n].value << endl;
		}

		// east cell
		if(e <= xNumCells-1){
			if(grid[e][currentCell.y].value != 1 && grid[e][currentCell.y].value < minValCell.value ){
				minValCell = grid[e][currentCell.y];
			}
			// cout << "East Value: " << grid[e][currentCell.y].value << endl;
		}

		// south cell
		if(s >= 0){
			if(grid[currentCell.x][s].value != 1 && grid[currentCell.x][s].value < minValCell.value ){
				minValCell = grid[currentCell.x][s];
			}
			// cout << "South Value: " << grid[currentCell.x][s].value << endl;
		}

		// west cell
		if(w >= 0){
			if(grid[w][currentCell.y].value != 1 && grid[w][currentCell.y].value < minValCell.value ){
				minValCell = grid[w][currentCell.y];
			}
			// cout << "West Value: " << grid[w][currentCell.y].value << endl;
		}

		wavefrontFile << minValCell.center.x << ", " << minValCell.center.y << endl;

		currentCell = minValCell;

		if(currentCell.x == goalGridX && currentCell.y == goalGridY){
			notAtGoal = false;
		}
	}

	wavefrontFile.close();

	cout << "FINALLY" << endl;

}

//////////////////   Wavefront with passed C-Space /////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//  wavefrontCSpace
////////////////////////////////////////////////////////////////////////////////
// make the wavefront for the grid map based on obstacles
void map::wavefrontCSpace(std::vector<std::vector<cell>> *c_grid, int gGX, int gGY, int xNC, int yNC){

	std::list<cell> cellList;
	// cout << "Max size of list: " << cellList.max_size() << endl;
	// cout << "Size of cellList: " << cellList.size() << endl;
	// add goal to cellList
	cell currentCell = (*c_grid)[gGX][gGY];

	cout << "Current Cell: " << currentCell.x << ", " << currentCell.y << endl;
	cellList.push_back(currentCell);
	int n,e,s,w;
	// int n,e,s,w; // cardinal directions

	while (cellList.size() != 0){

		// cout << "Current Value: " << currentCell.value << endl;
		// cout << "Current X,Y Grid Pos : " << currentCell.x << ", " << currentCell.y << endl;
		currentCell = cellList.front();
		// debugging
		// define cardinal directions
		n = currentCell.y + 1;
		e = currentCell.x + 1;
		s = currentCell.y - 1;
		w = currentCell.x - 1;

		// cout << "North: " << n << endl ;
		// cout << "East: " << e << endl ;
		// cout << "South: " << s << endl ;
		// cout << "West: " << w << endl ;

		// north cell
		if(n > yNC-1){
			n = 0;
			if((*c_grid)[currentCell.x][n].value != 1 && (*c_grid)[currentCell.x][n].value ==0){
				(*c_grid)[currentCell.x][n].value = currentCell.value + 1;
				cellList.push_back((*c_grid)[currentCell.x][n]);
				// cout << "North Good 1" << endl;
			}
		}
		else{
			if((*c_grid)[currentCell.x][n].value != 1 && (*c_grid)[currentCell.x][n].value ==0){
				(*c_grid)[currentCell.x][n].value = currentCell.value + 1;
				cellList.push_back((*c_grid)[currentCell.x][n]);
				// cout << "North Good 2" << endl;
			}

			// cout << "North Value: " << grid[currentCell.x][n].value << endl;
		}

		// east cell
		if(e > xNC-1){
			e = 0;
			if((*c_grid)[e][currentCell.y].value != 1 && (*c_grid)[e][currentCell.y].value == 0 ){
				(*c_grid)[e][currentCell.y].value = currentCell.value + 1;
				cellList.push_back((*c_grid)[e][currentCell.y]);
				// cout << "East Good 1" << endl;
			}
		}
		else{
			if((*c_grid)[e][currentCell.y].value != 1 && (*c_grid)[e][currentCell.y].value == 0 ){
				(*c_grid)[e][currentCell.y].value = currentCell.value + 1;
				cellList.push_back((*c_grid)[e][currentCell.y]);
				// cout << "East Good 2" << endl;
			}
			// cout << "East Value: " << grid[e][currentCell.y].value << endl;
		}

		// south cell
		if(s < 0){
			s = yNC-1;
			if((*c_grid)[currentCell.x][s].value != 1 && (*c_grid)[currentCell.x][s].value == 0){
				(*c_grid)[currentCell.x][s].value = currentCell.value + 1;
				cellList.push_back((*c_grid)[currentCell.x][s]);
				// cout << "South Good 1" << endl;
			}
		}
		else{
			if((*c_grid)[currentCell.x][s].value != 1 && (*c_grid)[currentCell.x][s].value == 0){
				(*c_grid)[currentCell.x][s].value = currentCell.value + 1;
				cellList.push_back((*c_grid)[currentCell.x][s]);
				// cout << "South Good 2" << endl;
			}
			// cout << "South Value: " << grid[currentCell.x][s].value << endl;
		}

		// west cell
		if(w < 0){
			w = xNC-1;
			if((*c_grid)[w][currentCell.y].value != 1 && (*c_grid)[w][currentCell.y].value == 0){
				(*c_grid)[w][currentCell.y].value = currentCell.value + 1;
				cellList.push_back((*c_grid)[w][currentCell.y]);
				// cout << "West Good 1" << endl;
			}
		}
		else{
			if((*c_grid)[w][currentCell.y].value != 1 && (*c_grid)[w][currentCell.y].value == 0){
				(*c_grid)[w][currentCell.y].value = currentCell.value + 1;
				cellList.push_back((*c_grid)[w][currentCell.y]);
				// cout << "West Good 2" << endl;
			}
		}

		//
		// cout << "Current Value: " << currentCell.value << endl;
		// cout << "Current X,Y Grid Pos : " << currentCell.x << ", " << currentCell.y << endl;

		cellList.pop_front();

		cout << "Size of cellList: " << cellList.size() << endl;

	}


	cout << "Size of cellList at end: " << cellList.size() << endl;

}

////////////////////////////////////////////////////////////////////////////////
//  wavefrontPathCSpace
////////////////////////////////////////////////////////////////////////////////
// make the wavefront for the grid map based on obstacles
void map::wavefrontPathCSpace(string filename,std::vector<std::vector<cell>> c_grid,int gGX, int gGY,int sGX, int sGY, int xNC, int yNC){

	cell currentCell = c_grid[sGX][sGY];

	cell minValCell; // cell with min value

	int n,e,s,w; // cardinal directions

	int count = 0;
	bool notAtGoal = true;

	ofstream wavefrontFile(filename.c_str());

	while(notAtGoal){

		minValCell = c_grid[currentCell.x][currentCell.y];
		cout << count << endl;

		// define cardinal directions
		n = currentCell.y + 1;
		e = currentCell.x + 1;
		s = currentCell.y - 1;
		w = currentCell.x - 1;
		// cout << "North: " << n << endl ;
		// cout << "East: " << e << endl ;
		// cout << "South: " << s << endl ;
		// cout << "West: " << w << endl ;

		// north cell
		if(n>yNC-1){

			n = 0;
			if(c_grid[currentCell.x][n].value != 1 && c_grid[currentCell.x][n].value < minValCell.value ){
				minValCell = c_grid[currentCell.x][n];
				cout << "North if" << endl;
			}
		}
		else{

			if(c_grid[currentCell.x][n].value != 1 && c_grid[currentCell.x][n].value < minValCell.value ){
				minValCell = c_grid[currentCell.x][n];
				cout << "North else" << endl;
			}

		}

		// east cell
		if(e > xNC-1){
			e = 0;
			if(c_grid[e][currentCell.y].value != 1 && c_grid[e][currentCell.y].value < minValCell.value ){
				minValCell = c_grid[e][currentCell.y];
				cout << "East if" << endl;
			}
		}
		else{
			if(c_grid[e][currentCell.y].value != 1 && c_grid[e][currentCell.y].value < minValCell.value ){
				minValCell = c_grid[e][currentCell.y];
				cout << "East else" << endl;
			}
			// cout << "East Value: " << grid[e][currentCell.y].value << endl;
		}

		// south cell
		if(s < 0){
			s = yNC -1;
			if(c_grid[currentCell.x][s].value != 1 && c_grid[currentCell.x][s].value < minValCell.value ){
				minValCell = c_grid[currentCell.x][s];
				cout << "South if" << endl;
			}
		}
		else{
			if(c_grid[currentCell.x][s].value != 1 && c_grid[currentCell.x][s].value < minValCell.value ){
				minValCell = c_grid[currentCell.x][s];
				cout << "South else" << endl;
			}
			// cout << "South Value: " << grid[currentCell.x][s].value << endl;
		}



		// west cell

		if(w < 0){
			w = xNC-1;
			if(c_grid[w][currentCell.y].value != 1 && c_grid[w][currentCell.y].value < minValCell.value ){
				minValCell = c_grid[w][currentCell.y];
				cout << "West if" << endl;
			}
		}
		else{
			if(c_grid[w][currentCell.y].value != 1 && c_grid[w][currentCell.y].value < minValCell.value ){
				minValCell = c_grid[w][currentCell.y];
				cout << "West else" << endl;
			}
			// cout << "West Value: " << grid[w][currentCell.y].value << endl;
		}

		//
		wavefrontFile << minValCell.center.x << ", " << minValCell.center.y << endl;

		cout << "New Min: " <<minValCell.center.x << ", " << minValCell.center.y << endl;
		currentCell = minValCell;

		if(currentCell.x == gGX && currentCell.y == gGY){
			notAtGoal = false;
		}

		count ++;
	}

	wavefrontFile.close();

	cout << "FINALLY" << endl;

}


////////////////////////////////////////////////////////////////////////////////
//  dist()
////////////////////////////////////////////////////////////////////////////////
// return distance between two points/Vertices
double map::dist(vertex v1, vertex v2){

	return sqrt((pow(v1.x-v2.x,2.0) +  pow(v1.y-v2.y,2.0)));

}

////////////////////////////   Debugging   /////////////////////////////////////
//
//
////////////////////////////////////////////////////////////////////////////////
//  printObs
////////////////////////////////////////////////////////////////////////////////
// print vertices of all obstacles in map
void map::printObs(){

	int polyCount = 1;

	for (const auto& poly_it : obstacles){

		cout << "Vertices of Polgyon #" << polyCount << endl;

		for (const auto& vert_it : poly_it.vertices){
			cout << vert_it.x << "," << vert_it.y << endl;
		}

		polyCount++;
	}

}


////////////////////////////////////////////////////////////////////////////////
//  printPolygon
////////////////////////////////////////////////////////////////////////////////
// print the vertices of a polygon. good for debugging
void map::printPolygon(polygon p){
	// cout << "Polygon Vertices" << endl;
	for(const auto it: p.vertices){
		cout << it.x << "," << it.y << endl;
	}
}

////////////////////////////////////////////////////////////////////////////////
//  printPolygon
////////////////////////////////////////////////////////////////////////////////
// print the vertices of a polygon. good for debugging
void map::printPolygons(){
	// cout << "Polygon Vertices" << endl;
	int count =1;

	for(const auto p_it:obstacles){
		cout << "Polygon #" << count << endl;
		printPolygon(p_it);
		count ++;
	}

}
