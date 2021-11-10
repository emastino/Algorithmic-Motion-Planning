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
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <chrono>				// time delay and measure elapsed time
#include <random>


#include "Eigen/Dense"

// namespaces
using namespace Eigen;
using namespace std;

// constants
#define PI 3.14159265


// vertex structure
struct vertex{
	double x{};
	double y{};
};

// Polygon struct
struct polygon{
	std::vector<vertex> vertices;
};

// grid cell structure for wavefron planner
struct cell{
	polygon cellPoly; // polygon defining cell
	vertex 	center; // center of polygon
	double 	value = 0; // velue used in the wavefront. Default 0, 1 for obstacles

	int x,y;	// position in the vectors of the cell

};

// Graph Node
struct node{
	int graph_number;
	vertex loc; // location of the node
	double h; // heuristic value
	double g;  // distance from it to start
	double f = 1000000.0; // some large value
	bool inO = false;
	bool inC = false;
	std::vector<int> adjacencts; // adjacent nodes
	std::vector<double> weights; // distance from one node to another

	int backpointer; // used for search algorithms to point to it backpointer/parent
									// node in the contect of the graph

	int timeStamp;
};

// structure to store PRM benchmark data for each run
struct PRM_Benchmark{

	int validSolution; // 1 if valid solution with a pth, 0 if no path can be generated
	double pathLength; // length of the path generated
	double compTime; 		// computation time of PRM in microsecond counts
};


struct robot{
	int robotNumber;
	vertex start; // robot start location
	int start_index;
	vertex goal;  // robot goal location
	int goal_index;
	double R = 0.50;
	bool atGoal = false;

};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// Map class
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
class map {
	double xMapMin, xMapMax, yMapMin, yMapMax, sx, sy,gx,gy;
	int numbOfRobots;

	public:
	std::vector<polygon> obstacles; // obstacles in the workspace
	std::vector<polygon> c_space_obs;	// c-space representation of the obstacles
	std::vector<std::vector<cell>> grid; // grid representation of the map
	int startGridX, startGridY, goalGridX, goalGridY, xNumCells, yNumCells;

	// graph stuff
	int goalNode, startNode;
	// std::vector<node> graphNodes;
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


	// PRMs and Graphs
	std::vector<robot> robots;
	void addNode(int, vertex, std::vector<node>*);
	void AstarSearch(std::vector<node> *);
	bool DijkstrasSearch(std::vector<node> *,bool ,bool,PRM_Benchmark *);
	PRM_Benchmark PRM(int, double,bool, bool, bool);
	PRM_Benchmark GoalBiasRRT(int,double,double,double, bool, bool);
	PRM_Benchmark GoalBiasRRT_Centralized(int,double,double,double, bool, bool, int);
	PRM_Benchmark GoalBiasRRT_Decentralized(int,double,double,double, bool, bool, int);
	void addRobots(std::vector<robot>);
	double minDistanceToAllObs(vertex);
	void printGraph(std::vector<node> *);


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
// check if a polygon collides with any obstacle on the map
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

	// vertex temp;
	//
	// double lambda = 0;
	// double delta_lambda = 0.01;
	//
	//
	// while (lambda <= 1){
	//
	// 	temp.x = v1.x*(1-lambda) + v2.x*lambda;
	// 	temp.y = v1.y*(1-lambda) + v2.y*lambda;
	//
	// 	if(pointCollision(temp)) {return true;}
	// 	else {lambda = lambda + delta_lambda;}
	//
	// }

	polygon poly;
	poly.vertices.push_back(v1);
	poly.vertices.push_back(v2);



	return polygonObstacleCollision(poly);

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
//  addANode
////////////////////////////////////////////////////////////////////////////////
// add a node to the graph
void map::addNode(int node_num, vertex location, std::vector<node> *graphNodes){

		node nodeToAdd;
		// cout << "Made Node" << endl;
		nodeToAdd.graph_number = node_num;
		nodeToAdd.loc = location;
		// cout << "-------------------------" << endl;
		// cout << "Node Added" << endl;
		// cout << "Node Number: " << nodeToAdd.graph_number << endl;
		// cout <<  "Node Location: " << nodeToAdd.loc.x << ", " << nodeToAdd.loc.y << endl;

		(*graphNodes).push_back(nodeToAdd);
}



////////////////////////////////////////////////////////////////////////////////
// A* Search
////////////////////////////////////////////////////////////////////////////////

void map::AstarSearch(std::vector<node> *graphNodes){

	int minNode = 0; 	// keep track of the node with min heuristic, initialize as
										// start node

	std::vector<node> graphNodesCopy = *graphNodes;


	std::vector<node> OList;
	std::vector<node> CList;
	// distance from start to start is 0
	graphNodesCopy[minNode].g = 0;
	graphNodesCopy[minNode].f = graphNodesCopy[minNode].g + graphNodesCopy[minNode].h;
	OList.push_back(graphNodesCopy[minNode]); // current node is start node
	graphNodesCopy[minNode].inO = true;

	int numberOfIterations = 0;

	cout << "             A* Started             " << endl;


	while (OList.size()>0){

		// cout << "----------------------------------" << endl;
		minNode = OList.front().graph_number; // make the min the first in the list

		// pick one with smallest heuristic in the OList
		int minNode_index = 0;
		int count = 0;
		for(const auto& OList_it : OList){
			if(OList_it.f < graphNodesCopy[minNode].f){
				minNode = OList_it.graph_number;
				minNode_index = count;
			}
			count++;
		}

		// cout << "Node With Min Heuristic: " << minNode << endl;
		// // earse min heuristic node from OList
		OList.erase(OList.begin() + minNode_index-1);
		graphNodesCopy[minNode].inC = false;

		// Add the min heuristic node to CList
		CList.push_back(graphNodesCopy[minNode]);
		graphNodesCopy[minNode].inC = true;

		// check if Goal Node is found
		if(minNode == goalNode){
			cout << "Goal Found" << endl;
			break;
		}

		// look at all of the neigbors of node with min heuristic
		for(int i = 0;  i < graphNodesCopy[minNode].adjacencts.size(); i++){

			int neighborNode = graphNodesCopy[minNode].adjacencts[i];
			int neighborNodeWeight = graphNodesCopy[minNode].weights[i];

			// cout << "Neighbor Node: " << neighborNode << endl;
			// cout << "Neighbor Node Weight: " << neighborNodeWeight<< endl;
			if(!graphNodesCopy[neighborNode].inC){
				// cout << "Node not in C" << endl;
				if(!graphNodesCopy[neighborNode].inO){
					// cout << "Node not in O" << endl;
					graphNodesCopy[neighborNode].g = graphNodesCopy[minNode].g + neighborNodeWeight;
					graphNodesCopy[neighborNode].f = graphNodesCopy[neighborNode].g + graphNodesCopy[neighborNode].h;
					OList.push_back(graphNodesCopy[neighborNode]);
					graphNodesCopy[neighborNode].inO = true;
					graphNodesCopy[neighborNode].backpointer = minNode;


				}
				else{
					// cout << "Node in O" << endl;
					if(graphNodesCopy[minNode].g + neighborNodeWeight < graphNodesCopy[neighborNode].g){
						// cout << "Update Back Pointer to of "<< neighborNode<< " to " << minNode << endl;
						graphNodesCopy[neighborNode].g = graphNodesCopy[minNode].g + neighborNodeWeight;
						graphNodesCopy[neighborNode].backpointer = minNode;
					}
				}

			}
		}


		numberOfIterations++;

	}



	cout << "---------- Path Information  ----------" << endl;

	// print path
	int currentNode = goalNode;

	while(currentNode != startNode){

		cout << currentNode << "<-" ;
		currentNode = graphNodesCopy[currentNode].backpointer;
	}
	cout << endl;
	cout << "Number of iterations: " << numberOfIterations << endl;
	cout << "Path Length: " << graphNodesCopy[goalNode].g << endl;
	cout << "---------------------------------------" << endl;

	cout << "             A* Complete             " << endl;
}


////////////////////////////////////////////////////////////////////////////////
// Dijkstra's Search
////////////////////////////////////////////////////////////////////////////////

bool map::DijkstrasSearch(std::vector<node> *graphNodes,bool outputPath, bool smoothing, PRM_Benchmark *benchM){

	bool goalFound = false;

	int minNode = 0; 	// keep track of the node with min heuristic, initialize as
										// start node

	std::vector<node> graphNodesCopy = *graphNodes;


	std::vector<node> OList;
	std::vector<node> CList;
	// distance from start to start is 0
	graphNodesCopy[minNode].g = 0;
	graphNodesCopy[minNode].f = graphNodesCopy[minNode].g;
	OList.push_back(graphNodesCopy[minNode]); // current node is start node
	graphNodesCopy[minNode].inO = true;

	int numberOfIterations = 0;

	// cout << "             Dijkstra's Started             " << endl;


	while (OList.size()>0){

		// cout << "----------------------------------" << endl;
		minNode = OList.front().graph_number; // make the min the first in the list

		// // pick one with smallest heuristic in the OList
		// int minNode_index = 0;
		// int count = 0;
		// for(const auto& OList_it : OList){
		// 	if(OList_it.f < graphNodesCopy[minNode].f){
		// 		minNode = OList_it.graph_number;
		// 		minNode_index = count;
		// 	}
		// 	count++;
		// }

		// cout << "Node With Min Heuristic: " << minNode << endl;
		// // earse min heuristic node from OList
		OList.erase(OList.begin());
		graphNodesCopy[minNode].inC = false;

		// Add the min heuristic node to CList
		CList.push_back(graphNodesCopy[minNode]);
		graphNodesCopy[minNode].inC = true;

		// check if Goal Node is found
		if(minNode == goalNode){
			// cout << "Goal Found" << endl;
			goalFound = true;
			break;
		}

		// look at all of the neigbors of node with min heuristic
		for(int i = 0;  i < graphNodesCopy[minNode].adjacencts.size(); i++){

			int neighborNode = graphNodesCopy[minNode].adjacencts[i];
			double neighborNodeWeight = graphNodesCopy[minNode].weights[i];

			// cout << "Neighbor Node: " << neighborNode << endl;
			// cout << "Neighbor Node Weight: " << neighborNodeWeight<< endl;
			if(!graphNodesCopy[neighborNode].inC){
				// cout << "Node not in C" << endl;
				if(!graphNodesCopy[neighborNode].inO){
					// cout << "Node not in O" << endl;
					graphNodesCopy[neighborNode].g = graphNodesCopy[minNode].g + neighborNodeWeight;
					graphNodesCopy[neighborNode].f = graphNodesCopy[neighborNode].g;
					OList.push_back(graphNodesCopy[neighborNode]);
					graphNodesCopy[neighborNode].inO = true;
					graphNodesCopy[neighborNode].backpointer = minNode;


				}
				else{
					// cout << "Node in O" << endl;
					if(graphNodesCopy[minNode].g + neighborNodeWeight < graphNodesCopy[neighborNode].g){
						// cout << "Update Back Pointer to of "<< neighborNode<< " to " << minNode << endl;
						graphNodesCopy[neighborNode].g = graphNodesCopy[minNode].g + neighborNodeWeight;
						graphNodesCopy[neighborNode].backpointer = minNode;
					}
				}

			}
		}


		numberOfIterations++;

	}


	if(goalFound){ // if goal was found
		// reverse the order of the list to get better smooting performance

		// no smoothing
		if(!smoothing){ // output path to file and no smoothing
			std::vector<int> pathNodeNumbers;
			int currentNode = goalNode;
			pathNodeNumbers.push_back(currentNode);

			while(currentNode != startNode){
				currentNode = graphNodesCopy[currentNode].backpointer;
				pathNodeNumbers.push_back(currentNode);

			}

			// for outputPath no smoothing
			(*benchM).validSolution = 1;
			(*benchM).pathLength = graphNodesCopy[goalNode].g;

			// cout << "UNsmooth Distance = " << (*benchM).pathLength << endl;

			pathNodeNumbers.resize(pathNodeNumbers.size());

			if(outputPath){
				ofstream PRM_Path("PRM_Path.txt");
				while(pathNodeNumbers.size()>0){
					PRM_Path << graphNodesCopy[pathNodeNumbers.back()].loc.x << ", " << graphNodesCopy[pathNodeNumbers.back()].loc.y << endl;
					pathNodeNumbers.pop_back();
				}
				PRM_Path.close();
			}
		}

		// smoothing
		// else{
		else{
			// cout << "---------- Path Information  ----------" << endl;
			std::vector<node> orderedPathVector;

			int currentNodeIndex = goalNode;
			int tempIndex;

			orderedPathVector.push_back(graphNodesCopy[currentNodeIndex]);

			node tempNode;

			while(currentNodeIndex != startNode){
				tempIndex = currentNodeIndex;
				currentNodeIndex = graphNodesCopy[currentNodeIndex].backpointer;
				tempNode = graphNodesCopy[currentNodeIndex];
				tempNode.backpointer = tempIndex;
				orderedPathVector.push_back(tempNode);
			}
			tempNode = graphNodesCopy[currentNodeIndex];
			tempNode.backpointer = tempIndex;
			orderedPathVector.push_back(tempNode);

			orderedPathVector.resize(orderedPathVector.size());

			std::vector<int> pathNodeNumbers;
			int currentNode = startNode;
			int nextNode, prevNextNode;
			double smoothDistance = 0;
			// pathNodeNumbers.push_front(currentNode);
			pathNodeNumbers.push_back(currentNode);

			// initialize
			nextNode = orderedPathVector.back().backpointer;
			prevNextNode = nextNode;

			while(nextNode != goalNode){

				if( lineCollision(graphNodesCopy[currentNode].loc,  graphNodesCopy[nextNode].loc) ){
					smoothDistance = smoothDistance + dist(graphNodesCopy[currentNode].loc, graphNodesCopy[prevNextNode].loc);
					currentNode = prevNextNode;
					pathNodeNumbers.push_back(currentNode);
				}
				else{
					orderedPathVector.pop_back();
					prevNextNode = nextNode;
					nextNode = orderedPathVector.back().backpointer;
				}

				// if next node == startNode
				if(nextNode == goalNode){

					if( lineCollision(graphNodesCopy[currentNode].loc,  graphNodesCopy[nextNode].loc) ){
						smoothDistance = smoothDistance + dist(graphNodesCopy[currentNode].loc, graphNodesCopy[prevNextNode].loc);
						currentNode = prevNextNode;
						pathNodeNumbers.push_back(currentNode);
						pathNodeNumbers.push_back(goalNode);
					}
					else{
						smoothDistance = smoothDistance + dist(graphNodesCopy[currentNode].loc, graphNodesCopy[goalNode].loc);
						currentNode = goalNode;
						pathNodeNumbers.push_back(goalNode);
					}
				}

			}

			// for outputPath no smoothing
			(*benchM).validSolution = 1;
			(*benchM).pathLength = smoothDistance;
			// cout << "Smooth Distance = " << (*benchM).pathLength << endl;

			pathNodeNumbers.resize(pathNodeNumbers.size());

			if(outputPath){
				if(smoothing){
					ofstream PRM_Path("PRM_Path_Smooth.txt");
					while(pathNodeNumbers.size()>0){
						PRM_Path << graphNodesCopy[pathNodeNumbers.back()].loc.x << ", " << graphNodesCopy[pathNodeNumbers.back()].loc.y << endl;
						pathNodeNumbers.pop_back();
					}
					PRM_Path.close();
				}
			}
		}
	}

	else{
		// cout << "               PATH NOT FOUND               " << endl;
		// cout << "            Dijkstra's Completed            " << endl;
		(*benchM).validSolution = 0;
		(*benchM).pathLength = 0;
	}


	return goalFound;
}



////////////////////////////////////////////////////////////////////////////////
// PRM()
////////////////////////////////////////////////////////////////////////////////
// connects all edges and finds a path once all of the nodes are present in
// the graph
PRM_Benchmark map::PRM(int n, double r, bool outputPath, bool outPutPRM, bool smooth){

	PRM_Benchmark bm;

	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();


	// random
	unsigned seedX = std::chrono::steady_clock::now().time_since_epoch().count();
	// cout << seedX << endl;
	srand (seedX);
	// xMapMin, xMapMax, yMapMin, yMapMax
	std::vector<node> graphNodes;

	// add start node
	vertex startNodeLocation;
	startNodeLocation.x = sx; startNodeLocation.y = sy;
	addNode(0, startNodeLocation, &graphNodes);

	// add goal node
	vertex goalNodeLocation;
	goalNodeLocation.x = gx; goalNodeLocation.y = gy;
	addNode(1, goalNodeLocation, &graphNodes);

	// temp node
	vertex temp;

	int numberOfNodes = 2;
	// start at 2 because we have already added the start and goal nodes
	for( int i = 0; i < n; i++){

		temp.x = (double) rand()*(xMapMax-xMapMin)/RAND_MAX +xMapMin; //(xMapMin-xMapMax) + xMapMin;
		temp.y = (double) rand()*(yMapMax-yMapMin)/RAND_MAX +yMapMin; //(yMapMin-yMapMax) + yMapMin;
		// temp.x = randomX ;//(double) rand()*(xMapMax-xMapMin)/RAND_MAX +xMapMin; //(xMapMin-xMapMax) + xMapMin;
	  // temp.y = randomY ;//(double) rand()*(yMapMax-yMapMin)/RAND_MAX +yMapMin; //(yMapMin-yMapMax) + yMapMin;
		// cout << "random location in map:  " << temp.x << ", " << temp.y << endl;

		if(!pointCollision(temp)){
			addNode(numberOfNodes,temp, &graphNodes);

			// cout << "No Collision. Added node #" <<  numberOfNodes << endl;

			numberOfNodes++;

		}
		else{
			// cout << "Collision: " << temp.x << ", " << temp.y << endl;
		}


	}

	// cout << "Total Nodes: " << numberOfNodes << endl;


	// connect the graph

	// resize graph
	graphNodes.resize(numberOfNodes);
	graphNodes[0].g = 0; // set start g =0
	graphNodes[0].h = dist(graphNodes[0].loc, goalNodeLocation);
	startNode = graphNodes[0].graph_number;
	// goal heuristic
	graphNodes[1].h = dist(graphNodes[0].loc, goalNodeLocation);
	goalNode = graphNodes[1].graph_number;


	// pupoltae graph with neighbors and weights
	for(int i = 0 ; i < numberOfNodes; i++){

		graphNodes[i].h = dist(graphNodes[i].loc, goalNodeLocation);

		for(int j = 0; j < numberOfNodes; j++){

			if(j != i){
				// cout << "Distance: " << dist(graphNodes[i].loc, graphNodes[j].loc) << endl;
				if(dist(graphNodes[i].loc,graphNodes[j].loc) <= r && !lineCollision(graphNodes[i].loc,graphNodes[j].loc )){
					// int graph_number;
					// vertex loc; // location of the node
					// double h; // heuristic value
					// double g;  // distance from it to start
					// double f = 1000000.0; // some large value
					// bool inO = false;
					// bool inC = false;
					// std::vector<int> adjacencts; // adjacent nodes
					// std::vector<double> weights; // distance from one node to another
					//
					// int backpointer; // used for search algorithms to point to it backpointer/parent
					// 								// node in the contect of the graph

					graphNodes[i].adjacencts.push_back(j);
					graphNodes[i].weights.push_back( dist(graphNodes[i].loc,graphNodes[j].loc));


					// cout << "Distance h: " << dist(graphNodes[i].loc, goalNodeLocation) << endl;

				}
			}
		}
	}


	// resize all of the wights and adjacent vectors
	for(int i = 0 ; i < numberOfNodes; i++){
		graphNodes[i].adjacencts.resize(graphNodes[i].adjacencts.size());
		graphNodes[i].weights.resize(graphNodes[i].weights.size());
	}


	// printGraph(&graphNodes);
	// int validSolution; // 1 if valid solution with a pth, 0 if no path can be generated
	// double pathLenght; // length of the path generated
	// double compTime; 		// computation time of PRM

	DijkstrasSearch(&graphNodes, outputPath, smooth, &bm);


	// output PRM into a txt file
	if (outPutPRM){
		ofstream PRM_Graph("PRM_Graph.txt");

		for(int i = 0 ; i < numberOfNodes; i++){
			PRM_Graph << graphNodes[i].loc.x << ", " << graphNodes[i].loc.y;
			for(const auto& adj_it : graphNodes[i].adjacencts){

					PRM_Graph << ", "<< graphNodes[adj_it].loc.x << ", " << graphNodes[adj_it].loc.y;
			}
			PRM_Graph << endl;
		}

			PRM_Graph.close();

		}

		std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

		// cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << " [s]" << std::endl;

		bm.compTime = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();

		// cout << "Benchmark Info" << endl;
		// cout << "Path Found: " << bm.validSolution << endl;
		// cout << "Path Lenght: " << bm.pathLength << endl;
		// cout << "Computation Time: " << bm.compTime << " [micro sec]" << endl;

		return bm;
}


////////////////////////////////////////////////////////////////////////////////
// GoalBiasRRT()
////////////////////////////////////////////////////////////////////////////////
// make an RRT biased to goal with prob p

PRM_Benchmark map::GoalBiasRRT(int n,double r,double p,double epsilon, bool outputPath, bool outPutTree){

	PRM_Benchmark bm;
	bm.validSolution = 0;
	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();


	// random
	unsigned seedX = std::chrono::steady_clock::now().time_since_epoch().count();
	// cout << seedX << endl;
	srand (seedX);

	// xMapMin, xMapMax, yMapMin, yMapMax
	std::vector<node> treeNodes;

	// add start node ROOT
	vertex startNodeLocation;
	startNodeLocation.x = sx; startNodeLocation.y = sy;
	addNode(0, startNodeLocation, &treeNodes);
	int numberOfNodes = 1;

	vertex goalNodeLocation;
	goalNodeLocation.x = gx; goalNodeLocation.y = gy;
	// min distance  and initialize
	double minDistanceTree = sqrt((pow(xMapMax-xMapMin,2.0) +  pow(yMapMax-yMapMin,2.0)));

	bool goalFound = false;

	// start at 2 because we have already added the start and goal nodes
	for( int i = 0; i < n; i++){
		vertex temp;
		double randomNum = (double) rand()/RAND_MAX;
		if(randomNum <= p){
			temp.x = gx;
			temp.y = gy;
		}
		else{
			temp.x = (double) rand()*(xMapMax-xMapMin)/RAND_MAX +xMapMin; //(xMapMin-xMapMax) + xMapMin;
			temp.y = (double) rand()*(yMapMax-yMapMin)/RAND_MAX +yMapMin; //(yMapMin-yMapMax) + yMapMin;
		}
		// cout << "q_rand: " << temp.x << ", " << temp.y << endl;

		node minNode;
		// find min node to
		for(const auto& tree_it: treeNodes){
			// cout << "Looking at Node #" << tree_it.graph_number << " at location " << tree_it.loc.x << ", " << tree_it.loc.y << endl;

			if(dist(tree_it.loc,temp)<minDistanceTree){
				minDistanceTree = dist(tree_it.loc,temp);
				minNode = tree_it;
			}
		}

		// cout << "nearest node: " << minNode.graph_number << " at location " << minNode.loc.x << ", " << minNode.loc.y << endl;


		vertex newLoc;
		if(dist(minNode.loc, temp) <=r){
			// new location is temp if it is less than r distance away
			newLoc = temp;
		}
		else{
			// see if node can be epsilon away
			double theta = atan2(temp.y-minNode.loc.y,temp.x-minNode.loc.x );
			// position of new node
			newLoc.x = r*cos(theta)+ minNode.loc.x;
			newLoc.y = r*sin(theta)+ minNode.loc.y;
		}


		// check if subpath has no intersections

		if(!lineCollision(newLoc,minNode.loc)){
			node newNode;
			newNode.backpointer = minNode.graph_number;
			newNode.loc = newLoc;
			newNode.graph_number = numberOfNodes;
			treeNodes.push_back(newNode);
			numberOfNodes++;

			if(dist(newNode.loc, goalNodeLocation)<= epsilon){
				node goaln;
				goaln.backpointer = newNode.graph_number;
				goaln.loc = goalNodeLocation;
				goaln.graph_number = numberOfNodes;
				treeNodes.push_back(goaln);
				goalFound = true;
				numberOfNodes++;
				// cout << "Goal Found" << endl;
				i = i+n;
			}


		}

		minDistanceTree = sqrt((pow(xMapMax-xMapMin,2.0) +  pow(yMapMax-yMapMin,2.0)));
	}

	// cout << "Total Nodes: " << numberOfNodes << endl;

	// resize graph
	treeNodes.resize(numberOfNodes);

	double treePathDist = 0;

	if(goalFound){
		bm.validSolution = 1;
		// vector storing path info
		std::vector<node> treePath;
		int currentIndex = numberOfNodes-1;


		while(treeNodes[currentIndex].backpointer != 0){
			treePath.push_back(treeNodes[currentIndex ]);

			treePathDist = treePathDist + dist(treeNodes[currentIndex ].loc, treeNodes[treeNodes[currentIndex ].backpointer].loc);

			currentIndex = treeNodes[currentIndex ].backpointer;

			// Tree_Path << treeNodes[currentIndex].loc.x << ", " << treeNodes[currentIndex].loc.y ;
			// Tree_Path << treeNodes[treeNodes[currentIndex].backpointer].loc.x << ", " << treeNodes[treeNodes[currentIndex].backpointer].loc.y ;
			// Tree_Path << endl;
		}
		treePathDist = treePathDist + dist(treeNodes[currentIndex ].loc, treeNodes[treeNodes[currentIndex ].backpointer].loc);
		treePath.push_back(treeNodes[currentIndex ]);

		// cout << "Path Distance = " << treePathDist << endl;

		if (outputPath){
			ofstream Tree_Path("Tree_Path.txt");
			int currentIndex = numberOfNodes-1;
			while(currentIndex != 0){

				Tree_Path << treeNodes[currentIndex].loc.x << ", " << treeNodes[currentIndex].loc.y << endl;
				// Tree_Path << treeNodes[treeNodes[currentIndex].backpointer].loc.x << ", " << treeNodes[treeNodes[currentIndex].backpointer].loc.y ;

				currentIndex = treeNodes[currentIndex].backpointer;
			}
			Tree_Path << treeNodes[currentIndex].loc.x << ", " << treeNodes[currentIndex].loc.y << endl ;

			Tree_Path.close();

			}


	}

	// output PRM into a txt file
	if (outPutTree){
		ofstream Tree("Tree.txt");

		for(int i = 1; i < numberOfNodes; i++){
			// cout  << treeNodes[i].loc.x << ", " << treeNodes[i].loc.y << ", " ;
			// cout << treeNodes[treeNodes[i].backpointer].loc.x << ", " << treeNodes[treeNodes[i].backpointer].loc.y ;
			// cout << endl;
			Tree << treeNodes[i].loc.x << ", " << treeNodes[i].loc.y << ", " ;
			Tree << treeNodes[treeNodes[i].backpointer].loc.x << ", " << treeNodes[treeNodes[i].backpointer].loc.y ;
			Tree << endl;
		}

			Tree.close();

	}
	// // resize all of the wights and adjacent vectors
	// for(int i = 0 ; i < numberOfNodes; i++){
	// 	treeNodes[i].adjacencts.resize(graphNodes[i].adjacencts.size());
	// 	treeNodes[i].weights.resize(graphNodes[i].weights.size());
	// }

	//
	// printGraph(&graphNodes);
	// int validSolution; // 1 if valid solution with a pth, 0 if no path can be generated
	// double pathLenght; // length of the path generated
	// double compTime; 		// computation time of PRM
	//



	//
		std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

		// cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << " [s]" << std::endl;
		bm.pathLength = treePathDist;

		bm.compTime = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();

		// cout << "Benchmark Info" << endl;
		// cout << "Path Found: " << bm.validSolution << endl;
		// cout << "Path Lenght: " << bm.pathLength << endl;
		// cout << "Computation Time: " << bm.compTime << " [micro sec]" << endl;
		//
		return bm;
}



////////////////////////////////////////////////////////////////////////////////
//  GoalBiasRRT_Centralized()
////////////////////////////////////////////////////////////////////////////////

PRM_Benchmark map::GoalBiasRRT_Centralized(int n,double r,double p,double epsilon, bool outputPath, bool outPutTree, int numOfRobs){

	// benchmark initialization
	PRM_Benchmark bm;

	// take time
	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();


	// random
	unsigned seedX = std::chrono::steady_clock::now().time_since_epoch().count();
	// cout << seedX << endl;
	srand (seedX);

	// xMapMin, xMapMax, yMapMin, yMapMax
	std::vector<std::vector<node>> robotsTree;
	robotsTree.resize(numOfRobs);

	std::vector<robot> robotsCopy = robots;

	// cout << "Number of Robots: " << numOfRobs << endl;


	double minDistanceTree = numOfRobs*sqrt((pow(xMapMax-xMapMin,2.0) +  pow(yMapMax-yMapMin,2.0)));

	// initialize the start location for all robots in their trees
	robotsCopy.resize(robots.size());

	for (int i = 0; i < numOfRobs; i++){
		node tempNode;

		tempNode.graph_number = 0;
		tempNode.loc = robots[i].start;
		robotsCopy[i].start_index = 0;
		robotsTree[i].push_back(tempNode);
	}

	// number of nodes per tree
	int numberOfNodes = 1;
	robotsTree.resize(numOfRobs, std::vector<node>(numberOfNodes));


	// vector to store the configuration of each robot
	std::vector<vertex> configurationVector;
	configurationVector.resize(numOfRobs);

	// number of robots at goal
	int atGoalCounter = 0;
	int atGoalIndexGlobal;
	// try to make n nodes..... //////////////////////////////////////////////////
	for(int i = 0; i < n ; i++){

		// cout << numberOfNodes << endl;
		// cout << "Check" << endl;
		bool noCollision = true; // no collisions
		int botsChecked = 0;	// number of bots checked

		std::vector<vertex> configurationVectorTemp;
		configurationVectorTemp.resize(numOfRobs);

		std::vector<node> minNodeVec;
		minNodeVec.resize(numOfRobs);


		// make random configuration ///////////////////////////////////////////////
		double randomNum = (double) rand()/RAND_MAX;
		if(randomNum <= p){
			for(int j = 0; j < numOfRobs; j++){
				vertex temp;
				// random location...need to add bias later
				if(robotsCopy[j].atGoal){
					temp = robotsCopy[j].goal;
				}
				else{
					temp.x = robotsCopy[j].goal.x; //(xMapMin-xMapMax) + xMapMin;
					temp.y = robotsCopy[j].goal.y; //(yMapMin-yMapMax) + yMapMin;
				}


				// node with min distance to q_rand
				configurationVectorTemp[j] = temp;

			}
		}
		else{
			for(int j = 0; j < numOfRobs; j++){
				vertex temp;
				// random location...need to add bias later
				if(robotsCopy[j].atGoal){
					temp = robotsCopy[j].goal;
				}
				else{
					temp.x = (double) rand()*(xMapMax-xMapMin)/RAND_MAX +xMapMin; //(xMapMin-xMapMax) + xMapMin;
					temp.y = (double) rand()*(yMapMax-yMapMin)/RAND_MAX +yMapMin; //(yMapMin-yMapMax) + yMapMin;
				}
				// node with min distance to q_rand
				configurationVectorTemp[j] = temp;

			}
		}


		// calculate min distance node
		double minDis = numOfRobs*sqrt((pow(xMapMax-xMapMin,2.0) +  pow(yMapMax-yMapMin,2.0)));
		int minIndex;

		for(int q = 0; q < numberOfNodes; q++ ){
			// cout << "Min dist 1" << endl;
			// cout << "Q " << q << endl;
			double tempMinDist = 0;
			for(int j = 0; j < numOfRobs; j++){
				// cout << "Min dist 2" << endl;
				tempMinDist = tempMinDist +  dist(configurationVectorTemp[j], robotsTree[j][q].loc);
				// cout << tempMinDist << endl;
				// cout << "Robot " << robotsTree[j][q].loc.x << ", " << robotsTree[j][q].loc.y << endl;
			}

			if(tempMinDist < minDis){
				minDis = tempMinDist;
				minIndex = q;
				// cout << "Min Distance = " << minDis << " at location " << minIndex << endl;
			}
		}

		// cout << "Min Index = " << minIndex << endl;
		// calculate the location of node r distance away for each robot
		// see if node can be epsilon away

		for(int j = 0 ; j < numOfRobs; j++){
			vertex newLoc;
			vertex temp = configurationVectorTemp[j];
			// cout << "temp: " << configurationVectorTemp[j].x <<  ", " <<configurationVectorTemp[j].y << endl;
			//
			// cout << "Length of trees = " << numberOfNodes <<" Vs Number minIndex = " << minIndex << endl;
			// cout << "minNode: " << robotsTree[j][minIndex].loc.x <<  ", " << robotsTree[j][minIndex].loc.y << endl;
			// cout << "Size of vector: " << robotsTree.size() << " x " << robotsTree[j].size() << endl;
			node minNode = robotsTree[j][minIndex];

			// cout << "MinNode " << minNode.loc.x << ", " << minNode.loc.y << endl;

			double theta = atan2(temp.y-minNode.loc.y,temp.x-minNode.loc.x );
			// position of new node
			newLoc.x = r*cos(theta)/sqrt(numOfRobs) + minNode.loc.x;
			newLoc.y = r*sin(theta)/sqrt(numOfRobs) + minNode.loc.y;


			configurationVectorTemp[j] = newLoc;
			// cout << "New Location: " << configurationVectorTemp[j].x <<", " << configurationVectorTemp[j].y << endl;
			// cout << "Check 1" << endl;
		}


		// see if configurationVectorTemp results in collision with obs or bots

		for (int j = 0 ; j < numOfRobs; j++){
			// cout << "Check 2" << endl;
			// obstacles
			// if(minDistanceToAllObs(configurationVectorTemp[j]) < robotsCopy[j].R){
			// 	noCollision = false;
			// 	// cout << "Obstacle Collision" << endl;
			// }


			// Make a polygon from the circular robots path
			polygon tempPoly;
			double theta = atan2(configurationVectorTemp[j].y-robotsTree[j][minIndex].loc.y,configurationVectorTemp[j].x-robotsTree[j][minIndex].loc.x );
			double distanceBetween = dist(configurationVectorTemp[j], robotsTree[j][minIndex].loc);


			vertex v1;
			v1.x = robotsTree[j][minIndex].loc.x + (robotsCopy[j].R*(-cos(theta) + sin(theta)));
			v1.y = robotsTree[j][minIndex].loc.y + (robotsCopy[j].R*(cos(theta) + sin(theta)));
			tempPoly.vertices.push_back(v1);

			vertex v2;
			v2.x = robotsTree[j][minIndex].loc.x + ((distanceBetween+robotsCopy[j].R)*cos(theta) + robotsCopy[j].R*sin(theta));
			v2.y = robotsTree[j][minIndex].loc.y + ((distanceBetween+robotsCopy[j].R)*sin(theta) - robotsCopy[j].R*cos(theta));
			tempPoly.vertices.push_back(v2);

			vertex v3;
			v3.x = robotsTree[j][minIndex].loc.x + ((distanceBetween+robotsCopy[j].R)*cos(theta) - robotsCopy[j].R*sin(theta));
			v3.y = robotsTree[j][minIndex].loc.y + ((distanceBetween+robotsCopy[j].R)*sin(theta) + robotsCopy[j].R*cos(theta));
			tempPoly.vertices.push_back(v3);

			vertex v4;
			v4.x = robotsTree[j][minIndex].loc.x + (robotsCopy[j].R*(-cos(theta) - sin(theta)));
			v4.y = robotsTree[j][minIndex].loc.y + (robotsCopy[j].R*(cos(theta) - sin(theta)));
			tempPoly.vertices.push_back(v4);

			// cout << "Angle Theta = " << 180*theta/PI << endl;
			// cout << "Distance =  " << distanceBetween << endl;
			// cout << "vertex Locations " << v1.x << ", " << v1.y << ", " << v2.x << ", " << v2.y << ", "<< v3.x << ", " << v3.y << ", " << v4.x << ", " << v4.y << endl;

			if(polygonObstacleCollision(tempPoly)){
				noCollision = false;
				// cout << "Path Collision" << endl;
			}



			// check robot-robot collision at this time step
			int numOfSteps = 10;

			for(int q = 0 ; q < numOfRobs; q++){

				if(q != j){
					double Max_Robot_Robot_Dist = robotsCopy[j].R + robotsCopy[q].R ;


					vertex loc1, loc2;

					for(int r = 0; r <= numOfSteps; r++ ){

						loc1.x = configurationVectorTemp[j].x + r*(configurationVectorTemp[j].x - robotsTree[j][minIndex].loc.x)/numOfSteps;
						loc1.y = configurationVectorTemp[j].y + r*(configurationVectorTemp[j].y - robotsTree[j][minIndex].loc.y)/numOfSteps;

						loc2.x = configurationVectorTemp[q].x + r*(configurationVectorTemp[q].x - robotsTree[q][minIndex].loc.x)/numOfSteps;
						loc2.y = configurationVectorTemp[q].y + r*(configurationVectorTemp[q].y - robotsTree[q][minIndex].loc.y)/numOfSteps;


						if(dist(loc1, loc2) <= Max_Robot_Robot_Dist){//robots[j].R+robots[q].R
							// cout << "Distance Between Bots" << dist(loc1, loc2) << endl;
							// cout << "Max Distance between bots " << Max_Robot_Robot_Dist << endl;
							noCollision = false;
							// cout << "Robot Collision" << endl;
						}




					}

				}
			}

			// check for temproral robot collision; each node will have a time step
			// based on its parents time step
			// Actually can check in real time as long as you update time stamp of
			// node in the above steps, you can check in the robot collsision function if




		}



		// add the configuration to the robotsTree vectors
		if(noCollision){
			// add all of the node info for each bot
			// cout << "No Collision" << endl;
			atGoalCounter = 0;
			for (int j = 0; j < numOfRobs; j++){


				node tempNode;
				// tempNode.graph_number = numberOfNodes;
				tempNode.loc = configurationVectorTemp[j];
				// tempNode.backpointer = minIndex;

				// ------------------------------------------------------

				if(robotsCopy[j].atGoal){
					// cout << "Robot" << j << " found goal" << endl;
					// tempNode.loc = robotsCopy[j].goal;
					// tempNode.graph_number = numberOfNodes;
					// tempNode.backpointer = minIndex;
					// robotsCopy[j].goal_index = tempNode.graph_number;
					atGoalCounter++;
				}
				else if( dist(tempNode.loc, robotsCopy[j].goal)<epsilon){
					tempNode.graph_number = numberOfNodes;
					tempNode.loc = robotsCopy[j].goal;
					tempNode.backpointer = minIndex;
					robotsCopy[j].goal_index = tempNode.graph_number;
					robotsCopy[j].atGoal = true;
				}
				else{
					tempNode.graph_number = numberOfNodes;
					tempNode.loc = configurationVectorTemp[j];
					tempNode.backpointer = minIndex;
				}

				// -------------------------------------------------------


				// if(dist(tempNode.loc, robotsCopy[j].goal)<epsilon && !robotsCopy[j].atGoal){
				// 	robotsCopy[j].goal_index = tempNode.graph_number;
				// 	robotsCopy[j].atGoal = true;
				// 	tempNode.loc = robotsCopy[j].goal;
				// }
				//
				// if(robotsCopy[j].atGoal){
				// 	atGoalCounter++;
				// }

				robotsTree[j].push_back(tempNode);
			}
			// cout << "Old Size of vector: " << robotsTree.size() << " x " << robotsTree[1].size() << endl;
			numberOfNodes++;
			robotsTree.resize(numOfRobs, vector<node>(numberOfNodes));

			if(atGoalCounter == numOfRobs){
				// cout << "FOUND GOAL" << endl;

				i = 2*n;
			} // terminate loop
			// cout << "New Size of vector: " << robotsTree.size() << " x " << robotsTree[0].size() << endl;
		}






	}


	// cout << "Number of Nodes: " << numberOfNodes << endl;
	// cout << "Number of Robots at goal: " << atGoalCounter << endl;

	if(outPutTree){
		// cout << "Make Tree" << endl;
		ofstream trees("Trees.txt");
		for(int j = 0; j < numOfRobs; j++){

			for(int i = 1; i<numberOfNodes; i++){
				trees << robotsTree[j][i].loc.x << ", " << robotsTree[j][i].loc.y << ", ";
				trees << robotsTree[j][robotsTree[j][i].backpointer].loc.x << ", " << robotsTree[j][robotsTree[j][i].backpointer].loc.y << ", ";
			}

			trees << endl;
		}

		trees.close();
	}

	if(atGoalCounter == numOfRobs){
		if(outputPath){
			// cout << "Make Path" << endl;
			ofstream treesPath("Trees_Path.txt");
			for(int j = 0; j < numOfRobs; j++){
				node currentNode = robotsTree[j][numberOfNodes-1] ; //robotsTree[j][robotsCopy[j].goal_index];
				while(currentNode.graph_number != robotsCopy[j].start_index){
					treesPath << currentNode.loc.x << ", " << currentNode.loc.y << ", ";
					currentNode = robotsTree[j][currentNode.backpointer];
				}
				treesPath << currentNode.loc.x << ", " << currentNode.loc.y << ", " << endl;

			}

			treesPath.close();
		}
	}

	// experimental

	//
		std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

		// cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << " [s]" << std::endl;
		// bm.pathLength = treePathDist;

		bm.compTime = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
		bm.pathLength = 0.0;
		bm.validSolution = numberOfNodes;

		// cout << "Benchmark Info" << endl;
		// cout << "Size of Tree: " << bm.validSolution << endl;
		// // cout << "Path Lenght: " << bm.pathLength << endl;
		// cout << "Computation Time: " << bm.compTime << " [micro sec]" << endl;
		//
		return bm;
}

////////////////////////////////////////////////////////////////////////////////
//  GoalBiasRRT_Decentralized()
////////////////////////////////////////////////////////////////////////////////

PRM_Benchmark map::GoalBiasRRT_Decentralized(int n,double r,double p,double epsilon, bool outputPath, bool outPutTree, int numOfRobs){

	// benchmark initialization
	PRM_Benchmark bm;

	// take time
	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();


	// random
	unsigned seedX = std::chrono::steady_clock::now().time_since_epoch().count();
	// cout << seedX << endl;
	srand (seedX);

	// xMapMin, xMapMax, yMapMin, yMapMax
	std::vector<std::vector<node>> robotsTreePaths;
	robotsTreePaths.resize(numOfRobs);

	std::vector<robot> robotsCopy = robots;

	// cout << "Number of Robots: " << numOfRobs << endl;


	double minDistanceTree = numOfRobs*sqrt((pow(xMapMax-xMapMin,2.0) +  pow(yMapMax-yMapMin,2.0)));

	// initialize the start location for all robots in their trees
	robotsCopy.resize(robots.size());

	int totalBotsAtGoal =0;



	for(int currentBot = 0; currentBot < numOfRobs; currentBot++){
		// try to make n nodes..... //////////////////////////////////////////////////
		std::vector<node> robotsTreeTemp;

		node tempNode;
		tempNode.graph_number = 0;
		tempNode.loc = robotsCopy[currentBot].start;
		tempNode.timeStamp = 0;
		robotsCopy[currentBot].start_index = 0;
		robotsTreeTemp.push_back(tempNode);
		int numberOfNodes = 1;
		robotsTreeTemp.resize(numberOfNodes);

		// while(!robotsCopy[currentBot].atGoal){
		for(int i = 0; i < n ; i++){

			bool noCollision = true; // no collisions
			int botsChecked = 0;	// number of bots checked

			vertex configurationVectorTemp;

			std::vector<node> minNodeVec;
			minNodeVec.resize(numOfRobs);


			// make random configuration ///////////////////////////////////////////////
			double randomNum = (double) rand()/RAND_MAX;
			if(randomNum <= p){
				vertex temp;

				temp.x = robotsCopy[currentBot].goal.x; //(xMapMin-xMapMax) + xMapMin;
				temp.y = robotsCopy[currentBot].goal.y; //(yMapMin-yMapMax) + yMapMin;
					// node with min distance to q_rand
				configurationVectorTemp = temp;
			}
			else{
				vertex temp;

				temp.x = (double) rand()*(xMapMax-xMapMin)/RAND_MAX +xMapMin; //(xMapMin-xMapMax) + xMapMin;
				temp.y = (double) rand()*(yMapMax-yMapMin)/RAND_MAX +yMapMin; //(yMapMin-yMapMax) + yMapMin;
				configurationVectorTemp = temp;

			}


			// calculate min distance node
			double minDis = sqrt((pow(xMapMax-xMapMin,2.0) +  pow(yMapMax-yMapMin,2.0)));
			int minIndex;
			double tempMinDist;

			for(int q = 0; q < numberOfNodes; q++ ){

				tempMinDist = dist(configurationVectorTemp, robotsTreeTemp[q].loc);
					// cout << tempMinDist << endl;
					// cout << "Robot " << robotsTreeTemp[q].loc.x << ", " << robotsTreeTemp[q].loc.y << endl;

				if(tempMinDist < minDis){
					minDis = tempMinDist;
					minIndex = q;
					// cout << "Min Distance = " << minDis << " at location " << minIndex << endl;
				}
			}

			// cout << "Min Index = " << minIndex << endl;
			// calculate the location of node r distance away for each robot
			// see if node can be epsilon away

			// finalize configuration
			node minNode = robotsTreeTemp[minIndex];
			double theta = atan2(configurationVectorTemp.y-minNode.loc.y,configurationVectorTemp.x-minNode.loc.x );
			// position of new node
			configurationVectorTemp.x = r*cos(theta) + minNode.loc.x;
			configurationVectorTemp.y = r*sin(theta) + minNode.loc.y;

			// make new candidate node
			node tempCandidate;
			tempCandidate.loc = configurationVectorTemp;
			tempCandidate.backpointer = minIndex;
			tempCandidate.timeStamp = robotsTreeTemp[minIndex].timeStamp+1;



			// see if configurationVectorTemp results in collision with obs or bot

			// Make a polygon from the circular robots path
			polygon tempPoly;
			// double theta = atan2(configurationVectorTemp.y-robotsTree[minIndex].loc.y,configurationVectorTemp.x-robotsTree[minIndex].loc.x );
			double distanceBetween = dist(configurationVectorTemp, robotsTreeTemp[minIndex].loc);


			vertex v1;
			v1.x = robotsTreeTemp[minIndex].loc.x + (robotsCopy[currentBot].R*(-cos(theta) + sin(theta)));
			v1.y = robotsTreeTemp[minIndex].loc.y + (robotsCopy[currentBot].R*(cos(theta) + sin(theta)));
			tempPoly.vertices.push_back(v1);

			vertex v2;
			v2.x = robotsTreeTemp[minIndex].loc.x + ((distanceBetween+robotsCopy[currentBot].R)*cos(theta) + robotsCopy[currentBot].R*sin(theta));
			v2.y = robotsTreeTemp[minIndex].loc.y + ((distanceBetween+robotsCopy[currentBot].R)*sin(theta) - robotsCopy[currentBot].R*cos(theta));
			tempPoly.vertices.push_back(v2);

			vertex v3;
			v3.x = robotsTreeTemp[minIndex].loc.x + ((distanceBetween+robotsCopy[currentBot].R)*cos(theta) - robotsCopy[currentBot].R*sin(theta));
			v3.y = robotsTreeTemp[minIndex].loc.y + ((distanceBetween+robotsCopy[currentBot].R)*sin(theta) + robotsCopy[currentBot].R*cos(theta));
			tempPoly.vertices.push_back(v3);

			vertex v4;
			v4.x = robotsTreeTemp[minIndex].loc.x + (robotsCopy[currentBot].R*(-cos(theta) - sin(theta)));
			v4.y = robotsTreeTemp[minIndex].loc.y + (robotsCopy[currentBot].R*(cos(theta) - sin(theta)));
			tempPoly.vertices.push_back(v4);

			// cout << "Angle Theta = " << 180*theta/PI << endl;
			// cout << "Distance =  " << distanceBetween << endl;
			// cout << "vertex Locations " << v1.x << ", " << v1.y << ", " << v2.x << ", " << v2.y << ", "<< v3.x << ", " << v3.y << ", " << v4.x << ", " << v4.y << endl;

			if(polygonObstacleCollision(tempPoly)){
				noCollision = false;
				// cout << "Path Collision" << endl;
			}


			for(int j = 0; j<currentBot; j++){

					if(j!=currentBot){
						vertex loc1,loc2;
						int numOfSteps = 10;
						double Max_Robot_Robot_Dist = robots[j].R+robots[currentBot].R;


					for(int r = 0; r <= numOfSteps; r++ ){

						loc1.x = robotsTreePaths[j][tempCandidate.timeStamp].loc.x + r*(robotsTreePaths[j][tempCandidate.timeStamp].loc.x - robotsTreePaths[j][tempCandidate.timeStamp-1].loc.x)/numOfSteps;
						loc1.y = robotsTreePaths[j][tempCandidate.timeStamp].loc.y + r*(robotsTreePaths[j][tempCandidate.timeStamp].loc.y - robotsTreePaths[j][tempCandidate.timeStamp-1].loc.y)/numOfSteps;

						loc2.x = configurationVectorTemp.x + r*(configurationVectorTemp.x - robotsTreeTemp[minIndex].loc.x)/numOfSteps;
						loc2.y = configurationVectorTemp.y + r*(configurationVectorTemp.y - robotsTreeTemp[minIndex].loc.y)/numOfSteps;


						if(dist(loc1, loc2) <= Max_Robot_Robot_Dist){//robots[j].R+robots[q].R
							// cout << "Distance Between Bots" << dist(loc1, loc2) << endl;
							// cout << "Max Distance between bots " << Max_Robot_Robot_Dist << endl;
							noCollision = false;
							// cout << "Robot Collision" << endl;
							}

						}

					}

				}

				for(int j = 0; j < numOfRobs; j++){
					if(j !=currentBot){
						if(dist(configurationVectorTemp, robotsCopy[j].goal) <= robotsCopy[j].R+robotsCopy[currentBot].R){
							noCollision = false;
							// cout << "Collision at goal" << endl;
						}
					}
				}


				if(noCollision){
					tempCandidate.graph_number = numberOfNodes;

					if(dist(tempCandidate.loc, robotsCopy[currentBot].goal) <= epsilon){
						tempCandidate.loc = robotsCopy[currentBot].goal;
						robotsCopy[currentBot].goal_index = numberOfNodes;
						robotsCopy[currentBot].atGoal  = true;
						totalBotsAtGoal++;
						i = 2*n;
						// break;
					}

					robotsTreeTemp.push_back(tempCandidate);
					numberOfNodes++;
					robotsTreeTemp.resize(numberOfNodes);

				}




		}


		if(robotsCopy[currentBot].atGoal){
			// make the path for the current robot in robotsTreePaths
			// cout << "----------- Found a Path for robot " << currentBot << " -----------" << endl;
			// cout << endl;
			// cout << "             Reverse Path           " << endl;
			node bing = robotsTreeTemp[robotsCopy[currentBot].goal_index];
			while(bing.timeStamp != 0){
				robotsTreePaths[currentBot].push_back(bing);

				// cout << bing.timeStamp << " : " << bing.loc.x << ", " << bing.loc.y << endl;
				bing = robotsTreeTemp[bing.backpointer];


			}
			// cout << bing.timeStamp << " : " << bing.loc.x << ", " << bing.loc.y << endl;
			robotsTreePaths[currentBot].push_back(bing);

			std::reverse(robotsTreePaths[currentBot].begin(),robotsTreePaths[currentBot].end());
			robotsTreePaths[currentBot].resize(robotsTreePaths[currentBot].size());

			// // debugging
			// cout << endl;
			// cout << "             Forward Path           " << endl;
			// int it = 0;
			// while(it != robotsTreePaths[currentBot].size()){
			// 	cout << robotsTreePaths[currentBot][it].timeStamp << " : " << robotsTreePaths[currentBot][it].loc.x << ", " << robotsTreePaths[currentBot][it].loc.y << endl;
			// 	it++;
			// }

		}
		else{
			currentBot = 2*numOfRobs;

		}
		cout << "Current Bot " << currentBot << endl;
	}


	// cout << "Number of Nodes: " << numberOfNodes << endl;
	// cout << "Number of Robots at goal: " << atGoalCounter << endl;

	// if(outPutTree){
	// 	// cout << "Make Tree" << endl;
	// 	ofstream trees("Trees.txt");
	// 	for(int j = 0; j < numOfRobs; j++){
	//
	// 		for(int i = 1; i<numberOfNodes; i++){
	// 			trees << robotsTree[j][i].loc.x << ", " << robotsTree[j][i].loc.y << ", ";
	// 			trees << robotsTree[j][robotsTree[j][i].backpointer].loc.x << ", " << robotsTree[j][robotsTree[j][i].backpointer].loc.y << ", ";
	// 		}
	//
	// 		trees << endl;
	// 	}
	//
	// 	trees.close();
	// }

	if(totalBotsAtGoal == numOfRobs){
		if(outputPath){
			// cout << "Make Path" << endl;
			ofstream treesPath("Trees_Path_Decentralized.txt");
			for(int j = 0; j < numOfRobs; j++){
				int it = 0;
				while(it != robotsTreePaths[j].size()){
				 	treesPath << robotsTreePaths[j][it].loc.x << ", " << robotsTreePaths[j][it].loc.y << ", " ;
				 	it++;
			 }
			 treesPath << endl;
			}

			treesPath.close();
		}
	}
	// }
	//
	// // experimental

	//
		std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

		// cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << " [s]" << std::endl;
		// bm.pathLength = treePathDist;

		bm.compTime = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
		// bm.pathLength = 0.0;
		// bm.validSolution = numberOfNodes;

		// cout << "Benchmark Info" << endl;
		// cout << "Size of Tree: " << bm.validSolution << endl;
		// // cout << "Path Lenght: " << bm.pathLength << endl;
		// cout << "Computation Time: " << bm.compTime << " [micro sec]" << endl;
		//
		return bm;
}

////////////////////////////////////////////////////////////////////////////////
//  addRobots()
////////////////////////////////////////////////////////////////////////////////
void map::addRobots(std::vector<robot> robotsToAdd){
	robots = robotsToAdd;
}

////////////////////////////////////////////////////////////////////////////////
//  addRobots()
////////////////////////////////////////////////////////////////////////////////
double map::minDistanceToAllObs(vertex current){

	double minDist = sqrt((pow(xMapMax-xMapMin,2.0) +  pow(yMapMax-yMapMin,2.0)));

		for(const auto& poly_it:obstacles){
			vertex now = minDistanceToObs(poly_it, current);
			double dist = sqrt((pow(now.x-current.x,2.0) +  pow(now.y-current.y,2.0)));

			if(dist<minDist){
				minDist = dist;
			}
		}

	return minDist;
}

////////////////////////////////////////////////////////////////////////////////
//  printGraph()
////////////////////////////////////////////////////////////////////////////////
void map::printGraph(std::vector<node> *graphNodes){

	for(const auto& graph_it: *graphNodes){
		cout << "------ Node #" << graph_it.graph_number << " ------" << endl;
		cout << "Node Location:  " << graph_it.loc.x << ", " << graph_it.loc.y << endl;
		cout << "Node Heuristic: " << graph_it.h  << endl;
		cout << "Number of neigbors: " << graph_it.adjacencts.size() << endl;

		cout << "Node Neighbors: ";
		for(int i = 0; i < graph_it.adjacencts.size(); i++){
			cout << graph_it.adjacencts[i] << ", ";
		}
		cout << endl;

		cout << "Node Neighbors Weights: ";
		for(int i = 0; i < graph_it.weights.size(); i++){
			cout << graph_it.weights[i] << ", ";
		}
		cout << endl;


	}
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
