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


// Map class
class map {
	double xMapMin, xMapMax, yMapMin, yMapMax, sx, sy,gx,gy;

	public:
	std::vector<polygon> obstacles;
	std::vector<polygon> c_space_obs;
	map( std::tuple<double,double>, std::tuple<double,double>, std::tuple<double,double>, std::tuple<double,double>, std::vector<polygon>);
	bool pointCollision(vertex); // find out if a point collides with any of the obstacles
	bool polygonCollision(polygon, polygon); // find out if a polygon collides with another polygon
	polygon rotate_polygon(polygon, vertex, double); // rotate a polygon
	polygon invert_polygon(polygon); // invert the points of a polygon
	polygon orderVertices(polygon p); // order the vertices of a polygon

	polygon makeCSpaceObstacle(polygon, polygon, double);
	void printObs();
	void printPolygon(polygon);
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

	// Debugging
	// cout << "xLim = [" << xMapMin << "," << xMapMax << "]" << endl;
	// cout << "yLim = [" << yMapMin << "," << yMapMax << "]" << endl;
	// cout << "start = (" << sx << "," << sy << ")" << endl;
	// cout << "goal = (" << gx << "," << gy << ")" << endl;
}

////////////////////////////////////////////////////////////////////////////////
// pointCollision
////////////////////////////////////////////////////////////////////////////////
// Test if a point collides with a convex polygon
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
					double zvalue = v_result(2);

					// cout << "zvalue: \n" << zvalue << endl;

					// cout << "Zval: " << zvalue << endl;
					// cout << "v_vertex: " << (first.x - current.x) << "," << (first.y - current.y) << "," << 1 << endl;
					// cout << "v_point: " << (point.x - current.x) << "," << (point.y - current.y) << "," << 1 << endl;

					// collition with a vertex
					if((point.x - current.x)== 0 && (point.y - current.y)==0){
						cout << "Collision with polygon " << polyCnt << " has occured" << endl;
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
						cout << "Collision with polygon " << polyCnt << " has occured" << endl;
						return true;
					}

					edgeCnt = edgeCnt +1;
					if (zvalue >0){
						zCnt = zCnt +1;
					}

			}

		}

		if(zCnt == edgeCnt){
			cout << "Collision with polygon " << polyCnt << " has occured" << endl;
			return true;
		}

		polyCnt++;
		zCnt = 0;
		edgeCnt = 0;
	}

	cout << "No point collision has occured" << endl;
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

	cout << "------------------ Check Robot Edges ------------------" << endl;
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
		cout << "Robot Min " << robotProjectedMin << endl;
		cout << "Robot Max " << robotProjectedMax << endl;
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

		cout << "Obstacle Min " << obsProjectedMin << endl;
		cout << "Obstacle Max " << obsProjectedMax << endl;

		if(robotProjectedMin >obsProjectedMax || robotProjectedMax < obsProjectedMin){
			cout << "No collsion with a robot" << endl;
			return false;
		}

	}

	cout << "------------------ Check Obstacle Edges ------------------" << endl;
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
		cout << "Robot Min " << robotProjectedMin << endl;
		cout << "Robot Max " << robotProjectedMax << endl;
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

		cout << "Obstacle Min " << obsProjectedMin << endl;
		cout << "Obstacle Max " << obsProjectedMax << endl;

		if(robotProjectedMin >obsProjectedMax || robotProjectedMax < obsProjectedMin){
			cout << "No collsion with a robot" << endl;
			return false;
		}

	}


	cout << "COLLISION OCCURED" <<endl;
	return true;
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
