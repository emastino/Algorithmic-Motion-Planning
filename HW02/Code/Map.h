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


using namespace std;





class Map {
	double width, height; // width and height of Map object
	double xs,ys,xg,yg;

	public:

    // limits of the map
    std::tuple<double, double> xlim;
    std::tuple<double, double> ylim;
    double lowX,lowY, highX, highY;

    // start and goal locations
    std::tuple<double, double> start;
    std::tuple<double, double> goal;

    // obstacles
    int numObs;
    std::tuple<double, double, double, double, double, double, double, double> *obstacles;

    // constructor
		Map(std::tuple<double, double>,tuple<double, double>,std::tuple<double, double>, tuple<double, double>, std::tuple<double, double, double, double, double, double, double, double>*, int); // constructor

		void Bug1();
		void navigate_bug_1(double*, double*);

		void Bug2();
		void navigate_bug_2(double*, double*);


		bool checkCollison(double, double);
		char getDir(std::tuple<bool,bool,bool,bool,bool,bool,bool,bool>);
		double distance(double, double, double, double);
		void saveRoute();
		std::list<std::tuple<double,double>> route_1;
		std::list<std::tuple<double,double>> route_2;

};
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

Map::Map(std::tuple<double, double> xl, std::tuple<double, double> yl, std::tuple<double, double> start, std::tuple<double, double> goal, std::tuple<double, double, double, double, double, double, double, double> *obstaclesPassed, int numberOfObs){

	cout << "Start Constructor" << endl;
	// define x limits
	std::tie(lowX, highX) = xl;
	width = highX-lowX; // acts as columns
	// define y limits
  std::tie(lowY, highY) = yl;
  height = highY-lowY;  // acts as number of rows

	// start
	std::tie(xs, ys) = start;
	route_1.push_back(start);
	route_2.push_back(start);
	// goal
	std::tie(xg, yg) = goal;

	// cout << "start " << xs << "," << ys << endl;
	// cout << "goal " << double(xg) << "," << yg << endl;
	// write map dimensions, start, and goal toa files
	ofstream mapfile;
	mapfile.open ("mapDim.txt");
	mapfile << lowX << "," << highX <<endl;
	mapfile << lowY << "," << highY <<endl;
	mapfile << xs << "," << ys <<endl;
	mapfile << xg << "," << yg <<endl;
	mapfile.close();

	// // // // // // // // // // // // // // //
	// Obstacles
  numObs = numberOfObs;
	obstacles = obstaclesPassed;
	// populate obstacles and write obstacles to a file
	ofstream obfile ;
	obfile.open ("obstacleFile.txt");

	double x1,y1,x2,y2,x3,y3,x4,y4;

	for (int i =0 ; i<numObs; i++){

		std::tie(x1,y1,x2,y2,x3,y3,x4,y4) = obstacles[i];

		obfile << x1 << "," << y1 << "," << x2 << "," << y2 << "," << x3 << "," <<y3 << "," << x4 << "," <<y4 << endl;
	}
	obfile.close();


}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void Map::Bug1(){

	// at goal flag
	bool atGoal = false;


	double currentx = xs;
	double currenty = ys;
	double lambda = 0.0001;

	// cout << "start " << xs << "," << ys << endl;
	// cout << "goal " << xg << "," << yg << endl;



	while(!atGoal){

		// move towards Goal
		currentx = (1.0-lambda)*currentx + lambda*xg;
		currenty = (1.0-lambda)*currenty + lambda*yg;


		// cout << currentx << "," << currenty << endl;


		////////////////////////////////////////////////////////////////////////////
		////////////////////////////////////////////////////////////////////////////
		// if we encounter an obstacle: move back and begin following walls
		if(checkCollison(currentx, currenty)){
			cout << "Upon Collision" << endl;
			cout << currentx << "," << currenty << endl;
			// path
			// move towards Goal
			while( checkCollison(currentx, currenty)){
				lambda = lambda + 0.001;
				currentx = (1.0-lambda)*currentx + lambda*xs;
				currenty = (1.0-lambda)*currenty + lambda*ys;
			}

			cout << "After going back" << endl;
			cout << currentx << "," << currenty << endl;
			//define cardinal directions
			route_1.push_back(make_tuple(currentx,currenty));

			navigate_bug_1(&currentx, &currenty);

			cout << "Left navigate_bug_1" << endl;
			lambda = 0;
		}

		////////////////////////////////////////////////////////////////////////////



		// routefile << currentx << "," << currenty << endl;
		// cout << currentx << "," << currenty << endl;
		route_1.push_back(make_tuple(currentx,currenty));


		if (distance(currentx, currenty, xg,yg) <= 0.01){
			cout << "Basically at Goal" << endl;
			atGoal=true;
		}


		lambda = lambda + 0.001;
	}




}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// navugate obstacle for bug 1

void Map:: navigate_bug_1(double * cx, double * cy){

	// hit point
	double hx = *cx;
	double hy = *cy;
	bool hflag = false;

	double range = 0.2;
	//north
	double n_x = *cx ;
	double n_y = *cy + range;
	bool nflag = checkCollison(n_x,n_y);

	// east
	double e_x = *cx + range;
	double e_y = *cy;
	bool eflag = checkCollison(e_x,e_y);

	// south
	double s_x = *cx;
	double s_y = *cy - range;
	bool sflag = checkCollison(s_x,s_y);

	// west
	double w_x = *cx - range;
	double w_y = *cy;
	bool wflag = checkCollison(w_x,w_y);

	// north east
	double ne_x = *cx + range;
	double ne_y = *cy + range;
	bool neflag = checkCollison(ne_x,ne_y);

	// south east
	double se_x = *cx + range;
	double se_y = *cy - range;
	bool seflag = checkCollison(se_x,se_y);

	// south west
	double sw_x = *cx - range;
	double sw_y = *cy - range;
	bool swflag = checkCollison(sw_x,sw_y);

	// north west
	double nw_x = *cx - range;
	double nw_y = *cy + range;
	bool nwflag = checkCollison(nw_x,nw_y);

	// direction
	std::tuple<bool,bool,bool,bool,bool,bool,bool,bool> dirVector = std::make_tuple(nflag,eflag,sflag, wflag, neflag,seflag,swflag,nwflag);
	char dir;

	// loop back stuff
	int steps =1; // the hit point is 1st
	std::list<std::tuple<double,double>> locationsAroundObs;
	locationsAroundObs.push_back(std::make_tuple(*cx,*cy));
	int minIndex = 1; // location with minimum distance to goal
	double minValue = distance (*cx, *cy, xg,yg); // actual distance


	//

	/////////////////////////////////////////////////////////////////////////////
	/////////// START NAVIGATING ///////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////


	// initialization
	dir = getDir(dirVector);

	if(dir == 'n'){
		*cx = *cx;
		*cy = *cy + range/2.0;
	}

	if(dir == 'e'){
		*cx = *cx + range/2.0;
		*cy = *cy;
	}

	if(dir == 's'){
		*cx = *cx;
		*cy = *cy -range/2.0;
	}

	if(dir == 'w'){
		*cx = *cx - range/2.0;
		*cy = *cy;
	}

	steps++;
	locationsAroundObs.push_back(std::make_tuple(*cx,*cy));
	// check minimum
	if(minValue > distance(*cx, *cy, xg,yg)){
		minIndex = steps;
		minValue = distance(*cx, *cy, xg,yg);
	}

	route_1.push_back(std::make_tuple(*cx,*cy));

	cout << "hx: " << hx <<  endl;
	cout << "hy: " << hy <<  endl;


	bool follow_flag = true;

	// wall follow
	while(follow_flag){

		//north
		n_x = *cx ;
		n_y = *cy + range;
		nflag = checkCollison(n_x,n_y);

		// east
		e_x = *cx + range;
		e_y = *cy;
		eflag = checkCollison(e_x,e_y);

		// south
		s_x = *cx;
		s_y = *cy - range;
		sflag = checkCollison(s_x,s_y);

		// west
		w_x = *cx - range;
		w_y = *cy;
		wflag = checkCollison(w_x,w_y);

		// north east
		ne_x = *cx + range;
		ne_y = *cy + range;
		neflag = checkCollison(ne_x,ne_y);

		// south east
		se_x = *cx + range;
		se_y = *cy - range;
		seflag = checkCollison(se_x,se_y);

		// south west
		sw_x = *cx - range;
		sw_y = *cy - range;
		swflag = checkCollison(sw_x,sw_y);

		// north west
		nw_x = *cx - range;
		nw_y = *cy + range;
		nwflag = checkCollison(nw_x,nw_y);

		dirVector = std::make_tuple(nflag,eflag,sflag, wflag, neflag,seflag,swflag,nwflag);

		dir = getDir(dirVector);
		// cout << "Dir = " << dir << endl;

		// figure out where to go next if we arent where we began
		if(dir == 'n'){
			*cx = *cx;
			*cy = *cy +range/2.0;
		}

		if(dir == 'e'){
			*cx = *cx + range/2.0;
			*cy = *cy;
		}

		if(dir == 's'){
			*cx = *cx;
			*cy = *cy - range/2.0;
		}

		if(dir == 'w'){
			*cx = *cx - range/2.0;
			*cy = *cy;
		}

		steps++;
		locationsAroundObs.push_back(std::make_tuple(*cx,*cy));

		// check minimum
		if(minValue > distance(*cx, *cy, xg,yg)){
			minIndex = steps;
			minValue = distance(*cx, *cy, xg, yg);
		}

		route_1.push_back(std::make_tuple(*cx,*cy));
		//
		// cout << "cx,hx  " << *cx << ", " << hx << endl;
		// cout << "cy,hy  " << *cy << ", " << hy << endl;

		// check if we are back at the hit point
		// since we moved up by range amount from the hit point
		if ( *cx >= (hx -range/1.5) &&  *cx <= (hx +range/1.5)){

			if ( *cy >= (hy -range/1.5) &&  *cy <= (hy +range/1.5)){

				cout << "Follow Flag False" << endl;
				follow_flag = false;

			}

		}

		// cout << "steps: " << steps << endl;
		if(steps >= 5000 ){
			break;
		}
	}

	cout << "cx,hx  " << *cx << ", " << hx << endl;
	cout << "cy,hy  " << *cy << ", " << hy << endl;

	cout << "MIN DISTANCE " << minValue << endl;
	cout << "MIN INDEX " << minIndex << endl;

	// we move back
	if((steps -minIndex) >= (minIndex-1)){
		cout << "Going Backwards" << endl;
		for(int i = steps; i > minIndex; i--){
			cout << "Going Backwards" << endl;
			std::tie(*cx, *cy) = locationsAroundObs.back();
			route_1.push_back(std::make_tuple(*cx,*cy));
			locationsAroundObs.pop_back();


		}
	}
	// move forward
	else{
		cout << "Going Forwards" << endl;
		for(int i = 1; i < minIndex; i++){
			cout << "Going forwards" << endl;
			std::tie(*cx, *cy) = locationsAroundObs.front();
			route_1.push_back(std::make_tuple(*cx,*cy));
			locationsAroundObs.pop_front();

		}


	}

	cout << "steps: " << steps << endl;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void Map::Bug2(){

	// at goal flag
	bool atGoal = false;


	double currentx = xs;
	double currenty = ys;
	double lambda = 0.0001;

	// cout << "start " << xs << "," << ys << endl;
	// cout << "goal " << xg << "," << yg << endl;



	while(!atGoal){

		// move towards Goal
		currentx = (1.0-lambda)*currentx + lambda*xg;
		currenty = (1.0-lambda)*currenty + lambda*yg;


		// cout << currentx << "," << currenty << endl;


		////////////////////////////////////////////////////////////////////////////
		////////////////////////////////////////////////////////////////////////////
		// if we encounter an obstacle: move back and begin following walls
		if(checkCollison(currentx, currenty)){
			cout << "Upon Collision" << endl;
			cout << currentx << "," << currenty << endl;
			// path
			// move towards Goal
			while( checkCollison(currentx, currenty)){
				lambda = lambda + 0.001;
				currentx = (1.0-lambda)*currentx + lambda*xs;
				currenty = (1.0-lambda)*currenty + lambda*ys;
			}

			cout << "After going back" << endl;
			cout << currentx << "," << currenty << endl;
			//define cardinal directions
			route_2.push_back(make_tuple(currentx,currenty));

			navigate_bug_2(&currentx, &currenty);

			cout << "Left navigate_bug_2" << endl;
			lambda = 0;
		}

		////////////////////////////////////////////////////////////////////////////



		// routefile << currentx << "," << currenty << endl;
		// cout << currentx << "," << currenty << endl;
		route_2.push_back(make_tuple(currentx,currenty));


		if (distance(currentx, currenty, xg,yg) <= 0.01){
			cout << "Basically at Goal" << endl;
			atGoal=true;
		}


		lambda = lambda + 0.0001;
	}




}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// navugate obstacle for bug 2

void Map:: navigate_bug_2(double * cx, double * cy){

	double m = (yg - ys)/(xg-xs);
	double bm = ys -m*xs;
	double xm = *cx;
	double ym = *cy;

	double range = 0.3;
	//north
	double n_x = *cx ;
	double n_y = *cy + range;
	bool nflag = checkCollison(n_x,n_y);

	// east
	double e_x = *cx + range;
	double e_y = *cy;
	bool eflag = checkCollison(e_x,e_y);

	// south
	double s_x = *cx;
	double s_y = *cy - range;
	bool sflag = checkCollison(s_x,s_y);

	// west
	double w_x = *cx - range;
	double w_y = *cy;
	bool wflag = checkCollison(w_x,w_y);

	// north east
	double ne_x = *cx + range;
	double ne_y = *cy + range;
	bool neflag = checkCollison(ne_x,ne_y);

	// south east
	double se_x = *cx + range;
	double se_y = *cy - range;
	bool seflag = checkCollison(se_x,se_y);

	// south west
	double sw_x = *cx - range;
	double sw_y = *cy - range;
	bool swflag = checkCollison(sw_x,sw_y);

	// north west
	double nw_x = *cx - range;
	double nw_y = *cy + range;
	bool nwflag = checkCollison(nw_x,nw_y);

	// direction
	std::tuple<bool,bool,bool,bool,bool,bool,bool,bool> dirVector = std::make_tuple(nflag,eflag,sflag, wflag, neflag,seflag,swflag,nwflag);
	char dir;


	//

	/////////////////////////////////////////////////////////////////////////////
	/////////// START NAVIGATING ///////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////


	// initialization
	dir = getDir(dirVector);

	if(dir == 'n'){
		*cx = *cx;
		*cy = *cy + range/2.0;
	}

	if(dir == 'e'){
		*cx = *cx + range/2.0;
		*cy = *cy;
	}

	if(dir == 's'){
		*cx = *cx;
		*cy = *cy -range/2.0;
	}

	if(dir == 'w'){
		*cx = *cx - range/2.0;
		*cy = *cy;
	}



	route_2.push_back(std::make_tuple(*cx,*cy));



	bool follow_flag = true;

	// wall follow
	while(follow_flag){

		//north
		n_x = *cx ;
		n_y = *cy + range;
		nflag = checkCollison(n_x,n_y);

		// east
		e_x = *cx + range;
		e_y = *cy;
		eflag = checkCollison(e_x,e_y);

		// south
		s_x = *cx;
		s_y = *cy - range;
		sflag = checkCollison(s_x,s_y);

		// west
		w_x = *cx - range;
		w_y = *cy;
		wflag = checkCollison(w_x,w_y);

		// north east
		ne_x = *cx + range;
		ne_y = *cy + range;
		neflag = checkCollison(ne_x,ne_y);

		// south east
		se_x = *cx + range;
		se_y = *cy - range;
		seflag = checkCollison(se_x,se_y);

		// south west
		sw_x = *cx - range;
		sw_y = *cy - range;
		swflag = checkCollison(sw_x,sw_y);

		// north west
		nw_x = *cx - range;
		nw_y = *cy + range;
		nwflag = checkCollison(nw_x,nw_y);

		dirVector = std::make_tuple(nflag,eflag,sflag, wflag, neflag,seflag,swflag,nwflag);

		dir = getDir(dirVector);
		// cout << "Dir = " << dir << endl;

		// figure out where to go next if we arent where we began
		if(dir == 'n'){
			*cx = *cx;
			*cy = *cy +range/2.0;
		}

		if(dir == 'e'){
			*cx = *cx + range/2.0;
			*cy = *cy;
		}

		if(dir == 's'){
			*cx = *cx;
			*cy = *cy -range/2.0;
		}

		if(dir == 'w'){
			*cx = *cx - range/2.0;
			*cy = *cy;
		}


		route_2.push_back(std::make_tuple(*cx,*cy));

		cout << "(cx,cy) = " <<*cx << "," << *cy << endl;

		if ( (yg - *cy)/(xg-*cx)>= (m - 0.001) && (yg - *cy)/(xg-*cx)<= (m + 0.001)){

			if (*cx >= xs && *cx <= xg){
				follow_flag = false;
			}
		}


	}

	cout << "Left Obstacle" << endl;



}
////////////////////////////////////////////////////////////////////////////////

char Map::getDir(std::tuple<bool,bool,bool,bool,bool,bool,bool,bool> dirVector){
		const char north = 'n';
		const char east = 'e';
		const char south = 's';
		const char west = 'w';

		// check north
		if(dirVector == make_tuple(false,true,true,false,true,true,true,false)){
			return north;
		}
		if(dirVector == make_tuple(false,true,false,false,true,true,false,false)){
			return north;
		}
		if(dirVector == make_tuple(false,true,false,false,true,false,false,false)){
			return north;
		}
		if(dirVector == make_tuple(false,false,false,false,true,false,false,false)){
			return north;
		}
		if(dirVector == make_tuple(false,true,false,false,false,true,false,false)){
			return north;
		}

		// check west
		if(dirVector == make_tuple(true,true,false,false,true,true,false,true)){
			return west;
		}
		if(dirVector == make_tuple(true,false,false,false,true,false,false,false)){
			return west;
		}
		if(dirVector == make_tuple(true,false,false,false,true,false,false,true)){
			return west;
		}
		if(dirVector == make_tuple(false,false,false,false,false,false,false,true)){
			return west;
		}
		if(dirVector == make_tuple(true,false,false,false,false,false,false,true)){
			return west;
		}

		// check south
		if(dirVector == make_tuple(false,false,false,true,false,false,true,true)){
			return south;
		}
		if(dirVector == make_tuple(false,false,false,true,false,false,true,false)){
			return south;
		}
		if(dirVector == make_tuple(false,false,false,false,false,false,true,false)){
			return south;
		}
		if(dirVector == make_tuple(false,false,false,true,false,false,false,true)){
			return south;
		}
		if(dirVector == make_tuple(true,false,false,true,true,false,true,true)){
			return south;
		}

		// check east
		if(dirVector == make_tuple(false,false,true,true,false,true,true,true)){
			return east;
		}
		if(dirVector == make_tuple(false,false,true,false,false,true,true,false)){
			return east;
		}
		if(dirVector == make_tuple(false,false,false,false,false,true,false,false)){
			return east;
		}
		if(dirVector == make_tuple(false,false,true,false,false,false,true,false)){
			return east;
		}
		if(dirVector == make_tuple(false,false,true,false,false,true,false,false)){
			return east;
		}

}

////////////////////////////////////////////////////////////////////////////////
void Map::saveRoute(){
	// route
	// ofstream routefile;
	// routefile.open ("routeFile.txt");
	// routefile << cx << "," << cy << endl;
	// routefile.close();
	ofstream routefile;
	routefile.open ("routeFileBug1.txt");

	double x,y;
	double dummyx = 0.0;
	double dummyy = 0.0;
	double dist = 0.0;

	while(!route_1.empty()){
		std::tie (x,y) = route_1.front();

		dist = dist + distance(x,y,dummyx,dummyy);

		dummyx = x;
		dummyy = y;

		routefile << x << "," << y << "," <<  dist<< endl;
		route_1.pop_front();
	}
	dist = 0.0;
	dummyx = 0.0;
	dummyy = 0.0;
	routefile.close();

	// Bug 2 ////////////////////////////////////

	ofstream route2file;
	route2file.open ("routeFileBug2.txt");

	while(!route_2.empty()){
		std::tie (x,y) = route_2.front();

		dist = dist + distance(x,y,dummyx,dummyy);

		dummyx = x;
		dummyy = y;

		route2file << x << "," << y << "," <<  dist<< endl;
		route_2.pop_front();
	}

	route2file.close();
}
// check collision
bool Map::checkCollison(double x, double y){
	double x1,y1,x2,y2,x3,y3,x4,y4;

	for (int i =0 ; i <numObs; i++){


		std::tie(x1, y1, x2, y2, x3, y3, x4, y4) = obstacles[i];


		if ((x >= x1) && (x <= x2)){
			if((y >= y1) && (y <= y3)){

				// cout << "Collision Detected" <<endl;
				// cout << x << "," << y <<endl;
				// cout << x1 << "," << y1 << "," << x2 << "," << y2 << "," << x3 << "," <<y3 << "," << x4 << "," <<y4 << endl;

				return true;

			}
		}

	}

	return false;
}

// distance
double Map::distance( double x1, double y1, double x2, double y2) {
  return (sqrt(pow((x2 - x1),2) + pow((y2 - y1),2)));
}
