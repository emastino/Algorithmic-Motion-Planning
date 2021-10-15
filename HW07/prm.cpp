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
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <chrono>
#include <windows.h>

// dense is part of the Eigen matrix library, when using it, make sure to Include
// the path to it in the compile command:
//
// g++ -I "D:\Users\emast\Personal\Personal Cpp" Map.cpp -o test
//
#include "Eigen/Dense" //using Dense inside Eigen

#include "Map.h"

using namespace Eigen;
using namespace std;
 using namespace std::chrono;

// make obstacle function
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


	// //////////////////////////////////////////////////////////////////////////////
	// // Exercise 1 a)
	// //
	// //////////////////////////////////////////////////////////////////////////////
	// // map limits
	// std::tuple<double, double> xLim = std::make_tuple(-1,11);
	// std::tuple<double, double> yLim = std::make_tuple(-3,3);
	// // map start and goal
	// std::tuple<double, double> start = std::make_tuple(0.0,0.0);
	// std::tuple<double, double> goal = std::make_tuple(10.0,0.0);
  //
  //
	// std::vector<polygon> obstacles = makeObstacles("obstacles_a.txt");
  //
	// // make a map object
	// map mapa(xLim, yLim, start, goal, obstacles);
  //
	// // Benchmark Parameters
	// int numNode[8] = {200,200,200,200,500,500,500,500};
	// double rad[8] = {0.5,1,1.5,2,0.5,1,1.5,2};

	// // file names
	// ofstream PRMBenchmarkFile_VS("PRM_Benchmark_validSol.txt"); // valid solutions
	// ofstream PRMBenchmarkFile_PL("PRM_Benchmark_pathLen.txt");	// path lengths
	// ofstream PRMBenchmarkFile_CT("PRM_Benchmark_compTime.txt"); // compile time
	//
	//
	//
	// for(int j = 0; j<8 ;j++){
	// 	// cout << "n = " << numNode[j] << endl;
	// 	// cout << "r = " << rad[j] << endl;
	// 	for (int i = 0 ; i <100 ; i++){
	//
	// 		PRM_Benchmark pm = mapa.PRM(numNode[j],rad[j], false,false,false);
	//
	// 		PRMBenchmarkFile_VS << pm.validSolution << ", " ;
	// 		PRMBenchmarkFile_PL << pm.pathLength << ", " ;
	// 		PRMBenchmarkFile_CT << pm.compTime << "," ;
	// 		Sleep(1000);
	// 	}
	//
	// 	PRMBenchmarkFile_VS << endl;
	// 	PRMBenchmarkFile_PL << endl;
	// 	PRMBenchmarkFile_CT << endl;
	// }
	//
	// PRMBenchmarkFile_VS.close();
	// PRMBenchmarkFile_PL.close();
	// PRMBenchmarkFile_CT.close();
	//
	// // Debug
	// PRM_Benchmark pm = mapa.PRM(200,1, true,true, true); //int n, double r, bool outputPath, bool outPutPRM, bool smooth

	// /////////                SMOOTHING              ////////////////////////
	// // file names
	// ofstream PRMBenchmarkFile_VS_smooth("PRM_Benchmark_validSol_Smooth_Ex_1_a4.txt"); // valid solutions
	// ofstream PRMBenchmarkFile_PL_smooth("PRM_Benchmark_pathLen_Smooth_Ex_1_a4.txt");	// path lengths
	// ofstream PRMBenchmarkFile_CT_smooth("PRM_Benchmark_compTime_Smooth_Ex_1_a4.txt"); // compile time
  //
  //
	// for(int j = 0; j<8 ;j++){
	// 	// cout << "n = " << numNode[j] << endl;
	// 	// cout << "r = " << rad[j] << endl;
	// 	for (int i = 0 ; i <100 ; i++){
  //
	// 		PRM_Benchmark pm = mapa.PRM(numNode[j],rad[j], false,false,true);
  //
	// 		PRMBenchmarkFile_VS_smooth << pm.validSolution << ", " ;
	// 		PRMBenchmarkFile_PL_smooth << pm.pathLength << ", " ;
	// 		PRMBenchmarkFile_CT_smooth << pm.compTime << "," ;
	// 		// Sleep(1000);
	// 	}
  //
	// 	PRMBenchmarkFile_VS_smooth << endl;
	// 	PRMBenchmarkFile_PL_smooth << endl;
	// 	PRMBenchmarkFile_CT_smooth << endl;
	// }
  //
	// PRMBenchmarkFile_VS_smooth.close();
	// PRMBenchmarkFile_PL_smooth.close();
	// PRMBenchmarkFile_CT_smooth.close();



  //
  // //////////////////////////////////////////////////////////////////////////////
	// // Exercise 1 b) Workspace 1
	// //
	// //////////////////////////////////////////////////////////////////////////////
	// // map limits
	// std::tuple<double, double> xLim = std::make_tuple(-1,14);
	// std::tuple<double, double> yLim = std::make_tuple(-1,14);
	// // map start and goal
	// std::tuple<double, double> start = std::make_tuple(0.0,0.0);
	// std::tuple<double, double> goal = std::make_tuple(10.0,10.0);
  //
  //
	// std::vector<polygon> obstacles = makeObstacles("obstacles_b.txt");
  //
	// // make a map object
	// map mapa(xLim, yLim, start, goal, obstacles);
  //
	// // Benchmark Parameters
	// int numNode[6] = {200,200,500,500,1000,1000};
	// double rad[6] = {1.0,2.0,1.0,2.0,1.0,2.0};
  //
  // // PRM_Benchmark pm = mapa.PRM(200,2.0, true,true, false); //int n, double r, bool outputPath, bool outPutPRM, bool smooth
  //
	// // file names
	// ofstream PRMBenchmarkFile_VS("PRM_Benchmark_validSol.txt"); // valid solutions
	// ofstream PRMBenchmarkFile_PL("PRM_Benchmark_pathLen.txt");	// path lengths
	// ofstream PRMBenchmarkFile_CT("PRM_Benchmark_compTime.txt"); // compile time
  //
  //
  //
	// for(int j = 0; j<6 ;j++){
	// 	// cout << "n = " << numNode[j] << endl;
	// 	// cout << "r = " << rad[j] << endl;
	// 	for (int i = 0 ; i <100 ; i++){
  //
	// 		PRM_Benchmark pm = mapa.PRM(numNode[j],rad[j], false,false,false);
  //
	// 		PRMBenchmarkFile_VS << pm.validSolution << ", " ;
	// 		PRMBenchmarkFile_PL << pm.pathLength << ", " ;
	// 		PRMBenchmarkFile_CT << pm.compTime << "," ;
	//		// Sleep(1000);
	// 	}
  //
	// 	PRMBenchmarkFile_VS << endl;
	// 	PRMBenchmarkFile_PL << endl;
	// 	PRMBenchmarkFile_CT << endl;
	// }
  //
	// PRMBenchmarkFile_VS.close();
	// PRMBenchmarkFile_PL.close();
	// PRMBenchmarkFile_CT.close();
  //
	// // Debug
  //
  //
	// /////////                SMOOTHING              ////////////////////////
	// // file names
	// ofstream PRMBenchmarkFile_VS_smooth("PRM_Benchmark_validSol_Smooth.txt"); // valid solutions
	// ofstream PRMBenchmarkFile_PL_smooth("PRM_Benchmark_pathLen_Smooth.txt");	// path lengths
	// ofstream PRMBenchmarkFile_CT_smooth("PRM_Benchmark_compTime_Smooth.txt"); // compile time
  //
  //
	// for(int j = 0; j<6 ;j++){
	// 	// cout << "n = " << numNode[j] << endl;
	// 	// cout << "r = " << rad[j] << endl;
	// 	for (int i = 0 ; i <100 ; i++){
  //
	// 		PRM_Benchmark pm = mapa.PRM(numNode[j],rad[j], false,false,true);
  //
	// 		PRMBenchmarkFile_VS_smooth << pm.validSolution << ", " ;
	// 		PRMBenchmarkFile_PL_smooth << pm.pathLength << ", " ;
	// 		PRMBenchmarkFile_CT_smooth << pm.compTime << "," ;
	// 		// Sleep(1000);
	// 	}
  //
	// 	PRMBenchmarkFile_VS_smooth << endl;
	// 	PRMBenchmarkFile_PL_smooth << endl;
	// 	PRMBenchmarkFile_CT_smooth << endl;
	// }
  //
	// PRMBenchmarkFile_VS_smooth.close();
	// PRMBenchmarkFile_PL_smooth.close();
	// PRMBenchmarkFile_CT_smooth.close();
  //
  //

  // //////////////////////////////////////////////////////////////////////////////
  // // Exercise 1 b) Workspace 2
  // //
  // //////////////////////////////////////////////////////////////////////////////
  // // map limits
  // std::tuple<double, double> xLim = std::make_tuple(-7,36);
  // std::tuple<double, double> yLim = std::make_tuple(-7,7);
  // // map start and goal
  // std::tuple<double, double> start = std::make_tuple(0.0,0.0);
  // std::tuple<double, double> goal = std::make_tuple(35.0,0.0);
  //
  //
  // std::vector<polygon> obstacles = makeObstacles("obstacles_c.txt");
  //
  // // make a map object
  // map mapa(xLim, yLim, start, goal, obstacles);
  //
  // // Benchmark Parameters
  // int numNode[6] = {200,200,500,500,1000,1000};
  // double rad[6] = {1.0,2.0,1.0,2.0,1.0,2.0};
  // //
  // PRM_Benchmark pm = mapa.PRM(1000,2.0, true,true, true); //int n, double r, bool outputPath, bool outPutPRM, bool smooth

  // // file names
  // ofstream PRMBenchmarkFile_VS("PRM_Benchmark_validSol.txt"); // valid solutions
  // ofstream PRMBenchmarkFile_PL("PRM_Benchmark_pathLen.txt");	// path lengths
  // ofstream PRMBenchmarkFile_CT("PRM_Benchmark_compTime.txt"); // compile time
  //
  //
  //
  // for(int j = 0; j<6 ;j++){
  //   // cout << "n = " << numNode[j] << endl;
  //   // cout << "r = " << rad[j] << endl;
  //   for (int i = 0 ; i <100 ; i++){
  //
  //     PRM_Benchmark pm = mapa.PRM(numNode[j],rad[j], false,false,false);
  //
  //     PRMBenchmarkFile_VS << pm.validSolution << ", " ;
  //     PRMBenchmarkFile_PL << pm.pathLength << ", " ;
  //     PRMBenchmarkFile_CT << pm.compTime << "," ;
  //     Sleep(1000);
  //   }
  //
  //   PRMBenchmarkFile_VS << endl;
  //   PRMBenchmarkFile_PL << endl;
  //   PRMBenchmarkFile_CT << endl;
  // }
  //
  // PRMBenchmarkFile_VS.close();
  // PRMBenchmarkFile_PL.close();
  // PRMBenchmarkFile_CT.close();
  //
  // // Debug
  //
  //
  // /////////                SMOOTHING              ////////////////////////
  // // file names
  // ofstream PRMBenchmarkFile_VS_smooth("PRM_Benchmark_validSol_Smooth.txt"); // valid solutions
  // ofstream PRMBenchmarkFile_PL_smooth("PRM_Benchmark_pathLen_Smooth.txt");	// path lengths
  // ofstream PRMBenchmarkFile_CT_smooth("PRM_Benchmark_compTime_Smooth.txt"); // compile time
  //
  //
  // for(int j = 0; j<6 ;j++){
  //   // cout << "n = " << numNode[j] << endl;
  //   // cout << "r = " << rad[j] << endl;
  //   for (int i = 0 ; i <100 ; i++){
  //
  //     PRM_Benchmark pm = mapa.PRM(numNode[j],rad[j], false,false,true);
  //
  //     PRMBenchmarkFile_VS_smooth << pm.validSolution << ", " ;
  //     PRMBenchmarkFile_PL_smooth << pm.pathLength << ", " ;
  //     PRMBenchmarkFile_CT_smooth << pm.compTime << "," ;
  //     // Sleep(1000);
  //   }
  //
  //   PRMBenchmarkFile_VS_smooth << endl;
  //   PRMBenchmarkFile_PL_smooth << endl;
  //   PRMBenchmarkFile_CT_smooth << endl;
  // }
  //
  // PRMBenchmarkFile_VS_smooth.close();
  // PRMBenchmarkFile_PL_smooth.close();
  // PRMBenchmarkFile_CT_smooth.close();


  //
  // //////////////////////////////////////////////////////////////////////////////
  // // Exercise 2 a) worspace 1
  // //
  // //////////////////////////////////////////////////////////////////////////////
	// // map limits
	// std::tuple<double, double> xLim = std::make_tuple(-1,14);
	// std::tuple<double, double> yLim = std::make_tuple(-1,14);
	// // map start and goal
	// std::tuple<double, double> start = std::make_tuple(0.0,0.0);
	// std::tuple<double, double> goal = std::make_tuple(10.0,10.0);
  //
  //
	// std::vector<polygon> obstacles = makeObstacles("obstacles_b.txt");
  //
	// // make a map object
	// map mapa(xLim, yLim, start, goal, obstacles);
  // //
  // // PRM_Benchmark pm = mapa.GoalBiasRRT(5000, 0.5 ,0.05,0.25, true, true);
  //
  // // Exercise 2 b) Worspace 1 //////////////////////////////////////////////////
  // // file names
  // ofstream PRMBenchmarkFile_VS("Tree_Benchmark_validSol.txt"); // valid solutions
  // ofstream PRMBenchmarkFile_PL("Tree_Benchmark_pathLen.txt");	// path lengths
  // ofstream PRMBenchmarkFile_CT("Tree_Benchmark_compTime.txt"); // compile time
  //
  //
  //
  // for (int i = 0 ; i <100 ; i++){
  //
  // 		PRM_Benchmark pm = mapa.GoalBiasRRT(5000, 0.5 ,0.05,0.25, false, false);
  //
  // 		PRMBenchmarkFile_VS << pm.validSolution << ", " ;
  // 		PRMBenchmarkFile_PL << pm.pathLength << ", " ;
  // 		PRMBenchmarkFile_CT << pm.compTime << "," ;
  // 		// Sleep(1000);
  // }
  //
	// PRMBenchmarkFile_VS << endl;
	// PRMBenchmarkFile_PL << endl;
	// PRMBenchmarkFile_CT << endl;
  //
  // PRMBenchmarkFile_VS.close();
  // PRMBenchmarkFile_PL.close();
  // PRMBenchmarkFile_CT.close();

  /////////////////////////////////////////////////////////////////////////////
  // Exercise 2 a) worspace 2
  //
  //////////////////////////////////////////////////////////////////////////////
  // map limits
  std::tuple<double, double> xLim = std::make_tuple(-7,36);
  std::tuple<double, double> yLim = std::make_tuple(-7,7);
  // map start and goal
  std::tuple<double, double> start = std::make_tuple(0.0,0.0);
  std::tuple<double, double> goal = std::make_tuple(35.0,0.0);


  std::vector<polygon> obstacles = makeObstacles("obstacles_c.txt");

  // make a map object
  map mapa(xLim, yLim, start, goal, obstacles);


  // PRM_Benchmark pm = mapa.GoalBiasRRT(5000, 0.5 ,0.05,0.25, true, true);
  // Exercise 2 b) Worspace 2 //////////////////////////////////////////////////
  // file names
  ofstream PRMBenchmarkFile_VS("Tree_Benchmark_validSol.txt"); // valid solutions
  ofstream PRMBenchmarkFile_PL("Tree_Benchmark_pathLen.txt");	// path lengths
  ofstream PRMBenchmarkFile_CT("Tree_Benchmark_compTime.txt"); // compile time



  for (int i = 0 ; i <100 ; i++){

  		PRM_Benchmark pm = mapa.GoalBiasRRT(5000, 0.5 ,0.05,0.25, false, false);

  		PRMBenchmarkFile_VS << pm.validSolution << ", " ;
  		PRMBenchmarkFile_PL << pm.pathLength << ", " ;
  		PRMBenchmarkFile_CT << pm.compTime << "," ;
  		// Sleep(1000);
  }

	PRMBenchmarkFile_VS << endl;
	PRMBenchmarkFile_PL << endl;
	PRMBenchmarkFile_CT << endl;

  PRMBenchmarkFile_VS.close();
  PRMBenchmarkFile_PL.close();
  PRMBenchmarkFile_CT.close();




	return 0;
}
