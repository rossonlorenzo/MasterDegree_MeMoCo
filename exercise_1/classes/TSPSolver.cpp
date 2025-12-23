/**
 * @file TSPSolver.cpp
 * @brief TSP solver (neighborhood search)
 *
 */

#include "TSPSolver.h"
#include <iostream>

bool TSPSolver::solve ( const TSP& tsp , const TSPSolution& initSol , TSPSolution& bestSol )
{
  try
  {
    bool stop = false;
    int  iter = 0;

    TSPSolution currSol(initSol);

    while ( ! stop ) {
		
    	/// TODO: replace the following by the local search iteration 
      //  that updates currSol if an improving neighbor exists and 
      //  stops otherwise
    	stop = true;
      
    }
    bestSol = currSol;
  }
  catch(std::exception& e)
  {
    std::cout << ">>>EXCEPTION: " << e.what() << std::endl;
    return false;
  }
  return true;
}

//TODO: "internal methods", if any

