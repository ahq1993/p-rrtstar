#define LIBBOT_PRESENT 0

#include <iostream>
#include <ctime>


#include "rrts.hpp"
#include "system_single_integrator.h"


using namespace RRTstar;
using namespace SingleIntegrator;

using namespace std;



typedef Planner<State,Trajectory,System> planner_t;
typedef Vertex<State,Trajectory,System> vertex_t;



int main () {
    
    
    planner_t rrts;
    
    cout << "RRTstar is alive" << endl;
    
    
    
    
    // Create the dynamical system
    System system;
    
    // Three dimensional configuration space
    system.setNumDimensions (3);
    
    // Define the operating region
    system.regionOperating.setNumDimensions(3);
    system.regionOperating.center[0] = 0.0;
    system.regionOperating.center[1] = 0.0;
    system.regionOperating.center[2] = 0.0;
    system.regionOperating.size[0] = 20.0;
    system.regionOperating.size[1] = 20.0;
    system.regionOperating.size[2] = 20.0;
    
    // Define the goal region
    system.regionGoal.setNumDimensions(3);
    system.regionGoal.center[0] = 2.0;
    system.regionGoal.center[1] = 2.0;
    system.regionGoal.center[2] = 2.0;
    system.regionGoal.size[0] = 2.0;
    system.regionGoal.size[1] = 2.0;
    system.regionGoal.size[2] = 2.0;
    
    
    // Define the obstacle region
    region *obstacle;
    
    obstacle = new region;
    obstacle->setNumDimensions(3);
    obstacle->center[0] = 0;
    obstacle->center[1] = 0;
    obstacle->center[2] = 6;
    obstacle->size[0] = 10;
    obstacle->size[1] = 10;
    obstacle->size[2] = 8;
    
    system.obstacles.push_front (obstacle);  // Add the obstacle to the list
    

    

    // Add the system to the planner
    rrts.setSystem (system);
    
    // Set up the root vertex
    vertex_t &root = rrts.getRootVertex();  
    State &rootState = root.getState();
    rootState[0] = 0.0;
    rootState[1] = 0.0;
    rootState[2] = 0.0;
    
    
    // Initialize the planner
    rrts.initialize ();
    
    // This parameter should be larger than 1.5 for asymptotic 
    //   optimality. Larger values will weigh on optimization 
    //   rather than exploration in the RRT* algorithm. Lower 
    //   values, such as 0.1, should recover the RRT.
    rrts.setGamma (1.5);

    
    
    clock_t start = clock();
    
    // Run the algorithm for 10000 iteartions
    for (int i = 0; i < 2000; i++) 
        rrts.iteration ();
    
    clock_t finish = clock();
    cout << "Time : " << ((double)(finish-start))/CLOCKS_PER_SEC << endl;

    

    
    return 1;
}

