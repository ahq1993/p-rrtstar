#define LIBBOT_PRESENT 0

#include <iostream>
#include <ctime>

#include <bot_core/bot_core.h>

#include <lcm/lcm.h>

#include <lcmtypes/lcmtypes.h>

#include "rrts.hpp"
#include "system_single_integrator.h"


using namespace RRTstar;
using namespace SingleIntegrator;

using namespace std;



typedef Planner<State,Trajectory,System> planner_t;
typedef Vertex<State,Trajectory,System> vertex_t;


int publishTree (lcm_t *lcm, planner_t& planner, System& system);
int publishTraj (lcm_t *lcm, planner_t& planner, System& system);
int publishEnvironment (lcm_t *lcm, region& regionOperating, region& regionGoal, list<region*>& obstacles);


int main () {
    
    
    planner_t rrts;
    
    cout << "RRTstar is alive" << endl;
    
    
    // Get lcm
    lcm_t *lcm = bot_lcm_get_global (NULL);
    
    
    // Create the dynamical system
    System system;
    
    // Configuration space   
    system.setNumDimensions (2);
    
    // Define the operating region
    system.regionOperating.setNumDimensions(2);
    system.regionOperating.center[0] = 0.0;
    system.regionOperating.center[1] = 0.0;

    system.regionOperating.size[0] = 400.0;
    system.regionOperating.size[1] = 400.0;
    
    
    // Define the goal region 
    system.regionGoal.setNumDimensions(2);
    system.regionGoal.center[0] = -25.0;	  
    system.regionGoal.center[1] = -120.0; 

    system.regionGoal.size[0] = 2.0;
    system.regionGoal.size[1] = 2.0;
    
    
    // Define the obstacle region
    region *obstaclel1, *obstaclel2, *obstaclel3, *obstacler1, *obstacler2, *obstacler3;      
    
    obstaclel1 = new region;   
    obstaclel1->setNumDimensions(2);
    obstaclel1->center[0] = 25;	
    obstaclel1->center[1] = 100;
    obstaclel1->size[0] = 20;
    obstaclel1->size[1] = 5;

    obstaclel2 = new region;   
    obstaclel2->setNumDimensions(2);
    obstaclel2->center[0] = 35;	
    obstaclel2->center[1] = 120;
    obstaclel2->size[0] = 5;
    obstaclel2->size[1] = 40;

    obstaclel3 = new region;   
    obstaclel3->setNumDimensions(2);
    obstaclel3->center[0] = 15;	
    obstaclel3->center[1] = 120;
    obstaclel3->size[0] = 5;
    obstaclel3->size[1] = 40;

    obstacler1 = new region;   
    obstacler1->setNumDimensions(2);
    obstacler1->center[0] = -25;	
    obstacler1->center[1] = -100;
    obstacler1->size[0] = 20;
    obstacler1->size[1] = 5;
  
    obstacler2 = new region;   
    obstacler2->setNumDimensions(2);
    obstacler2->center[0] = -35;	
    obstacler2->center[1] = -120;
    obstacler2->size[0] = 5;
    obstacler2->size[1] = 40;

    obstacler3 = new region;   
    obstacler3->setNumDimensions(2);
    obstacler3->center[0] = -15;	
    obstacler3->center[1] = -120;
    obstacler3->size[0] = 5;
    obstacler3->size[1] = 40;



    system.obstacles.push_front (obstaclel1);
    system.obstacles.push_front (obstaclel2);  
    system.obstacles.push_front (obstaclel3);
    system.obstacles.push_front (obstacler1);
    system.obstacles.push_front (obstacler2); 
    system.obstacles.push_front (obstacler3);

    publishEnvironment (lcm, system.regionOperating, system.regionGoal, system.obstacles);

    

    // Add the system to the planner
    rrts.setSystem (system);
    
    //ROOT VERTEX
    vertex_t &root = rrts.getRootVertex();  
    State &rootState = root.getState(); 
    rootState[0] = 25.0;
    rootState[1] = 120.0;
     
    
    
    // Initialize the planner
    rrts.initialize ();
    
    // This parameter should be larger than 1.5 for asymptotic 
    //   optimality. Larger values will weigh on optimization 
    //   rather than exploration in the RRT* algorithm. Lower 
    //   values, such as 0.1, should recover the RRT.
    rrts.setGamma (1.5);

    
    
    clock_t start = clock();
    
    // Run the algorithm for 10000 iteartions
    for (int i = 0; i < 10000; i++) {
        rrts.iteration ();
		if (&rrts.getBestVertex() != NULL) {
			cout << "Found best vertex in " << i << " iterations" << endl;
			break;
		}
		
    }
    
     clock_t finish = clock();
    cout << "Time : " << ((double)(finish-start))/CLOCKS_PER_SEC << endl;
    
    publishTree (lcm, rrts, system);
    
    publishTraj (lcm, rrts, system);
    

    return 1;
}



int publishEnvironment (lcm_t *lcm, region& regionOperating, region& regionGoal, list<region*>& obstacles) {
    
    // Publish the environment
    lcmtypes_environment_t *environment = (lcmtypes_environment_t*) malloc (sizeof(lcmtypes_environment_t));

    environment->operating.center[0] = regionOperating.center[0];
    environment->operating.center[1] = regionOperating.center[1];
   
    environment->operating.size[0] = regionOperating.size[0];
    environment->operating.size[1] = regionOperating.size[1];


    environment->goal.center[0] = regionGoal.center[0];
    environment->goal.center[1] = regionGoal.center[1];
  
    environment->goal.size[0] = regionGoal.size[0];
    environment->goal.size[1] = regionGoal.size[1];

    
    environment->num_obstacles = obstacles.size();
    
    if (environment->num_obstacles > 0) 
        environment->obstacles = (lcmtypes_region_3d_t *) malloc (sizeof(lcmtypes_region_3d_t));
    
    int idx_obstacles = 0;
    for (list<region*>::iterator iter = obstacles.begin(); iter != obstacles.end(); iter++){
        
        region* obstacleCurr = *iter;
       
        environment->obstacles[idx_obstacles].center[0] = obstacleCurr->center[0];
        environment->obstacles[idx_obstacles].center[1] = obstacleCurr->center[1];
    
        environment->obstacles[idx_obstacles].size[0] = obstacleCurr->size[0];
        environment->obstacles[idx_obstacles].size[1] = obstacleCurr->size[1];

        
        idx_obstacles++;
    }
    
    lcmtypes_environment_t_publish (lcm, "ENVIRONMENT", environment);
    
    return 1;
}


int publishTraj (lcm_t *lcm, planner_t& planner, System& system) {
    
    
    cout << "Publishing trajectory -- start" << endl;
    
    vertex_t& vertexBest = planner.getBestVertex ();
    
    if (&vertexBest == NULL) {
        cout << "No best vertex" << endl;
        return 0;
    }
    
    list<double*> stateList;
    
    planner.getBestTrajectory (stateList);
    
    lcmtypes_trajectory_t *opttraj = (lcmtypes_trajectory_t *) malloc (sizeof (lcmtypes_trajectory_t));
    
    opttraj->num_states = stateList.size();
    opttraj->states = (lcmtypes_state_t *) malloc (opttraj->num_states * sizeof (lcmtypes_state_t));
    
    int stateIndex = 0;
    for (list<double*>::iterator iter = stateList.begin(); iter != stateList.end(); iter++) {
        
        double* stateRef = *iter;
        opttraj->states[stateIndex].x = stateRef[0];
        opttraj->states[stateIndex].y = stateRef[1];
        if (system.getNumDimensions() > 2)
            opttraj->states[stateIndex].z = stateRef[2];
        else
            opttraj->states[stateIndex].z = 0.0;
        
        delete [] stateRef;
        
        stateIndex++;
    }
    
    
    lcmtypes_trajectory_t_publish (lcm, "TRAJECTORY", opttraj);
    
    lcmtypes_trajectory_t_destroy (opttraj);
    
    cout << "Publishing trajectory -- end" << endl;
    
    
    
    return 1;
}




int publishTree (lcm_t *lcm, planner_t& planner, System& system) {
    
    
    cout << "Publishing the tree -- start" << endl;
    
    bool plot3d = (system.getNumDimensions() > 2);
    
    lcmtypes_graph_t *graph = (lcmtypes_graph_t *) malloc (sizeof (lcmtypes_graph_t));
    graph->num_vertices = planner.numVertices; 
    
    
    if (graph->num_vertices > 0) {    
        
        graph->vertices = (lcmtypes_vertex_t *) malloc (graph->num_vertices * sizeof(lcmtypes_vertex_t));
        
        int vertexIndex = 0;
        for (list<vertex_t*>::iterator iter = planner.listVertices.begin(); iter != planner.listVertices.end(); iter++) {
            
            
            vertex_t &vertexCurr = **iter;
            State &stateCurr = vertexCurr.getState ();
            
            graph->vertices[vertexIndex].state.x = stateCurr[0];
            graph->vertices[vertexIndex].state.y = stateCurr[1];
            if (plot3d) 
                graph->vertices[vertexIndex].state.z = stateCurr[2];
            else 
                graph->vertices[vertexIndex].state.z = 0.0;
            
            vertexIndex++;
            
        }
        
    }
    else {
        graph->vertices = NULL;
    }
    
    if (graph->num_vertices > 1) {
        
        graph->num_edges = graph->num_vertices - 1;
        graph->edges = (lcmtypes_edge_t *) malloc (graph->num_edges * sizeof(lcmtypes_edge_t));
        
        
        int edgeIndex = 0;
        for (list<vertex_t*>::iterator iter = planner.listVertices.begin(); iter != planner.listVertices.end(); iter++) {
            
            vertex_t &vertexCurr = **iter;
            
            vertex_t &vertexParent = vertexCurr.getParent();
            
            if ( &vertexParent == NULL ) 
                continue;
            
            State &stateCurr = vertexCurr.getState ();
            State &stateParent = vertexParent.getState();
            
            
            graph->edges[edgeIndex].vertex_src.state.x = stateParent[0];
            graph->edges[edgeIndex].vertex_src.state.y = stateParent[1];
            if (plot3d)
                graph->edges[edgeIndex].vertex_src.state.z = stateParent[2];
            else 
                graph->edges[edgeIndex].vertex_src.state.z = 0.0;
            
            
            graph->edges[edgeIndex].vertex_dst.state.x = stateCurr[0];
            graph->edges[edgeIndex].vertex_dst.state.y = stateCurr[1];
            if (plot3d)
                graph->edges[edgeIndex].vertex_dst.state.z = stateCurr[2];
            else 
                graph->edges[edgeIndex].vertex_dst.state.z = 0.0;
            
            graph->edges[edgeIndex].trajectory.num_states = 0;
            graph->edges[edgeIndex].trajectory.states = NULL;
            
            edgeIndex++;
        }
        
    }
    else {
        graph->num_edges = 0;
        graph->edges = NULL;
    }
    
    lcmtypes_graph_t_publish (lcm, "GRAPH", graph);
    
    lcmtypes_graph_t_destroy (graph);
    
    cout << "Publishing the tree -- end" << endl;
    
    return 1;
}

