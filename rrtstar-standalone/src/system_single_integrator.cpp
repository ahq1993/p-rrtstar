#include "system_single_integrator.h"
#include <cmath>
#include <cstdlib>

#include <iostream>

using namespace std;
using namespace SingleIntegrator;

#define DISCRETIZATION_STEP 0.01


region::region () {
    
    numDimensions = 0;
    
    center = NULL;
    size = NULL;
}


region::~region () {
    
    if (center)
        delete [] center;
    if (size)
        delete [] size;
    
}


int region::setNumDimensions (int numDimensionsIn) {
    
    numDimensions = numDimensionsIn;
    
    if (center)
        delete [] center;
    center = new double[numDimensions];
    
    if (size)
        delete [] size;
    size = new double[numDimensions];
    
    return 1;
    
}


State::State () {
    
    numDimensions = 0;
    
    x = NULL;
}


State::~State () {
    
    if (x)
        delete [] x;
}


State::State (const State &stateIn) {
    
    numDimensions = stateIn.numDimensions;
    
    if (numDimensions > 0) {
        x = new double[numDimensions];
        
        for (int i = 0; i < numDimensions; i++) 
            x[i] = stateIn.x[i];
    }
    else {
        x = NULL;
    }
}


State& State::operator=(const State &stateIn){
    
    if (this == &stateIn)
        return *this;
    
    if (numDimensions != stateIn.numDimensions) {
        if (x) 
            delete [] x;
        numDimensions = stateIn.numDimensions;
        if (numDimensions > 0)
            x = new double[numDimensions];
    }
    
    for (int i = 0; i < numDimensions; i++) 
        x[i] = stateIn.x[i];
    
    return *this;
}


int State::setNumDimensions (int numDimensionsIn) {
    
    if (x)
        delete [] x;
    
    if (numDimensions < 0)
        return 0;
    
    numDimensions = numDimensionsIn;
    
    if (numDimensions > 0)
        x = new double[numDimensions];
    
    return 1;
}


Trajectory::Trajectory () {
    
    endState = NULL;
}


Trajectory::~Trajectory () {
    
    if (endState)
        delete endState;
}


Trajectory::Trajectory (const Trajectory &trajectoryIn) {
    
    endState = new State (trajectoryIn.getEndState()); 

}


Trajectory& Trajectory::operator=(const Trajectory &trajectoryIn) {
    
    if (this == &trajectoryIn)
        return *this;
    
    if (endState)
        delete endState;
    
    
    endState = new State (trajectoryIn.getEndState());
    
    totalVariation = trajectoryIn.totalVariation;
    
    return *this;
}


double Trajectory::evaluateCost () {
    
    return totalVariation;
}


System::System () {
    
    numDimensions = 0;
}


System::~System () {
    
}


int System::setNumDimensions (int numDimensionsIn) {
    
    if (numDimensions < 0)
        return 0;
    
    numDimensions = numDimensionsIn;
    
    rootState.setNumDimensions (numDimensions);
    
    return 1;
}


int System::getStateKey (State& stateIn, double* stateKey) {
    
    for (int i = 0; i < numDimensions; i++) 
        stateKey[i] =  stateIn.x[i] / regionOperating.size[i];
    
    return 1;
}



bool System::isReachingTarget (State &stateIn) {
    
    
    for (int i = 0; i < numDimensions; i++) {
        
        if (fabs(stateIn.x[i] - regionGoal.center[i]) > regionGoal.size[i]/2.0 ) 
            return false;
    }
    
    return true;
}


bool System::IsInCollision (double *stateIn) {
    
    for (list<region*>::iterator iter = obstacles.begin(); iter != obstacles.end(); iter++) {
        
        region *obstacleCurr = *iter;
        bool collisionFound = true;
        
        for (int i = 0; i < numDimensions; i++) 
            if (fabs(obstacleCurr->center[i] - stateIn[i]) > obstacleCurr->size[i]/2.0 ) {
                collisionFound = false;
                break;
            }
        
        if (collisionFound) {
            return true;
        }
    }
    
    return false;
} 


int System::sampleState (State &randomStateOut) {
    
    randomStateOut.setNumDimensions (numDimensions);
    
    for (int i = 0; i < numDimensions; i++) {
        
        randomStateOut.x[i] = (double)rand()/(RAND_MAX + 1.0)*regionOperating.size[i] 
        - regionOperating.size[i]/2.0 + regionOperating.center[i];
    }
    
    if (IsInCollision (randomStateOut.x))
        return 0;
    
    return 1;
}



int System::extendTo (State &stateFromIn, State &stateTowardsIn, Trajectory &trajectoryOut, bool &exactConnectionOut) {
    
    double *dists = new double[numDimensions];
    for (int i = 0; i < numDimensions; i++) 
        dists[i] = stateTowardsIn.x[i] - stateFromIn.x[i];
    
    double distTotal = 0.0;
    for (int i = 0; i < numDimensions; i++) 
        distTotal += dists[i]*dists[i];
    distTotal = sqrt (distTotal);
    
    double incrementTotal = distTotal/DISCRETIZATION_STEP;
    
    // normalize the distance according to the disretization step
    for (int i = 0; i < numDimensions; i++)
        dists[i] /= incrementTotal;
    
    int numSegments = (int)floor(incrementTotal);
    
    double *stateCurr = new double[numDimensions];
    for (int i = 0; i < numDimensions; i++) 
        stateCurr[i] = stateFromIn.x[i];
    
    for (int i = 0; i < numSegments; i++) {
        
        if (IsInCollision (stateCurr))  
            return 0;
        
        for (int i = 0; i < numDimensions; i++)
            stateCurr[i] += dists[i];
    }
    
    if (IsInCollision (stateTowardsIn.x))
        return 0;
    
    trajectoryOut.endState = new State (stateTowardsIn);
    trajectoryOut.totalVariation = distTotal;
    
    delete [] dists;
    delete [] stateCurr;
    
    exactConnectionOut = true;
    
    return 1;
}


double System::evaluateExtensionCost (State& stateFromIn, State& stateTowardsIn, bool &exactConnectionOut) {
    
    
    exactConnectionOut = true;
    
    double distTotal = 0.0;
    for (int i = 0; i < numDimensions; i++) {
        double distCurr = stateTowardsIn.x[i] - stateFromIn.x[i];
        distTotal += distCurr*distCurr;
    }
    
    return sqrt(distTotal);
    
}


int System::getTrajectory (State& stateFromIn, State& stateToIn, list<double*>& trajectoryOut) {
    
    double *stateArr = new double[numDimensions];
    for (int i = 0; i < numDimensions; i++)
        stateArr[i] = stateToIn[i];
    trajectoryOut.push_front (stateArr);
    
    return 1;
    
}


double System::evaluateCostToGo (State& stateIn) {
    
    double radius = 0.0;
    for (int i = 0; i < numDimensions; i++) 
        radius += regionGoal.size[i] * regionGoal.size[i];
    radius = sqrt(radius);
    
    double dist = 0.0;
    for (int i = 0; i < numDimensions; i++) 
        dist += (stateIn[i] - regionGoal.center[i])*(stateIn[0] - regionGoal.center[i]);
    dist = sqrt(dist);
    
    return dist - radius;
}
