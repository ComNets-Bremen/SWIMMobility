/******************************************************************************
 * SWIMMobility - A SWIM implementation for the INET Framework of the OMNeT++ 
 * Simulator.
 *
 * Copyright (C) 2016, Sustainable Communication Networks, University of Bremen, Germany
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; version 3 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>
 *
 *
 ******************************************************************************/

/**
* The C++ include file of the SWIM mobility model for the INET Framework 
* in OMNeT++.
*
* @author : Anas bin Muslim (anas1@uni-bremen.de)
*
*/

#ifndef __INET_SWIMMOBILITY_H
#define __INET_SWIMMOBILITY_H

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "inet/common/INETMath.h"
#include "inet/common/INETDefs.h"
#include "inet/mobility/base/LineSegmentsMobilityBase.h"

struct loc{
    int noOfNodesPresent;

    double myCoordX;
    double myCoordY;
    double myCoordZ;
};

struct nodeProp{
    int seen = 0;

    double locCoordX;
    double locCoordY;
    double locCoordZ;
    double weight = 0;
};

namespace inet {

class INET_API SWIMMobility : public LineSegmentsMobilityBase , public cListener
{
protected:
    bool created;
    bool firstStep;
    bool nextMoveIsWait;

    int update;
    int popularityDecisionThreshold;
    int neighbourLocationLimit;
    int returnHomePercentage;

    double speed;
    double alpha = 0;
    double radius;

    char *updateNodeCountSignal;

    std::vector<loc> locations;
    std::vector<nodeProp> neighborLocs;
    std::vector<nodeProp> visitingLocs;

    Coord neew;

    simsignal_t updateSig;

private:
    Coord Home;

public:
    int noOfLocs;
    int nodes;

    double maxAreaX;
    double maxAreaY;
    double maxAreaZ;


protected:
    virtual int numInitStages() const override { return NUM_INIT_STAGES; }

    virtual void initialize(int stage) override;

    virtual void setTargetPosition() override;

    virtual void move() override;

    /** Create locations **/
    virtual bool createLocations();

    /** Reads the locations from file **/
    virtual bool readLocations();

    /** Separates the visiting and neighboring locations and updates their weights **/
    virtual void seperateAndUpdateWeights();

    /** Decides whether neighbor or visiting locations is to be visited next **/
    virtual Coord decision();

    /** Chooses a random set of Coordinates from given array **/
    virtual Coord chooseDestination(std::vector<nodeProp> array);

    /** Updates the number of nodes for given coordinates **/
    virtual int updateNodesCount(Coord update, bool inc);

    /** Overrides the receive signal to implement extended functionality **/
    virtual void receiveSignal(cComponent *source, simsignal_t signalID, const char *s, cObject *details) override;


public:
    /** Constructor **/
    SWIMMobility();

    /** Destructor **/
    ~SWIMMobility();

};

}//namespace inet

#endif
