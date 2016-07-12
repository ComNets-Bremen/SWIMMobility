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
* The C++ implementation file of the SWIM mobility model for the INET Framework 
* in OMNeT++.
*
* @author : Anas bin Muslim (anas1@uni-bremen.de)
*
*/

#include <algorithm>

#include "inet/mobility/single/SWIMMobility.h"

namespace inet{

Define_Module(SWIMMobility);

bool sortByWeight(const nodeProp &a, const nodeProp &b) { return a.weight > b.weight; }

SWIMMobility::SWIMMobility()
{
    nextMoveIsWait = false;
    created = false;
    firstStep = true;
    maxAreaX = 400;
    maxAreaY = 400;
    maxAreaZ = 0;
}

void SWIMMobility::initialize(int stage){

    srand(time(NULL));

    LineSegmentsMobilityBase::initialize(stage);

    if(stage == 0){
        updateSig = registerSignal("updateSignal");
        subscribe("updateSignal", this);
        updateNodeCountSignal = (char*) malloc(sizeof(char)*25);

        speed = par("speed");
        alpha = par("alpha");
        noOfLocs = par("noOfLocations");
        radius = par("radius");
        popularityDecisionThreshold = par("popularityDecisionThreshold");
        neighbourLocationLimit = par("neighbourLocationLimit");
        returnHomePercentage = par("returnHomePercentage");

        if(radius == 0) radius = 1;

        locations.resize(noOfLocs);

        if(created == false && !(std::ifstream("locations.txt"))){
            created = createLocations();

            if(created) EV<<"Locations created"<<endl;
            else EV<<"Locations can't be created"<<endl;
        }
    }
    if(stage==13){
        srand(time(NULL));

        nodes = par("Hosts");
        maxAreaX = par("maxAreaX");
        maxAreaY = par("maxAreaY");
        maxAreaZ = par("maxAreaZ");

        Home = this->getCurrentPosition();

        EV<<"My Home (Node"<<this->getId()<<") is "<<Home<<endl;
    }
}

void SWIMMobility::setTargetPosition(){

    srand(time(NULL));

    if(nextMoveIsWait){
        simtime_t waitTime = par("waitTime");
        nextChange = simTime() + waitTime.dbl();
    }

    else{
        if(((double)rand()/(double)RAND_MAX) < (returnHomePercentage/100.00)){
            targetPosition = Home;
            Coord positionDelta = targetPosition - lastPosition;
            double distance = positionDelta.length();
            nextChange = simTime() + distance/speed;
        }

        else{
            if(firstStep){
                if(readLocations())
                    EV<<"Locations read successfully"<<endl;
            }

            seperateAndUpdateWeights();

            if(!firstStep){
                sprintf(updateNodeCountSignal,"%g %g %g %d", neew.x, neew.y, neew.z, 0);
                emit(updateSig, updateNodeCountSignal);
            }

            targetPosition = decision();
            Coord positionDelta = targetPosition - lastPosition;
            double distance = positionDelta.length();
            nextChange = simTime() + distance/speed;

            sprintf(updateNodeCountSignal,"%g %g %g %d", neew.x, neew.y, neew.z, 1);
            emit(updateSig, updateNodeCountSignal);
        }
    }
    firstStep=false;
    nextMoveIsWait = !nextMoveIsWait;
}

void SWIMMobility::move(){
    LineSegmentsMobilityBase::move();
    raiseErrorIfOutside();
}

bool SWIMMobility::createLocations(){
    srand(time(NULL));

    bool opn = false;

    std::ofstream outfile;
    outfile.open("locations.txt",std::ios::out|std::ios::trunc);

    if(outfile.is_open())
        opn = 1;

    for(int i=0;i<noOfLocs;i++){
        if(maxAreaX>0)locations[i].myCoordX=rand()%((int)round(maxAreaX));
        else locations[i].myCoordX= 0;
        if(maxAreaY>0)locations[i].myCoordY=rand()%((int)round(maxAreaY));
        else locations[i].myCoordY = 0;
        if(maxAreaZ>0)locations[i].myCoordZ=rand()%((int)round(maxAreaZ));
        else locations[i].myCoordZ = 0;
        locations[i].noOfNodesPresent=0;
        outfile<<locations[i].myCoordX<<" "<<locations[i].myCoordY<<" "<<locations[i].myCoordZ<<" "<<locations[i].noOfNodesPresent<<endl;
    }

    outfile.close();
    return opn;
}

bool SWIMMobility::readLocations(){
    std::ifstream infile;
    infile.open("locations.txt",std::ios::in);

    if(!(infile.is_open()))return 0;

    for(int i=0;i<noOfLocs;i++){
        infile>>locations[i].myCoordX;
        infile>>locations[i].myCoordY;
        infile>>locations[i].myCoordZ;
        infile>>locations[i].noOfNodesPresent;
    }

    infile.close();
    return 1;
}

void SWIMMobility::seperateAndUpdateWeights(){
    Coord temp;
    int noOfNeighbors = 0, n = 0, v = 0;

    double maxWeight = alpha * ( sqrt( pow(maxAreaX,2) + pow(maxAreaY,2) + pow(maxAreaZ,2)) ) + (1 - alpha) * nodes;

    for(int i=0;i<noOfLocs;i++){
        temp.x = locations[i].myCoordX;
        temp.y = locations[i].myCoordY;
        temp.z = locations[i].myCoordZ;

        if(temp.distance(Home)<=neighbourLocationLimit){
            noOfNeighbors++;
        }
    }
    neighborLocs.resize(noOfNeighbors);
    visitingLocs.resize(noOfLocs-noOfNeighbors);

    for(int i=0;i<noOfLocs;i++){
        temp.x = locations[i].myCoordX;
        temp.y = locations[i].myCoordY;
        temp.z = locations[i].myCoordZ;

        if(temp.distance(Home)<=neighbourLocationLimit){
            neighborLocs[n].locCoordX = locations[i].myCoordX;
            neighborLocs[n].locCoordY = locations[i].myCoordY;
            neighborLocs[n].locCoordZ = locations[i].myCoordZ;
            neighborLocs[n].seen = locations[i].noOfNodesPresent;
            neighborLocs[n].weight = (alpha * (temp.distance(Home)) + (1-alpha) * neighborLocs[n].seen);
            neighborLocs[n].weight = neighborLocs[n].weight/maxWeight;
            n++;
        }
        else{
            visitingLocs[v].locCoordX = locations[i].myCoordX;
            visitingLocs[v].locCoordY = locations[i].myCoordY;
            visitingLocs[v].locCoordZ = locations[i].myCoordZ;
            visitingLocs[v].seen = locations[i].noOfNodesPresent;
            visitingLocs[v].weight = (alpha * (temp.distance(Home)) + (1-alpha) * visitingLocs[v].seen);
            visitingLocs[v].weight = visitingLocs[v].weight/maxWeight;
            v++;
        }
    }
}

Coord SWIMMobility::decision(){
    srand(time(NULL));

    Coord dest;
    nodeProp temp;

    sort(neighborLocs.begin(), neighborLocs.end(), sortByWeight);
    sort(visitingLocs.begin(), visitingLocs.end(), sortByWeight);

    if ((rand()%10)<=(alpha*10)){

        //Choose Neighbor location as next destination
        dest = chooseDestination(neighborLocs);
        if(!(dest.x <= 0 && dest.y <= 0 && dest.z <= 0))
            return dest;

        //Choose visiting location as next destination (NL 0 0 0)
        else{
            return chooseDestination(visitingLocs);
        }
    }
    else{

        //Choose visiting location as next destination
        dest  = chooseDestination(visitingLocs);
        if (!(dest.x <= 0.0 && dest.y <= 0.0 && dest.z <= 0.0))
            return dest;

        //Choose Neighbor location as next destination (VL 0 0 0)
        else{
            return chooseDestination(neighborLocs);
        }
    }
}

Coord SWIMMobility::chooseDestination(std::vector<nodeProp> array){
    srand(time(NULL));

    int size = array.size();
    int random;
    int popular = 0;
    int notPopular = 0;

    Coord temp;
    Coord target;

    random = rand();

    if(size>0){
        random=random%size;
    }
    else{
        //There are no locations to be chosen from
        temp.x = 0;
        temp.y = 0;
        temp.z = 0;
        return temp;
    }

    for(int i=0; i<size; i++){
        if(array[i].weight > 0.75)
            popular++;
    }
    notPopular = array.size() - popular;

    if(popular > 0){
        if(rand()%10 > (10 - popularityDecisionThreshold)){
            //Choosing popular location
            random = rand()%popular;
            temp.x = array[random].locCoordX;
            temp.y = array[random].locCoordY;
            temp.z = array[random].locCoordZ;
        }
        else{
            //Choosing not much popular location
            random = rand()%notPopular;
            temp.x = array[random+popular].locCoordX;
            temp.y = array[random+popular].locCoordY;
            temp.z = array[random+popular].locCoordZ;
        }
    }
    else{
        if(notPopular > 0){
            //Choosing not much popular location
            random = rand()%notPopular;
            temp.x = array[random+popular].locCoordX;
            temp.y = array[random+popular].locCoordY;
            temp.z = array[random+popular].locCoordZ;
        }
        else{
            //There are no locations to be chosen from such category
            temp.x = 0;
            temp.y = 0;
            temp.z = 0;
            return temp;
        }
    }

    target.x = temp.x - radius + (rand()%int(radius*2));
    target.y = sqrt(pow(radius,2) - pow(temp.x-target.x,2));
    if(target.y>0){
        target.y = temp.y - target.y + (rand()%int(target.y*2));
    }
    else{
        target.y=temp.y;
    }
    target.z = temp.z;

    neew = temp;

    if(target.x<0) target.x=target.x*(-1);
    if(target.y<0) target.y=target.y*(-1);
    if(target.z<0) target.z=target.z*(-1);

    return target;
}

int SWIMMobility::updateNodesCount(Coord update,bool inc){
    bool inc_success=false;
    bool dec_success=false;

    Coord temp3;

    for(int i=0;i<noOfLocs;i++){
        temp3.x=locations[i].myCoordX;
        temp3.y=locations[i].myCoordY;
        temp3.z=locations[i].myCoordZ;
        if(inc==false && update==temp3){
            if(locations[i].noOfNodesPresent>0){
                locations[i].noOfNodesPresent--;
                dec_success=true;
            }
        }
        if(inc==true && update==temp3){
            locations[i].noOfNodesPresent++;
            inc_success=true;
        }
    }

    if(inc_success)return 1;
    else if(dec_success)return 2;
    else return 0;
}

void SWIMMobility :: receiveSignal(cComponent *source, simsignal_t signalID, const char *s, cObject *details)
{
    int i=0;
    double cord[4];
    char string[25];
    char * temp;
    Coord updateThisLocation;

    strncpy(string,s,25);
    temp = strtok(string, " ");
    cord[i]=atof(temp);

    for(int i=1;i<=3;i++){
        temp = strtok(NULL," ");
        cord[i]=atof(temp);
    }

    updateThisLocation.x=cord[0];
    updateThisLocation.y=cord[1];
    updateThisLocation.z=cord[2];

    if(cord[3]==0){
        update = updateNodesCount(neew,false);
    }
    else{
        update = updateNodesCount(neew,true);
    }
}

SWIMMobility::~SWIMMobility(){
    free(updateNodeCountSignal);
}

} // namespace inet
