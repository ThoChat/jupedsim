#ifndef COGNITIVEMAP_H
#define COGNITIVEMAP_H


#include "associations.h"
#include "waypoints.h"
#include "landmark.h"
#include "../GraphNetwork.h"

#include <queue>
#include <memory>
#include <vector>

class Pedestrian;
class Building;
class JEllipse;



//class priorityCheck
//{
//public:
//  priorityCheck(){}
//  bool operator() (const Waypoint& lhs, const Waypoint& rhs) const
//  {
//    if (lhs.GetPriority() <= rhs.GetPriority())
//      return true;
//    else
//        return false;
//  }
//};


using Waypoints = std::vector<ptrWaypoint>;// std::priority_queue<Waypoint,std::vector<Waypoint>,priorityCheck>;
using ptrBuilding = const Building*;
using ptrPed = const Pedestrian*;
using ptrLandmark = std::shared_ptr<Landmark>;
using ptrGraphNetwork = std::shared_ptr<GraphNetwork>;


class CognitiveMap
{
public:
    CognitiveMap();
    CognitiveMap(ptrBuilding b, ptrPed ped);
    ~CognitiveMap();
    void AddLandmarks(std::vector<ptrLandmark> landmarks);
    void AssessDoors();
    bool IsAroundWaypoint(const Waypoint& waypoint, GraphEdge* edge) const;
    ptrGraphNetwork GetGraphNetwork() const;

private:
    ptrBuilding _building;
    ptrPed _ped;
    ptrGraphNetwork _network;
    Associations _assoContainer;
    std::vector<ptrLandmark> _landmarks;
    Waypoints _waypContainer;


};

#endif // COGNITIVEMAP_H
