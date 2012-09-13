/*
 * GraphRouter.cpp
 *
 *  Created on: Aug 20, 2012
 *      Author: David Haensel
 */

#include "GraphRouter.h"


/******************************
 * GraphGrouter Methods
 *****************************/


GraphRouter::GraphRouter() 
{

}

GraphRouter::~GraphRouter() 
{

}

int GraphRouter::FindExit(Pedestrian* p) 
{
    if(p->GetLastDestination() == -1) {
	//this is needed for initialisation
	p->ChangedSubRoom();
	//Set Initial Route at the beginning
	// get next destination for person in subroom (in the subroom not next to a crossing) 
	ExitDistance ed = g.GetGraph(p->GetKnownClosedDoors())->GetNextDestination(p);
	p->SetExitIndex(ed.GetDest()->id);
	p->SetExitLine(ed.GetDest()->nav_line);
	return 1;
    } else {

	
	//the pedestrian at least had a route, now check if he needs a new one
	//if the pedestrian changed the subroom he needs a new route
	if(p->ChangedSubRoom()) {
	    
	    ExitDistance ed = g.GetGraph(p->GetKnownClosedDoors())->GetNextDestination(p->GetLastDestination(), p);
	    // check if the next destination is in the right subroom
	    // if the routing graph changes, it could happen, that the pedestrian has to turn.
	    if(ed.GetSubRoom()->GetRoomID() != p->GetRoomID() || ed.GetSubRoom()->GetSubRoomID() != p->GetSubRoomID()) {
		p->SetExitIndex(p->GetLastDestination());
		p->SetExitLine(ed.GetSrc()->nav_line);

		return 1;
	    }
	    p->SetExitIndex(ed.GetDest()->id);
	    p->SetExitLine(ed.GetDest()->nav_line);

	    return 1;
	}
	if(p->GetNextDestination() != -1 && !g.GetGraph(p->GetKnownClosedDoors())->GetVertex(p->GetLastDestination())) {
	    	ExitDistance ed = g.GetGraph(p->GetKnownClosedDoors())->GetNextDestination(p);
		p->SetExitIndex(ed.GetDest()->id);
		p->SetExitLine(ed.GetDest()->nav_line);
	}
	//check if the pedestrian reached an hline
	Hline * hline = dynamic_cast<Hline*>(g.GetGraph(p->GetKnownClosedDoors())->GetVertex(p->GetNextDestination())->nav_line);
	if(hline) {

	    // check if the pedestrian is near the Line or In LIne
	    if(g.GetGraph(p->GetKnownClosedDoors())->GetVertex(p->GetNextDestination())->nav_line->DistTo(p->GetPos()) < EPS*10 )  {
		//std::cout << "new route from HLINE" << std::endl; 
		ExitDistance ed = g.GetGraph(p->GetKnownClosedDoors())->GetNextDestination(p->GetLastDestination(),p);
	   
		p->SetExitIndex(ed.GetDest()->id);
		p->SetExitLine(ed.GetDest()->nav_line);
		return 1;
	    }
	}
	Transition * transition = dynamic_cast<Transition*>(g.GetGraph(p->GetKnownClosedDoors())->GetVertex(p->GetNextDestination())->nav_line);
	if(transition) {
	    if(!transition->IsOpen() && transition->DistTo(p->GetPos()) < EPS_INFO_DIST) {
		p->AddKnownClosedDoor(transition->GetUniqueID());
		ExitDistance ed = g.GetGraph(p->GetKnownClosedDoors())->GetNextDestination(p);
		p->SetExitIndex(ed.GetDest()->id);
		p->SetExitLine(ed.GetDest()->nav_line);
	    } 
	}

    // if(p-> GetPedIndex() == 55)
	// std::cout << "55 : " << p->GetKnownClosedDoors().size()<< "\n" << g.GetGraph(p->GetKnownClosedDoors()) << "\n";
	//share Information about closed Doors
	if(p->GetKnownClosedDoors() != empty_set) {
	    // std::cout << "ped" << p->GetPedIndex() << std::endl;
	    
	    SubRoom * sub  = building->GetRoom(p->GetRoomID())->GetSubRoom(p->GetSubRoomID());
	    const vector<Pedestrian*> ps = sub->GetAllPedestrians();
	     
	    for(unsigned int i = 0; i < ps.size(); i++) {
		if((p->GetPos() - ps[i]->GetPos()).NormSquare() < EPS_INFO_DIST * 10) {
		    if(ps[i]->GetKnownClosedDoors() != p->GetKnownClosedDoors())
		    {
			ps[i]->MergeKnownClosedDoors(p->GetKnownClosedDoors());
			//maybe the other pedestrian needs a new route
			ExitDistance ed = g.GetGraph(ps[i]->GetKnownClosedDoors())->GetNextDestination(ps[i]);
			ps[i]->SetExitIndex(ed.GetDest()->id);
			ps[i]->SetExitLine(ed.GetDest()->nav_line);
		    }
		    
		    
		}
		
	    }
	    
	}
	
	return 1;
    }
}


void GraphRouter::Init(Building* b) 
{
  Log->write("ERROR: Router is not ready to use yet");
  building = b;
  g.init(b);
  
  std::cout <<  b->GetTransition("200E Normal Exit E3")->IsOpen() << std::endl; 
  b->GetTransition("200E Normal Exit E3")->Close();

  std::cout <<  b->GetTransition("200E Normal Exit E3")->IsOpen() << std::endl; 

  

}
