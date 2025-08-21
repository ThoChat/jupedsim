#include "HumanoidModelV0.hpp"

#include "Ellipse.hpp"
#include "GenericAgent.hpp"
#include "Macros.hpp"
#include "Mathematics.hpp"
#include "NeighborhoodSearch.hpp"
#include "OperationalModel.hpp"
#include "OperationalModelType.hpp"
#include "Simulation.hpp"
#include "HumanoidModelV0Data.hpp"

#include <Logger.hpp>
#include <iostream>
#include <stdexcept>
#include <cmath>


HumanoidModelV0::HumanoidModelV0(double bodyForce_, double friction_)
    : bodyForce(bodyForce_), friction(friction_){};

OperationalModelType HumanoidModelV0::Type() const
{
    return OperationalModelType::HUMANOID_V0;
}

std::unique_ptr<OperationalModel> HumanoidModelV0::Clone() const
{
    return std::make_unique<HumanoidModelV0>(*this);
}


OperationalModelUpdate HumanoidModelV0::ComputeNewPosition(
    double dT,
    const GenericAgent& ped,
    const CollisionGeometry& geometry,
    const NeighborhoodSearchType& neighborhoodSearch) const
{
    const auto& model = std::get<HumanoidModelV0Data>(ped.model);
    HumanoidModelV0Update update{};
    auto forces = DrivingForce(ped);

    const auto neighborhood = neighborhoodSearch.GetNeighboringAgents(ped.pos, this->_cutOffRadius);
    Point F_rep;
    for(const auto& neighbor : neighborhood) {
        if(neighbor.id == ped.id) {
            continue;
        }
        F_rep += AgentForce(ped, neighbor);  
    }
    forces += F_rep / model.mass;
    const auto& walls = geometry.LineSegmentsInApproxDistanceTo(ped.pos);

    const auto obstacle_f = std::accumulate(
        walls.cbegin(),
        walls.cend(),
        Point(0, 0),
        [this, &ped](const auto& acc, const auto& element) {
            return acc + ObstacleForce(ped, element);
        });
    forces += obstacle_f / model.mass;

    // Physical interaction flag
    bool has_nearby_agent = std::any_of(
        neighborhood.cbegin(),
        neighborhood.cend(),
        [&ped, &model](const GenericAgent& neighbor) {
            if (neighbor.id == ped.id) return false;
            const double distance = (ped.pos - neighbor.pos).Norm();
            return distance < 2 * model.radius;
        }
    );

    bool has_nearby_wall = std::any_of(
        walls.cbegin(),
        walls.cend(),
        [&ped, &model](const LineSegment& wall) {
            const Point closest_point = wall.ShortestPoint(ped.pos);
            const double dist = (ped.pos - closest_point).Norm();
            return dist < 2 * model.radius;
        }
    );

    bool physical_interaction_flag = has_nearby_agent || has_nearby_wall;



    // creating update of pelvis position based on the collision avoidance model
    update.velocity = model.velocity + forces * dT;
    update.position = ped.pos + update.velocity * dT;


    /// #### Humanoid paradigm #####


    // create the update containing the full body motion after gait computation
    HumanoidModelV0Update update_gait_motion;
    
    
    if (model.head_position.z <= 0.001) 
        {
        //################### initialise posture //###################
        // Create a new instance of HumanoidModelV0Data
        HumanoidModelV0Data init_model = model; // copy current model 

        // change radius to prevent the feet to enter walls (temporary)
        // init_model.radius = 2;

        // change joint position to match the initial position and direction

        // pelvis
        init_model.pelvis_position.x = ped.pos.x;
        init_model.pelvis_position.y = ped.pos.y;
        init_model.pelvis_position.z = model.height * ( LEG_SCALING_FACTOR + ANKLE_SCALING_FACTOR);

        // std::cout << "Pelvis init position: " << init_model.pelvis_position.x << ", "
        //                                     << init_model.pelvis_position.y  << std::endl;

        // head
        init_model.head_position.x = ped.pos.x + update.velocity.x * dT; // here the CoM moves forward to initiate the walking motion
        init_model.head_position.y = ped.pos.y + update.velocity.x * dT;
        init_model.head_position.z = model.height * (  ANKLE_SCALING_FACTOR
                                                        + LEG_SCALING_FACTOR 
                                                        + TRUNK_HEIGHT_SCALING_FACTOR
                                                        + NECK_SCALING_FACTOR);

        // Xcom
        // double w0 = std::sqrt(9.81 / (model.height * ( LEG_SCALING_FACTOR + ANKLE_SCALING_FACTOR)));
        init_model.Xcom = ped.pos;

        Point orientation = update.velocity.Normalized();
        Point normal_orientation = orientation.Rotate90Deg();
        
        // right foot
        // # heel
        init_model.heel_right_position.x = (ped.pos.x - normal_orientation.x * model.height 
                                                                            * PELVIS_WIDTH_SCALING_FACTOR * 0.5);
        init_model.heel_right_position.y = (ped.pos.y - normal_orientation.y* model.height 
                                                                            * PELVIS_WIDTH_SCALING_FACTOR* 0.5);
        init_model.heel_right_position.z = 0.0;
 

        // # toe
        init_model.toe_right_position = init_model.heel_right_position;
        init_model.toe_right_position.x += orientation.x * model.height * (FOOT_FORWARD_SCALING_FACTOR + FOOT_BACKWARD_SCALING_FACTOR);
        init_model.toe_right_position.y += orientation.y * model.height * (FOOT_FORWARD_SCALING_FACTOR + FOOT_BACKWARD_SCALING_FACTOR);

        // left foot
        init_model.heel_left_position.x = (ped.pos.x + normal_orientation.x * model.height 
                                                                            * PELVIS_WIDTH_SCALING_FACTOR * 0.5);
        init_model.heel_left_position.y = (ped.pos.y + normal_orientation.y* model.height 
                                                                            * PELVIS_WIDTH_SCALING_FACTOR* 0.5);
        init_model.heel_left_position.z = 0.0;
        // # toe
        init_model.toe_left_position = init_model.heel_left_position;
        init_model.toe_left_position.x += orientation.x * model.height * (FOOT_FORWARD_SCALING_FACTOR + FOOT_BACKWARD_SCALING_FACTOR);
        init_model.toe_left_position.y += orientation.y * model.height * (FOOT_FORWARD_SCALING_FACTOR + FOOT_BACKWARD_SCALING_FACTOR);

        // print normal_orientation
        // std::cout << "Normal orientation: " << normal_orientation.x << ", " << normal_orientation.y << std::endl;

 
        // initial step index 
        // the Closest foot in the agent's orientation is considered to be the folmer stepping foot
        if ((init_model.heel_right_position.To2D() - init_model.pelvis_position.To2D()).ScalarProduct(update.velocity) < ((init_model.heel_left_position.To2D() - init_model.pelvis_position.To2D()).ScalarProduct(update.velocity)) )
        {
            init_model.stepping_foot_index = 1; // left foot support, right foot stepping
        }
        else
        {
            init_model.stepping_foot_index = -1; // right foot support, left foot stepping
        }
        //###########################################################



        // compute update based on initialized state
        update_gait_motion = ComputeMotionHof2008(init_model, update, dT); 

        
        

        }
    else
        {
        // compute update based on previous state and physical_interaction_flag
        // if physical_interaction_flag ist true the motion is governed by the navigation 
        // else the gait motion is computed based on Hof 2008. locomotion model.
        if(physical_interaction_flag)
        {
            update_gait_motion = ComputeMotionPhysicalInteraction(model, update, ped, dT);
        }
        else
        {
            update_gait_motion = ComputeMotionHof2008(model, update, dT);
        }
        
        }


    // apply the update 
    update = update_gait_motion;

    update.position = update_gait_motion.pelvis_position.To2D(); // update the position of the agent to the pelvis position



 
    return update;
}

void HumanoidModelV0::ApplyUpdate(const OperationalModelUpdate& update, GenericAgent& agent) const
{
    auto& model = std::get<HumanoidModelV0Data>(agent.model);
    const auto& upd = std::get<HumanoidModelV0Update>(update);

    // update the SFM navigation
    agent.pos = upd.position;
    model.velocity = upd.velocity;
    agent.orientation = upd.velocity.Normalized();
    // update the Humanoid model
    // # gait variables
    model.step_duration = upd.step_duration;
    model.step_timer = upd.step_timer;
    model.stepping_foot_index = upd.stepping_foot_index;

    model.Xcom= upd.Xcom;


    // # body motion variables
    model.head_position = upd.head_position; 
    model.pelvis_position = upd.pelvis_position;
    model.pelvis_rotation_angle_z = upd.pelvis_rotation_angle_z;
    model.shoulder_rotation_angle_z = upd.shoulder_rotation_angle_z;
    model.trunk_rotation_angle_x = upd.trunk_rotation_angle_x;
    model.trunk_rotation_angle_y = upd.trunk_rotation_angle_y;
    model.heel_right_position = upd.heel_right_position;
    model.heel_left_position = upd.heel_left_position;
    model.toe_right_position = upd.toe_right_position;
    model.toe_left_position = upd.toe_left_position;
}



void HumanoidModelV0::CheckModelConstraint(
    const GenericAgent& agent,
    const NeighborhoodSearchType& neighborhoodSearch,
    const CollisionGeometry& geometry) const
{
    // none of these constraint are given by the paper but are useful to create a simulation that
    // does not break immediately
    auto throwIfNegative = [](double value, std::string name) {
        if(value < 0) {
            throw SimulationError(
                "Model constraint violation: {} {} not in allowed range, "
                "{} needs to be positive",
                name,
                value,
                name);
        }
    };

    const auto& model = std::get<HumanoidModelV0Data>(agent.model);

    const auto mass = model.mass;
    throwIfNegative(mass, "mass");

    const auto desiredSpeed = model.desiredSpeed;
    throwIfNegative(desiredSpeed, "desired speed");

    const auto reactionTime = model.reactionTime;
    throwIfNegative(reactionTime, "reaction time");

    const auto radius = model.radius;
    throwIfNegative(radius, "radius");

    const auto neighbors = neighborhoodSearch.GetNeighboringAgents(agent.pos, 2);
    for(const auto& neighbor : neighbors) {
        const auto distance = (agent.pos - neighbor.pos).Norm();

        if(model.radius >= distance) {
            throw SimulationError(
                "Model constraint violation: Agent {} too close to agent {}: distance {}, "
                "radius {}",
                agent.pos,
                neighbor.pos,
                distance,
                model.radius);
        }
    }
    const auto maxRadius = model.radius / 2;
    const auto lineSegments = geometry.LineSegmentsInDistanceTo(maxRadius, agent.pos);
    if(std::begin(lineSegments) != std::end(lineSegments)) {
        throw SimulationError(
            "Model constraint violation: Agent {} too close to geometry boundaries, distance <= "
            "{}/2",
            agent.pos,
            model.radius);
    }
}

Point HumanoidModelV0::DrivingForce(const GenericAgent& agent)
{
    const auto& model = std::get<HumanoidModelV0Data>(agent.model);
    const Point e0 = (agent.destination - agent.pos).Normalized();
    return (e0 * model.desiredSpeed - model.velocity) / model.reactionTime;
};
double HumanoidModelV0::PushingForceLength(double A, double B, double r, double distance)
{
    return A * exp((r - distance) / B);
}

Point HumanoidModelV0::AgentForce(const GenericAgent& ped1, const GenericAgent& ped2) const
{
    const auto& model1 = std::get<HumanoidModelV0Data>(ped1.model);
    const auto& model2 = std::get<HumanoidModelV0Data>(ped2.model);

    const double total_radius = model1.radius + model2.radius;

    return ForceBetweenPoints(
        ped1.pos,
        ped2.pos,
        model1.agentScale,
        model1.forceDistance,
        total_radius,
        model2.velocity - model1.velocity);
};

Point HumanoidModelV0::ObstacleForce(const GenericAgent& agent, const LineSegment& segment) const
{
    const auto& model = std::get<HumanoidModelV0Data>(agent.model);
    const Point pt = segment.ShortestPoint(agent.pos);
    return ForceBetweenPoints(
        agent.pos, pt, model.obstacleScale, model.forceDistance, model.radius, model.velocity);
}

Point HumanoidModelV0::ForceBetweenPoints(
    const Point pt1,
    const Point pt2,
    const double A,
    const double B,
    const double radius,
    const Point velocity) const
{
    // todo reduce range of force to 180 degrees
    const double dist = (pt1 - pt2).Norm();
    double pushing_force_length = PushingForceLength(A, B, radius, dist) ; // if this is 0 agents bumps into each other 
    double friction_force_length = 0;
    const Point n_ij = (pt1 - pt2).Normalized();
    const Point tangent = n_ij.Rotate90Deg();
    if(dist < radius) {
        pushing_force_length += this->bodyForce * (radius - dist);
        friction_force_length = this->friction * (radius - dist) * (velocity.ScalarProduct(tangent));
    }
    return n_ij * pushing_force_length + tangent * friction_force_length;
}


/***************** Function For Humanoid motion *****************/


HumanoidModelV0Update HumanoidModelV0::ComputeMotionHof2008(
                                            const HumanoidModelV0Data& model,
                                            const HumanoidModelV0Update& update,
                                            double dT
                                    ) const

{



    //copy current updare pointer values
    HumanoidModelV0Update update_gait_motion = update;

    // parameters (have to be added with the other parameters of the model)
    double sc = 0.75; // model.height * 0.42; // prefered step length (0.75 in the paper) (To Do: controle this with spreferend speed)
    double wc = model.height * PELVIS_WIDTH_SCALING_FACTOR; // prefered strid width (0.1 in the paper)
    double Tc = 0.5; // prefered step duration
    double w0 = std::sqrt(9.81 / (model.height * (LEG_SCALING_FACTOR + ANKLE_SCALING_FACTOR)));
    double bx = sc / (std::exp(w0 * Tc) - 1); // step length offset
    double by = wc / (std::exp(w0 * Tc) + 1); // step width offset
    double k1 = 0.1; // control gain for control feedback loop

    // Precomputation
    double step_width = by * (std::exp(w0 * Tc) + 1 ) / (1 - k1*0.5*(std::exp(w0 * Tc) - 1 ));
    double step_length = update_gait_motion.velocity.Norm() * Tc; // prefered step length;
    if (step_length > model.height*0.5) // Limit step length
    {
        step_length = model.height*0.5;
        std::cout << "## step length limit##" << std::endl;
    }
    update_gait_motion.step_timer = model.step_timer;

    // Walking orientation
    // make sure the agent do not turn more than 90 degrees in one step
    if (model.velocity.ScalarProduct(update.velocity) < 0)
    {
        std::cout << "## rotation regulation ##" << std::endl;
        update_gait_motion.step_timer = 0; // reset step timer to force a new step
        if (model.velocity.Rotate90Deg().ScalarProduct(update.velocity) > 0)
        {
            // if the agent is turning more than 90 degrees, set the velocity to the normal direction
            update_gait_motion.velocity = model.velocity.Rotate90Deg().Normalized() * update.velocity.Norm();
        }
        else{
            update_gait_motion.velocity = model.velocity.Rotate90Deg().Normalized() * - update.velocity.Norm();
        }
    }

    Point orientation = update_gait_motion.velocity.Normalized(); // To do: make sure the agent do not turn more than 90 degrees in one step
    Point normal_orientation = orientation.Rotate90Deg();
    Point BoS_center; // position of the Base of Support (center of the support feet)


    // if step_timer == 0: compute the next step
    if (update_gait_motion.step_timer == 0)
    {
        // switch support foot
        // update_gait_motion.stepping_foot_index = -1 * model.stepping_foot_index;// 1 == left foot swinging; -1 == right foot swinging
        if (model.stepping_foot_index == 1)
        {update_gait_motion.stepping_foot_index = -1;}
        else 
        {update_gait_motion.stepping_foot_index = 1;}

        // compute next Xcom position

        update_gait_motion.Xcom =   model.Xcom 
                                    + orientation * step_length 
                                    - normal_orientation * step_width * update_gait_motion.stepping_foot_index;

        // The CoP (approximated here by the support heel) is change intantaneously
        Point CoP = update_gait_motion.Xcom 
                    - orientation * bx
                    - normal_orientation * by * update_gait_motion.stepping_foot_index ;

        // compute the next step duration
        update_gait_motion.step_duration = static_cast<int>(
        std::round(
            (1/(w0*dT))
            * std::log( (step_length/ (update_gait_motion.Xcom - CoP).ScalarProduct(orientation)) + 1)
        )
        );
        // set step timer
        update_gait_motion.step_timer = update_gait_motion.step_duration;



        // move the support foot to the CoP
        if(update_gait_motion.stepping_foot_index == 1) // right foot support, left foot stepping
        {
            update_gait_motion.heel_right_position.x = CoP.x - orientation.x * model.height * FOOT_BACKWARD_SCALING_FACTOR;
            update_gait_motion.heel_right_position.y = CoP.y - orientation.y * model.height * FOOT_BACKWARD_SCALING_FACTOR;
            update_gait_motion.heel_right_position.z = 0.0;

            update_gait_motion.toe_right_position.x = CoP.x + orientation.x * model.height * FOOT_FORWARD_SCALING_FACTOR ;
            update_gait_motion.toe_right_position.y = CoP.y + orientation.y * model.height * FOOT_FORWARD_SCALING_FACTOR ;
            update_gait_motion.toe_right_position.z = 0.0;

            // new BoS
            BoS_center = update_gait_motion.toe_right_position.To2D();// = (update_gait_motion.heel_right_position.To2D() + update_gait_motion.toe_right_position.To2D()) * 0.5;

            // pass on swinging foot position
            update_gait_motion.heel_left_position = model.heel_left_position;
            update_gait_motion.toe_left_position = model.toe_left_position;
        }
        else if (update_gait_motion.stepping_foot_index == -1) // left foot support, right foot stepping
        {
            update_gait_motion.heel_left_position.x = CoP.x - orientation.x * model.height * FOOT_BACKWARD_SCALING_FACTOR;
            update_gait_motion.heel_left_position.y = CoP.y - orientation.y * model.height * FOOT_BACKWARD_SCALING_FACTOR;
            update_gait_motion.heel_left_position.z = 0.0;

            update_gait_motion.toe_left_position.x = CoP.x + orientation.x * model.height * FOOT_FORWARD_SCALING_FACTOR;
            update_gait_motion.toe_left_position.y = CoP.y + orientation.y * model.height * FOOT_FORWARD_SCALING_FACTOR;
            update_gait_motion.toe_left_position.z = 0.0;

            // new BoS
            BoS_center = update_gait_motion.toe_left_position.To2D(); // = (update_gait_motion.heel_left_position.To2D() + update_gait_motion.toe_left_position.To2D()) * 0.5;

            // pass on swinging foot position
            update_gait_motion.heel_right_position = model.heel_right_position;
            update_gait_motion.toe_right_position = model.toe_right_position;
        }
        
    }
    else{
        update_gait_motion.step_timer -= 1; // decrement step timer
        // pass on the current step variables
        update_gait_motion.step_duration = model.step_duration;
        update_gait_motion.stepping_foot_index = model.stepping_foot_index;
        update_gait_motion.Xcom = model.Xcom;

        double step_completion_factor = 1.0 - (static_cast<double>(update_gait_motion.step_timer) / update_gait_motion.step_duration);

        Point swinging_foot_displacement; 
        

        if(update_gait_motion.stepping_foot_index == 1) // right foot support, left foot stepping
        {
            
            swinging_foot_displacement = (
                                            update_gait_motion.Xcom - model.heel_left_position.To2D()
                                            + orientation * (step_length - bx - model.height * FOOT_BACKWARD_SCALING_FACTOR) 
                                            + normal_orientation * (step_width )
                                        )
                                        * (1/static_cast<double>(model.step_timer));
            update_gait_motion.heel_left_position.x = model.heel_left_position.x + swinging_foot_displacement.x;
            update_gait_motion.heel_left_position.y = model.heel_left_position.y + swinging_foot_displacement.y;
            update_gait_motion.heel_left_position.z = -4*0.25*step_completion_factor*(step_completion_factor-1);

            update_gait_motion.toe_left_position.x = update_gait_motion.heel_left_position.x 
                                                    + orientation.x * model.height * (FOOT_FORWARD_SCALING_FACTOR + FOOT_BACKWARD_SCALING_FACTOR);
            update_gait_motion.toe_left_position.y = update_gait_motion.heel_left_position.y 
                                                    + orientation.y * model.height * (FOOT_FORWARD_SCALING_FACTOR + FOOT_BACKWARD_SCALING_FACTOR);
            update_gait_motion.toe_left_position.z = -4*0.25*step_completion_factor*(step_completion_factor-1);

            // update suport foot position
            update_gait_motion.heel_right_position = model.heel_right_position;
            update_gait_motion.toe_right_position = model.toe_right_position;
            BoS_center = update_gait_motion.toe_right_position.To2D();// = (update_gait_motion.heel_right_position.To2D() + update_gait_motion.toe_right_position.To2D()) * 0.5;
        }
        else if (update_gait_motion.stepping_foot_index == -1) // left foot support, right foot stepping
        {
            swinging_foot_displacement = (
                                            update_gait_motion.Xcom - model.heel_right_position.To2D()
                                            + orientation * (step_length - bx - model.height * FOOT_BACKWARD_SCALING_FACTOR) 
                                            - normal_orientation * (step_width ) 
                                        )
                                        * (1/static_cast<double>(model.step_timer));
            update_gait_motion.heel_right_position.x = model.heel_right_position.x + swinging_foot_displacement.x;
            update_gait_motion.heel_right_position.y = model.heel_right_position.y + swinging_foot_displacement.y;
            update_gait_motion.heel_right_position.z = -4*0.25*step_completion_factor*(step_completion_factor-1);    

            update_gait_motion.toe_right_position.x = update_gait_motion.heel_right_position.x 
                                                    + orientation.x * model.height * (FOOT_FORWARD_SCALING_FACTOR + FOOT_BACKWARD_SCALING_FACTOR);
            update_gait_motion.toe_right_position.y = update_gait_motion.heel_right_position.y 
                                                    + orientation.y * model.height * (FOOT_FORWARD_SCALING_FACTOR + FOOT_BACKWARD_SCALING_FACTOR);
            update_gait_motion.toe_right_position.z = -4*0.25*step_completion_factor*(step_completion_factor-1);

            // update support foot position
            update_gait_motion.heel_left_position = model.heel_left_position;
            update_gait_motion.toe_left_position = model.toe_left_position;
            BoS_center = update_gait_motion.toe_left_position.To2D();// = (update_gait_motion.heel_left_position.To2D() + update_gait_motion.toe_left_position.To2D()) * 0.5;
        }
        // support foot (CoP) stays in position, the other foot moves toward the potential best next step position
    }

    // update pelvis position
    // the pelvis moves towards the Base of support (center of the feet)
    // Point CoM_2d = (model.pelvis_position.To2D() + update_gait_motion.Xcom * dT * w0) / (1 + dT * w0);
    Point CoM_displacement =  (update_gait_motion.Xcom - model.pelvis_position.To2D()) * w0 * dT;
    update_gait_motion.pelvis_position.x = model.pelvis_position.x + CoM_displacement.x;
    update_gait_motion.pelvis_position.y = model.pelvis_position.y + CoM_displacement.y;
    update_gait_motion.pelvis_position.z = model.height * ( LEG_SCALING_FACTOR + ANKLE_SCALING_FACTOR);

    // agent update position is set to the pelvis position
    update_gait_motion.position = update_gait_motion.pelvis_position.To2D();
    
    //  update head position
    update_gait_motion.head_position = update_gait_motion.pelvis_position;
    update_gait_motion.head_position.z = update_gait_motion.pelvis_position.z
                                                + model.height * (TRUNK_HEIGHT_SCALING_FACTOR 
                                                                + NECK_SCALING_FACTOR);


    // pelvis rotation
    // following the line in between both feet
    Point in_between_feet_vector = update_gait_motion.heel_right_position.To2D()
                                    - update_gait_motion.heel_left_position.To2D();
    // compute the angle of the pelvis rotation (between 0 an 2*pi)
    if (in_between_feet_vector.y >= 0) 
    {
        update_gait_motion.pelvis_rotation_angle_z = std::atan2(in_between_feet_vector.y, in_between_feet_vector.x);
        // update_gait_motion.shoulder_rotation_angle_z = M_PI - update_gait_motion.pelvis_rotation_angle_z; // shoulder rotation is opposite to pelvis rotation
    }
    else 
    {
        update_gait_motion.pelvis_rotation_angle_z = std::atan2(in_between_feet_vector.y, in_between_feet_vector.x) + 2 * M_PI;
        // update_gait_motion.shoulder_rotation_angle_z = M_PI - update_gait_motion.pelvis_rotation_angle_z; // shoulder rotation is opposite to pelvis rotation
    }

    // shoulder rotation
    // the shoulder rotation is perpendicular to the agent orientation
    if (orientation.y >= 0)
    {
        update_gait_motion.shoulder_rotation_angle_z = std::atan2(normal_orientation.y, normal_orientation.x);
    }
    else 
    {
        update_gait_motion.shoulder_rotation_angle_z = std::atan2(normal_orientation.y, normal_orientation.x) + 2 * M_PI;
    }

    // //#############
    // // std::cout << " "<< std::endl;

    // Point swinging_foot_displacement = (
    //                                             orientation * (step_length - bx) 
    //                                             + normal_orientation * (step_width + by * update_gait_motion.stepping_foot_index)
    //                                         )
    //                                         * (2/static_cast<double>(model.step_duration));

    // plot. everything:
    // Point CoP = update_gait_motion.Xcom 
    //             - orientation * bx
    //             - normal_orientation * by * update_gait_motion.stepping_foot_index ;
    // std::cout << "Actual Step duration: " << (1/w0)
    //     * std::log( (step_length/ (update_gait_motion.Xcom - CoP).ScalarProduct(orientation)) + 1) << std::endl;
    // std::cout << "Step duration: " << update_gait_motion.step_duration << std::endl;
    // std::cout << "Step timer: " << update_gait_motion.step_timer << std::endl;
    // std::cout << " Step length: " << step_length << std::endl;
    // std::cout << "Stepping foot index: " << update_gait_motion.stepping_foot_index << std::endl;
    // std::cout << "Xcom: " << update_gait_motion.Xcom.x << ", " 
    //             << update_gait_motion.Xcom.y << std::endl;   
    // std::cout << "Previous pelvis position: " << model.pelvis_position.x << ", " 
    // << model.pelvis_position.y << std::endl; 
    // std::cout << "Updated pelvis position: " << update_gait_motion.pelvis_position.x << ", " 
    //             << update_gait_motion.pelvis_position.y << std::endl;    
    // std::cout << "Heel right position: " << update_gait_motion.heel_right_position.x << ", " 
    //             << update_gait_motion.heel_right_position.y << std::endl;
    // std::cout << "Heel left position: " << update_gait_motion.heel_left_position.x << ", " 
    //             << update_gait_motion.heel_left_position.y << std::endl;

    // std::cin.get();
// //#############

    // to do:   
        // - prevent step through walls


    

    return  update_gait_motion;

}


HumanoidModelV0Update HumanoidModelV0::ComputeMotionPhysicalInteraction(
                                            const HumanoidModelV0Data& model,
                                            const HumanoidModelV0Update& update,
                                            const GenericAgent& agent,
                                            double dT
                                    ) const

{
    //copy current updare pointer values
    HumanoidModelV0Update update_gait_motion = update;

    // here the swinging foot will follows as much as possible the Xcom position 
    // with an offset in the normal direction to account for step width
    // the shoulder will turn to alligne with the agent derection and not with the nav velocity 
    // the support foot will stay in place

    // parameters (have to be added with the other parameters of the model)
    double sc = 0.5; // model.height * 0.42; // prefered step length (0.75 in the paper) (To Do: controle this with spreferend speed)
    double s_max = 0.75; // maximum step length
    double wc = model.height * PELVIS_WIDTH_SCALING_FACTOR; // prefered strid width (0.1 in the paper)
    double Tc = 0.5; // prefered step duration
    double w0 = std::sqrt(9.81 / (model.height * (LEG_SCALING_FACTOR + ANKLE_SCALING_FACTOR)));
    double bx = sc / (std::exp(w0 * Tc) - 1); // step length offset
    double by = wc / (std::exp(w0 * Tc) + 1); // step width offset
    // double k1 = 0.1; // control gain for control feedback loop

    // update Pelvis (CoM) an Xcom positions
    update_gait_motion.pelvis_position.x = model.pelvis_position.x + update_gait_motion.velocity.x * dT;
    update_gait_motion.pelvis_position.y = model.pelvis_position.y + update_gait_motion.velocity.y * dT;
    update_gait_motion.pelvis_position.z = model.pelvis_position.z; 

    update_gait_motion.Xcom = update_gait_motion.pelvis_position.To2D() + update_gait_motion.velocity / w0;


    // compute step timming
    update_gait_motion.step_timer = model.step_timer; 

    // Make sure the step is not too large
    // if the future step will be too far, reset step_timer to force new step
    /////////// !!!!!!!!!! MAKE SURE THE AGENT DO NOT CHAGE STEPPING STATE ALL THE TIME 
    if (model.stepping_foot_index == 1) // right foot support, left foot stepping
    {
        if ((update_gait_motion.Xcom - model.heel_right_position.To2D()).Norm() > s_max)
        {
            update_gait_motion.step_timer = 0; // reset step timer to force new step
        }
    }
    else if (model.stepping_foot_index == -1) // left foot support, right foot stepping
    {
        if ((update_gait_motion.Xcom - model.heel_left_position.To2D()).Norm() > s_max)
        {
            update_gait_motion.step_timer = 0; // reset step timer to force new step
        }
    }

    if (update_gait_motion.step_timer == 0)
    {
        // switch support foot
        if (model.stepping_foot_index == 1)
        {update_gait_motion.stepping_foot_index = -1;}
        else 
        {update_gait_motion.stepping_foot_index = 1;}

        // compute next step duration
        update_gait_motion.step_duration = static_cast<int>(
            std::round(
                (1/(w0*dT))
                * std::log( (sc/ (update_gait_motion.Xcom - update_gait_motion.pelvis_position.To2D()).ScalarProduct(update_gait_motion.velocity.Normalized())) + 1)
            )
        );
        // set step timer
        update_gait_motion.step_timer = update_gait_motion.step_duration;

    }
    else{
        update_gait_motion.step_timer -= 1; // decrement step timer
            // pass on the current step variables
        update_gait_motion.step_duration = model.step_duration;
        update_gait_motion.stepping_foot_index = model.stepping_foot_index;
    }

    // compute the next feet position
    Point orientation = update_gait_motion.velocity.Normalized(); // To do: make sure the agent do not turn more than 90 degrees in one step
    Point normal_orientation = orientation.Rotate90Deg();
    double step_completion_factor = 1.0 - (static_cast<double>(update_gait_motion.step_timer) / update_gait_motion.step_duration);

    if (update_gait_motion.stepping_foot_index == 1) // right foot support, left foot stepping
    {
        // toe position
        update_gait_motion.toe_left_position.x = model.toe_left_position.x 
                                                    + (
                                                        update_gait_motion.Xcom.x - model.toe_left_position.x
                                                        + orientation.x * (model.height * FOOT_FORWARD_SCALING_FACTOR - bx)
                                                        + normal_orientation.x * by
                                                    )/ (update_gait_motion.step_timer + 1);
        update_gait_motion.toe_left_position.y = model.toe_left_position.y 
                                                 + (
                                                        update_gait_motion.Xcom.y - model.toe_left_position.y
                                                        + orientation.y * (model.height * FOOT_FORWARD_SCALING_FACTOR - bx)
                                                        + normal_orientation.y * by
                                                    )/ (update_gait_motion.step_timer + 1);
        update_gait_motion.toe_left_position.z = -4*0.25*step_completion_factor*(step_completion_factor-1);

        // heel position
        update_gait_motion.heel_left_position = update_gait_motion.toe_left_position;
        update_gait_motion.heel_left_position.x -= orientation.x * model.height * (FOOT_FORWARD_SCALING_FACTOR + FOOT_BACKWARD_SCALING_FACTOR);
        update_gait_motion.heel_left_position.y -= orientation.y * model.height * (FOOT_FORWARD_SCALING_FACTOR + FOOT_BACKWARD_SCALING_FACTOR);

        // pass on swinging foot position
        update_gait_motion.heel_right_position = model.heel_right_position;
        update_gait_motion.toe_right_position = model.toe_right_position;
    }
    else if (update_gait_motion.stepping_foot_index == -1) // left foot support, right foot stepping
    {
        // toe position
        update_gait_motion.toe_right_position.x = model.toe_right_position.x 
                                                    + (
                                                            update_gait_motion.Xcom.x - model.toe_right_position.x
                                                            + orientation.x * (model.height * FOOT_FORWARD_SCALING_FACTOR - bx)
                                                            - normal_orientation.x * by
                                                        )/ (update_gait_motion.step_timer + 1);
                                                
        update_gait_motion.toe_right_position.y = model.toe_right_position.y 
                                                    + (
                                                            update_gait_motion.Xcom.y - model.toe_right_position.y
                                                            + orientation.y * (model.height * FOOT_FORWARD_SCALING_FACTOR - bx)
                                                            - normal_orientation.y * by
                                                        )/ (update_gait_motion.step_timer + 1);
                                                 
        update_gait_motion.toe_right_position.z = -4*0.25*step_completion_factor*(step_completion_factor-1);

        // heel position
        update_gait_motion.heel_right_position = update_gait_motion.toe_right_position;
        update_gait_motion.heel_right_position.x -= orientation.x * model.height * (FOOT_FORWARD_SCALING_FACTOR + FOOT_BACKWARD_SCALING_FACTOR);
        update_gait_motion.heel_right_position.y -= orientation.y * model.height * (FOOT_FORWARD_SCALING_FACTOR + FOOT_BACKWARD_SCALING_FACTOR);


        // pass on swinging foot position
        update_gait_motion.heel_left_position = model.heel_left_position;
        update_gait_motion.toe_left_position = model.toe_left_position;

        //  update head position
        update_gait_motion.head_position = update_gait_motion.pelvis_position;
        update_gait_motion.head_position.z = update_gait_motion.pelvis_position.z 
                                                    + model.height * (TRUNK_HEIGHT_SCALING_FACTOR 
                                                                    + NECK_SCALING_FACTOR);


    }

    // update shoulder rotation
    // the shoulder rotation is perpendicular to the agent orientation
    Point shoulder_orientation = agent.destination.Rotate90Deg().Normalized();
    if (orientation.y >= 0)
    {
        update_gait_motion.shoulder_rotation_angle_z = std::atan2(shoulder_orientation.y, shoulder_orientation.x);
    }
    else 
    {
        update_gait_motion.shoulder_rotation_angle_z = std::atan2(shoulder_orientation.y, shoulder_orientation.x) + 2 * M_PI;
    }

    // printf("Velocity: %f, %f\n", update_gait_motion.velocity.x, update_gait_motion.velocity.y);
    // printf("Xcom: %f, %f\n", update_gait_motion.Xcom.x, update_gait_motion.Xcom.y);
    // printf("Pelvis position: %f, %f\n", update_gait_motion.pelvis_position.x, update_gait_motion.pelvis_position.y);
    // printf("Stepping foot index: %d\n", update_gait_motion.stepping_foot_index);
    // printf("Step duration: %d\n", update_gait_motion.step_duration);
    // printf("Step timer: %d\n", update_gait_motion.step_timer);
    // printf("Heel left position: %f, %f\n", update_gait_motion.heel_left_position.x, update_gait_motion.heel_left_position.y);
    // printf("Heel right position: %f, %f\n", update_gait_motion.heel_right_position.x, update_gait_motion.heel_right_position.y);

    // std::cin.get();

    return  update_gait_motion;

    // to investigate: - why are the feet shot backard ?
 
}