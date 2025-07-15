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
#include <Eigen/Dense>

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
        // head
        init_model.head_position.x = ped.pos.x;
        init_model.head_position.y = ped.pos.y;
        init_model.head_position.z = model.height * (  ANKLE_SCALING_FACTOR
                                                        + LEG_SCALING_FACTOR 
                                                        + TRUNK_HEIGHT_SCALING_FACTOR
                                                        + NECK_SCALING_FACTOR);
        // pelvis
        init_model.pelvis_position.x = ped.pos.x;
        init_model.pelvis_position.y = ped.pos.y;
        init_model.pelvis_position.z = model.height * ( LEG_SCALING_FACTOR + ANKLE_SCALING_FACTOR);

        Point normal_orientation = update.velocity.Normalized().Rotate90Deg();
        // right foot
        init_model.heel_right_position.x = (ped.pos - normal_orientation*( model.height 
                                                                            * PELVIS_WIDTH_SCALING_FACTOR * 0.5)).x;
        init_model.heel_right_position.y = (ped.pos - normal_orientation*( model.height 
                                                                            * PELVIS_WIDTH_SCALING_FACTOR* 0.5)).y;
        init_model.heel_right_position.z = 0.0;

        // left foot
        init_model.heel_left_position.x = (ped.pos + normal_orientation*(  model.height 
                                                                            * PELVIS_WIDTH_SCALING_FACTOR* 0.5)).x;
        init_model.heel_left_position.y = (ped.pos + normal_orientation*(  model.height 
                                                                            * PELVIS_WIDTH_SCALING_FACTOR* 0.5)).y;
        init_model.heel_left_position.z = 0.0;

        // initial step index 
        // the furthest foot in the agent's orientation is the stepping foot

        if ((init_model.heel_right_position.To2D() - init_model.pelvis_position.To2D()).ScalarProduct(update.velocity) > ((init_model.heel_left_position.To2D() - init_model.pelvis_position.To2D()).ScalarProduct(update.velocity)) )
        {
            init_model.stepping_foot_index = 1; // left foot support, right foot stepping
        }
        else
        {
            init_model.stepping_foot_index = -1; // right foot support, left foot stepping
        }
        //###########################################################



        // compute update based on initialized state
        update_gait_motion = ComputeGaitMotion(init_model, update, dT); 
        

        }
    else
        {
        // compute update based on previous state
        update_gait_motion = ComputeGaitMotion(model, update, dT);
        }


    // apply the update 
    update = update_gait_motion;



  
    // print the updated joint positions

    // // // std::cout << "Pelvis: " << update.position.x <<", "<< update.position.y << std::endl;
    // // std::cout << "Right Heel: " << update.heel_right_position.x << ", " 
    //                             << update.heel_right_position.y << ", " 
    //                             << update.heel_right_position.z << std::endl;
    // // std::cout << "Left Heel: "  << update.heel_left_position.x << ", " 
    //                             << update.heel_left_position.y << ", " 
    //                             << update.heel_left_position.z << std::endl;

    // // // pause
    // std::cin.get();  
    // // clear the system console
    // std::system("clear");

 
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

    model.step_target= upd.step_target;


    // # body motion variables
    model.head_position = upd.head_position; 
    model.pelvis_position = upd.pelvis_position;
    model.pelvis_rotation_angle_z = upd.pelvis_rotation_angle_z;
    model.shoulder_rotation_angle_z = upd.shoulder_rotation_angle_z;
    model.trunk_rotation_angle_x = upd.trunk_rotation_angle_x;
    model.trunk_rotation_angle_y = upd.trunk_rotation_angle_y;
    model.heel_right_position = upd.heel_right_position;
    model.heel_left_position = upd.heel_left_position;
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
    double pushing_force_length = PushingForceLength(A, B, radius, dist) * 0; // if this is 0 agents bumps into each other 
    double friction_force_length = 0;
    const Point n_ij = (pt1 - pt2).Normalized();
    const Point tangent = n_ij.Rotate90Deg();
    if(dist < radius) {
        pushing_force_length += this->bodyForce * (radius - dist);
        friction_force_length =
            this->friction * (radius - dist) * (velocity.ScalarProduct(tangent));
    }
    return n_ij * pushing_force_length + tangent * friction_force_length;
}


/***************** Function For Humanoid motion *****************/

/*** Matrix opperators using Eigen ***/
// Denavit-Hartenberg (DH) convention
// compute T^{n-1}_n
Eigen::Matrix4d HumanoidModelV0::DHTransformationMatrix(
    double theta, 
    double d, 
    double r, 
    double alpha) const
    {
    Eigen::Matrix4d mat;
    mat << cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), r*cos(theta),
            sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), r*sin(theta),
            0, sin(alpha), cos(alpha), d,
            0, 0, 0, 1;
    return mat;
}

/*** rotation matrix  ***/
 

Eigen::Matrix4d HumanoidModelV0::CreateTransformationMatrix(
    const Eigen::Vector3d &translation,
    const Eigen::Vector3d &rotation
) const {
    
    double roll = rotation[0], pitch = rotation[1], yaw = rotation[2];
    
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
    
    Eigen::Quaterniond MergedRotations = yawAngle * pitchAngle * rollAngle;
    Eigen::Matrix3d RotationMatrix = MergedRotations.toRotationMatrix();

    // Create transformation matrix
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block(0,0,3,3) = RotationMatrix;  // Or use rotationQuaternionsMethod
    T.block(0,3,3,1) = translation;
    
    return T;
}



HumanoidModelV0Update HumanoidModelV0::ComputeGaitMotion(
                                            const HumanoidModelV0Data& model,
                                            const HumanoidModelV0Update& update,
                                            double dT
                                    ) const

{
    //copy current updare pointer values
    HumanoidModelV0Update update_gait_motion = update;



    //#### Step timing computation
    if (model.stepping_foot_index != 0) // if the agent is not in a double support phase
    {
        // std::cout << "1" << std::endl;
        if (model.step_timer == 0) // initialisation of gait process
            {
                // std::cout << "2" << std::endl;
            // Compute the next step_duration (int number of time step require to complete the step)
            // constant stepping time of 0.5 for an agent of 1.7m
            update_gait_motion.step_duration = static_cast<int>(std::round((model.height * 0.3 / (1.7 * dT))));
            update_gait_motion.step_timer=update_gait_motion.step_duration;
            // pass on stepping_foot_index
            update_gait_motion.stepping_foot_index = model.stepping_foot_index;

            // Update stepping_foot_index // the update will be done at the end of the double support phase
            // if (model.stepping_foot_index == 1) {
            //     update_gait_motion.stepping_foot_index = -1; // switch to right foot stepping
            // } else {
            //     update_gait_motion.stepping_foot_index = 1;  // switch to left foot stepping
            // }

        } 
        else // leg swinging phase
        {
            // std::cout << "3" << std::endl;
            // pass on stepping_foot_index and step_duration
            update_gait_motion.stepping_foot_index = model.stepping_foot_index;
            update_gait_motion.step_duration = model.step_duration;
            // update step timer
            update_gait_motion.step_timer = model.step_timer - 1;
            // if step timer is 0, the swing phase is over and the agent enter double support phase
            if (update_gait_motion.step_timer == 0) {
                update_gait_motion.stepping_foot_index = 0; // double support phase
                
            }
        }
    }
    else // if the agent is in a double support phase
    {
        // std::cout << "4" << std::endl;
        // pass on the step duration and step timer
        update_gait_motion.stepping_foot_index = model.stepping_foot_index;
        // the double support phase is active as long as the CoM haven't reached the BoS
    }


    //#### Precomputation before updating step motion #####
    double leg_length = model.height * (LEG_SCALING_FACTOR + ANKLE_SCALING_FACTOR); 
    double pelvis_width = model.height * PELVIS_WIDTH_SCALING_FACTOR;

    // compute the natural frequency XCoM pendulum
    // omega_0 = sqrt(g / l) where g is the gravity and l is the leg length
    // this is used to compute the pelvis position 
    double omega_0 = std::sqrt(9.81 / leg_length); 

    // compute step_complition_factor
    // this represent the avancement of the curent step. this factor == 1 when the step is over.
    double step_complition_factor = 1.0 - (static_cast<double>(update_gait_motion.step_timer) / update_gait_motion.step_duration);
    //#######


    //#### Step motion computation depending on the stepping phase (control by stepping_foot_index) 
    if (update_gait_motion.stepping_foot_index == 0) // if the agent is in a double support phase
    {    
        // during the double support phase, the feet are not moving, the pelvis is moving to maintan the "XCoM in BoS" condition

        // compute the position in between the two feet (for now this will be the target withing the BoS)
        Point mid_feet_location = (model.heel_left_position.To2D() + model.heel_right_position.To2D()) * 0.5
                        + update_gait_motion.velocity.Normalized() * model.height * FOOT_FORWARD_SCALING_FACTOR * 0.5; 

         
        if( (model.pelvis_position.To2D() - mid_feet_location).Norm() > model.height * FOOT_FORWARD_SCALING_FACTOR * 0.5)
            // if the pelvis havent reached the BoS yet 
            // then the pelvis moves at Vnav in the direction of the BoS
        {
            // std::cout << "6" << std::endl;
            // std::cout << "mid_feet_location: " << mid_feet_location.x << ", "
            //             << mid_feet_location.y << std::endl;
            // std::cout << "model.heel_left_position.To2D(): " << model.heel_left_position.To2D().x << ", "
                        // << model.heel_left_position.To2D().y << std::endl;
            // std::cout << "model.heel_right_position.To2D(): " << model.heel_right_position.To2D().x << ", "
                        // << model.heel_right_position.To2D().y << std::endl;
            // std::cout << "model.pelvis_position.To2D(): " << model.pelvis_position.To2D().x << ", "
                        // << model.pelvis_position.To2D().y << std::endl;
            
            // the pelvis moves towards the BoS
            update_gait_motion.pelvis_position.x =  model.pelvis_position.x 
                                                +(mid_feet_location.x-model.pelvis_position.x)*omega_0* dT;
            update_gait_motion.pelvis_position.y =  model.pelvis_position.y 
                                                +(mid_feet_location.y-model.pelvis_position.y)*omega_0* dT ;
            update_gait_motion.pelvis_position.z = model.pelvis_position.z; // pelvis keeps the same height
        }
        else
        {
            // std::cout << "7" << std::endl;
            // the furthest foot in the agent's orientation is the stepping foot
            if ((model.heel_right_position.To2D() - model.pelvis_position.To2D()).ScalarProduct(update_gait_motion.velocity) 
                    > ((model.heel_left_position.To2D() - model.pelvis_position.To2D()).ScalarProduct(update_gait_motion.velocity)) )
            {
                update_gait_motion.stepping_foot_index = 1; // left foot support, right foot stepping
            }
            else
            {
                update_gait_motion.stepping_foot_index = -1; // right foot support, left foot stepping
            }
            // the pelvis moves towards the new BoS
            if (update_gait_motion.stepping_foot_index == 1) // left foot support, right foot stepping
            {
                update_gait_motion.pelvis_position.x =  model.pelvis_position.x 
                                                +(model.heel_left_position.x-model.pelvis_position.x)*omega_0* dT;
                update_gait_motion.pelvis_position.y =  model.pelvis_position.y 
                                                +(model.heel_left_position.y-model.pelvis_position.y)*omega_0* dT;
            }
            else
            {
                update_gait_motion.pelvis_position.x =  model.pelvis_position.x 
                                                +(model.heel_right_position.x-model.pelvis_position.x)*omega_0* dT;
                update_gait_motion.pelvis_position.y =  model.pelvis_position.y 
                                                +(model.heel_right_position.y-model.pelvis_position.y)*omega_0* dT;
            }
            update_gait_motion.pelvis_position.z = model.pelvis_position.z; // pelvis keeps the same height
        }
        // During the double support phase, the feet are not moving
        update_gait_motion.heel_left_position = model.heel_left_position;
        update_gait_motion.heel_right_position = model.heel_right_position;


    }
    else if(update_gait_motion.stepping_foot_index == 1) // if the right foot is support foot (left foot stepping)
    {
        // std::cout << "8" << std::endl;
        // Step 1: computation of the next stepping foot position
        // left foot stepping
        // the stepping foot travel double the distance that the navigation model predict fot the pelvis
        update_gait_motion.heel_left_position.x = model.heel_left_position.x + update.velocity.x * dT;
        update_gait_motion.heel_left_position.y = model.heel_left_position.y + update.velocity.y * dT;
        update_gait_motion.heel_left_position.z = -0.4*step_complition_factor*(step_complition_factor-1); 
                                                    // the vertical displacement of the stepping foot is 
                                                    // a parabola with a maximum at 0.1m/ z=0.40(t)(t - 1);
                                                     

        // Step 2: computation of the pelvis and support hip position
        // The pelvis position is updated so that the XCoM is on the position of the support foot (locomotion stability creteria) 
        //  - under assumption pervis center = CoM

        update_gait_motion.pelvis_position.x =  model.pelvis_position.x 
                                                +(model.heel_right_position.x-model.pelvis_position.x)*omega_0* dT;
        update_gait_motion.pelvis_position.y =  model.pelvis_position.y 
                                                +(model.heel_right_position.y-model.pelvis_position.y)*omega_0* dT;

        

        // position of the support hip (right hip)
        Point normal_orientation = update.velocity.Normalized().Rotate90Deg();
        Point support_hip_position;

        support_hip_position.x = update_gait_motion.pelvis_position.x - pelvis_width * 0.5 * normal_orientation.x;
        support_hip_position.y = update_gait_motion.pelvis_position.y - pelvis_width * 0.5 * normal_orientation.y;

        double distance_support_hip_foot_xy = (support_hip_position - model.heel_right_position.To2D()).Norm();
        // the height of the pelvis is based on the Leg length and position of the hip relative to the support foot
        // pyhtagore with leg legth and support foot-hip positions
        update_gait_motion.pelvis_position.z =  sqrt(std::pow(leg_length, 2)- std::pow(distance_support_hip_foot_xy,2));

        // Step 3: The support foot (right foot) remain at the same place (on the ground)
        update_gait_motion.heel_right_position = model.heel_right_position;
        update_gait_motion.heel_right_position.z = 0;                               
                                                
    } 
    else if(update_gait_motion.stepping_foot_index == -1)// if the left foot is support foot (right foot stepping)
    {
        // std::cout << "9" << std::endl;
        // Step 1: computation of the next stepping foot position
        // right foot stepping
        // the stepping foot travel double the distance that the navigation model predict fot the pelvis
        update_gait_motion.heel_right_position.x = model.heel_right_position.x + update.velocity.x * dT; 
        update_gait_motion.heel_right_position.y = model.heel_right_position.y + update.velocity.y * dT;
        update_gait_motion.heel_right_position.z = -0.4*step_complition_factor*(step_complition_factor-1); 
                                                    // the vertical displacement of the stepping foot is 
                                                    // a parabola with a maximum at 0.1m/ z=0.40(t)(t - 1);

        
        // Step 2: computation of the pelvis and support hip position
        // The pelvis position is updated so that the XCoM is on the position of the support foot (locomotion stability creteria) 
        //  - under assumption pervis center = CoM
        update_gait_motion.pelvis_position.x =  model.pelvis_position.x 
                                                +(model.heel_left_position.x-model.pelvis_position.x)*omega_0* dT;
        update_gait_motion.pelvis_position.y =  model.pelvis_position.y 
                                                +(model.heel_left_position.y-model.pelvis_position.y)*omega_0* dT;

        // position of the support hip (right hip)
        Point normal_orientation = update.velocity.Normalized().Rotate90Deg();
        Point support_hip_position;

        support_hip_position.x = update_gait_motion.pelvis_position.x + pelvis_width * 0.5 * normal_orientation.x;
        support_hip_position.y = update_gait_motion.pelvis_position.y + pelvis_width * 0.5 * normal_orientation.y;

        double distance_support_hip_foot_xy = (support_hip_position - model.heel_left_position.To2D()).Norm();

        // the height of the pelvis is based on the Leg length and position of the hip relative to the support foot
        // pyhtagore with leg legth and support foot-hip positions
        update_gait_motion.pelvis_position.z =  sqrt(std::pow(leg_length, 2)- std::pow(distance_support_hip_foot_xy,2));

        // Step 3: The support foot (left foot) remain at the same place (on the ground)
        update_gait_motion.heel_left_position = model.heel_left_position;
        update_gait_motion.heel_left_position.z = 0; 

    }

    // Spet 4: update head position
    update_gait_motion.head_position = update_gait_motion.pelvis_position;
    update_gait_motion.head_position.z = update_gait_motion.pelvis_position.z
                                                + model.height * (TRUNK_HEIGHT_SCALING_FACTOR 
                                                                + NECK_SCALING_FACTOR);

    // Step 5: The agent update position is set to the pelvis position
    update_gait_motion.position.x = update_gait_motion.pelvis_position.x;
    update_gait_motion.position.y = update_gait_motion.pelvis_position.y;

    // //#############
    // // std::cout << " "<< std::endl;
    // // std::cout << "update_gait_motion.stepping_foot_index: " << update_gait_motion.stepping_foot_index << std::endl;
    // // std::cout << "(model.heel_left_position.x-model.pelvis_position.x): " << (model.heel_right_position.x-model.pelvis_position.x) << std::endl;
    // // print left heal right heel and pelvis position


    // std::cout<< "update_gait_motion.step_timer: " << update_gait_motion.step_timer << std::endl;
    // std::cout << "update_gait_motion.stepping_foot_index: " << update_gait_motion.stepping_foot_index << std::endl;
    // std::cout << "Pelvis Position: " << update_gait_motion.pelvis_position.x << ", "
                // << update_gait_motion.pelvis_position.y << std::endl;    
    // std::cout << "Left Heel Position: " << model.heel_left_position.x << ", "
                // << model.heel_left_position.y << std::endl;
    // std::cout << "Right Heel Position: " << model.heel_right_position.x << ", "
                // << model.heel_right_position.y << std::endl;
    // std::cin.get();
    // //#############
    



    return  update_gait_motion;
;

}
