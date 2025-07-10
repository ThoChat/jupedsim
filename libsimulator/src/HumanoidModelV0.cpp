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


    /// #### Humanoid model #####

    // initialise the updated joint positions matrix
    Eigen::MatrixXd updated_joint_positions_matrix; 
    updated_joint_positions_matrix = Eigen::MatrixXd::Zero(11, 3); // 11 joints, 3 positions each

    // Rows: joints * 11, Columns: x/y/z rotation, 
    // the referencial of the coordinate in linked to the orientation of the agent
    // x == sagittal, y == frontal, z == vertical (up) axis                    
    // list of all simulated joints
    /**
    0 - right heel
    1 - right ankle
    2 - right hip
    3 - left hip
    4 - left ankle
    5 - left heel
    6 - pelvis/CoM
    7 - right shoulder
    8 - C7 / neck
    9 - left shoulder
    10 - head 
     **/


    // Steps computation
    if (model.step_timer == 0) // double support
    {


        // Compute the next step_duration (int number of time step require to complete the step)
        // constant stepping time of 0.5 for an agent of 1.7m
        update.step_duration = static_cast<int>(std::round((model.height * 0.5 / (1.7 * dT))));
        update.step_timer=update.step_duration;

        // Update stepping_foot_index
        if (model.stepping_foot_index == 1) {
            update.stepping_foot_index = -1; // switch to right foot stepping
        } else {
            update.stepping_foot_index = 1;  // switch to left foot stepping
        }
        ///////////

        // initialise posture 
        if (model.head_position.z <= 0.001) 
        {
        // Create a new instance of HumanoidModelV0Data
        auto init_model = std::make_unique<HumanoidModelV0Data>(model); //copy current model
        // change joint position to match the initial position and direction

        // head
        init_model->head_position.x = ped.pos.x;
        init_model->head_position.y = ped.pos.y;
        init_model->head_position.z = model.height * (  ANKLE_SCALING_FACTOR
                                                        + LEG_SCALING_FACTOR 
                                                        + TRUNK_HEIGHT_SCALING_FACTOR
                                                        + NECK_SCALING_FACTOR);

        Point normal_orientation = update.velocity.Normalized().Rotate90Deg();
        // right foot
        init_model->heel_right_position.x = (ped.pos - normal_orientation*( model.height 
                                                                            * PELVIS_WIDTH_SCALING_FACTOR * 0.5)).x;
        init_model->heel_right_position.y = (ped.pos - normal_orientation*( model.height 
                                                                            * PELVIS_WIDTH_SCALING_FACTOR* 0.5)).y;
        init_model->heel_right_position.z = 0.0;

        // left foot
        init_model->heel_left_position.x = (ped.pos + normal_orientation*(  model.height 
                                                                            * PELVIS_WIDTH_SCALING_FACTOR* 0.5)).x;
        init_model->heel_left_position.y = (ped.pos + normal_orientation*(  model.height 
                                                                            * PELVIS_WIDTH_SCALING_FACTOR* 0.5)).y;
        init_model->heel_left_position.z = 0.0;

        // compute update based on initialized state
        updated_joint_positions_matrix = ComputeJointPositionGait(*init_model, update, dT); 
        
         
        }
        else
        {
        // compute update based on previous state
        updated_joint_positions_matrix = ComputeJointPositionGait(model, update, dT);
        }
        

        
    } 
    else // single support
    {
        
        // pass on stepping_foot_index and step_duration
        update.stepping_foot_index = model.stepping_foot_index;
        update.step_duration = model.step_duration;
        // update step timer
        update.step_timer = model.step_timer - 1;
        ///////////


        // New Single Support Gait function
        updated_joint_positions_matrix = ComputeJointPositionGait(model, update, dT);



    }

    // Update position of the joints 
    //  General position = pelvis position
    update.position.x = updated_joint_positions_matrix(6, 0);
    update.position.y = updated_joint_positions_matrix(6, 1);

    // // head position
    update.head_position.x = updated_joint_positions_matrix(10, 0);
    update.head_position.y = updated_joint_positions_matrix(10, 1);
    update.head_position.z = updated_joint_positions_matrix(10, 2);

    // // pelvis position (3D)
    update.pelvis_position.x = updated_joint_positions_matrix(6, 0);
    update.pelvis_position.y = updated_joint_positions_matrix(6, 1);
    update.pelvis_position.z = updated_joint_positions_matrix(6, 2);

    // // right heel position
    update.heel_right_position.x = updated_joint_positions_matrix(0, 0);
    update.heel_right_position.y = updated_joint_positions_matrix(0, 1);
    update.heel_right_position.z = updated_joint_positions_matrix(0, 2);
    
    // // left heel position
    update.heel_left_position.x = updated_joint_positions_matrix(5, 0);
    update.heel_left_position.y = updated_joint_positions_matrix(5, 1);
    update.heel_left_position.z = updated_joint_positions_matrix(5, 2);



    // ## shoulders
    update.shoulder_rotation_angle_z = 0.0;
    // ## trunk
    // ### along the frontal axis (x) of this agent
    update.trunk_rotation_angle_x = 0.0;
    // ### along sagittal axis (y) of this agent
    update.trunk_rotation_angle_y = 0.0;

    // print the updated joint positions

    // // std::cout << "Pelvis: " << update.position.x <<", "<< update.position.y << std::endl;
    // std::cout << "Right Heel: " << update.heel_right_position.x << ", " 
    //                             << update.heel_right_position.y << ", " 
    //                             << update.heel_right_position.z << std::endl;
    // std::cout << "Left Heel: "  << update.heel_left_position.x << ", " 
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



Eigen::MatrixXd HumanoidModelV0::ComputeJointPositionGait(
                                            const HumanoidModelV0Data& model,
                                            const HumanoidModelV0Update& update,
                                            double dT
                                    ) const

{
    //#### Precomputation

    // initialise the updated joint angles matrix
    Eigen::MatrixXd updated_joint_positions_matrix(11, 3); // 11 joints, 3 angles each

    double leg_length = model.height * (LEG_SCALING_FACTOR + ANKLE_SCALING_FACTOR); 
    double pelvis_width = model.height * PELVIS_WIDTH_SCALING_FACTOR;

    // compute step_complition_factor
    // this represent the avancement of the curent step. this factor == 1 when the step is over.
    double step_complition_factor = 1.0 - (static_cast<double>(update.step_timer) / update.step_duration);

    //#######
    

    if(update.stepping_foot_index == 1) // if the right foot is support foot (left foot stepping)
    {
        // Step 1: computation of the next stepping foot position
        // left foot stepping
        updated_joint_positions_matrix.row(5) <<    model.heel_left_position.x + update.velocity.x * dT,
                                                    model.heel_left_position.y + update.velocity.y * dT,
                                                    -0.4*step_complition_factor*(step_complition_factor-1); 
                                                    // the vertical displacement of the stepping foot is 
                                                    // a parabola with a maximum at 0.1m/ z=0.40(t)(t - 1);
                                                     

        // Step 2: computation of the pelvis and support hip position
        // the center of the pelvis is placed between both feet
        updated_joint_positions_matrix(6, 0) = model.heel_right_position.x + (updated_joint_positions_matrix(5, 0)-model.heel_right_position.x)/2;
        updated_joint_positions_matrix(6, 1) = model.heel_right_position.y + (updated_joint_positions_matrix(5, 1)-model.heel_right_position.y)/2;

        // position of the support hip (right hip)
        Point normal_orientation = update.velocity.Normalized().Rotate90Deg();
        Point3D support_hip_position;

        support_hip_position.x = updated_joint_positions_matrix(6, 0) - pelvis_width * 0.5 * normal_orientation.x;
        support_hip_position.y = updated_joint_positions_matrix(6, 1) - pelvis_width * 0.5 * normal_orientation.y;
        support_hip_position.z = model.heel_right_position.z; // to insure a planar computation of distance_support_hip_foot_xy

        double distance_support_hip_foot_xy = (support_hip_position - model.heel_right_position).Norm();

        // the height of the pelvis is based on the Leg length and position of the hip relative to the support foot
        // pyhtagore with leg legth and support foot-hip positions
        updated_joint_positions_matrix(6, 2) =  sqrt(std::pow(leg_length, 2)- std::pow(distance_support_hip_foot_xy,2));

        // Step 3: The support foot (right foot) remain at the same place (on the ground)
        updated_joint_positions_matrix(0, 0) = model.heel_right_position.x;
        updated_joint_positions_matrix(0, 1) = model.heel_right_position.y;
        updated_joint_positions_matrix(0, 2) = 0;                               
                                                
    } 
    else // if the left foot is support foot (right foot stepping)
    {
        // Step 1: computation of the next stepping foot position
        // left foot stepping
        updated_joint_positions_matrix.row(0) <<    model.heel_right_position.x + update.velocity.x * dT,
                                                    model.heel_right_position.y + update.velocity.y * dT,
                                                    -0.4*step_complition_factor*(step_complition_factor-1); 
                                            // the vertical displacement of the stepping foot is 
                                            // a parabola with a maximum at 0.1m;

        
        // Step 2: computation of the pelvis and support hip position
        // the center of the pelvis is placed between both feet
        updated_joint_positions_matrix(6, 0) = model.heel_left_position.x + (updated_joint_positions_matrix(0, 0)-model.heel_left_position.x)/2;
        updated_joint_positions_matrix(6, 1) = model.heel_left_position.y + (updated_joint_positions_matrix(0, 1)-model.heel_left_position.y)/2;

        // position of the support hip (right hip)
        Point normal_orientation = update.velocity.Normalized().Rotate90Deg();
        Point3D support_hip_position;

        support_hip_position.x = updated_joint_positions_matrix(6, 0) + pelvis_width * 0.5 * normal_orientation.x;
        support_hip_position.y = updated_joint_positions_matrix(6, 1) + pelvis_width * 0.5 * normal_orientation.y;
        support_hip_position.z = model.heel_left_position.z; // to insure a planar computation of distance_support_hip_foot_xy

        double distance_support_hip_foot_xy = (support_hip_position - model.heel_left_position).Norm();

        // the height of the pelvis is based on the Leg length and position of the hip relative to the support foot
        // pyhtagore with leg legth and support foot-hip positions
        updated_joint_positions_matrix(6, 2) =  sqrt(std::pow(leg_length, 2)- std::pow(distance_support_hip_foot_xy,2));

        // Step 3: The support foot (right foot) remain at the same place (on the ground)
        updated_joint_positions_matrix(5, 0) = model.heel_left_position.x;
        updated_joint_positions_matrix(5, 1) = model.heel_left_position.y;
        updated_joint_positions_matrix(5, 2) = 0; 

    }

    // Spet 4: update head position
    updated_joint_positions_matrix(10, 0) = updated_joint_positions_matrix(6, 0);
    updated_joint_positions_matrix(10, 1) = updated_joint_positions_matrix(6, 1);
    updated_joint_positions_matrix(10, 2) = updated_joint_positions_matrix(6, 2) 
                                                + model.height * (TRUNK_HEIGHT_SCALING_FACTOR 
                                                                + NECK_SCALING_FACTOR);


    return updated_joint_positions_matrix;

}

// In this function the motion of a step is initiated after a double support phase.
// The function computes the joint angles then the joint position 
//  based on the step length, step width, and the current model state.
// should return: update.stepping_foot_index, update.step_timer update.joint_angles_matrix update.joint_positions_matrix
Eigen::MatrixXd HumanoidModelV0::ComputeJointAnglesStepDoubleSupports(
                                        const GenericAgent& agent,
                                        double step_length, 
                                        double step_width) const
{
    // Get the model data from the agent
    auto model = std::get<HumanoidModelV0Data>(agent.model);

    // initialise the updated joint angles matrix
    Eigen::MatrixXd updated_joint_angles_matrix(11, 3); // 11 joints, 3 angles each

    

    // Rotation 
    // only walk without rotation is implemented for now (rotation_index != 0)


    double theta, phi_a;

    theta = asin(step_length / (2 * model.height * LEG_SCALING_FACTOR));;
    phi_a = asin((model.height * PELVIS_WIDTH_SCALING_FACTOR  - step_width) 
            / (2 * model.height * LEG_SCALING_FACTOR));

    
    // Update joint angles
    if (model.stepping_foot_index == 1) // if the right foot is support foot (left foot stepping)
    {
        // right ankle
        updated_joint_angles_matrix.row(1) <<    phi_a,        
                                                - theta, 
                                                0; 
        // right hip
        updated_joint_angles_matrix.row(2) <<   -phi_a, 
                                                theta,
                                                0; 
        // pelvis-trunk
        updated_joint_angles_matrix.row(6) <<   0, 
                                                0,
                                                0; 
        // left hip    
        updated_joint_angles_matrix.row(3) <<   - phi_a, 
                                                - theta, 
                                                0; 
        // left ankle
        updated_joint_angles_matrix.row(4) <<   phi_a, 
                                                theta, 
                                                0;

    } 
    else if (model.stepping_foot_index == -1) // if support foot is the left foot, right foot stepping 
    { 
        // left ankle
        updated_joint_angles_matrix.row(4) <<   -phi_a,        
                                                -theta, 
                                                0; 
        // left hip
        updated_joint_angles_matrix.row(3) <<   phi_a , 
                                                theta,
                                                0; 
        // pelvis-trunk
        updated_joint_angles_matrix.row(6) <<   0, 
                                                0,
                                                0; 
        // right hip    
        updated_joint_angles_matrix.row(2) <<   phi_a, 
                                                -theta, 
                                                0; 
        // right ankle
        updated_joint_angles_matrix.row(1) <<   -phi_a, 
                                                theta, 
                                                0;
    }


    return updated_joint_angles_matrix;

    // ... thenUpdate joint positions
    // call function UpdateLimbPositions to compute the new joint positions


}




Eigen::MatrixXd HumanoidModelV0::ComputeJointAnglesGaitSingleSupport(const GenericAgent& agent,
                                        double step_length, 
                                        double step_width, 
                                        // the following argument have to be removed in the future
                                        double step_duration
                                    ) const

{
    auto model = std::get<HumanoidModelV0Data>(agent.model);

    // initialise the updated joint angles matrix
    Eigen::MatrixXd updated_joint_angles_matrix(11, 3); // 11 joints, 3 angles each


    // compute step_complition_factor
    // this represent the avancement of the curent step. this factor == 1 when the step is over.
    double step_complition_factor = 1 - model.step_timer/step_duration; 

    // Rotation 
    // only walk without rotation is implemented for now (rotation_index != 0)
    double traveled_step_length; // length traveled by the stepping foot during this time step (former "sl_p")
    double lean_angle; // What is this angle ? // It allows the head to oscillate perdendicularly to the walking direction, so the head trajectory looks like a sine wave

    if (model.step_timer > step_duration/2) {
        traveled_step_length = 2 * std::pow(step_complition_factor, 2) //
                                    * (step_complition_factor + step_length) - step_length; 
        lean_angle  =  (4 * step_complition_factor)*M_PI/180 ;
    }
    else 
    {
        traveled_step_length = step_length - 2 * std::pow(1-step_complition_factor, 2) 
                                    * (2 * step_length);
        lean_angle  =  (4 - 4 * step_complition_factor)*M_PI/180 ;
    }

    double theta, phi_a;
    // only walk without rotation is implemented for now (rotation_index != 0)  
    theta = asin(traveled_step_length / (2 * model.height * LEG_SCALING_FACTOR));
    phi_a = asin(((model.height * PELVIS_WIDTH_SCALING_FACTOR - step_width) 
            / (2 * model.height * LEG_SCALING_FACTOR))+ lean_angle);


    // Update joint angles
    if (model.stepping_foot_index == 1) // if the right foot is support foot (left foot stepping)
    {
         // right ankle
        updated_joint_angles_matrix.row(1) <<    phi_a,        
                                                - theta, 
                                                0; 
        // right hip
        updated_joint_angles_matrix.row(2) <<   -phi_a, 
                                                theta,
                                                0; 
        // pelvis-trunk
        updated_joint_angles_matrix.row(6) <<   0, 
                                                0,
                                                0; 
        // left hip    
        updated_joint_angles_matrix.row(3) <<   - phi_a, 
                                                - theta, 
                                                0; 
        // left ankle
        updated_joint_angles_matrix.row(4) <<   phi_a, 
                                                theta, 
                                                0;

    } 
    else if (model.stepping_foot_index == -1) // if support foot is the left foot, right foot stepping 
    { 
       // left ankle
        updated_joint_angles_matrix.row(4) <<   -phi_a,        
                                                -theta, 
                                                0; 
        // left hip
        updated_joint_angles_matrix.row(3) <<   phi_a , 
                                                theta,
                                                0; 
        // pelvis-trunk
        updated_joint_angles_matrix.row(6) <<   0, 
                                                0,
                                                0; 
        // right hip    
        updated_joint_angles_matrix.row(2) <<   phi_a, 
                                                -theta, 
                                                0; 
        // right ankle
        updated_joint_angles_matrix.row(1) <<   -phi_a, 
                                                theta, 
                                                0;
    }

    
    return updated_joint_angles_matrix;
    
    // ... thenUpdate joint positions
    // call function UpdateLimbPositions to compute the new joint positions


}



Eigen::MatrixXd HumanoidModelV0::ComputeJointPositionsfromJointAngles (
                            const GenericAgent& agent,
                            const HumanoidModelV0Update& update, // this is the orientation angle of the agent's frame
                            Eigen::MatrixXd updated_joint_angles_matrix
                    
    ) const
{
 
    
    // Get the model data from the agent
    auto model = std::get<HumanoidModelV0Data>(agent.model);

    
    // initialise the updated joint positions matrix
    Eigen::MatrixXd updated_joint_positions_matrix; 
    updated_joint_positions_matrix = Eigen::MatrixXd::Zero(11, 3); // 11 joints, 3 positions each


    // Compute all segment lengths based on the model height
    // double foot_forward_length = model.height * FOOT_FORWARD_LENGTH_SCALING_FACTOR;
    double foot_backward_length = model.height * FOOT_BACKWARD_SCALING_FACTOR;
    double ankle_length = model.height * ANKLE_SCALING_FACTOR;
    double leg_length = model.height * LEG_SCALING_FACTOR; 
    double pelvis_width = model.height * PELVIS_WIDTH_SCALING_FACTOR;
    double neck_length = model.height * NECK_SCALING_FACTOR;
    // double shoulder_width = model.height * SHOULDER_WIDTH_SCALING_FACTOR;
    double trunk_length = model.height * TRUNK_HEIGHT_SCALING_FACTOR;
    // double trunk_width = model.height * TRUNK_WIDTH_SCALING_FACTOR;



    // Tranformation from the Agent frame to the World frame
    double updated_orientation_angle = acos(update.velocity.Normalized().x);
    if (update.velocity.Normalized().x < 0) {
        updated_orientation_angle = 2 * M_PI - updated_orientation_angle; // adjust the angle to be in the range [0, 2*pi] 
    }
    // this transformation is used to compute the position of the joints in the world frame
    Eigen::Matrix4d T_world_agent = CreateTransformationMatrix(
        Eigen::Vector3d(update.position.x, update.position.y, leg_length+ankle_length), 
        Eigen::Vector3d(0, 0, updated_orientation_angle) // rotation vector (yaw angle)
    );
    Eigen::Vector4d origin_vector ={0,0,0,1}; // origin vector for the computation of the joint positions

    // pelvis ==> no translation, only rotation
    Eigen::Matrix4d T_agent_pelvis = CreateTransformationMatrix(
        Eigen::Vector3d(0, 0, 0), // no translation
        Eigen::Vector3d(updated_joint_angles_matrix.row(6)) // rotation of the pelvis
    );
    Eigen::Vector4d pelvis_vector =     T_world_agent   
                                        * T_agent_pelvis * origin_vector; // position of the pelvis in the world frame
    updated_joint_positions_matrix.row(6) <<    pelvis_vector[0], 
                                                pelvis_vector[1], 
                                                pelvis_vector[2]; // update the position of the pelvis

    //  right hip ==> translation of -pelvis_width/2 along the y-axis 
    Eigen::Matrix4d T_pelvis_righthip  = CreateTransformationMatrix(
        Eigen::Vector3d(-pelvis_width/2, 0, 0),  // translation of -pelvis_width/2 along the y-axis 
        Eigen::Vector3d(updated_joint_angles_matrix.row(2)) // rotation of the right hip joint
    ); 

    Eigen::Vector4d right_hip_vector =  T_world_agent 
                                        * T_agent_pelvis 
                                        * T_pelvis_righthip * origin_vector; // position of the right hip in the world frame
    updated_joint_positions_matrix.row(2) <<    right_hip_vector[0], 
                                                right_hip_vector[1], 
                                                right_hip_vector[2]; // update the position of the right hip
    
    //  right ankle ==> translation of -leg_length along the z-axis
    Eigen::Matrix4d T_righthip_rightankle = CreateTransformationMatrix(
        Eigen::Vector3d(0, 0, -leg_length), // translation of -leg_length along the z-axis
        Eigen::Vector3d(updated_joint_angles_matrix.row(1)) // rotation of the right ankle joint
    );
    
    Eigen::Vector4d right_ankle_vector =    T_world_agent 
                                            * T_agent_pelvis 
                                            * T_pelvis_righthip 
                                            * T_righthip_rightankle * origin_vector;
    updated_joint_positions_matrix.row(1) <<    right_ankle_vector[0],
                                                right_ankle_vector[1], 
                                                right_ankle_vector[2]; // update the position of the right ankle
                                                
    //  right heel ==> translation of -ankle_length along the z-axis and -foot_backward_length along the x-axis
    Eigen::Matrix4d T_rightankle_rightheel = CreateTransformationMatrix(
        Eigen::Vector3d(-foot_backward_length, 0, -ankle_length),               
        Eigen::Vector3d(0, 0, 0) // no rotation for the heel
    );
    Eigen::Vector4d right_heel_vector =     T_world_agent 
                                            * T_agent_pelvis
                                            * T_pelvis_righthip 
                                            * T_righthip_rightankle 
                                            * T_rightankle_rightheel * origin_vector;
    updated_joint_positions_matrix.row(0) <<    right_heel_vector[0],
                                                right_heel_vector[1], 
                                                right_heel_vector[2]; // update the position of the right heel

    //  left hip ==> translation of pelvis_width/2 along the y-axis
    Eigen::Matrix4d T_pelvis_lefthip = CreateTransformationMatrix(
        Eigen::Vector3d(pelvis_width/2, 0, 0), // translation of pelvis_width/2 along the y-axis 
        Eigen::Vector3d(updated_joint_angles_matrix.row(3)) // rotation of the left hip joint
    );
    
    Eigen::Vector4d left_hip_vector =      T_world_agent 
                                            * T_agent_pelvis 
                                            * T_pelvis_lefthip * origin_vector;
                                            
    updated_joint_positions_matrix.row(3) <<    left_hip_vector[0],
                                                left_hip_vector[1], 
                                                left_hip_vector[2]; // update the position of the left hip
    //  left ankle ==> translation of -leg_length along the z-axis
    Eigen::Matrix4d T_lefthip_leftankle = CreateTransformationMatrix(
        Eigen::Vector3d(0, 0, -leg_length), // translation of -leg_length along the z-axis
        Eigen::Vector3d(updated_joint_angles_matrix.row(4)) // rotation of the left ankle joint
    );
    
    Eigen::Vector4d left_ankle_vector =    T_world_agent 
                                            * T_agent_pelvis 
                                            * T_pelvis_lefthip 
                                            * T_lefthip_leftankle * origin_vector;
    updated_joint_positions_matrix.row(4) <<    left_ankle_vector[0],
                                                left_ankle_vector[1], 
                                                left_ankle_vector[2]; // update the position of the left ankle

    //  left heel ==> translation of -ankle_length along the z-axis and -foot_backward_length along the x-axis
    Eigen::Matrix4d T_leftankle_leftheel = CreateTransformationMatrix(
        Eigen::Vector3d(-foot_backward_length, 0, -ankle_length), // translation of -foot_backward_length along the x-axis and -ankle_length along the z-axis
        Eigen::Vector3d(0, 0, 0) // no rotation for the heel
    );
    Eigen::Vector4d left_heel_vector =     T_world_agent 
                                            * T_agent_pelvis
                                            * T_pelvis_lefthip 
                                            * T_lefthip_leftankle 
                                            * T_leftankle_leftheel * origin_vector;
    updated_joint_positions_matrix.row(5) <<    left_heel_vector[0],
                                                left_heel_vector[1], 
                                                left_heel_vector[2]; // update the position of the left heel

    //  C7/trunk ==> translation of trunk_length along the z-axis 
    Eigen::Matrix4d T_pelvis_trunk = CreateTransformationMatrix(
        Eigen::Vector3d(0, 0, trunk_length), // translation of trunk_length along the z-axis
        Eigen::Vector3d(updated_joint_angles_matrix.row(8)) // rotation of the trunk joint
    );
    Eigen::Vector4d trunk_vector =        T_world_agent 
                                            * T_agent_pelvis 
                                            * T_pelvis_trunk * origin_vector;
    updated_joint_positions_matrix.row(8) <<    trunk_vector[0],
                                                trunk_vector[1], 
                                                trunk_vector[2]; // update the position of the trunk

    //  head ==> translation of neck_length along the z-axis
    Eigen::Matrix4d T_trunk_head = CreateTransformationMatrix(
        Eigen::Vector3d(0, 0, neck_length), // translation of neck_length along the z-axis
        Eigen::Vector3d(updated_joint_angles_matrix.row(10)) // rotation of the head joint
    );
    Eigen::Vector4d head_vector =         T_world_agent 
                                            * T_agent_pelvis 
                                            * T_pelvis_trunk 
                                            * T_trunk_head * origin_vector;
    updated_joint_positions_matrix.row(10) <<   head_vector[0],
                                                head_vector[1], 
                                                head_vector[2]; // update the position of the head


    return updated_joint_positions_matrix;
}



