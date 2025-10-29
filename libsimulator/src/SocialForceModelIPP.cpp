// SPDX-License-Identifier: LGPL-3.0-or-later
#include "SocialForceModelIPP.hpp"

#include "Ellipse.hpp"
#include "GenericAgent.hpp"
#include "Macros.hpp"
#include "Mathematics.hpp"
#include "NeighborhoodSearch.hpp"
#include "OperationalModel.hpp"
#include "OperationalModelType.hpp"
#include "Simulation.hpp"
#include "SocialForceModelIPPData.hpp"

#include <Logger.hpp>
#include <iostream>
#include <stdexcept>

SocialForceModelIPP::SocialForceModelIPP(double bodyForce_, double friction_)
    : bodyForce(bodyForce_), friction(friction_) {};

OperationalModelType SocialForceModelIPP::Type() const
{
    return OperationalModelType::SOCIAL_FORCE_IPP;
}

std::unique_ptr<OperationalModel> SocialForceModelIPP::Clone() const
{
    return std::make_unique<SocialForceModelIPP>(*this);
}

OperationalModelUpdate SocialForceModelIPP::ComputeNewPosition(
    double dT,
    const GenericAgent& ped,
    const CollisionGeometry& geometry,
    const NeighborhoodSearchType& neighborhoodSearch) const
{
    const auto& model = std::get<SocialForceModelIPPData>(ped.model);
    SocialForceModelIPPUpdate update{};
    
    const auto neighborhood = neighborhoodSearch.GetNeighboringAgents(ped.pos, this->_cutOffRadius);
    const auto& walls = geometry.LineSegmentsInApproxDistanceTo(ped.pos);

    // Collision avoidance model: Social Forces
    auto social_forces = DrivingForce(ped);
    Point F_rep_agents;
    for(const auto& neighbor : neighborhood) {
        if(neighbor.id == ped.id) {
            continue;
        }
        F_rep_agents += AgentSocialForce(ped, neighbor);
    }
    social_forces += F_rep_agents / model.mass;

    const auto F_social_obstacles = std::accumulate(
        walls.cbegin(),
        walls.cend(),
        Point(0, 0),
        [this, &ped](const auto& acc, const auto& element) {
            return acc + ObstacleSocialForce(ped, element);
        });
    social_forces += F_social_obstacles / model.mass;



    // Physical interactions: Contact forces
    Point contact_forces;
    for(const auto& neighbor : neighborhood) {
        if(neighbor.id == ped.id) {
            continue;
        }
        F_rep_agents += AgentContactForce(ped, neighbor);
    }
    contact_forces += F_rep_agents / model.mass;

    const auto F_contact_obstacles = std::accumulate(
        walls.cbegin(),
        walls.cend(),
        Point(0, 0),
        [this, &ped](const auto& acc, const auto& element) {
            return acc + ObstacleSocialForce(ped, element);
        });
    contact_forces += F_contact_obstacles / model.mass;



    if (contact_forces.Norm() < 0.01) {
        // Locomotion mode
        // The upper body follow the gorund support
        // ## ground support 
        update.ground_support_velocity =    model.ground_support_velocity + 
                                        (   
                                            social_forces
                                        )* dT;
        update.ground_support_position = model.ground_support_position + update.ground_support_velocity * dT;

        // ## upper body
        update.velocity = model.velocity 
                                    + ( 
                                        (update.ground_support_position - ped.pos) * LAMBDA_LOCOMOTION_1 
                                        + (update.ground_support_velocity - model.velocity) * LAMBDA_LOCOMOTION_2
                                        - model.velocity * LAMBDA_LOCOMOTION_3
                                    ) * dT;
        update.position = ped.pos + update.velocity * dT;



    } else {
        // Recovery mode
        // The ground support follow the upper body, The contact and colision avoidance comes from the upper body
        // ## upper body
        update.velocity = model.velocity 
                                    + ( 
                                        social_forces
                                        + contact_forces
                                        - model.velocity * LAMBDA_RECOVERY_3
                                    ) * dT;
        update.position = ped.pos + update.velocity * dT;

        // ## ground support
        update.ground_support_velocity =    model.ground_support_velocity +
                                        (   
                                            (update.position - model.ground_support_position) * LAMBDA_RECOVERY_1 
                                            + (update.velocity - model.ground_support_velocity) * LAMBDA_RECOVERY_2
                                            - model.ground_support_velocity * LAMBDA_RECOVERY_3
                                        )* dT;
        update.ground_support_position = model.ground_support_position + update.ground_support_velocity * dT;
    }



    

    // printf(
    //     "pos=(%.2f, %.2f), vel=(%.2f, %.2f), gs_pos=(%.2f, %.2f), gs_vel=(%.2f, %.2f)\n forces=(%.2f, %.2f)\n",
    //     update.position.x,
    //     update.position.y,
    //     update.velocity.x,
    //     update.velocity.y,
    //     update.ground_support_position.x,
    //     update.ground_support_position.y,
    //     update.ground_support_velocity.x,
    //     update.ground_support_velocity.y,
    //     forces.x, forces.y);
    // std::cin.get(); //pause

    return update;
}

void SocialForceModelIPP::ApplyUpdate(const OperationalModelUpdate& update, GenericAgent& agent) const
{
    auto& model = std::get<SocialForceModelIPPData>(agent.model);
    const auto& upd = std::get<SocialForceModelIPPUpdate>(update);
    agent.pos = upd.position;
    agent.orientation = upd.velocity.Normalized();
    model.velocity = upd.velocity;
    model.ground_support_position = upd.ground_support_position;
    model.ground_support_velocity = upd.ground_support_velocity;
    
}

void SocialForceModelIPP::CheckModelConstraint(
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

    const auto& model = std::get<SocialForceModelIPPData>(agent.model);

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

Point SocialForceModelIPP::DrivingForce(const GenericAgent& agent)
{
    const auto& model = std::get<SocialForceModelIPPData>(agent.model);
    const Point e0 = (agent.destination - agent.pos).Normalized();
    return (e0 * model.desiredSpeed - model.velocity) / model.reactionTime;
};
double SocialForceModelIPP::PushingForceLength(double A, double B, double r, double distance)
{
    return A * exp((r - distance) / B);
}

Point SocialForceModelIPP::AgentSocialForce(const GenericAgent& ped1, const GenericAgent& ped2) const
{
    const auto& model1 = std::get<SocialForceModelIPPData>(ped1.model);
    const auto& model2 = std::get<SocialForceModelIPPData>(ped2.model);

    const double total_radius = model1.radius + model2.radius;

    return SocialForceBetweenPoints(
        ped1.pos,
        ped2.pos,
        model1.agentScale,
        model1.forceDistance,
        total_radius);
};

Point SocialForceModelIPP::ObstacleSocialForce(const GenericAgent& agent, const LineSegment& segment) const
{
    const auto& model = std::get<SocialForceModelIPPData>(agent.model);
    const Point pt = segment.ShortestPoint(agent.pos);
    return SocialForceBetweenPoints(
        agent.pos, pt, model.obstacleScale, model.forceDistance, model.radius);
}

Point SocialForceModelIPP::AgentContactForce(const GenericAgent& ped1, const GenericAgent& ped2) const
{
    const auto& model1 = std::get<SocialForceModelIPPData>(ped1.model);
    const auto& model2 = std::get<SocialForceModelIPPData>(ped2.model);

    const double total_radius = model1.radius + model2.radius;

    return ContactForceBetweenPoints(
        ped1.pos,
        ped2.pos,
        total_radius,
        model2.velocity - model1.velocity);
};

Point SocialForceModelIPP::ObstacleContactForce(const GenericAgent& agent, const LineSegment& segment) const
{
    const auto& model = std::get<SocialForceModelIPPData>(agent.model);
    const Point pt = segment.ShortestPoint(agent.pos);
    return ContactForceBetweenPoints(
        agent.pos, pt, model.radius, model.velocity);
}


Point SocialForceModelIPP::SocialForceBetweenPoints(
    const Point pt1,
    const Point pt2,
    const double A,
    const double B,
    const double radius) const
{
    // todo reduce range of force to 180 degrees
    const double dist = (pt1 - pt2).Norm();
    double pushing_force_norm = PushingForceLength(A, B, radius, dist); // if == 0, the SF model is removed, only physical interactions are taken into account 
    const Point n_ij = (pt1 - pt2).Normalized();
    return n_ij * pushing_force_norm;
}

Point SocialForceModelIPP::ContactForceBetweenPoints(
    const Point pt1,
    const Point pt2,
    const double radius,
    const Point velocity) const
{
    // todo reduce range of force to 180 degrees
    const double dist = (pt1 - pt2).Norm();
    double pushing_force_norm = 0;
    double friction_force_norm = 0;
    const Point n_ij = (pt1 - pt2).Normalized();
    const Point tangent = n_ij.Rotate90Deg();
    if(dist < radius) {
        pushing_force_norm += this->bodyForce * (radius - dist);
        friction_force_norm =
            this->friction * (radius - dist) * (velocity.ScalarProduct(tangent));
    }
    return n_ij * pushing_force_norm + tangent * friction_force_norm;
}
