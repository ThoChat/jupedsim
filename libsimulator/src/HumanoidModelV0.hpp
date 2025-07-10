// Copyright © 2012-2024 Forschungszentrum Jülich GmbH
// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "CollisionFreeSpeedModelData.hpp"
#include "CollisionGeometry.hpp"
#include "NeighborhoodSearch.hpp"
#include "OperationalModel.hpp"
#include "HumanoidModelV0Data.hpp"
#include "UniqueID.hpp"

#include <Eigen/Dense>


struct GenericAgent;

class HumanoidModelV0 : public OperationalModel
{
public:
    using NeighborhoodSearchType = NeighborhoodSearch<GenericAgent>;

private:
    double _cutOffRadius{2.5};
    double bodyForce;
    double friction;

public:
    HumanoidModelV0(double bodyForce_, double friction_);
    ~HumanoidModelV0() override = default;
    OperationalModelType Type() const override;
    OperationalModelUpdate ComputeNewPosition(
        double dT,
        const GenericAgent& ped,
        const CollisionGeometry& geometry,
        const NeighborhoodSearchType& neighborhoodSearch) const override;
    void ApplyUpdate(const OperationalModelUpdate& update, GenericAgent& agent) const override;
    void CheckModelConstraint(
        const GenericAgent& agent,
        const NeighborhoodSearchType& neighborhoodSearch,
        const CollisionGeometry& geometry) const override;
    std::unique_ptr<OperationalModel> Clone() const override;

private:
    /**
     * Driving force acting on pedestrian <agent>
     * @param agent reference to Pedestrian
     *
     * @return vector with driving force of pedestrian
     */
    static Point DrivingForce(const GenericAgent& agent);
    /**
     *  Repulsive force acting on pedestrian <ped1> from pedestrian <ped2>
     * @param ped1 reference to Pedestrian 1 on whom the force acts on
     * @param ped2 reference to Pedestrian 2, from whom the force originates
     * @return vector with the repulsive force
     */
    Point AgentForce(const GenericAgent& ped1, const GenericAgent& ped2) const;
    /**
     *  Repulsive force acting on pedestrian <agent> from line segment <segment>
     * @param agent reference to the Pedestrian on whom the force acts on
     * @param segment reference to line segment, from which the force originates
     * @return vector with the repulsive force
     */
    Point ObstacleForce(const GenericAgent& agent, const LineSegment& segment) const;
    /**
     * calculates the pushing and friction forces acting between <pt1> and <pt2>
     * @param pt1 Point on which the forces act
     * @param pt2 Point from which the forces originate
     * @param A Agent scale
     * @param B force distance
     * @param r radius
     * @param velocity velocity difference
     */
    Point ForceBetweenPoints(
        const Point pt1,
        const Point pt2,
        const double A,
        const double B,
        const double radius,
        const Point velocity) const;
    /**
     *  exponential function that specifies the length of the pushing force between two points
     * @param A Agent scale
     * @param B force distance
     * @param r radius
     * @param distance distance between the two points
     * @return length of pushing force between the two points
     */
    static double PushingForceLength(double A, double B, double r, double distance);



    HumanoidModelV0Update ComputeGaitMotion(
                            const HumanoidModelV0Data& model,
                            const HumanoidModelV0Update& update,
                            double dT) 
                            const;

    /*#########################################################################
    ###     Relative to the Humanoid Representation Paradigm of agents      ###
    #########################################################################*/

    // ### Constants ###


    public: 
    // # Anthropometric scaling factors #
    /* The following parameters are multiplyed by agents height (HumanoidModelV0Data.height) 
        to obtain the legth of the considered Limb.  */ 
    static constexpr double NECK_SCALING_FACTOR = 0.1396;
    static constexpr double SHOULDER_WIDTH_SCALING_FACTOR = 0.45/1.7; 
    static constexpr double TRUNK_WIDTH_SCALING_FACTOR = 0.1470; // called l_r previously
    static constexpr double TRUNK_HEIGHT_SCALING_FACTOR = 0.3495; 
    static constexpr double PELVIS_WIDTH_SCALING_FACTOR = 0.4 / 1.7; 
    static constexpr double LEG_SCALING_FACTOR = 0.4791; //0.2522 (shank) + 0.2269 (thigh)
    static constexpr double ANKLE_SCALING_FACTOR = 0.0451; 
    static constexpr double FOOT_FORWARD_SCALING_FACTOR = 0.1470/2; // 
    static constexpr double FOOT_BACKWARD_SCALING_FACTOR = 0.1470/4; // 
    static constexpr double FOOT_WIDTH_SCALING_FACTOR = 0.1470*8/50; // 

    // have to be set private later
    /**
     * Denavit-Hartenberg matrix transformation
     * This function calculates the Denavit-Hartenberg matrix based on the given parameters.
     * The Denavit-Hartenberg convention is a standardized way to represent the joint parameters of a robotic manipulator.
     * It is used to describe the relationship between the joint angles and the position and orientation of the end-effector.
     * @param theta joint angle
     * @param d distance along the previous z-axis
     * @param a distance along the current x-axis
     * @param alpha angle between the previous z-axis and the current z-axis
     * @return Denavit-Hartenberg matrix
    
     */
    Eigen::Matrix4d DHTransformationMatrix(
    double theta, 
    double d, 
    double r, 
    double alpha) const;

    Eigen::Matrix4d CreateTransformationMatrix( const Eigen::Vector3d &translation, 
                                    const Eigen::Vector3d &rotation) const;


    

    

};
