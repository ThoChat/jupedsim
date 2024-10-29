// Copyright © 2012-2024 Forschungszentrum Jülich GmbH
// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "error.h"
#include "export.h"
#include "operational_model.h"
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Opaque type for a HumanoidModelV0 Builder
 */
typedef struct JPS_HumanoidModelV0Builder_t* JPS_HumanoidModelV0Builder;

/**
 * Creates a HumanoidModelV0 builder.
 * @param bodyForce describes the strength with which an agent is influenced
 * by pushing forces from obstacles and neighbors in its direct proximity.
 * @param friction describes the strength with which an agent is influenced
 * by frictional forces from obstacles and neighbors in its direct proximity.
 * */
JUPEDSIM_API JPS_HumanoidModelV0Builder
JPS_HumanoidModelV0Builder_Create(double bodyForce, double friction);

/**
 * Creates a JPS_OperationalModel of type HumanoidModelV0 Model from the
 * JPS_HumanoidModelV0Builder.
 * @param handle the builder to operate on
 * @param[out] errorMessage if not NULL: will be set to a JPS_ErrorMessage in case of an error
 * @return a JPS_HumanoidModelV0 or NULL if an error occured.
 */
JUPEDSIM_API JPS_OperationalModel JPS_HumanoidModelV0Builder_Build(
    JPS_HumanoidModelV0Builder handle,
    JPS_ErrorMessage* errorMessage);

/**
 * Frees a JPS_HumanoidModelV0Builder
 * @param handle to the JPS_HumanoidModelV0Builder to free.
 */
JUPEDSIM_API void JPS_HumanoidModelV0Builder_Free(JPS_HumanoidModelV0Builder handle);

/**
 * Opaque type of Social Force model state
 */
typedef struct JPS_HumanoidModelV0State_t* JPS_HumanoidModelV0State;

/**
 * Read Velocity of this agent.
 * @param handle of the Agent to access.
 * @return Velocity of this agent.
 */
JUPEDSIM_API JPS_Point JPS_HumanoidModelV0State_GetVelocity(JPS_HumanoidModelV0State handle);

/**
 * Write Velocity of this agent.
 * @param handle of the Agent to access.
 * @param velocity Velocity of this agent.
 */
JUPEDSIM_API void
JPS_HumanoidModelV0State_SetVelocity(JPS_HumanoidModelV0State handle, JPS_Point velocity);

/**
 * Read mass of this agent.
 * @param handle of the Agent to access.
 * @return mass in kg of this agent
 */
JUPEDSIM_API double JPS_HumanoidModelV0State_GetMass(JPS_HumanoidModelV0State handle);

/**
 * Write mass of this agent.
 * @param handle of the Agent to access.
 * @param mass in kg of this agent.
 */
JUPEDSIM_API void JPS_HumanoidModelV0State_SetMass(JPS_HumanoidModelV0State handle, double mass);

/**
 * Read desired Speed of this agent.
 * @param handle of the Agent to access.
 * @return desired Speed in m/s of this agent
 */
JUPEDSIM_API double JPS_HumanoidModelV0State_GetDesiredSpeed(JPS_HumanoidModelV0State handle);

/**
 * Write desired Speed of this agent.
 * @param handle of the Agent to access.
 * @param desiredSpeed in m/s of this agent.
 */
JUPEDSIM_API void
JPS_HumanoidModelV0State_SetDesiredSpeed(JPS_HumanoidModelV0State handle, double desiredSpeed);

/**
 * Read reaction Time of this agent.
 * @param handle of the Agent to access.
 * @return reaction Time in s of this agent
 */
JUPEDSIM_API double JPS_HumanoidModelV0State_GetReactionTime(JPS_HumanoidModelV0State handle);

/**
 * Write reaction Time of this agent.
 * @param handle of the Agent to access.
 * @param reactionTime in s of this agent.
 */
JUPEDSIM_API void
JPS_HumanoidModelV0State_SetReactionTime(JPS_HumanoidModelV0State handle, double reactionTime);

/**
 * Read agent Scale of this agent.
 * @param handle of the Agent to access.
 * @return agent Scale of this agent
 */
JUPEDSIM_API double JPS_HumanoidModelV0State_GetAgentScale(JPS_HumanoidModelV0State handle);

/**
 * Write agent Scale of this agent.
 * @param handle of the Agent to access.
 * @param agentScale of this agent.
 */
JUPEDSIM_API void
JPS_HumanoidModelV0State_SetAgentScale(JPS_HumanoidModelV0State handle, double agentScale);

/**
 * Read obstacle Scale of this agent.
 * @param handle of the Agent to access.
 * @return obstacle Scale of this agent
 */
JUPEDSIM_API double JPS_HumanoidModelV0State_GetObstacleScale(JPS_HumanoidModelV0State handle);

/**
 * Write obstacle Scale of this agent.
 * @param handle of the Agent to access.
 * @param obstacleScale of this agent.
 */
JUPEDSIM_API void
JPS_HumanoidModelV0State_SetObstacleScale(JPS_HumanoidModelV0State handle, double obstacleScale);

/**
 * Read force Distance of this agent.
 * @param handle of the Agent to access.
 * @return force Distance of this agent
 */
JUPEDSIM_API double JPS_HumanoidModelV0State_GetForceDistance(JPS_HumanoidModelV0State handle);

/**
 * Write force Distance of this agent.
 * @param handle of the Agent to access.
 * @param forceDistance of this agent.
 */
JUPEDSIM_API void
JPS_HumanoidModelV0State_SetForceDistance(JPS_HumanoidModelV0State handle, double forceDistance);

/**
 * Read radius of this agent.
 * @param handle of the Agent to access.
 * @return radius in m of this agent
 */
JUPEDSIM_API double JPS_HumanoidModelV0State_GetRadius(JPS_HumanoidModelV0State handle);

/**
 * Write radius of this agent.
 * @param handle of the Agent to access.
 * @param radius in m of this agent.
 */
JUPEDSIM_API void
JPS_HumanoidModelV0State_SetRadius(JPS_HumanoidModelV0State handle, double radius);

/**
 * Read head position of this agent.
 * @param handle of the Agent to access.
 * @return head position of this agent.
 */
JUPEDSIM_API JPS_Point JPS_HumanoidModelV0State_GetHeadPosition(JPS_HumanoidModelV0State handle);

/**
 * Write head position of this agent.
 * @param handle of the Agent to access.
 * @param head position of this agent.
 */
JUPEDSIM_API void
JPS_HumanoidModelV0State_SetHeadPosition(JPS_HumanoidModelV0State handle, JPS_Point head_position);

/**
 * Read head velocity of this agent.
 * @param handle of the Agent to access.
 * @return head velocity of this agent.
 */
JUPEDSIM_API JPS_Point JPS_HumanoidModelV0State_GetHeadVelocity(JPS_HumanoidModelV0State handle);

/**
 * Write head velocity of this agent.
 * @param handle of the Agent to access.
 * @param head velocity of this agent.
 */
JUPEDSIM_API void
JPS_HumanoidModelV0State_SetHeadVelocity(JPS_HumanoidModelV0State handle, JPS_Point head_velocity);

/**
 * Read right shoulder position of this agent.
 * @param handle of the Agent to access.
 * @return right shoulder velocity of this agent.
 */
JUPEDSIM_API JPS_Point JPS_HumanoidModelV0State_GetShoulderRightPosition(JPS_HumanoidModelV0State handle);

/**
 * Write right shoulder position of this agent.
 * @param handle of the Agent to access.
 * @param right shoulder position of this agent.
 */
JUPEDSIM_API void
JPS_HumanoidModelV0State_SetShoulderRightPosition(JPS_HumanoidModelV0State handle, JPS_Point shoulder_right_position);

/**
 * Read right shoulder velocity of this agent.
 * @param handle of the Agent to access.
 * @return right shoulder velocity of this agent.
 */
JUPEDSIM_API JPS_Point JPS_HumanoidModelV0State_GetShoulderRightVelocity(JPS_HumanoidModelV0State handle);

/**
 * Write right shoulder velocity of this agent.
 * @param handle of the Agent to access.
 * @param right shoulder velocity of this agent.
 */
JUPEDSIM_API void
JPS_HumanoidModelV0State_SetShoulderRightVelocity(JPS_HumanoidModelV0State handle, JPS_Point shoulder_right_velocity);

/**
 * Read right shoulder position of this agent.
 * @param handle of the Agent to access.
 * @return right shoulder velocity of this agent.
 */
JUPEDSIM_API JPS_Point JPS_HumanoidModelV0State_GetShoulderLeftPosition(JPS_HumanoidModelV0State handle);

/**
 * Write right shoulder position of this agent.
 * @param handle of the Agent to access.
 * @param right shoulder position of this agent.
 */
JUPEDSIM_API void
JPS_HumanoidModelV0State_SetShoulderLeftPosition(JPS_HumanoidModelV0State handle, JPS_Point shoulder_left_position);

/**
 * Read right shoulder velocity of this agent.
 * @param handle of the Agent to access.
 * @return right shoulder velocity of this agent.
 */
JUPEDSIM_API JPS_Point JPS_HumanoidModelV0State_GetShoulderLeftVelocity(JPS_HumanoidModelV0State handle);

/**
 * Write right shoulder velocity of this agent.
 * @param handle of the Agent to access.
 * @param right shoulder velocity of this agent.
 */
JUPEDSIM_API void
JPS_HumanoidModelV0State_SetShoulderLeftVelocity(JPS_HumanoidModelV0State handle, JPS_Point shoulder_left_velocity);

/**
 * Read pelvis right position of this agent.
 * @param handle of the Agent to access.
 * @return pelvis right position of this agent.
 */
JUPEDSIM_API JPS_Point JPS_HumanoidModelV0State_GetPelvisRightPosition(JPS_HumanoidModelV0State handle);

/**
 * Write pelvis right position of this agent.
 * @param handle of the Agent to access.
 * @param pelvis right position of this agent.
 */
JUPEDSIM_API void
JPS_HumanoidModelV0State_SetPelvisRightPosition(JPS_HumanoidModelV0State handle, JPS_Point pelvis_right_position);

/**
 * Read pelvis right velocity of this agent.
 * @param handle of the Agent to access.
 * @return pelvis right velocity of this agent.
 */
JUPEDSIM_API JPS_Point JPS_HumanoidModelV0State_GetPelvisRightVelocity(JPS_HumanoidModelV0State handle);

/**
 * Write pelvis right velocity of this agent.
 * @param handle of the Agent to access.
 * @param pelvis right velocity of this agent.
 */
JUPEDSIM_API void
JPS_HumanoidModelV0State_SetPelvisRightVelocity(JPS_HumanoidModelV0State handle, JPS_Point pelvis_right_velocity);

/**
 * Read pelvis left position of this agent.
 * @param handle of the Agent to access.
 * @return pelvis left position of this agent.
 */
JUPEDSIM_API JPS_Point JPS_HumanoidModelV0State_GetPelvisLeftPosition(JPS_HumanoidModelV0State handle);

/**
 * Write pelvis left position of this agent.
 * @param handle of the Agent to access.
 * @param pelvis left position of this agent.
 */
JUPEDSIM_API void
JPS_HumanoidModelV0State_SetPelvisLeftPosition(JPS_HumanoidModelV0State handle, JPS_Point pelvis_left_position);

/**
 * Read pelvis left velocity of this agent.
 * @param handle of the Agent to access.
 * @return pelvis left velocity of this agent.
 */
JUPEDSIM_API JPS_Point JPS_HumanoidModelV0State_GetPelvisLeftVelocity(JPS_HumanoidModelV0State handle);

/**
 * Write pelvis left velocity of this agent.
 * @param handle of the Agent to access.
 * @param pelvis left velocity of this agent.
 */
JUPEDSIM_API void
JPS_HumanoidModelV0State_SetPelvisLeftVelocity(JPS_HumanoidModelV0State handle, JPS_Point pelvis_left_velocity);
/**
 * Read heel right position of this agent.
 * @param handle of the Agent to access.
 * @return heel right position of this agent.
 */
JUPEDSIM_API JPS_Point JPS_HumanoidModelV0State_GetHeelRightPosition(JPS_HumanoidModelV0State handle);

/**
 * Write heel right position of this agent.
 * @param handle of the Agent to access.
 * @param heel right position of this agent.
 */
JUPEDSIM_API void
JPS_HumanoidModelV0State_SetHeelRightPosition(JPS_HumanoidModelV0State handle, JPS_Point heel_right_position);

/**
 * Read heel right velocity of this agent.
 * @param handle of the Agent to access.
 * @return heel right velocity of this agent.
 */
JUPEDSIM_API JPS_Point JPS_HumanoidModelV0State_GetHeelRightVelocity(JPS_HumanoidModelV0State handle);

/**
 * Write heel right velocity of this agent.
 * @param handle of the Agent to access.
 * @param heel right velocity of this agent.
 */
JUPEDSIM_API void
JPS_HumanoidModelV0State_SetHeelRightVelocity(JPS_HumanoidModelV0State handle, JPS_Point heel_right_velocity);

/**
 * Read heel left position of this agent.
 * @param handle of the Agent to access.
 * @return heel left position of this agent.
 */
JUPEDSIM_API JPS_Point JPS_HumanoidModelV0State_GetHeelLeftPosition(JPS_HumanoidModelV0State handle);

/**
 * Write heel left position of this agent.
 * @param handle of the Agent to access.
 * @param heel left position of this agent.
 */
JUPEDSIM_API void
JPS_HumanoidModelV0State_SetHeelLeftPosition(JPS_HumanoidModelV0State handle, JPS_Point heel_left_position);

/**
 * Read heel left velocity of this agent.
 * @param handle of the Agent to access.
 * @return heel left velocity of this agent.
 */
JUPEDSIM_API JPS_Point JPS_HumanoidModelV0State_GetHeelLeftVelocity(JPS_HumanoidModelV0State handle);

/**
 * Write heel left velocity of this agent.
 * @param handle of the Agent to access.
 * @param heel left velocity of this agent.
 */
JUPEDSIM_API void
JPS_HumanoidModelV0State_SetHeelLeftVelocity(JPS_HumanoidModelV0State handle, JPS_Point heel_left_velocity);


/**
 * Describes parameters of an Agent in HumanoidModelV0
 */
typedef struct JPS_HumanoidModelV0AgentParameters {
    /**
     * Position of the agent.
     * The position needs to inside the accessible area.
     */
    JPS_Point position{0, 0};
    /*
     * Orientation vector of the agent.
     * The orientation vector will internally be normalized.
     */
    JPS_Point orientation{0, 0};
    /**
     * Defines the journey this agent will take use
     */
    JPS_JourneyId journeyId = 0;
    /**
     * Defines the current stage of its journey
     */
    JPS_StageId stageId = 0;
    /**
     * Initial velocity of the Agent
     */
    JPS_Point velocity = {0, 0};
    /**
     * Mass of the agent
     */
    double mass = 80;
    /**
     * desired Speed of the agent
     */
    double desiredSpeed = 0.8;
    /**
     * reaction Time of the agent
     */
    double reactionTime = 0.5;
    /**
     * agent Scale of the agent
     */
    double agentScale = 2000;
    /**
     * obstacle Scale of the agent
     */
    double obstacleScale = 2000;
    /**
     * force Distance of the agent
     */
    double forceDistance = 0.08;
    /**
     * radius of the agent
     */
    double radius = 0.3;
    /**
     * Humanoid model variables
     */
    JPS_Point head_position = {0, 0}; // Position of the agent's head
    JPS_Point head_velocity = {0, 0}; // Velocity of the agent's head
    JPS_Point shoulder_right_position = {0, 0}; // Position of the agent's right shoulder
    JPS_Point shoulder_right_velocity = {0, 0}; // Velocity of the agent's right shoulder
    JPS_Point shoulder_left_position = {0, 0}; // Position of the agent's left shoulder
    JPS_Point shoulder_left_velocity = {0, 0}; // Velocity of the agent's left shoulder
    JPS_Point pelvis_right_position = {0, 0}; // Position of the agent's right pelvis
    JPS_Point pelvis_right_velocity = {0, 0}; // Velocity of the agent's right pelvis
    JPS_Point pelvis_left_position = {0, 0}; // Position of the agent's left pelvis
    JPS_Point pelvis_left_velocity = {0, 0}; // Velocity of the agent's left pelvis
    JPS_Point heel_right_position = {0, 0}; // Position of the agent's right heel
    JPS_Point heel_right_velocity = {0, 0}; // Velocity of the agent's right heel
    JPS_Point heel_left_position = {0, 0}; // Position of the agent's left heel
    JPS_Point heel_left_velocity = {0, 0}; // Velocity of the agent's left heel



} JPS_HumanoidModelV0AgentParameters;

#ifdef __cplusplus
}
#endif
