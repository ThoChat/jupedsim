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
 * Opaque type for a SocialForceModelIPP Builder
 */
typedef struct JPS_SocialForceModelIPPBuilder_t* JPS_SocialForceModelIPPBuilder;

/**
 * Creates a SocialForceModelIPP builder.
 * @param bodyForce describes the strength with which an agent is influenced
 * by pushing forces from obstacles and neighbors in its direct proximity.
 * @param friction describes the strength with which an agent is influenced
 * by frictional forces from obstacles and neighbors in its direct proximity.
 * */
JUPEDSIM_API JPS_SocialForceModelIPPBuilder
JPS_SocialForceModelIPPBuilder_Create(double bodyForce, double friction);

/**
 * Creates a JPS_OperationalModel of type SocialForceModelIPP Model from the
 * JPS_SocialForceModelIPPBuilder.
 * @param handle the builder to operate on
 * @param[out] errorMessage if not NULL: will be set to a JPS_ErrorMessage in case of an error
 * @return a JPS_SocialForceModelIPP or NULL if an error occured.
 */
JUPEDSIM_API JPS_OperationalModel JPS_SocialForceModelIPPBuilder_Build(
    JPS_SocialForceModelIPPBuilder handle,
    JPS_ErrorMessage* errorMessage);

/**
 * Frees a JPS_SocialForceModelIPPBuilder
 * @param handle to the JPS_SocialForceModelIPPBuilder to free.
 */
JUPEDSIM_API void JPS_SocialForceModelIPPBuilder_Free(JPS_SocialForceModelIPPBuilder handle);

/**
 * Opaque type of Social Force model state
 */
typedef struct JPS_SocialForceModelIPPState_t* JPS_SocialForceModelIPPState;

/**
 * Read Velocity of this agent.
 * @param handle of the Agent to access.
 * @return Velocity of this agent.
 */
JUPEDSIM_API JPS_Point JPS_SocialForceModelIPPState_GetVelocity(JPS_SocialForceModelIPPState handle);

/**
 * Write Velocity of this agent.
 * @param handle of the Agent to access.
 * @param velocity Velocity of this agent.
 */
JUPEDSIM_API void
JPS_SocialForceModelIPPState_SetVelocity(JPS_SocialForceModelIPPState handle, JPS_Point velocity);

/**
 * Read ground support position of this agent.
 * @param handle of the Agent to access.
 * @return Ground support position of this agent.
 */
JUPEDSIM_API JPS_Point JPS_SocialForceModelIPPState_GetGroundSupportPosition(JPS_SocialForceModelIPPState handle);

/**
 * Write Ground support position of this agent.
 * @param handle of the Agent to access.
 * @param Ground support position of this agent.
 */
JUPEDSIM_API void
JPS_SocialForceModelIPPState_SetGroundSupportPosition(JPS_SocialForceModelIPPState handle, JPS_Point ground_support_position);


/**
 * Read ground support position of this agent.
 * @param handle of the Agent to access.
 * @return Ground support position of this agent.
 */
JUPEDSIM_API JPS_Point JPS_SocialForceModelIPPState_GetGroundSupportVelocity(JPS_SocialForceModelIPPState handle);

/**
 * Write Ground support position of this agent.
 * @param handle of the Agent to access.
 * @param Ground support position of this agent.
 */
JUPEDSIM_API void
JPS_SocialForceModelIPPState_SetGroundSupportVelocity(JPS_SocialForceModelIPPState handle, JPS_Point ground_support_position);


/**
 * Read mass of this agent.
 * @param handle of the Agent to access.
 * @return mass in kg of this agent
 */
JUPEDSIM_API double JPS_SocialForceModelIPPState_GetMass(JPS_SocialForceModelIPPState handle);

/**
 * Write mass of this agent.
 * @param handle of the Agent to access.
 * @param mass in kg of this agent.
 */
JUPEDSIM_API void JPS_SocialForceModelIPPState_SetMass(JPS_SocialForceModelIPPState handle, double mass);

/**
 * Read desired Speed of this agent.
 * @param handle of the Agent to access.
 * @return desired Speed in m/s of this agent
 */
JUPEDSIM_API double JPS_SocialForceModelIPPState_GetDesiredSpeed(JPS_SocialForceModelIPPState handle);

/**
 * Write desired Speed of this agent.
 * @param handle of the Agent to access.
 * @param desiredSpeed in m/s of this agent.
 */
JUPEDSIM_API void
JPS_SocialForceModelIPPState_SetDesiredSpeed(JPS_SocialForceModelIPPState handle, double desiredSpeed);

/**
 * Read reaction Time of this agent.
 * @param handle of the Agent to access.
 * @return reaction Time in s of this agent
 */
JUPEDSIM_API double JPS_SocialForceModelIPPState_GetReactionTime(JPS_SocialForceModelIPPState handle);

/**
 * Write reaction Time of this agent.
 * @param handle of the Agent to access.
 * @param reactionTime in s of this agent.
 */
JUPEDSIM_API void
JPS_SocialForceModelIPPState_SetReactionTime(JPS_SocialForceModelIPPState handle, double reactionTime);

/**
 * Read agent Scale of this agent.
 * @param handle of the Agent to access.
 * @return agent Scale of this agent
 */
JUPEDSIM_API double JPS_SocialForceModelIPPState_GetAgentScale(JPS_SocialForceModelIPPState handle);

/**
 * Write agent Scale of this agent.
 * @param handle of the Agent to access.
 * @param agentScale of this agent.
 */
JUPEDSIM_API void
JPS_SocialForceModelIPPState_SetAgentScale(JPS_SocialForceModelIPPState handle, double agentScale);

/**
 * Read obstacle Scale of this agent.
 * @param handle of the Agent to access.
 * @return obstacle Scale of this agent
 */
JUPEDSIM_API double JPS_SocialForceModelIPPState_GetObstacleScale(JPS_SocialForceModelIPPState handle);

/**
 * Write obstacle Scale of this agent.
 * @param handle of the Agent to access.
 * @param obstacleScale of this agent.
 */
JUPEDSIM_API void
JPS_SocialForceModelIPPState_SetObstacleScale(JPS_SocialForceModelIPPState handle, double obstacleScale);

/**
 * Read force Distance of this agent.
 * @param handle of the Agent to access.
 * @return force Distance of this agent
 */
JUPEDSIM_API double JPS_SocialForceModelIPPState_GetForceDistance(JPS_SocialForceModelIPPState handle);

/**
 * Write force Distance of this agent.
 * @param handle of the Agent to access.
 * @param forceDistance of this agent.
 */
JUPEDSIM_API void
JPS_SocialForceModelIPPState_SetForceDistance(JPS_SocialForceModelIPPState handle, double forceDistance);

/**
 * Read radius of this agent.
 * @param handle of the Agent to access.
 * @return radius in m of this agent
 */
JUPEDSIM_API double JPS_SocialForceModelIPPState_GetRadius(JPS_SocialForceModelIPPState handle);

/**
 * Write radius of this agent.
 * @param handle of the Agent to access.
 * @param radius in m of this agent.
 */
JUPEDSIM_API void
JPS_SocialForceModelIPPState_SetRadius(JPS_SocialForceModelIPPState handle, double radius);

/**
 * Describes parameters of an Agent in SocialForceModelIPP
 */
typedef struct JPS_SocialForceModelIPPAgentParameters {
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
     * Initial Ground support position of the Agent
     */
    JPS_Point ground_support_position = {0, 0};
    /**
     * Initial Ground support velocity of the Agent
     */
    JPS_Point ground_support_velocity = {0, 0};
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

} JPS_SocialForceModelIPPAgentParameters;

#ifdef __cplusplus
}
#endif
