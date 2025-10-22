// SPDX-License-Identifier: LGPL-3.0-or-later
#include "jupedsim/social_force_model_IPP.h"
#include "jupedsim/error.h"

#include "Conversion.hpp"
#include "ErrorMessage.hpp"

#include <SocialForceModelIPP.hpp>
#include <SocialForceModelIPPBuilder.hpp>
#include <SocialForceModelIPPData.hpp>

using jupedsim::detail::intoJPS_Point;
using jupedsim::detail::intoPoint;
using jupedsim::detail::intoTuple;

////////////////////////////////////////////////////////////////////////////////////////////////////
/// SocialForceModelIPP Model Builder
////////////////////////////////////////////////////////////////////////////////////////////////////
JUPEDSIM_API JPS_SocialForceModelIPPBuilder
JPS_SocialForceModelIPPBuilder_Create(double bodyForce, double friction)
{
    return reinterpret_cast<JPS_SocialForceModelIPPBuilder>(
        new SocialForceModelIPPBuilder(bodyForce, friction));
}

JUPEDSIM_API JPS_OperationalModel JPS_SocialForceModelIPPBuilder_Build(
    JPS_SocialForceModelIPPBuilder handle,
    JPS_ErrorMessage* errorMessage)
{
    assert(handle != nullptr);
    auto builder = reinterpret_cast<SocialForceModelIPPBuilder*>(handle);
    JPS_OperationalModel result{};
    try {
        result = reinterpret_cast<JPS_OperationalModel>(new SocialForceModelIPP(builder->Build()));
    } catch(const std::exception& ex) {
        if(errorMessage) {
            *errorMessage = reinterpret_cast<JPS_ErrorMessage>(new JPS_ErrorMessage_t{ex.what()});
        }
    } catch(...) {
        if(errorMessage) {
            *errorMessage = reinterpret_cast<JPS_ErrorMessage>(
                new JPS_ErrorMessage_t{"Unknown internal error."});
        }
    }
    return result;
}

JUPEDSIM_API void JPS_SocialForceModelIPPBuilder_Free(JPS_SocialForceModelIPPBuilder handle)
{
    delete reinterpret_cast<SocialForceModelIPPBuilder*>(handle);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// SocialForceModelIPPState
////////////////////////////////////////////////////////////////////////////////////////////////////
JUPEDSIM_API JPS_Point JPS_SocialForceModelIPPState_GetVelocity(JPS_SocialForceModelIPPState handle)
{
    assert(handle);
    const auto state = reinterpret_cast<const SocialForceModelIPPData*>(handle);
    return intoJPS_Point(state->velocity);
}

JUPEDSIM_API void
JPS_SocialForceModelIPPState_SetVelocity(JPS_SocialForceModelIPPState handle, JPS_Point velocity)
{
    assert(handle);
    const auto state = reinterpret_cast<SocialForceModelIPPData*>(handle);
    state->velocity = intoPoint(velocity);
}

JUPEDSIM_API JPS_Point JPS_SocialForceModelIPPState_GetGroundSupportPosition(JPS_SocialForceModelIPPState handle)
{
    assert(handle);
    const auto state = reinterpret_cast<const SocialForceModelIPPData*>(handle);
    return intoJPS_Point(state->ground_support_position);
}

JUPEDSIM_API void
JPS_SocialForceModelIPPState_SetGroundSupportPosition(JPS_SocialForceModelIPPState handle, JPS_Point ground_support_position)
{
    assert(handle);
    const auto state = reinterpret_cast<SocialForceModelIPPData*>(handle);
    state->ground_support_position = intoPoint(ground_support_position);;
}


JUPEDSIM_API JPS_Point JPS_SocialForceModelIPPState_GetGroundSupportVelocity(JPS_SocialForceModelIPPState handle)
{
    assert(handle);
    const auto state = reinterpret_cast<const SocialForceModelIPPData*>(handle);
    return intoJPS_Point(state->ground_support_velocity);
}

JUPEDSIM_API void
JPS_SocialForceModelIPPState_SetGroundSupportVelocity(JPS_SocialForceModelIPPState handle, JPS_Point ground_support_velocity)
{
    assert(handle);
    const auto state = reinterpret_cast<SocialForceModelIPPData*>(handle);
    state->ground_support_velocity = intoPoint(ground_support_velocity);;
}

JUPEDSIM_API double JPS_SocialForceModelIPPState_GetMass(JPS_SocialForceModelIPPState handle)
{
    assert(handle);
    const auto state = reinterpret_cast<const SocialForceModelIPPData*>(handle);
    return state->mass;
}

JUPEDSIM_API void JPS_SocialForceModelIPPState_SetMass(JPS_SocialForceModelIPPState handle, double mass)
{
    assert(handle);
    const auto state = reinterpret_cast<SocialForceModelIPPData*>(handle);
    state->mass = mass;
}

JUPEDSIM_API double JPS_SocialForceModelIPPState_GetDesiredSpeed(JPS_SocialForceModelIPPState handle)
{
    assert(handle);
    const auto state = reinterpret_cast<const SocialForceModelIPPData*>(handle);
    return state->desiredSpeed;
}

JUPEDSIM_API void
JPS_SocialForceModelIPPState_SetDesiredSpeed(JPS_SocialForceModelIPPState handle, double desiredSpeed)
{
    assert(handle);
    const auto state = reinterpret_cast<SocialForceModelIPPData*>(handle);
    state->desiredSpeed = desiredSpeed;
}

JUPEDSIM_API double JPS_SocialForceModelIPPState_GetReactionTime(JPS_SocialForceModelIPPState handle)
{
    assert(handle);
    const auto state = reinterpret_cast<const SocialForceModelIPPData*>(handle);
    return state->reactionTime;
}

JUPEDSIM_API void
JPS_SocialForceModelIPPState_SetReactionTime(JPS_SocialForceModelIPPState handle, double reactionTime)
{
    assert(handle);
    const auto state = reinterpret_cast<SocialForceModelIPPData*>(handle);
    state->reactionTime = reactionTime;
}

JUPEDSIM_API double JPS_SocialForceModelIPPState_GetAgentScale(JPS_SocialForceModelIPPState handle)
{
    assert(handle);
    const auto state = reinterpret_cast<const SocialForceModelIPPData*>(handle);
    return state->agentScale;
}

JUPEDSIM_API void
JPS_SocialForceModelIPPState_SetAgentScale(JPS_SocialForceModelIPPState handle, double agentScale)
{
    assert(handle);
    const auto state = reinterpret_cast<SocialForceModelIPPData*>(handle);
    state->agentScale = agentScale;
}

JUPEDSIM_API double JPS_SocialForceModelIPPState_GetObstacleScale(JPS_SocialForceModelIPPState handle)
{
    assert(handle);
    const auto state = reinterpret_cast<const SocialForceModelIPPData*>(handle);
    return state->obstacleScale;
}

JUPEDSIM_API void
JPS_SocialForceModelIPPState_SetObstacleScale(JPS_SocialForceModelIPPState handle, double obstacleScale)
{
    assert(handle);
    const auto state = reinterpret_cast<SocialForceModelIPPData*>(handle);
    state->obstacleScale = obstacleScale;
}

JUPEDSIM_API double JPS_SocialForceModelIPPState_GetForceDistance(JPS_SocialForceModelIPPState handle)
{
    assert(handle);
    const auto state = reinterpret_cast<const SocialForceModelIPPData*>(handle);
    return state->forceDistance;
}

JUPEDSIM_API void
JPS_SocialForceModelIPPState_SetForceDistance(JPS_SocialForceModelIPPState handle, double forceDistance)
{
    assert(handle);
    const auto state = reinterpret_cast<SocialForceModelIPPData*>(handle);
    state->forceDistance = forceDistance;
}

JUPEDSIM_API double JPS_SocialForceModelIPPState_GetRadius(JPS_SocialForceModelIPPState handle)
{
    assert(handle);
    const auto state = reinterpret_cast<const SocialForceModelIPPData*>(handle);
    return state->radius;
}

JUPEDSIM_API void
JPS_SocialForceModelIPPState_SetRadius(JPS_SocialForceModelIPPState handle, double radius)
{
    assert(handle);
    const auto state = reinterpret_cast<SocialForceModelIPPData*>(handle);
    state->radius = radius;
}
