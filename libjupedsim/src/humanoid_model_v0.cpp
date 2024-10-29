// Copyright © 2012-2024 Forschungszentrum Jülich GmbH
// SPDX-License-Identifier: LGPL-3.0-or-later
#include "jupedsim/humanoid_model_v0.h"
#include "jupedsim/error.h"

#include "Conversion.hpp"
#include "ErrorMessage.hpp"

#include <HumanoidModelV0.hpp>
#include <HumanoidModelV0Builder.hpp>
#include <HumanoidModelV0Data.hpp>

using jupedsim::detail::intoJPS_Point;
using jupedsim::detail::intoPoint;
using jupedsim::detail::intoTuple;

////////////////////////////////////////////////////////////////////////////////////////////////////
/// HumanoidModelV0 Model Builder
////////////////////////////////////////////////////////////////////////////////////////////////////
JUPEDSIM_API JPS_HumanoidModelV0Builder
JPS_HumanoidModelV0Builder_Create(double bodyForce, double friction)
{
    return reinterpret_cast<JPS_HumanoidModelV0Builder>(
        new HumanoidModelV0Builder(bodyForce, friction));
}

JUPEDSIM_API JPS_OperationalModel JPS_HumanoidModelV0Builder_Build(
    JPS_HumanoidModelV0Builder handle,
    JPS_ErrorMessage* errorMessage)
{
    assert(handle != nullptr);
    auto builder = reinterpret_cast<HumanoidModelV0Builder*>(handle);
    JPS_OperationalModel result{};
    try {
        result = reinterpret_cast<JPS_OperationalModel>(new HumanoidModelV0(builder->Build()));
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

JUPEDSIM_API void JPS_HumanoidModelV0Builder_Free(JPS_HumanoidModelV0Builder handle)
{
    delete reinterpret_cast<HumanoidModelV0Builder*>(handle);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// HumanoidModelV0State
////////////////////////////////////////////////////////////////////////////////////////////////////
JUPEDSIM_API JPS_Point JPS_HumanoidModelV0State_GetVelocity(JPS_HumanoidModelV0State handle)
{
    assert(handle);
    const auto state = reinterpret_cast<const HumanoidModelV0Data*>(handle);
    return intoJPS_Point(state->velocity);
}

JUPEDSIM_API void
JPS_HumanoidModelV0State_SetVelocity(JPS_HumanoidModelV0State handle, JPS_Point velocity)
{
    assert(handle);
    const auto state = reinterpret_cast<HumanoidModelV0Data*>(handle);
    state->velocity = intoPoint(velocity);
}

JUPEDSIM_API double JPS_HumanoidModelV0State_GetMass(JPS_HumanoidModelV0State handle)
{
    assert(handle);
    const auto state = reinterpret_cast<const HumanoidModelV0Data*>(handle);
    return state->mass;
}

JUPEDSIM_API void JPS_HumanoidModelV0State_SetMass(JPS_HumanoidModelV0State handle, double mass)
{
    assert(handle);
    const auto state = reinterpret_cast<HumanoidModelV0Data*>(handle);
    state->mass = mass;
}

JUPEDSIM_API double JPS_HumanoidModelV0State_GetDesiredSpeed(JPS_HumanoidModelV0State handle)
{
    assert(handle);
    const auto state = reinterpret_cast<const HumanoidModelV0Data*>(handle);
    return state->desiredSpeed;
}

JUPEDSIM_API void
JPS_HumanoidModelV0State_SetDesiredSpeed(JPS_HumanoidModelV0State handle, double desiredSpeed)
{
    assert(handle);
    const auto state = reinterpret_cast<HumanoidModelV0Data*>(handle);
    state->desiredSpeed = desiredSpeed;
}

JUPEDSIM_API double JPS_HumanoidModelV0State_GetReactionTime(JPS_HumanoidModelV0State handle)
{
    assert(handle);
    const auto state = reinterpret_cast<const HumanoidModelV0Data*>(handle);
    return state->reactionTime;
}

JUPEDSIM_API void
JPS_HumanoidModelV0State_SetReactionTime(JPS_HumanoidModelV0State handle, double reactionTime)
{
    assert(handle);
    const auto state = reinterpret_cast<HumanoidModelV0Data*>(handle);
    state->reactionTime = reactionTime;
}

JUPEDSIM_API double JPS_HumanoidModelV0State_GetAgentScale(JPS_HumanoidModelV0State handle)
{
    assert(handle);
    const auto state = reinterpret_cast<const HumanoidModelV0Data*>(handle);
    return state->agentScale;
}

JUPEDSIM_API void
JPS_HumanoidModelV0State_SetAgentScale(JPS_HumanoidModelV0State handle, double agentScale)
{
    assert(handle);
    const auto state = reinterpret_cast<HumanoidModelV0Data*>(handle);
    state->agentScale = agentScale;
}

JUPEDSIM_API double JPS_HumanoidModelV0State_GetObstacleScale(JPS_HumanoidModelV0State handle)
{
    assert(handle);
    const auto state = reinterpret_cast<const HumanoidModelV0Data*>(handle);
    return state->obstacleScale;
}

JUPEDSIM_API void
JPS_HumanoidModelV0State_SetObstacleScale(JPS_HumanoidModelV0State handle, double obstacleScale)
{
    assert(handle);
    const auto state = reinterpret_cast<HumanoidModelV0Data*>(handle);
    state->obstacleScale = obstacleScale;
}

JUPEDSIM_API double JPS_HumanoidModelV0State_GetForceDistance(JPS_HumanoidModelV0State handle)
{
    assert(handle);
    const auto state = reinterpret_cast<const HumanoidModelV0Data*>(handle);
    return state->forceDistance;
}

JUPEDSIM_API void
JPS_HumanoidModelV0State_SetForceDistance(JPS_HumanoidModelV0State handle, double forceDistance)
{
    assert(handle);
    const auto state = reinterpret_cast<HumanoidModelV0Data*>(handle);
    state->forceDistance = forceDistance;
}

JUPEDSIM_API double JPS_HumanoidModelV0State_GetRadius(JPS_HumanoidModelV0State handle)
{
    assert(handle);
    const auto state = reinterpret_cast<const HumanoidModelV0Data*>(handle);
    return state->radius;
}

JUPEDSIM_API void
JPS_HumanoidModelV0State_SetRadius(JPS_HumanoidModelV0State handle, double radius)
{
    assert(handle);
    const auto state = reinterpret_cast<HumanoidModelV0Data*>(handle);
    state->radius = radius;
}

JUPEDSIM_API JPS_Point JPS_HumanoidModelV0State_GetHeadPosition(JPS_HumanoidModelV0State handle)
{
    assert(handle);
    const auto state = reinterpret_cast<const HumanoidModelV0Data*>(handle);
    return intoJPS_Point(state->head_position);
}

JUPEDSIM_API void
JPS_HumanoidModelV0State_SetHeadPosition(JPS_HumanoidModelV0State handle, JPS_Point head_position)
{
    assert(handle);
    const auto state = reinterpret_cast<HumanoidModelV0Data*>(handle);
    state->head_position = intoPoint(head_position);
}

JUPEDSIM_API JPS_Point JPS_HumanoidModelV0State_GetHeadVelocity(JPS_HumanoidModelV0State handle)
{
    assert(handle);
    const auto state = reinterpret_cast<const HumanoidModelV0Data*>(handle);
    return intoJPS_Point(state->head_velocity);
}

JUPEDSIM_API void
JPS_HumanoidModelV0State_SetHeadVelocity(JPS_HumanoidModelV0State handle, JPS_Point head_velocity)
{
    assert(handle);
    const auto state = reinterpret_cast<HumanoidModelV0Data*>(handle);
    state->head_velocity = intoPoint(head_velocity);
}

JUPEDSIM_API JPS_Point JPS_HumanoidModelV0State_GetShoulderRightPosition(JPS_HumanoidModelV0State handle)
{
    assert(handle);
    const auto state = reinterpret_cast<const HumanoidModelV0Data*>(handle);
    return intoJPS_Point(state->shoulder_right_position);
}

JUPEDSIM_API void
JPS_HumanoidModelV0State_SetShoulderRightPosition(JPS_HumanoidModelV0State handle, JPS_Point shoulder_right_position)
{
    assert(handle);
    const auto state = reinterpret_cast<HumanoidModelV0Data*>(handle);
    state->shoulder_right_position = intoPoint(shoulder_right_position);
}

JUPEDSIM_API JPS_Point JPS_HumanoidModelV0State_GetShoulderRightVelocity(JPS_HumanoidModelV0State handle)
{
    assert(handle);
    const auto state = reinterpret_cast<const HumanoidModelV0Data*>(handle);
    return intoJPS_Point(state->shoulder_right_velocity);
}

JUPEDSIM_API void
JPS_HumanoidModelV0State_SetShoulderRightVelocity(JPS_HumanoidModelV0State handle, JPS_Point shoulder_right_velocity)
{
    assert(handle);
    const auto state = reinterpret_cast<HumanoidModelV0Data*>(handle);
    state->shoulder_right_velocity = intoPoint(shoulder_right_velocity);
}

JUPEDSIM_API JPS_Point JPS_HumanoidModelV0State_GetShoulderLeftPosition(JPS_HumanoidModelV0State handle)
{
    assert(handle);
    const auto state = reinterpret_cast<const HumanoidModelV0Data*>(handle);
    return intoJPS_Point(state->shoulder_left_position);
}

JUPEDSIM_API void
JPS_HumanoidModelV0State_SetShoulderLeftPosition(JPS_HumanoidModelV0State handle, JPS_Point shoulder_left_position)
{
    assert(handle);
    const auto state = reinterpret_cast<HumanoidModelV0Data*>(handle);
    state->shoulder_left_position = intoPoint(shoulder_left_position);
}

JUPEDSIM_API JPS_Point JPS_HumanoidModelV0State_GetShoulderLeftVelocity(JPS_HumanoidModelV0State handle)
{
    assert(handle);
    const auto state = reinterpret_cast<const HumanoidModelV0Data*>(handle);
    return intoJPS_Point(state->shoulder_left_velocity);
}

JUPEDSIM_API void
JPS_HumanoidModelV0State_SetShoulderLeftVelocity(JPS_HumanoidModelV0State handle, JPS_Point shoulder_left_velocity)
{
    assert(handle);
    const auto state = reinterpret_cast<HumanoidModelV0Data*>(handle);
    state->shoulder_left_velocity = intoPoint(shoulder_left_velocity);
}

JUPEDSIM_API JPS_Point JPS_HumanoidModelV0State_GetPelvisRightPosition(JPS_HumanoidModelV0State handle) { assert(handle); const auto state = reinterpret_cast<const HumanoidModelV0Data*>(handle); return intoJPS_Point(state->pelvis_right_position); }

JUPEDSIM_API void JPS_HumanoidModelV0State_SetPelvisRightPosition(JPS_HumanoidModelV0State handle, JPS_Point pelvis_right_position) { assert(handle); const auto state = reinterpret_cast<HumanoidModelV0Data*>(handle); state->pelvis_right_position = intoPoint(pelvis_right_position); }

JUPEDSIM_API JPS_Point JPS_HumanoidModelV0State_GetPelvisRightVelocity(JPS_HumanoidModelV0State handle) { assert(handle); const auto state = reinterpret_cast<const HumanoidModelV0Data*>(handle); return intoJPS_Point(state->pelvis_right_velocity); }

JUPEDSIM_API void JPS_HumanoidModelV0State_SetPelvisRightVelocity(JPS_HumanoidModelV0State handle, JPS_Point pelvis_right_velocity) { assert(handle); const auto state = reinterpret_cast<HumanoidModelV0Data*>(handle); state->pelvis_right_velocity = intoPoint(pelvis_right_velocity); }

JUPEDSIM_API JPS_Point JPS_HumanoidModelV0State_GetPelvisLeftPosition(JPS_HumanoidModelV0State handle) { assert(handle); const auto state = reinterpret_cast<const HumanoidModelV0Data*>(handle); return intoJPS_Point(state->pelvis_left_position); }

JUPEDSIM_API void JPS_HumanoidModelV0State_SetPelvisLeftPosition(JPS_HumanoidModelV0State handle, JPS_Point pelvis_left_position) { assert(handle); const auto state = reinterpret_cast<HumanoidModelV0Data*>(handle); state->pelvis_left_position = intoPoint(pelvis_left_position); }

JUPEDSIM_API JPS_Point JPS_HumanoidModelV0State_GetPelvisLeftVelocity(JPS_HumanoidModelV0State handle) { assert(handle); const auto state = reinterpret_cast<const HumanoidModelV0Data*>(handle); return intoJPS_Point(state->pelvis_left_velocity); }

JUPEDSIM_API void JPS_HumanoidModelV0State_SetPelvisLeftVelocity(JPS_HumanoidModelV0State handle, JPS_Point pelvis_left_velocity) { assert(handle); const auto state = reinterpret_cast<HumanoidModelV0Data*>(handle); state->pelvis_left_velocity = intoPoint(pelvis_left_velocity); }

JUPEDSIM_API JPS_Point JPS_HumanoidModelV0State_GetHeelRightPosition(JPS_HumanoidModelV0State handle) { assert(handle); const auto state = reinterpret_cast<const HumanoidModelV0Data*>(handle); return intoJPS_Point(state->heel_right_position); }

JUPEDSIM_API void JPS_HumanoidModelV0State_SetHeelRightPosition(JPS_HumanoidModelV0State handle, JPS_Point heel_right_position) { assert(handle); const auto state = reinterpret_cast<HumanoidModelV0Data*>(handle); state->heel_right_position = intoPoint(heel_right_position); }

JUPEDSIM_API JPS_Point JPS_HumanoidModelV0State_GetHeelRightVelocity(JPS_HumanoidModelV0State handle) { assert(handle); const auto state = reinterpret_cast<const HumanoidModelV0Data*>(handle); return intoJPS_Point(state->heel_right_velocity); }

JUPEDSIM_API void JPS_HumanoidModelV0State_SetHeelRightVelocity(JPS_HumanoidModelV0State handle, JPS_Point heel_right_velocity) { assert(handle); const auto state = reinterpret_cast<HumanoidModelV0Data*>(handle); state->heel_right_velocity = intoPoint(heel_right_velocity); }

JUPEDSIM_API JPS_Point JPS_HumanoidModelV0State_GetHeelLeftPosition(JPS_HumanoidModelV0State handle) { assert(handle); const auto state = reinterpret_cast<const HumanoidModelV0Data*>(handle); return intoJPS_Point(state->heel_left_position); }

JUPEDSIM_API void JPS_HumanoidModelV0State_SetHeelLeftPosition(JPS_HumanoidModelV0State handle, JPS_Point heel_left_position) { assert(handle); const auto state = reinterpret_cast<HumanoidModelV0Data*>(handle); state->heel_left_position = intoPoint(heel_left_position); }

JUPEDSIM_API JPS_Point JPS_HumanoidModelV0State_GetHeelLeftVelocity(JPS_HumanoidModelV0State handle) { assert(handle); const auto state = reinterpret_cast<const HumanoidModelV0Data*>(handle); return intoJPS_Point(state->heel_left_velocity); }

JUPEDSIM_API void JPS_HumanoidModelV0State_SetHeelLeftVelocity(JPS_HumanoidModelV0State handle, JPS_Point heel_left_velocity) { assert(handle); const auto state = reinterpret_cast<HumanoidModelV0Data*>(handle); state->heel_left_velocity = intoPoint(heel_left_velocity); }
