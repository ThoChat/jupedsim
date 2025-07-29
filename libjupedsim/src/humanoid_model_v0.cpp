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

using jupedsim::detail::intoJPS_Point3D;
using jupedsim::detail::intoPoint3D;
using jupedsim::detail::intoTuple3D;

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

JUPEDSIM_API double JPS_HumanoidModelV0State_GetHeight(JPS_HumanoidModelV0State handle)
{
    assert(handle);
    const auto state = reinterpret_cast<const HumanoidModelV0Data*>(handle);
    return state->height;
}

JUPEDSIM_API void JPS_HumanoidModelV0State_SetHeight(JPS_HumanoidModelV0State handle, double height)
{
    assert(handle);
    const auto state = reinterpret_cast<HumanoidModelV0Data*>(handle);
    state->height = height;
}

JUPEDSIM_API int JPS_HumanoidModelV0State_GetStepDuration (JPS_HumanoidModelV0State handle)
{
    assert(handle);
    const auto state = reinterpret_cast<const HumanoidModelV0Data*>(handle);
    return state->step_duration;
}

JUPEDSIM_API void JPS_HumanoidModelV0State_SetStepDuration(JPS_HumanoidModelV0State handle, int step_duration)
{
    assert(handle);
    const auto state = reinterpret_cast<HumanoidModelV0Data*>(handle);
    state->step_duration = step_duration;
}

JUPEDSIM_API int JPS_HumanoidModelV0State_GetStepTimer (JPS_HumanoidModelV0State handle)
{
    assert(handle);
    const auto state = reinterpret_cast<const HumanoidModelV0Data*>(handle);
    return state->step_timer;
}

JUPEDSIM_API void JPS_HumanoidModelV0State_SetStepTimer(JPS_HumanoidModelV0State handle, int step_timer)
{
    assert(handle);
    const auto state = reinterpret_cast<HumanoidModelV0Data*>(handle);
    state->step_timer = step_timer;
}

JUPEDSIM_API int JPS_HumanoidModelV0State_GetSteppingFootIndex(JPS_HumanoidModelV0State handle)
{
    assert(handle);
    const auto state = reinterpret_cast<const HumanoidModelV0Data*>(handle);
    return state->stepping_foot_index;
}

JUPEDSIM_API void JPS_HumanoidModelV0State_SetSteppingFootIndex(JPS_HumanoidModelV0State handle, int stepping_foot_index)
{
    assert(handle);
    const auto state = reinterpret_cast<HumanoidModelV0Data*>(handle);
    state->stepping_foot_index = stepping_foot_index;
}


JUPEDSIM_API JPS_Point JPS_HumanoidModelV0State_GetStepTarget(JPS_HumanoidModelV0State handle)
{
    assert(handle);
    const auto state = reinterpret_cast<const HumanoidModelV0Data*>(handle);
    return intoJPS_Point(state->step_target);
}

JUPEDSIM_API void JPS_HumanoidModelV0State_SetStepTarget(JPS_HumanoidModelV0State handle, JPS_Point step_target)
{
    assert(handle);
    const auto state = reinterpret_cast<HumanoidModelV0Data*>(handle);
    state->step_target = intoPoint(step_target);
}


JUPEDSIM_API JPS_Point3D JPS_HumanoidModelV0State_GetHeadPosition(JPS_HumanoidModelV0State handle)
{
    assert(handle);
    const auto state = reinterpret_cast<const HumanoidModelV0Data*>(handle);
    return intoJPS_Point3D(state->head_position);
}

JUPEDSIM_API void
JPS_HumanoidModelV0State_SetHeadPosition(JPS_HumanoidModelV0State handle, JPS_Point3D head_position)
{
    assert(handle);
    const auto state = reinterpret_cast<HumanoidModelV0Data*>(handle);
    state->head_position = intoPoint3D(head_position);
}

JUPEDSIM_API JPS_Point3D JPS_HumanoidModelV0State_GetPelvisPosition(JPS_HumanoidModelV0State handle)
{
    assert(handle);
    const auto state = reinterpret_cast<const HumanoidModelV0Data*>(handle);
    return intoJPS_Point3D(state->pelvis_position);
}

JUPEDSIM_API void
JPS_HumanoidModelV0State_SetPelvisPosition(JPS_HumanoidModelV0State handle, JPS_Point3D pelvis_position)
{
    assert(handle);
    const auto state = reinterpret_cast<HumanoidModelV0Data*>(handle);
    state->pelvis_position = intoPoint3D(pelvis_position);
}

JUPEDSIM_API double JPS_HumanoidModelV0State_GetPelvisRotationAngleZ(JPS_HumanoidModelV0State handle)
{
    assert(handle);
    const auto state = reinterpret_cast<const HumanoidModelV0Data*>(handle);
    return state->pelvis_rotation_angle_z;
}

JUPEDSIM_API void
JPS_HumanoidModelV0State_SetPelvisRotationAngleZ(JPS_HumanoidModelV0State handle, double pelvis_rotation_angle_z)
{
    assert(handle);
    const auto state = reinterpret_cast<HumanoidModelV0Data*>(handle);
    state->pelvis_rotation_angle_z = pelvis_rotation_angle_z;
}

JUPEDSIM_API double JPS_HumanoidModelV0State_GetShoulderRotationAngleZ(JPS_HumanoidModelV0State handle)
{
    assert(handle);
    const auto state = reinterpret_cast<const HumanoidModelV0Data*>(handle);
    return state->shoulder_rotation_angle_z;
}

JUPEDSIM_API void
JPS_HumanoidModelV0State_SetShoulderRotationAngleZ(JPS_HumanoidModelV0State handle, double shoulder_rotation_angle_z)
{
    assert(handle);
    const auto state = reinterpret_cast<HumanoidModelV0Data*>(handle);
    state->shoulder_rotation_angle_z = shoulder_rotation_angle_z;
}


JUPEDSIM_API double JPS_HumanoidModelV0State_GetTrunkRotationAngleX(JPS_HumanoidModelV0State handle)
{
    assert(handle);
    const auto state = reinterpret_cast<const HumanoidModelV0Data*>(handle);
    return state->trunk_rotation_angle_x;
}

JUPEDSIM_API void
JPS_HumanoidModelV0State_SetTrunkRotationAngleX(JPS_HumanoidModelV0State handle, double trunk_rotation_angle_x)
{
    assert(handle);
    const auto state = reinterpret_cast<HumanoidModelV0Data*>(handle);
    state->trunk_rotation_angle_x = trunk_rotation_angle_x;
}


JUPEDSIM_API double JPS_HumanoidModelV0State_GetTrunkRotationAngleY(JPS_HumanoidModelV0State handle)
{
    assert(handle);
    const auto state = reinterpret_cast<const HumanoidModelV0Data*>(handle);
    return state->trunk_rotation_angle_y;
}

JUPEDSIM_API void
JPS_HumanoidModelV0State_SetTrunkRotationAngleY(JPS_HumanoidModelV0State handle, double trunk_rotation_angle_y)
{
    assert(handle);
    const auto state = reinterpret_cast<HumanoidModelV0Data*>(handle);
    state->trunk_rotation_angle_y = trunk_rotation_angle_y;
}


JUPEDSIM_API JPS_Point3D JPS_HumanoidModelV0State_GetHeelRightPosition(JPS_HumanoidModelV0State handle) { assert(handle); const auto state = reinterpret_cast<const HumanoidModelV0Data*>(handle); return intoJPS_Point3D(state->heel_right_position); }

JUPEDSIM_API void JPS_HumanoidModelV0State_SetHeelRightPosition(JPS_HumanoidModelV0State handle, JPS_Point3D heel_right_position) { assert(handle); const auto state = reinterpret_cast<HumanoidModelV0Data*>(handle); state->heel_right_position = intoPoint3D(heel_right_position); }


JUPEDSIM_API JPS_Point3D JPS_HumanoidModelV0State_GetHeelLeftPosition(JPS_HumanoidModelV0State handle) { assert(handle); const auto state = reinterpret_cast<const HumanoidModelV0Data*>(handle); return intoJPS_Point3D(state->heel_left_position); }

JUPEDSIM_API void JPS_HumanoidModelV0State_SetHeelLeftPosition(JPS_HumanoidModelV0State handle, JPS_Point3D heel_left_position) { assert(handle); const auto state = reinterpret_cast<HumanoidModelV0Data*>(handle); state->heel_left_position = intoPoint3D(heel_left_position); }

JUPEDSIM_API JPS_Point3D JPS_HumanoidModelV0State_GetToeRightPosition(JPS_HumanoidModelV0State handle) { assert(handle); const auto state = reinterpret_cast<const HumanoidModelV0Data*>(handle); return intoJPS_Point3D(state->toe_right_position); }

JUPEDSIM_API void JPS_HumanoidModelV0State_SetToeRightPosition(JPS_HumanoidModelV0State handle, JPS_Point3D toe_right_position) { assert(handle); const auto state = reinterpret_cast<HumanoidModelV0Data*>(handle); state->toe_right_position = intoPoint3D(toe_right_position); }

JUPEDSIM_API JPS_Point3D JPS_HumanoidModelV0State_GetToeLeftPosition(JPS_HumanoidModelV0State handle) { assert(handle); const auto state = reinterpret_cast<const HumanoidModelV0Data*>(handle); return intoJPS_Point3D(state->toe_left_position); }

JUPEDSIM_API void JPS_HumanoidModelV0State_SetToeLeftPosition(JPS_HumanoidModelV0State handle, JPS_Point3D toe_left_position) { assert(handle); const auto state = reinterpret_cast<HumanoidModelV0Data*>(handle); state->toe_left_position = intoPoint3D(toe_left_position); }
