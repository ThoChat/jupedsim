// SPDX-License-Identifier: LGPL-3.0-or-later
#include "GenericAgent.hpp"
#include "GeometryBuilder.hpp"
#include "SocialForceModelIPP.hpp"
#include "SocialForceModelIPPData.hpp"
#include "SocialForceModelIPPUpdate.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace
{
// Default parameters matching the paper's density-wave setting (Table 1, supplementary)
// lambda_u = 0.5, lambda_b = 1.0, v = 1.0, A = 5.0, B = 0.5, B_leg = 0.3, lambda = 1.0
SocialForceModelIPPData MakeData(
    Point velocity = {0, 0},
    Point gs_pos = {0, 0},
    Point gs_vel = {0, 0},
    double height = 1.75,
    double radius = 0.15)
{
    SocialForceModelIPPData data{};
    data.velocity = velocity;
    data.ground_support_position = gs_pos;
    data.ground_support_velocity = gs_vel;
    data.height = height;
    data.desiredSpeed = 1.0;
    data.reactionTime = 0.5;
    data.lambdaU = 0.5;
    data.lambdaB = 1.0;
    data.balanceSpeed = 1.0;
    data.damping = 1.0;
    data.agentScale = 5.0;
    data.obstacleScale = 5.0;
    data.forceDistance = 0.5;
    data.obstacleForceDistance = 0.3;
    data.legForceDistance = 0.3;
    data.radius = radius;
    return data;
}

GenericAgent MakeAgent(
    Point pos,
    Point velocity = {0, 0},
    Point gs_pos = {0, 0},
    Point gs_vel = {0, 0},
    double height = 1.75,
    double radius = 0.15)
{
    auto data = MakeData(velocity, gs_pos, gs_vel, height, radius);
    return GenericAgent(
        GenericAgent::ID{},
        jps::UniqueID<Journey>::Invalid,
        jps::UniqueID<BaseStage>::Invalid,
        pos,
        Point(1, 0),
        std::move(data));
}

// Build a large box geometry so walls are far away and don't affect tests
CollisionGeometry MakeLargeBox()
{
    GeometryBuilder builder;
    builder.AddAccessibleArea({{-50, -50}, {50, -50}, {50, 50}, {-50, 50}});
    return builder.Build();
}

// Helper: hand-compute one Euler step from Eq. 9 (supplementary material)
//
//   v_n(t+dt) = v_n(t) + dt * [ driving + lambda_u*(v*e_n - v_n) - lambda*v_n - sum grad_V ]
//   x_n(t+dt) = x_n(t) + dt * v_n(t+dt)
//
//   v_leg(t+dt) = v_leg(t) + dt * [ lambda_b*(v*e_n - v_leg) - sum grad_V_leg ]
//   x_leg(t+dt) = x_leg(t) + dt * v_leg(t+dt)
struct ExpectedState {
    Point ub_vel;
    Point ub_pos;
    Point gs_vel;
    Point gs_pos;
};

ExpectedState HandCompute(
    const GenericAgent& agent,
    double dT,
    const std::vector<GenericAgent>& neighbors = {})
{
    const auto& m = std::get<SocialForceModelIPPData>(agent.model);

    // e_n: unit vector from legs toward upper body
    const Point sep = agent.pos - m.ground_support_position;
    Point e_n;
    if(sep.Norm() > 1e-10) {
        e_n = sep.Normalized();
    } else {
        const Point toGoal = agent.destination - agent.pos;
        e_n = (toGoal.Norm() > 1e-10) ? toGoal.Normalized() : Point(0, 0);
    }

    // Driving force: (v0 * e0 - v) / tau
    const Point e0 = (agent.destination - agent.pos).Normalized();
    Point acc_ub = (e0 * m.desiredSpeed - m.velocity) / m.reactionTime;

    // Unbalancing: lambda_u * (v_s * e_n - v)
    acc_ub = acc_ub + (e_n * m.balanceSpeed - m.velocity) * m.lambdaU;

    // Damping: -lambda * v
    acc_ub = acc_ub - m.velocity * m.damping;

    // Upper body repulsion from neighbors
    for(const auto& nb : neighbors) {
        const double dist = (agent.pos - nb.pos).Norm();
        if(dist > 1e-10) {
            const Point n_ij = (agent.pos - nb.pos).Normalized();
            acc_ub = acc_ub + n_ij * m.agentScale * std::exp(-dist / m.forceDistance);
        }
    }

    // Leg acceleration
    // Balance recovery: lambda_b * (v_s * e_n - v_leg)
    Point acc_gs = (e_n * m.balanceSpeed - m.ground_support_velocity) * m.lambdaB;

    // Leg repulsion from neighbors' legs
    for(const auto& nb : neighbors) {
        const auto& nm = std::get<SocialForceModelIPPData>(nb.model);
        const double dist = (m.ground_support_position - nm.ground_support_position).Norm();
        if(dist > 1e-10) {
            const Point n_ij = (m.ground_support_position - nm.ground_support_position).Normalized();
            acc_gs = acc_gs + n_ij * m.agentScale * std::exp(-dist / m.legForceDistance);
        }
    }

    // Euler integration (Eq. 9)
    ExpectedState out;
    out.ub_vel = m.velocity + acc_ub * dT;
    out.ub_pos = agent.pos + out.ub_vel * dT;
    out.gs_vel = m.ground_support_velocity + acc_gs * dT;
    out.gs_pos = m.ground_support_position + out.gs_vel * dT;
    return out;
}

constexpr double TOL = 1e-10;
} // namespace

// --------------------------------------------------------------------------
// Original unit tests
// --------------------------------------------------------------------------

TEST(SocialForceModelIPP, DrivingForcePointsTowardDestination)
{
    auto agent = MakeAgent(Point(0, 0));
    agent.destination = Point(10, 0);

    // Driving force = (e0 * v0 - v) / tau
    // e0 = (1,0), v0 = 1.0, v = (0,0), tau = 0.5
    // F = (1,0) * 1.0 / 0.5 = (2, 0)
    const auto& model = std::get<SocialForceModelIPPData>(agent.model);
    const Point e0 = (agent.destination - agent.pos).Normalized();
    const Point expected = (e0 * model.desiredSpeed - model.velocity) / model.reactionTime;

    EXPECT_DOUBLE_EQ(expected.x, 2.0);
    EXPECT_DOUBLE_EQ(expected.y, 0.0);
}

TEST(SocialForceModelIPP, ExponentialRepulsionIsPositive)
{
    const double A = 5.0;
    const double B = 0.5;
    const double dist = 1.0;
    const double force = A * std::exp(-dist / B);
    EXPECT_GT(force, 0.0);
}

TEST(SocialForceModelIPP, ExponentialRepulsionDecaysWithDistance)
{
    const double A = 5.0;
    const double B = 0.5;
    const double force_near = A * std::exp(-0.5 / B);
    const double force_far = A * std::exp(-2.0 / B);
    EXPECT_GT(force_near, force_far);
}

TEST(SocialForceModelIPP, LegForceDistanceShorterThanUpperBody)
{
    auto agent = MakeAgent(Point(0, 0));
    const auto& model = std::get<SocialForceModelIPPData>(agent.model);
    EXPECT_LT(model.legForceDistance, model.forceDistance);
}

TEST(SocialForceModelIPP, GroundSupportScalingFactorIsPositive)
{
    EXPECT_GT(SocialForceModelIPP::GS_SCALING_FACTOR, 0.0);
    EXPECT_LT(SocialForceModelIPP::GS_SCALING_FACTOR, 1.0);
}

// --------------------------------------------------------------------------
// Integration tests: ComputeNewPosition vs hand-computed Eq. 9
// --------------------------------------------------------------------------

// Isolated agent at rest: only driving force + unbalancing + damping act.
// Upper body and legs co-located at origin, destination along +x.
TEST(SocialForceModelIPP, SingleAgentAtRestColocated)
{
    const double dT = 0.01;
    auto geometry = MakeLargeBox();
    SocialForceModelIPP model;
    NeighborhoodSearch<GenericAgent> ns(3);

    // Upper body and legs both at (5,5), velocity zero
    auto agent = MakeAgent(Point(5, 5), {0, 0}, Point(5, 5), {0, 0});
    agent.destination = Point(15, 5);
    ns.Update({agent});

    const auto update = model.ComputeNewPosition(dT, agent, geometry, ns);
    const auto& upd = std::get<SocialForceModelIPPUpdate>(update);
    const auto expected = HandCompute(agent, dT);

    EXPECT_NEAR(upd.velocity.x, expected.ub_vel.x, TOL);
    EXPECT_NEAR(upd.velocity.y, expected.ub_vel.y, TOL);
    EXPECT_NEAR(upd.position.x, expected.ub_pos.x, TOL);
    EXPECT_NEAR(upd.position.y, expected.ub_pos.y, TOL);
    EXPECT_NEAR(upd.ground_support_velocity.x, expected.gs_vel.x, TOL);
    EXPECT_NEAR(upd.ground_support_velocity.y, expected.gs_vel.y, TOL);
    EXPECT_NEAR(upd.ground_support_position.x, expected.gs_pos.x, TOL);
    EXPECT_NEAR(upd.ground_support_position.y, expected.gs_pos.y, TOL);

    // With co-located body/legs and zero velocity, e_n falls back to destination direction (+x).
    // Driving force pushes +x, unbalancing pushes +x, damping is zero → agent accelerates +x.
    EXPECT_GT(upd.velocity.x, 0.0);
    EXPECT_NEAR(upd.velocity.y, 0.0, TOL);
}

// Isolated agent with offset legs: upper body at (5,5), legs at (4.8,5).
// e_n points in +x, non-zero separation engages the coupling terms.
TEST(SocialForceModelIPP, SingleAgentWithOffsetLegs)
{
    const double dT = 0.01;
    auto geometry = MakeLargeBox();
    SocialForceModelIPP model;
    NeighborhoodSearch<GenericAgent> ns(3);

    auto agent = MakeAgent(Point(5, 5), {0.5, 0}, Point(4.8, 5), {0.3, 0});
    agent.destination = Point(15, 5);
    ns.Update({agent});

    const auto update = model.ComputeNewPosition(dT, agent, geometry, ns);
    const auto& upd = std::get<SocialForceModelIPPUpdate>(update);
    const auto expected = HandCompute(agent, dT);

    EXPECT_NEAR(upd.velocity.x, expected.ub_vel.x, TOL);
    EXPECT_NEAR(upd.velocity.y, expected.ub_vel.y, TOL);
    EXPECT_NEAR(upd.position.x, expected.ub_pos.x, TOL);
    EXPECT_NEAR(upd.position.y, expected.ub_pos.y, TOL);
    EXPECT_NEAR(upd.ground_support_velocity.x, expected.gs_vel.x, TOL);
    EXPECT_NEAR(upd.ground_support_velocity.y, expected.gs_vel.y, TOL);
    EXPECT_NEAR(upd.ground_support_position.x, expected.gs_pos.x, TOL);
    EXPECT_NEAR(upd.ground_support_position.y, expected.gs_pos.y, TOL);
}

// Two agents facing each other: verify upper-body and leg repulsion are accounted for.
TEST(SocialForceModelIPP, TwoAgentRepulsion)
{
    const double dT = 0.01;
    auto geometry = MakeLargeBox();
    SocialForceModelIPP model;
    NeighborhoodSearch<GenericAgent> ns(3);

    // Agent A at (5,5), agent B at (5.5,5) — close together
    auto agentA = MakeAgent(Point(5, 5), {0, 0}, Point(5, 5), {0, 0});
    agentA.destination = Point(15, 5);
    auto agentB = MakeAgent(Point(5.5, 5), {0, 0}, Point(5.5, 5), {0, 0});
    agentB.destination = Point(-5, 5);
    ns.Update({agentA, agentB});

    const auto updateA = model.ComputeNewPosition(dT, agentA, geometry, ns);
    const auto& updA = std::get<SocialForceModelIPPUpdate>(updateA);
    const auto expectedA = HandCompute(agentA, dT, {agentB});

    EXPECT_NEAR(updA.velocity.x, expectedA.ub_vel.x, TOL);
    EXPECT_NEAR(updA.velocity.y, expectedA.ub_vel.y, TOL);
    EXPECT_NEAR(updA.position.x, expectedA.ub_pos.x, TOL);
    EXPECT_NEAR(updA.position.y, expectedA.ub_pos.y, TOL);
    EXPECT_NEAR(updA.ground_support_velocity.x, expectedA.gs_vel.x, TOL);
    EXPECT_NEAR(updA.ground_support_velocity.y, expectedA.gs_vel.y, TOL);
    EXPECT_NEAR(updA.ground_support_position.x, expectedA.gs_pos.x, TOL);
    EXPECT_NEAR(updA.ground_support_position.y, expectedA.gs_pos.y, TOL);

    // Agent A's upper body should be pushed in -x by B's repulsion
    // (repulsion > driving force at this distance: A*exp(-0.5/0.5) = 5*e^-1 ≈ 1.84)
    // Combined with driving force = 2.0 in +x, net is positive but reduced.
    // The repulsion should reduce the x-velocity compared to an isolated agent.
    NeighborhoodSearch<GenericAgent> nsAlone(3);
    nsAlone.Update({agentA});
    const auto updateAlone = model.ComputeNewPosition(dT, agentA, geometry, nsAlone);
    const auto& updAlone = std::get<SocialForceModelIPPUpdate>(updateAlone);
    EXPECT_LT(updA.velocity.x, updAlone.velocity.x);
}

// Diagonal separation: legs offset in both x and y.
// Ensures 2D e_n direction is computed correctly.
TEST(SocialForceModelIPP, DiagonalLegOffset)
{
    const double dT = 0.01;
    auto geometry = MakeLargeBox();
    SocialForceModelIPP model;
    NeighborhoodSearch<GenericAgent> ns(3);

    // Upper body at (5,5), legs at (4.9, 4.9) — diagonal offset
    auto agent = MakeAgent(Point(5, 5), {0, 0}, Point(4.9, 4.9), {0, 0});
    agent.destination = Point(15, 5);
    ns.Update({agent});

    const auto update = model.ComputeNewPosition(dT, agent, geometry, ns);
    const auto& upd = std::get<SocialForceModelIPPUpdate>(update);
    const auto expected = HandCompute(agent, dT);

    EXPECT_NEAR(upd.velocity.x, expected.ub_vel.x, TOL);
    EXPECT_NEAR(upd.velocity.y, expected.ub_vel.y, TOL);
    EXPECT_NEAR(upd.position.x, expected.ub_pos.x, TOL);
    EXPECT_NEAR(upd.position.y, expected.ub_pos.y, TOL);
    EXPECT_NEAR(upd.ground_support_velocity.x, expected.gs_vel.x, TOL);
    EXPECT_NEAR(upd.ground_support_velocity.y, expected.gs_vel.y, TOL);

    // e_n has both +x and +y components → unbalancing contributes to both axes
    // Legs should recover toward upper body, gaining +y velocity
    EXPECT_GT(upd.ground_support_velocity.y, 0.0);
}

// Standing crowd (no driving force): desiredSpeed = 0.
// Only unbalancing, damping, and interaction forces act.
// This matches the paper's pure model (Eq. 1 without external driving).
TEST(SocialForceModelIPP, StandingCrowdNoDriving)
{
    const double dT = 0.01;
    auto geometry = MakeLargeBox();
    SocialForceModelIPP model;
    NeighborhoodSearch<GenericAgent> ns(3);

    // Upper body offset from legs, no desired speed
    auto agent = MakeAgent(Point(5.1, 5), {0, 0}, Point(5, 5), {0, 0});
    agent.destination = Point(5.1, 5); // destination = current pos → e0 ill-defined
    auto& data = std::get<SocialForceModelIPPData>(agent.model);
    data.desiredSpeed = 0.0;
    ns.Update({agent});

    const auto update = model.ComputeNewPosition(dT, agent, geometry, ns);
    const auto& upd = std::get<SocialForceModelIPPUpdate>(update);

    // With v=0 and offset, unbalancing pushes upper body further in +x (e_n direction).
    // This is the inverted pendulum instability from the paper.
    EXPECT_GT(upd.velocity.x, 0.0);

    // Legs should chase the upper body in +x (balance recovery)
    EXPECT_GT(upd.ground_support_velocity.x, 0.0);
}

// Verify the leg-separation clamp: when the integration step would produce
// a separation exceeding LEG_SCALING_FACTOR * height, the upper body is pulled back.
TEST(SocialForceModelIPP, LegSeparationClamp)
{
    const double dT = 0.01;
    auto geometry = MakeLargeBox();
    SocialForceModelIPP model;
    NeighborhoodSearch<GenericAgent> ns(3);

    const double height = 1.75;
    const double maxSep = SocialForceModelIPP::LEG_SCALING_FACTOR * height;

    // Place upper body far from legs with high velocity to overshoot the clamp
    auto agent = MakeAgent(
        Point(5 + maxSep - 0.01, 5), // just under max separation
        Point(10, 0),                 // high velocity in +x
        Point(5, 5),                  // legs at origin
        {0, 0},
        height);
    agent.destination = Point(100, 5);
    ns.Update({agent});

    const auto update = model.ComputeNewPosition(dT, agent, geometry, ns);
    const auto& upd = std::get<SocialForceModelIPPUpdate>(update);

    // After clamping, separation must not exceed max
    const double sep = (upd.position - upd.ground_support_position).Norm();
    EXPECT_LE(sep, maxSep + 1e-10);
}

// ApplyUpdate correctly transfers all state fields to the agent.
TEST(SocialForceModelIPP, ApplyUpdateTransfersState)
{
    SocialForceModelIPP model;
    auto agent = MakeAgent(Point(5, 5), {0, 0}, Point(5, 5), {0, 0});

    SocialForceModelIPPUpdate upd;
    upd.position = Point(5.1, 5.2);
    upd.velocity = Point(0.3, 0.4);
    upd.ground_support_position = Point(5.05, 5.1);
    upd.ground_support_velocity = Point(0.2, 0.15);

    OperationalModelUpdate update = upd;
    model.ApplyUpdate(update, agent);

    const auto& data = std::get<SocialForceModelIPPData>(agent.model);
    EXPECT_DOUBLE_EQ(agent.pos.x, 5.1);
    EXPECT_DOUBLE_EQ(agent.pos.y, 5.2);
    EXPECT_DOUBLE_EQ(data.velocity.x, 0.3);
    EXPECT_DOUBLE_EQ(data.velocity.y, 0.4);
    EXPECT_DOUBLE_EQ(data.ground_support_position.x, 5.05);
    EXPECT_DOUBLE_EQ(data.ground_support_position.y, 5.1);
    EXPECT_DOUBLE_EQ(data.ground_support_velocity.x, 0.2);
    EXPECT_DOUBLE_EQ(data.ground_support_velocity.y, 0.15);

    // Orientation should be set to normalized velocity
    const Point expectedOri = upd.velocity.Normalized();
    EXPECT_NEAR(agent.orientation.x, expectedOri.x, TOL);
    EXPECT_NEAR(agent.orientation.y, expectedOri.y, TOL);
}

// Multi-step integration: verify the system evolves plausibly over many steps.
// An isolated agent walking toward a destination should approach it monotonically.
TEST(SocialForceModelIPP, MultiStepAgentApproachesDestination)
{
    const double dT = 0.01;
    auto geometry = MakeLargeBox();
    SocialForceModelIPP model;

    auto agent = MakeAgent(Point(5, 5), {0, 0}, Point(5, 5), {0, 0});
    agent.destination = Point(15, 5);

    double prevDist = (agent.destination - agent.pos).Norm();
    for(int step = 0; step < 200; ++step) {
        NeighborhoodSearch<GenericAgent> ns(3);
        ns.Update({agent});
        const auto update = model.ComputeNewPosition(dT, agent, geometry, ns);
        model.ApplyUpdate(update, agent);
    }
    double finalDist = (agent.destination - agent.pos).Norm();

    // Agent should have made significant progress toward destination
    EXPECT_LT(finalDist, prevDist - 1.0);
    // Legs should follow the upper body
    const auto& data = std::get<SocialForceModelIPPData>(agent.model);
    EXPECT_GT(data.ground_support_position.x, 5.5);
}

// Symmetry: two agents placed symmetrically about the y-axis with symmetric
// destinations should produce mirror-image updates.
TEST(SocialForceModelIPP, SymmetricAgentsProduceMirrorUpdates)
{
    const double dT = 0.01;
    auto geometry = MakeLargeBox();
    SocialForceModelIPP model;

    auto agentL = MakeAgent(Point(4, 5), {0, 0}, Point(4, 5), {0, 0});
    agentL.destination = Point(-10, 5);
    auto agentR = MakeAgent(Point(6, 5), {0, 0}, Point(6, 5), {0, 0});
    agentR.destination = Point(20, 5);

    // Midpoint is x=5. agentL is at 5-1, agentR at 5+1.
    NeighborhoodSearch<GenericAgent> ns(3);
    ns.Update({agentL, agentR});

    const auto updL = std::get<SocialForceModelIPPUpdate>(
        model.ComputeNewPosition(dT, agentL, geometry, ns));
    const auto updR = std::get<SocialForceModelIPPUpdate>(
        model.ComputeNewPosition(dT, agentR, geometry, ns));

    // Velocities should be equal magnitude, opposite x-direction
    EXPECT_NEAR(updL.velocity.x, -updR.velocity.x, TOL);
    EXPECT_NEAR(updL.velocity.y, updR.velocity.y, TOL);

    // Positions should be mirror-symmetric about x=5
    EXPECT_NEAR(updL.position.x + updR.position.x, 10.0, TOL);
    EXPECT_NEAR(updL.position.y, updR.position.y, TOL);
}
