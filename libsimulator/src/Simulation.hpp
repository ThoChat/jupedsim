// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "AgentRemovalSystem.hpp"
#include "GenericAgent.hpp"
#include "Journey.hpp"
#include "NeighborhoodSearch.hpp"
#include "OperationalDecisionSystem.hpp"
#include "OperationalModel.hpp"
#include "OperationalModelType.hpp"
#include "Point.hpp"
#include "SimulationClock.hpp"
#include "Stage.hpp"
#include "StageDescription.hpp"
#include "StageManager.hpp"
#include "StageSystem.hpp"
#include "StrategicalDesicionSystem.hpp"
#include "TacticalDecisionSystem.hpp"
#include "Tracing.hpp"

#include <boost/iterator/zip_iterator.hpp>

#include <memory>
#include <unordered_map>
#include <vector>

class Simulation
{
    SimulationClock _clock;
    StrategicalDecisionSystem _stategicalDecisionSystem{};
    TacticalDecisionSystem _tacticalDecisionSystem{};
    OperationalDecisionSystem _operationalDecisionSystem;
    AgentRemovalSystem<GenericAgent> _agentRemovalSystem{};
    StageManager _stageManager{};
    StageSystem _stageSystem{};
    NeighborhoodSearch<GenericAgent> _neighborhoodSearch{2.2};
    std::unordered_map<
        CollisionGeometry::ID,
        std::tuple<std::unique_ptr<CollisionGeometry>, std::unique_ptr<RoutingEngine>>>
        geometries{};
    RoutingEngine* _routingEngine;
    CollisionGeometry* _geometry;
    std::vector<GenericAgent> _agents;
    std::vector<GenericAgent::ID> _removedAgentsInLastIteration;
    std::unordered_map<Journey::ID, std::unique_ptr<Journey>> _journeys;
    PerfStats _perfStats{};

public:
    Simulation(
        std::unique_ptr<OperationalModel>&& operationalModel,
        std::unique_ptr<CollisionGeometry>&& geometry,
        double dT);
    Simulation(const Simulation& other) = delete;
    Simulation& operator=(const Simulation& other) = delete;
    Simulation(Simulation&& other) = delete;
    Simulation& operator=(Simulation&& other) = delete;
    ~Simulation() = default;
    const SimulationClock& Clock() const;
    void SetTracing(bool on);
    PerfStats GetLastStats() const;
    void Iterate();
    Journey::ID AddJourney(const std::map<BaseStage::ID, TransitionDescription>& stages);
    BaseStage::ID AddStage(const StageDescription stageDescription);
    void MarkAgentForRemoval(GenericAgent::ID id);
    const std::vector<GenericAgent::ID>& RemovedAgents() const;
    size_t AgentCount() const;
    double ElapsedTime() const;
    double DT() const;
    void
    SwitchAgentJourney(GenericAgent::ID agent_id, Journey::ID journey_id, BaseStage::ID stage_id);
    uint64_t Iteration() const;
    std::vector<GenericAgent::ID> AgentsInRange(Point p, double distance);
    /// Returns IDs of all agents inside the defined polygon
    /// @param polygon Required to be a simple convex polygon with CCW ordering.
    std::vector<GenericAgent::ID> AgentsInPolygon(const std::vector<Point>& polygon);
    GenericAgent::ID AddAgent(GenericAgent&& agent);
    const GenericAgent& Agent(GenericAgent::ID id) const;
    GenericAgent& Agent(GenericAgent::ID id);
    std::vector<GenericAgent>& Agents();
    OperationalModelType ModelType() const;
    StageProxy Stage(BaseStage::ID stageId);
    CollisionGeometry Geo() const;
    void SwitchGeometry(std::unique_ptr<CollisionGeometry>&& geometry);

private:
    void ValidateGeometry(const std::unique_ptr<CollisionGeometry>& geometry) const;
};
