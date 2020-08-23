
/// This file contains definitions of common data structures used in traffic manager.

#pragma once

#include <chrono>
#include <deque>
#include <vector>

#include "carla/client/Actor.h"
#include "carla/geom/Location.h"
#include "carla/geom/Rotation.h"
#include "carla/geom/Vector3D.h"
#include "carla/rpc/ActorId.h"
#include "carla/rpc/Command.h"
#include "carla/rpc/TrafficLightState.h"

#include "carla/trafficmanager/SimpleWaypoint.h"

namespace carla {
namespace traffic_manager {

namespace chr = std::chrono;
namespace cc = carla::client;
namespace cg = carla::geom;

using ActorId = carla::ActorId;
using ActorPtr = carla::SharedPtr<cc::Actor>;
using JunctionID = carla::road::JuncId;
using SimpleWaypointPtr = std::shared_ptr<SimpleWaypoint>;
using Buffer = std::deque<SimpleWaypointPtr>;
using BufferMap = std::unordered_map<carla::ActorId, Buffer>;
using TimeInstance = chr::time_point<chr::system_clock, chr::nanoseconds>;
using TLS = carla::rpc::TrafficLightState;

struct MTS_Leader{
  ActorId main_leader;
  ActorId potential_leader;
  ActorId pre_main_leader;
  ActorId pre_potential_leader;
};

struct MTS_Neighbor{
  bool updated;
  std::vector< ActorId > vehicles;
};

struct MTS_Surrounding
{
    // MTS_Leader* leader;

    // MTS_Vehicle* mPassedLeadingVehicle;
    // MTS_Vehicle* mExpectedLeadingVehicle;
    // MTS_Vehicle* prevFirstLeader;
    // MTS_Vehicle* prevSecondLeader;

    std::unordered_map<int, MTS_Neighbor*> neighbor;

    // std::vector< std::pair< MTS_Vehicle*,float > > mNeighbor;
    // std::vector< MTS_Vehicle* > mLeftRearVehicles;
    // std::vector< MTS_Vehicle* > mLeftFrontVehicles;
    // std::vector< MTS_Vehicle* > mRightRearVehicles;
    // std::vector< MTS_Vehicle* > mRightFrontVehicles;
    ActorId mLeftVehicle;
    ActorId mRightVehicle;
    // bool mLeftRearVehiclesUpdated;
    // bool mLeftFrontVehiclesUpdated;
    // bool mRightRearVehiclesUpdated;
    // bool mRightFrontVehiclesUpdated;
};

struct MTS_Region
{
    float leftBorder;
    float rightBorder;
    float offset;
    float width;
    float maxPassingSpeed;
    float gap;
    float safety;
    ActorId leftBorderVehicle, rightBorderVehicle;
    std::vector< ActorId > frontVehicles;
    std::vector< ActorId > rearVehicles;
    float preference;
};

struct MTS_SituationData {
  MTS_Neighbor* mNeighbor;
  std::vector<MTS_Region>	mRegion;
  MTS_Region mCurrentRegion;
  bool mSpaceOriented;
  //MTS_Lane* mLane;
  //MTS_Edge* mEdge;
  float safety;
};

struct LocalizationData {
  SimpleWaypointPtr junction_end_point;
  SimpleWaypointPtr safe_point;
  bool is_at_junction_entrance;
  MTS_Leader leader;
  MTS_Surrounding surrounding;
  MTS_Region region;

};
using LocalizationFrame = std::vector<LocalizationData>;

struct CollisionHazardData {
  float available_distance_margin;
  ActorId hazard_actor_id;
  bool hazard;
};
using CollisionFrame = std::vector<CollisionHazardData>;

using ControlFrame = std::vector<carla::rpc::Command>;

using TLFrame = std::vector<bool>;



/// Structure to hold the actuation signals.
struct ActuationSignal {
  float throttle;
  float brake;
  float steer;
};

/// Structure to hold the controller state.
struct StateEntry {
  TimeInstance time_instance;
  float deviation;
  float velocity;
  float deviation_integral;
  float velocity_integral;
};

} // namespace traffic_manager
} // namespace carla
