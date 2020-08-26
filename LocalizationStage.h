
#pragma once

#include <memory>

#include "carla/client/DebugHelper.h"

#include "carla/trafficmanager/DataStructures.h"
#include "carla/trafficmanager/InMemoryMap.h"
#include "carla/trafficmanager/LocalizationUtils.h"
#include "carla/trafficmanager/Parameters.h"
#include "carla/trafficmanager/RandomGenerator.h"
#include "carla/trafficmanager/TrackTraffic.h"
#include "carla/trafficmanager/SimulationState.h"
#include "carla/trafficmanager/Stage.h"

namespace carla {
namespace traffic_manager {

namespace cc = carla::client;

using LocalMapPtr = std::shared_ptr<InMemoryMap>;
using LaneChangeLocationMap = std::unordered_map<ActorId, cg::Location>;

/// This class has functionality to maintain a horizon of waypoints ahead
/// of the vehicle for it to follow.
/// The class is also responsible for managing lane change decisions and
/// modify the waypoint trajectory appropriately.
class LocalizationStage : Stage {
private:
  const std::vector<ActorId> &vehicle_id_list;
  BufferMap &buffer_map;
  const SimulationState &simulation_state;
  TrackTraffic &track_traffic;
  const LocalMapPtr &local_map;
  Parameters &parameters;
  LocalizationFrame &output_array;
  cc::DebugHelper &debug_helper;
  LaneChangeLocationMap last_lane_change_location;
  ActorIdSet vehicles_at_junction;
  using SimpleWaypointPair = std::pair<SimpleWaypointPtr, SimpleWaypointPtr>;
  std::unordered_map<ActorId, SimpleWaypointPair> vehicles_at_junction_entrance;
  RandomGenerator<> pgen;

  SimpleWaypointPtr AssignLaneChange(const ActorId actor_id,
                                     const cg::Location vehicle_location,
                                     const float vehicle_speed,
                                     bool force, bool direction);

  void DrawBuffer(Buffer &buffer);

  void ExtendAndFindSafeSpace(const ActorId actor_id,
                              const bool is_at_junction_entrance,
                              Buffer &waypoint_buffer);

public:
  LocalizationStage(const std::vector<ActorId> &vehicle_id_list,
                    BufferMap &buffer_map,
                    const SimulationState &simulation_state,
                    TrackTraffic &track_traffic,
                    const LocalMapPtr &local_map,
                    Parameters &parameters,
                    LocalizationFrame &output_array,
                    cc::DebugHelper &debug_helper);

  void Update(const unsigned long index) override;

  void RemoveActor(const ActorId actor_id) override;

  void Reset() override;

  void DrawLeader(ActorId actor_id, LocalizationData &output);

  void UpdateLeader(const unsigned long index);

  void UpdateNeighbor(const unsigned long index);

  void GetLeftVehicle(const unsigned long index);

  void GetRightVehicle(const unsigned long index);

  void GetSurroundVehicle(const unsigned long index);

  bool isOverlapped(ActorId actor_id, ActorId target_id, float target_location_y) const;
  
  std::array<float, 4> GlobalToLocal(ActorId actor_id, cg::Location global_location);

  std::array<float, 4> matrixMultiply(std::array<float, 16> M, std::array<float, 4>  V);

};

} // namespace traffic_manager
} // namespace carla
