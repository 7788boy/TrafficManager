
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

  void DrawBuffer(Buffer &buffer, cc::DebugHelper::Color color);

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

  void DrawNeighbor(ActorId actor_id, LocalizationData &output);

  void DrawRegion(ActorId actor_id, LocalizationData &output);

  void DrawOtherBuffer(std::vector<SimpleWaypointPtr> &buffer, cc::DebugHelper::Color color);

  void DrawBug(ActorId target_id);

  void DrawBestRegion(ActorId actor_id, MTS_Region region);

  void MTSUpdate(const unsigned long index);

  MTS_Region GetRegion(ActorId actor_id, boost::optional<ActorId> target_id, SimpleWaypointPtr target_waypoint, float direction);
  
  float ComputeBestLateralOffset(ActorId actor_id, const unsigned long index);
  bool CheckLeftSafety(ActorId actor_id, float desired_offset , float *safe_offset , MTS_Region region, LocalizationData &localization);
  bool CheckSafety(ActorId actor_id, ActorId target_id, float moveSpeed, float moveTime, float* safeTime);
  bool CheckRightSafety(ActorId actor_id, float desired_offset , float *safe_offset , MTS_Region region, LocalizationData &localization);
  float GetLateralTime(float desired_lateral_offset, ActorId actor_id, LocalizationData &localization);
  float GetLongitudinalTime(ActorId actor_id, LocalizationData &localization);
  float GetGapToStopLine(ActorId actor_id);

  // std::array<float, 4> GlobalToLocal(ActorId actor_id, cg::Location global_location);
  // cg::Location LocalToGlobal(ActorId actor_id, cg::Location local_location);
  std::array<float, 4> matrixMultiply(std::array<float, 16> M, std::array<float, 4>  V);
  float GetGap(ActorId actor_id, ActorId target_id);
  float GetRelativeOffset(ActorId actor_id, ActorId target_id);
  float GetLateralSeparation(ActorId actor_id, ActorId target_id);
  float GetDynamicWidth(ActorId actor_id);
  float GetDynamicLength(ActorId actor_id);

  void GlobalToLocal(ActorId actor_id, cg::Location &location);
  void LocalToGlobal(ActorId actor_id, cg::Location &location);

  // void UpdateLeader(const unsigned long index);
  // void UpdateNeighbor(const unsigned long index);
  // void GetLeftVehicle(const unsigned long index);
  // void GetRightVehicle(const unsigned long index);
  // void GetSurroundVehicle(const unsigned long index);
  // bool isOverlapped(ActorId actor_id, ActorId target_id, float target_location_y) const;
  // bool isLogitudinalOverlapped(ActorId actor_id, ActorId target_id, float target_location_x) const;
  //cg::Location GlobalToLocal(ActorId actor_id, cg::Location target_location);
};

} // namespace traffic_manager
} // namespace carla
