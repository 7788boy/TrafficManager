
#include "carla/trafficmanager/Constants.h"
#include "math.h"
#include "carla/trafficmanager/LocalizationStage.h"

namespace carla {
namespace traffic_manager {

using namespace constants::PathBufferUpdate;
using namespace constants::LaneChange;
using namespace constants::WaypointSelection;
using namespace constants::MTSCar;

LocalizationStage::LocalizationStage(
  const std::vector<ActorId> &vehicle_id_list,
  BufferMap &buffer_map,
  const SimulationState &simulation_state,
  TrackTraffic &track_traffic,
  const LocalMapPtr &local_map,
  Parameters &parameters,
  LocalizationFrame &output_array,
  cc::DebugHelper &debug_helper)
  : vehicle_id_list(vehicle_id_list),
    buffer_map(buffer_map),
    simulation_state(simulation_state),
    track_traffic(track_traffic),
    local_map(local_map),
    parameters(parameters),
    output_array(output_array),
    debug_helper(debug_helper) {}

void LocalizationStage::Update(const unsigned long index) {

  const ActorId actor_id = vehicle_id_list.at(index);
  const cg::Location vehicle_location = simulation_state.GetLocation(actor_id);
  const cg::Vector3D heading_vector = simulation_state.GetHeading(actor_id);
  const cg::Vector3D vehicle_velocity_vector = simulation_state.GetVelocity(actor_id);
  const float vehicle_speed = vehicle_velocity_vector.Length();

  // Speed dependent waypoint horizon length.
  float horizon_length = std::min(vehicle_speed * HORIZON_RATE + MINIMUM_HORIZON_LENGTH, MAXIMUM_HORIZON_LENGTH);
  const float horizon_square = SQUARE(horizon_length);

  if (buffer_map.find(actor_id) == buffer_map.end()) {
    buffer_map.insert({actor_id, Buffer()});
  }

  Buffer &waypoint_buffer = buffer_map.at(actor_id);

  // Clear buffer if vehicle is too far from the first waypoint in the buffer.
  if (!waypoint_buffer.empty() &&
      cg::Math::DistanceSquared(waypoint_buffer.front()->GetLocation(),
                                vehicle_location) > SQUARE(MAX_START_DISTANCE)) {

    auto number_of_pops = waypoint_buffer.size();
    for (uint64_t j = 0u; j < number_of_pops; ++j) {
      PopWaypoint(actor_id, track_traffic, waypoint_buffer);
    }
  }

  bool is_at_junction_entrance = false;

  if (!waypoint_buffer.empty()) {
    // Purge passed waypoints.
    float dot_product = DeviationDotProduct(vehicle_location, heading_vector, waypoint_buffer.front()->GetLocation());
    while (dot_product <= 0.0f && !waypoint_buffer.empty()) {

      PopWaypoint(actor_id, track_traffic, waypoint_buffer);
      if (!waypoint_buffer.empty()) {
        dot_product = DeviationDotProduct(vehicle_location, heading_vector, waypoint_buffer.front()->GetLocation());
      }
    }

    if (!waypoint_buffer.empty()) {
      // Determine if the vehicle is at the entrance of a junction.
      SimpleWaypointPtr look_ahead_point = GetTargetWaypoint(waypoint_buffer, JUNCTION_LOOK_AHEAD).first;
      is_at_junction_entrance = !waypoint_buffer.front()->CheckJunction() && look_ahead_point->CheckJunction();
      if (is_at_junction_entrance
          // Exception for roundabout in Town03.
          && local_map->GetMapName() == "Town03"
          && vehicle_location.SquaredLength() < SQUARE(30)) {
        is_at_junction_entrance = false;
      }
    }

    // Purge waypoints too far from the front of the buffer.
    while (!is_at_junction_entrance
           && !waypoint_buffer.empty()
           && waypoint_buffer.back()->DistanceSquared(waypoint_buffer.front()) > horizon_square) {
      PopWaypoint(actor_id, track_traffic, waypoint_buffer, false);
    }
  }

  // Initializing buffer if it is empty.
  if (waypoint_buffer.empty()) {
    SimpleWaypointPtr closest_waypoint = local_map->GetWaypoint(vehicle_location);
    PushWaypoint(actor_id, track_traffic, waypoint_buffer, closest_waypoint);
  }

  /**************************
   *********MTS CALL*********
   **************************/
  MTSUpdate(index);
  float best_offset = ComputeBestLateralOffset(actor_id, index);
  bool direction = false, force = false;
  const SimpleWaypointPtr actor_waypoint = local_map->GetWaypoint(vehicle_location);
  float lane_half_width = float(actor_waypoint->GetWaypoint()->GetLaneWidth()) / 2.0;
  
  if (best_offset > lane_half_width){
    direction = true;
    force = true;
  }
  else if(best_offset < -lane_half_width){
    direction = false;
    force = true;
  }
  //std::cout << "best offset: " << best_offset << ", force: " << force << ", direction: " << direction << std::endl;
  
  // Assign a lane change.
  //const ChangeLaneInfo lane_change_info = parameters.GetForceLaneChange(actor_id);
  //bool force_lane_change = lane_change_info.change_lane;
  //bool lane_change_direction = lane_change_info.direction;
  bool force_lane_change = force;
  bool lane_change_direction = direction;

  // if (!force_lane_change) {
  //   float perc_keep_right = parameters.GetKeepRightPercentage(actor_id);
  //   if (perc_keep_right >= 0.0f && perc_keep_right >= pgen.next()) {
  //     force_lane_change = true;
  //     lane_change_direction = true;
  //   }
  // }

  const SimpleWaypointPtr front_waypoint = waypoint_buffer.front();
  const float lane_change_distance = SQUARE(std::max(10.0f * vehicle_speed, INTER_LANE_CHANGE_DISTANCE));

  bool recently_not_executed_lane_change = last_lane_change_location.find(actor_id) == last_lane_change_location.end();
  bool done_with_previous_lane_change = true;
  if (!recently_not_executed_lane_change) {
    float distance_frm_previous = cg::Math::DistanceSquared(last_lane_change_location.at(actor_id), vehicle_location);
    done_with_previous_lane_change = distance_frm_previous > lane_change_distance;
  }
  //bool auto_or_force_lane_change = parameters.GetAutoLaneChange(actor_id) || force_lane_change;
  bool auto_or_force_lane_change = force_lane_change;
  bool front_waypoint_not_junction = !front_waypoint->CheckJunction();

  if (auto_or_force_lane_change
      && front_waypoint_not_junction
      && (recently_not_executed_lane_change || done_with_previous_lane_change)) {

    SimpleWaypointPtr change_over_point = AssignLaneChange(actor_id, vehicle_location, vehicle_speed,
                                                           force_lane_change, lane_change_direction);

    if (change_over_point != nullptr) {
      if (last_lane_change_location.find(actor_id) != last_lane_change_location.end()) {
        last_lane_change_location.at(actor_id) = vehicle_location;
      } else {
        last_lane_change_location.insert({actor_id, vehicle_location});
      }
      auto number_of_pops = waypoint_buffer.size();
      for (uint64_t j = 0u; j < number_of_pops; ++j) {
        PopWaypoint(actor_id, track_traffic, waypoint_buffer);
      }
      PushWaypoint(actor_id, track_traffic, waypoint_buffer, change_over_point);
    }
  }

  // Populating the buffer.
  while (waypoint_buffer.back()->DistanceSquared(waypoint_buffer.front()) <= horizon_square) {

    std::vector<SimpleWaypointPtr> next_waypoints = waypoint_buffer.back()->GetNextWaypoint();
    uint64_t selection_index = 0u;
    // Pseudo-randomized path selection if found more than one choice.
    if (next_waypoints.size() > 1) {
      selection_index = static_cast<uint64_t>(pgen.next()) % next_waypoints.size();
    }
    SimpleWaypointPtr next_wp = next_waypoints.at(selection_index);
    if (next_wp == nullptr) {
      for (auto &wp : next_waypoints) {
        if (wp != nullptr) {
          next_wp = wp;
          break;
        }
      }
    }
    PushWaypoint(actor_id, track_traffic, waypoint_buffer, next_wp);
  }

  ExtendAndFindSafeSpace(actor_id, is_at_junction_entrance, waypoint_buffer);

  // Editing output array
  LocalizationData &output = output_array.at(index);
  output.is_at_junction_entrance = is_at_junction_entrance;

  if (is_at_junction_entrance) {
    const SimpleWaypointPair &safe_space_end_points = vehicles_at_junction_entrance.at(actor_id);
    output.junction_end_point = safe_space_end_points.first;
    output.safe_point = safe_space_end_points.second;
  } else {
    output.junction_end_point = nullptr;
    output.safe_point = nullptr;
  }

  // Updating geodesic grid position for actor.
  track_traffic.UpdateGridPosition(actor_id, waypoint_buffer);

  /**************************
   *********MTS CALL*********
   **************************/
  MTSUpdate(index);
  //UpdateLeader(index);
  //UpdateNeighbor(index);

  if (actor_id == vehicle_id_list.at(0))
  {
    // test
    // cg::Location test_location = vehicle_location; // original
    // cg::Location test_local_location = test_location;
    // TestGlobalToLocal(actor_id, test_local_location); // global to local
    // cg::Location test_global_location = test_local_location;
    // TestLocalToGlobal(actor_id, test_global_location); // local to global

    // cg::Vector3D test_velocity = vehicle_velocity_vector; //original
    // cg::Rotation test_rotation = simulation_state.GetRotation(actor_id);
    // cg::Vector3D test_local_velocity = test_velocity;
    // test_rotation.InverseRotateVector(test_local_velocity); // global to local
    // cg::Vector3D test_global_velocity = test_local_velocity;
    // test_rotation.RotateVector(test_global_velocity); // local to global.

    // std::cout << "Original location: x = " << test_location.x << ", y = " << test_location.y << ", z = " << test_location.z << std::endl;
    // std::cout << "Local location: x = " << test_local_location.x << ", y = " << test_local_location.y << ", z = " << test_local_location.z << std::endl;
    // std::cout << "Global location: x = " << test_global_location.x << ", y = " << test_global_location.y << ", z = " << test_global_location.z << std::endl;

    // std::cout << "Original velocity: x = " << test_velocity.x << ", y = " << test_velocity.y << ", z = " << test_velocity.z << std::endl;
    // std::cout << "Local velocity: x = " << test_local_velocity.x << ", y = " << test_local_velocity.y << ", z = " << test_local_velocity.z << std::endl;
    // std::cout << "Global velocity: x = " << test_global_velocity.x << ", y = " << test_global_velocity.y << ", z = " << test_global_velocity.z << std::endl;

    DrawLeader(actor_id, output);
    DrawNeighbor(actor_id, output);
    DrawRegion(actor_id, output);
    DrawBuffer(waypoint_buffer, {0u, 0u, 0u});
  }
}

void LocalizationStage::ExtendAndFindSafeSpace(const ActorId actor_id,
                                               const bool is_at_junction_entrance,
                                               Buffer &waypoint_buffer) {

  SimpleWaypointPtr junction_end_point = nullptr;
  SimpleWaypointPtr safe_point_after_junction = nullptr;

  if (is_at_junction_entrance
      && vehicles_at_junction_entrance.find(actor_id) == vehicles_at_junction_entrance.end()) {

    bool entered_junction = false;
    bool past_junction = false;
    bool safe_point_found = false;
    SimpleWaypointPtr current_waypoint = nullptr;
    SimpleWaypointPtr junction_begin_point = nullptr;
    float safe_distance_squared = SQUARE(SAFE_DISTANCE_AFTER_JUNCTION);

    // Scanning existing buffer points.
    for (unsigned long i = 0u; i < waypoint_buffer.size() && !safe_point_found; ++i) {
      current_waypoint = waypoint_buffer.at(i);
      if (!entered_junction && current_waypoint->CheckJunction()) {
        entered_junction = true;
        junction_begin_point = current_waypoint;
      }
      if (entered_junction && !past_junction && !current_waypoint->CheckJunction()) {
        past_junction = true;
        junction_end_point = current_waypoint;
      }
      if (past_junction && junction_end_point->DistanceSquared(current_waypoint) > safe_distance_squared) {
        safe_point_found = true;
        safe_point_after_junction = current_waypoint;
      }
    }

    // Extend buffer if safe point not found.
    if (!safe_point_found) {
      while (!past_junction) {
        current_waypoint = current_waypoint->GetNextWaypoint().front();
        PushWaypoint(actor_id, track_traffic, waypoint_buffer, current_waypoint);
        if (!current_waypoint->CheckJunction()) {
          past_junction = true;
          junction_end_point = current_waypoint;
        }
      }

      while (!safe_point_found) {
        std::vector<SimpleWaypointPtr> next_waypoints = current_waypoint->GetNextWaypoint();
        if ((junction_end_point->DistanceSquared(current_waypoint) > safe_distance_squared)
            || next_waypoints.size() > 1
            || current_waypoint->CheckJunction()) {

          safe_point_found = true;
          safe_point_after_junction = current_waypoint;
        } else {
          current_waypoint = next_waypoints.front();
          PushWaypoint(actor_id, track_traffic, waypoint_buffer, current_waypoint);
        }
      }
    }

    if (junction_begin_point->DistanceSquared(junction_end_point) < SQUARE(MIN_JUNCTION_LENGTH)) {
      junction_end_point = nullptr;
      safe_point_after_junction = nullptr;
    }

    vehicles_at_junction_entrance.insert({actor_id, {junction_end_point, safe_point_after_junction}});
  }
  else if (!is_at_junction_entrance
           && vehicles_at_junction_entrance.find(actor_id) != vehicles_at_junction_entrance.end()) {

    vehicles_at_junction_entrance.erase(actor_id);
  }
}

void LocalizationStage::RemoveActor(ActorId actor_id) {
    last_lane_change_location.erase(actor_id);
    vehicles_at_junction.erase(actor_id);
}

void LocalizationStage::Reset() {
  last_lane_change_location.clear();
  vehicles_at_junction.clear();
}

SimpleWaypointPtr LocalizationStage::AssignLaneChange(const ActorId actor_id,
                                                      const cg::Location vehicle_location,
                                                      const float vehicle_speed,
                                                      bool force, bool direction) {

  // Waypoint representing the new starting point for the waypoint buffer
  // due to lane change. Remains nullptr if lane change not viable.
  SimpleWaypointPtr change_over_point = nullptr;

  // Retrieve waypoint buffer for current vehicle.
  const Buffer &waypoint_buffer = buffer_map.at(actor_id);

  // Check buffer is not empty.
  if (!waypoint_buffer.empty()) {
    // Get the left and right waypoints for the current closest waypoint.
    const SimpleWaypointPtr &current_waypoint = waypoint_buffer.front();
    const SimpleWaypointPtr left_waypoint = current_waypoint->GetLeftWaypoint();
    const SimpleWaypointPtr right_waypoint = current_waypoint->GetRightWaypoint();

    // Retrieve vehicles with overlapping waypoint buffers with current vehicle.
    // const auto blocking_vehicles = track_traffic.GetOverlappingVehicles(actor_id);

    // // Find immediate in-lane obstacle and check if any are too close to initiate lane change.
    // bool obstacle_too_close = false;
    // float minimum_squared_distance = std::numeric_limits<float>::infinity();
    // ActorId obstacle_actor_id = 0u;
    // for (auto i = blocking_vehicles.begin();
    //      i != blocking_vehicles.end() && !obstacle_too_close && !force;
    //      ++i) {
    //   const ActorId &other_actor_id = *i;
    //   // Find vehicle in buffer map and check if it's buffer is not empty.
    //   if (buffer_map.find(other_actor_id) != buffer_map.end() && !buffer_map.at(other_actor_id).empty()) {
    //     const Buffer &other_buffer = buffer_map.at(other_actor_id);
    //     const SimpleWaypointPtr &other_current_waypoint = other_buffer.front();
    //     const cg::Location other_location = other_current_waypoint->GetLocation();

    //     const cg::Vector3D reference_heading = current_waypoint->GetForwardVector();
    //     cg::Vector3D reference_to_other = other_location - current_waypoint->GetLocation();
    //     const cg::Vector3D other_heading = other_current_waypoint->GetForwardVector();

    //     WaypointPtr current_raw_waypoint = current_waypoint->GetWaypoint();
    //     WaypointPtr other_current_raw_waypoint = other_current_waypoint->GetWaypoint();
    //     // Check both vehicles are not in junction,
    //     // Check if the other vehicle is in front of the current vehicle,
    //     // Check if the two vehicles have acceptable angular deviation between their headings.
    //     if (!current_waypoint->CheckJunction()
    //         && !other_current_waypoint->CheckJunction()
    //         && other_current_raw_waypoint->GetRoadId() == current_raw_waypoint->GetRoadId()
    //         && other_current_raw_waypoint->GetLaneId() == current_raw_waypoint->GetLaneId()
    //         && cg::Math::Dot(reference_heading, reference_to_other) > 0.0f
    //         && cg::Math::Dot(reference_heading, other_heading) > MAXIMUM_LANE_OBSTACLE_CURVATURE) {
    //       float squared_distance = cg::Math::DistanceSquared(vehicle_location, other_location);
    //       // Abort if the obstacle is too close.
    //       if (squared_distance > SQUARE(MINIMUM_LANE_CHANGE_DISTANCE)) {
    //         // Remember if the new vehicle is closer.
    //         if (squared_distance < minimum_squared_distance && squared_distance < SQUARE(MAXIMUM_LANE_OBSTACLE_DISTANCE)) {
    //           minimum_squared_distance = squared_distance;
    //           obstacle_actor_id = other_actor_id;
    //         }
    //       } else {
    //         obstacle_too_close = true;
    //       }
    //     }
    //   }
    // }

    // // If a valid immediate obstacle found.
    // if (!obstacle_too_close && obstacle_actor_id != 0u && !force) {
    //   const Buffer &other_buffer = buffer_map.at(obstacle_actor_id);
    //   const SimpleWaypointPtr &other_current_waypoint = other_buffer.front();
    //   const auto other_neighbouring_lanes = {other_current_waypoint->GetLeftWaypoint(),
    //                                          other_current_waypoint->GetRightWaypoint()};

    //   // Flags reflecting whether adjacent lanes are free near the obstacle.
    //   bool distant_left_lane_free = false;
    //   bool distant_right_lane_free = false;

    //   // Check if the neighbouring lanes near the obstructing vehicle are free of other vehicles.
    //   bool left_right = true;
    //   for (auto &candidate_lane_wp : other_neighbouring_lanes) {
    //     if (candidate_lane_wp != nullptr &&
    //         track_traffic.GetPassingVehicles(candidate_lane_wp->GetId()).size() == 0) {

    //       if (left_right)
    //         distant_left_lane_free = true;
    //       else
    //         distant_right_lane_free = true;
    //     }
    //     left_right = !left_right;
    //   }

    //   // Based on what lanes are free near the obstacle,
    //   // find the change over point with no vehicles passing through them.
    //   if (distant_right_lane_free && right_waypoint != nullptr
    //       && track_traffic.GetPassingVehicles(right_waypoint->GetId()).size() == 0) {
    //     change_over_point = right_waypoint;
    //   } else if (distant_left_lane_free && left_waypoint != nullptr
    //            && track_traffic.GetPassingVehicles(left_waypoint->GetId()).size() == 0) {
    //     change_over_point = left_waypoint;
    //   }
    // } else 
    if (force) {
      if (direction && right_waypoint != nullptr) {
        change_over_point = right_waypoint;
      } else if (!direction && left_waypoint != nullptr) {
        change_over_point = left_waypoint;
      }
    }

    if (change_over_point != nullptr) {
      const float change_over_distance = cg::Math::Clamp(1.5f * vehicle_speed, 3.0f, 20.0f);
      const auto starting_point = change_over_point;
      while (change_over_point->DistanceSquared(starting_point) < SQUARE(change_over_distance) &&
             !change_over_point->CheckJunction()) {
        change_over_point = change_over_point->GetNextWaypoint()[0];
      }
    }
  }

  return change_over_point;
}


void LocalizationStage::DrawBuffer(Buffer &buffer, cc::DebugHelper::Color color) {
  uint64_t buffer_size = buffer.size();
  uint64_t step_size =  buffer_size/20u;
  //cc::DebugHelper::Color color {0u, 0u, 0u};
  cg::Location two_meters_up = cg::Location(0.0f, 0.0f, 2.0f);
  for (uint64_t i = 0u; i + step_size < buffer_size; i += step_size) {
    if (!buffer.at(i)->CheckJunction() && !buffer.at(i + step_size)->CheckJunction()) {
      color.g = 255u;
    }
    debug_helper.DrawLine(buffer.at(i)->GetLocation() + two_meters_up,
                          buffer.at(i + step_size)->GetLocation() + two_meters_up,
                          0.2f, color, 0.05f);
    color = {0u, 0u, 0u};
  }
}
/**MTS SECTION**/
void LocalizationStage::DrawLeader(ActorId actor_id, LocalizationData &output)
{
  cg::Vector3D box_size(0.3f,0.3f,0.3f);
  cg::Location actor_location = simulation_state.GetLocation(actor_id);
  cg::Rotation actor_rotation = simulation_state.GetRotation(actor_id);
  actor_location.z += 3.0f;
  cc::DebugHelper::Color color_ego {255u, 0u, 0u};
  debug_helper.DrawBox( cg::BoundingBox(actor_location, box_size), actor_rotation, 0.1f, color_ego, 0.01f, true);
  
  //Draw range line
  //const cg::Vector3D actor_heading = simulation_state.GetHeading(actor_id);
  //cg::Vector3D actor_heading_unit = actor_heading.MakeUnitVector();
  //cg::Location actor_location_end = actor_location + cg::Location(actor_heading_unit *= MAX_OBSERVING_DISTANCE);
  //debug_helper.DrawLine(actor_location, actor_location_end, 0.1f, color_ego, 0.3f, true);

  if(output.leader.MainLeader)
  {
    cg::Location first_location = simulation_state.GetLocation(output.leader.MainLeader.get());
    first_location.z += 3.0f;
    cc::DebugHelper::Color color_main_leader {0u, 255u, 0u}; //Green
    debug_helper.DrawBox(cg::BoundingBox(first_location, box_size), actor_rotation, 0.1f, color_main_leader, 0.01f, true);
    }

  if(output.leader.PotentialLeader)
  { 
    cg::Location second_location = simulation_state.GetLocation(output.leader.PotentialLeader.get());
    second_location.z += 3.0f;
    cc::DebugHelper::Color color_potential_leader {0u, 0u, 255u}; //Blue
    debug_helper.DrawBox(cg::BoundingBox(second_location, box_size), actor_rotation, 0.1f, color_potential_leader, 0.01f, true);
  }
}

void LocalizationStage::DrawNeighbor(ActorId actor_id, LocalizationData &output)
{
  cg::Vector3D box_size(0.3f,0.3f,0.3f);
  cg::Rotation actor_rotation = simulation_state.GetRotation(actor_id);
  
  if(output.neighbor.LeftVehicle)
  {
    cg::Location left_location = simulation_state.GetLocation(output.neighbor.LeftVehicle.get());
    left_location.z += 3.0f;
    cc::DebugHelper::Color color_left {255u, 0u, 255u}; //Magenta 洋紅
    debug_helper.DrawBox( cg::BoundingBox(left_location, box_size), actor_rotation, 0.1f, color_left, 0.3f, true);
  }

  if(output.neighbor.RightVehicle)
  { 
    cg::Location right_location = simulation_state.GetLocation(output.neighbor.RightVehicle.get());
    right_location.z += 3.0f;
    cc::DebugHelper::Color color_right {255u, 255u, 0u}; //Yelllow
    debug_helper.DrawBox( cg::BoundingBox(right_location, box_size), actor_rotation, 0.1f, color_right, 0.3f, true);
  }
  
  if(output.neighbor.RightFrontVehicle)
  { 
    cg::Location right_front_location = simulation_state.GetLocation(output.neighbor.RightFrontVehicle.get());
    right_front_location.z += 3.0f;
    cc::DebugHelper::Color color_right_front {0u, 0u, 0u}; //Black
    debug_helper.DrawBox( cg::BoundingBox(right_front_location, box_size), actor_rotation, 0.1f, color_right_front, 0.3f, true);
  }

  if(output.neighbor.LeftFrontVehicle)
  { 
    cg::Location left_front_location = simulation_state.GetLocation(output.neighbor.LeftFrontVehicle.get());
    left_front_location.z += 3.0f;
    cc::DebugHelper::Color color_left_front {0u, 255u, 255u}; //blue green
    debug_helper.DrawBox( cg::BoundingBox(left_front_location, box_size), actor_rotation, 0.1f, color_left_front, 0.3f, true);
  }

  if(output.neighbor.RightRearVehicle)
  { 
    cg::Location right_rear_location = simulation_state.GetLocation(output.neighbor.RightRearVehicle.get());
    right_rear_location.z += 3.0f;
    cc::DebugHelper::Color color_right_rear {155u, 55u, 255u}; //Purple
    debug_helper.DrawBox( cg::BoundingBox(right_rear_location, box_size), actor_rotation, 0.1f, color_right_rear, 0.3f, true);
  }

  if(output.neighbor.LeftRearVehicle)
  { 
    cg::Location left_rear_location = simulation_state.GetLocation(output.neighbor.LeftRearVehicle.get());
    left_rear_location.z += 3.0f;
    cc::DebugHelper::Color color_left_rear {255u, 255u, 255u}; //White
    debug_helper.DrawBox( cg::BoundingBox(left_rear_location, box_size), actor_rotation, 0.1f, color_left_rear, 0.3f, true);
  }

}

void LocalizationStage::DrawBestRegion(ActorId actor_id, MTS_Region region)
{
  cg::Vector3D box_size(region.gap / 2.0f, region.width / 2.0f, 0.0f);
  cg::Location location = region.location;
  cg::Rotation rotation = simulation_state.GetRotation(actor_id);
  location.z += 0.5f;
  cc::DebugHelper::Color color {255u, 0u, 0u};
  debug_helper.DrawBox(cg::BoundingBox(location, box_size), rotation, 0.1f, color, 0.3f, true);
}

void LocalizationStage::DrawRegion(ActorId actor_id, LocalizationData &output)
{
  for(auto &region : output.situation.CandidateRegions)
  {
    cg::Vector3D box_size(region.gap / 2.0f, region.width / 2.0f, 0.0f);
    cg::Location location = region.location;
    cg::Rotation rotation = simulation_state.GetRotation(actor_id);
    location.z += 0.5f;
    cc::DebugHelper::Color color {255u, 0u, 0u};
    debug_helper.DrawBox(cg::BoundingBox(location, box_size), rotation, 0.1f, color, 0.3f, true);
  }
}

void LocalizationStage::DrawBug(ActorId target_id)
{
  cg::Vector3D box_size(1.0f,1.0f,1.0f);
  cg::Location target_location = simulation_state.GetLocation(target_id);
  cg::Rotation target_rotation = simulation_state.GetRotation(target_id);
  target_location.z += 3.0f;
  cc::DebugHelper::Color color_ego {255u, 0u, 0u};
  debug_helper.DrawBox( cg::BoundingBox(target_location, box_size), target_rotation, 0.1f, color_ego, 0.01f, true);
}

void LocalizationStage::DrawOtherBuffer(std::vector<SimpleWaypointPtr> &buffer, cc::DebugHelper::Color color) {
  uint64_t buffer_size = buffer.size();
  uint64_t step_size =  buffer_size/20u;
  cg::Location two_meters_up = cg::Location(0.0f, 0.0f, 2.0f);
  for (uint64_t i = 0u; i + step_size < buffer_size; i += step_size) {
    
    debug_helper.DrawLine(buffer.at(i)->GetLocation() + two_meters_up,
                          buffer.at(i + step_size)->GetLocation() + two_meters_up,
                          0.2f, color, 0.05f);
  }
}

// Merge update leader, left, right, surround
void LocalizationStage::MTSUpdate(const unsigned long index)
{
  const ActorId actor_id = vehicle_id_list.at(index);
  
  //Basic actor information
  const cg::Location actor_location = simulation_state.GetLocation(actor_id);
  const cg::Vector3D actor_heading = simulation_state.GetHeading(actor_id);
  float actor_half_length = simulation_state.GetDimensions(actor_id).x;
  float actor_half_width = simulation_state.GetDimensions(actor_id).y;

  //Define search bound (Radius of circle)
  const float bound = actor_half_length + MAX_OBSERVING_DISTANCE;

  //The distance will be shorter after finding new target
  float leader_distance = FLT_MAX;
  float left_distance = FLT_MAX;
  float right_distance = FLT_MAX;
  float left_front_distance = FLT_MAX;
  float right_front_distance = FLT_MAX;
  float left_rear_distance = FLT_MAX;
  float right_rear_distance = FLT_MAX;

  //Temporary result
  boost::optional<ActorId> potential_leader; // c++17: std, but c++14: boost instead.
  boost::optional<ActorId> main_leader;
  boost::optional<ActorId> leftVeh;
  boost::optional<ActorId> rightVeh;
  boost::optional<ActorId> leftFrontVeh;
  boost::optional<ActorId> rightFrontVeh;
  boost::optional<ActorId> leftRearVeh;
  boost::optional<ActorId> rightRearVeh;

  
  //Road information
  LocalRoadInfo mid_info, left_info, right_info;
  SimpleWaypointPtr actor_waypoint = local_map->GetWaypoint(actor_location);
  SimpleWaypointPtr actor_left_waypoint = actor_waypoint->GetLeftWaypoint();
  SimpleWaypointPtr actor_right_waypoint = actor_waypoint->GetRightWaypoint();
  
  bool hasLeftLane = false;
  bool hasRightLane = false;
  bool isJunction = actor_waypoint->CheckJunction();

  
  crd::RoadId actor_road_id;
  double actor_road_length;
  if(!isJunction){
    const crd::Lane& actor_lane = local_map->GetLane(actor_waypoint);
    actor_road_id = actor_lane.GetRoad()->GetId();
    actor_road_length = actor_lane.GetRoad()->GetLength();
    GetLocalRoadInfo(mid_info, actor_lane);
    if(actor_left_waypoint != nullptr){
      hasLeftLane = true;
      const crd::Lane& left_lane = local_map->GetLeftLane(actor_waypoint);
      GetLocalRoadInfo(left_info, left_lane);
    }
    if(actor_right_waypoint != nullptr){
      hasRightLane = true;
      const crd::Lane& right_lane = local_map->GetRightLane(actor_waypoint);
      GetLocalRoadInfo(right_info, right_lane);
    }
  }


  
  // LocalRoadInfo::iterator it = mid_info.begin();
  // while(it != mid_info.end()){
  //   std::cerr << "Key  : " << it->first.first << ", " << it->first.second << "\n";
  //   std::cerr << "Value: " << it->second << "\n";
  //   it++;
  // }

  //Scan the whole vehicle list
  for(ActorId target_id: vehicle_id_list)
  {
    //Filter: current referenced vehicle
    if(target_id == actor_id)
      continue;

    const cg::Location target_location = simulation_state.GetLocation(target_id);
    const cg::Vector3D target_heading = simulation_state.GetHeading(target_id);
    float dot_heading = VectorDotProduct(actor_heading, target_heading);

    //Filter: vehicles are on opposite lane
    if(dot_heading < 0.0f)
      continue;
    
    cg::Location target_local_location = target_location;
    GlobalToLocal(actor_id, target_local_location);
    float target_local_location_x = target_local_location.x; 
    float target_local_location_y = target_local_location.y;
    float target_distance = actor_location.Distance(target_location);

    //Filter: vehicles are out of bounds
    if(target_distance > bound){
      continue;
    }
    bool isFront = target_local_location_x > 0.0f ? true : false ; //REAR is false
    bool isLongitudinalOverlapped = std::abs(target_local_location_x) < (actor_half_length) ? true : false;
    
    WaypointPtr target_waypoint = local_map->GetWaypoint(target_location)->GetWaypoint();
    crd::RoadId target_road_id = target_waypoint->GetRoadId();
    crd::SectionId target_section_id = target_waypoint->GetSectionId();
    crd::LaneId target_lane_id = target_waypoint->GetLaneId();
    RoadSectionPair key_pair = std::make_pair(target_road_id, target_section_id);
    
    if(mid_info.find(key_pair) != mid_info.end() && mid_info.at(key_pair) == target_lane_id){
      if(isFront && target_distance < leader_distance){
        potential_leader = main_leader;
        main_leader = target_id;
        leader_distance = target_distance;
      }
    }
    else if(hasLeftLane && left_info.find(key_pair) != left_info.end() && left_info.at(key_pair) == target_lane_id){
      if(isLongitudinalOverlapped){ 
        if(target_distance < left_distance){
          leftVeh = target_id;
          left_distance = target_distance;
        }
      }
      else{
        if(isFront){
          if(target_distance < left_front_distance){
            leftFrontVeh = target_id;
            left_front_distance = target_distance;
          }
        }
        else{
          if(target_distance < left_rear_distance){
            leftRearVeh = target_id;
            left_rear_distance = target_distance;
          }
        }
      }
    }
    else if(hasRightLane && right_info.find(key_pair) != right_info.end() && right_info.at(key_pair) == target_lane_id){
      if(isLongitudinalOverlapped){ 
        if(target_distance < right_distance){
          rightVeh = target_id;
          right_distance = target_distance;
        }
      }
      else{
        if(isFront){
          if(target_distance < right_front_distance){
            rightFrontVeh = target_id;
            right_front_distance = target_distance;
          }
        }
        else{
          if(target_distance < right_rear_distance){
            rightRearVeh = target_id;
            right_rear_distance = target_distance;
          }
        }
      }
    }  
  }//for-loop

  //Current region.
  SimpleWaypointPtr current_waypoint = local_map->GetWaypoint(actor_location);
  // MTS_Region current_region = GetRegion(actor_id, main_leader, current_waypoint, 0.0); // middle direction = 0
  MTS_Region current_region;
  float lane_width = float(current_waypoint->GetWaypoint()->GetLaneWidth());
  float half_lane_width = lane_width / 2.0f;
  
  float gap = MAX_OBSERVING_DISTANCE;
  float maxSpeed = simulation_state.GetSpeedLimit(actor_id);
  if(main_leader)
  {
    gap = GetGap(actor_id, main_leader.get());
    maxSpeed = std::min(simulation_state.GetVelocity(main_leader.get()).Length(), maxSpeed); // local
  }
  current_region.gap = gap;
  current_region.maxPassingSpeed = maxSpeed;

  cg::Location target = cg::Location(gap / 2.0f + actor_half_length, 0.0f, 0.0f);
  LocalToGlobal(actor_id, target);
cg::Location left_border = cg::Location(gap / 2.0f + actor_half_length, -half_lane_width, 0.0f);
  LocalToGlobal(actor_id, left_border);
  cg::Location right_border = cg::Location(gap / 2.0f + actor_half_length, half_lane_width, 0.0f);
  LocalToGlobal(actor_id, right_border);
  
  current_region.location = target; //now is global, need to trans to local?
  current_region.leftBorder = left_border;
  current_region.rightBorder = right_border;
  current_region.width = lane_width;
  current_region.leftBorderVehicle = leftVeh; 
  current_region.rightBorderVehicle = rightVeh;

  //Left region.
  SimpleWaypointPtr left_lane_waypoint = current_waypoint->GetLeftWaypoint(); 
  MTS_Region left_region;

  if(left_lane_waypoint)
  {  
    //left_region = GetRegion(actor_id, leftFrontVeh, left_lane, -1.0); // left direction = -1
    float left_gap = MAX_OBSERVING_DISTANCE;
    float left_max_speed = maxSpeed;
    if(leftFrontVeh)
    {
      left_gap = GetGap(actor_id, leftFrontVeh.get());
      left_max_speed = simulation_state.GetVelocity(leftFrontVeh.get()).Length();
    }
    left_region.gap = left_gap;
    left_region.maxPassingSpeed = left_max_speed;

    cg::Location left_target = cg::Location(left_gap / 2.0f + actor_half_length, -lane_width, 0.0f);
    LocalToGlobal(actor_id, left_target);
    cg::Location left_left_border = cg::Location(gap / 2.0f + actor_half_length, -lane_width - half_lane_width, 0.0f);
    LocalToGlobal(actor_id, left_left_border);
    cg::Location left_right_border = cg::Location(gap / 2.0f + actor_half_length, lane_width + half_lane_width, 0.0f);
    LocalToGlobal(actor_id, left_right_border);

    left_region.location = left_target; //left_lane->GetLocation(); //offset -> location
    left_region.leftBorder = left_left_border;
    left_region.rightBorder = left_right_border;
    left_region.width = lane_width;
    left_region.rightBorderVehicle = actor_id;
  }
  
  //Right region.
  SimpleWaypointPtr right_lane_waypoint = current_waypoint->GetRightWaypoint();
  MTS_Region right_region;

  if(right_lane_waypoint)
  {  
    //right_region = GetRegion(actor_id, rightFrontVeh, right_lane, 1.0); // right direction = 1
    float right_gap = MAX_OBSERVING_DISTANCE;
    float right_max_speed = maxSpeed;

    if(rightFrontVeh)
    {
      right_gap = GetGap(actor_id, rightFrontVeh.get());
      right_max_speed = simulation_state.GetVelocity(rightFrontVeh.get()).Length();
    }
    right_region.gap = right_gap;
    right_region.maxPassingSpeed = right_max_speed;

    cg::Location right_target = cg::Location(right_gap / 2.0f + actor_half_length, lane_width, 0.0f);
    LocalToGlobal(actor_id, right_target);
    cg::Location right_left_border = cg::Location(gap / 2.0f + actor_half_length, -lane_width - half_lane_width, 0.0f);
    LocalToGlobal(actor_id, right_left_border);
    cg::Location right_right_border = cg::Location(gap / 2.0f + actor_half_length, lane_width + half_lane_width, 0.0f);
    LocalToGlobal(actor_id, right_right_border);

    right_region.location = right_target;//left_lane->GetLocation(); //local
    right_region.leftBorder = right_left_border;
    right_region.rightBorder = right_right_border;
    right_region.width = lane_width;
    right_region.leftBorderVehicle = actor_id;
  }

  //Store final result
  LocalizationData &output = output_array.at(index);
  output.leader.MainLeader = main_leader;
  output.leader.PotentialLeader = potential_leader;
  output.neighbor.LeftVehicle = leftVeh;
  output.neighbor.RightVehicle = rightVeh;
  output.neighbor.LeftFrontVehicle = leftFrontVeh;
  output.neighbor.RightFrontVehicle = rightFrontVeh;
  output.neighbor.LeftRearVehicle = leftRearVeh;
  output.neighbor.RightRearVehicle = rightRearVeh;
  output.situation.CurrentRegion = current_region;
  output.situation.CandidateRegions.push_back(current_region);
  output.situation.CandidateRegions.push_back(left_region);
  output.situation.CandidateRegions.push_back(right_region);
}

// Choose region
float LocalizationStage::ComputeBestLateralOffset(ActorId actor_id, const unsigned long index)
{
  LocalizationData &localization = output_array.at(index);
  std::vector<MTS_Region> candidate_region = localization.situation.CandidateRegions;
  //ActorId *left = localization.neighbor.LeftVehicle; //veh->getLeftVehicle();
  //ActorId *right = localization.neighbor.RightVehicle; //veh->getRightVehicle();
  //MTS_Edge *edge = veh->getLane()->getEdge();

  //int typeCode = veh->getVehicleType()->getTypeCode();
  float actor_half_width = simulation_state.GetDimensions(actor_id).y; // veh->getVehicleType()->getStaticWidth() / 2.0f;
  //float actor_lateral_offset = simulation_state.GetLocation(actor_id).y; //veh->getLateralOffset();
  //float vehOffset = simulation_state.GetLocation(actor_id).x + simulation_state.GetDimensions(actor_id).x;
  cg::Vector3D actor_velocity = simulation_state.GetVelocity(actor_id);
  float actor_forward_velocity = actor_velocity.Length(); //veh->getCurrentSpeed();
  //MTS_Region &validSpace = localization.situation.mCurrentRegion; //param->currentRegion;
  int valid_space_ID = -1;
  MTS_Region best_region;
  //float max_cost = 0.0f;
  float max_valid_cost = 0.0f;
  //int spaceSize = localization.situation.mRegion.size() ;//param->allRegion.size();
  size_t size = candidate_region.size();

  if(size == 0)
    return 0.0;

  for(size_t i = 0; i < size; ++i) // auto& region : localization.situation.CandidateRegions) // int i = 0; i < localization.situation.mRegion.size(); ++i)
  {
    if(candidate_region[i].width < 1.8f * actor_half_width )
      continue;
    //float laneCenter = region->offset;
    // std::array<float, 4> region_local_location = GlobalToLocal(actor_id, candidate_region.at(i).location);
    // float offset_diff = region_local_location.at(1); //region->offset - actor_lateral_offset;
    cg::Location region_local_location = candidate_region.at(i).location;
    GlobalToLocal(actor_id, region_local_location);
    float offset_diff = region_local_location.y;

    float safe_offset;
    bool isSafeSpace = true;

    if(candidate_region[i].width < actor_half_width * 2)
    {
      isSafeSpace = false;
      candidate_region[i].safety = 0.0f;
    }
    else if(offset_diff < 0) // check if the front and the rear of subject vehicle at the desired offset are safe
    {
      bool safeLeftSpace = CheckLeftSafety(actor_id, offset_diff , &safe_offset , candidate_region.at(i), localization);
      bool adjustOffset = localization.situation.SpaceOriented  && safe_offset < 0.0f; //actor_lateral_offset;

      if( !safeLeftSpace && !adjustOffset ) 
        isSafeSpace = false;

      else if( !safeLeftSpace )
        offset_diff = safe_offset; // - actor_lateral_offset;
    }
    else if( offset_diff > 0 ) // check if the front and the rear of subject vehicle at the desired offset are safe
    {
      bool safeRightSpace = CheckRightSafety(actor_id, offset_diff, &safe_offset , candidate_region.at(i), localization);
      bool adjustOffset = localization.situation.SpaceOriented  && safe_offset > 0.0f; //actor_lateral_offset;

      if( !safeRightSpace && !adjustOffset )
        isSafeSpace = false;

      else if( !safeRightSpace )
        offset_diff = safe_offset; // - actor_lateral_offset;
    }
    
    offset_diff = std::abs(offset_diff);
    float gap = candidate_region.at(i).gap;

    cg::Vector3D actor_local_velocity = actor_velocity;
    simulation_state.GetRotation(actor_id).InverseRotateVector(actor_local_velocity);
    bool velCosistent = (offset_diff * actor_local_velocity.y) > 0;
    float speed_diff = candidate_region.at(i).maxPassingSpeed - actor_forward_velocity;
    float safety = candidate_region.at(i).safety;
    //std::cout << "actor_local velocity: " << actor_local_velocity.y << std::endl;
    float w_speed	= 1.8517f; //param->mRegionSelectionWeight->weight_speed;
    float w_gap		= 50.0f; //param->mRegionSelectionWeight->weight_gap;
    float w_dis		= -50.0f; //param->mRegionSelectionWeight->weight_lateralDistance;
    float w_vel		= 30.0f; //param->mRegionSelectionWeight->weight_velocityConsistency;
    float w_safe	= 100.0f; //param->mRegionSelectionWeight->weight_safety;

    // bool dirPriority = false;
    // int laneIdx = edge->getLaneID( region->offset );
    // MTS_Lane *lane = edge->getLane( laneIdx );
    // bool priority = lane->havePriority( typeCode );
    // bool permission = lane->havePermission( typeCode );
    // bool target = laneIdx == veh->getDesireLane();
  
    // MTS_Vehicle *brokenVehicle = lane->getBlockage();
    // bool blockage = lane->endOfRoad() || ( brokenVehicle != NULL && _checkBlockage( region ,  brokenVehicle ) );
    
    // float turnControl = _turnControl( veh , region );
    
    // float w_dir			= param->mRegionSelectionWeight->weight_targetDirection;
    // float w_blockage	= -param->mRegionSelectionWeight->weight_blockage;
    // float w_priority	= param->mRegionSelectionWeight->weight_priority;
    // float w_permission	= param->mRegionSelectionWeight->weight_permission; 
    // float w_target		= param->mRegionSelectionWeight->weight_targetLane;
    // float w_turnControl = param->mRegionSelectionWeight->weight_turnControl;

    candidate_region.at(i).preference = w_speed * speed_diff + w_gap * gap + w_dis * offset_diff + w_vel * velCosistent + w_safe * safety;
                //+ w_dir * dirPriority + w_priority * priority + w_permission * permission + w_target * target + w_blockage * blockage + w_turnControl * turnControl;
    
    if(isSafeSpace && candidate_region.at(i).preference > max_valid_cost)
    {
      valid_space_ID = i;
      max_valid_cost = candidate_region.at(i).preference;
    }
  }
  
  if(valid_space_ID == -1 )
  {
    if(candidate_region.at(0).safety > 0.8f )// veh->needTocutIn &&
    {
      // std::array<float, 4> target_location = GlobalToLocal(actor_id, candidate_region.at(0).location);
      // return target_location.at(1);
      cg::Location target_location = candidate_region.at(0).location;
      GlobalToLocal(actor_id, target_location);
      return target_location.y;
    } 
    return 0.0f;
  }

  // if(actor_id == vehicle_id_list.at(0))
  //   DrawBestRegion(actor_id, candidate_region.at(valid_space_ID));
  
  // std::array<float, 4> target_location = GlobalToLocal(actor_id, candidate_region.at(valid_space_ID).location);
  // std::array<float, 4> target_left_border = GlobalToLocal(actor_id, candidate_region.at(valid_space_ID).leftBorder);
  // std::array<float, 4> target_right_border = GlobalToLocal(actor_id, candidate_region.at(valid_space_ID).rightBorder);
  cg::Location target_location = candidate_region.at(valid_space_ID).location;
  GlobalToLocal(actor_id, target_location);
  cg::Location target_left_border = candidate_region.at(valid_space_ID).leftBorder;
  GlobalToLocal(actor_id, target_left_border);
  cg::Location target_right_border = candidate_region.at(valid_space_ID).rightBorder;
  GlobalToLocal(actor_id, target_right_border);

  if(valid_space_ID == 0)
    return target_location.y;
  
  if(0.0f > target_location.y)
    return target_right_border.y - simulation_state.GetDimensions(actor_id).y * 2;
  
  if(0.0f < target_location.y)
    return target_left_border.y + simulation_state.GetDimensions(actor_id).y * 2;
}

bool LocalizationStage::CheckLeftSafety(ActorId actor_id, float desired_offset , float *safe_offset , MTS_Region region, LocalizationData &localization)
{
  //float lateral_offet = simulation_state.GetLocation(actor_id).y; //local
  float t = GetLateralTime(desired_offset, actor_id, localization);
  float v = desired_offset / t;
  
  //check BorderVehicle
  if(region.leftBorderVehicle)
  {
    ActorId leftVeh = region.leftBorderVehicle.get();
    cg::Location left_veh_location = simulation_state.GetLocation(leftVeh);
    // std::array<float, 4> local_location = GlobalToLocal(actor_id, left_veh_location);
    cg::Location local_location = left_veh_location;
    GlobalToLocal(actor_id, local_location);
    float local_location_x = local_location.x;
    float local_location_y = local_location.y;
    bool lateralOverlap = (GetDynamicWidth(leftVeh) + GetDynamicWidth(actor_id)) / 2.0f < std::abs(local_location_y - desired_offset);
    
    if(lateralOverlap)
    { 
      float actor_head_offset = GetDynamicLength(actor_id);
      const float target_head_offset = local_location_x + GetDynamicLength(leftVeh);
      if(actor_head_offset <= target_head_offset)
        return 0.0f;
    }
  }

  bool safe = true;
  float safe_time;
  float min_safe_time = FLT_MAX;

  if(safe_offset) 
    *safe_offset = desired_offset;
  region.safety = 1.0;
  
  // for( ; it != vehEnd ; ++it )
  // {
  if(localization.neighbor.LeftRearVehicle)
  {
    ActorId left_rear_vehicle = localization.neighbor.LeftRearVehicle.get();
    bool checkSafe = CheckSafety(actor_id, left_rear_vehicle, v, t, &safe_time);
  
  //float patience = mSubject->getPatience();
  // if( !checkSafe && patience > 0.75)
  // {
  //   region->safety = 0.85;
  //   checkSafe = true;
  // }

    if( !checkSafe )
    {
      if( safe_offset && safe_time < min_safe_time )
      {
        min_safe_time = safe_time;
        cg::Location left_rear_location = simulation_state.GetLocation(left_rear_vehicle);
        cg::Location local_location = left_rear_location;
        GlobalToLocal(actor_id, local_location);
        //float local_location_x = local_location.x;
        float local_location_y = local_location.y;
        cg::Vector3D current_speed = simulation_state.GetVelocity(left_rear_vehicle);
        simulation_state.GetRotation(actor_id).InverseRotateVector(current_speed);
        float min_dis = simulation_state.GetDimensions(actor_id).y; //(GetPsychoWidth(simulation_state.GetVelocity(actor_id).x) + simulation_state.GetDimensions(actor_id).y * 2) / 2.0f + 2.0f;
        *safe_offset = local_location_y + current_speed.y * safe_time + min_dis;
      }
      safe = false;
      region.safety = 0.0;
    }
  }

  // const std::vector< MTS_Vehicle* > &frontVehicles = mSubject->getLeftFrontVehicles();
  
  // it = frontVehicles.begin();
  // vehEnd = frontVehicles.end();

  // for( ; it != vehEnd ; ++it )
  // {
  if(localization.neighbor.LeftFrontVehicle)
  {
    ActorId left_front_vehicle = localization.neighbor.LeftFrontVehicle.get();
    bool checkSafe = CheckSafety(actor_id, left_front_vehicle, v, t, &safe_time);
  
  //float patience = mSubject->getPatience();	
  // if( !checkSafe && patience > 0.75)
  // {
  //   if( mSubject->getRelativeOffset( (*it) ) - (*it)->getCurrentController()->getLength()*(-2) > 0.0 )
  //   {
  //     checkSafe = true;
  //     region->safety = 0.5;
  //   }
  // }

    if(!checkSafe)
    {
      if(safe_offset && safe_time < min_safe_time)
      {
        min_safe_time = safe_time;
        cg::Location left_front_location = simulation_state.GetLocation(left_front_vehicle);
        cg::Location local_location = left_front_location;
        GlobalToLocal(actor_id, local_location);
        float local_location_y = local_location.y;
        cg::Vector3D current_speed = simulation_state.GetVelocity(left_front_vehicle);
        simulation_state.GetRotation(actor_id).InverseRotateVector(current_speed);
        float min_dis = simulation_state.GetDimensions(actor_id).y; 
        *safe_offset = local_location_y + current_speed.y * safe_time + min_dis;
      }
      safe = false;
      region.safety = 0.0;
    }
  }

  return safe;
}

bool LocalizationStage::CheckSafety(ActorId actor_id, ActorId target_id, float moveSpeed, float moveTime, float* safeTime)
{
  cg::Vector3D actor_speed = simulation_state.GetVelocity(actor_id);
  simulation_state.GetRotation(actor_id).InverseRotateVector(actor_speed);
  cg::Vector3D target_speed = simulation_state.GetVelocity(target_id);
  simulation_state.GetRotation(actor_id).InverseRotateVector(target_speed);
  //std::cout << "target speed: " << target_speed.y << std::endl;
  cg::Vector2D v_s(actor_speed.x, moveSpeed);
  cg::Vector2D v_o(target_speed.x, target_speed.y);
  float relative_x = GetRelativeOffset(actor_id, target_id);
  float relative_y = GetLateralSeparation(actor_id, target_id);
  cg::Vector2D p_r(relative_x, relative_y);
  cg::Vector2D v_r = v_s - v_o;

  float hw_s = simulation_state.GetDimensions(actor_id).y;
  float hw_o = simulation_state.GetDimensions(target_id).y; //GetPsychoWidth(simulation_state.GetVelocity(actor_id).x)/2.0f; //?
  float hl_s = simulation_state.GetDimensions(actor_id).x;
  float hl_o = simulation_state.GetDimensions(target_id).x;
  float d_y = std::abs(p_r.y);
  float scale = (d_y - hw_s - hw_o) / d_y; 
  if(scale < 0.0)
    scale = 0.0;

  float t_y = p_r.y * scale / v_r.y;

  if(safeTime) 
    *safeTime = t_y;

  if(t_y < 0 || t_y > moveTime) 
  {
    return true;
  }
  
  float d_o = v_o.x * RESPONSE_TIME + (v_o.x * -v_r.x) / (2 * sqrt(MAX_ACC * COMFORTABLE_DEC));
  float d_s = v_s.x * RESPONSE_TIME + (v_s.x * v_r.x) / (2 * sqrt(MAX_ACC * COMFORTABLE_DEC));

  if(d_o < 0.0)
    d_o = 0.0;

  if(d_s < 0.0)
    d_s = 0.0;

  //float p_r_s = p_r.x + v_o.x * t_y;
  //float p_r_o = p_r.x - v_s.x * t_y;
  float p_r_t = p_r.x - v_r.x * t_y;
  //float d_x = std::abs( p_r_t );

  if((p_r_t < 0 && p_r_t + hl_s + hl_o < -d_o * GAP_ACCEPT_RATIO) || (p_r_t >= 0 && p_r_t - hl_s - hl_o > d_s * GAP_ACCEPT_RATIO))
    return true;
  
  return false;
}

bool LocalizationStage::CheckRightSafety(ActorId actor_id, float desired_offset , float *safe_offset , MTS_Region region, LocalizationData &localization)
{
  float t = GetLateralTime(desired_offset, actor_id, localization);
  float v = desired_offset / t;

  //check BorderVehicle
  if(region.rightBorderVehicle)
  {
    ActorId rightVeh = region.rightBorderVehicle.get();
    cg::Location right_veh_location = simulation_state.GetLocation(rightVeh);
    cg::Location local_location = right_veh_location;
    GlobalToLocal(actor_id, local_location);
    bool lateralOverlap = (GetDynamicWidth(rightVeh) + GetDynamicWidth(actor_id)) / 2.0f < std::abs(local_location.y - desired_offset);

    if( lateralOverlap )
    {
      float actor_head_offset = GetDynamicLength(actor_id);
      const float target_head_offset = local_location.x + GetDynamicLength(rightVeh);
      if( actor_head_offset <= target_head_offset )
        return 0.0f;
    }
  }

  bool safe = true;
  float safe_time;
  float min_safe_time = FLT_MAX;
  
  if( safe_offset ) 
    *safe_offset = desired_offset;
  
  region.safety = 1.0;
  
  if(localization.neighbor.RightRearVehicle)
  {
    ActorId right_rear_vehicle = localization.neighbor.RightRearVehicle.get();
    bool checkSafe = CheckSafety(actor_id, right_rear_vehicle, v, t, &safe_time);

    if(!checkSafe)
    {
      if( safe_offset && safe_time < min_safe_time )
      {
        min_safe_time = safe_time;
        cg::Location right_rear_location = simulation_state.GetLocation(right_rear_vehicle);
        cg::Location local_location = right_rear_location;
        GlobalToLocal(actor_id, local_location);
        cg::Vector3D current_speed = simulation_state.GetVelocity(right_rear_vehicle);
        simulation_state.GetRotation(actor_id).InverseRotateVector(current_speed);
        float min_dis = simulation_state.GetDimensions(actor_id).y;
        *safe_offset = local_location.y + current_speed.y * safe_time + min_dis;
      }
      safe = false;
      region.safety = 0.0;
    }
  }

  if(localization.neighbor.RightFrontVehicle)
  {
    ActorId right_front_vehicle = localization.neighbor.RightFrontVehicle.get();
    bool checkSafe = CheckSafety(actor_id, right_front_vehicle, v, t, &safe_time);

    if(!checkSafe)
    {
      if(safe_offset && safe_time < min_safe_time )
      {
        min_safe_time = safe_time;
        cg::Location right_front_location = simulation_state.GetLocation(right_front_vehicle);
        cg::Location local_location = right_front_location;
        GlobalToLocal(actor_id, local_location);
        cg::Vector3D current_speed = simulation_state.GetVelocity(right_front_vehicle);
        simulation_state.GetRotation(actor_id).InverseRotateVector(current_speed);
        float min_dis = simulation_state.GetDimensions(actor_id).y;
        *safe_offset = local_location.y + current_speed.y * safe_time + min_dis;
      }
      safe = false;
      region.safety = 0.0;
    }
  }

  return safe;
}

float LocalizationStage::GetLateralTime(float desired_lateral_offset, ActorId actor_id, LocalizationData &localization)
{
  cg::Vector3D actor_speed = simulation_state.GetVelocity(actor_id);
  simulation_state.GetRotation(actor_id).InverseRotateVector(actor_speed);
  float lateral_veocity = actor_speed.y;
  float lateral_time = 2.0f * desired_lateral_offset / lateral_veocity;

  // if the time to decelerate to zero lateral speed is smaller than maximum movement time
  if( lateral_time > 0.0f && lateral_time < MAX_MOVEMENT_TIME ) 
    return lateral_time;

  return GetLongitudinalTime(actor_id, localization);
}

float LocalizationStage::GetLongitudinalTime(ActorId actor_id, LocalizationData &localization)
{
  float longitudinal_velocity = simulation_state.GetVelocity(actor_id).Length(); 
  float longitudinal_time = GetGapToStopLine(actor_id) / longitudinal_velocity;

  if(localization.leader.MainLeader)
  {
    float s = GetGap(actor_id, localization.leader.MainLeader.get());
    longitudinal_time = std::min(s / longitudinal_velocity, longitudinal_time);
  }

  if( longitudinal_time > 0 && longitudinal_time < MAX_MOVEMENT_TIME )
    return longitudinal_time;

  return MAX_MOVEMENT_TIME;
}

float LocalizationStage::GetGapToStopLine(ActorId actor_id)
{
  float halfVehLen = simulation_state.GetDimensions(actor_id).x;
  float stopOffset = MAX_OBSERVING_DISTANCE; //getLane()->getLength();

  return stopOffset - halfVehLen;
}

// Transform
void LocalizationStage::GlobalToLocal(ActorId actor_id, cg::Location &location)
{
  //use actor_id to get transform matrix(actor as origin of coordinate)
  cg::Location actor_location = simulation_state.GetLocation(actor_id);
  cg::Rotation actor_rotation = simulation_state.GetRotation(actor_id);
  cg::Transform transform(actor_location, actor_rotation);
  transform.InverseTransformPoint(location);
}

void LocalizationStage::LocalToGlobal(ActorId actor_id, cg::Location &location)
{
  cg::Location actor_location = simulation_state.GetLocation(actor_id);
  cg::Rotation actor_rotation = simulation_state.GetRotation(actor_id);
  cg::Transform transform(actor_location, actor_rotation);
  transform.TransformPoint(location);
}

// Tool
float LocalizationStage::GetGap(ActorId actor_id, ActorId target_id)
{
	float halfPredLen = simulation_state.GetDimensions(target_id).x; //pred->getCurrentController()->getLength() / 2.0f;
	float halfVehLen = simulation_state.GetDimensions(actor_id).x; //getLength() / 2.0f;
	float relativeOffset = GetRelativeOffset(actor_id, target_id);
	float gap = relativeOffset - (halfPredLen + halfVehLen);
	
	return gap;
}

float LocalizationStage::GetRelativeOffset(ActorId actor_id, ActorId target_id)
{
  cg::Location target_location = simulation_state.GetLocation(target_id);
  GlobalToLocal(actor_id, target_location);
  return target_location.x;
}


float LocalizationStage::GetDynamicWidth(ActorId actor_id)
{
  float yaw_angle = simulation_state.GetRotation(actor_id).yaw;
  float width = simulation_state.GetDimensions(actor_id).y * 2;

  if(yaw_angle == 0)
    return width;

  float length = simulation_state.GetDimensions(actor_id).x * 2;
  float diagonal_length = sqrt(length * length + width * width);
  float base_angle = acos(length / diagonal_length);
  float angle_1 = std::abs(yaw_angle + base_angle);
  float angle_2 = std::abs(yaw_angle - base_angle);

  if(yaw_angle > 0)
    return diagonal_length * sin(angle_1);
  
  return diagonal_length * sin(angle_2);
}

float LocalizationStage::GetDynamicLength(ActorId actor_id)
{
  float yaw_angle = simulation_state.GetRotation(actor_id).yaw;
  float length = simulation_state.GetDimensions(actor_id).x * 2;

  if(yaw_angle == 0)
    return length;

  float width = simulation_state.GetDimensions(actor_id).y * 2;
  float diagonal_length = sqrt(length * length + width * width);
  float base_angle = acos(length / diagonal_length);

  float angle_1 = std::abs(yaw_angle + base_angle);
  float angle_2 = std::abs(yaw_angle - base_angle);

  if(yaw_angle > 0)
    return diagonal_length * cos(angle_2);
 
  return diagonal_length * cos(angle_1);
}

float LocalizationStage::GetLateralSeparation(ActorId actor_id, ActorId target_id)
{
  cg::Location target_location = simulation_state.GetLocation(target_id);
  GlobalToLocal(actor_id, target_location);
  return -target_location.y;
} 

// Wait
MTS_Region LocalizationStage::GetRegion(ActorId actor_id,  boost::optional<ActorId> target_id, SimpleWaypointPtr target_waypoint, float direction)
{
  MTS_Region tmp_region;
  
  float gap = MAX_OBSERVING_DISTANCE;
  float max_speed = simulation_state.GetSpeedLimit(actor_id);
  if(target_id.get())
  {
    gap = GetGap(actor_id, target_id.get());
    max_speed = std::min(simulation_state.GetVelocity(target_id.get()).Length(), max_speed);
  }
  tmp_region.gap = gap;
  tmp_region.maxPassingSpeed = max_speed;

  float actor_half_length = simulation_state.GetDimensions(actor_id).x;
  float lane_width = float(target_waypoint->GetWaypoint()->GetLaneWidth());
  float half_lane_width = lane_width / 2.0f;
  cg::Location center = cg::Location(gap / 2.0f + actor_half_length, direction * lane_width, 0.0f);
  LocalToGlobal(actor_id, center);
  cg::Location left_border = cg::Location(gap / 2.0f + actor_half_length, direction * lane_width - half_lane_width, 0.0f);
  LocalToGlobal(actor_id, left_border);
  cg::Location right_border = cg::Location(gap / 2.0f + actor_half_length, direction * lane_width + half_lane_width, 0.0f);
  LocalToGlobal(actor_id, right_border);

  tmp_region.location = center;
  tmp_region.leftBorder = left_border;
  tmp_region.rightBorder = right_border;
  tmp_region.width = lane_width;

  return tmp_region;
}

void LocalizationStage::GetLocalRoadInfo(LocalRoadInfo& info, const crd::Lane& lane)
{  
  //Add current info
  crd::LaneId lane_id = lane.GetId();
  crd::SectionId section_id = lane.GetLaneSection()->GetId();
  crd::RoadId road_id = lane.GetRoad()->GetId();
  info.insert({{std::make_pair(road_id, section_id), lane_id}});
  
  bool isOtherRoad = false;
  crd::RoadId flag_road_id = road_id;

  //Add next info 
  std::vector<crd::Lane*> next_lanes = lane.GetNextLanes();
  while(!next_lanes.empty() && next_lanes.size() == 1)
  {
    crd::Lane* next_lane = next_lanes.at(0);
    crd::LaneId next_lane_id = next_lane->GetId();
    crd::SectionId next_section_id = next_lane->GetLaneSection()->GetId();
    crd::RoadId next_road_id = next_lane->GetRoad()->GetId();
    if(next_road_id != flag_road_id)
    {
      if(isOtherRoad)
        break;
      isOtherRoad = true;
      flag_road_id = next_road_id;
    }
    info.insert({{std::make_pair(next_road_id, next_section_id), next_lane_id}});
    next_lanes = next_lane->GetNextLanes();//TODO:Modify it or it will be infinitly loop!
  }
  
  isOtherRoad = false;
  flag_road_id = road_id;

  //Add Previous info
  std::vector<crd::Lane*> previous_lanes = lane.GetPreviousLanes();
  while(!previous_lanes.empty() && previous_lanes.size() == 1)
  {
    crd::Lane* previous_lane = previous_lanes.at(0);
    crd::LaneId previous_lane_id = previous_lane->GetId();
    crd::SectionId previous_section_id = previous_lane->GetLaneSection()->GetId();
    crd::RoadId previous_road_id = previous_lane->GetRoad()->GetId();
    if(previous_road_id != flag_road_id)
    {
      if(isOtherRoad)
        break;
      isOtherRoad = true;
      flag_road_id = previous_road_id;
    }
    info.insert({{std::make_pair(previous_road_id, previous_section_id), previous_lane_id}});
    previous_lanes = previous_lane->GetPreviousLanes();//TODO:Modify it or it will be infinitly loop!
  }
}

/*
// Region
void LocalizationStage::UpdateRegion(ActorId actor_id)
{
  // for( int i=0;i<mRegion.size();++i )
  // {
  //   mRegion[i].frontVehicles.clear();
  //   mRegion[i].rearVehicles.clear();
  // }
  const cg::Location vehicle_location = simulation_state.GetLocation(actor_id);
  SimpleWaypointPtr current_waypoint = local_map->GetWaypoint(vehicle_location);

  localization.situation.CandidateRegions.clear();
  localization.situation.CurrentRegion.clear();

  // if(!localization.situation.SpaceOriented) // mSubject->getVehicleType()->getTypeCode() != 2 )
  // {
    UpdateCurrentRegion(actor_id, current_waypoint, localization.situation.CurrentRegion);
    FindLane(actor_id, current_waypoint, localization.situation.CandidateRegions);
  // }
  // else
  // {
  //   if(!this->mSubject->needTocutIn) // 
  //   {
  //     UpdateBaseSpaceRegion(actor_id, localization.neighbor.LeftVehicle, localization.neighbor.RightVehicle); //mSubject->getLeftVehicle() , mSubject->getRightVehicle() );
  //     FindSpace(actor_id , pred , localization.situation.CandidateRegions );
  //   }
  //   else
  //   {
  //     UpdateCurrentRegion(actor_id, current_waypoint, localization.situation.CurrentRegion);
  //     FindLane(actor_id, current_waypoint, localization.situation.CandidateRegions);
  //   }
  // }
}

void LocalizationStage::UpdateCurrentRegion(const ActorId actor_id, SimpleWaypointPtr current_waypoint, MTS_Region &current_region)
{
  float gap = GetGapToStopLine();
  ActorId leader = localization.leader.MainLeader;
  if(leader)
    gap = GetGap(actor_id, leader);

  float half_lane_width = current_waypoint->GetLaneWidth() / 2.0f; //lane->getWidth()/2.0f;

  current_region.gap = gap;
  current_region.offset = current_waypoint.GetLocation().y; //local //lane->getCentralOffset();
  current_region.leftBorder = current_region.offset - half_lane_width;
  current_region.rightBorder = current_region.offset + half_lane_width;
  current_region.width = 2.0f * half_lane_width;

  float maxSpeed = std::min(simulation_state.GetSpeedLimit(actor_id);, GetDesiredSpeed());
  
  if(leader)
    maxSpeed = std::min(simulation_state.GetVelocity(leader).x, maxSpeed); // local
  
  current_region.maxPassingSpeed = maxSpeed;
}
 
void LocalizationStage::FindLane(const ActorId actor_id, SimpleWaypointPtr current_waypoint, std::vector< MTS_Region > &result )
{
  MTS_Region temp_region;

  SimpleWaypointPtr left_lane = current_waypoint.GetLeftWaypoint(); //veh->getLane()->getLeftLane();
  SimpleWaypointPtr right_lane = current_waypoint.GetRightWaypoint();//veh->getLane()->getRightLane();

  result.push_back(localization.situation.CurrentRegion);

  temp_region = SetLaneRegion(actor_id, current_waypoint);
  result.push_back(temp_region);
  if(left_lane)
  {
    temp_region = SetLaneRegion(actor_id, left_lane);
    result.push_back(temp_region);
  }
  
  if(right_lane)
  {
    temp_region = SetLaneRegion(actor_id, right_lane);
    result.push_back(temp_region);
  }
}

MTS_Region LocalizationStage::SetLaneRegion(const ActorId actor_id, const  SimpleWaypointPtr *lane )
{
  MTS_Region result_region;
  //MTS_VehicleController *vehController =  veh->getVehicleController();
  float half_lane_width = lane->GetLaneWidth() / 2.0f;

  result_region.offset = lane->GetLocation().y; //local
  result_region.leftBorder = result_region.offset - half_lane_width;
  result_region.rightBorder = result_region.offset + half_lane_width;
  result_region.width = 2.0f * half_lane_width;
  result_region.leftBorderVehicle = output.neighbor.LeftVehicle; //((MTS_Vehicle *)veh)->getLeftVehicle();
  result_region.rightBorderVehicle = output.neighbor.RightVehicle; //((MTS_Vehicle *)veh)->getRightVehicle();

  
  float minGap = lane->getLength();
  if(output.neighbor.LeftFrontVehicle)
  {
    minGap = GetGap(actor_id, output.neighbor.LeftFrontVehicle)
  }
  result_region.gap = minGap;

  float maxSpeed = std::min(lane->getMaxPassingSpeed(), GetDesiredSpeed());
  if(output.neighbor.LeftFrontVehicle)
  {
    maxSpeed = simulation_state.GetVelocity(output.neighbor.LeftFrontVehicle).x;
  }
  result_region.maxPassingSpeed = maxSpeed;

  // float minOffset = simulation_state.GetLocation(actor_id).x + simulation_state.GetDimensions(actor_id).x;//veh->getOffset() + vehController->getLength() / 2.0f;
  // float maxOffset = minOffset + 500.0f;
  // std::vector< MTS_Vehicle* > predVeh;
  // lane->getVehicleInBlock( minOffset , maxOffset , predVeh );
  // float predVehSize = predVeh.size();

  // for( int i = 0 ; i < predVehSize ; ++i )
  // {
  //   float offset = predVeh[i]->getOffset() - predVeh[i]->getVehicleController()->getLength()/2.0f;
  //   if( offset > minOffset && offset < minGap ) 
  //   {
  //     minGap = offset;
  //     maxSpeed = predVeh[i]->getCurrentSpeed();
  //   }
  // }

  // cause we have only one left front vehicle now.

  //minGap -= minOffset;
  
  return resultSpace;
}


float LocalizationStage::GetGapToStopLine() const
{
  float half_veh_len = simulation_state.GetDimensions(actor_id).x;
  // float headOffset = getOffset() + halfVehLen;
  //float stopOffset = getLane()->getLength();
  //trafficlight location - halflength

  return MAX_OBSERVING_DISTANCE - half_veh_len; //stopOffset - headOffset;
}

void LocalizationStage::UpdateBaseSpaceRegion( const ActorId *actor_id, const ActorId *left_veh, const ActorId *right_veh)
{
  // MTS_Vehicle *pred = veh->getPassedLeadingVehicle();
  // MTS_Lane *lane = veh->getLane();
  // MTS_Edge *edge = lane->getEdge();
  // MTS_VehicleController *controller = veh->getCurrentController();
  
  float gap = GetGapToStopLine();
  ActorId leader = localization.leader.MainLeader;
  if(leader)
    gap = GetGap(actor_id, leader);

  CurrentRegion.gap = gap;

  float left_offset, right_offset;

  if(!left_veh)
    left_offset = edge->getMinOffset(); // local road left distance
  else
    left_offset = simulation_state.GetLocation(left_veh).y + GetPsychoWidth(left_veh, simulation_state.GetVelocity(actor_id).x) / 2.0f; //local

  if(!right_veh)
    right_offset =edge->getMaxOffset(); // local road right distance
  else
    right_offset = simulation_state.GetLocation(right_veh).y - GetPsychoWidth(right_veh, simulation_state.GetVelocity(actor_id).x) / 2.0f;

  CurrentRegion.leftBorder = left_offset;
  CurrentRegion.rightBorder = right_offset;
  CurrentRegion.width = right_offset - left_offset;
  CurrentRegion.offset = simulation_state.GectLocation(actor_id).y // local //veh->getLateralOffset();

  float max_speed = std::min(lane->getMaxPassingSpeed(), GetDesiredSpeed(actor_id));//
  if(leader)
    max_speed = std::min(simulation_state.GetVelocity(leader), max_speed);
  CurrentRegion.maxPassingSpeed = max_speed;
}

void LocalizationStage::FindSpace(const ActorId *actor_id, MTS_Vehicle *pred , std::vector< MTS_Region > &result )
{
  std::vector<MTS_Region> candidateSpace;
  MTS_Region temp_space;
  MTS_Region temp_space_2;
  //float vehLatOffset = simulation_state.GetLocation(actor_id).y;//local //veh->getLateralOffset();

  // compute the width of the space, the preferred lateral offset, and the maximum passing speed
  result.push_back(CurrentRegion);

  ActorId leftVehicle = localization.neighbor.LeftVehicle; //veh->getLeftVehicle();
  ActorId rightVehicle = localization.neighbor.RightVehicle; //veh->getRightVehicle();
  ActorId pre_leftVehicle;
  ActorId pre_rightVehicle;
  
  ActorId leader = localization.leader.MainLeader;
  if(leader)
  {
    pre_leftVehicle = leader.localization.neighbor.LeftVehicle; //pred->getLeftVehicle();
    pre_rightVehicle =  leader.localization.neighbor.RightVehicle; //pred->getRightVehicle();
  }
  else
    return;

  temp_space = SetSpaceRegion( veh , leftVehicle ,1.0f, leader ,-1.0f, DesiredDirection::DIR_NONE );
  result.push_back( temp_space );

  if(!leftVehicle)
  {
    temp_space_2 = SetSpaceRegion( veh , pre_leftVehicle ,1.0f, leader ,-1.0f, DesiredDirection::DIR_NONE );
    result.push_back( temp_space_2 );
  }
  else
  {
    float sl_offset = simulation_state.GetLocation(leftVehicle).y + GetDynamicWidth(leftVehicle) / 2.0f; //local
    float ol_offset = -1.0f;
    if(leader)
    {
      ol_offset = simulation_state.GetLocation(pred).y - GetDynamicWidth(leader) / 2.0f; //local
    }
    
    if( sl_offset > ol_offset )
    {
      temp_space_2 = SetSpaceRegion( actor_id , pre_leftVehicle ,1.0f, leader ,-1.0f, DesiredDirection::DIR_NONE );
      result.push_back( temp_space_2 );
    }		
  }
  
  temp_space = SetSpaceRegion( actor_id , leader ,1.0f, rightVehicle ,-1.0f, DesiredDirection::DIR_NONE );
  result.push_back( temp_space );

  if(!rightVehicle)
  {
    temp_space_2 = SetSpaceRegion( actor_id , leader ,1.0f, pre_rightVehicle ,-1.0f, DesiredDirection::DIR_NONE );
    result.push_back( temp_space_2 );
  }
  else
  {
    float sr_offset = simulation_state.GetLocation(rightVehicle).y + GetDynamicWidth(rightVehicle) / 2.0f;
    float or_offset = FLT_MAX;
    if(leader)
    {
      or_offset = simulation_state.GetLocation(pred).y - GetDynamicWidth(leader) / 2.0f; //local
    }
    if( sr_offset < or_offset )
    {
      temp_space_2 = SetSpaceRegion( actor_id , leader ,1.0f, pre_rightVehicle ,-1.0f, DesiredDirection::DIR_NONE );
      result.push_back( temp_space_2 );
    }
  }
}

MTS_Region LocalizationStage::SetSpaceRegion( const MTS_Vehicle *veh , const MTS_Vehicle *leftVeh , float leftSign , const MTS_Vehicle *rightVeh , float rightSign , DesiredDirection preferedDirection )
{
  float leftOffset, rightOffset;
  MTS_Vehicle *passedVeh = veh->getPassedLeadingVehicle();

  MTS_Edge *edge = veh->getLane()->getEdge();

  MTS_Region resultSpace;

  if( leftVeh == NULL )
    leftOffset = edge->getMinOffset();
  else{
    float leftVehWidth = leftVeh->getVehicleType()->getDynamicWidth( leftVeh->getYawAngle() );
    leftOffset = leftVeh->getLateralOffset() + leftSign*( leftVeh->getVehicleController()->getPsychoWidth( mSubject->getCurrentSpeed() ) + leftVehWidth )/2.0f;
  }
  if( rightVeh == NULL )
    rightOffset = edge->getMaxOffset();
  else{
    float rightVehWidth = rightVeh->getVehicleType()->getDynamicWidth( rightVeh->getYawAngle() );
    rightOffset = rightVeh->getLateralOffset() + rightSign*(rightVeh->getVehicleController()->getPsychoWidth( mSubject->getCurrentSpeed() ) + rightVehWidth)/2.0f;
  }
  resultSpace.leftBorderVehicle = (MTS_Vehicle*)leftVeh;
  resultSpace.rightBorderVehicle = (MTS_Vehicle*)rightVeh;
  resultSpace.leftBorder = leftOffset  ;
  resultSpace.rightBorder = rightOffset ;

  resultSpace.width = resultSpace.rightBorder - resultSpace.leftBorder;
  
  MTS_Lane *currentLane = veh->getLane();
  
  float vehWidth = veh->getVehicleController()->getWidth();
  float vehStaticWidth = veh->getVehicleType()->getStaticWidth();
  float halfVehStaticWidth = vehStaticWidth / 2.0f;
  float halfLaneWidth = currentLane->getWidth() / 2.0f;

  float offset_center = ( leftOffset + rightOffset ) / 2.0f;
  float offset_left = leftOffset + halfLaneWidth;
  float offset_right = rightOffset - halfLaneWidth;
  float vehOffset = veh->getLateralOffset();

  float dis_left = std::abs((offset_left - vehOffset));
  float dis_right = std::abs((offset_right - vehOffset));

  // prefered direction instruct which direction the subject want to move
  if( resultSpace.width <= 2*halfLaneWidth )
    resultSpace.offset = offset_center;
  else if( dis_left < dis_right )
    resultSpace.offset = offset_left;
  else
    resultSpace.offset = offset_right;
  
  float responseTime = veh->getResponseTime();
  
  int desiredLaneIdx = edge->getLaneID( resultSpace.offset );
  MTS_Lane *desiredLane = edge->getLane( desiredLaneIdx );

  _findGapAndSpeed( veh , resultSpace.leftBorder , resultSpace.rightBorder ,resultSpace);
  resultSpace.maxPassingSpeed = std::min(veh->getDesiredSpeed() , resultSpace.maxPassingSpeed );
  resultSpace.maxPassingSpeed = std::min(desiredLane->getMaxPassingSpeed() , resultSpace.maxPassingSpeed );

  resultSpace.safety = 0.0;
  return resultSpace;
}

void LocalizationStage::FindGapAndSpeed( const MTS_Vehicle *veh , float minLateralBorder , float maxLateralBorder , MTS_Region &region)
{
  float vehHead = veh->getOffset() + veh->getVehicleType()->getDynamicLength( veh->getYawAngle() ) / 2.0f;
  float vehSpeed = veh->getCurrentSpeed();

  MTS_Lane* currentLane = veh->getLane();
  MTS_Edge* edge = currentLane->getEdge();
  int laneSize = edge->getLaneSize();
  float currentMin = currentLane->getLength()-veh->getOffset()-veh->getVehicleType()->getStaticLength();
  MTS_Vehicle* minObject  = NULL;
  for( int i=0;i<laneSize;++i )
  {
    MTS_Lane* lane = edge->getLane(i);
    float gap ;
    MTS_Vehicle* object = lane->getVehCloseAndBiger( minLateralBorder,maxLateralBorder,vehHead,gap, vehSpeed);
    if( object!=NULL && gap<currentMin  ) currentMin = gap;

  }
  region.gap = currentMin;
  if( minObject!= NULL )
  {
    region.maxPassingSpeed = minObject->getCurrentSpeed();
  }
  else
  {
    region.maxPassingSpeed = veh->getLane()->getMaxPassingSpeed();
  }
}

float LocalizationStage::GetDynamicWidth(ActorId actor_id) const
{
  float yaw_angle = simulation_state.GetRotation(actor_id).yaw;
  float width = simulation_state.GetDimensions(actor_id).y * 2;

  if(yaw_angle == 0)
    return width;

  float length = simulation_state.GetDimensions(actor_id).x * 2;
  float diagonal_length = sqrt(length * length + width * width);
  float base_angle = acos(length / diagonal_length);
  float angle_1 = std::abs(yaw_angle + base_angle);
  float angle_2 = std::abs(yaw_angle - base_angle);

  if(yaw_angle > 0)
    return diagonal_length * sin(angle_1);
  
  return diagonal_length * sin(angle_2);
}

// Safety
void LocalizationStage::evaluateSafety()
{
  MTS_VehicleController *subjectController = mSubject->getCurrentController();
  float mDesiredLateralOffset = subjectController->getDesiredLateralOffset();

  float x_current = subjectController->getLateralOffset();
  float v_current = subjectController->getLateralSpeed();
  float x_safe;
  mCollisionTime = 2.0f;
  float t = 3.0f;
  
  if( v_current > 0 )
  {
    bool safe = _checkRightCollision( v_current , t , &x_safe );
    
    if( !safe )
    {
      mDesiredLateralOffset = MIN(  mDesiredLateralOffset , x_safe );
      safety = 0.0f;
      return ;
      
    }
  }
  else if( v_current < 0 )
  {
    bool safe = _checkLeftCollision( v_current , t , &x_safe );
    
    if( !safe )
    {
      mDesiredLateralOffset = MAX(  mDesiredLateralOffset , x_safe );
      safety = 0.0f;
      return ;
      
    }
  }
  safety = 1.0f;

}

bool LocalizationStage::_checkLeftCollision( float v , float t , float *safeOffset )
{
  const std::vector< MTS_Vehicle* > &rearVehicles = mSubject->getLeftRearVehicles();
  std::vector< MTS_Vehicle* >::const_iterator it = rearVehicles.begin();
  std::vector< MTS_Vehicle* >::const_iterator vehEnd = rearVehicles.end();

  float safeTime;
  float minSafeTime = t;
  bool safe = true;
  float t_s = mSubject->getResponseTime();

  for( ; it != vehEnd ; ++it )
  {
    bool safetyResult = _checkSafety( mSubject , *it , v , t , true , true , &safeTime );
    if( !safetyResult && mSubject->getPatience() > 0.75)
    {
      safetyResult = true;
    }

    if( !safetyResult )
    {
      if( safeOffset && safeTime < minSafeTime )
      {
        minSafeTime = safeTime;
        MTS_VehicleController *controller = (*it)->getCurrentController();
        float currentOffset = controller->getLateralOffset();
        float currentSpeed = controller->getLateralSpeed();
        float minDis = ( controller->getPsychoWidth( mSubject->getCurrentSpeed() ) + mSubject->getVehicleType()->getStaticWidth() ) / 2.0f + 2.0f;
        *safeOffset = currentOffset + currentSpeed * safeTime + minDis;
      }
      safe = false;
    }
  }

  const std::vector< MTS_Vehicle* > &frontVehicles = mSubject->getLeftFrontVehicles();
  
  it = frontVehicles.begin();
  vehEnd = frontVehicles.end();

  for( ; it != vehEnd ; ++it )
  {
    bool safetyResult = _checkSafety( mSubject , *it , v , t , false , true , &safeTime );
    if( !safetyResult )
    {
      if( safeOffset && safeTime < minSafeTime )
      {
        minSafeTime = safeTime;
        MTS_VehicleController *controller = (*it)->getCurrentController();
        float currentOffset = controller->getLateralOffset();
        float currentSpeed = controller->getLateralSpeed();
        float minDis = ( controller->getPsychoWidth( mSubject->getCurrentSpeed() ) + mSubject->getVehicleType()->getStaticWidth() ) / 2.0f + 2.0f;
        *safeOffset = currentOffset +currentSpeed * safeTime + minDis;
      }
      safe = false;
    }
  }

  if( minSafeTime != 0.0f )
    mCollisionTime = minSafeTime;

  return safe;
}

bool LocalizationStage::_checkRightCollision( float v , float t , float *safeOffset )
{
  const std::vector< MTS_Vehicle* > &rearVehicles = mSubject->getRightRearVehicles();
  std::vector< MTS_Vehicle* >::const_iterator it = rearVehicles.begin();
  std::vector< MTS_Vehicle* >::const_iterator vehEnd = rearVehicles.end();

  float safeTime;
  float minSafeTime = t;
  bool safe = true; 
  float t_s = mSubject->getResponseTime();

  for( ; it != vehEnd ; ++it )
  {
    MTS_VehicleController *controller = (*it)->getCurrentController();

    bool safetyResult = _checkSafety( mSubject , *it , v , t , true , true , &safeTime );
    if( !safetyResult )
    {
      if( safeOffset && safeTime < minSafeTime )
      {
        minSafeTime = safeTime;
        
        float currentOffset = controller->getLateralOffset();
        float currentSpeed = controller->getLateralSpeed();
        float minDis = ( controller->getPsychoWidth( mSubject->getCurrentSpeed() ) + mSubject->getVehicleType()->getStaticWidth() ) / 2.0f + 2.0f;
        *safeOffset = currentOffset +currentSpeed * safeTime - minDis;
      }
      safe = false;
    }
  }

  const std::vector< MTS_Vehicle* > &frontVehicles = mSubject->getRightFrontVehicles();
  
  it = frontVehicles.begin();
  vehEnd = frontVehicles.end();

  for( ; it != vehEnd ; ++it )
  {
    MTS_VehicleController *controller = (*it)->getCurrentController();
    bool safetyResult = _checkSafety( mSubject , *it , v , t , false , true , &safeTime );
    if( !safetyResult )
    {
      if( safeOffset && safeTime < minSafeTime )
      {
        minSafeTime = safeTime;
        
        float currentOffset = controller->getLateralOffset();
        float currentSpeed = controller->getLateralSpeed();
        float minDis = ( controller->getPsychoWidth( mSubject->getCurrentSpeed() ) + mSubject->getVehicleType()->getStaticWidth() ) / 2.0f + 2.0f;
        *safeOffset = currentOffset +currentSpeed * safeTime - minDis;
      }
      safe = false;
    }
  }

  if( minSafeTime != 0.0f )
    mCollisionTime = minSafeTime;
  
  return safe;
}

bool LocalizationStage::_checkSafety( MTS_Vehicle *subject , MTS_Vehicle *object , float moveSpeed , float moveTime , bool subjectAsLeader , bool subjectAsFollower , float* safeTime ) const
{
  MTS_VehicleController *subjectController = subject->getCurrentController();
  MTS_VehicleController *objectController = object->getCurrentController();
  const MTS_Edge *last = objectController->cooperate( subjectController );

  Vector2 v_s( subjectController->getCurrentSpeed() , moveSpeed );
  Vector2 v_o( objectController->getCurrentSpeed() , objectController->getLateralSpeed() );

  Vector2 p_r;
  p_r.y = object->getLateralSeparation( subject );
  p_r.x = object->getRelativeOffset( subject );

  Vector2 v_r = v_s - v_o;
  
  float hw_s = subjectController->getWidth()/2.0f;
  float hw_o = objectController->getPsychoWidth( mSubject->getCurrentSpeed() )/2.0f;

  float hl_s = subjectController->getLength()/2.0f;
  float hl_o = objectController->getLength()/2.0f;

  objectController->bindEdge( last );

  float d_y = ABS( p_r.y );

  float scale = ( d_y - hw_s - hw_o ) / d_y;
  scale = MAX( 0 , scale );

  float t_y = p_r.y * scale / v_r.y;
  if( safeTime ) *safeTime = t_y;

  
  if( t_y < 0 || t_y > moveTime) 
  {
    return true;
  }
  
  float d_o = 
        v_o.x * object->getResponseTime() + 
        ( v_o.x * -v_r.x ) / ( 2 * sqrt( object->getMaxAcceleration() * object->getComfortableDeceleration() ) );
  float d_s = 
        v_s.x * subject->getResponseTime() + 
        ( v_s.x * v_r.x ) / ( 2 * sqrt( subject->getMaxAcceleration() * subject->getComfortableDeceleration() ) );

  d_o = MAX( 0 , d_o);
  d_s = MAX( 0 , d_s);

  float p_r_s = p_r.x + v_o.x * t_y;
  float p_r_o = p_r.x - v_s.x * t_y;
  float p_r_t = p_r.x - v_r.x * t_y ;
  float d_x = ABS( p_r_t );
  const float ACEPTED_RATIO = 0.7f;

  if( ( p_r_t < 0 && p_r_t + hl_s + hl_o < -d_o * ACEPTED_RATIO ) || (  p_r_t >= 0 && p_r_t - hl_s - hl_o > d_s * ACEPTED_RATIO ) )
    return true;
  
  return false;
}
*/
/*******Leader and Neighbor Part*******/
/****Unused Function START****
// Leader
void LocalizationStage::UpdateLeader(const unsigned long index)
{
  const ActorId actor_id = vehicle_id_list.at(index);
  const cg::Location actor_location = simulation_state.GetLocation(actor_id);
  const cg::Vector3D actor_heading = simulation_state.GetHeading(actor_id);
  float actor_offset = simulation_state.GetDimensions(actor_id).x;
  
  LocalizationData &output = output_array.at(index);
  boost::optional<ActorId> potential_leader; // c++17: std, but c++14: boost instead.
  boost::optional<ActorId> main_leader;

  float minOffset = actor_offset;
  float maxOffset = minOffset + MAX_OBSERVING_DISTANCE;
  
  for(ActorId target_id: vehicle_id_list)
  {
    if( target_id == actor_id)
      continue;

    const cg::Location target_location = simulation_state.GetLocation(target_id);
    const cg::Vector3D target_heading = simulation_state.GetHeading(target_id);

    // distance.
    float target_distance = actor_location.Distance(target_location);
    // heading.
    float dot_heading = VectorDotProduct(actor_heading, target_heading);

    if(dot_heading >= 0.0f && target_distance <= maxOffset) 
    {
      float target_length = simulation_state.GetDimensions(target_id).x;
      std::array<float, 4> target_local_location = GlobalToLocal(actor_id, target_location);
      float target_offset = target_local_location.at(0) - target_length; 
      bool blocked = isOverlapped(actor_id, target_id, target_local_location.at(1));

      if( target_offset > 0.0f && blocked )
      {
        potential_leader = main_leader;
        main_leader = target_id;
        maxOffset = target_distance;
      }
    }
  }

  output.leader.MainLeader = main_leader;
  output.leader.PotentialLeader = potential_leader;
}

// Neighbor
void LocalizationStage::UpdateNeighbor(const unsigned long index)
{
  GetLeftVehicle(index);
  GetRightVehicle(index);
  GetSurroundVehicle(index);
}

void LocalizationStage::GetLeftVehicle(const unsigned long index)
{
  const ActorId actor_id = vehicle_id_list.at(index);
  //const cg::Location actor_location = simulation_state.GetLocation(actor_id);
  const cg::Vector3D actor_heading = simulation_state.GetHeading(actor_id);
  float maxOffset = simulation_state.GetDimensions(actor_id).x;
  
  LocalizationData &output = output_array.at(index);
  boost::optional<ActorId> leftVeh;
  float min_lat_diff = FLT_MAX;
  float block_len = maxOffset;

  for(ActorId target_id: vehicle_id_list)
  {
    if( target_id == actor_id)
      continue;

    const cg::Location target_location = simulation_state.GetLocation(target_id);
    const cg::Vector3D target_heading = simulation_state.GetHeading(target_id);
    
    float dot_heading = VectorDotProduct(actor_heading, target_heading);

    if(dot_heading >= 0.0f)
    {
      std::array<float, 4> target_local_location = GlobalToLocal(actor_id, target_location);
      float halfVehLen = simulation_state.GetDimensions(target_id).x;
      float vehOffset = target_local_location.at(0); 
      float dis_long = std::abs(vehOffset);
      
      float lat_diff = target_local_location.at(1);
      float lat_diff_abs = std::abs(lat_diff);
      
      // placed in the block // left hand side or right hand side of the baseOffset // distance is smaller
      if(dis_long < block_len + halfVehLen && (lat_diff < 0) && lat_diff_abs < min_lat_diff) 
      {
        leftVeh = target_id;
        min_lat_diff = lat_diff_abs;
      }
    }
    
  }

  output.neighbor.LeftVehicle = leftVeh;
}

void LocalizationStage::GetRightVehicle(const unsigned long index)
{
  
  const ActorId actor_id = vehicle_id_list.at(index);
  //const cg::Location actor_location = simulation_state.GetLocation(actor_id);
  const cg::Vector3D actor_heading = simulation_state.GetHeading(actor_id);
  float maxOffset = simulation_state.GetDimensions(actor_id).x;

  LocalizationData &output = output_array.at(index);
  boost::optional<ActorId> rightVeh;
  float min_lat_diff = FLT_MAX;
  float block_len = maxOffset;

  for(ActorId target_id: vehicle_id_list)
  {
    if( target_id == actor_id)
      continue;

    const cg::Location target_location = simulation_state.GetLocation(target_id);
    const cg::Vector3D target_heading = simulation_state.GetHeading(target_id);
    
    float dot_heading = VectorDotProduct(actor_heading, target_heading);
    if(dot_heading >= 0.0f)
    {
      std::array<float, 4> target_local_location = GlobalToLocal(actor_id, target_location);
      float halfVehLen = simulation_state.GetDimensions(target_id).x;
      float vehOffset = target_local_location[0]; 
      float dis_long = std::abs(vehOffset);
      
      float lat_diff = target_local_location[1];
      float lat_diff_abs = std::abs(lat_diff);
      
      // placed in the block // left hand side or right hand side of the baseOffset // distance is smaller
      if(dis_long < block_len + halfVehLen && (lat_diff > 0) && lat_diff_abs < min_lat_diff)
      {
        rightVeh = target_id;
        min_lat_diff = lat_diff_abs;
      }
    }
  }

  output.neighbor.RightVehicle = rightVeh;
}

void LocalizationStage::GetSurroundVehicle(const unsigned long index)
{
  float observe_distance = 80.0;
  float max_distance = observe_distance;
  float min_distance = -observe_distance; //Modified
  const ActorId actor_id = vehicle_id_list.at(index);

  LocalizationData &output = output_array.at(index);
  boost::optional<ActorId> leftFrontVeh;
  boost::optional<ActorId> rightFrontVeh;
  boost::optional<ActorId> leftRearVeh;
  boost::optional<ActorId> rightRearVeh;
  
  for(ActorId target_id: vehicle_id_list)
  {
    if( target_id == actor_id)
      continue;

    // target location transform to local coordinate.
    const cg::Location target_location = simulation_state.GetLocation(target_id);
    const std::array<float, 4> target_local_location = GlobalToLocal(actor_id, target_location);
    float target_local_location_x = target_local_location.at(0);
    float target_local_location_y = target_local_location.at(1);

    // need to filter other road or lane or direction
    if( !isOverlapped(actor_id, target_id, target_local_location_y))
    {
      if(target_local_location_x >= 0.0 && target_local_location_x <= max_distance)
      {
        if(target_local_location_y < 0 )
        {
          leftFrontVeh = target_id;
        }
        else if(target_local_location_y > 0 )
        {
          rightFrontVeh = target_id;
        }
      }
      else if (target_local_location_x < 0.0 && target_local_location_x >= min_distance)
      {
        if(target_local_location_y < 0 )
        {
          leftRearVeh = target_id;
        }
        else if(target_local_location_y > 0 )
        {
          rightRearVeh = target_id;
        }
      }
    }
  }

  output.neighbor.LeftFrontVehicle = leftFrontVeh;
  output.neighbor.RightFrontVehicle = rightFrontVeh;
  output.neighbor.LeftRearVehicle = leftRearVeh;
  output.neighbor.RightRearVehicle = rightRearVeh;
}

void LocalizationStage::getVehicleInBlock( float minOffset , float maxOffset , std::vector<MTS_Vehicle*> &result ) const
{
  std::deque<MTS_Vehicle*>::const_iterator it = mVehicles.begin();
  
  float block_middle = (minOffset+maxOffset)/2.0f;
  float block_len = maxOffset - block_middle;
  const float MAXLEN = 120.0f;

  for( ; it!=mVehicles.end() ; ++it )
  {
    if ( !(*it)->getActive() ) continue;
    MTS_VehicleController *controller = (*it)->getCurrentController();
    const MTS_Edge *last = controller->bindEdge( mEdge );
    float halfVehLen = controller->getLength() / 2.0f;
    float vehOffset = controller->getOffset();
    float dis_long = ABS( (block_middle-vehOffset) );
    controller->bindEdge( last );
    if( dis_long < block_len + halfVehLen )
    {
      result.push_back( *it );
    }
  //	else if( block_middle-vehOffset > MAXLEN + block_len )
  //		break;
    
  }
}

float LocalizationStage::getExtendedGap( const MTS_Vehicle *pred ) const
{
  MTS_VehicleController *controller = pred->getCurrentController();

  if( controller->getYawAngle() == 0.0f )
    return 0.0f;

  float sepLatOffset = controller->getSeparationLateralOffset();
  float myLatOffset = getLateralOffset();

  float predHalfWidth = pred->getCurrentController()->getPsychoWidth( mSubject->getCurrentSpeed() ) / 2.0f;
  float myHalfWidth = mSubject->mType->getStaticWidth() / 2.0f;
  float latSeparation = mSubject->getLateralSeparation( pred );
  latSeparation = ABS( latSeparation );
  if( latSeparation > predHalfWidth + myHalfWidth )
    return 0.0f;

  float myLeftLatOffset = myLatOffset - myHalfWidth;
  float myRightLatOffset = myLatOffset + myHalfWidth;
  float extendedGap = 0.0f;
  
  if( myLeftLatOffset > sepLatOffset )
    extendedGap = controller->getExtendedDistance( myLeftLatOffset );
  else if( myRightLatOffset < sepLatOffset )
    extendedGap = controller->getExtendedDistance( myRightLatOffset );

  return extendedGap;
}

float MTS_VehicleController::getExtendedDistance( float lateralOffset ) const
{
  float dis = 0.0f;
  if( lateralOffset > mGapVariable.SeparationLateralOffset )
  {
    float ratio = ( lateralOffset - mGapVariable.SeparationLateralOffset ) / mGapVariable.RightWidth;
    ratio = MIN( 1.0f , ratio );
    dis = mGapVariable.MaxRightGap * ratio;
  }
  else
  {
    float ratio = ( mGapVariable.SeparationLateralOffset  -  lateralOffset ) / mGapVariable.LeftWidth;
    ratio = MIN( 1.0f , ratio );
    dis = mGapVariable.MaxLeftGap * ratio;
  }
  return dis;
}

// tool
bool LocalizationStage::isOverlapped(ActorId actor_id, ActorId target_id, float target_location_y) const
{
  //don't need to worry about that while go straight forward.
  //float test = mSubject->getOrientation().dotProduct( veh->getOrientation() );
  //if( test  >= 0.8f ) 
  //	return mSubject->getLane()->isLateralOverlapping( mSubject , veh );

  //float myWidth = simulate mSubject->mType->getStaticWidth();
  //float vehWidth = veh->getCurrentController()->getPsychoWidth( mSubject->getCurrentSpeed() );
  //float width_sum =  vehWidth + myWidth;

  float actor_width = simulation_state.GetDimensions(actor_id).y;
  // float actor_location_y = simulation_state.GetLocation(actor_id).y;
  // //float actor_velocity_y = simulation_state.GetVelocity(actor_id).y;
  float target_width = simulation_state.GetDimensions(target_id).y;
  // float target_location_y = simulation_state.GetLocation(target_id).y;
  //float target_velocity_y = simulation_state.GetVelocity(target_id).y;

  float width_sum =  actor_width + target_width;
  //float myOffset_lat = getLateralOffset();

  //float vehOffset_lat = veh->getCurrentController()->getLateralOffset();
  //float x_dis = ABS( myOffset_lat - vehOffset_lat );

  //float x_dis = mSubject->getLateralSeparation( veh );
  
  float y_dis = std::abs(target_location_y);

  if( y_dis >= width_sum ) //Modified
  {
    //if(  ABS(mSubject->getLateralSeparation( veh , 0.25f)) < width_sum/2.0f )
    //	return true;
    return false;
  }
  return true;
}

bool LocalizationStage::isLogitudinalOverlapped(ActorId actor_id, ActorId target_id, float target_location_x) const
{
  float actor_length = simulation_state.GetDimensions(actor_id).x;
  float target_length = simulation_state.GetDimensions(target_id).x;


  float length_sum =  actor_length + target_length;
  
  float x_dis = std::abs(target_location_x);

  if( x_dis >= length_sum )
  {
    return false;
  }
  return true;
}
******Unused Function END******/

} // namespace traffic_manager
} // namespace carla
