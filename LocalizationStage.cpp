
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

  // Assign a lane change.
  const ChangeLaneInfo lane_change_info = parameters.GetForceLaneChange(actor_id);
  bool force_lane_change = lane_change_info.change_lane;
  bool lane_change_direction = lane_change_info.direction;

  if (!force_lane_change) {
    float perc_keep_right = parameters.GetKeepRightPercentage(actor_id);
    if (perc_keep_right >= 0.0f && perc_keep_right >= pgen.next()) {
      force_lane_change = true;
      lane_change_direction = true;
    }
  }

  const SimpleWaypointPtr front_waypoint = waypoint_buffer.front();
  const float lane_change_distance = SQUARE(std::max(10.0f * vehicle_speed, INTER_LANE_CHANGE_DISTANCE));

  bool recently_not_executed_lane_change = last_lane_change_location.find(actor_id) == last_lane_change_location.end();
  bool done_with_previous_lane_change = true;
  if (!recently_not_executed_lane_change) {
    float distance_frm_previous = cg::Math::DistanceSquared(last_lane_change_location.at(actor_id), vehicle_location);
    done_with_previous_lane_change = distance_frm_previous > lane_change_distance;
  }
  bool auto_or_force_lane_change = parameters.GetAutoLaneChange(actor_id) || force_lane_change;
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

  /*MTS CALL*/
  UpdateLeader(index);
  //UpdateNeighbor(index);
  
  // Editing output array
  LocalizationData &output = output_array.at(index);
  output.is_at_junction_entrance = is_at_junction_entrance;

  if (actor_id == vehicle_id_list.at(0))
  {
    DrawLeader(actor_id, output);
  }

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
    const auto blocking_vehicles = track_traffic.GetOverlappingVehicles(actor_id);

    // Find immediate in-lane obstacle and check if any are too close to initiate lane change.
    bool obstacle_too_close = false;
    float minimum_squared_distance = std::numeric_limits<float>::infinity();
    ActorId obstacle_actor_id = 0u;
    for (auto i = blocking_vehicles.begin();
         i != blocking_vehicles.end() && !obstacle_too_close && !force;
         ++i) {
      const ActorId &other_actor_id = *i;
      // Find vehicle in buffer map and check if it's buffer is not empty.
      if (buffer_map.find(other_actor_id) != buffer_map.end() && !buffer_map.at(other_actor_id).empty()) {
        const Buffer &other_buffer = buffer_map.at(other_actor_id);
        const SimpleWaypointPtr &other_current_waypoint = other_buffer.front();
        const cg::Location other_location = other_current_waypoint->GetLocation();

        const cg::Vector3D reference_heading = current_waypoint->GetForwardVector();
        cg::Vector3D reference_to_other = other_location - current_waypoint->GetLocation();
        const cg::Vector3D other_heading = other_current_waypoint->GetForwardVector();

        WaypointPtr current_raw_waypoint = current_waypoint->GetWaypoint();
        WaypointPtr other_current_raw_waypoint = other_current_waypoint->GetWaypoint();
        // Check both vehicles are not in junction,
        // Check if the other vehicle is in front of the current vehicle,
        // Check if the two vehicles have acceptable angular deviation between their headings.
        if (!current_waypoint->CheckJunction()
            && !other_current_waypoint->CheckJunction()
            && other_current_raw_waypoint->GetRoadId() == current_raw_waypoint->GetRoadId()
            && other_current_raw_waypoint->GetLaneId() == current_raw_waypoint->GetLaneId()
            && cg::Math::Dot(reference_heading, reference_to_other) > 0.0f
            && cg::Math::Dot(reference_heading, other_heading) > MAXIMUM_LANE_OBSTACLE_CURVATURE) {
          float squared_distance = cg::Math::DistanceSquared(vehicle_location, other_location);
          // Abort if the obstacle is too close.
          if (squared_distance > SQUARE(MINIMUM_LANE_CHANGE_DISTANCE)) {
            // Remember if the new vehicle is closer.
            if (squared_distance < minimum_squared_distance && squared_distance < SQUARE(MAXIMUM_LANE_OBSTACLE_DISTANCE)) {
              minimum_squared_distance = squared_distance;
              obstacle_actor_id = other_actor_id;
            }
          } else {
            obstacle_too_close = true;
          }
        }
      }
    }

    // If a valid immediate obstacle found.
    if (!obstacle_too_close && obstacle_actor_id != 0u && !force) {
      const Buffer &other_buffer = buffer_map.at(obstacle_actor_id);
      const SimpleWaypointPtr &other_current_waypoint = other_buffer.front();
      const auto other_neighbouring_lanes = {other_current_waypoint->GetLeftWaypoint(),
                                             other_current_waypoint->GetRightWaypoint()};

      // Flags reflecting whether adjacent lanes are free near the obstacle.
      bool distant_left_lane_free = false;
      bool distant_right_lane_free = false;

      // Check if the neighbouring lanes near the obstructing vehicle are free of other vehicles.
      bool left_right = true;
      for (auto &candidate_lane_wp : other_neighbouring_lanes) {
        if (candidate_lane_wp != nullptr &&
            track_traffic.GetPassingVehicles(candidate_lane_wp->GetId()).size() == 0) {

          if (left_right)
            distant_left_lane_free = true;
          else
            distant_right_lane_free = true;
        }
        left_right = !left_right;
      }

      // Based on what lanes are free near the obstacle,
      // find the change over point with no vehicles passing through them.
      if (distant_right_lane_free && right_waypoint != nullptr
          && track_traffic.GetPassingVehicles(right_waypoint->GetId()).size() == 0) {
        change_over_point = right_waypoint;
      } else if (distant_left_lane_free && left_waypoint != nullptr
               && track_traffic.GetPassingVehicles(left_waypoint->GetId()).size() == 0) {
        change_over_point = left_waypoint;
      }
    } else if (force) {
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

void LocalizationStage::DrawBuffer(Buffer &buffer) {
  uint64_t buffer_size = buffer.size();
  uint64_t step_size =  buffer_size/20u;
  cc::DebugHelper::Color color {0u, 0u, 0u};
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

void LocalizationStage::DrawLeader(ActorId actor_id, LocalizationData &output)
{
  cg::Location actor_location = simulation_state.GetLocation(actor_id);
  actor_location.z += 3.0f;
  cc::DebugHelper::Color color_ego {255u, 0u, 0u};
  cc::DebugHelper::Color color_main_leader {0u, 255u, 0u};
  cc::DebugHelper::Color color_potential_leader {0u, 0u, 255u};
  //Draw range line
  const cg::Vector3D actor_heading = simulation_state.GetHeading(actor_id);
  cg::Vector3D actor_heading_unit = actor_heading.MakeUnitVector();
  cg::Location actor_location_end = actor_location + cg::Location(actor_heading_unit *= MAX_OBSERVING_DISTANCE);
  debug_helper.DrawLine(actor_location, actor_location_end, 0.1f, color_ego, 0.2f, true);
  // debug_helper.DrawPoint(actor_location, 1.0f, color_ego, 0.1, true);
  debug_helper.DrawBox( cg::BoundingBox(actor_location, cg::Vector3D(0.3,0.3,0.3)), simulation_state.GetRotation(actor_id), 0.1f, color_ego, 0.2f, true);
  if(output.leader.main_leader)
  {
    cg::Location first_location = simulation_state.GetLocation(output.leader.main_leader.get());
    first_location.z += 3.0f;
    //debug_helper.DrawPoint(first_location, 0.1f, color_main_leader, 0.1, true);
    debug_helper.DrawBox( cg::BoundingBox(first_location, cg::Vector3D(0.3,0.3,0.3)), simulation_state.GetRotation(actor_id), 0.1f, color_main_leader, 0.2f, true);
    }
  if(output.leader.potential_leader)
  { 
    cg::Location second_location = simulation_state.GetLocation(output.leader.potential_leader.get());
    second_location.z += 3.0f;
    //debug_helper.DrawPoint(second_location, 1.0f, color_potential_leader, 0.1, true);
    debug_helper.DrawBox( cg::BoundingBox(second_location, cg::Vector3D(0.3,0.3,0.3)), simulation_state.GetRotation(actor_id), 0.1f, color_potential_leader, 0.2f, true);
  }
}

/************************
*******MTS SECTION*******
************************/

// Leader
void LocalizationStage::UpdateLeader(const unsigned long index)
{
  const ActorId actor_id = vehicle_id_list.at(index);
  
  const cg::Location actor_location = simulation_state.GetLocation(actor_id);
  LocalizationData &output = output_array.at(index);
  // std::vector<std::vector<float>> M;
  // M.resize(4, std::vector<float>(4, 0.0f));
  // M = getMatrix(actor_location, actor_rotation);
  // std::vector<std::vector<float>> M_inv;
  // M_inv.resize(4, std::vector<float>(4, 0.0f));
  // M_inv = inverse(M);
  //float actor_local_location[4][1] = GlobalToLocal(M_inv, actor_location);

  float actor_offset = simulation_state.GetDimensions(actor_id).x;
  SimpleWaypointPtr actor_waypoint = local_map->GetWaypoint(actor_location);
  // crd::RoadId actor_road = actor_waypoint->GetWaypoint()->GetRoadId();
  // crd::LaneId actor_lane = actor_waypoint->GetWaypoint()->GetLaneId();
  boost::optional<ActorId> expectedPreVeh; // c++17, but c++14.
  boost::optional<ActorId> passedPreVeh;

  float minOffset = actor_offset;
  float maxOffset = minOffset + MAX_OBSERVING_DISTANCE;
  //float passOffset = FLT_MAX;
  //float expectOffset = FLT_MAX;
  
  for(ActorId target_id: vehicle_id_list)
  {
    if( target_id == actor_id)
      continue;

    const cg::Location target_location = simulation_state.GetLocation(target_id);
    SimpleWaypointPtr target_waypoint = local_map->GetWaypoint(target_location);
    // crd::RoadId target_road = target_waypoint->GetWaypoint()->GetRoadId();
    // crd::LaneId target_lane = target_waypoint->GetWaypoint()->GetLaneId();
    const cg::Vector3D actor_forward_vector = simulation_state.GetHeading(actor_id);
    //cg::Vector3D actor_forward_vector = actor_waypoint->GetForwardVector();
    const cg::Vector3D target_forward_vector = simulation_state.GetHeading(target_id);
    //cg::Vector3D target_forward_vector = target_waypoint->GetForwardVector();
    
    // location heady
    //float target_direction = DeviationDotProduct(actor_location, actor_forward_vector, target_location);
    // distance.
    float target_distance = actor_waypoint->Distance(target_waypoint);
    // heading.
    float target_heading = VectorDotProduct(actor_forward_vector, target_forward_vector);

    if(target_heading >= 0.0f && target_distance <= maxOffset) //&& target_direction > 0.0f 
    // if (actor_road == target_road && abs(actor_lane -  target_lane) <= 1 && actor_waypoint->Distance(target_waypoint) <= maxOffset)
    {
      float target_length = simulation_state.GetDimensions(target_id).x;
      std::array<float, 4> target_local_location = GlobalToLocal(actor_id, target_location);
      float target_offset = target_local_location[0] - target_length; 
      bool blocked = isOverlapped(actor_id, target_id, target_local_location[1]);

      if( target_offset > 0.0f && blocked ) //target_offset < passOffset && blocked ) 
      {
        expectedPreVeh = passedPreVeh;
        passedPreVeh = target_id;
        maxOffset = target_distance;
        // expectOffset = passOffset;
        //passOffset = target_offset;

      }
      // else if(target_offset < expectOffset && target_offset > actor_offset && blocked )
      // {
      //   expectedPreVeh = target_id; 
      //   expectOffset = target_offset;
      // }
    }
  }
  output.leader.main_leader = passedPreVeh;
  output.leader.potential_leader = expectedPreVeh;
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
  LocalizationData &output = output_array.at(index);
  const ActorId actor_id = vehicle_id_list.at(index);
  const cg::Location actor_location = simulation_state.GetLocation(actor_id);
  SimpleWaypointPtr actor_waypoint = local_map->GetWaypoint(actor_location);
  //cg::Vector3D actor_forward_vector = actor_waypoint->GetForwardVector();
  //float minOffset = -simulation_state.GetDimensions(actor_id).x;
  float maxOffset = simulation_state.GetDimensions(actor_id).x;
  boost::optional<ActorId> leftVeh;
  float min_lat_diff = FLT_MAX;
  float block_len = maxOffset;

  for(ActorId target_id: vehicle_id_list)
  {
    if( target_id == actor_id)
      continue;

    const cg::Location target_location = simulation_state.GetLocation(target_id);
    SimpleWaypointPtr target_waypoint = local_map->GetWaypoint(target_location);
    //float target_direction = DeviationDotProduct(actor_location, actor_forward_vector, target_location);
    // need to filter oppsite vehicle?
    std::array<float, 4> target_local_location = GlobalToLocal(actor_id, target_location);
    float halfVehLen = simulation_state.GetDimensions(target_id).x;
    float vehOffset = target_local_location[0]; 
    float dis_long = std::abs(vehOffset);
    float lat_diff = -target_local_location[1];
    float lat_diff_abs = std::abs(lat_diff);
    
    // placed in the block // left hand side or right hand side of the baseOffset // make sure the vehicle is not the referenced vehicle // distance is smaller
    if(dis_long < block_len + halfVehLen && (lat_diff < 0) && lat_diff_abs != 0.0f && lat_diff_abs < min_lat_diff)
    {
      leftVeh = target_id;
      min_lat_diff = lat_diff_abs;
    }
  }

  output.surrounding.LeftVehicle = leftVeh;
}

void LocalizationStage::GetRightVehicle(const unsigned long index)
{
  LocalizationData &output = output_array.at(index);
  const ActorId actor_id = vehicle_id_list.at(index);
  //float minOffset = -simulation_state.GetDimensions(actor_id).x;
  float maxOffset = simulation_state.GetDimensions(actor_id).x;
  boost::optional<ActorId> rightVeh;
  float min_lat_diff = FLT_MAX;
  //float block_middle = 0.0f;
  float block_len = maxOffset;

  for(ActorId target_id: vehicle_id_list)
  {
    if( target_id == actor_id)
      continue;

    const cg::Location target_location = simulation_state.GetLocation(target_id);
    SimpleWaypointPtr target_waypoint = local_map->GetWaypoint(target_location);
    //float target_direction = DeviationDotProduct(actor_location, heading_vector, target_location);
    // need to filter oppsite vehicle?
    std::array<float, 4> target_local_location = GlobalToLocal(actor_id, target_location);
    float halfVehLen = simulation_state.GetDimensions(target_id).x;
    float vehOffset = target_local_location[0]; 
    float dis_long = std::abs(vehOffset);
    float lat_diff = -target_local_location[1];
    float lat_diff_abs = std::abs(lat_diff);
    
    // placed in the block // left hand side or right hand side of the baseOffset // make sure the vehicle is not the referenced vehicle // distance is smaller
    if(dis_long < block_len + halfVehLen && (lat_diff > 0) && lat_diff_abs != 0.0f && lat_diff_abs < min_lat_diff)
    {
      rightVeh = target_id;
      min_lat_diff = lat_diff_abs;
    }
  }

  output.surrounding.RightVehicle = rightVeh;
}

void LocalizationStage::GetSurroundVehicle(const unsigned long index)
{
  float observe_distance = 2000.0;
  float max_distance = observe_distance;
  float min_distance = observe_distance; 
  LocalizationData &output = output_array.at(index);
  const ActorId actor_id = vehicle_id_list.at(index);
  
  for(ActorId target_id: vehicle_id_list)
  {
    if( target_id == actor_id)
      continue;

    // target location transform to local coordinate.
    const cg::Location target_location = simulation_state.GetLocation(target_id);
    std::array<float, 4> target_local_location = GlobalToLocal(actor_id, target_location);

    // need to filter other road or lane or direction
    if( !isOverlapped(actor_id, target_id, target_local_location[1]))
    {
      if(target_local_location[0] >= 0.0 && target_local_location[0] <= max_distance)
      {
        if(target_local_location[1] < 0 )
        {
          output.surrounding.neighbor[Direction::LEFTFRONT]->vehicles.push_back(target_id);
        }
        else if(target_local_location[1] > 0 )
        {
          output.surrounding.neighbor[Direction::RIGHTFRONT]->vehicles.push_back(target_id);
        }
      }
      else if (target_local_location[0] < 0.0 && target_local_location[0] >= min_distance)
      {
        if(target_local_location[1] < 0 )
        {
          output.surrounding.neighbor[Direction::LEFTREAR]->vehicles.push_back(target_id);
        }
        else if(target_local_location[1] > 0 )
        {
          output.surrounding.neighbor[Direction::RIGHTREAR]->vehicles.push_back(target_id);
        }
      }
    }
  }
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

  float actor_width = simulation_state.GetDimensions(actor_id).y;// controller->getLength();
  // float actor_location_y = simulation_state.GetLocation(actor_id).y;
  // //float actor_velocity_y = simulation_state.GetVelocity(actor_id).y;
  float target_width = simulation_state.GetDimensions(target_id).y;// controller->getLength();
  // float target_location_y = simulation_state.GetLocation(target_id).y;
  //float target_velocity_y = simulation_state.GetVelocity(target_id).y;

  float width_sum =  actor_width + target_width;
  //float myOffset_lat = getLateralOffset();

  //float vehOffset_lat = veh->getCurrentController()->getLateralOffset();
  //float x_dis = ABS( myOffset_lat - vehOffset_lat );

  //float x_dis = mSubject->getLateralSeparation( veh );
  
  float y_dis = std::abs(target_location_y);

  if( 2.0f * y_dis >= width_sum )
  {
    //if(  ABS(mSubject->getLateralSeparation( veh , 0.25f)) < width_sum/2.0f )
    //	return true;
    return false;
  }
  return true;
}

// transform
std::array<float, 4> LocalizationStage::GlobalToLocal(ActorId actor_id, cg::Location location)
{
  cg::Location actor_location = simulation_state.GetLocation(actor_id);
  cg::Rotation actor_rotation = simulation_state.GetRotation(actor_id);
  cg::Transform transform(actor_location, actor_rotation);
  std::array<float, 16> M_inv = transform.GetInverseMatrix(); // world to local
  std::array<float, 4> global_location = {location.x, location.y, location.z, 1.0f};
  
  return matrixMultiply(M_inv, global_location);
}

std::array<float, 4> LocalizationStage::matrixMultiply(std::array<float, 16> M, std::array<float, 4> V)
{
  std::array<float, 4> result;
  
  for(size_t i=0; i < V.size(); i++ ){ // should check column or row major
    result[i]= M[4*i] * V[0] + M[4*i+1] * V[1] + M[4*i+2] * V[2] + M[4*i+3] * V[3];
  }

  return result;
}


/*******Transform Part*******/
/****Unused Function START****
std::array<float, 4> LocalizationStage::LocalToGlobal(ActorId actor_id, cg::Location local_location)
{
  cg::Location actor_location = simulation_state.GetLocation(actor_id);
  cg::Rotation actor_rotation = simulation_state.GetRotation(actor_id);
  cg::Transform transform(actor_location, actor_rotation);
  std::array<float, 16> M = transform.GetMatrix(); // local to world
  return matrixMultiply(M, local_location);
}
******Unused Function END*****/

/******Find Leader Part******/
/****Unused Function START****
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
******Unused Function END******/

/******Find Region Part******/
/****Unused Function START****
// Situation
void LocalizationStage::updateRegion()
{
  for( int i=0;i<mRegion.size();++i )
  {
    mRegion[i].frontVehicles.clear();
    mRegion[i].rearVehicles.clear();
  }
  
  mRegion.clear();
  
  //mSpaceOriented = false;
  MTS_Vehicle *pred = mSubject->getPassedLeadingVehicle();

  if( mSubject->getVehicleType()->getTypeCode() != 2 )
  {
    _updateBaseRegion( mSubject , mSubject->getLane() );
    _findLane( mSubject, mRegion );
  }
  else
  {
    float lateralMovingForce = getLateralMovingForce()*3.0f;

    lateralMovingForce = 3.0f;
    float randValue = (rand()%100000) / 100000.0f;
    if( randValue < lateralMovingForce && !this->mSubject->needTocutIn)
    {
      _updateBaseRegion( mSubject , mSubject->getLeftVehicle() , mSubject->getRightVehicle() );
      _findSpace( mSubject , pred , mRegion );

    }
    else
    {
      _updateBaseRegion( mSubject , mSubject->getLane() );
      _findLane( mSubject, mRegion );
    }
    
  }
    
  mLane = mSubject->getLane();
  mEdge = mSubject->getLane()->getEdge();
}
****Unused Function END****/

/******Check Safety Part******/
/****Unused Function START****

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

//Should be region part 
void LocalizationStage::_updateBaseRegion( const MTS_Vehicle *veh , const MTS_Lane *lane )
{
  MTS_Vehicle *pred = veh->getPassedLeadingVehicle();
  MTS_VehicleController *controller =  veh->getCurrentController();
  float halfLaneWidth = lane->getWidth()/2.0f;

  float gap = controller->getGapToStopLine();
  if( pred != NULL )
    gap = controller->getGap( pred );
  mCurrentRegion.gap = gap;

  mCurrentRegion.offset = lane->getCentralOffset();
  mCurrentRegion.leftBorder = mCurrentRegion.offset - halfLaneWidth;
  mCurrentRegion.rightBorder = mCurrentRegion.offset + halfLaneWidth;
  mCurrentRegion.width = 2.0f * halfLaneWidth;

  float maxSpeed = controller->getDesiredSpeed();
  maxSpeed = MIN( lane->getMaxPassingSpeed() , maxSpeed );
  if( pred ) maxSpeed = MIN( pred->getCurrentSpeed() , maxSpeed );
  mCurrentRegion.maxPassingSpeed = maxSpeed;
}

//Should be region part 
void LocalizationStage::_updateBaseRegion( const MTS_Vehicle *veh , const MTS_Vehicle *leftVeh , const MTS_Vehicle *rightVeh )
{
  MTS_Vehicle *pred = veh->getPassedLeadingVehicle();
  MTS_Lane *lane = veh->getLane();
  MTS_Edge *edge = lane->getEdge();

  float leftOffset, rightOffset;
  MTS_VehicleController *controller = veh->getCurrentController();
  float gap = controller->getGapToStopLine();
  if( pred != NULL )
    gap = controller->getGap( pred );

  mCurrentRegion.gap = gap;

  if( leftVeh == NULL )
    leftOffset = edge->getMinOffset() ;
  else
    leftOffset = leftVeh->getLateralOffset() + leftVeh->getVehicleController()->getPsychoWidth( mSubject->getCurrentSpeed() )/2.0f;

  if( rightVeh == NULL )
    rightOffset =edge->getMaxOffset() ;
  else
    rightOffset = rightVeh->getLateralOffset() - rightVeh->getVehicleController()->getPsychoWidth( mSubject->getCurrentSpeed() )/2.0f;

  mCurrentRegion.leftBorder = leftOffset;
  mCurrentRegion.rightBorder = rightOffset;

  mCurrentRegion.width = rightOffset - leftOffset;
  
  float vehWidth = veh->getVehicleController()->getWidth();
  float vehStaticWidth = veh->getVehicleType()->getStaticWidth();
  float halfVehStaticWidth = vehStaticWidth / 2.0f;

  mCurrentRegion.offset = veh->getLateralOffset();

  float maxSpeed = controller->getDesiredSpeed();
  maxSpeed = MIN( lane->getMaxPassingSpeed() , maxSpeed );
  if( pred ) maxSpeed = MIN( pred->getCurrentSpeed() , maxSpeed );
  mCurrentRegion.maxPassingSpeed = maxSpeed;
}

//Should be region part 
void LocalizationStage::_findSpace(  MTS_Vehicle *veh , MTS_Vehicle *pred , std::vector< MTS_Region > &result )
{
  std::vector<MTS_Region> candidateSpace;
  MTS_Region spaceData;
  MTS_Region spaceData2;
  float vehLatOffset = veh->getLateralOffset();

  // compute the width of the space, the preferred lateral offset, and the maximum passing speed
  result.push_back( mCurrentRegion );

  MTS_Vehicle* leftVehicle = veh->getLeftVehicle();
  MTS_Vehicle* rightVehicle = veh->getRightVehicle();
  MTS_Vehicle* pre_leftVehicle = NULL;
  MTS_Vehicle* pre_rightVehicle = NULL;
  if( pred != NULL )
  {
    pre_leftVehicle = pred->getLeftVehicle();
    pre_rightVehicle = pred->getRightVehicle();
  }
  else return;

  spaceData = _setRegionParameter( veh , leftVehicle ,1.0f, pred ,-1.0f, DesiredDirection::DIR_NONE );
  result.push_back( spaceData );

  if( veh->getLeftVehicle() == NULL )
  {
    spaceData2 = _setRegionParameter( veh , pre_leftVehicle ,1.0f, pred ,-1.0f, DesiredDirection::DIR_NONE );
    result.push_back( spaceData2 );
  }
  else
  {
    float sl_offset = leftVehicle->getLateralOffset() + leftVehicle->getVehicleType()->getDynamicWidth(  leftVehicle->getCurrentController()->getYawAngle() )/2.0f ;
    float ol_offset = -1.0f;
    if( pred!=NULL )
    {
      ol_offset = pred->getLateralOffset() -  pred->getVehicleType()->getDynamicWidth( pred->getCurrentController()->getYawAngle() )/2.0f;
    }
    
    if( sl_offset > ol_offset )
    {
      spaceData2 = _setRegionParameter( veh , pre_leftVehicle ,1.0f, pred ,-1.0f, DesiredDirection::DIR_NONE );
      result.push_back( spaceData2 );
    }		
  }
  
  spaceData = _setRegionParameter( veh , pred ,1.0f, rightVehicle ,-1.0f, DesiredDirection::DIR_NONE );
  result.push_back( spaceData );

  if( veh->getRightVehicle() == NULL )
  {
    spaceData2 = _setRegionParameter( veh , pred ,1.0f, pre_rightVehicle ,-1.0f, DesiredDirection::DIR_NONE );
    result.push_back( spaceData2 );
  }
  else
  {
    float sr_offset = rightVehicle->getLateralOffset() - rightVehicle->getVehicleType()->getDynamicWidth(  rightVehicle->getCurrentController()->getYawAngle() )/2.0f ;
    float or_offset = FLT_MAX;
    if( pred!=NULL )
    {
      or_offset = pred->getLateralOffset() +  pred->getVehicleType()->getDynamicWidth( pred->getCurrentController()->getYawAngle() )/2.0f;
    }
    if( sr_offset < or_offset )
    {
      spaceData2 = _setRegionParameter( veh , pred ,1.0f, pre_rightVehicle ,-1.0f, DesiredDirection::DIR_NONE );
      result.push_back( spaceData2 );
    }
  }
}

//Should be region part 
void LocalizationStage::_findLane( const MTS_Vehicle *veh , std::vector< MTS_Region > &result )
{
  MTS_Region spaceData;
  std::vector< MTS_Region > candidateSpace;

  MTS_Lane* leftLane = veh->getLane()->getLeftLane();
  MTS_Lane* rightLane = veh->getLane()->getRightLane();

  result.push_back( mCurrentRegion );

  spaceData = _setRegionParameter( veh , veh->getLane() );
  result.push_back( spaceData );
  if( leftLane )
  {
    spaceData = _setRegionParameter( veh , leftLane );
    result.push_back( spaceData );
  }
  
  if( rightLane )
  {
    spaceData = _setRegionParameter( veh , rightLane );
    result.push_back( spaceData );
  }

}

MTS_Region LocalizationStage::_setRegionParameter( const MTS_Vehicle *veh , const MTS_Lane *lane )
{
  MTS_Region resultSpace;
  MTS_VehicleController *vehController =  veh->getVehicleController();
  float halfLaneWidth = lane->getWidth()/2.0f;

  resultSpace.offset = lane->getCentralOffset();
  resultSpace.leftBorder = resultSpace.offset - halfLaneWidth;
  resultSpace.rightBorder = resultSpace.offset + halfLaneWidth;
  resultSpace.width = 2.0f * halfLaneWidth;

  resultSpace.leftBorderVehicle = ((MTS_Vehicle *)veh)->getLeftVehicle();
  resultSpace.rightBorderVehicle = ((MTS_Vehicle *)veh)->getRightVehicle();

  float minOffset = veh->getOffset() + vehController->getLength() / 2.0f;
  float maxOffset = minOffset + 500.0f;
  float laneLen = lane->getLength();
  std::vector< MTS_Vehicle* > predVeh;

  lane->getVehicleInBlock( minOffset , maxOffset , predVeh );
  float predVehSize = predVeh.size();
  float maxSpeed = vehController->getDesiredSpeed();
  maxSpeed = MIN( lane->getMaxPassingSpeed() , maxSpeed );

  float minGap = laneLen;
  for( int i = 0 ; i < predVehSize ; ++i )
  {
    float offset = predVeh[i]->getOffset() - predVeh[i]->getVehicleController()->getLength()/2.0f;
    if( offset > minOffset && offset < minGap ) 
    {
      minGap = offset;
      maxSpeed = predVeh[i]->getCurrentSpeed();
    }
  }

  minGap -= minOffset;
  resultSpace.maxPassingSpeed = maxSpeed;
  resultSpace.gap = minGap;

  return resultSpace;
}

MTS_Region LocalizationStage::_setRegionParameter( const MTS_Vehicle *veh , const MTS_Vehicle *leftVeh , float leftSign , const MTS_Vehicle *rightVeh , float rightSign , DesiredDirection preferedDirection )
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

  float dis_left = ABS( (offset_left-vehOffset) );
  float dis_right = ABS( (offset_right-vehOffset) );

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
  resultSpace.maxPassingSpeed = MIN( veh->getDesiredSpeed() , resultSpace.maxPassingSpeed );
  resultSpace.maxPassingSpeed = MIN( desiredLane->getMaxPassingSpeed() , resultSpace.maxPassingSpeed );

  resultSpace.safety = 0.0;
  return resultSpace;
}

void LocalizationStage::_findGapAndSpeed( const MTS_Vehicle *veh , float minLateralBorder , float maxLateralBorder , MTS_Region &region)
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
****Unused Function END****/


} // namespace traffic_manager
} // namespace carla
