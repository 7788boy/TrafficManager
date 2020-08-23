
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
  //const cg::Rotation vehicle_rotation = simulation_state.GetRotation(actor_id);
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

  updateLeader(index);

  

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

void LocalizationStage::DrawLeader(ActorId actor_id, LocalizationData &output) {
  cg::Location actor_location = simulation_state.GetLocation(actor_id);
  cg::Location first_location = simulation_state.GetLocation(output.leader.main_leader);
  cg::Location second_location = simulation_state.GetLocation(output.leader.potential_leader);
  debug_helper.DrawPoint(actor_location, 1.0f, {255u, 0u, 0u}, 0.5, true);
  debug_helper.DrawPoint(first_location, 1.0f, {0u, 255u, 0u}, 0.5, true);
  debug_helper.DrawPoint(second_location, 1.0f, {0u, 0u, 255u}, 0.5, true);
}

// Leader
void LocalizationStage::updateLeader(const unsigned long index)
{
  LocalizationData &output = output_array.at(index);

  //mSubject->mSituationData->mNeighbor->mPassedLeadingVehicle = NULL;
  //mSubject->mSituationData->mNeighbor->mExpectedLeadingVehicle = NULL;
  //mSubject->mBlockage = NULL;
  
  std::pair<ActorId, ActorId> leaders = collectLeadingVehicle(index); // first = main leader, second = potential leader.
  output.leader.main_leader = leaders.first;
  output.leader.potential_leader = leaders.second;
}

std::pair<ActorId, ActorId> LocalizationStage::collectLeadingVehicle(const unsigned long index)
{
  const ActorId actor_id = vehicle_id_list.at(index);
  
  const cg::Location actor_location = simulation_state.GetLocation(actor_id); // need to transfrom to local.
  const cg::Rotation actor_rotation = simulation_state.GetRotation(actor_id);
  std::vector<std::vector<float>> M;
  M.resize(4, std::vector<float>(4, 0.0f));
  M = getMatrix(actor_location, actor_rotation);
  std::vector<std::vector<float>> M_inv;
  M_inv.resize(4, std::vector<float>(4, 0.0f));
  M_inv = inverse(M);
  //float actor_local_location[4][1] = GlobalToLocal(M_inv, actor_location);

  float actor_offset = simulation_state.GetDimensions(actor_id).x;  //getOffset() + getLength() / 2.0f;
  SimpleWaypointPtr actor_waypoint = local_map->GetWaypoint(actor_location);
  crd::RoadId actor_road = actor_waypoint->GetWaypoint()->GetRoadId();
  crd::LaneId actor_lane = actor_waypoint->GetWaypoint()->GetLaneId();
  //bool isPassFind = false;
  //bool isExpFind = false;
  ActorId expectedPreVeh = 1000000;
  ActorId passedPreVeh = 1000000;

  // Find leading vehicles: one is able to be overtaked, one is expected to leading vehicle after overtaking
  // MTS_VehicleController *vehController =  subject->getCurrentController();
  // MTS_Lane *nextLane = subject->getNextLane();
  // MTS_Edge *nextEdge = mEdge;
  // if( nextLane ) nextEdge = nextLane->getEdge();

  // if the subject is the begin of this lane, it means that the leading vehicle may be the next lane 
  // which the subject will enter
  
  //const float LOOK_DIS = 1500.0f;
  float minOffset = actor_offset;
  float maxOffset = minOffset + MAX_OBSERVING_DISTANCE; //subject->getMaxObservingDistance();	
  float passOffset = FLT_MAX;
  float expectOffset = FLT_MAX;
  //float blockageOffset = FLT_MAX;

  // MTS_Vehicle *blockage = subject->getBlockage();
  // if( blockage && blockage->getCurrentController()->onEdge( mEdge )  ) 
  //   blockageOffset = blockage->getCurrentController()->getOffset() - 
  //                   blockage->getCurrentController()->getLength() / 2.0f;

  // //**************Section 1 - Find Leaders in coming queue *************//
  // /*if( ( minOffset < 0 || maxOffset < 0 ) && !subject->getTransitController()->idle() )
  //   collectLeadingVehicleInComingQueue( subject , subjectOffset , passedPreVeh , expectedPreVeh );*/
  // if( vehController->onEdge( mEdge ) && ( minOffset < 0 || maxOffset < 0 ) )
  //   collectLeadingVehicleInComingQueue( subject , subjectOffset , passedPreVeh , expectedPreVeh );
  // bool passFound = passedPreVeh != NULL;
  // bool expectFound = expectedPreVeh != NULL;

  // if( expectFound ) return;

  //**************Section 2 - Find Leaders on this edge *************//
  // First, collect vehicles in the block between minOffset and maxOffset.
  // std::vector< MTS_Vehicle* > vehInBlock;
  // getVehicleInBlock( minOffset , maxOffset , vehInBlock );
  // MTS_Lane *leftLane = getLeftLane();
  // MTS_Lane *rightLane = getRightLane();

  // if(leftLane) 
  //   leftLane->getVehicleInBlock( minOffset , maxOffset , vehInBlock );
  // if(rightLane) 
  //   rightLane->getVehicleInBlock( minOffset , maxOffset , vehInBlock );

  // Second, check if the vehicle is overlapped with the subject vehicle and the offset is closer to vehOffset
  //int vehSize = vehicle_id_list.size();

  //std::iterator::std::vector<ActorId> it = vehicle_id_list.begin();
  for(ActorId target_id: vehicle_id_list)
  {
    if( target_id == actor_id) //vehInBlock[i] == subject )
      continue;

    //MTS_VehicleController *controller = vehInBlock[i]->getCurrentController();
    //len / 2.0f;
    
     //vehInBlock[i] );}
    // float extendedGap = vehController->getExtendedGap( vehInBlock[i] );

    const cg::Location target_location = simulation_state.GetLocation(target_id);
    SimpleWaypointPtr target_waypoint = local_map->GetWaypoint(target_location);
    crd::RoadId target_road = target_waypoint->GetWaypoint()->GetRoadId();
    crd::LaneId target_lane = target_waypoint->GetWaypoint()->GetLaneId();

    if (actor_road == target_road && abs(actor_lane -  target_lane) <= 1 && actor_waypoint->Distance(target_waypoint) <= maxOffset)
    {
      float target_length = simulation_state.GetDimensions(target_id).x;// controller->getLength();
      std::vector<float> target_local_location(3, 0.0f);
      //target_local_location.resize(3, 0.0f);
      target_local_location= GlobalToLocal(M_inv, target_location);
      float target_offset = target_local_location[0] - target_length; 

      bool blocked = isOverlapped(actor_id, target_id, target_local_location[1]);

      if( target_offset < passOffset && target_offset > actor_offset && blocked ) // !passFound && // Situation 1: the is closer than current passed leading vehicle
      {
      
        //this part need to consider the none-lane case, need to consider the lateral information(remove the)

        expectedPreVeh = passedPreVeh;
        passedPreVeh = target_id; //vehInBlock[i];

        expectOffset = passOffset;
        passOffset = target_offset;
        //isPassFind = true;
      }
      else if(target_offset < expectOffset && target_offset > actor_offset && blocked ) //  !expectFound &&  // Situation 2: the is closer than current expected leading vehicle
      {
        expectedPreVeh = target_id; //vehInBlock[i];
        expectOffset = target_offset;
        //isExpFind = true;
      }
    }
    // if( blocked && vehInBlock[i]->broken()  && 
    //   candidateOffset < blockageOffset )
    // {
    //   subject->setBlockage( vehInBlock[i] );
    //   blockageOffset = candidateOffset;
    // }
  }

  //expectFound = expectedPreVeh != NULL;

  std::pair<ActorId, ActorId> leaders = std::make_pair(passedPreVeh, expectedPreVeh);

  return leaders;
  //**************Section 3 - Find Leaders on next edge *************//
  
  // MTS_Lane* tempNextLane = subject->findNextLane();
  // if( tempNextLane != NULL && tempNextLane != this && !expectFound)
  // {
  //   float temp_offset = subject->getOffset();
  //   float temp_lateralOffset = subject->getLateralOffset();
  //   tempNextLane->positionTranslate(this,temp_offset,temp_lateralOffset); 
  //   float subjectOffsetOnNextEdge = temp_offset+ vehController->getLength() / 2.0f;
  //   tempNextLane->collectLeadingVehicle( subject , subjectOffsetOnNextEdge , passedPreVeh , expectedPreVeh );
  // }

}
/*
// Situation
void LocalizationStage::updateRegion()
{
  for( int i=0;i<mRegion.size();++i )
  {
    mRegion[i].frontVehicles.clear();
    mRegion[i].rearVehicles.clear();
  }
  
  mRegion.clear();
  mSpaceOriented = false;
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

// Neighbor
void LocalizationStage::resetNeighbor()
{
  mSituationData->mNeighbor->reset();
  mSituationData->mNeighbor->mLeftVehicle = this;
  mSituationData->mNeighbor->mRightVehicle = this;
} 

const std::vector< MTS_Vehicle* >& LocalizationStage::getLeftRearVehicles( )
{
  if( mSituationData->mNeighbor->mLeftRearVehiclesUpdated )
    return mSituationData->mNeighbor->mLeftRearVehicles;
  mSituationData->mNeighbor->mLeftRearVehiclesUpdated = true;
  mSituationData->mNeighbor->mLeftRearVehicles.clear();
  getLeftRearVehicles( 2000.0f , mSituationData->mNeighbor->mLeftRearVehicles );
  return mSituationData->mNeighbor->mLeftRearVehicles;
}

const std::vector< MTS_Vehicle* >& LocalizationStage::getRightRearVehicles()
{
  if( mSituationData->mNeighbor->mRightRearVehiclesUpdated )
    return mSituationData->mNeighbor->mRightRearVehicles;
  mSituationData->mNeighbor->mRightRearVehiclesUpdated = true;
  mSituationData->mNeighbor->mRightRearVehicles.clear();
  getRightRearVehicles( 2000.0f , mSituationData->mNeighbor->mRightRearVehicles );
  return mSituationData->mNeighbor->mRightRearVehicles;
}

const std::vector< MTS_Vehicle* >& LocalizationStage::getLeftFrontVehicles()
{
  if( mSituationData->mNeighbor->mLeftFrontVehiclesUpdated )
    return mSituationData->mNeighbor->mLeftFrontVehicles;

  mSituationData->mNeighbor->mLeftFrontVehiclesUpdated = true;
  mSituationData->mNeighbor->mLeftFrontVehicles.clear();
  getLeftFrontVehicles( 2000.0f , mSituationData->mNeighbor->mLeftFrontVehicles );
  return mSituationData->mNeighbor->mLeftFrontVehicles;
}

const std::vector< MTS_Vehicle* >& LocalizationStage::getRightFrontVehicles()
{
  if( mSituationData->mNeighbor->mRightFrontVehiclesUpdated )
    return mSituationData->mNeighbor->mRightFrontVehicles;
  mSituationData->mNeighbor->mRightFrontVehiclesUpdated = true;
  mSituationData->mNeighbor->mRightFrontVehicles.clear();
  getRightFrontVehicles( 2000.0f , mSituationData->mNeighbor->mRightFrontVehicles );
  return mSituationData->mNeighbor->mRightFrontVehicles;
}

MTS_Vehicle* LocalizationStage::getLeftVehicle()
{
  if( mSituationData->mNeighbor->mLeftVehicle != this ) 
    return mSituationData->mNeighbor->mLeftVehicle;

  float minOffset = mOffset - mVehicleController->getLength() / 2.0f;
  float maxOffset = mOffset + mVehicleController->getLength() / 2.0f;
  MTS_Lane* lane = this->getLane();

  if( lane == NULL ) return NULL;

  MTS_Vehicle* leftVeh;

  while(1)
  {
    // find the vehicle on my left and closest to me 
    leftVeh = lane->getVehicle( this , minOffset , maxOffset , mLateralOffset , true );
    lane = lane->getLeftLane(); // continue to consider the left of current lane

    if( leftVeh != NULL || lane == NULL ) break;
  } 

  mSituationData->mNeighbor->mLeftVehicle = leftVeh;
  return mSituationData->mNeighbor->mLeftVehicle;
}

MTS_Vehicle* LocalizationStage::getRightVehicle()
{
  if( mSituationData->mNeighbor->mRightVehicle != this ) 
    return mSituationData->mNeighbor->mRightVehicle;
  float minOffset = mOffset - mVehicleController->getLength() / 2.0f;
  float maxOffset = mOffset + mVehicleController->getLength() / 2.0f;
  MTS_Lane* lane = this->getLane();

  if(lane==NULL) return NULL;

  MTS_Vehicle* rightVeh;
  
  while(1)
  {
    rightVeh = lane->getVehicle( this , minOffset , maxOffset , mLateralOffset , false );
    lane = lane->getRightLane(); // continue to consider the left of current lane

    if( rightVeh != NULL || lane == NULL ) break;
  }

  mSituationData->mNeighbor->mRightVehicle = rightVeh; 
  return mSituationData->mNeighbor->mRightVehicle;
}

void LocalizationStage::getLeftRearVehicles( float distance , std::vector< MTS_Vehicle* > &result )
{
  float halfVehLength = mCurrentController->getLength() / 2.0f;
  float halfVehWidth = mCurrentController->getWidth() / 2.0f;
  float vehOffset = mCurrentController->getOffset();
  MTS_Lane *lane = mCurrentController->getLane();
  MTS_Edge *edge = lane->getEdge();
  int numLanes = edge->getLaneSize();
  int indexRange = ceil( halfVehWidth / lane->getWidth() );
  int startLaneIdx = lane->getIndex();
  int targetLaneIdx = MIN( numLanes , startLaneIdx + indexRange ); 
  float maxOffset = vehOffset;
  float minOffset = maxOffset - distance;
  std::vector< MTS_Vehicle* > tmpResult;

  if( minOffset >= 0 || maxOffset >= 0 )
  {
    for( int i=startLaneIdx ; i<=targetLaneIdx ; ++i )
    {
      MTS_Lane *lane = edge->getLane( i );
      lane->getVehicleInBlock( minOffset , maxOffset , tmpResult );
    }
  }

  // check if the coming vehicle is safe for the subject 
  // if the observing boundary is backward the start of lane
  // add it to temporary result
  if( minOffset < 0 || maxOffset < 0 )
  {
    for( int i=startLaneIdx ; i<=targetLaneIdx ; ++i )
    {
      MTS_Lane *lane = edge->getLane( i );
      lane->getVehicleInComingQueue( minOffset , maxOffset , tmpResult );
    }
  }


  for( int i=0 ; i < tmpResult.size() ; ++i )
  {
    if( tmpResult[i] == this ) continue;
    MTS_VehicleController *controller = tmpResult[i]->getCurrentController();
    const MTS_Edge *last = controller->bindEdge( edge );
    if( !mCurrentController->cooperated( controller ) )
      continue;

    if( !mCurrentController->isOverlapped( tmpResult[i] ) && 
      tmpResult[i]->getLateralSeparation( this ) < 0 ) 
      result.push_back( tmpResult[i] );

    tmpResult[i]->getCurrentController()->bindEdge( last );
  }
    
}

void LocalizationStage::getRightRearVehicles( float distance , std::vector< MTS_Vehicle* > &result )
{
  float halfVehLength = mCurrentController->getLength() / 2.0f;
  float halfVehWidth = mCurrentController->getWidth() / 2.0f;
  float vehOffset = mCurrentController->getOffset();
  MTS_Lane *lane = mCurrentController->getLane();
  MTS_Edge *edge = lane->getEdge();
  int indexRange = ceil( halfVehWidth / lane->getWidth() );
  int startLaneIdx = lane->getIndex();
  int targetLaneIdx = MAX( 0 , startLaneIdx - indexRange ); 
  float maxOffset = vehOffset;
  float minOffset = maxOffset - distance;
  std::vector< MTS_Vehicle* > tmpResult;

  if( minOffset >= 0 || maxOffset >= 0 )
  {
    for( int i=startLaneIdx ; i>=targetLaneIdx ; --i )
    {
      MTS_Lane *lane = edge->getLane( i );
      lane->getVehicleInBlock( minOffset , maxOffset , tmpResult );
    }
  }

  // check if the coming vehicle is safe for the subject 
  // if the observing boundary is backward the start of lane
  // add it to temporary result
  if( minOffset < 0 || maxOffset < 0 )
  {
    for( int i=startLaneIdx ; i>=targetLaneIdx ; --i )
    {
      MTS_Lane *lane = edge->getLane( i );
      lane->getVehicleInComingQueue( minOffset , maxOffset , tmpResult );
    }
  }

  for( int i=0 ; i < tmpResult.size() ; ++i )
  {
    if( tmpResult[i] == this ) continue;
    MTS_VehicleController *controller = tmpResult[i]->getCurrentController();
    const MTS_Edge *last = controller->bindEdge( edge );
    if( !mCurrentController->cooperated( controller ) )
      continue;

    if( !mCurrentController->isOverlapped( tmpResult[i] ) && 
      tmpResult[i]->getLateralSeparation( this ) > 0 ) 
      result.push_back( tmpResult[i] );

    tmpResult[i]->getCurrentController()->bindEdge( last );
  }
    
}

void LocalizationStage::getLeftFrontVehicles( float distance , std::vector< MTS_Vehicle* > &result )
{
  float halfVehLength = mCurrentController->getLength() / 2.0f;
  float halfVehWidth = mCurrentController->getWidth() / 2.0f;
  float vehOffset = mCurrentController->getOffset();
  MTS_Lane *lane = mCurrentController->getLane();
  MTS_Edge *edge = lane->getEdge();
  float laneLength = lane->getLength();
  
  if( mTransitController->ready() ) laneLength = mTransitController->getEndOffset();

  int numLanes = edge->getLaneSize();
  int indexRange = ceil( halfVehWidth / lane->getWidth() );
  int startLaneIdx = lane->getIndex();
  int targetLaneIdx = MIN( numLanes , startLaneIdx + indexRange ); 
  float minOffset = vehOffset;
  float maxOffset = minOffset + distance;
  std::vector< MTS_Vehicle* > tmpResult;

  if( minOffset >=0 || maxOffset >= 0 )
  {
    for( int i=startLaneIdx ; i<=targetLaneIdx ; ++i )
    {
      MTS_Lane *lane = edge->getLane( i );
      lane->getVehicleInBlock( minOffset , maxOffset , tmpResult );
    }
  }

  if( minOffset < 0 || maxOffset < 0 )
  {
    for( int i=startLaneIdx ; i<=targetLaneIdx ; ++i )
    {
      MTS_Lane *lane = edge->getLane( i );
      lane->getVehicleInComingQueue( minOffset , maxOffset , tmpResult );
    }
  }
  // check if the coming vehicle is safe for the subject 
  // if the observing boundary is backward the start of lane
  // add it to temporary result

  MTS_Edge *nextEdge = NULL;
  MTS_Lane *nextLane = getNextLane();
  if( nextLane ) nextEdge = nextLane->getEdge();
  if( nextLane != lane && nextEdge && ( minOffset > laneLength || maxOffset > laneLength ) ) 
  {
    minOffset -= laneLength;
    maxOffset -= laneLength;
    for( int i=startLaneIdx ; i<=targetLaneIdx ; ++i )
    {
      MTS_Lane *lane = nextEdge->getLane( i );
      lane->getVehicleInBlock( minOffset , maxOffset , tmpResult );
    }
  }


  for( int i=0 ; i < tmpResult.size() ; ++i )
  {
    if( tmpResult[i] == this ) continue;
    MTS_VehicleController *controller = tmpResult[i]->getCurrentController();
    const MTS_Edge *last = controller->bindEdge( edge );
    if( !mCurrentController->cooperated( controller ) )
      continue;
    
    // �`�N: The other vehicles could be on the left-front and on the left-rear at the same time. 
    //           Bacause the restriction  for rear vehicles is looser, so one vehicle will be regarded as the front vehicle 
    //           if it only belong to the front vehicles.
    bool totallyFront = controller->getOffset() - controller->getLength() / 2.0f > minOffset;
    if( totallyFront == false )
      int a=0;
    if( !mCurrentController->isOverlapped( tmpResult[i] ) &&
      tmpResult[i]->getLateralSeparation( this ) < 0 ) 
      result.push_back( tmpResult[i] );

    tmpResult[i]->getCurrentController()->bindEdge( last );
  }
    
}

void LocalizationStage::getRightFrontVehicles( float distance , std::vector< MTS_Vehicle* > &result )
{
  float halfVehLength = mCurrentController->getLength() / 2.0f;
  float halfVehWidth = mCurrentController->getWidth() / 2.0f;
  float vehOffset = mCurrentController->getOffset();
  MTS_Lane *lane = mCurrentController->getLane();
  MTS_Edge *edge = lane->getEdge();
  float laneLength = lane->getLength();
  
  if( mTransitController->ready() ) laneLength = mTransitController->getEndOffset();

  int indexRange = ceil( halfVehWidth / lane->getWidth() );
  int startLaneIdx = lane->getIndex();
  int targetLaneIdx = MAX( 0 , startLaneIdx - indexRange ); 
  float minOffset = vehOffset;
  float maxOffset = minOffset + distance;
  std::vector< MTS_Vehicle* > tmpResult;

  if( minOffset >=0 || maxOffset >= 0 )
  {
    for( int i=startLaneIdx ; i>=targetLaneIdx ; --i )
    {
      MTS_Lane *lane = edge->getLane( i );
      lane->getVehicleInBlock( minOffset , maxOffset , tmpResult );
    }
  }

  if( minOffset < 0 || maxOffset < 0 )
  {
    for( int i=startLaneIdx ; i>=targetLaneIdx ; --i )
    {
      MTS_Lane *lane = edge->getLane( i );
      lane->getVehicleInComingQueue( minOffset , maxOffset , tmpResult );
    }
  }

  // check if the coming vehicle is safe for the subject 
  // if the observing boundary is backward the start of lane
  // add it to temporary result

  MTS_Edge *nextEdge = NULL;
  MTS_Lane *nextLane = getNextLane();
  if( nextLane ) nextEdge = nextLane->getEdge();
  if( nextLane != lane && nextEdge && ( minOffset > laneLength || maxOffset > laneLength ) ) 
  {
    minOffset -= laneLength;
    maxOffset -= laneLength;
    for( int i=startLaneIdx ; i>=targetLaneIdx ; --i )
    {
      MTS_Lane *lane = nextEdge->getLane( i );
      lane->getVehicleInBlock( minOffset , maxOffset , tmpResult );
    }
  }


  for( int i=0 ; i < tmpResult.size() ; ++i )
  {
    if( tmpResult[i] == this ) continue;
    MTS_VehicleController *controller = tmpResult[i]->getCurrentController();
    const MTS_Edge *last = controller->bindEdge( edge );
    if( !mCurrentController->cooperated( controller ) )
      continue;

    //  �`�N: The other vehicles could be on the right-front and on the right-rear at the same time. 
    //           Bacause the restriction  for rear vehicles is looser, so one vehicle will be regarded as the front vehicle 
    //           if it only belong to the front vehicles. 
    bool totallyFront = controller->getOffset() - controller->getLength() / 2.0f > minOffset;
    if( totallyFront == false )
      int a=0;

    if( !mCurrentController->isOverlapped( tmpResult[i] )  &&
      tmpResult[i]->getLateralSeparation( this ) > 0 ) 
      result.push_back( tmpResult[i] );

    tmpResult[i]->getCurrentController()->bindEdge( last );
  }
    
}

*/
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
/*
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
*/

// transform
std::vector<std::vector<float>> LocalizationStage::getMatrix(cg::Location actor_location, cg::Rotation actor_rotation) //local transfer to global
{
  const float pi = acosf(-1);
  
  float c_y = cosf(actor_rotation.yaw * pi / 180.0f);
  float s_y = sinf(actor_rotation.yaw * pi / 180.0f);
  float c_r = cosf(actor_rotation.roll * pi / 180.0f);
  float s_r = sinf(actor_rotation.roll * pi / 180.0f);
  float c_p = cosf(actor_rotation.pitch * pi / 180.0f);
  float s_p = sinf(actor_rotation.pitch * pi / 180.0f);

  std::vector<std::vector<float>> M;
  M.resize(4, std::vector<float>(4, 0.0f));
  //std::vector<std::vector<float>>  M[4][4]; //matrix =  //np.array(np.identity(4))
  M[3][0] = 0.0f;
  M[3][1] = 0.0f;
  M[3][2] = 0.0f;
  M[3][3] = 1.0f;
  M[0][3] = actor_location.x;
  M[1][3] = actor_location.y;
  M[2][3] = actor_location.z;
  M[0][0] = c_p * c_y;
  M[0][1] = c_y * s_p * s_r - s_y * c_r;
  M[0][2] = -c_y * s_p * c_r - s_y * s_r;
  M[1][0] = s_y * c_p;
  M[1][1] = s_y * s_p * s_r + c_y * c_r;
  M[1][2] = -s_y * s_p * c_r + c_y * s_r;
  M[2][0] = s_p;
  M[2][1] = -c_p * s_r;
  M[2][2] = c_p * c_r;

  return M;
}

std::vector<float> LocalizationStage::GlobalToLocal(std::vector<std::vector<float>> M, cg::Location global_location)
{
  //float inv_M[4][4];
  std::vector<float> local_location(3, 0.0f);
  //float local_velocity[4][1];

  // if(inverse(M, inv_M))
  // {
  //cg::Location actor_location = simulation_state.GetLocation(target_id);
    //Vector3D actor_velocity = simulation_state.GectVelocity(actor_id);
  std::vector<float> location(4, 0.0f);
  location[0] = global_location.x;
  location[1] = global_location.y;
  location[2] = global_location.z;
  location[3] = 1.0f;

  // float velocity[4][1];
  // velocity[0][0] = actor_velocity.x;
  // velocity[1][0] = actor_velocity.y;
  // velocity[2][0] = actor_velocity.z;
  // velocity[3][0] = 0.0f;

  matrixMultiply(M, location, local_location); // vector call by ref?
  //matrixMultiply(inv_M, velocity, local_velocity);
  
  return local_location; //, velocity)
  //}
}

void LocalizationStage::matrixMultiply(std::vector<std::vector<float>> M, std::vector<float>  V, std::vector<float> result){
  for(unsigned long i=0; i < V.size() - 1; i++ ){
    result[i]= M[i][0] * V[0] + M[i][1] * V[1] + M[i][2] * V[2] + M[i][3] * V[3];
  }
}

void LocalizationStage::getCofactor(std::vector<std::vector<float>> A, std::vector<std::vector<float>> temp, unsigned p, unsigned q, unsigned n) 
{ 
    unsigned i = 0, j = 0; 
  
    // Looping for each element of the matrix 
    for (unsigned row = 0; row < n; row++) 
    { 
        for (unsigned col = 0; col < n; col++) 
        { 
            //  Copying into temporary matrix only those element 
            //  which are not in given row and column 
            if (row != p && col != q) 
            { 
                temp[i][j++] = A[row][col]; 
  
                // Row is filled, so increase row index and 
                // reset col index 
                if (j == n - 1) 
                { 
                    j = 0; 
                    i++; 
                } 
            } 
        } 
    } 
} 
  
/* Recursive function for finding determinant of matrix. 
   n is current dimension of A[][]. */
float LocalizationStage::determinant(std::vector<std::vector<float>> A, unsigned n) 
{ 
    float D = 0.0f; // Initialize result 
  
    //  Base case : if matrix contains single element 
    if (n == 1) 
        return A[0][0]; 
  
    std::vector<std::vector<float>> temp; // To store cofactors 
    temp.resize(4, std::vector<float>(4, 0.0f));
    float sign = 1.0f;  // To store sign multiplier 
  
     // Iterate for each element of first row 
    for (unsigned f = 0; f < n; f++) 
    { 
        // Getting Cofactor of A[0][f] 
        getCofactor(A, temp, 0, f, n); 
        D += sign * A[0][f] * determinant(temp, n - 1); 
  
        // terms are to be added with alternate sign 
        sign = -sign; 
    } 
  
    return D; 
} 
  
// Function to get adjoint of A[N][N] in adj[N][N]. 
void LocalizationStage::adjoint(std::vector<std::vector<float>> A, std::vector<std::vector<float>> adj) 
{ 
  // temp is used to store cofactors of A[][] 
  bool sign = false;
  std::vector<std::vector<float>> temp;
  temp.resize(4, std::vector<float>(4, 0.0f));

  for (unsigned i=0; i<4; i++) 
  { 
      for (unsigned j=0; j<4; j++) 
      { 
          // Get cofactor of A[i][j] 
          getCofactor(A, temp, i, j, 4); 

          // sign of adj[j][i] positive if sum of row 
          // and column indexes is even. 
          sign = ((i+j)%2==0)? true: false; 

          // Interchanging rows and columns to get the 
          // transpose of the cofactor matrix 
          adj[j][i] = sign?(determinant(temp, 3)):-(determinant(temp, 3)); 
      } 
  } 
} 
  
// Function to calculate and store inverse, returns false if 
// matrix is singular 
std::vector<std::vector<float>> LocalizationStage::inverse(std::vector<std::vector<float>> A) 
{ 
  //float inverse[4][4];
  std::vector<std::vector<float>> inverse;
  inverse.resize(4, std::vector<float>(4, 0.0f));
  // Find determinant of A[][] 
  float det = determinant(A, 4); 
  if (det <= 0.0001f) 
  { 
      //cout << "Singular matrix, can't find its inverse"; 
      return inverse; 
  } 

  // Find adjoint 
  std::vector<std::vector<float>> adj;
  adj.resize(4, std::vector<float>(4, 0.0f));
  
  adjoint(A, adj); 

  // Find Inverse using formula "inverse(A) = adj(A)/det(A)" 
  for (unsigned i=0; i<4; i++) 
      for (unsigned j=0; j<4; j++) 
          inverse[i][j] = adj[i][j]/float(det); 

  return inverse; 
} 


} // namespace traffic_manager
} // namespace carla
