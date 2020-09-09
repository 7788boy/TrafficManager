
#include "carla/trafficmanager/Constants.h"
#include "carla/trafficmanager/PIDController.h"

#include "carla/trafficmanager/MotionPlanStage.h"

namespace carla {
namespace traffic_manager {

using namespace constants::MotionPlan;
using namespace constants::WaypointSelection;
using namespace constants::SpeedThreshold;
using namespace constants::MTSCar;

using constants::HybridMode::HYBRID_MODE_DT;

MotionPlanStage::MotionPlanStage(
  const std::vector<ActorId> &vehicle_id_list,
  const SimulationState &simulation_state,
  const Parameters &parameters,
  const BufferMap &buffer_map,
  const TrackTraffic &track_traffic,
  const std::vector<float> &urban_longitudinal_parameters,
  const std::vector<float> &highway_longitudinal_parameters,
  const std::vector<float> &urban_lateral_parameters,
  const std::vector<float> &highway_lateral_parameters,
  const LocalizationFrame &localization_frame,
  const CollisionFrame&collision_frame,
  const TLFrame &tl_frame,
  ControlFrame &output_array)
  : vehicle_id_list(vehicle_id_list),
    simulation_state(simulation_state),
    parameters(parameters),
    buffer_map(buffer_map),
    track_traffic(track_traffic),
    urban_longitudinal_parameters(urban_longitudinal_parameters),
    highway_longitudinal_parameters(highway_longitudinal_parameters),
    urban_lateral_parameters(urban_lateral_parameters),
    highway_lateral_parameters(highway_lateral_parameters),
    localization_frame(localization_frame),
    collision_frame(collision_frame),
    tl_frame(tl_frame),
    output_array(output_array) {}

void MotionPlanStage::Update(const unsigned long index) {
  const ActorId actor_id = vehicle_id_list.at(index);
  const cg::Location ego_location = simulation_state.GetLocation(actor_id);
  const cg::Vector3D ego_velocity = simulation_state.GetVelocity(actor_id);
  const float ego_speed = ego_velocity.Length();
  const cg::Vector3D ego_heading = simulation_state.GetHeading(actor_id);
  const bool ego_physics_enabled = simulation_state.IsPhysicsEnabled(actor_id);
  const Buffer &waypoint_buffer = buffer_map.at(actor_id);
  const LocalizationData &localization = localization_frame.at(index);
  const CollisionHazardData &collision_hazard = collision_frame.at(index); //
  const bool &tl_hazard = tl_frame.at(index);

  const float target_point_distance = std::max(ego_speed * TARGET_WAYPOINT_TIME_HORIZON,
                                               TARGET_WAYPOINT_HORIZON_LENGTH);
  const SimpleWaypointPtr &target_waypoint = GetTargetWaypoint(waypoint_buffer, target_point_distance).first;
  const cg::Location target_location = target_waypoint->GetLocation();
  float dot_product = DeviationDotProduct(ego_location, ego_heading, target_location);
  float cross_product = DeviationCrossProduct(ego_location, ego_heading, target_location);
  dot_product = 1.0f - dot_product;
  if (cross_product < 0.0f) {
    dot_product *= -1.0f;
  }
  const float current_deviation = dot_product;

  // If previous state for vehicle not found, initialize state entry.
  if (pid_state_map.find(actor_id) == pid_state_map.end()) {
    const auto initial_state = StateEntry{chr::system_clock::now(), 0.0f, 0.0f, 0.0f, 0.0f};
    pid_state_map.insert({actor_id, initial_state});
  }

  // Retrieving the previous state.
  traffic_manager::StateEntry previous_state;
  previous_state = pid_state_map.at(actor_id);

  // Select PID parameters.
  std::vector<float> longitudinal_parameters;
  std::vector<float> lateral_parameters;
  if (ego_speed > HIGHWAY_SPEED) {
    longitudinal_parameters = highway_longitudinal_parameters;
    lateral_parameters = highway_lateral_parameters;
  } else {
    longitudinal_parameters = urban_longitudinal_parameters;
    lateral_parameters = urban_lateral_parameters;
  }

  // Target velocity for vehicle.
  const float ego_speed_limit = simulation_state.GetSpeedLimit(actor_id); //
  float max_target_velocity = parameters.GetVehicleTargetVelocity(actor_id, ego_speed_limit) / 3.6f; // km/hr -> m/s

  // Collision handling and target velocity correction.
  std::pair<bool, float> collision_response = CollisionHandling(collision_hazard, tl_hazard, ego_velocity, ego_heading, max_target_velocity); //
  bool collision_emergency_stop = collision_response.first; //false;
  float dynamic_target_velocity = collision_response.second; //ego_speed + 0.02f * GetLongitudinalAcc(localization, actor_id);

  // Don't enter junction if there isn't enough free space after the junction.
  bool safe_after_junction = SafeAfterJunction(localization, tl_hazard, collision_emergency_stop);

  // In case of collision or traffic light hazard.
  bool emergency_stop = tl_hazard || collision_emergency_stop || !safe_after_junction;

  ActuationSignal actuation_signal{0.0f, 0.0f, 0.0f};
  cg::Transform teleportation_transform;

  // If physics is enabled for the vehicle, use PID controller.
  const auto current_time = chr::system_clock::now();
  StateEntry current_state;
  if (ego_physics_enabled) {

    // State update for vehicle.
    current_state = PID::StateUpdate(previous_state, ego_speed, dynamic_target_velocity,
                                     current_deviation, current_time);

    // Controller actuation.
    actuation_signal = PID::RunStep(current_state, previous_state,
                                    longitudinal_parameters, lateral_parameters);

    if (emergency_stop) {

      current_state.deviation_integral = 0.0f;
      current_state.velocity_integral = 0.0f;
      actuation_signal.throttle = 0.0f;
      actuation_signal.brake = 1.0f;
    }
  }
  // For physics-less vehicles, determine position and orientation for teleportation.
  else {
    // Flushing controller state for vehicle.
    current_state = {chr::system_clock::now(),
                     0.0f, 0.0f,
                     0.0f, 0.0f};

    // Add entry to teleportation duration clock table if not present.
    if (teleportation_instance.find(actor_id) == teleportation_instance.end()) {
      teleportation_instance.insert({actor_id, chr::system_clock::now()});
    }

    // Measuring time elapsed since last teleportation for the vehicle.
    chr::duration<float> elapsed_time = current_time - teleportation_instance.at(actor_id);

    // Find a location ahead of the vehicle for teleportation to achieve intended velocity.
    if (!emergency_stop && (parameters.GetSynchronousMode() || elapsed_time.count() > HYBRID_MODE_DT)) {

      // Target displacement magnitude to achieve target velocity.
      const float target_displacement = dynamic_target_velocity * HYBRID_MODE_DT;
      const SimpleWaypointPtr teleport_target_waypoint = GetTargetWaypoint(waypoint_buffer, target_displacement).first;

      // Construct target transform to accurately achieve desired velocity.
      float missing_displacement = 0.0f;
      const float base_displacement = teleport_target_waypoint->Distance(ego_location);
      if (base_displacement < target_displacement) {
        missing_displacement = target_displacement - base_displacement;
      }
      cg::Transform target_base_transform = teleport_target_waypoint->GetTransform();
      cg::Location target_base_location = target_base_transform.location;
      cg::Vector3D target_heading = target_base_transform.GetForwardVector();
      cg::Location teleportation_location = target_base_location + cg::Location(target_heading * missing_displacement);
      teleportation_transform = cg::Transform(teleportation_location, target_base_transform.rotation);
    }
    // In case of an emergency stop, stay in the same location.
    // Also, teleport only once every dt in asynchronous mode.
    else {
      teleportation_transform = cg::Transform(ego_location, simulation_state.GetRotation(actor_id));
    }
  }

  // Updating PID state.
  StateEntry &state = pid_state_map.at(actor_id);
  state = current_state;

  // Constructing the actuation signal.
  if (ego_physics_enabled) {
    carla::rpc::VehicleControl vehicle_control;
    vehicle_control.throttle = actuation_signal.throttle;
    vehicle_control.brake = actuation_signal.brake;
    vehicle_control.steer = actuation_signal.steer;

    output_array.at(index) = carla::rpc::Command::ApplyVehicleControl(actor_id, vehicle_control);
  } else {
    output_array.at(index) = carla::rpc::Command::ApplyTransform(actor_id, teleportation_transform);
  }
}

bool MotionPlanStage::SafeAfterJunction(const LocalizationData &localization,
                                        const bool tl_hazard,
                                        const bool collision_emergency_stop) {

  SimpleWaypointPtr junction_end_point = localization.junction_end_point;
  SimpleWaypointPtr safe_point = localization.safe_point;

  bool safe_after_junction = true;

  if (!tl_hazard && !collision_emergency_stop
      && localization.is_at_junction_entrance
      && junction_end_point != nullptr && safe_point != nullptr
      && junction_end_point->DistanceSquared(safe_point) > SQUARE(MIN_SAFE_INTERVAL_LENGTH)) {

    ActorIdSet initial_set = track_traffic.GetPassingVehicles(junction_end_point->GetId());
    float safe_interval_length_squared = junction_end_point->DistanceSquared(safe_point);
    cg::Location mid_point = (junction_end_point->GetLocation() + safe_point->GetLocation())/2.0f;

    // Scan through the safe interval and find if any vehicles are present in it
    // by finding their occupied waypoints.
    for (SimpleWaypointPtr current_waypoint = junction_end_point;
         current_waypoint->DistanceSquared(junction_end_point) < safe_interval_length_squared && safe_after_junction;
         current_waypoint = current_waypoint->GetNextWaypoint().front()) {

      ActorIdSet current_set = track_traffic.GetPassingVehicles(current_waypoint->GetId());
      ActorIdSet difference;
      std::set_difference(current_set.begin(), current_set.end(),
                          initial_set.begin(), initial_set.end(),
                          std::inserter(difference, difference.begin()));
      if (difference.size() > 0) {
        for (const ActorId &blocking_id: difference) {
          cg::Location blocking_actor_location = simulation_state.GetLocation(blocking_id);
          if (cg::Math::DistanceSquared(blocking_actor_location, mid_point) < SQUARE(MAX_JUNCTION_BLOCK_DISTANCE)
              && simulation_state.GetVelocity(blocking_id).SquaredLength() < SQUARE(AFTER_JUNCTION_MIN_SPEED)) {
            safe_after_junction = false;
          }
        }
      }
    }
  }

  return safe_after_junction;
}

std::pair<bool, float> MotionPlanStage::CollisionHandling(const CollisionHazardData &collision_hazard,
                                                          const bool tl_hazard,
                                                          const cg::Vector3D ego_velocity,
                                                          const cg::Vector3D ego_heading,
                                                          const float max_target_velocity) {
  bool collision_emergency_stop = false;
  float dynamic_target_velocity = max_target_velocity;
  
  if (collision_hazard.hazard && !tl_hazard) {
    const ActorId other_actor_id = collision_hazard.hazard_actor_id;
    const cg::Vector3D other_velocity = simulation_state.GetVelocity(other_actor_id);
    const float ego_relative_speed = (ego_velocity - other_velocity).Length();
    const float available_distance_margin = collision_hazard.available_distance_margin;
    const float other_speed_along_heading = cg::Math::Dot(other_velocity, ego_heading);
    
    // Consider collision avoidance decisions only if there is positive relative velocity
    // of the ego vehicle (meaning, ego vehicle is closing the gap to the lead vehicle).
    if (ego_relative_speed > EPSILON_RELATIVE_SPEED) {
      // If other vehicle is approaching lead vehicle and lead vehicle is further
      // than follow_lead_distance 0 kmph -> 5m, 100 kmph -> 10m.
      float follow_lead_distance = ego_relative_speed * FOLLOW_DISTANCE_RATE + MIN_FOLLOW_LEAD_DISTANCE;
      if (available_distance_margin > follow_lead_distance) {
        // Then reduce the gap between the vehicles till FOLLOW_LEAD_DISTANCE
        // by maintaining a relative speed of RELATIVE_APPROACH_SPEED
        dynamic_target_velocity = other_speed_along_heading + RELATIVE_APPROACH_SPEED;
      }
      // If vehicle is approaching a lead vehicle and the lead vehicle is further
      // than CRITICAL_BRAKING_MARGIN but closer than FOLLOW_LEAD_DISTANCE.
      else if (available_distance_margin > CRITICAL_BRAKING_MARGIN) {
        // Then follow the lead vehicle by acquiring it's speed along current heading.
        dynamic_target_velocity = std::max(other_speed_along_heading, RELATIVE_APPROACH_SPEED);
      } else {
        // If lead vehicle closer than CRITICAL_BRAKING_MARGIN, initiate emergency stop.
        collision_emergency_stop = true;
      }
    }
    if (available_distance_margin < CRITICAL_BRAKING_MARGIN) {
      collision_emergency_stop = true;
    }
  }

  dynamic_target_velocity = std::min(max_target_velocity, dynamic_target_velocity);

  return {collision_emergency_stop, dynamic_target_velocity}; 
}

void MotionPlanStage::RemoveActor(const ActorId actor_id) {
  pid_state_map.erase(actor_id);
  teleportation_instance.erase(actor_id);
}

void MotionPlanStage::Reset() {
  pid_state_map.clear();
  teleportation_instance.clear();
}

// /************************
// *******MTS SECTION*******
// ************************/

// Longitudinal
float MotionPlanStage::GetLongitudinalAcc(const LocalizationData &localization, ActorId actor_id) 
{
  float acc_free = GetFreeAcc(actor_id);
  float brake_first = 0.0f;
  float brake_second = 0.0f;
  
  if(localization.leader.MainLeader)
    brake_first = GetAcc(actor_id, localization.leader.MainLeader.get());
  if(localization.leader.PotentialLeader)
    brake_second = GetAcc(actor_id, localization.leader.PotentialLeader.get());
  
  float max_deceleration = std::max(brake_first, brake_second);

  // if(traffic_light_hazard)
  // {
  // 	float brake_stopLine = GetDecForStopLine(actor_id);
  //   max_deceleration = std::max(max_deceleration, brake_stopLine)
  // }
  
  float acc = acc_free - max_deceleration;

  return acc;

}

float MotionPlanStage::GetFreeAcc(ActorId actor_id)
{
  const float ego_speed_limit = simulation_state.GetSpeedLimit(actor_id);
  const float v_current = simulation_state.GetVelocity(actor_id).Length();
  float v_desired = parameters.GetVehicleTargetVelocity(actor_id, ego_speed_limit) / 3.6f;
  
  if( v_desired == 0.0f ) return 0.0f; 

  float acc = MAX_ACC * (1.0f - pow( v_current / v_desired, FREE_ACCELERATION_EXP));
  return acc;
}

float MotionPlanStage::GetAcc(ActorId actor_id, ActorId target_id)
{
  if(!target_id) 
    return 0.0f;

  float gap = GetGap(actor_id, target_id);
  bool gapExtended = false;
  
  if( gap < 0.0f) // current leader but the gap less than 0
  {
  	// try to extend the gap according to the yaw angle of the leader
  	gapExtended = true;
  	gap += GetExtendedGap(actor_id, target_id);
  }

  if( gap < 0.0f ) // the gap is less than 0 (even after extension)
  	return 0.0f;

  const float actor_velocity = simulation_state.GetVelocity(actor_id).Length();
  const float target_velocity = simulation_state.GetVelocity(target_id).Length();
  float s = ComputeDesiredGap(actor_velocity, target_velocity, target_id);
  
  bool approaching = (actor_velocity - target_velocity) >= 0.0f;
  bool closeEnough = gap <= 1.2f * s;
  float acc = MAX_ACC * std::pow(s / gap, 2.0f) * (approaching || closeEnough);

  if(!approaching && closeEnough)
    acc = MAX_ACC * (s / gap);
  
  if(gapExtended == false && acc > MAX_ACC)
  {
  	float extendedGap = GetExtendedGap(actor_id, target_id);
  	if(extendedGap != 0)
  	{
  		float gapRatio = gap / (gap + extendedGap);
  		acc *= gapRatio * gapRatio;
  	}
  }

  return acc;
}

float MotionPlanStage::ComputeDesiredGap(float actor_velocity, float target_velocity, ActorId target_id)
{
  if(!target_id)
  	return MIN_GAP + actor_velocity * RESPONSE_TIME + actor_velocity * actor_velocity / (2 * sqrt(MAX_ACC * COMFORTABLE_DEC));

  float speedCon = (actor_velocity * RESPONSE_TIME + actor_velocity * (actor_velocity - target_velocity) / (2 * sqrt(MAX_ACC * COMFORTABLE_DEC)));
  float desired_gap = (MIN_GAP + speedCon);
  
  if(desired_gap < 0.0f)
    desired_gap = MIN_GAP;

  return desired_gap;
}

/*
float MotionPlanStage::GetDecForStopLine(ActorId actor_id, float stopLine)
{
	// compute gap to stopline and evaluate the deceleration
	float currentSpeed = simulation_state.GetVelocity(actor_id).Length();
	float gap = getGapToStopLine(stopLine);

	if( gap <= 1.0 )
    gap = 1.0;

	float s = MIN_GAP + currentSpeed * RESPONSE_TIME + currentSpeed * currentSpeed / (2*sqrt(COMFORTABLE_DEC * MAX_ACC));

	return COMFORTABLE_DEC * (s * s) / (gap * gap);
}*/
/*
// Lateral velocity.
float MotionPlanStage::GetSpeedChange(ActorId* actor_id)
{
  float lateral_offset = GetDesiredLateralOffset(); //??? 
  float lateral_veocity = simulation_state.GetVelocity(actor_id).y; //local
  float lateral_time = 2.0f * lateral_offset / lateral_veocity;

  if(lateral_time <= 0.0 || lateral_time >= MAX_MOVEMENT_TIME)
    lateral_time = GetLongitudinalTime(actor_id);

  return GetLateralSpeed(actor_id, lateral_time);
}

float MotionPlanStage::GetLongitudinalTime(ActorId* actor_id) const
{
  ActorId *pred = localization.leader.main_leader.get();
  float longitudinal_velocity = simulation_state.GetVelocity(actor_id).Length(); 
  float longitudinal_time = GetGapToStopLine() / longitudinal_velocity;

  if(pred)
  {
    float s = GetGap(actor_id, pred);
    longitudinal_time = std::min(s / longitudinal_velocity, longitudinal_time);
  }

  if( longitudinal_time > 0 && longitudinal_time < MAX_MOVEMENT_TIME )
    return longitudinal_time;

  return MAX_MOVEMENT_TIME;
}

float MotionPlanStage::GetLateralSpeed(ActorId* actor_id, float move_time) const
{
  float lateral_offset = GetDesiredLateralOffset(); //???
  float current_location = simulation_state.GetLocation(actor_id).y; //local
  float lateral_velocity = simulation_state.GetVelocity(actor_id).y; //local
  float acc = 2 * ((lateral_offset - current_location) / (move_time * move_time) - lateral_velocity / move_time);

  if( acc < 0 )
    acc = std::max(-MAX_ACC, acc);
  else
    acc = std::min(MAX_ACC, acc);

  return acc;
}

// Lateral desired offset.
float MotionPlanStage::ComputeBestLateralOffset(ActorId actor_id, LocalizationData &localization)
{
  ActorId *left = localization.surrounding.LeftVehicle; //veh->getLeftVehicle();
  ActorId *right = localization.surrounding.RightVehicle; //veh->getRightVehicle();
  //MTS_Edge *edge = veh->getLane()->getEdge();

  //int typeCode = veh->getVehicleType()->getTypeCode();
  float actor_half_width = simulation_state.GetDimensions(actor_id).y; // veh->getVehicleType()->getStaticWidth() / 2.0f;
  float actor_lateral_offset = simulation_state.GetLocation(actor_id).y; //veh->getLateralOffset();
  //float vehOffset = simulation_state.GetLocation(actor_id).x + simulation_state.GetDimensions(actor_id).x;
  float actor_velocity = simulation_state.GetVelocity(actor_id).x //veh->getCurrentSpeed();
  //MTS_Region &validSpace = localization.situation.mCurrentRegion; //param->currentRegion;
  int valid_space_ID = -1;
  //int best_space_ID = -1;
  float max_cost = 0.0f;
  float max_valid_cost = 0.0f;
  //int spaceSize = localization.situation.mRegion.size() ;//param->allRegion.size();

  for( int i = 0; i < localization.situation.mRegion.size(); ++i)
  {
    if( localization.situation.mRegion[i]->width < 1.8f * actor_half_width )
      continue;
    //float laneCenter = localization.situation.mRegion[i]->offset;

    float offset_diff = localization.situation.mRegion[i]->offset - actor_lateral_offset;
    float safe_offset;

    bool safeSpace = true;
    if( localization.situation.mRegion[i]->width < actor_half_width * 2)
    {
      safeSpace = false;
      localization.situation.mRegion[i]->safety = 0.0f;
    }
    else if( offset_diff < 0 ) // check if the front and the rear of subject vehicle at the desired offset are safe
    {
      bool safeLeftSpace = CheckLeftSafety(actor_id, localization.situation.mRegion[i]->offset , &safe_offset , localization.situation.mRegion[i]);
      bool adjustOffset = localization.situation.mSpaceOriented  && safe_offset < actor_lateral_offset;

      if( !safeLeftSpace && !adjustOffset ) 
        safeSpace = false;

      else if( !safeLeftSpace )
        offset_diff = safe_offset - actor_lateral_offset;
    }
    else if( offset_diff > 0 ) // check if the front and the rear of subject vehicle at the desired offset are safe
    {
      bool safeRightSpace = CheckRightSafety(actor_id, localization.situation.mRegion[i]->offset , &safe_offset , localization.situation.mRegion[i]);
      bool adjustOffset = localization.situation.mSpaceOriented  && safe_offset > actor_lateral_offset;

      if( !safeRightSpace && !adjustOffset )
        safeSpace = false;

      else if( !safeRightSpace )
        offset_diff = safe_offset - actor_lateral_offset;
    }

    
    offset_diff = std::abs(offset_diff);
    float gap = localization.situation.mRegion[i]->gap;
    bool velCosistent = (offset_diff * simulation_state.GetVelocity(actor_id).y) > 0; //local	
    float speed_diff = localization.situation.mRegion[i]->maxPassingSpeed - actor_velocity;
    float safety = localization.situation.mRegion[i]->safety;

    float w_speed	= param->mRegionSelectionWeight->weight_speed;
    float w_gap		= param->mRegionSelectionWeight->weight_gap;
    float w_dis		= param->mRegionSelectionWeight->weight_lateralDistance;
    float w_vel		= param->mRegionSelectionWeight->weight_velocityConsistency;
    float w_safe	= param->mRegionSelectionWeight->weight_safety;

    // bool dirPriority = false;
    // int laneIdx = edge->getLaneID( localization.situation.mRegion[i]->offset );
    // MTS_Lane *lane = edge->getLane( laneIdx );
    // bool priority = lane->havePriority( typeCode );
    // bool permission = lane->havePermission( typeCode );
    // bool target = laneIdx == veh->getDesireLane();
  
    // MTS_Vehicle *brokenVehicle = lane->getBlockage();
    // bool blockage = lane->endOfRoad() || ( brokenVehicle != NULL && _checkBlockage( localization.situation.mRegion[i] ,  brokenVehicle ) );
    
    // float turnControl = _turnControl( veh , localization.situation.mRegion[i] );
    
    // float w_dir			= param->mRegionSelectionWeight->weight_targetDirection;
    // float w_blockage	= -param->mRegionSelectionWeight->weight_blockage;
    // float w_priority	= param->mRegionSelectionWeight->weight_priority;
    // float w_permission	= param->mRegionSelectionWeight->weight_permission; 
    // float w_target		= param->mRegionSelectionWeight->weight_targetLane;
    // float w_turnControl = param->mRegionSelectionWeight->weight_turnControl;

    localization.situation.mRegion[i]->preference = w_speed * speed_diff + w_gap * gap + w_dis * offset_diff + w_vel * velCosistent + w_safe * (safety);
                //+ w_dir * dirPriority + w_priority * priority + w_permission * permission + w_target * target + w_blockage * blockage + w_turnControl * turnControl;

    // if( localization.situation.mRegion[i]->preference > max_cost )
    // {
    //   best_space_ID = i;
    //   max_cost = localization.situation.mRegion[i]->preference;
    // }
    
    if(safeSpace && localization.situation.mRegion[i]->preference > max_valid_cost)
    {
      valid_space_ID = i;
      max_valid_cost = localization.situation.mRegion[i]->preference;
    }
  }

  if(valid_space_ID == -1 )
  {
    if(localization.situation.mRegion[0]->safety > 0.8f ) // veh->needTocutIn &&
      return localization.situation.mRegion[0]->offset;

    return actor_lateral_offset;
  }

  if(valid_space_ID == 0)
  {
    return localization.situation.mRegion[valid_space_ID]->offset;
  }

  if(actor_lateral_offset > localization.situation.mRegion[valid_space_ID]->offset)
  {
    return localization.situation.mRegion[valid_space_ID]->rightBorder - actor_half_width * 2;
  }
  if(actor_lateral_offset < localization.situation.mRegion[valid_space_ID]->offset)
  {
    return localization.situation.mRegion[valid_space_ID]->leftBorder + actor_half_width * 2;
  }
  
}

bool MotionPlanStage::CheckLeftSafety(ActorId actor_id, float desired_offset , float *safe_offset , MTS_Region *region) const
{
  float lateral_offet = simulation_state.GetLocation(actor_id).y; //local
  float t = GetLateralTime(desired_offset);
  float v = (desired_offset - lateral_offet) / t;
  
  //check BorderVehicle
  if(region->leftBorderVehicle)
  {
    ActorId *leftVeh = region->leftBorderVehicle;
    float leftVeh_desiredLateralOffset = GetDesiredLateralOffset(leftVeh); // ???
    bool lateralOverlap = (GetDynamicWidth(leftVeh) + GetDynamicWidth(actor_id)) / 2.0f < std::abs(leftVeh_desiredLateralOffset - desired_offset);
    
    if(lateralOverlap)
    { 
      float actor_head_offset = simulation_state.GetLocation(actor_id).x + GetDynamicLength(actor_id); //local
      float target_head_offset = simulation_state.GetLocation(leftVeh).x + GetDynamicLength(leftVeh); //local
      if(actor_head_offset <= target_head_offset)
        return 0.0f;
    }
  }

  bool safe = true;
  float safe_time;
  float min_safe_time = FLT_MAX;

  if(safe_offset) 
    *safe_offset = desired_offset;
  region->safety = 1.0;
  
  // for( ; it != vehEnd ; ++it )
  // {
  if(localization.surrounding.LeftRearVehicle)
    ActorId left_rear_vehicle = localization.surrounding.LeftRearVehicle.get();
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
      //MTS_VehicleController *controller = (*it)->getCurrentController();
      float current_offset = simulation_state.GetLocation(left_rear_vehicle).y; //local //controller->getLateralOffset();
      float current_speed = simulation_state.GetVelocity(left_rear_vehicle).y;//local //controller->getLateralSpeed();
      float min_dis = (GetPsychoWidth(simulation_state.GetVelocity(actor_id).x) + simulation_state.GetDimensions(actor_id).y * 2) / 2.0f + 2.0f; // local
      *safe_offset = current_offset + current_speed * safe_time + min_dis;
    }
    safe = false;
    region->safety = 0.0;
  }
  //}

  // const std::vector< MTS_Vehicle* > &frontVehicles = mSubject->getLeftFrontVehicles();
  
  // it = frontVehicles.begin();
  // vehEnd = frontVehicles.end();

  // for( ; it != vehEnd ; ++it )
  // {
  if(localization.surrounding.LeftFrontVehicle)
    ActorId left_front_vehicle = localization.surrounding.LeftFrontVehicle.get();
  
  bool checkSafe = CheckSafety(actor_id, left_front_vehicle, v, t, &safe_time);
  float patience = mSubject->getPatience();	
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
      //MTS_VehicleController *controller = (*it)->getCurrentController();
      float current_offset = simulation_state.GetLocation(left_front_vehicle).y; //controller->getLateralOffset();
      float current_speed = simulation_state.GetVelocity(left_front_vehicle).y; //controller->getLateralSpeed();
      float min_dis = (GetPsychoWidth(simulation_state.GetVelocity(actor_id).x) + simulation_state.GetDimensions(actor_id).y * 2) / 2.0f + 2.0f;
      *safe_offset = current_offset + current_speed * safe_time + min_dis;
    }
    safe = false;
    region->safety = 0.0;
  }
  //}
  return safe;
}

bool MotionPlanStage::CheckSafety(ActorId *actor_id, ActorId *target_id, float moveSpeed, float moveTime, float* safeTime) const
{
  cg::Vector2D v_s(simulation_state.GetVelocity(actor_id).x, moveSpeed ); // need to transform local
  cg::Vector2D v_o(simulation_state.GetVelocity(target_id).x, simulation_state.GetVelocity(target_id).y); // need to transform local
  cg::Vector2D p_r(GetRelativeOffset(actor_id, target_id), GetLateralSeparation(actor_id, target_id));
  cg::Vector2D v_r = v_s - v_o;

  float hw_s = simulation_state.GetDimensions(actor_id).y;
  float hw_o = GetPsychoWidth(simulation_state.GetVelocity(actor_id).x)/2.0f; //?
  float hl_s = simulation_state.GetDimensions(actor_id).x;
  float hl_o = simulation_state.GetDimensions(target_id).x;
  float d_y = std::abs(p_r.y);
  float scale = std::max(0, (d_y - hw_s - hw_o) / d_y);
  float t_y = p_r.y * scale / v_r.y;

  if(safeTime) 
    *safeTime = t_y;

  if(t_y < 0 || t_y > moveTime) 
  {
    return true;
  }
  
  float d_o = v_o.x * RESPONSE_TIME + (v_o.x * -v_r.x) / (2 * sqrt(MAX_ACC * COMFORTABLE_DEC));
  float d_s = v_s.x * RESPONSE_TIME + (v_s.x * v_r.x) / (2 * sqrt(MAX_ACC * COMFORTABLE_DEC));

  d_o = std::max( 0 , d_o);
  d_s = std::max( 0 , d_s);

  float p_r_s = p_r.x + v_o.x * t_y;
  float p_r_o = p_r.x - v_s.x * t_y;
  float p_r_t = p_r.x - v_r.x * t_y;
  float d_x = std::abs( p_r_t );

  if((p_r_t < 0 && p_r_t + hl_s + hl_o < -d_o * GAP_ACCEPT_RATIO) || (p_r_t >= 0 && p_r_t - hl_s - hl_o > d_s * GAP_ACCEPT_RATIO))
    return true;
  
  return false;
}

bool MotionPlanStage::CheckRightSafety(ActorId actor_id, float desired_offset , float *safe_offset , MTS_Region *region) const
{
  float lateral_offet = simulation_state.GetLocation(actor_id).y; //local 
  float t = GetLateralTime(desired_offset);
  float v = (desired_offset - lateral_offet) / t;

  //check BorderVehicle
  if(region->rightBorderVehicle)
  {
    ActorId *rightVeh = region->rightBorderVehicle;
    float rightVeh_desiredLateralOffset = GetDesiredLateralOffset(rightVeh); //???
    bool lateralOverlap = (GetDynamicWidth(rightVeh) + GetDynamicWidth(actor_id)) / 2.0f < std::abs(rightVeh_desiredLateralOffset - desired_offset);

    if( lateralOverlap )
    {
      float actor_head_offset = simulation_state.GetLocation(actor_id).x + GetDynamicLength(actor_id); //local
      float target_head_offset = simulation_state.GetLocation(rightVeh).x + GetDynamicLength(rightVeh); //local
      if( actor_head_offset <= target_head_offset )
        return 0.0f;
    }
  }

  bool safe = true;
  float safe_time;
  float min_safe_time = FLT_MAX;
  
  if( safe_offset ) 
    *safe_offset = desired_offset;
  
  region->safety = 1.0;
  
  // for( ; it != vehEnd ; ++it )
  // {
  if(c)
    ActorId right_rear_vehicle = localization.surrounding.RightRearVehicle.get();
  
  bool checkSafe = CheckSafety(actor_id, right_rear_vehicle, v, t, &safe_time);
  // float patience = mSubject->getPatience();
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
      //MTS_VehicleController *controller = (*it)->getCurrentController();
      float current_offset = simulation_state.GetLocation(right_rear_vehicle).y; //local
      float current_speed = simulation_state.GetVelocity(right_rear_vehicle).y;//local
      float min_dis = (GetPsychoWidth(simulation_state.GetVelocity(actor_id).x + simulation_state.GetDimensions(actor_id).y * 2) / 2.0f + 2.0f; //local
      *safe_offset = current_offset + current_speed * safe_time - min_dis;
    }
    safe = false;
    region->safety = 0.0;
  }
  //}

  // const std::vector< MTS_Vehicle* > &frontVehicles = mSubject->getRightFrontVehicles();
  
  // it = frontVehicles.begin();
  // vehEnd = frontVehicles.end();

  // for( ; it != vehEnd ; ++it )
  // {

  if(localization.surrounding.RightFrontVehicle)
  ActorId right_front_vehicle = localization.surrounding.RightFrontVehicle.get();
  
  bool checkSafe = CheckSafety(actor_id, right_front_vehicle, v, t, &safe_time);
  // float patience = mSubject->getPatience();
  // if( !checkSafe && patience > 0.75)
  // {
    
    
  //   if( mSubject->getRelativeOffset( (*it) ) - (*it)->getCurrentController()->getLength()*(-2) > 0.0 )
  //   {
  //     checkSafe = true;
  //     region->safety = 0.5;
  //   }
  // }

  if( !checkSafe )
  {
    if( safe_offset && safe_time < min_safe_time )
    {
      min_safe_time = safe_time;
      //MTS_VehicleController *controller = (*it)->getCurrentController();
      float current_offset = simulation_state.GetLocation(right_front_vehicle).y; //local
      float current_speed = simulation_state.GetVelocity(right_front_vehicle).y; //local
      float min_dis = (GetPsychoWidth(simulation_state.GetVelocity(actor_id).x) + simulation_state.GetDimensions(actor_id).y * 2) / 2.0f + 2.0f;
      *safe_offset = current_offset + current_speed  * safe_time - min_dis;
    }
    safe = false;
    region->safety = 0.0;
  }
  //}

  return safe;
}

float MotionPlanStage::GetLateralTime(float desired_lateral_offset, ActorId actor_id) const
{
  float lateral_offset = desired_lateral_offset - simulation_state.GetLocation(actor_id).y; //local
  float lateral_veocity = simulation_state.GetVelocity(actor_id).y;
  float lateral_time = 2.0f * lateral_offset / lateral_veocity;

  // if the time to decelerate to zero lateral speed is smaller than maximum movement time
  if( lateral_time > 0.0f && lateral_time < MAX_MOVEMENT_TIME ) 
    return lateral_time;

  return GetLongitudinalTime(sctor_id);
}

void MotionPlanStage::UpdateBestLateralOffset( MTS_MovingModelParameter* param , MTS_Vehicle* mSubject)
{
  float newLateralOffset = ComputeBestLateralOffset( mSubject , param );
  SetDesiredLateralOffset( newLateralOffset );
}

bool MotionPlanStage::CheckBlockage(LocalizationData &localization, ActorId *blockage ) const
{
  float space_center = (localization.region.leftBorder + localization.region.rightBorder) / 2.0f;
  float blockage_center = simulation_state.GetLocation(blockage).y; //local
  float space_width = localization.region.width;
  float blockage_width = simulation_state.GetDimensions(blockage).y; 
  float lateral_separation = std::abs(space_center - blockage_center);
  if(lateral_separation <= (space_width + blockage_width) / 2.0f)
    return true;

  return false;
}
*/
// Tool
/*
float MotionPlanStage::GetGapToStopLine(ActorId actor_id, float stopOffset) const
{
	float halfVehLen = simulation_state.GetDimensions(actor_id).x;
	//float headOffset = getOffset() + halfVehLen; // self.location

	return stopOffset - halfVehLen;
}

float MotionPlanStage::GetGapToStopLine() const
{
  float halfVehLen = simulation_state.GetDimensions(actor_id).x;
  // float headOffset = getOffset() + halfVehLen;
  float stopOffset = getLane()->getLength();
  //trafficlight location - halflength

  return stopOffset - headOffset;
}
*/
float MotionPlanStage::GetGap(ActorId actor_id, ActorId target_id)
{
	float halfPredLen = simulation_state.GetDimensions(target_id).x; //pred->getCurrentController()->getLength() / 2.0f;
	float halfVehLen = simulation_state.GetDimensions(actor_id).x; //getLength() / 2.0f;
	float relativeOffset = GetRelativeOffset(actor_id, target_id);
	float gap = relativeOffset - (halfPredLen + halfVehLen);
	
	return gap;
}

float MotionPlanStage::GetExtendedGap(ActorId actor_id, ActorId target_id)
{
  float actor_yaw = simulation_state.GetRotation(actor_id).yaw;
  
  if(actor_yaw == 0.0f)
    return 0.0f;

  float actor_yaw_abs = std::abs(actor_yaw);
	float halfVehLen = simulation_state.GetDimensions(actor_id).x;
  float halfWidth = simulation_state.GetDimensions(actor_id).y;

  float width1 = halfVehLen * 2 * sin(actor_yaw_abs);
  float width2 = halfWidth * 2 * cos(actor_yaw_abs);
  float length1 = halfVehLen * 2 * cos(actor_yaw_abs);
	float length2 = halfWidth * 2 * sin(actor_yaw_abs);

  float LeftWidth = 0.0f;
  float RightWidth = 0.0f;
  float MaxLeftGap = 0.0f;
  float MaxRightGap = 0.0f;

  if( actor_yaw > 0 )
  {
    LeftWidth = width2;
    RightWidth = width1;
    MaxLeftGap = length2;
    MaxRightGap = length1;
  }
  
  else
  {
    LeftWidth = width1;
    RightWidth = width2;
    MaxLeftGap = length1;
    MaxRightGap = length2;
  }
    
  float sepLatOffset = LeftWidth - halfWidth;
  float predHalfWidth = simulation_state.GetDimensions(target_id).y; 
  float myHalfWidth = simulation_state.GetDimensions(actor_id).y;
  float latSeparation = std::abs(GetLateralSeparation(actor_id, target_id));

  if( latSeparation > predHalfWidth + myHalfWidth )
    return 0.0f;

  float myLeftLatOffset = -myHalfWidth;
  float myRightLatOffset = myHalfWidth;
  float extendedGap = 0.0f;
  
  if(myLeftLatOffset > sepLatOffset)
  {
    float ratio = std::min((myLeftLatOffset - sepLatOffset) / RightWidth, 1.0f);
    extendedGap = MaxRightGap * ratio;
  }
  else if(myRightLatOffset < sepLatOffset)
  {
    float ratio = std::min((sepLatOffset - myRightLatOffset) / LeftWidth, 1.0f);
		extendedGap = MaxLeftGap * ratio;
  }

  return extendedGap;
}

float MotionPlanStage::GetRelativeOffset(ActorId actor_id, ActorId target_id)
{
  cg::Location target_location = simulation_state.GetLocation(target_id);

  std::array<float, 4> result = GlobalToLocal(actor_id, target_location);

  return -result[0];
} 

float MotionPlanStage::GetLateralSeparation(ActorId actor_id, ActorId target_id)
{
  cg::Location target_location = simulation_state.GetLocation(target_id);

  std::array<float, 4> result = GlobalToLocal(actor_id, target_location);

  return -result[1];
} 

float MotionPlanStage::GetDynamicWidth(ActorId actor_id) const
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

float MotionPlanStage::GetDynamicLength(ActorId actor_id) const
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

// Transform
std::array<float, 4> MotionPlanStage::GlobalToLocal(ActorId actor_id, cg::Location location)
{
  cg::Location actor_location = simulation_state.GetLocation(actor_id);
  cg::Rotation actor_rotation = simulation_state.GetRotation(actor_id);
  cg::Transform transform(actor_location, actor_rotation);
  std::array<float, 16> M_inv = transform.GetInverseMatrix(); // world to local
  std::array<float, 4> global_location = {location.x, location.y, location.z, 1.0f};
  
  return matrixMultiply(M_inv, global_location);
}

std::array<float, 4> MotionPlanStage::matrixMultiply(std::array<float, 16> M, std::array<float, 4> V)
{
  std::array<float, 4> result;
  
  for(size_t i=0; i < V.size(); i++ ){ // should check column or row major
    result[i]= M[4*i] * V[0] + M[4*i+1] * V[1] + M[4*i+2] * V[2] + M[4*i+3] * V[3];
  }

  return result;
}

} // namespace traffic_manager
} // namespace carla
