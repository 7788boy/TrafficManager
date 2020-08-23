
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
  const CollisionHazardData &collision_hazard = collision_frame.at(index);
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
  const float ego_speed_limit = simulation_state.GetSpeedLimit(actor_id);
  float max_target_velocity = parameters.GetVehicleTargetVelocity(actor_id, ego_speed_limit) / 3.6f;

  // Collision handling and target velocity correction.
  std::pair<bool, float> collision_response = CollisionHandling(collision_hazard, tl_hazard, ego_velocity,
                                                                ego_heading, max_target_velocity);
  bool collision_emergency_stop = collision_response.first;
  float dynamic_target_velocity = collision_response.second;


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

    //dynamic_target_velocity = GetLongitudinalAcc(ego_velocity.Length(), tl_hazard, float desired_velocity, other_velocity.Length(), float gap);
    // Consider collision avoidance decisions only if there is positive relative velocity of the ego vehicle (meaning, ego vehicle is closing the gap to the lead vehicle).
    if (ego_relative_speed > EPSILON_RELATIVE_SPEED) {
      // If other vehicle is approaching lead vehicle and lead vehicle is further than follow_lead_distance 0 kmph -> 5m, 100 kmph -> 10m.
      float follow_lead_distance = ego_relative_speed * FOLLOW_DISTANCE_RATE + MIN_FOLLOW_LEAD_DISTANCE;
      if (available_distance_margin > follow_lead_distance) {
        // Then reduce the gap between the vehicles till FOLLOW_LEAD_DISTANCE by maintaining a relative speed of RELATIVE_APPROACH_SPEED
        dynamic_target_velocity = other_speed_along_heading + RELATIVE_APPROACH_SPEED;
      }
      // If vehicle is approaching a lead vehicle and the lead vehicle is further than CRITICAL_BRAKING_MARGIN but closer than FOLLOW_LEAD_DISTANCE.
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

// // Longitudinal
// float MotionPlannerStage::GetLongitudinalAcc(float current_velocity, bool traffic_light_hazard, float desired_velocity, float object_velocity, float gap) 
// {
//   float acc_free = GetFreeAcc( current_velocity , desired_velocity);
//   float brake_first = GetAcc(gap,  current_velocity,  object_velocity);
//   float brake_second = GetAcc( gap,  current_velocity,  object_velocity);
//   float max_deceleration = std::max(brake_first, brake_second);

//   if( traffic_light_hazard )
//   {
//   	float brake_stopLine = GetDecForStopLine( param->stopLineOffset , acc_free , mSituationData ,mAgentData );
//     max_deceleration = std::max(max_deceleration, brake_stopLine)
//   }
  
//   float acc = acc_free - max_deceleration;

//   return acc;

// }

// float MotionPlannerStage::GetFreeAcc( float v_current , float v_desired)
// {
//   if( v_desired == 0.0f ) return 0.0f; 

//   float acc = MAX_ACC * ( 1.0f - pow( v_current/v_desired , FREE_ACCELERATION_EXP) );
//   return acc;
// }

// float MotionPlannerStage::GetAcc(float gap, float current_velocity, float object_velocity)
// {
//   if( object == NULL ) return 0.0f;

//   float gap = getGap(actor_id, object_id);
//   //float gap_prepare = object_velocity * TIME_TO_CONTACT;

//   //gap += gap_prepare;

//   bool gapExtended = false;
  
//   if( gap < 0.0f) // current leader but the gap less than 0
//   {
//   	// try to extend the gap according to the yaw angle of the leader
//   	gapExtended = true;
//   	gap += subjectController->getExtendedGap( object );
//   }

//   if( gap < 0.0f ) // the gap is less than 0 (even after extension)
//   	return 0.0f;
  

//   float s = ComputeDesiredGap(current_velocity, object_velocity);
  
//   bool approaching = current_velocity - object_velocity >= 0.0f;
//   bool closeEnough = gap <= 1.2f * s;

//   float acc_gapApproachSpeed = 2.0f;//mAgentData->mDriver->mGapdAccelerationExponent;
//   float acc = MAX_ACC * std::pow( s / gap , acc_gapApproachSpeed ) * ( approaching || closeEnough );

//   if( !approaching && closeEnough )
//     acc = MAX_ACC * (s/gap);
  
  
//   if( gapExtended == false && acc > maxDec )
//   {
//   	float extendedGap = subjectController->getExtendedGap( object );
//   	if( extendedGap != 0 )
//   	{
//   		float gapRatio = gap / ( gap + extendedGap );
//   		acc *= gapRatio * gapRatio;
//   	}
//   }

//   return acc;
// }

// float MotionPlannerStage::GetDecForStopLine( float stopLine , float freeAcc , MTS_SituationData*	mSituationData , MTS_AgentData*	mAgentData ) const
// {
// 	// compute gap to stopline and evaluate the deceleration
// 	float currentSpeed = mSituationData->mSubject->getCurrentSpeed();
// 	float gap = mSituationData->mSubject->getCurrentController()->getGapToStopLine(stopLine) ;
// 	if( gap <= 1.0 ) gap = 1.0;

// 	float acc_stopline ;//= freeAcc;

// 	float b_com = mAgentData->mDriver->mComfortableDeceleration;
// 	float a_max = mAgentData->mVehicleCapability->mMaxAcceleration;
// 	float MinGap =  mAgentData->mDriver->mMinGap;
// 	float responseTime = mAgentData->mDriver->mResponseTime ;
// 	float s = MinGap + currentSpeed*responseTime + currentSpeed*currentSpeed / (2*sqrt(b_com*a_max));
// 	acc_stopline = b_com * (s*s) / (gap*gap) ;


// 	return acc_stopline;
// }

// float MotionPlannerStage::ComputeDesiredGap(float current_velocity, float object_velocity)
// {
//   if(object == NULL) // how to judge
//   	return min_gap + current_velocity * RESPONSE_TIME + current_velocity * current_velocity / (2 * sqrt(MAX_ACC * COMFORTABLE_DEC));
  
//   float speedCon = (current_velocity * RESPONSE_TIME + current_velocity * (current_velocity - object_velocity) / (2 * sqrt(MAX_ACC * COMFORTABLE_DEC)));

//   float desired_gap = (MIN_GAP  + speedCon);
//   if(desired_gap < 0.0f) desired_gap = MIN_GAP;

//   return desired_gap;
// }

// // Lateral
// float MotionPlannerStage::getSpeedChange( MTS_MovingModelParameter* param , MTS_Vehicle* mSubject)
// {
//   //float lateralSpeedChange;

//   this->mSubject = mSubject;
  
//   float diff_lat = mSubject->getDesiredLateralOffset() - mSubject->getCurrentController()->getLateralOffset();
//   float v_lat = mSubject->getCurrentController()->getLateralSpeed();
//   float t_lat = 2.0f * diff_lat / v_lat;

//   // if the time to decelerate to zero lateral speed is smaller than maximum movement time
//   if( t_lat > 0.0f && t_lat < 2.0 ) 
//   {
//     //t_lat = t_lat;
//   }
//   else
//   {
//     t_lat = _getLongitudinalTime( mSubject , 2.0f );
//   }


//   float lateralSpeedChange = _getLateralSpeed( mSubject , t_lat );

//   return lateralSpeedChange;
// }

// float MotionPlannerStage::_getLongitudinalTime( MTS_Vehicle* mSubject , float mMaxMovementTime ) const
// {
//   MTS_Vehicle *pred = mSubject->getPassedLeadingVehicle();

//   float v = mSubject->getCurrentController()->getCurrentSpeed();
//   float t_long = mSubject->getCurrentController()->getGapToStopLine() / v;

//   if( pred != NULL )
//   {
//     float v_pred = 0.0f;
//     float deltaV = v - v_pred;

//     float s = mSubject->getCurrentController()->getGap( pred ); 
    
//     float ttc = s / deltaV;
//     if( ttc < t_long )
//       t_long = ttc;
//   }
//   if( t_long > 0 && t_long < mMaxMovementTime )
//     return t_long;

//   return mMaxMovementTime;
// }

// float MotionPlannerStage::_getLateralSpeed( const MTS_Vehicle *veh , float moveTime ) const
// {
//   MotionPlannerStage::ControllerType controllerType = veh->getControllerType();
//   const MTS_VehicleController *vehController = veh->getCurrentController();
  
//   //float a_max = veh->getMaxAcceleration();

//   float x_desired = vehController->getDesiredLateralOffset();//getDynamicDesiredOffset( veh );
//   float x_current = vehController->getLateralOffset();
//   float v_lateral = vehController->getLateralSpeed();

//   //float t = moveTime;

//   //float acc = 0.0f;

//   float acc = 2 * ((x_desired-x_current) / (moveTime * moveTime) - v_lateral / moveTime);

//   if( acc < 0 )
//     acc = MAX( -MAX_ACC , acc );
//   else
//     acc = MIN( MAX_ACC , acc );

//   return acc;

// }

// float MotionPlannerStage::_computeBestLateralOffset( MTS_Vehicle *veh , MTS_MovingModelParameter* param )
// {
//   MTS_Vehicle *left = veh->getLeftVehicle();
//   MTS_Vehicle *right = veh->getRightVehicle();
//   MTS_Edge *edge = veh->getLane()->getEdge();

//   int typeCode = veh->getVehicleType()->getTypeCode();
//   float halfVehWidth = veh->getVehicleType()->getStaticWidth() / 2.0f;
//   float vehLateralOffset = veh->getLateralOffset();
//   float vehOffset = veh->getOffset() + veh->getVehicleType()->getStaticLength() / 2.0f;
//   float vehSpeed = veh->getCurrentSpeed();
//   MTS_Region &validSpace = param->currentRegion;

//   int validSpaceID = -1;
//   int bestSpaceID = -1;
//   float maxCost = 0.0f;
//   float maxValidCost = 0.0f;

//   int spaceSize = param->allRegion.size();
  
//   for( int i = 0 ; i < spaceSize ; ++i )
//   {
//     if( param->allRegion[i]->width < 1.8f * halfVehWidth )
//       continue;
//     float laneCenter = param->allRegion[i]->offset;

//     float offset_diff = param->allRegion[i]->offset - vehLateralOffset;
//     float safeOffset;

//     bool safeSpace = true;
//     if( param->allRegion[i]->width < veh->getVehicleType()->getStaticWidth() )
//     {
//       safeSpace = false;
//       param->allRegion[i]->safety = 0.0f;
//     }
//     else if( offset_diff < 0 ) // check if the front and the rear of subject vehicle at the desired offset are safe
//     {
//       bool safeLeftSpace = _checkLeftSafety( param->allRegion[i]->offset , &safeOffset , param->allRegion[i] );
//       bool adjustOffset = param->mSpaceOriented  && safeOffset < vehLateralOffset;

//       if( !safeLeftSpace && !adjustOffset ) 
//         safeSpace = false;

//       else if( !safeLeftSpace )
//         offset_diff = safeOffset - vehLateralOffset;
//     }
//     else if( offset_diff > 0 ) // check if the front and the rear of subject vehicle at the desired offset are safe
//     {
//       bool safeRightSpace = _checkRightSafety( param->allRegion[i]->offset , &safeOffset , param->allRegion[i]);
//       bool adjustOffset = param->mSpaceOriented  && safeOffset > vehLateralOffset;

//       if( !safeRightSpace && !adjustOffset )
//         safeSpace = false;

//       else if( !safeRightSpace )
//         offset_diff = safeOffset - vehLateralOffset;
//     }

//     bool dirPriority = false;
//     offset_diff = ABS( offset_diff );
//     float gap = param->allRegion[i]->gap;
//     int laneIdx = edge->getLaneID( param->allRegion[i]->offset );
//     MTS_Lane *lane = edge->getLane( laneIdx );
//     bool priority = lane->havePriority( typeCode );
//     bool permission = lane->havePermission( typeCode );
//     bool target = laneIdx == veh->getDesireLane();
//     bool velCosistent = ( offset_diff * veh->getLateralSpeed() ) > 0;		
//     float speed_diff = param->allRegion[i]->maxPassingSpeed - vehSpeed;
//     MTS_Vehicle *brokenVehicle = lane->getBlockage();
//     bool blockage = lane->endOfRoad() || ( brokenVehicle != NULL && _checkBlockage( param->allRegion[i] ,  brokenVehicle ) );
//     float safety = param->allRegion[i]->safety;
//     float turnControl = _turnControl( veh , param->allRegion[i] );
//     float w_speed		= param->mRegionSelectionWeight->weight_speed;
//     float w_gap			= param->mRegionSelectionWeight->weight_gap;
//     float w_dis			= param->mRegionSelectionWeight->weight_lateralDistance;
//     float w_blockage	= -param->mRegionSelectionWeight->weight_blockage;
//     float w_priority	= param->mRegionSelectionWeight->weight_priority;
//     float w_permission	= param->mRegionSelectionWeight->weight_permission; 
//     float w_target		= param->mRegionSelectionWeight->weight_targetLane;
//     float w_dir			= param->mRegionSelectionWeight->weight_targetDirection;
//     float w_vel			= param->mRegionSelectionWeight->weight_velocityConsistency;
//     float w_safe		= param->mRegionSelectionWeight->weight_safety;
//     float w_turnControl = param->mRegionSelectionWeight->weight_turnControl;

//     param->allRegion[i]->preference = 
//                 w_speed * speed_diff +
//                 w_gap * gap + 
//                 w_dis * offset_diff + 
//                 w_priority * priority +
//                 w_permission * permission +
//                 w_target * target +
//                 w_blockage * blockage +
//                 w_dir * dirPriority +
//                 w_vel * velCosistent +
//                 w_safe * (safety) +
//                 w_turnControl * turnControl
//                 ;
          

//     if( param->allRegion[i]->preference > maxCost )
//     {
//       bestSpaceID = i;
//       maxCost = param->allRegion[i]->preference;
//     }
    
//     if( safeSpace && param->allRegion[i]->preference > maxValidCost )
//     {
//       validSpaceID = i;
//       maxValidCost = param->allRegion[i]->preference;
//     }
//   }

//   if( validSpaceID == -1 )
//   {
//     if( veh->needTocutIn && param->allRegion[0]->safety > 0.8f ) return param->allRegion[ 0 ]->offset;
//     return vehLateralOffset;
//   }

//   if( validSpaceID == 0 )
//   {
//     return param->allRegion[ validSpaceID ]->offset;
//   }

//   if( veh->getLateralOffset() > param->allRegion[ validSpaceID ]->offset )
//   {
//     return param->allRegion[ validSpaceID ]->rightBorder - veh->getVehicleType()->getStaticWidth();
//   }
//   if( veh->getLateralOffset() < param->allRegion[ validSpaceID ]->offset )
//   {
//     return param->allRegion[ validSpaceID ]->leftBorder + veh->getVehicleType()->getStaticWidth();
//   }
  
// }

// bool MotionPlannerStage::_checkLeftSafety( float desiredOffset , float *safeOffset , MTS_Region *region) const
// {
//   const std::vector< MTS_Vehicle* > &rearVehicles = mSubject->getLeftRearVehicles();
//   std::vector< MTS_Vehicle* >::const_iterator it = rearVehicles.begin();
//   std::vector< MTS_Vehicle* >::const_iterator vehEnd = rearVehicles.end();
//   float lateralOffet = mSubject->getCurrentController()->getLateralOffset();
//   float t = getLateralTime( desiredOffset );
//   float v = ( desiredOffset - lateralOffet ) / t;
//   bool safe = true;
//   float safeTime;
//   float minSafeTime = FLT_MAX;

//   //check BorderVehicle
//   if( region->leftBorderVehicle!=NULL && region->leftBorderVehicle->getActive() )
//   {
//     MTS_Vehicle* leftVeh = region->leftBorderVehicle;
//     float leftVeh_desiredLateralOffset = leftVeh->getDesiredLateralOffset();
//     float leftVehAngle = leftVeh->getCurrentController()->getYawAngle();
//     float subjectAngle = mSubject->getCurrentController()->getYawAngle();
//     bool lateralOverlap = (leftVeh->getVehicleType()->getDynamicWidth( leftVehAngle ) + mSubject->getVehicleType()->getDynamicWidth( subjectAngle )  )/2.0f < abs(leftVeh_desiredLateralOffset-desiredOffset) ;
//     if( lateralOverlap )
//     { 
//       float vehHeadOffset = mSubject->getOffset() + mSubject->getVehicleType()->getDynamicLength( mSubject->getCurrentController()->getYawAngle() );
//       float objetVehHeadOffset = leftVeh->getOffset() + leftVeh->getVehicleType()->getDynamicLength( leftVeh->getCurrentController()->getYawAngle() );
//       if( vehHeadOffset <= objetVehHeadOffset )
//         return 0.0f;
//     }
//   }

//   if( safeOffset ) *safeOffset = desiredOffset;
//   region->safety = 1.0;
  
//   for( ; it != vehEnd ; ++it )
//   {
//     bool checkSafe = _checkSafety( mSubject , *it , v , t , true , true , &safeTime );
//     float patience = mSubject->getPatience();
//     if( !checkSafe && patience > 0.75)
//     {
//       /*
//       if( mSubject->getRelativeOffset( (*it) ) - (*it)->getCurrentController()->getLength()*(-2)*patience > 0.0 )
//       {
//         checkSafe = true;
//         region.safety = 0.5;
//       }
//       */
//       region->safety = 0.85;
//       checkSafe = true;
//     }

//     if( !checkSafe )
//     {
//       if( safeOffset && safeTime < minSafeTime )
//       {
//         minSafeTime = safeTime;
//         MTS_VehicleController *controller = (*it)->getCurrentController();
//         float currentOffset = controller->getLateralOffset();
//         float currentSpeed = controller->getLateralSpeed();
//         float minDis = ( controller->getPsychoWidth( mSubject->getCurrentSpeed() ) + mSubject->getVehicleType()->getStaticWidth() ) / 2.0f + 2.0f;
//         *safeOffset = currentOffset + currentSpeed * safeTime + minDis;
      
//       }
//       safe = false;
//       region->safety = 0.0;
//     }
    
//     //if( safeTime >= 0 && mNecessityValue == 1.0f )
//     //	involvedRear.push_back( InvolvedVehiclePair( *it , safeTime ) );
//   }

//   const std::vector< MTS_Vehicle* > &frontVehicles = mSubject->getLeftFrontVehicles();
  
//   it = frontVehicles.begin();
//   vehEnd = frontVehicles.end();

//   for( ; it != vehEnd ; ++it )
//   {
//     bool checkSafe = _checkSafety( mSubject , *it , v , t , true , true , &safeTime );
//     float patience = mSubject->getPatience();	
//     if( !checkSafe && patience > 0.75)
//     {

      
//       if( mSubject->getRelativeOffset( (*it) ) - (*it)->getCurrentController()->getLength()*(-2) > 0.0 )
//       {
//         checkSafe = true;
//         region->safety = 0.5;
//       }
//     }

//     if( !checkSafe )
//     {
//       if( safeOffset && safeTime < minSafeTime )
//       {
//         minSafeTime = safeTime;
//         MTS_VehicleController *controller = (*it)->getCurrentController();
//         float currentOffset = controller->getLateralOffset();
//         float currentSpeed = controller->getLateralSpeed();
//         float minDis = ( controller->getPsychoWidth( mSubject->getCurrentSpeed() ) + mSubject->getVehicleType()->getStaticWidth() ) / 2.0f + 2.0f;
//         *safeOffset = currentOffset + currentSpeed * safeTime + minDis;
//       }
//       safe = false;
//       region->safety = 0.0;
//     }
//     //if( safeTime >= 0 && mNecessityValue == 1.0f )
//     //	involvedFront.push_back( InvolvedVehiclePair( *it , safeTime ) );
//   }

//   return safe;

// }

// bool MotionPlannerStage::_checkSafety( MTS_Vehicle *subject , MTS_Vehicle *object , float moveSpeed , float moveTime , bool subjectAsLeader , bool subjectAsFollower , float* safeTime ) const
// {
//   MTS_VehicleController *subjectController = subject->getCurrentController();
//   MTS_VehicleController *objectController = object->getCurrentController();
//   const MTS_Edge *last = objectController->cooperate( subjectController );
//   Vector2 v_s( subjectController->getCurrentSpeed() , moveSpeed );
//   Vector2 v_o( objectController->getCurrentSpeed() , objectController->getLateralSpeed() );
//   Vector2 p_r;
//   p_r.y = object->getLateralSeparation( subject );
//   p_r.x = object->getRelativeOffset( subject );
//   Vector2 v_r = v_s - v_o;
//   float hw_s = subjectController->getWidth()/2.0f;
//   float hw_o = objectController->getPsychoWidth( mSubject->getCurrentSpeed() )/2.0f;
//   float hl_s = subjectController->getLength()/2.0f;
//   float hl_o = objectController->getLength()/2.0f;
//   float d_y = ABS( p_r.y );
//   float scale = ( d_y - hw_s - hw_o ) / d_y;
//   scale = MAX( 0 , scale );
//   float t_y = p_r.y * scale / v_r.y;

//   if( safeTime ) *safeTime = t_y;

//   if( t_y < 0 || t_y > moveTime) 
//   {
//     return true;
//   }
  
//   float d_o = 
//         v_o.x * object->getResponseTime() + 
//         ( v_o.x * -v_r.x ) / ( 2 * sqrt( object->getMaxAcceleration() * object->getComfortableDeceleration() ) );
  
//   float d_s = 
//         v_s.x * subject->getResponseTime() + 
//         ( v_s.x * v_r.x ) / ( 2 * sqrt( subject->getMaxAcceleration() * subject->getComfortableDeceleration() ) );

//   d_o = MAX( 0 , d_o);
//   d_s = MAX( 0 , d_s);

//   float p_r_s = p_r.x + v_o.x * t_y;
//   float p_r_o = p_r.x - v_s.x * t_y;
//   float p_r_t = p_r.x - v_r.x * t_y ;
//   float d_x = ABS( p_r_t );

//   if( ( p_r_t < 0 && p_r_t + hl_s + hl_o < -d_o * subject->getGapAcceptRatio() ) || (  p_r_t >= 0 && p_r_t - hl_s - hl_o > d_s * subject->getGapAcceptRatio() ) )
//     return true;
  
//   return false;
// }

// bool MotionPlannerStage::_checkRightSafety( float desiredOffset , float *safeOffset , MTS_Region *region) const
// {
//   const std::vector< MTS_Vehicle* > &rearVehicles = mSubject->getRightRearVehicles();
//   std::vector< MTS_Vehicle* >::const_iterator it = rearVehicles.begin();
//   std::vector< MTS_Vehicle* >::const_iterator vehEnd = rearVehicles.end();
//   float t = getLateralTime( desiredOffset );
//   float lateralOffset = mSubject->getCurrentController()->getLateralOffset();
//   float v = ( desiredOffset - lateralOffset ) / t;

//   //check BorderVehicle
//   if( region->rightBorderVehicle!=NULL && region->rightBorderVehicle->getActive() )
//   {
//     MTS_Vehicle* rightVeh = region->rightBorderVehicle;
//     float rightVeh_desiredLateralOffset = rightVeh->getDesiredLateralOffset();
//     float rightVehAngle = rightVeh->getCurrentController()->getYawAngle();
//     float subjectAngle = mSubject->getCurrentController()->getYawAngle();
//     bool lateralOverlap = (rightVeh->getVehicleType()->getDynamicWidth( rightVehAngle ) + mSubject->getVehicleType()->getDynamicWidth( subjectAngle ) )/2.0f < abs(rightVeh_desiredLateralOffset-desiredOffset) ;
//     if( lateralOverlap )
//     {
//       float vehHeadOffset = mSubject->getOffset() + mSubject->getVehicleType()->getDynamicLength( mSubject->getCurrentController()->getYawAngle() );
//       float objetVehHeadOffset = rightVeh->getOffset() + rightVeh->getVehicleType()->getDynamicLength( rightVeh->getCurrentController()->getYawAngle() );
//       if( vehHeadOffset <= objetVehHeadOffset )
//         return 0.0f;
//     }

//   }

//   bool safe = true;
//   float safeTime;
//   float minSafeTime = FLT_MAX;
  
//   if( safeOffset ) *safeOffset = desiredOffset;
  
//   region->safety = 1.0;
  
//   for( ; it != vehEnd ; ++it )
//   {
//     bool checkSafe = _checkSafety( mSubject , *it , v , t , true , true , &safeTime );
//     float patience = mSubject->getPatience();
//     if( !checkSafe && patience > 0.75)
//     {
//       region->safety = 0.85;
//       checkSafe = true;
//     }

//     if( !checkSafe )
//     {
//       if( safeOffset && safeTime < minSafeTime )
//       {
//         minSafeTime = safeTime;
//         MTS_VehicleController *controller = (*it)->getCurrentController();
//         float currentOffset = controller->getLateralOffset();
//         float currentSpeed = controller->getLateralSpeed();
//         float minDis = ( controller->getPsychoWidth( mSubject->getCurrentSpeed() ) + mSubject->getVehicleType()->getStaticWidth() ) / 2.0f + 2.0f;
//         *safeOffset = currentOffset + currentSpeed * safeTime - minDis;
//       }
//       safe = false;
//       region->safety = 0.0;
//     }
//   }

//   const std::vector< MTS_Vehicle* > &frontVehicles = mSubject->getRightFrontVehicles();
  
//   it = frontVehicles.begin();
//   vehEnd = frontVehicles.end();

//   for( ; it != vehEnd ; ++it )
//   {
//     bool checkSafe = _checkSafety( mSubject , *it , v , t , true , true , &safeTime );
//     float patience = mSubject->getPatience();
//     if( !checkSafe && patience > 0.75)
//     {
      
      
//       if( mSubject->getRelativeOffset( (*it) ) - (*it)->getCurrentController()->getLength()*(-2) > 0.0 )
//       {
//         checkSafe = true;
//         region->safety = 0.5;
//       }
//     }

//     if( !checkSafe )
//     {
//       if( safeOffset && safeTime < minSafeTime )
//       {
//         minSafeTime = safeTime;
//         MTS_VehicleController *controller = (*it)->getCurrentController();
//         float currentOffset = controller->getLateralOffset();
//         float currentSpeed = controller->getLateralSpeed();
//         float minDis = ( controller->getPsychoWidth( mSubject->getCurrentSpeed() ) + mSubject->getVehicleType()->getStaticWidth() ) / 2.0f + 2.0f;
//         *safeOffset = currentOffset + currentSpeed  * safeTime - minDis;
//       }
//       safe = false;
//       region->safety = 0.0;
//     }
//   }

//   return safe;
// }

// float MotionPlannerStage::getLateralTime( float mDesiredLateralOffset ) const
// {
//   float diff_lat = mDesiredLateralOffset - mSubject->getCurrentController()->getLateralOffset();
//   float v_lat = mSubject->getCurrentController()->getLateralSpeed();
//   float t_lat = 2.0f * diff_lat / v_lat;

//   // if the time to decelerate to zero lateral speed is smaller than maximum movement time
//   if( t_lat > 0.0f && t_lat < mMaxMovementTime ) 
//     return t_lat;

//   float ttc = getLongitudinalTime();

//   return ttc;
// }

// void MotionPlannerStage::updateBestLateralOffset( MTS_MovingModelParameter* param , MTS_Vehicle* mSubject)
// {
//   float newLateralOffset = _computeBestLateralOffset( mSubject , param );
//   mSubject->setDesiredLateralOffset( newLateralOffset );
// }

// bool MotionPlannerStage::_checkBlockage( MTS_Region *space, const MTS_Vehicle *blockage ) const
// {
//   float spaceCenter = ( space->leftBorder + space->rightBorder ) / 2.0f;
//   float blockageCenter = blockage->getCurrentController()->getLateralOffset();
//   float spaceWidth = space->width;
//   float blockageWidth = blockage->getCurrentController()->getWidth(); 
//   float lateralSeparation = ABS( spaceCenter - blockageCenter );
//   if( lateralSeparation <= ( spaceWidth + blockageWidth ) / 2.0f )
//     return true;

//   return false;
// }


// // Tool
// float MotionPlannerStage::getGap(int actor_id, int object_id) const
// {
// 	float halfPredLen = simulation_state.GetDimensions(object_id).x; //pred->getCurrentController()->getLength() / 2.0f;
// 	float halfVehLen = simulation_state.GetDimensions(actor_id).x; //getLength() / 2.0f;
// 	float relativeOffset = getRelativeOffset(actor_id, object_id);
// 	float gap = relativeOffset - (halfPredLen + halfVehLen);
	
// 	return gap;
// }

// float MotionPlannerStage::getGapToStopLine(float stopOffset, int actor_id) const
// {
// 	float halfVehLen = simulation_state.GetDimensions(actor_id).x;
// 	float headOffset = getOffset() + halfVehLen; // self.location

// 	return stopOffset - headOffset;
// }

// float MotionPlannerStage::GetRelativeOffset(int actor_id, int actor_id) const
// {
//   float object_offset = simulation_state.GetLocation(object_id);
//   float self_offset = simulation_state.GetLocation(actor_id);

//   return self_offset - object_offset;
// } 

// float MotionPlannerStage::getExtendedGap( const MTS_Vehicle *pred ) const
// {
//   MTS_VehicleController *controller = pred->getCurrentController();

//   if( controller->getYawAngle() == 0.0f )
//     return 0.0f;

//   float sepLatOffset = controller->getSeparationLateralOffset();
//   float myLatOffset = getLateralOffset();

//   float predHalfWidth = pred->getCurrentController()->getPsychoWidth( mSubject->getCurrentSpeed() ) / 2.0f;
//   float myHalfWidth = simulation_state.GetDimensions(actor_id).x; //mSubject->mType->getStaticWidth() / 2.0f;
//   float latSeparation = mSubject->getLateralSeparation( pred );
//   latSeparation = ABS( latSeparation );
  
//   if( latSeparation > predHalfWidth + myHalfWidth )
//     return 0.0f;

//   float myLeftLatOffset = myLatOffset - myHalfWidth;
//   float myRightLatOffset = myLatOffset + myHalfWidth;
//   float extendedGap = 0.0f;
  
//   if( myLeftLatOffset > sepLatOffset )
//     extendedGap = controller->getExtendedDistance( myLeftLatOffset );
//   else if( myRightLatOffset < sepLatOffset )
//     extendedGap = controller->getExtendedDistance( myRightLatOffset );

//   return extendedGap;
// }


} // namespace traffic_manager
} // namespace carla
