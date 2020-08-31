// // 更新狀態
// _updateSituation();

// // 更新參數, 找carla中同意義的attribute
// //mBehaviorController->caculateParameter(&param,mSituationData,mAgentData);

// // 更新IDM 縱向加速度 - partial done
// speedChange = mLongitudinalModel->getSpeedChange(&param,mSituationData,mAgentData);

// // 更新側向位移
// if( mSituationData->safety <= 0.0f  || changeLane )
// {
//     mLateralModel->updateBestLateralOffset(&param,this);
// }

// // 更新側向加速度 （follow IDM）
// lateralSpeedChange = mLateralModel->getSpeedChange(&param,this);
// // 清除neighbor
// resetNeighbor();

// void MTS_Vehicle::_updateSituation()
// {
//     ///////////////////////////////////////////
//     // update leader
//     mSituationData->mNeighbor->prevFirstLeader = mSituationData->mNeighbor->mPassedLeadingVehicle;
//     mSituationData->mNeighbor->prevSecondLeader = mSituationData->mNeighbor->mExpectedLeadingVehicle;
//     mCurrentController->updateLeader();
//     ///////////////////////////////////////////
//     mSituationData->updateNeighbor();
//     mSituationData->updateRegion();
//     mSituationData->evaluateSafety();

// }

// void updateLeader()
// {
//     float minOffset = subjectOffset;
//     float maxOffset = minOffset + subject->getMaxObservingDistance();	
    
//     std::deque<MTS_Vehicle*>::const_iterator it = mVehicles.begin();
		
//     float block_middle = (minOffset+maxOffset)/2.0f;
//     float block_len = maxOffset - block_middle;
//     const float MAXLEN = 120.0f;

//     for( ; it!=mVehicles.end() ; ++it )
//     {
//         if ( !(*it)->getActive() ) continue;
//         MTS_VehicleController *controller = (*it)->getCurrentController();
//         const MTS_Edge *last = controller->bindEdge( mEdge );
//         float halfVehLen = controller->getLength() / 2.0f;
//         float vehOffset = controller->getOffset();
//         float dis_long = ABS( (block_middle-vehOffset) );
//         controller->bindEdge( last );
//         if( dis_long < block_len + halfVehLen )
//         {
//             result.push_back( *it );
//         } 

//         // 根據距離及車道來判斷屬於左前左後右前右後
//         // 把neighbor的判斷包進來
//     }
// }

// void MTS_SituationData::_updateBaseRegion( const MTS_Vehicle *veh , const MTS_Lane *lane )
// 	{
// 		MTS_Vehicle *pred = veh->getPassedLeadingVehicle();
// 		MTS_VehicleController *controller =  veh->getCurrentController();
// 		float halfLaneWidth = lane->getWidth()/2.0f;

// 		float gap = controller->getGapToStopLine();
// 		if( pred != NULL )
// 			gap = controller->getGap( pred );
// 		mCurrentRegion.gap = gap;

// 		mCurrentRegion.offset = lane->getCentralOffset();
// 		mCurrentRegion.leftBorder = mCurrentRegion.offset - halfLaneWidth;
// 		mCurrentRegion.rightBorder = mCurrentRegion.offset + halfLaneWidth;
// 		mCurrentRegion.width = 2.0f * halfLaneWidth;

// 		float maxSpeed = controller->getDesiredSpeed();
// 		maxSpeed = MIN( lane->getMaxPassingSpeed() , maxSpeed );
// 		if( pred ) maxSpeed = MIN( pred->getCurrentSpeed() , maxSpeed );
// 		mCurrentRegion.maxPassingSpeed = maxSpeed;
// 	}

// MTS_Region MTS_SituationData::_setRegionParameter( const MTS_Vehicle *veh , const MTS_Lane *lane )

// 	{
// 		MTS_Region resultSpace;
		
// 		MTS_VehicleController *vehController =  veh->getVehicleController();
// 		float halfLaneWidth = lane->getWidth()/2.0f;

// 		resultSpace.offset = lane->getCentralOffset();
// 		resultSpace.leftBorder = resultSpace.offset - halfLaneWidth;
// 		resultSpace.rightBorder = resultSpace.offset + halfLaneWidth;
// 		resultSpace.width = 2.0f * halfLaneWidth;

// 		resultSpace.leftBorderVehicle = ((MTS_Vehicle *)veh)->getLeftVehicle();
// 		resultSpace.rightBorderVehicle = ((MTS_Vehicle *)veh)->getRightVehicle();

// 		float minOffset = veh->getOffset() + vehController->getLength() / 2.0f;
// 		float maxOffset = minOffset + 500.0f;
// 		float laneLen = lane->getLength();
// 		std::vector< MTS_Vehicle* > predVeh;

// 		lane->getVehicleInBlock( minOffset , maxOffset , predVeh );
// 		float predVehSize = predVeh.size();
// 		float maxSpeed = vehController->getDesiredSpeed();
// 		maxSpeed = MIN( lane->getMaxPassingSpeed() , maxSpeed );

// 		float minGap = laneLen;
// 		for( int i = 0 ; i < predVehSize ; ++i )
// 		{
// 			//avgSpeed += predVeh[i]->getCurrentSpeed();
// 			float offset = predVeh[i]->getOffset() - predVeh[i]->getVehicleController()->getLength()/2.0f;
// 			if( offset > minOffset && offset < minGap ) 
// 			{
// 				minGap = offset;
// 				maxSpeed = predVeh[i]->getCurrentSpeed();
// 			}
// 		}

// 		//avgSpeed /= predVehSize;
// 		minGap -= minOffset;

// 		resultSpace.maxPassingSpeed = maxSpeed;
// 		resultSpace.gap = minGap;

// 		return resultSpace;


// 	}

// bool MTS_SituationData::_checkSafety( MTS_Vehicle *subject , MTS_Vehicle *object , float moveSpeed , float moveTime , bool subjectAsLeader , bool subjectAsFollower , float* safeTime ) const
// {
//     MTS_VehicleController *subjectController = subject->getCurrentController();
//     MTS_VehicleController *objectController = object->getCurrentController();

//     const MTS_Edge *last = objectController->cooperate( subjectController );

//     //Vector2 p_s( 0 , subjectController->getLateralOffset() );
//     //Vector2 p_o( 0 , objectController->getLateralOffset() );
//     Vector2 v_s( subjectController->getCurrentSpeed() , moveSpeed );
//     Vector2 v_o( objectController->getCurrentSpeed() , objectController->getLateralSpeed() );

//     Vector2 p_r;
//     p_r.y = object->getLateralSeparation( subject );
//     p_r.x = object->getRelativeOffset( subject );

//     Vector2 v_r = v_s - v_o;
    
//     float hw_s = subjectController->getWidth()/2.0f;
//     float hw_o = objectController->getPsychoWidth( mSubject->getCurrentSpeed() )/2.0f;

//     float hl_s = subjectController->getLength()/2.0f;
//     float hl_o = objectController->getLength()/2.0f;

//     objectController->bindEdge( last );

//     float d_y = ABS( p_r.y );

//     float scale = ( d_y - hw_s - hw_o ) / d_y;
//     scale = MAX( 0 , scale );

//     float t_y = p_r.y * scale / v_r.y;
//     if( safeTime ) *safeTime = t_y;

//     // subject will not be the leader or the follower of object
//     if( t_y < 0 || t_y > moveTime) 
//     {
//         //MTS::SystemLog << "V" << subject->getID() << " and V" << object->getID() << ": Safe(not involved)\n";
//         //MTS::SystemLog << " t= " << t_y <<  "t_lat= " << moveTime << '\n';
//         return true;
//     }
    
//     float d_o = 
//                 v_o.x * object->getResponseTime() + 
//                 ( v_o.x * -v_r.x ) / ( 2 * sqrt( object->getMaxAcceleration() * object->getComfortableDeceleration() ) );
//     float d_s = 
//                 v_s.x * subject->getResponseTime() + 
//                 ( v_s.x * v_r.x ) / ( 2 * sqrt( subject->getMaxAcceleration() * subject->getComfortableDeceleration() ) );

//     d_o = MAX( 0 , d_o);
//     d_s = MAX( 0 , d_s);

    
//     //float d_ex_s = subjectController->getExtendedGap( object );
//     //float d_ex_o = objectController->getExtendedGap( subject );
//     float p_r_s = p_r.x + v_o.x * t_y;
//     float p_r_o = p_r.x - v_s.x * t_y;
//     float p_r_t = p_r.x - v_r.x * t_y ;
//     float d_x = ABS( p_r_t );
//     //p_r_t *= ( d_x - hl_s - hl_o ) / d_x;

//     if( ( p_r_t < 0 && p_r_t + hl_s + hl_o < -d_o * ACEPTED_RATIO ) || (  p_r_t >= 0 && p_r_t - hl_s - hl_o > d_s * ACEPTED_RATIO ) )
//         return true;

//     //MTS::SystemLog << "V" << subject->getID() << " and V" << object->getID() << ": Danger\n";
//     //MTS::SystemLog << "dis= " << p_r_t << ", disS= " << d_s << ", disO= " << d_o << ", t= " << t_y << "t_lat= " << moveTime << '\n'; 
//     //MTS::SystemLog << "v_lat= " << subject->getLateralSpeed() << ", v_long= " << subject->getCurrentSpeed()<<'\n';
    
//     return false;
// }




// intersection update