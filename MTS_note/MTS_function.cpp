// ////////////////////////////
// /////Localization  Part/////
// ////////////////////////////

// /***********************
// ******Main Calller******
// ***********************/

// //MTS.Vehicle.cpp
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

// /*************************
//  *****Update Leader*******
// *************************/

// //MTS_VehicleController.cpp
// void MTS_VehicleController::updateLeader()
// {
//     mSubject->mSituationData->mNeighbor->mPassedLeadingVehicle = NULL;
//     mSubject->mSituationData->mNeighbor->mExpectedLeadingVehicle = NULL;
//     mSubject->mBlockage = NULL;

//     float subjectOffset = getOffset() + getLength() / 2.0f;
//     mSubject->getCurrentRoad()->collectLeadingVehicle( mSubject , subjectOffset , mSubject->mSituationData->mNeighbor->mPassedLeadingVehicle , mSubject->mSituationData->mNeighbor->mExpectedLeadingVehicle );
// }

// //MTS_Lane.cpp
// void MTS_Lane::collectLeadingVehicle( MTS_Vehicle *subject , 
//                                     float subjectOffset ,
//                                     MTS_Vehicle* &passedPreVeh , 
//                                     MTS_Vehicle* &expectedPreVeh )
// {
//     // Find leading vehicles: one is able to be overtaked, one is expected to leading vehicle after overtaking
//     MTS_VehicleController *vehController =  subject->getCurrentController();
//     MTS_Lane *nextLane = subject->getNextLane();
//     MTS_Edge *nextEdge = mEdge;
//     if( nextLane ) nextEdge = nextLane->getEdge();

//     // if the subject is the begin of this lane, it means that the leading vehicle may be the next lane 
//     // which the subject will enter
    
//     //const float LOOK_DIS = 1500.0f;
//     float minOffset = subjectOffset;
//     float maxOffset = minOffset + subject->getMaxObservingDistance();	
    
//     float passOffset = FLT_MAX;
//     float expectOffset = FLT_MAX;
//     float blockageOffset = FLT_MAX;
//     MTS_Vehicle *blockage = subject->getBlockage();
//     if( blockage && blockage->getCurrentController()->onEdge( mEdge )  ) 
//         blockageOffset = blockage->getCurrentController()->getOffset() - 
//                                         blockage->getCurrentController()->getLength() / 2.0f;

//     //**************Section 1 - Find Leaders in coming queue *************//
//     //預期進入本車道的車輛
//     /*if( ( minOffset < 0 || maxOffset < 0 ) && !subject->getTransitController()->idle() )
//         collectLeadingVehicleInComingQueue( subject , subjectOffset , passedPreVeh , expectedPreVeh );*/
//     if( vehController->onEdge( mEdge ) && ( minOffset < 0 || maxOffset < 0 ) )
//         collectLeadingVehicleInComingQueue( subject , subjectOffset , passedPreVeh , expectedPreVeh );
//     bool passFound = passedPreVeh != NULL;
//     bool expectFound = expectedPreVeh != NULL;

//     if( expectFound ) return;

//     //**************Section 2 - Find Leaders on this edge *************//
//     //目前在車道上的車輛
//     // First, collect vehicles in the block between minOffset and maxOffset.
//     std::vector< MTS_Vehicle* > vehInBlock;
//     getVehicleInBlock( minOffset , maxOffset , vehInBlock );
//     MTS_Lane *leftLane = getLeftLane();
//     MTS_Lane *rightLane = getRightLane();

//     if(leftLane) 
//         leftLane->getVehicleInBlock( minOffset , maxOffset , vehInBlock );
//     if(rightLane) 
//         rightLane->getVehicleInBlock( minOffset , maxOffset , vehInBlock );

//     // Second, check if the vehicle is overlapped with the subject vehicle and the offset is closer to vehOffset
//     int vehSize = vehInBlock.size();

//     for( int i=0 ; i<vehSize ; ++i )
//     {
//         if( vehInBlock[i] == subject )
//             continue;
//         MTS_VehicleController *controller = vehInBlock[i]->getCurrentController();
//         const MTS_Edge *localLast = controller->bindEdge( mEdge ); // bind edge to the current edge for consistent coordinate system
        
//         float len = controller->getLength();
//         float offset = controller->getOffset();
//         float candidateOffset = offset - len / 2.0f;

//         bool blocked ;
//         //FindSpaceMode -> 1:Lane-based 3:Probability
//         if( mMTS->mFindSpaceMode == 1  || mMTS->mFindSpaceMode == 3 ){ blocked = ( vehInBlock[i]->getLane() == subject->getLane() );}
//         else {blocked = vehController->isOverlapped( vehInBlock[i] );}

//         float extendedGap = vehController->getExtendedGap( vehInBlock[i] );
//         if( !passFound && candidateOffset < passOffset && candidateOffset + extendedGap > subjectOffset &&
//             blocked )// Situation 1: the is closer than current passed leading vehicle
//         {
//             //this part need to consider the none-lane case, need to consider the lateral information(remove the)

//             expectedPreVeh = passedPreVeh;
//             passedPreVeh = vehInBlock[i];

//             expectOffset = passOffset;
//             passOffset = candidateOffset;
//         }
//         else if( !expectFound && candidateOffset < expectOffset && candidateOffset > subjectOffset && 
//                         blocked ) // Situation 2: the is closer than current expected leading vehicle
//         {
//             expectedPreVeh = vehInBlock[i];
//             expectOffset = candidateOffset;
//         }
//         if( blocked && vehInBlock[i]->broken()  && 
//             candidateOffset < blockageOffset )
//         {
//             subject->setBlockage( vehInBlock[i] );
//             blockageOffset = candidateOffset;
//         }

//         controller->bindEdge( localLast ); // unbind the edge
//     }

    
//     expectFound = expectedPreVeh != NULL;
//     //**************Section 3 - Find Leaders on next edge *************//
    
//     MTS_Lane* tempNextLane = subject->findNextLane();
//     if( tempNextLane != NULL && tempNextLane != this && !expectFound)
//     {
//         float temp_offset = subject->getOffset();
//         float temp_lateralOffset = subject->getLateralOffset();
//         tempNextLane->positionTranslate(this,temp_offset,temp_lateralOffset); 
//         float subjectOffsetOnNextEdge = temp_offset+ vehController->getLength() / 2.0f;
//         tempNextLane->collectLeadingVehicle( subject , subjectOffsetOnNextEdge , passedPreVeh , expectedPreVeh );
//     }

// }

// //MTS_Lane.cpp
// void MTS_Lane::getVehicleInBlock( float minOffset , float maxOffset , std::vector<MTS_Vehicle*> &result ) const
// {
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
//     //	else if( block_middle-vehOffset > MAXLEN + block_len )
//     //		break;
        
//     }
// }

// //MTS_VehicleController.cpp
// bool MTS_VehicleController::isOverlapped( MTS_Vehicle *veh ) const
// {
//     //don't need to worry about that while go straight forward.
//     //float test = mSubject->getOrientation().dotProduct( veh->getOrientation() );
//     //if( test  >= 0.8f ) 
//     //	return mSubject->getLane()->isLateralOverlapping( mSubject , veh );

//     float myWidth = mSubject->mType->getStaticWidth();
//     float vehWidth = veh->getCurrentController()->getPsychoWidth( mSubject->getCurrentSpeed() );
//     float width_sum =  vehWidth + myWidth;
//     //float myOffset_lat = getLateralOffset();

//     //float vehOffset_lat = veh->getCurrentController()->getLateralOffset();
//     //float x_dis = ABS( myOffset_lat - vehOffset_lat );

//     float x_dis = mSubject->getLateralSeparation( veh );
//     float x_dis_abs = ABS( x_dis );
//     if( 2.0f * x_dis_abs >= width_sum )
//     {
//         //if(  ABS(mSubject->getLateralSeparation( veh , 0.25f)) < width_sum/2.0f )
//         //	return true;
//         return false;
//     }
//     return true;
// }

// //MTS_VehicleController.cpp
// float MTS_VehicleController::getExtendedGap( const MTS_Vehicle *pred ) const
// {
//     MTS_VehicleController *controller = pred->getCurrentController();

//     if( controller->getYawAngle() == 0.0f )
//         return 0.0f;

//     float sepLatOffset = controller->getSeparationLateralOffset();
//     float myLatOffset = getLateralOffset();
    
//     /*
//     //float myLatOffset = getHeadLateralOffset();
//     MTS_Lane *nextLane = mSubject->getNextLane();
//     if( nextLane && controller->getLane()->getEdge() == nextLane->getEdge() )
//         myLatOffset = getLateralOffsetOnNextEdge();
//     */

//     float predHalfWidth = pred->getCurrentController()->getPsychoWidth( mSubject->getCurrentSpeed() ) / 2.0f;
//     float myHalfWidth = mSubject->mType->getStaticWidth() / 2.0f;
//     float latSeparation = mSubject->getLateralSeparation( pred );
//     latSeparation = ABS( latSeparation );
//     if( latSeparation > predHalfWidth + myHalfWidth )
//         return 0.0f;

//     float myLeftLatOffset = myLatOffset - myHalfWidth;
//     float myRightLatOffset = myLatOffset + myHalfWidth;
//     float extendedGap = 0.0f;
    
//     if( myLeftLatOffset > sepLatOffset )
//         extendedGap = controller->getExtendedDistance( myLeftLatOffset );
//     else if( myRightLatOffset < sepLatOffset )
//         extendedGap = controller->getExtendedDistance( myRightLatOffset );

//     return extendedGap;
// }

// //MTS_VehicleController.cpp
// float MTS_VehicleController::getPsychoWidth( float observerSpeed ) const
// {
//     /*
//     float vehicleSpeed = this->getCurrentSpeed();
//     const float minDis= mSubject->mAgentData->mDriver->mMinPsychoWidth;
//     if( observerSpeed-vehicleSpeed < 0.0f ) return minDis*2;

//     float w = 0.5f*getWidth()*powf( (observerSpeed-vehicleSpeed)*mSubject->mAgentData->mDriver->mPsychoWidthRatio,mSubject->mMTS->mPsychoWidthExp);
    

//     return (w + minDis)*2;*/
//     return getWidth() +mSubject->mAgentData->mDriver->mMinPsychoWidth;
// }

// //MTS_Vehicle.cpp
// float MTS_Vehicle::getLateralSeparation( const MTS_Vehicle *veh ,float time ) const
// {
//     // the function is to compute the relative offset from veh to this
//     // the relative offset is cross-edge

//     // A stands for the object vehicle
//     MTS_VehicleController *controllerA = veh->getCurrentController();
//     MTS_Lane *laneA = controllerA->getLane();
//     MTS_Edge *edgeA = laneA->getEdge();	
//     MTS_Lane *nextLaneA = veh->getNextLane();
//     MTS_Edge *nextEdgeA = NULL;
//     if( nextLaneA ) nextEdgeA = nextLaneA->getEdge();

//     // B stands for the subject vehicle
//     MTS_VehicleController *controllerB = mCurrentController;
//     MTS_Lane *laneB = controllerB->getLane();
//     MTS_Edge *edgeB = laneB->getEdge();	
//     MTS_Lane *nextLaneB = getNextLane();
//     MTS_Edge *nextEdgeB = NULL;
//     if( nextLaneB ) nextEdgeB = nextLaneB->getEdge();

//     float offsetA, offsetB;
    
//     offsetA = controllerA->getLateralOffset() + time*veh->getLateralSpeed();
//     offsetB = controllerB->getLateralOffset() + time*getLateralSpeed();

//     if( edgeA != edgeB )
//     {
//         if( nextEdgeB == edgeA )
//         {
            
//         }
//         else if( nextEdgeA == edgeB )
//         {
            
//         }
            
//     }
//     return offsetB - offsetA;
// }

// //MTS_VehilcleController.cpp
// float MTS_VehicleController::getExtendedDistance( float lateralOffset ) const
// {
//     float dis = 0.0f;
//     if( lateralOffset > mGapVariable.SeparationLateralOffset )
//     {
//         float ratio = ( lateralOffset - mGapVariable.SeparationLateralOffset ) / mGapVariable.RightWidth;
//         ratio = MIN( 1.0f , ratio );
//         dis = mGapVariable.MaxRightGap * ratio;
//     }
//     else
//     {
//         float ratio = ( mGapVariable.SeparationLateralOffset  -  lateralOffset ) / mGapVariable.LeftWidth;
//         ratio = MIN( 1.0f , ratio );
//         dis = mGapVariable.MaxLeftGap * ratio;
//     }
//     return dis;
// }


// /*************************
//  *****Update Neighbor*****
// *************************/

// //MTS_SituationData.cpp
// void MTS_SituationData::updateNeighbor()
// {

//     //�ثe�]�p���h�����o�Ӱʧ@�|�۰ʧ�s
//     //�p�G��time step�w�g��s�L�h�۰ʦ^��
//     mSubject->getLeftFrontVehicles();
//     mSubject->getLeftRearVehicles();
//     mSubject->getRightFrontVehicles();
//     mSubject->getRightRearVehicles();

// }

// //MTS_Vehicle.cpp
// const std::vector< MTS_Vehicle* >& MTS_Vehicle::getLeftFrontVehicles()
// {
//     if( mSituationData->mNeighbor->mLeftFrontVehiclesUpdated )
//         return mSituationData->mNeighbor->mLeftFrontVehicles;

//     mSituationData->mNeighbor->mLeftFrontVehiclesUpdated = true;
//     mSituationData->mNeighbor->mLeftFrontVehicles.clear();
//     getLeftFrontVehicles( 2000.0f , mSituationData->mNeighbor->mLeftFrontVehicles );
//     return mSituationData->mNeighbor->mLeftFrontVehicles;
// }

// //MTS_Vehicle.cpp
// const std::vector< MTS_Vehicle* >& MTS_Vehicle::getLeftRearVehicles( )
// {
//     if( mSituationData->mNeighbor->mLeftRearVehiclesUpdated )
//         return mSituationData->mNeighbor->mLeftRearVehicles;
//     mSituationData->mNeighbor->mLeftRearVehiclesUpdated = true;
//     mSituationData->mNeighbor->mLeftRearVehicles.clear();
//     getLeftRearVehicles( 2000.0f , mSituationData->mNeighbor->mLeftRearVehicles );
//     return mSituationData->mNeighbor->mLeftRearVehicles;
// }

// //MTS_Vehicle.cpp
// const std::vector< MTS_Vehicle* >& MTS_Vehicle::getRightFrontVehicles()
// {
//     if( mSituationData->mNeighbor->mRightFrontVehiclesUpdated )
//         return mSituationData->mNeighbor->mRightFrontVehicles;
//     mSituationData->mNeighbor->mRightFrontVehiclesUpdated = true;
//     mSituationData->mNeighbor->mRightFrontVehicles.clear();
//     getRightFrontVehicles( 2000.0f , mSituationData->mNeighbor->mRightFrontVehicles );
//     return mSituationData->mNeighbor->mRightFrontVehicles;
// }

// //MTS_Vehicle.cpp
// const std::vector< MTS_Vehicle* >& MTS_Vehicle::getRightRearVehicles()
// {
//     if( mSituationData->mNeighbor->mRightRearVehiclesUpdated )
//         return mSituationData->mNeighbor->mRightRearVehicles;
//     mSituationData->mNeighbor->mRightRearVehiclesUpdated = true;
//     mSituationData->mNeighbor->mRightRearVehicles.clear();
//     getRightRearVehicles( 2000.0f , mSituationData->mNeighbor->mRightRearVehicles );
//     return mSituationData->mNeighbor->mRightRearVehicles;
// }


// /*************************
//  *****Update Reigion******
// *************************/

// //MTS_SituationData.cpp
// void MTS_SituationData::updateRegion()
// {
    
    
//     for( int i=0;i<mRegion.size();++i )
//     {
//         mRegion[i].frontVehicles.clear();
//         mRegion[i].rearVehicles.clear();
//     }mRegion.clear();

//     mSpaceOriented = false;
//     MTS_Vehicle *pred = mSubject->getPassedLeadingVehicle();
    
    

//     if( mSubject->getVehicleType()->getTypeCode() != 2 )
//     {
//         _updateBaseRegion( mSubject , mSubject->getLane() );
//         _findLane( mSubject, mRegion );
//     }
//     else
//     {
//         if( mMTS->mFindSpaceMode == 0)
//         {
//             _updateBaseRegion( mSubject , mSubject->getLeftVehicle() , mSubject->getRightVehicle() );
//             _findSpace( mSubject , pred , mRegion );
//         }
//         if( mMTS->mFindSpaceMode == 1)
//         {
//             _updateBaseRegion( mSubject , mSubject->getLane() );
//             _findLane( mSubject, mRegion );
//         }
//         if( mMTS->mFindSpaceMode == 2)
//         {
//             float lateralMovingForce = getLateralMovingForce()*3.0f;

//             lateralMovingForce = 3.0f;
//             float randValue = (rand()%100000) / 100000.0f;
//             if( randValue < lateralMovingForce && !this->mSubject->needTocutIn)
//             {
//                 _updateBaseRegion( mSubject , mSubject->getLeftVehicle() , mSubject->getRightVehicle() );
//                 _findSpace( mSubject , pred , mRegion );

//             }
//             else
//             {
//                 _updateBaseRegion( mSubject , mSubject->getLane() );
//                 _findLane( mSubject, mRegion );
//             }
//         }
//         if( mMTS->mFindSpaceMode == 3 )
//         {
//             _updateBaseRegion( mSubject , mSubject->getLane() );
//             _findLaneByPMatrix();
//         }

//     }
        
//     mLane = mSubject->getLane();
//     mEdge = mSubject->getLane()->getEdge();
// }

// //MTS_SituationData.cpp
// void MTS_SituationData::_updateBaseRegion( const MTS_Vehicle *veh , const MTS_Lane *lane )
// {
//     MTS_Vehicle *pred = veh->getPassedLeadingVehicle();
//     MTS_VehicleController *controller =  veh->getCurrentController();
//     float halfLaneWidth = lane->getWidth()/2.0f;

//     float gap = controller->getGapToStopLine();
//     if( pred != NULL )
//         gap = controller->getGap( pred );
//     mCurrentRegion.gap = gap;

//     mCurrentRegion.offset = lane->getCentralOffset();
//     mCurrentRegion.leftBorder = mCurrentRegion.offset - halfLaneWidth;
//     mCurrentRegion.rightBorder = mCurrentRegion.offset + halfLaneWidth;
//     mCurrentRegion.width = 2.0f * halfLaneWidth;

//     float maxSpeed = controller->getDesiredSpeed();
//     maxSpeed = MIN( lane->getMaxPassingSpeed() , maxSpeed );
//     if( pred ) maxSpeed = MIN( pred->getCurrentSpeed() , maxSpeed );
//     mCurrentRegion.maxPassingSpeed = maxSpeed;
// }

// //MTS_SituationData.cpp
// void MTS_SituationData::_updateBaseRegion( const MTS_Vehicle *veh , const MTS_Vehicle *leftVeh , const MTS_Vehicle *rightVeh )
// {
//     MTS_Vehicle *pred = veh->getPassedLeadingVehicle();
//     MTS_Lane *lane = veh->getLane();
//     MTS_Edge *edge = lane->getEdge();

//     float leftOffset, rightOffset;
//     MTS_VehicleController *controller = veh->getCurrentController();
//     float gap = controller->getGapToStopLine();
//     if( pred != NULL )
//         gap = controller->getGap( pred );

//     mCurrentRegion.gap = gap;

//     if( leftVeh == NULL )
//         leftOffset = edge->getMinOffset() ;
//     else
//         leftOffset = leftVeh->getLateralOffset() + leftVeh->getVehicleController()->getPsychoWidth( mSubject->getCurrentSpeed() )/2.0f;

//     if( rightVeh == NULL )
//         rightOffset =edge->getMaxOffset() ;
//     else
//         rightOffset = rightVeh->getLateralOffset() - rightVeh->getVehicleController()->getPsychoWidth( mSubject->getCurrentSpeed() )/2.0f;

//     mCurrentRegion.leftBorder = leftOffset;
//     mCurrentRegion.rightBorder = rightOffset;

//     mCurrentRegion.width = rightOffset - leftOffset;
    
//     float vehWidth = veh->getVehicleController()->getWidth();
//     float vehStaticWidth = veh->getVehicleType()->getStaticWidth();
//     float halfVehStaticWidth = vehStaticWidth / 2.0f;

//     mCurrentRegion.offset = veh->getLateralOffset();

//     float maxSpeed = controller->getDesiredSpeed();
//     maxSpeed = MIN( lane->getMaxPassingSpeed() , maxSpeed );
//     if( pred ) maxSpeed = MIN( pred->getCurrentSpeed() , maxSpeed );
//     mCurrentRegion.maxPassingSpeed = maxSpeed;
// }

// //MTS_SituationData.cpp
// void MTS_SituationData::_findLane( const MTS_Vehicle *veh , std::vector< MTS_Region > &result )
// {
//     MTS_Region spaceData;
//     std::vector< MTS_Region > candidateSpace;

//     MTS_Lane* leftLane = veh->getLane()->getLeftLane();
//     MTS_Lane* rightLane = veh->getLane()->getRightLane();

//     result.push_back( mCurrentRegion );

//     spaceData = _setRegionParameter( veh , veh->getLane() );
//     result.push_back( spaceData );
//     if( leftLane )
//     {
//         spaceData = _setRegionParameter( veh , leftLane );
//         result.push_back( spaceData );
//     }
    
//     if( rightLane )
//     {
//         spaceData = _setRegionParameter( veh , rightLane );
//         result.push_back( spaceData );
//     }

// }

// //MTS_SituationData.cpp
// void MTS_SituationData::_findSpace(  MTS_Vehicle *veh , MTS_Vehicle *pred , 
// 									   std::vector< MTS_Region > &result )
// {
//     std::vector<MTS_Region> candidateSpace;
//     MTS_Region spaceData;
//     MTS_Region spaceData2;
//     float vehLatOffset = veh->getLateralOffset();

//     // compute the width of the space, the preferred lateral offset, and the maximum passing speed

//     result.push_back( mCurrentRegion );

    
//     MTS_Vehicle* leftVehicle = veh->getLeftVehicle();
//     MTS_Vehicle* rightVehicle = veh->getRightVehicle();
//     MTS_Vehicle* pre_leftVehicle = NULL;
//     MTS_Vehicle* pre_rightVehicle = NULL;
//     if( pred != NULL )
//     {
//         pre_leftVehicle = pred->getLeftVehicle();
//         pre_rightVehicle = pred->getRightVehicle();
//     }
//     else return;


//     spaceData = _setRegionParameter( veh , leftVehicle ,1.0f, pred ,-1.0f, DesiredDirection::DIR_NONE );
//     result.push_back( spaceData );

//     if( veh->getLeftVehicle() == NULL )
//     {
//         spaceData2 = _setRegionParameter( veh , pre_leftVehicle ,1.0f, pred ,-1.0f, DesiredDirection::DIR_NONE );
//         result.push_back( spaceData2 );
//     }
//     else
//     {
//         float sl_offset = leftVehicle->getLateralOffset() + leftVehicle->getVehicleType()->getDynamicWidth(  leftVehicle->getCurrentController()->getYawAngle() )/2.0f ;
//         float ol_offset = -1.0f;
//         if( pred!=NULL )
//         {
//             ol_offset = pred->getLateralOffset() -  pred->getVehicleType()->getDynamicWidth( pred->getCurrentController()->getYawAngle() )/2.0f;
//         }
        
//         if( sl_offset > ol_offset )
//         {
//             spaceData2 = _setRegionParameter( veh , pre_leftVehicle ,1.0f, pred ,-1.0f, DesiredDirection::DIR_NONE );
//             result.push_back( spaceData2 );
//         }		
//     }

    
// /*
//     if( spaceData.width <= spaceData2.width )
//         result.push_back( spaceData );
//     else
//         result.push_back( spaceData2 );
// */
    

//     spaceData = _setRegionParameter( veh , pred ,1.0f, rightVehicle ,-1.0f, DesiredDirection::DIR_NONE );
//     result.push_back( spaceData );

//     if( veh->getRightVehicle() == NULL )
//     {
//         spaceData2 = _setRegionParameter( veh , pred ,1.0f, pre_rightVehicle ,-1.0f, DesiredDirection::DIR_NONE );
//         result.push_back( spaceData2 );
//     }
//     else
//     {
//         float sr_offset = rightVehicle->getLateralOffset() - rightVehicle->getVehicleType()->getDynamicWidth(  rightVehicle->getCurrentController()->getYawAngle() )/2.0f ;
//         float or_offset = FLT_MAX;
//         if( pred!=NULL )
//         {
//             or_offset = pred->getLateralOffset() +  pred->getVehicleType()->getDynamicWidth( pred->getCurrentController()->getYawAngle() )/2.0f;
//         }
//         if( sr_offset < or_offset )
//         {
//             spaceData2 = _setRegionParameter( veh , pred ,1.0f, pre_rightVehicle ,-1.0f, DesiredDirection::DIR_NONE );
//             result.push_back( spaceData2 );
//         }
//     }

    
// /*
//     if( spaceData.width <= spaceData2.width )
//         result.push_back( spaceData );
//     else
//         result.push_back( spaceData2 );
// */
      
// }

// //MTS_SituationData.cpp
// MTS_Region MTS_SituationData::_setRegionParameter( const MTS_Vehicle *veh , const MTS_Vehicle *leftVeh , float leftSign , const MTS_Vehicle *rightVeh , float rightSign , DesiredDirection preferedDirection )
// {
//     float leftOffset, rightOffset;
//     MTS_Vehicle *passedVeh = veh->getPassedLeadingVehicle();

//     MTS_Edge *edge = veh->getLane()->getEdge();

//     MTS_Region resultSpace;

    
    
//     if( leftVeh == NULL )
//         leftOffset = edge->getMinOffset();
//     else{
//         float leftVehWidth = leftVeh->getVehicleType()->getDynamicWidth( leftVeh->getYawAngle() );
//         leftOffset = leftVeh->getLateralOffset() + leftSign*( leftVeh->getVehicleController()->getPsychoWidth( mSubject->getCurrentSpeed() ) + leftVehWidth )/2.0f;
//     }
//     if( rightVeh == NULL )
//         rightOffset = edge->getMaxOffset();
//     else{
//         float rightVehWidth = rightVeh->getVehicleType()->getDynamicWidth( rightVeh->getYawAngle() );
//         rightOffset = rightVeh->getLateralOffset() + rightSign*(rightVeh->getVehicleController()->getPsychoWidth( mSubject->getCurrentSpeed() ) + rightVehWidth)/2.0f;
//     }
//     resultSpace.leftBorderVehicle = (MTS_Vehicle*)leftVeh;
//     resultSpace.rightBorderVehicle = (MTS_Vehicle*)rightVeh;
//     resultSpace.leftBorder = leftOffset  ;
//     resultSpace.rightBorder = rightOffset ;

//     resultSpace.width = resultSpace.rightBorder - resultSpace.leftBorder;
    
//     MTS_Lane *currentLane = veh->getLane();
    
//     float vehWidth = veh->getVehicleController()->getWidth();
//     float vehStaticWidth = veh->getVehicleType()->getStaticWidth();
//     float halfVehStaticWidth = vehStaticWidth / 2.0f;
//     float halfLaneWidth = currentLane->getWidth() / 2.0f;

//     float offset_center = ( leftOffset + rightOffset ) / 2.0f;
//     float offset_left = leftOffset + halfLaneWidth;
//     float offset_right = rightOffset - halfLaneWidth;
//     float vehOffset = veh->getLateralOffset();

//     float dis_left = ABS( (offset_left-vehOffset) );
//     float dis_right = ABS( (offset_right-vehOffset) );

//     // prefered direction instruct which direction the subject want to move
    
//     if( resultSpace.width <= 2*halfLaneWidth )
//         resultSpace.offset = offset_center;
//     else if( dis_left < dis_right )
//         resultSpace.offset = offset_left;
//     else
//         resultSpace.offset = offset_right;
    
//     float responseTime = veh->getResponseTime();
    
//     int desiredLaneIdx = edge->getLaneID( resultSpace.offset );
//     MTS_Lane *desiredLane = edge->getLane( desiredLaneIdx );

//     //resultSpace.maxPassingSpeed = (resultSpace.width - vehStaticWidth) * veh->getDriverErrorConstant() / responseTime;
//     _findGapAndSpeed( veh , resultSpace.leftBorder , resultSpace.rightBorder ,resultSpace);
//     resultSpace.maxPassingSpeed = MIN( veh->getDesiredSpeed() , resultSpace.maxPassingSpeed );
//     resultSpace.maxPassingSpeed = MIN( desiredLane->getMaxPassingSpeed() , resultSpace.maxPassingSpeed );

    

//     /*
//     float dis = ABS( resultSpace.offset - veh->getLateralOffset() );
//     if( dis > resultSpace.width / 2.0f && resultSpace.maxPassingSpeed > 0 && MTS::VehicleSelect->isSubjectVehicle(veh) )
//         int a=0;*/
//     resultSpace.safety = 0.0;
//     return resultSpace;
// }

// //MTS_SituationData.cpp
// void MTS_SituationData::_findGapAndSpeed( const MTS_Vehicle *veh , float minLateralBorder , float maxLateralBorder , MTS_Region &region)
// {
//     /*
//     int neighborSize = mNeighbor->mLeftFrontVehicles.size();
//     float maxGap = -10.0f;
//     float maxSpeed = -10.0f;
//     float vehHeadOffset = veh->getOffset() + veh->getVehicleType()->getDynamicLength( veh->getCurrentController()->getYawAngle() )/2.0f;
//     for( int i=0;i<neighborSize;++i )
//     {
//         MTS_Vehicle* object = mNeighbor->mLeftFrontVehicles[i];
//         float dynamicWidth = object->getVehicleType()->getDynamicWidth( object->getCurrentController()->getYawAngle() );
//         float object_lateral_min = object->getLateralOffset() - dynamicWidth/2.0f;
//         float object_lateral_max = object->getLateralOffset() + dynamicWidth/2.0f;
        
        
//         if( (object_lateral_max < maxLateralBorder && object_lateral_max > minLateralBorder) ||
//             (object_lateral_min < maxLateralBorder && object_lateral_min > minLateralBorder)
//             )
//         {
//             float gap = vehHeadOffset- (object->getOffset() - object->getVehicleType()->getDynamicLength( object->getCurrentController()->getYawAngle() )/2.0f) ;
//             if( gap > maxGap ){ maxGap = gap;}
//         }

//     }
    
//     neighborSize = mNeighbor->mRightFrontVehicles.size();
//     for( int i=0;i<neighborSize;++i )
//     {
//         MTS_Vehicle* object = mNeighbor->mRightFrontVehicles[i];
//         float dynamicWidth = object->getVehicleType()->getDynamicWidth( object->getCurrentController()->getYawAngle() );
//         float object_lateral_min = object->getLateralOffset() - dynamicWidth/2.0f;
//         float object_lateral_max = object->getLateralOffset() + dynamicWidth/2.0f;
        
        
//         if( object_lateral_max < maxLateralBorder && object_lateral_min > minLateralBorder &&
//             object_lateral_min < maxLateralBorder && object_lateral_max > minLateralBorder
//             )
//         {
//             float gap = vehHeadOffset- (object->getOffset() - object->getVehicleType()->getDynamicLength( object->getCurrentController()->getYawAngle() )/2.0f) ;
//             if( gap > maxGap ) maxGap = gap;
//         }

//     }

//     if( maxGap <= -10.0f ){ maxGap = veh->getLane()->getLength(); }
//     return maxGap;
// } */

//     float vehHead = veh->getOffset() + veh->getVehicleType()->getDynamicLength( veh->getYawAngle() ) / 2.0f;
//     float vehSpeed = veh->getCurrentSpeed();

//     MTS_Lane* currentLane = veh->getLane();
//     MTS_Edge* edge = currentLane->getEdge();
//     int laneSize = edge->getLaneSize();
//     float currentMin = currentLane->getLength()-veh->getOffset()-veh->getVehicleType()->getStaticLength();
//     MTS_Vehicle* minObject  = NULL;
//     for( int i=0;i<laneSize;++i )
//     {
//         MTS_Lane* lane = edge->getLane(i);
//         float gap ;
//         MTS_Vehicle* object = lane->getVehCloseAndBiger( minLateralBorder,maxLateralBorder,vehHead,gap, vehSpeed);
//         if( object!=NULL && gap<currentMin  ) currentMin = gap;

//     }
//     region.gap = currentMin;
//     if( minObject!= NULL )
//     {
//         region.maxPassingSpeed = minObject->getCurrentSpeed();
//     }
//     else
//     {
//         region.maxPassingSpeed = veh->getLane()->getMaxPassingSpeed();
//     }

// }

// /*************************
//  *****Evalute Safety******
// *************************/

// //MTS_SituationData.cpp
// void MTS_SituationData::evaluateSafety()
// {

//     MTS_VehicleController *subjectController = mSubject->getCurrentController();
//     float mDesiredLateralOffset = subjectController->getDesiredLateralOffset();

//     float x_current = subjectController->getLateralOffset();
//     float v_current = subjectController->getLateralSpeed();
//     float x_safe;
//     mCollisionTime = 2.0f;
//     float t = 3.0f;
    
//     if( v_current > 0 )
//     {
//         bool safe = _checkRightCollision( v_current , t , &x_safe );
//         // ��ڤW�o�̷|��car following ���ͼv�T( neighbor )
//         //_buildInvolvedVehicles( frontVehicles , rearVehicles );
//         if( !safe )
//         {
//             mDesiredLateralOffset = MIN(  mDesiredLateralOffset , x_safe );
//             safety = 0.0f;
//             return ;
//             //return true;
//         }
//     }
//     else if( v_current < 0 )
//     {
//         bool safe = _checkLeftCollision( v_current , t , &x_safe );
//         //_buildInvolvedVehicles( frontVehicles , rearVehicles );
//         if( !safe )
//         {
//             mDesiredLateralOffset = MAX(  mDesiredLateralOffset , x_safe );
//             safety = 0.0f;
//             return ;
//             //return true;
//         }
//     }

//     safety = 1.0f;
//     //return false;	

// }

// //MTS_SituationData.cpp
// bool MTS_SituationData::_checkRightCollision( float v , float t , float *safeOffset )
// {
//     //MTS::SystemLog << "Check right collision ( " << MTS::SimulationTime << " ) ****************************************\n";

//     const std::vector< MTS_Vehicle* > &rearVehicles = mSubject->getRightRearVehicles();

//     //mSubject->getRightRearVehicles( 2000.0f , rearVehicles );

//     std::vector< MTS_Vehicle* >::const_iterator it = rearVehicles.begin();
//     std::vector< MTS_Vehicle* >::const_iterator vehEnd = rearVehicles.end();

//     float safeTime;
//     float minSafeTime = t;
//     bool safe = true; 
//     float t_s = mSubject->getResponseTime();

//     /*
//     //���b�Q�ϥΪ̱���N���ޥ��F
//     if( MTS::VehicleSelect->isSubjectVehicle( mSubject ) )
//     {
//         MTS::VehicleSelect->setRightFrontVehicle( NULL );
//         MTS::VehicleSelect->setRightRearVehicle( NULL );
//     }
//     */

//     for( ; it != vehEnd ; ++it )
//     {
//         MTS_VehicleController *controller = (*it)->getCurrentController();

//         bool safetyResult = _checkSafety( mSubject , *it , v , t , true , true , &safeTime );
//         if( !safetyResult )
//         {
//             if( safeOffset && safeTime < minSafeTime )
//             {
//                 minSafeTime = safeTime;
                
//                 float currentOffset = controller->getLateralOffset();
//                 float currentSpeed = controller->getLateralSpeed();
//                 float minDis = ( controller->getPsychoWidth( mSubject->getCurrentSpeed() ) + mSubject->getVehicleType()->getStaticWidth() ) / 2.0f + 2.0f;
//                 *safeOffset = currentOffset +currentSpeed * safeTime - minDis;

//                 //MTS::SystemLog << "V" << mSubject->getID() << " and V" << (*it)->getID() << ": SafeOffset= " << *safeOffset << ", Acc= " << mSubject->getLateralSpeedChange() << '\n';
//                 /*float minDis = ( (*it)->getPsychoWidth() + mSubject->getVehicleType()->getStaticWidth() ) / 2.0f;
//                 *safeOffset = (*it)->getLateralOffset() - minDis;*/
//             }
//             safe = false;

//             /*
//             if( MTS::VehicleSelect->isSubjectVehicle( mSubject ) )
//                 MTS::VehicleSelect->setRightRearVehicle( *it );
//             */
//         }
//         /*if( safeTime >= 0 && ( safeTime < 3.0f || false ) )
//             (*it)->addNeighbor( mSubject , safeTime );*/
//     }


//     const std::vector< MTS_Vehicle* > &frontVehicles = mSubject->getRightFrontVehicles();
//     //mSubject->getRightFrontVehicles( 2000.0f , frontVehicles );
    
//     it = frontVehicles.begin();
//     vehEnd = frontVehicles.end();

//     for( ; it != vehEnd ; ++it )
//     {
//         MTS_VehicleController *controller = (*it)->getCurrentController();
//         bool safetyResult = _checkSafety( mSubject , *it , v , t , false , true , &safeTime );
//         if( !safetyResult )
//         {
//             if( safeOffset && safeTime < minSafeTime )
//             {
//                 minSafeTime = safeTime;
                
//                 float currentOffset = controller->getLateralOffset();
//                 float currentSpeed = controller->getLateralSpeed();
//                 float minDis = ( controller->getPsychoWidth( mSubject->getCurrentSpeed() ) + mSubject->getVehicleType()->getStaticWidth() ) / 2.0f + 2.0f;
//                 *safeOffset = currentOffset +currentSpeed * safeTime - minDis;

//                 //MTS::SystemLog << "V" << mSubject->getID() << " and V" << (*it)->getID() << ": SafeOffset= " << *safeOffset << ", Acc= " << mSubject->getLateralSpeedChange() << '\n';
//                 /*float minDis = ( (*it)->getPsychoWidth() + mSubject->getVehicleType()->getStaticWidth() ) / 2.0f;
//                 *safeOffset = (*it)->getLateralOffset() - minDis;*/
//             }
//             safe = false;
//             /*
//             if( MTS::VehicleSelect->isSubjectVehicle( mSubject ) )
//                 MTS::VehicleSelect->setRightFrontVehicle( *it );
//             */
//         }
//         /*if( safeTime >= 0 && ( safeTime < 3.0f || false ) )
//             mSubject->addNeighbor( *it , safeTime );*/

//     }

//     if( minSafeTime != 0.0f )
//         mCollisionTime = minSafeTime;
    
//     return safe;
// }

// //MTS_SituationData.cpp
// bool MTS_SituationData::_checkLeftCollision( float v , float t , float *safeOffset )
// {
//     const std::vector< MTS_Vehicle* > &rearVehicles = mSubject->getLeftRearVehicles();

//     std::vector< MTS_Vehicle* >::const_iterator it = rearVehicles.begin();
//     std::vector< MTS_Vehicle* >::const_iterator vehEnd = rearVehicles.end();

//     float safeTime;
//     float minSafeTime = t;
//     bool safe = true;
//     float t_s = mSubject->getResponseTime();
    
//     /*
//     if( MTS::VehicleSelect->isSubjectVehicle( mSubject ) )
//     {
//         MTS::VehicleSelect->setLeftFrontVehicle( NULL );
//         MTS::VehicleSelect->setLeftRearVehicle( NULL );
//     }
//     */

//     for( ; it != vehEnd ; ++it )
//     {
//         bool safetyResult = _checkSafety( mSubject , *it , v , t , true , true , &safeTime );
//         if( !safetyResult && mSubject->getPatience() > 0.75)
//         {
//             safetyResult = true;
//         }

//         if( !safetyResult )
//         {
//             if( safeOffset && safeTime < minSafeTime )
//             {
//                 minSafeTime = safeTime;
//                 MTS_VehicleController *controller = (*it)->getCurrentController();
//                 float currentOffset = controller->getLateralOffset();
//                 float currentSpeed = controller->getLateralSpeed();
//                 float minDis = ( controller->getPsychoWidth( mSubject->getCurrentSpeed() ) + mSubject->getVehicleType()->getStaticWidth() ) / 2.0f + 2.0f;
//                 *safeOffset = currentOffset + currentSpeed * safeTime + minDis;

//                 //MTS::SystemLog << "V" << mSubject->getID() << " and V" << (*it)->getID() << ": SafeOffset= " << *safeOffset << ", Acc= " << mSubject->getLateralSpeedChange() << '\n';
//                 /*float minDis = ( (*it)->getPsychoWidth() + mSubject->getVehicleType()->getStaticWidth() ) / 2.0f;
//                 *safeOffset = (*it)->getLateralOffset() - minDis;*/
//             }
//             safe = false;
//             /*
//             if( MTS::VehicleSelect->isSubjectVehicle( mSubject ) )
//                 MTS::VehicleSelect->setLeftRearVehicle( *it );
//             */
//         }
//         /*if( safeTime >= 0 && ( safeTime < 3.0f || false ) )
//             (*it)->addNeighbor( mSubject , safeTime );*/
//     }

//     const std::vector< MTS_Vehicle* > &frontVehicles = mSubject->getLeftFrontVehicles();
//     //mSubject->getLeftFrontVehicles( 2000.0f , frontVehicles );
    
//     it = frontVehicles.begin();
//     vehEnd = frontVehicles.end();

//     for( ; it != vehEnd ; ++it )
//     {
//         bool safetyResult = _checkSafety( mSubject , *it , v , t , false , true , &safeTime );
//         if( !safetyResult )
//         {
//             if( safeOffset && safeTime < minSafeTime )
//             {
//                 minSafeTime = safeTime;
//                 MTS_VehicleController *controller = (*it)->getCurrentController();
//                 float currentOffset = controller->getLateralOffset();
//                 float currentSpeed = controller->getLateralSpeed();
//                 float minDis = ( controller->getPsychoWidth( mSubject->getCurrentSpeed() ) + mSubject->getVehicleType()->getStaticWidth() ) / 2.0f + 2.0f;
//                 *safeOffset = currentOffset +currentSpeed * safeTime + minDis;
                
//                 //MTS::SystemLog << "V" << mSubject->getID() << " and V" << (*it)->getID() << ": SafeOffset= " << *safeOffset << ", Acc= " << mSubject->getLateralSpeedChange() << '\n';
//                 /*float minDis = ( (*it)->getPsychoWidth() + mSubject->getVehicleType()->getStaticWidth() ) / 2.0f;
//                 *safeOffset = (*it)->getLateralOffset() - minDis;*/
//             }
//             safe = false;
//             /*
//             if( MTS::VehicleSelect->isSubjectVehicle( mSubject ) )
//                 MTS::VehicleSelect->setLeftFrontVehicle( *it );
//             */
//         }
//         /*if( safeTime >= 0 && ( safeTime < 3.0f || false ) )
//             mSubject->addNeighbor( *it , safeTime );*/
//     }

//     if( minSafeTime != 0.0f )
//         mCollisionTime = minSafeTime;

//     return safe;
// }

// //MTS_SituationData.cpp
// bool MTS_SituationData::_checkSafety( MTS_Vehicle *subject , MTS_Vehicle *object , 
// 										  float moveSpeed , float moveTime , bool subjectAsLeader , bool subjectAsFollower , float* safeTime ) const
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

//     const float ACEPTED_RATIO = 0.7f;
//     if( subjectAsLeader )
//     {
//         //( *mRearVehicles ).push_back( InvolvedVehiclePair( object , t_y ) );
//         //MTS::SystemLog << "V" << subject->getID() << " and V" << object->getID() << ": Safe(leader)\n";
//         //MTS::SystemLog << "dis= " << p_r_o + hl_s + hl_o << ", disS= " << d_s << ", disO= " << d_o << ", t= " << t_y <<  "t_lat= " << moveTime << '\n'; 
//         //MTS::SystemLog << "v_lat= " << subject->getLateralSpeed() << ", v_long= " << subject->getCurrentSpeed()<<'\n';
//     }
//     else if( subjectAsFollower )
//     {
//         //( *mFrontVehicles ).push_back( InvolvedVehiclePair( object , t_y ) );
//         //MTS::SystemLog << "V" << subject->getID() << " and V" << object->getID() << ": Safe(follower)\n";
//         //MTS::SystemLog << "dis= " << p_r_s - hl_s - hl_o << ", disS= " << d_s << ", disO= " << d_o << ", t= " << t_y <<  "t_lat= " << moveTime << '\n'; 
//         //MTS::SystemLog << "v_lat= " << subject->getLateralSpeed() << ", v_long= " << subject->getCurrentSpeed()<<'\n';
//     }

//     if( ( p_r_t < 0 && p_r_t + hl_s + hl_o < -d_o * ACEPTED_RATIO ) || (  p_r_t >= 0 && p_r_t - hl_s - hl_o > d_s * ACEPTED_RATIO ) )
//         return true;

//     //MTS::SystemLog << "V" << subject->getID() << " and V" << object->getID() << ": Danger\n";
//     //MTS::SystemLog << "dis= " << p_r_t << ", disS= " << d_s << ", disO= " << d_o << ", t= " << t_y << "t_lat= " << moveTime << '\n'; 
//     //MTS::SystemLog << "v_lat= " << subject->getLateralSpeed() << ", v_long= " << subject->getCurrentSpeed()<<'\n';
    
//     return false;
// }

// ///////////////////////////
// /////Motion Plan  Part/////
// ///////////////////////////

// /***********************
// ******Main Calller******
// ***********************/

// //MTS_Vehicle.cpp
// void MTS_Vehicle::computeAcceleration()
// {
//     float speedChange =0.0f ;
//     float lateralSpeedChange =0.0f ;

//     if( !counted || ( mBoundaryControl || mRePlayRecord ) && mMTS->mVehicleLearningMode != 3 && mMTS->mVehicleLearningMode != 5 )
//     {
//         this->needTocutIn = false;
//         mCurrentController->updateLeader();
//         if( mSituationData->mNeighbor->mPassedLeadingVehicle != NULL )
//             setCurrentGap( getCurrentController()->getGap( mSituationData->mNeighbor->mPassedLeadingVehicle) );
//         return;
//     }

    
//     ////////////////////////////////////////////
//     // if the vehicle is controlled by the user, get the acceleration from vehicle selecter
//     if( isUserContorl )
//     {
//         mCurrentController->setSpeedChange( speedChangeUserContorl );
//         mCurrentController->setLateralSpeedChange( lateralSpeedChangeUserContorl );
//         return;
//     }
//     ///////////////////////////////////////////
    
    
//     if( mResponseTimer >= mAgentData->mDriver->mResponseTime )
//     {
//         mResponseTimer =  mResponseTimer - mAgentData->mDriver->mResponseTime;
//     }
//     else if( getPassedLeadingVehicle() != NULL && getPassedLeadingVehicle()->getLane() != NULL && getCurrentController()->getGap(getPassedLeadingVehicle()) <  (mAgentData->mDriver->mResponseTime-mResponseTimer)*this->getCurrentSpeed() || changeLane )
//     {
//         mResponseTimer = 0;
//     }
//     else
//     {
//         mResponseTimer += mMTS->mSimulationTimeStep ;
//         return;
//     }
    
//     //���}�B�z ���f���s �M ��L���p
//     if( mBehaviorController->isBehavior("Transition") )
//     {
//         /*

//         _updateTransitionSituation();
    
//         MTS_MovingModelParameter param;

//         mBehaviorController->caculateParameter(&param,mSituationData,mAgentData);
        
//         speedChange = mLongitudinalModel->getSpeedChange(&param,mSituationData,mAgentData);
        
//         mDecisionTimer = mDecisionTimer - MTS::SimulationTimeStep ;
//         if( mDecisionTimer <= 0.0f || mSituationData->safety <= 0.0f )
//         {
//             mLateralModel->updateBestLateralOffset(&param,this);
//             mDecisionTimer = mDecisionTimeStep;
//         }
//         lateralSpeedChange = mLateralModel->getSpeedChange(&param,this);

//         mCurrentController->setSpeedChange( speedChange );
//         mCurrentController->setLateralSpeedChange( lateralSpeedChange );

//         resetNeighbor();
//         */
//     }
//     else
//     {


//         _updateSituation();
    
//         MTS_MovingModelParameter param;

//         mBehaviorController->caculateParameter(&param,mSituationData,mAgentData);
        
//         speedChange = mLongitudinalModel->getSpeedChange(&param,mSituationData,mAgentData);
        
//         if( speedChange!=speedChange )
//         {
//             speedChange = 0.0f;
//         }
//         //if( speedChange < -mMTS->mStrongBraking ) speedChange = -mMTS->mStrongBraking;

//         mDecisionTimer = mDecisionTimer - mMTS->mSimulationTimeStep ;
//         if( mDecisionTimer <= 0.0f || mSituationData->safety <= 0.0f  || changeLane )
//         {

//             mLateralModel->updateBestLateralOffset(&param,this);
//             mDecisionTimer = mDecisionTimeStep;
            
//         }
//         lateralSpeedChange = mLateralModel->getSpeedChange(&param,this);
        
        


//         // handle the deccelereation from intersection .
//         if( mRouter->isInPrepareRange() )
//         {
//             //handle the first in event
//             if( mRouter->getPath() == NULL )
//             {
//                 //�إߥ��ѩΪ̲Ĥ@���i�J�A���ӭn�I�s�Ы�Path
//                 mRouter->getNextIntersection()->vehicleInPrepareArea( this );
//             }
//             else
//             {
//                 checkRebuildPath();
//                 //float pathDec = mRouter->getPath()->updateSpeedChange( mMTS->mSimulationTimeStep );
//                 //if( pathDec < speedChange ) speedChange = pathDec;
//             }
    
//         }

//         //check if the speed change is reasonable
//         /*
//         {
//             //check for the system strong break
//             float strongBreak = - mMTS->mStrongBraking;
//             if( speedChange < strongBreak  )
//             {
//                 //strong break should be record, but not nessearily being the limit. 
//                 //speedChange = strongBreak;
//             }

//             float vehicleLimit = - mAgentData->mVehicleCapability->mMaxAcceleration;
//             if( speedChange < vehicleLimit  ) speedChange = vehicleLimit;

//         }
//         */

//         if( mMTS->mVehicleLearningMode == 3)
//         {
        
//         }
//         else if( mMTS->mVehicleLearningMode == 4)
//         {
        
//         }
//         else if( mMTS->mVehicleLearningMode == 5)
//         {
//             mCurrentController->setSpeedChange( speedChange );
//             mCurrentController->setLateralSpeedChange( lateralSpeedChange );
            
//         }
//         else
//         {
//             mCurrentController->setSpeedChange( speedChange );
//             mCurrentController->setLateralSpeedChange( lateralSpeedChange );
//         }

//         //changeLane = false;
//         if( mMTS->mRecordAtion )
//         {
//             if( temp_lastAction != temp_currentAction ) temp_lastActionChangeTime = mMTS->mSimulationTime ;
//             temp_lastAction = temp_currentAction;
//             if( this->mLateralSpeed == 0.0f  )
//             {
//                 temp_currentAction = 0;
//             }
//             else if( this->mLateralSpeed < 0.0f )
//             {
//                 temp_currentAction = 1;
//             }
//             else if( this->mLateralSpeed > 0.0f )
//             {
//                 temp_currentAction = 2;
//             }			
//         }

        

//         resetNeighbor();

//     }

//     /*
//     if( this->getCurrentSpeed() == 0.0f  && this->getLateralSpeed() == 0.0f  && speedChange < 0.0f && mLateralSpeed == 0.0f)
//     {
//         float mRand ;
//         mRand = (float)(rand() % 200) /100 -1.0f ;
//         this->getAgentData()->mDriver->mResponseTime =1.5f +mRand;
//     }else
//     {
//         this->getAgentData()->mDriver->mResponseTime =0.02;
//     }*/

//     /*
//     //////////////////////////////////////////
//     // for GUI & debugMessage
//     if( MTS::VehicleSelect->isSubjectVehicle( this ) )
//     {
//         MTS::SceneMod->showDesiredOffset( this , this->getDesiredLateralOffset() );
//         MTS::mVideoPlayer->addDesiredOffset( this , this->getDesiredLateralOffset() );
//     }
//     ///////////////////////////////////////////
//     */
// }

// /*****************************
// *****Longitudinal Model*******
// *****************************/

// //MTS_LongitudinalModel.cpp
// float MTS_LongitudinalModel::getSpeedChange( MTS_MovingModelParameter* param , MTS_SituationData*	mSituationData , MTS_AgentData*	mAgentData )
// {
//     float acc = 0;

//     if( mMTS->mFollowingMode == 0 ) acc =  _MTS_getSpeedChange(param ,mSituationData,mAgentData );
//     if( mMTS->mFollowingMode == 1 ) acc =  _IDM_getSpeedChange(param ,mSituationData,mAgentData );
//     if( mMTS->mFollowingMode == 2 ) acc = _CAH_getSpeedChange(param ,mSituationData,mAgentData );

//     return acc;
// }

// //MTS_LongitudinalModel.cpp
// float MTS_LongitudinalModel::_MTS_getSpeedChange( MTS_MovingModelParameter* param , MTS_SituationData*	mSituationData , MTS_AgentData*	mAgentData) 
// {
//     float maxAcc = mAgentData->mVehicleCapability->mMaxAcceleration;
//     float currentSpeed = mSituationData->mSubject->getCurrentSpeed();

//     float acc_free = getFreeAccleration( currentSpeed , param->desiresdSpeed , maxAcc , mAgentData->mDriver->mSpeedAccelerationExponent);
//     const float MAX_DEC = mMTS->mStrongBraking;
    
//     float brake_first = _getAcceleration( param , mAgentData , mSituationData->mSubject , mSituationData->mNeighbor->mPassedLeadingVehicle , 0.0f , maxAcc );
//     if( param->ignoreLeadingVehicle ) brake_first = 0.0f;
//     //brake_first = MIN( brake_first , acc_free );
    
//     float brake_second = _getAcceleration( param , mAgentData , mSituationData->mSubject , mSituationData->mNeighbor->mExpectedLeadingVehicle , 0.0f , maxAcc );
//     //brake_second = MIN( brake_second , acc_free );

//     float maxDeceleration;
//     if( brake_first > brake_second )
//     {
//         maxDeceleration = brake_first;
        
//         mSituationData->mSubject->setCurrentGap( mSituationData->mSubject->getCurrentController()->getGap( mSituationData->mNeighbor->mPassedLeadingVehicle ) );
//     }
//     else
//     {
//         maxDeceleration = brake_second;
//         if( mSituationData->mNeighbor->mExpectedLeadingVehicle != NULL )
//             mSituationData->mSubject->setCurrentGap( mSituationData->mSubject->getCurrentController()->getGap( mSituationData->mNeighbor->mExpectedLeadingVehicle ) );
//         else mSituationData->mSubject->setCurrentGap( 0.0f);
//     }
//     //float maxDeceleration = MAX( brake_first , brake_second ); 

//     if( param->hasStopLine )
//     {
//         float brake_stopLine =  _decelerationForStopLine( param->stopLineOffset , acc_free , mSituationData ,mAgentData );

//         if( maxDeceleration < brake_stopLine )
//         {
//             mSituationData->mSubject->setCurrentGap( mSituationData->mSubject->getCurrentController()->getGapToStopLine( param->stopLineOffset )  );
//             maxDeceleration = brake_stopLine;
//         }

//     }
    
//     float acc = acc_free - maxDeceleration;
//     if( acc <= - MAX_DEC )
//     {
//         //acc = MAX_DEC;
//         //std::cout<< "Vehicle " << mSituationData->mSubject->getID() <<" strong break at  " << mMTS->mSimulationTime <<std::endl;
//     }

//     return acc;

// }

// //MTS_LongitudinalModel.cpp
// float MTS_LongitudinalModel::getFreeAccleration( float v_current , float v_desired , float a_max , float freeAccelerationExp )
// 	{
// 		if( v_desired == 0.0f ) return 0.0f; 

// 		float acc = a_max * ( 1.0 - pow( v_current/v_desired , freeAccelerationExp ) );
// 		return acc;
// 	}

// //MTS_LongitudinalModel.cpp
// float MTS_LongitudinalModel::_getAcceleration( MTS_MovingModelParameter* param , MTS_AgentData*	mAgentData , MTS_Vehicle *subject , MTS_Vehicle *object , float timeToContact , float maxDec )
// {
//     if( object == NULL ) return 0.0f;

//     MTS_VehicleController *subjectController = subject->getCurrentController();
//     MTS_VehicleController *objectController = object->getCurrentController();

//     const MTS_Edge *last = objectController->cooperate( subjectController );
//     bool cooperated = subjectController->cooperated( objectController );
//     if( !cooperated )
//     {
//         objectController->bindEdge( last );
//         return 0.0f;
//     }

//     float gap = subjectController->getGap( object );
    
//     //float b_com = param->comfortableDeceleration;
//     float a_max = param->maxAcceleration;
//     float v_s = subjectController->getCurrentSpeed();
//     float v_o = objectController->getCurrentSpeed();
//     float gap_prepare = v_o * timeToContact;

//     gap += gap_prepare;

//     bool gapExtended = false;
//     if( gap < 0.0f && timeToContact == 0.0f ) // current leader but the gap less than 0
//     {
//         // try to extend the gap according to the yaw angle of the leader
//         gapExtended = true;
//         gap += subjectController->getExtendedGap( object );
//     }

//     if( gap < 0.0f ) // the gap is less than 0 (even after extension)
//     {
//         objectController->bindEdge( last );
//         return 0.0f;
//     }

//     float s = _computeDesiredGap( param , mAgentData , subject , object );
    
//     bool approaching = v_s - v_o >= 0.0f;
//     bool closeEnough = gap <= 1.2f * s;

//     int acc_gapApproachSpeed = mAgentData->mDriver->mGapdAccelerationExponent;
//     float acc = a_max * pow( s / gap , acc_gapApproachSpeed ) * ( approaching || closeEnough );//a_max * (s*s) / (gap*gap) * ( approaching || closeEnough );

//     if( !approaching && closeEnough )
//     {
//         acc = a_max* (s/gap);
//     }

//     //float maxAcc = -v_s / mMTS->mSimulationTimeStep;

//     if( gapExtended == false && acc > maxDec )
//     {
//         float extendedGap = subjectController->getExtendedGap( object );
//         if( extendedGap != 0 )
//         {
//             float gapRatio = gap / ( gap + extendedGap );
//             acc *= gapRatio * gapRatio;
//         }
//     }
//     objectController->bindEdge( last );
//     if( acc!=acc )
//     {
//         //system("pause");
//         acc = 0.0f;
//     }
//     return acc;
// }

// //MTS_VehicleController.cpp
// float MTS_VehicleController::getGap( MTS_Vehicle *pred ) const
// {
//     float halfPredLen = pred->getCurrentController()->getLength() / 2.0f;
//     float halfVehLen = getLength() / 2.0f;

//     float relativeOffset = pred->getRelativeOffset( mSubject );
//     float gap = relativeOffset - ( halfPredLen + halfVehLen );
//     //gap += getExtendedGap( pred );
//     return gap;
// }

// //MTS_Vehicle.cpp
// float MTS_Vehicle::getRelativeOffset( const MTS_Vehicle *veh ) const
// {
//     // the function is to compute the relative offset from veh to this
//     // the relative offset is cross-edge

//     // A stands for the object vehicle
//     MTS_VehicleController *controllerA = veh->getCurrentController();
//     MTS_Lane *laneA = controllerA->getLane();
//     MTS_Edge *edgeA = laneA->getEdge();	
//     MTS_Lane *nextLaneA = veh->findNextLane();
//     MTS_Edge *nextEdgeA = NULL;
//     if( nextLaneA ) nextEdgeA = nextLaneA->getEdge();

//     // B stands for the subject vehicle
//     MTS_VehicleController *controllerB = mCurrentController;
//     MTS_Lane *laneB = controllerB->getLane();
//     MTS_Edge *edgeB = laneB->getEdge();	
//     MTS_Lane *nextLaneB = findNextLane();
//     MTS_Edge *nextEdgeB = NULL;
//     if( nextLaneB ) nextEdgeB = nextLaneB->getEdge();

//     float offsetA, offsetB;
    
//     offsetA = controllerA->getOffset();
//     offsetB = controllerB->getOffset();

//     if( edgeA != edgeB )
//     {
//         if( nextEdgeB == edgeA )
//         {
//             float lateraloffset = getLateralOffset();
//             float offset = getOffset();
//             veh->getCurrentRoad()->positionTranslate( mRoadStructure , offset , lateraloffset );
//             offsetB = offset;
//         }
//         else if( nextEdgeA == edgeB )
//         {
//             float lateraloffset = veh->getLateralOffset();
//             float offset = veh->getOffset();
        
            
//             mRoadStructure->positionTranslate( veh->getCurrentRoad() , offset , lateraloffset );
//             offsetA = offset;
            
//         }
//     }
//     return offsetB - offsetA;
// } 

// //MTS_Lane.cpp
// void MTS_Lane::positionTranslate( MTS_RoadStructure* now , float &offset , float &lateralOffset)
// {
//     if( now->getType() == RoadStructureType::LANE )
//     {
//         MTS_Lane *currentLane = (MTS_Lane *) now;

//         float laneLen = currentLane->getLength();
//         float transitDistance = 0.0f;

//         Vector3 targetPos = getPosition( lateralOffset , 0 );
//         Vector3 sourcePos = currentLane->getPosition( lateralOffset , laneLen -1.01f);
//         //Vector3 targetPos = getEdge()->getPosition(0.0f);
//         //Vector3 sourcePos = currentLane->getEdge()->getPosition( laneLen-0.01 );
//         transitDistance = sourcePos.distance( targetPos );

//         offset = -( transitDistance + laneLen - offset );
//         return;
//     }
//     else if( now->getType() == RoadStructureType::INTERSECTION )
//     {
//         MTS_Lane *currentLane = (MTS_Lane *) now;
//         float laneLen = currentLane->getLength();

//         MTS_Intersection *current = (MTS_Intersection *) now;
//         Vector3 sourcePos = current->getPosition( lateralOffset , laneLen );
//         lateralOffset = sourcePos.x;
//         offset = sourcePos.z;
//     }

// }

// //MTS_Intersection.cpp
// void MTS_Intersection::positionTranslate( MTS_RoadStructure* now , float &offset , float &lateralOffset)
// {
//     if( now->getType() == RoadStructureType::LANE )
//     {
        
//         return;
//     }
//     else if( now->getType() == RoadStructureType::INTERSECTION )
//     {
//         MTS_Lane *currentLane = (MTS_Lane *) now;
//         float laneLen = currentLane->getLength();

//         Vector3 sourcePos = currentLane->getPosition( lateralOffset , laneLen );
//         lateralOffset = sourcePos.x;
//         offset = sourcePos.z;
//     }

// }

// //MTS_VehicleController.cpp
// float MTS_VehicleController::getExtendedGap( const MTS_Vehicle *pred ) const
// {
//     MTS_VehicleController *controller = pred->getCurrentController();

//     if( controller->getYawAngle() == 0.0f )
//         return 0.0f;

//     float sepLatOffset = controller->getSeparationLateralOffset();
//     float myLatOffset = getLateralOffset();
    
//     /*
//     //float myLatOffset = getHeadLateralOffset();
//     MTS_Lane *nextLane = mSubject->getNextLane();
//     if( nextLane && controller->getLane()->getEdge() == nextLane->getEdge() )
//         myLatOffset = getLateralOffsetOnNextEdge();
//     */

//     float predHalfWidth = pred->getCurrentController()->getPsychoWidth( mSubject->getCurrentSpeed() ) / 2.0f;
//     float myHalfWidth = mSubject->mType->getStaticWidth() / 2.0f;
//     float latSeparation = mSubject->getLateralSeparation( pred );
//     latSeparation = ABS( latSeparation );
//     if( latSeparation > predHalfWidth + myHalfWidth )
//         return 0.0f;

//     float myLeftLatOffset = myLatOffset - myHalfWidth;
//     float myRightLatOffset = myLatOffset + myHalfWidth;
//     float extendedGap = 0.0f;
    
//     if( myLeftLatOffset > sepLatOffset )
//         extendedGap = controller->getExtendedDistance( myLeftLatOffset );
//     else if( myRightLatOffset < sepLatOffset )
//         extendedGap = controller->getExtendedDistance( myRightLatOffset );

//     return extendedGap;
// }

// //MTS_VehicleController.cpp
// float MTS_VehicleController::getPsychoWidth( float observerSpeed ) const
// {
//     /*
//     float vehicleSpeed = this->getCurrentSpeed();
//     const float minDis= mSubject->mAgentData->mDriver->mMinPsychoWidth;
//     if( observerSpeed-vehicleSpeed < 0.0f ) return minDis*2;

//     float w = 0.5f*getWidth()*powf( (observerSpeed-vehicleSpeed)*mSubject->mAgentData->mDriver->mPsychoWidthRatio,mSubject->mMTS->mPsychoWidthExp);
    

//     return (w + minDis)*2;*/
//     return getWidth() +mSubject->mAgentData->mDriver->mMinPsychoWidth;
// }

// //MTS_Vehicle.cpp
// float MTS_Vehicle::getLateralSeparation( const MTS_Vehicle *veh ,float time ) const
// {
//     // the function is to compute the relative offset from veh to this
//     // the relative offset is cross-edge

//     // A stands for the object vehicle
//     MTS_VehicleController *controllerA = veh->getCurrentController();
//     MTS_Lane *laneA = controllerA->getLane();
//     MTS_Edge *edgeA = laneA->getEdge();	
//     MTS_Lane *nextLaneA = veh->getNextLane();
//     MTS_Edge *nextEdgeA = NULL;
//     if( nextLaneA ) nextEdgeA = nextLaneA->getEdge();

//     // B stands for the subject vehicle
//     MTS_VehicleController *controllerB = mCurrentController;
//     MTS_Lane *laneB = controllerB->getLane();
//     MTS_Edge *edgeB = laneB->getEdge();	
//     MTS_Lane *nextLaneB = getNextLane();
//     MTS_Edge *nextEdgeB = NULL;
//     if( nextLaneB ) nextEdgeB = nextLaneB->getEdge();

//     float offsetA, offsetB;
    
//     offsetA = controllerA->getLateralOffset() + time*veh->getLateralSpeed();
//     offsetB = controllerB->getLateralOffset() + time*getLateralSpeed();

//     if( edgeA != edgeB )
//     {
//         if( nextEdgeB == edgeA )
//         {
            
//         }
//         else if( nextEdgeA == edgeB )
//         {
            
//         }
            
//     }
//     return offsetB - offsetA;
// }

// //MTS_Vehicle.cpp
// float MTS_Vehicle::getLateralSeparation( const MTS_Vehicle *veh ,float time ) const
// {
//     // the function is to compute the relative offset from veh to this
//     // the relative offset is cross-edge

//     // A stands for the object vehicle
//     MTS_VehicleController *controllerA = veh->getCurrentController();
//     MTS_Lane *laneA = controllerA->getLane();
//     MTS_Edge *edgeA = laneA->getEdge();	
//     MTS_Lane *nextLaneA = veh->getNextLane();
//     MTS_Edge *nextEdgeA = NULL;
//     if( nextLaneA ) nextEdgeA = nextLaneA->getEdge();

//     // B stands for the subject vehicle
//     MTS_VehicleController *controllerB = mCurrentController;
//     MTS_Lane *laneB = controllerB->getLane();
//     MTS_Edge *edgeB = laneB->getEdge();	
//     MTS_Lane *nextLaneB = getNextLane();
//     MTS_Edge *nextEdgeB = NULL;
//     if( nextLaneB ) nextEdgeB = nextLaneB->getEdge();

//     float offsetA, offsetB;
    
//     offsetA = controllerA->getLateralOffset() + time*veh->getLateralSpeed();
//     offsetB = controllerB->getLateralOffset() + time*getLateralSpeed();

//     if( edgeA != edgeB )
//     {
//         if( nextEdgeB == edgeA )
//         {
            
//         }
//         else if( nextEdgeA == edgeB )
//         {
            
//         }
            
//     }
//     return offsetB - offsetA;
// }

// //MTS_VehilcleController.cpp
// float MTS_VehicleController::getExtendedDistance( float lateralOffset ) const
// {
//     float dis = 0.0f;
//     if( lateralOffset > mGapVariable.SeparationLateralOffset )
//     {
//         float ratio = ( lateralOffset - mGapVariable.SeparationLateralOffset ) / mGapVariable.RightWidth;
//         ratio = MIN( 1.0f , ratio );
//         dis = mGapVariable.MaxRightGap * ratio;
//     }
//     else
//     {
//         float ratio = ( mGapVariable.SeparationLateralOffset  -  lateralOffset ) / mGapVariable.LeftWidth;
//         ratio = MIN( 1.0f , ratio );
//         dis = mGapVariable.MaxLeftGap * ratio;
//     }
//     return dis;
// }

// //MTS_LongitudinalModel.cpp
// float MTS_LongitudinalModel::_computeDesiredGap( const MTS_MovingModelParameter* param , const MTS_AgentData*	mAgentData , MTS_Vehicle *subject , const MTS_Vehicle *object )
// {


//     MTS_VehicleController *subjectController = subject->getCurrentController();
    
//     float v_s = subjectController->getCurrentSpeed();
//     float a_max = param->maxAcceleration;
//     float b_com = param->comfortableDeceleration;
//     float t= mAgentData->mDriver->mResponseTime;

//     if( object == NULL )
//         return subject->getMinGap() + v_s * t + v_s * v_s / ( 2*sqrt(a_max*b_com) );
    

//     MTS_VehicleController *objectController = object->getCurrentController();
//     float v_o = objectController->getCurrentSpeed();
//     float w = _computeInfluence( subjectController , objectController );
//     //if( mMTS->mFollowingMode != -1 )
//     //{
//     //	w = 1.0f;
//     //}
//     //if( MTS::FindSpaceMode == 1 )
    
//     w = 1.0f;

//     float test_aggressive = 1.0f ;
//     float speedCon = (v_s * t + v_s * ( v_s - v_o ) / ( 2*sqrt(a_max*b_com) ) )*test_aggressive;

//     float desired_gap = ( subject->getMinGap()  + speedCon )* w ;
//     if( desired_gap < 0.0 ) desired_gap = subject->getMinGap();

//     if( desired_gap!= desired_gap )
//     { 
//         desired_gap = subject->getMinGap(); 
//     }

//     if( mMTS->mLoopingControl && subject->temp_desiredGap > 0.0f && subject->temp_desiredGap>desired_gap-subject->getMinGap()*w )
//     {
//         return subject->temp_desiredGap;
//     }
//     else
//     {
//         subject->temp_desiredGap = desired_gap;
//         return desired_gap;
//     }
// }

// //MTS_LongitudinalModel.cpp
// float MTS_LongitudinalModel::_computeInfluence( const MTS_VehicleController *subjectController , const MTS_VehicleController *objectController )
// {
//     float x_dis = subjectController->getSubject()->getLateralSeparation( objectController->getSubject() );
//     float x_dis_abs = ABS( x_dis );
    
//     float vehW = subjectController->getSubject()->getVehicleType()->getStaticWidth();
//     float preVehW = objectController->getPsychoWidth( subjectController->getCurrentSpeed() );
//     float minDis = ( vehW + preVehW ) / 2.0f;
    
//     float preWeight = 1 - x_dis_abs / minDis;
//     preWeight = MAX( 0.0f , preWeight );

//     return preWeight;
// }

// //MTS_VehicleController.cpp
// float MTS_VehicleController::getGapToStopLine( float stopOffset ) const
// {
//     float halfVehLen = getLength()/2.0f;
//     float headOffset = getOffset() + halfVehLen;

//     return stopOffset - headOffset;
// }

// /*****************************
// ********Lateral Model*********
// ********Offfset Part**********
// *****************************/

// //MTS_LateralModel.cpp
// void MTS_LateralModel::updateBestLateralOffset( MTS_MovingModelParameter* param , MTS_Vehicle* mSubject)
// {
//     /*
//         �ثe����n�b�o�̥[�J���v�ҫ��A�ӥB�٫ܦ��i��O��statistical data �Ӱ�.
//     */
    
//     if( !mMTS->mAllowPassing )
//     {
//         mSubject->setDesiredLateralOffset( mSubject->getLateralOffset() );
//         return;
//     }

//     if( mMTS->mVehicleLearningMode == 3 )
//     {
//         float bestOffset ; 
//         if( _updateBestLateralOffsetByPMatrix(bestOffset,false) )
//         {
//             _safetyCheckMatrix( bestOffset );
//             mSubject->setDesiredLateralOffset( bestOffset );
//             return;
//         } 
//         else
//         {
//             bool safe = _safetyCheckMatrix( bestOffset );
//             if( safe ) 
//             {
//                 mSubject->setDesiredLateralOffset( bestOffset );
//                 return ;
//             }
//             else
//             {
                
//                 for( int i=0;i<MTS_PossiableMatrix::Dimension;++i )
//                 {
//                     mSubject->temp_pMatrixData[i] =  -1;
//                 }	
//                 mSubject->getSituationData()->forceReupdateRegion(0);
//                 param->allRegion.clear();
//                 for( int i=0;i<mSubject->getSituationData()->mRegion.size();i++ )
//                 {
//                     param->allRegion.push_back(&mSubject->getSituationData()->mRegion[i]);
//                 }
//                 float newLateralOffset = _computeBestLateralOffset( mSubject , param );
//                 mSubject->setDesiredLateralOffset( newLateralOffset );
                


//             }
            
//         }

//     }

//     if( mMTS->mVehicleLearningMode == 4 || mMTS->mVehicleLearningMode == 5)
//     {
//         float bestOffset ; 
//         if( _updateBestLateralOffsetByPMatrix(bestOffset,false) )
//         {
//             _safetyCheckMatrix( bestOffset );
//             mSubject->setDesiredLateralOffset( bestOffset );
//             return;
//         } 
//         else
//         {
//             bool safe = _safetyCheckMatrix( bestOffset );
//             if( safe ) 
//             {
//                 mSubject->setDesiredLateralOffset( bestOffset );
//                 return ;
//             }
//             else
//             {
                
//                 for( int i=0;i<MTS_PossiableMatrix::Dimension;++i )
//                 {
//                     mSubject->temp_pMatrixData[i] =  -1;
//                 }	
//                 mSubject->getSituationData()->forceReupdateRegion(0);
//                 param->allRegion.clear();
//                 for( int i=0;i<mSubject->getSituationData()->mRegion.size();i++ )
//                 {
//                     param->allRegion.push_back(&mSubject->getSituationData()->mRegion[i]);
//                 }
//                 float newLateralOffset = _computeBestLateralOffset( mSubject , param );
//                 mSubject->setDesiredLateralOffset( newLateralOffset );
                


//             }
            
//         }

//     }

//     float newLateralOffset = _computeBestLateralOffset( mSubject , param );
//     if( newLateralOffset!= newLateralOffset )
//     {
//         newLateralOffset = mSubject->getLateralOffset();
//     }
//     mSubject->setDesiredLateralOffset( newLateralOffset );
// }

// //MTS_LateralModel.cpp
// float MTS_LateralModel::_computeBestLateralOffset( MTS_Vehicle *veh , MTS_MovingModelParameter* param )
// {
//     MTS_Vehicle *left = veh->getLeftVehicle();
//     MTS_Vehicle *right = veh->getRightVehicle();
//     MTS_Edge *edge = veh->getLane()->getEdge();

//     int typeCode = veh->getVehicleType()->getTypeCode();
//     float halfVehWidth = veh->getVehicleType()->getStaticWidth() / 2.0f;
//     float vehLateralOffset = veh->getLateralOffset();
//     float vehOffset = veh->getOffset() + veh->getVehicleType()->getStaticLength() / 2.0f;
//     float vehSpeed = veh->getCurrentSpeed();
//     MTS_Region &validSpace = param->currentRegion;

//     int validSpaceID = -1;
//     int bestSpaceID = -1;
//     float maxCost = 0.0f;
//     float maxValidCost = 0.0f;

//     int spaceSize = param->allRegion.size();
    
//     for( int i = 0 ; i < spaceSize ; ++i )
//     {
//         if( param->allRegion[i]->width < 1.8f * halfVehWidth )
//             continue;
//         float laneCenter = param->allRegion[i]->offset;

//         float offset_diff = param->allRegion[i]->offset - vehLateralOffset;
//         float safeOffset;

//         bool safeSpace = true;
//         if( param->allRegion[i]->width < veh->getVehicleType()->getStaticWidth() )
//         {
//             safeSpace = false;
//             param->allRegion[i]->safety = 0.0f;
//         }
//         else if( offset_diff < 0 ) // check if the front and the rear of subject vehicle at the desired offset are safe
//         {
//             //_bindFrontVehicles( param->allRegion[i]->frontVehicles );
//             //_bindRearVehicles( param->allRegion[i]->rearVehicles );
//             bool safeLeftSpace = _checkLeftSafety( param->allRegion[i]->offset , &safeOffset , param->allRegion[i] );
//             bool adjustOffset = param->mSpaceOriented  && safeOffset < vehLateralOffset;

//             /*if( MTS::AutoTracking && !enoughLeftSpace )
//                 MTS::VehicleSelect->setSubjectVehicle( veh );*/
//             if( !safeLeftSpace && !adjustOffset ) 
//                 safeSpace = false;
//             else if( !safeLeftSpace )
//                 offset_diff = safeOffset - vehLateralOffset;
//         }
//         else if( offset_diff > 0 ) // check if the front and the rear of subject vehicle at the desired offset are safe
//         {
//             //_bindFrontVehicles( param->allRegion[i]->frontVehicles );
//             //_bindRearVehicles( param->allRegion[i]->rearVehicles );
//             bool safeRightSpace = _checkRightSafety( param->allRegion[i]->offset , &safeOffset , param->allRegion[i]);
//             bool adjustOffset = param->mSpaceOriented  && safeOffset > vehLateralOffset;
//             /*if( MTS::AutoTracking && !enoughRightSpace )
//                 MTS::VehicleSelect->setSubjectVehicle( veh );*/
//             if( !safeRightSpace && !adjustOffset )
//                 safeSpace = false;
//             else if( !safeRightSpace )
//                 offset_diff = safeOffset - vehLateralOffset;
//         }

//         bool dirPriority = false;//( offset_diff < 0 && mTargetDirection ==DIR_LEFT ) || ( offset_diff > 0 && mTargetDirection ==DIR_RIGHT );

//         offset_diff = ABS( offset_diff );
//         float gap = param->allRegion[i]->gap;

//         int laneIdx = edge->getLaneID( param->allRegion[i]->offset );
//         MTS_Lane *lane = edge->getLane( laneIdx );
//         bool priority = lane->havePriority( typeCode );
//         bool permission = lane->havePermission( typeCode );
//         bool target = laneIdx == veh->getDesireLane();
//         bool velCosistent = ( offset_diff * veh->getLateralSpeed() ) > 0;		
//         float speed_diff = param->allRegion[i]->maxPassingSpeed - vehSpeed;

//         MTS_Vehicle *brokenVehicle = lane->getBlockage();
//         bool blockage = lane->endOfRoad() || 
//                                         ( brokenVehicle != NULL && _checkBlockage( param->allRegion[i] ,  brokenVehicle ) );

//         float safety = param->allRegion[i]->safety;
//         float turnControl = _turnControl( veh , param->allRegion[i] );
        
//         float w_speed		= param->mRegionSelectionWeight->weight_speed;
//         float w_gap			= param->mRegionSelectionWeight->weight_gap;
//         float w_dis			= param->mRegionSelectionWeight->weight_lateralDistance;
//         float w_blockage	= -param->mRegionSelectionWeight->weight_blockage;
//         float w_priority	= param->mRegionSelectionWeight->weight_priority;
//         float w_permission	= param->mRegionSelectionWeight->weight_permission; 
//         float w_target		= param->mRegionSelectionWeight->weight_targetLane;
//         float w_dir			= param->mRegionSelectionWeight->weight_targetDirection;
//         float w_vel			= param->mRegionSelectionWeight->weight_velocityConsistency;
//         float w_safe		= param->mRegionSelectionWeight->weight_safety;
//         float w_turnControl = param->mRegionSelectionWeight->weight_turnControl;

//     /*	float lateralTime = ABS(offset_diff / veh->getLateralSpeed());
//         lateralTime = lateralTime > 1.0f ? 1.0f: lateralTime ;
//         float lomgitudinalTime =  param->allRegion[ 0 ]->gap / veh->getCurrentSpeed();
//         if( lomgitudinalTime  < lateralTime  ){ w_safe=0.0f; }
//         */

//         param->allRegion[i]->preference = 
//                                 w_speed * speed_diff +
//                                 w_gap * gap + 
//                                 w_dis * offset_diff + 
//                                 w_priority * priority +
//                                 w_permission * permission +
//                                 w_target * target +
//                                 w_blockage * blockage +
//                                 w_dir * dirPriority +
//                                 w_vel * velCosistent +
//                                 w_safe * (safety) +
//                                 w_turnControl * turnControl
//                                 ;
                    
//         //for controlled vehicle
//         if( mMTS->mVehicleLearningMode == 1 )
//         {
//             float lateralDiff = abs(veh->getConfigureControlLOffset()-param->allRegion[i]->offset);
//             param->allRegion[i]->preference =  50000 * (safety)-500*lateralDiff  ;
    
//         }


//         if( param->allRegion[i]->preference > maxCost )
//         {
//             bestSpaceID = i;
//             maxCost = param->allRegion[i]->preference;
//         }
        
//         if( safeSpace && param->allRegion[i]->preference > maxValidCost )
//         {
//             validSpaceID = i;
//             maxValidCost = param->allRegion[i]->preference;
//         }
//     }
//     //if( validSpaceID <= 0 && bestSpaceID > 0 )
//     //	_buildInvolvedVehicles( param->allRegion[ bestSpaceID ]->frontVehicles , param->allRegion[ bestSpaceID ]->rearVehicles );

//     int finalLateralOffset = _normalDistPossility(  veh ,  param  );

//     if( validSpaceID == -1 )
//     {
//         if( veh->needTocutIn && param->allRegion[0]->safety > 0.8f ) return param->allRegion[ 0 ]->offset;
//         return vehLateralOffset;
//     }

//     //if(  param->allRegion[ 0 ]->gap < ABS( param->allRegion[ 0 ]->offset-param->allRegion[ validSpaceID ]->offset )  ){ return param->allRegion[ 0 ]->offset; }
//     //if(  param->allRegion[ 0 ]->gap < veh->getMinGap()*2.0f ){ return param->allRegion[ 0 ]->offset; }

//     if( validSpaceID == 0 )
//     {
//         return param->allRegion[ validSpaceID ]->offset;
//     }

    
//     if( veh->getLateralOffset() > param->allRegion[ validSpaceID ]->offset )
//     {
//         return param->allRegion[ validSpaceID ]->rightBorder - veh->getVehicleType()->getStaticWidth();
//     }
//     if( veh->getLateralOffset() < param->allRegion[ validSpaceID ]->offset )
//     {
//         return param->allRegion[ validSpaceID ]->leftBorder + veh->getVehicleType()->getStaticWidth();
//     }
    
//     //return param->allRegion[ validSpaceID ]->offset;
// }

// //MTS_LateralModel.cpp
// bool MTS_LateralModel::_checkLeftSafety( float desiredOffset , float *safeOffset , MTS_Region *region) const
// {
//     const std::vector< MTS_Vehicle* > &rearVehicles = mSubject->getLeftRearVehicles();
//     //mSubject->getLeftRearVehicles( 2000.0f , rearVehicles );

//     std::vector< MTS_Vehicle* >::const_iterator it = rearVehicles.begin();
//     std::vector< MTS_Vehicle* >::const_iterator vehEnd = rearVehicles.end();

//     float lateralOffet = mSubject->getCurrentController()->getLateralOffset();
//     float t = getLateralTime( desiredOffset );
//     float v = ( desiredOffset - lateralOffet ) / t;
    
//     bool safe = true;
//     float safeTime;
//     float minSafeTime = FLT_MAX;

//     //check BorderVehicle
//     if( region->leftBorderVehicle!=NULL && region->leftBorderVehicle->getActive() )
//     {
//         MTS_Vehicle* leftVeh = region->leftBorderVehicle;
//         float leftVeh_desiredLateralOffset = leftVeh->getDesiredLateralOffset();
//         float leftVehAngle = leftVeh->getCurrentController()->getYawAngle();
//         float subjectAngle = mSubject->getCurrentController()->getYawAngle();
//         bool lateralOverlap = (leftVeh->getVehicleType()->getDynamicWidth( leftVehAngle ) + mSubject->getVehicleType()->getDynamicWidth( subjectAngle )  )/2.0f < abs(leftVeh_desiredLateralOffset-desiredOffset) ;
//         if( lateralOverlap )
//         { 
//             float vehHeadOffset = mSubject->getOffset() + mSubject->getVehicleType()->getDynamicLength( mSubject->getCurrentController()->getYawAngle() );
//             float objetVehHeadOffset = leftVeh->getOffset() + leftVeh->getVehicleType()->getDynamicLength( leftVeh->getCurrentController()->getYawAngle() );
//             if( vehHeadOffset <= objetVehHeadOffset )
//                 return 0.0f;
//         }
//     }

//     if( safeOffset ) *safeOffset = desiredOffset;
//     region->safety = 1.0;
//     for( ; it != vehEnd ; ++it )
//     {
//         bool checkSafe = _checkSafety( mSubject , *it , v , t , true , true , &safeTime );
//         float patience = mSubject->getPatience();
//         if( !checkSafe && patience > 0.75)
//         {
//             /*
//             if( mSubject->getRelativeOffset( (*it) ) - (*it)->getCurrentController()->getLength()*(-2)*patience > 0.0 )
//             {
//                 checkSafe = true;
//                 region.safety = 0.5;
//             }
//             */
//             region->safety = 0.85;
//             checkSafe = true;
//         }

//         if( !checkSafe )
//         {
//             if( safeOffset && safeTime < minSafeTime )
//             {
//                 minSafeTime = safeTime;
//                 MTS_VehicleController *controller = (*it)->getCurrentController();
//                 float currentOffset = controller->getLateralOffset();
//                 float currentSpeed = controller->getLateralSpeed();
//                 float minDis = ( controller->getPsychoWidth( mSubject->getCurrentSpeed() ) + mSubject->getVehicleType()->getStaticWidth() ) / 2.0f + 2.0f;
//                 *safeOffset = currentOffset + currentSpeed * safeTime + minDis;
            
//             }
//             safe = false;
//             region->safety = 0.0;
//         }
        
//         //if( safeTime >= 0 && mNecessityValue == 1.0f )
//         //	involvedRear.push_back( InvolvedVehiclePair( *it , safeTime ) );
//     }

//     const std::vector< MTS_Vehicle* > &frontVehicles = mSubject->getLeftFrontVehicles();
//     //mSubject->getLeftFrontVehicles( 2000.0f , frontVehicles );
    
//     it = frontVehicles.begin();
//     vehEnd = frontVehicles.end();

//     for( ; it != vehEnd ; ++it )
//     {
//         bool checkSafe = _checkSafety( mSubject , *it , v , t , true , true , &safeTime );
//         float patience = mSubject->getPatience();	
//         if( !checkSafe && patience > 0.75)
//         {

            
//             if( mSubject->getRelativeOffset( (*it) ) - (*it)->getCurrentController()->getLength()*(-2) > 0.0 )
//             {
//                 checkSafe = true;
//                 region->safety = 0.5;
//             }
//         }

//         if( !checkSafe )
//         {
//             if( safeOffset && safeTime < minSafeTime )
//             {
//                 minSafeTime = safeTime;
//                 MTS_VehicleController *controller = (*it)->getCurrentController();
//                 float currentOffset = controller->getLateralOffset();
//                 float currentSpeed = controller->getLateralSpeed();
//                 float minDis = ( controller->getPsychoWidth( mSubject->getCurrentSpeed() ) + mSubject->getVehicleType()->getStaticWidth() ) / 2.0f + 2.0f;
//                 *safeOffset = currentOffset + currentSpeed * safeTime + minDis;
//             }
//             safe = false;
//             region->safety = 0.0;
//         }
//         //if( safeTime >= 0 && mNecessityValue == 1.0f )
//         //	involvedFront.push_back( InvolvedVehiclePair( *it , safeTime ) );
//     }

//     return safe;


// }

// //MTS_LateralModel.cpp
// bool MTS_LateralModel::_checkRightSafety( float desiredOffset , float *safeOffset , MTS_Region *region) const
// {

//     //MTS::SystemLog << "Check right safety ( " << MTS::SimulationTime << " ) ****************************************\n";

//     const std::vector< MTS_Vehicle* > &rearVehicles = mSubject->getRightRearVehicles();
//     //mSubject->getRightRearVehicles( 2000.0f , rearVehicles );

//     std::vector< MTS_Vehicle* >::const_iterator it = rearVehicles.begin();
//     std::vector< MTS_Vehicle* >::const_iterator vehEnd = rearVehicles.end();

//     float t = getLateralTime( desiredOffset );
//     float lateralOffset = mSubject->getCurrentController()->getLateralOffset();
//     float v = ( desiredOffset - lateralOffset ) / t;

//     //check BorderVehicle
//     if( region->rightBorderVehicle!=NULL && region->rightBorderVehicle->getActive() )
//     {
//         MTS_Vehicle* rightVeh = region->rightBorderVehicle;
//         float rightVeh_desiredLateralOffset = rightVeh->getDesiredLateralOffset();
//         float rightVehAngle = rightVeh->getCurrentController()->getYawAngle();
//         float subjectAngle = mSubject->getCurrentController()->getYawAngle();
//         bool lateralOverlap = (rightVeh->getVehicleType()->getDynamicWidth( rightVehAngle ) + mSubject->getVehicleType()->getDynamicWidth( subjectAngle ) )/2.0f < abs(rightVeh_desiredLateralOffset-desiredOffset) ;
//         if( lateralOverlap )
//         {
//             float vehHeadOffset = mSubject->getOffset() + mSubject->getVehicleType()->getDynamicLength( mSubject->getCurrentController()->getYawAngle() );
//             float objetVehHeadOffset = rightVeh->getOffset() + rightVeh->getVehicleType()->getDynamicLength( rightVeh->getCurrentController()->getYawAngle() );
//             if( vehHeadOffset <= objetVehHeadOffset )
//                 return 0.0f;
//         }

//     }

//     bool safe = true;
//     float safeTime;
//     float minSafeTime = FLT_MAX;
//     if( safeOffset ) *safeOffset = desiredOffset;
//     region->safety = 1.0;
//     for( ; it != vehEnd ; ++it )
//     {
//         bool checkSafe = _checkSafety( mSubject , *it , v , t , true , true , &safeTime );
//         float patience = mSubject->getPatience();
//         if( !checkSafe && patience > 0.75)
//         {
//             /*
//             if( mSubject->getRelativeOffset( (*it) ) - (*it)->getCurrentController()->getLength()*(-2) > 0.0 )
//             {
//                 checkSafe = true;
//                 region.safety = 0.5;
//             }
//             */
//             region->safety = 0.85;
//             checkSafe = true;
//         }

//         if( !checkSafe )
//         {
//             if( safeOffset && safeTime < minSafeTime )
//             {
//                 minSafeTime = safeTime;
//                 MTS_VehicleController *controller = (*it)->getCurrentController();
//                 float currentOffset = controller->getLateralOffset();
//                 float currentSpeed = controller->getLateralSpeed();
//                 float minDis = ( controller->getPsychoWidth( mSubject->getCurrentSpeed() ) + mSubject->getVehicleType()->getStaticWidth() ) / 2.0f + 2.0f;
//                 *safeOffset = currentOffset + currentSpeed * safeTime - minDis;
//             }
//             safe = false;
//             region->safety = 0.0;
//         }
//     }

//     const std::vector< MTS_Vehicle* > &frontVehicles = mSubject->getRightFrontVehicles();
//     //mSubject->getRightFrontVehicles( 2000.0f , frontVehicles );
    
//     it = frontVehicles.begin();
//     vehEnd = frontVehicles.end();

//     for( ; it != vehEnd ; ++it )
//     {
//         bool checkSafe = _checkSafety( mSubject , *it , v , t , true , true , &safeTime );
//         float patience = mSubject->getPatience();
//         if( !checkSafe && patience > 0.75)
//         {
            
            
//             if( mSubject->getRelativeOffset( (*it) ) - (*it)->getCurrentController()->getLength()*(-2) > 0.0 )
//             {
//                 checkSafe = true;
//                 region->safety = 0.5;
//             }
//         }

//         if( !checkSafe )
//         {
//             if( safeOffset && safeTime < minSafeTime )
//             {
//                 minSafeTime = safeTime;
//                 MTS_VehicleController *controller = (*it)->getCurrentController();
//                 float currentOffset = controller->getLateralOffset();
//                 float currentSpeed = controller->getLateralSpeed();
//                 float minDis = ( controller->getPsychoWidth( mSubject->getCurrentSpeed() ) + mSubject->getVehicleType()->getStaticWidth() ) / 2.0f + 2.0f;
//                 *safeOffset = currentOffset + currentSpeed  * safeTime - minDis;
//             }
//             safe = false;
//             region->safety = 0.0;
//         }
//     }


//     return safe;
// }

// //MTS_LateralModel.cpp
// float MTS_LateralModel::getLateralTime( float mDesiredLateralOffset ) const
// 	{
// 		float diff_lat = mDesiredLateralOffset - mSubject->getCurrentController()->getLateralOffset();
// 		float v_lat = mSubject->getCurrentController()->getLateralSpeed();
// 		float t_lat = 2.0f * diff_lat / v_lat;

// 		// if the time to decelerate to zero lateral speed is smaller than maximum movement time
// 		if( t_lat > 0.0f && t_lat < mMaxMovementTime ) 
// 			return t_lat;

// 		float ttc = getLongitudinalTime();

// 		return ttc;
// 	}

// //MTS_LateralModel.cpp
// bool MTS_LateralModel::_checkSafety( MTS_Vehicle *subject , MTS_Vehicle *object , 
//                                         float moveSpeed , float moveTime , bool subjectAsLeader , bool subjectAsFollower , float* safeTime ) const
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

//     if( subjectAsLeader )
//     {
//         //( *mRearVehicles ).push_back( InvolvedVehiclePair( object , t_y ) );
//         //MTS::SystemLog << "V" << subject->getID() << " and V" << object->getID() << ": Safe(leader)\n";
//         //MTS::SystemLog << "dis= " << p_r_o + hl_s + hl_o << ", disS= " << d_s << ", disO= " << d_o << ", t= " << t_y <<  "t_lat= " << moveTime << '\n'; 
//         //MTS::SystemLog << "v_lat= " << subject->getLateralSpeed() << ", v_long= " << subject->getCurrentSpeed()<<'\n';
//     }
//     else if( subjectAsFollower )
//     {
//         //( *mFrontVehicles ).push_back( InvolvedVehiclePair( object , t_y ) );
//         //MTS::SystemLog << "V" << subject->getID() << " and V" << object->getID() << ": Safe(follower)\n";
//         //MTS::SystemLog << "dis= " << p_r_s - hl_s - hl_o << ", disS= " << d_s << ", disO= " << d_o << ", t= " << t_y <<  "t_lat= " << moveTime << '\n'; 
//         //MTS::SystemLog << "v_lat= " << subject->getLateralSpeed() << ", v_long= " << subject->getCurrentSpeed()<<'\n';
//     }

//     if( ( p_r_t < 0 && p_r_t + hl_s + hl_o < -d_o * subject->getGapAcceptRatio() ) || (  p_r_t >= 0 && p_r_t - hl_s - hl_o > d_s * subject->getGapAcceptRatio() ) )
//         return true;

//     //MTS::SystemLog << "V" << subject->getID() << " and V" << object->getID() << ": Danger\n";
//     //MTS::SystemLog << "dis= " << p_r_t << ", disS= " << d_s << ", disO= " << d_o << ", t= " << t_y << "t_lat= " << moveTime << '\n'; 
//     //MTS::SystemLog << "v_lat= " << subject->getLateralSpeed() << ", v_long= " << subject->getCurrentSpeed()<<'\n';
    
//     return false;
// }

// //MTS_LateralModel.cpp
// bool MTS_LateralModel::_checkBlockage( MTS_Region *space, const MTS_Vehicle *blockage ) const
// {
//     float spaceCenter = ( space->leftBorder + space->rightBorder ) / 2.0f;
//     float blockageCenter = blockage->getCurrentController()->getLateralOffset();
//     float spaceWidth = space->width;
//     float blockageWidth = blockage->getCurrentController()->getWidth(); 
//     float lateralSeparation = ABS( spaceCenter - blockageCenter );
//     if( lateralSeparation <= ( spaceWidth + blockageWidth ) / 2.0f )
//         return true;
//     return false;
// }

// //MTS_LateralModel.cpp
// int MTS_LateralModel::_normalDistPossility(  MTS_Vehicle *veh , MTS_MovingModelParameter* param )
// {
//     float preference[10];
//     int ID[10];
//     float sumPre = 0.0f;
//     for( int i=0;i<param->allRegion.size();++i)
//     {
//         if( preference[i] < 0.0f )
//         {
//             preference[i] = 0.0f ;
//             ID[i] = i;
//             sumPre += 0.0f;						
//         }
//         else
//         {
//             preference[i] = param->allRegion[i]->preference ;
//             ID[i]= i;
//             sumPre += preference[i];			
//         }

//     }
//     if( sumPre < 0.001 ){ return -1; }
//     for( int i=0;i<param->allRegion.size();++i) preference[i] = preference[i]/sumPre;

//     for( int i=0;i<param->allRegion.size();++i )
//     {
//         float maxPref = -1.0f;
//         int maxID =-1;
//         int maxPos = -1;
//         for( int j=i;j<param->allRegion.size();++j  )
//         {
//             if( preference[j] >maxPref )
//             {
//                 maxPref = preference[j];
//                 maxID = ID[j];
//                 maxPos = j;
//             }
//         }
//         float tempPref = preference[i];
//         int tempID = ID[i];
//         preference[i] = maxPref;
//         ID[i] = maxID;
//             preference[ maxPos ] = tempPref;
//             ID[ maxPos ] = tempID;
//     }
    
//     for( int i=1;i<param->allRegion.size();++i)  preference[i] = preference[i] +preference[i-1];

//     std::default_random_engine generator(time(0));
//     std::normal_distribution<double> distribution(0.5,0.2);
//     float randomValue = (float)distribution(generator);
//     if( randomValue > 1.0f ) randomValue = 1.0f;
//     else if( randomValue < 0.0f ) randomValue = 0.0f;
//     int pickID = -1;
//     for( int i=0;i<param->allRegion.size();++i) 
//     {
//         if( randomValue-0.5 < preference[i]/2.0f ){ pickID = ID[i] ;break; }
//     }

//     return pickID;

// }

// /*****************************
// ********Lateral Model*********
// *********Speed Part***********
// *****************************/

// //MTS_LateralModel.cpp
// float MTS_LateralModel::getSpeedChange( MTS_MovingModelParameter* param , MTS_Vehicle* mSubject)
// {
//     float lateralSpeedChange;

//     this->mSubject = mSubject;
    
//     float diff_lat = mSubject->getDesiredLateralOffset() - mSubject->getCurrentController()->getLateralOffset();
//     float v_lat = mSubject->getCurrentController()->getLateralSpeed();
//     float t_lat = 2.0f * diff_lat / v_lat;

//     // if the time to decelerate to zero lateral speed is smaller than maximum movement time
//     if( t_lat > 0.0f && t_lat < 2.0 ) 
//     {
//         //t_lat = t_lat;
//     }
//     else
//     {
//         t_lat = _getLongitudinalTime( mSubject , 2.0f );
//     }


//     lateralSpeedChange = _getLateralSpeed( mSubject , t_lat );

//     return lateralSpeedChange;
// }

// //MTS_LateralModel.cpp
// float MTS_LateralModel::_getLongitudinalTime( MTS_Vehicle* mSubject , float mMaxMovementTime ) const
// {
//     MTS_Vehicle *pred = mSubject->getPassedLeadingVehicle();

//     float v = mSubject->getCurrentController()->getCurrentSpeed();
//     float t_long = mSubject->getCurrentController()->getGapToStopLine() / v;

//     if( pred != NULL )
//     {
//         float v_pred = 0.0f;
//         float deltaV = v - v_pred;

//         float s = mSubject->getCurrentController()->getGap( pred ); 
        
//         float ttc = s / deltaV;
//         if( ttc < t_long )
//             t_long = ttc;
//     }
//     if( t_long > 0 && t_long < mMaxMovementTime )
//         return t_long;
//     return mMaxMovementTime;
// }

// //MTS_VehicleController.cpp
// float MTS_VehicleController::getGapToStopLine() const
// {
//     float halfVehLen = getLength()/2.0f;
//     float headOffset = getOffset() + halfVehLen;
//     float stopOffset = getLane()->getLength();

//     return stopOffset - headOffset;
// }

// //MTS_VehicleController.cpp
// float MTS_VehicleController::getGap( MTS_Vehicle *pred ) const
// {
//     float halfPredLen = pred->getCurrentController()->getLength() / 2.0f;
//     float halfVehLen = getLength() / 2.0f;

//     float relativeOffset = pred->getRelativeOffset( mSubject );
//     float gap = relativeOffset - ( halfPredLen + halfVehLen );
//     //gap += getExtendedGap( pred );
//     return gap;
// }

// //MTS_Vehicle.cpp
// float MTS_Vehicle::getRelativeOffset( const MTS_Vehicle *veh ) const
// {
//     // the function is to compute the relative offset from veh to this
//     // the relative offset is cross-edge

//     // A stands for the object vehicle
//     MTS_VehicleController *controllerA = veh->getCurrentController();
//     MTS_Lane *laneA = controllerA->getLane();
//     MTS_Edge *edgeA = laneA->getEdge();	
//     MTS_Lane *nextLaneA = veh->findNextLane();
//     MTS_Edge *nextEdgeA = NULL;
//     if( nextLaneA ) nextEdgeA = nextLaneA->getEdge();

//     // B stands for the subject vehicle
//     MTS_VehicleController *controllerB = mCurrentController;
//     MTS_Lane *laneB = controllerB->getLane();
//     MTS_Edge *edgeB = laneB->getEdge();	
//     MTS_Lane *nextLaneB = findNextLane();
//     MTS_Edge *nextEdgeB = NULL;
//     if( nextLaneB ) nextEdgeB = nextLaneB->getEdge();

//     float offsetA, offsetB;
    
//     offsetA = controllerA->getOffset();
//     offsetB = controllerB->getOffset();

//     if( edgeA != edgeB )
//     {
//         if( nextEdgeB == edgeA )
//         {
//             float lateraloffset = getLateralOffset();
//             float offset = getOffset();
//             veh->getCurrentRoad()->positionTranslate( mRoadStructure , offset , lateraloffset );
//             offsetB = offset;
//         }
//         else if( nextEdgeA == edgeB )
//         {
//             float lateraloffset = veh->getLateralOffset();
//             float offset = veh->getOffset();
        
            
//             mRoadStructure->positionTranslate( veh->getCurrentRoad() , offset , lateraloffset );
//             offsetA = offset;
            
//         }
//     }
//     return offsetB - offsetA;
// }  

// //MTS_Lane.cpp
// void MTS_Lane::positionTranslate( MTS_RoadStructure* now , float &offset , float &lateralOffset)
// {
//     if( now->getType() == RoadStructureType::LANE )
//     {
//         MTS_Lane *currentLane = (MTS_Lane *) now;

//         float laneLen = currentLane->getLength();
//         float transitDistance = 0.0f;

//         Vector3 targetPos = getPosition( lateralOffset , 0 );
//         Vector3 sourcePos = currentLane->getPosition( lateralOffset , laneLen -1.01f);
//         //Vector3 targetPos = getEdge()->getPosition(0.0f);
//         //Vector3 sourcePos = currentLane->getEdge()->getPosition( laneLen-0.01 );
//         transitDistance = sourcePos.distance( targetPos );

//         offset = -( transitDistance + laneLen - offset );
//         return;
//     }
//     else if( now->getType() == RoadStructureType::INTERSECTION )
//     {
//         MTS_Lane *currentLane = (MTS_Lane *) now;
//         float laneLen = currentLane->getLength();

//         MTS_Intersection *current = (MTS_Intersection *) now;
//         Vector3 sourcePos = current->getPosition( lateralOffset , laneLen );
//         lateralOffset = sourcePos.x;
//         offset = sourcePos.z;
//     }

// }

// //MTS_LateralModel.cpp
// float MTS_LateralModel::_getLateralSpeed( const MTS_Vehicle *veh , float moveTime ) const
// {
//     MTS_Vehicle::ControllerType controllerType = veh->getControllerType();
//     const MTS_VehicleController *vehController = veh->getCurrentController();
    
//     float a_max = veh->getMaxAcceleration();

//     float x_desired = getDynamicDesiredOffset( veh );
//     float x_current = vehController->getLateralOffset();
//     float v_lateral = vehController->getLateralSpeed();

//     float t = moveTime;

//     float acc = 0.0f;

//     acc = 2 * ( (x_desired-x_current)/(t*t) - v_lateral/t );

//     if( acc < 0 )
//         acc = MAX( -a_max , acc );
//     else
//         acc = MIN( a_max , acc );

//     if( acc != acc )
//         int a=0;
//     return acc;

// }

// //MTS_LateralModel.cpp
// float MTS_LateralModel::getDynamicDesiredOffset( const MTS_Vehicle *veh ) const
// {
//     MTS_VehicleController *controller = veh->getCurrentController(); 
//     float staticWidth = veh->getVehicleType()->getStaticWidth();
//     float dynamicWidth = controller->getWidth();
//     float widthDiff = ( dynamicWidth - staticWidth ) / 2.0f;
//     float sign = ( controller->getYawAngle() >= 0 ) ? 1.0f : -1.0f;
//     float desiredOffset = controller->getDesiredLateralOffset();
//     return desiredOffset;
//     return desiredOffset - sign * widthDiff;
// }

// ////////////////////////////
// /////Other Utility Part/////
// ////////////////////////////

// /***********************************
// *****Original Transform Matrix******
// ************************************/

// std::vector<std::vector<float>> LocalizationStage::getMatrix(cg::Location actor_location, cg::Rotation actor_rotation) //local transfer to global
// {
//   const float pi = acosf(-1);
  
//   float c_y = cosf(actor_rotation.yaw * pi / 180.0f);
//   float s_y = sinf(actor_rotation.yaw * pi / 180.0f);
//   float c_r = cosf(actor_rotation.roll * pi / 180.0f);
//   float s_r = sinf(actor_rotation.roll * pi / 180.0f);
//   float c_p = cosf(actor_rotation.pitch * pi / 180.0f);
//   float s_p = sinf(actor_rotation.pitch * pi / 180.0f);

//   std::vector<std::vector<float>> M;
//   M.resize(4, std::vector<float>(4, 0.0f));
//   //std::vector<std::vector<float>>  M[4][4]; //matrix =  //np.array(np.identity(4))
//   M[3][0] = 0.0f;
//   M[3][1] = 0.0f;
//   M[3][2] = 0.0f;
//   M[3][3] = 1.0f;
//   M[0][3] = actor_location.x;
//   M[1][3] = actor_location.y;
//   M[2][3] = actor_location.z;
//   M[0][0] = c_p * c_y;
//   M[0][1] = c_y * s_p * s_r - s_y * c_r;
//   M[0][2] = -c_y * s_p * c_r - s_y * s_r;
//   M[1][0] = s_y * c_p;
//   M[1][1] = s_y * s_p * s_r + c_y * c_r;
//   M[1][2] = -s_y * s_p * c_r + c_y * s_r;
//   M[2][0] = s_p;
//   M[2][1] = -c_p * s_r;
//   M[2][2] = c_p * c_r;

//   return M;
// }

// std::vector<float> LocalizationStage::GlobalToLocal(std::vector<std::vector<float>> M, cg::Location global_location)
// {
//   //float inv_M[4][4];
//   std::vector<float> local_location(3, 0.0f);
//   //float local_velocity[4][1];

//   // if(inverse(M, inv_M))
//   // {
//   //cg::Location actor_location = simulation_state.GetLocation(target_id);
//     //Vector3D actor_velocity = simulation_state.GectVelocity(actor_id);
//   std::vector<float> location(4, 0.0f);
//   location[0] = global_location.x;
//   location[1] = global_location.y;
//   location[2] = global_location.z;
//   location[3] = 1.0f;

//   // float velocity[4][1];
//   // velocity[0][0] = actor_velocity.x;
//   // velocity[1][0] = actor_velocity.y;
//   // velocity[2][0] = actor_velocity.z;
//   // velocity[3][0] = 0.0f;

//   matrixMultiply(M, location, local_location); // vector call by ref?
//   //matrixMultiply(inv_M, velocity, local_velocity);
  
//   return local_location; //, velocity)
//   //}
// }

// void LocalizationStage::matrixMultiply(std::vector<std::vector<float>> M, std::vector<float>  V, std::vector<float> result){
//   for(unsigned long i=0; i < V.size() - 1; i++ ){
//     result[i]= M[i][0] * V[0] + M[i][1] * V[1] + M[i][2] * V[2] + M[i][3] * V[3];
//   }
// }

// void LocalizationStage::getCofactor(std::vector<std::vector<float>> A, std::vector<std::vector<float>> temp, unsigned p, unsigned q, unsigned n) 
// { 
//     unsigned i = 0, j = 0; 
  
//     // Looping for each element of the matrix 
//     for (unsigned row = 0; row < n; row++) 
//     { 
//         for (unsigned col = 0; col < n; col++) 
//         { 
//             //  Copying into temporary matrix only those element 
//             //  which are not in given row and column 
//             if (row != p && col != q) 
//             { 
//                 temp[i][j++] = A[row][col]; 
  
//                 // Row is filled, so increase row index and 
//                 // reset col index 
//                 if (j == n - 1) 
//                 { 
//                     j = 0; 
//                     i++; 
//                 } 
//             } 
//         } 
//     } 
// } 
  
// float LocalizationStage::determinant(std::vector<std::vector<float>> A, unsigned n) 
// { 
//     float D = 0.0f; // Initialize result 
  
//     //  Base case : if matrix contains single element 
//     if (n == 1) 
//         return A[0][0]; 
  
//     std::vector<std::vector<float>> temp; // To store cofactors 
//     temp.resize(4, std::vector<float>(4, 0.0f));
//     float sign = 1.0f;  // To store sign multiplier 
  
//      // Iterate for each element of first row 
//     for (unsigned f = 0; f < n; f++) 
//     { 
//         // Getting Cofactor of A[0][f] 
//         getCofactor(A, temp, 0, f, n); 
//         D += sign * A[0][f] * determinant(temp, n - 1); 
  
//         // terms are to be added with alternate sign 
//         sign = -sign; 
//     } 
  
//     return D; 
// } 
  
// // Function to get adjoint of A[N][N] in adj[N][N]. 
// void LocalizationStage::adjoint(std::vector<std::vector<float>> A, std::vector<std::vector<float>> adj) 
// { 
//   // temp is used to store cofactors of A[][] 
//   bool sign = false;
//   std::vector<std::vector<float>> temp;
//   temp.resize(4, std::vector<float>(4, 0.0f));

//   for (unsigned i=0; i<4; i++) 
//   { 
//       for (unsigned j=0; j<4; j++) 
//       { 
//           // Get cofactor of A[i][j] 
//           getCofactor(A, temp, i, j, 4); 

//           // sign of adj[j][i] positive if sum of row 
//           // and column indexes is even. 
//           sign = ((i+j)%2==0)? true: false; 

//           // Interchanging rows and columns to get the 
//           // transpose of the cofactor matrix 
//           adj[j][i] = sign?(determinant(temp, 3)):-(determinant(temp, 3)); 
//       } 
//   } 
// } 
  
// // Function to calculate and store inverse, returns false if matrix is singular 
// std::vector<std::vector<float>> LocalizationStage::inverse(std::vector<std::vector<float>> A) 
// { 
//   //float inverse[4][4];
//   std::vector<std::vector<float>> inverse;
//   inverse.resize(4, std::vector<float>(4, 0.0f));
//   // Find determinant of A[][] 
//   float det = determinant(A, 4); 
//   if (det <= 0.0001f) 
//   { 
//       //cout << "Singular matrix, can't find its inverse"; 
//       return inverse; 
//   } 

//   // Find adjoint 
//   std::vector<std::vector<float>> adj;
//   adj.resize(4, std::vector<float>(4, 0.0f));
  
//   adjoint(A, adj); 

//   // Find Inverse using formula "inverse(A) = adj(A)/det(A)" 
//   for (unsigned i=0; i<4; i++) 
//       for (unsigned j=0; j<4; j++) 
//           inverse[i][j] = adj[i][j]/float(det); 

//   return inverse; 
// } 
