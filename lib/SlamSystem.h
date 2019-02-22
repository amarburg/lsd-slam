/**
* This file is part of LSD-SLAM.
*
* Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical University of Munich)
* For more information see <http://vision.in.tum.de/lsdslam>
*
* LSD-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* LSD-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with LSD-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once
#include <vector>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <boost/thread/shared_mutex.hpp>
#include <memory>
#include <chrono>

#include "util/settings.h"
#include "IOWrapper/Timestamp.h"
#include "opencv2/core/core.hpp"

#include "IOWrapper/Output3DWrapper.h"

#include "DataStructures/Frame.h"
#include "DataStructures/ImageSet.h"

#include "DepthEstimation/DepthMap.h"

#include "util/SophusUtil.h"
#include "util/MovingAverage.h"
#include "util/Configuration.h"
#include "util/Timer.h"
#include "util/ThreadMutexObject.h"

#include "Tracking/Relocalizer.h"

namespace lsd_slam
{

	class KeyFrameGraph;
	class Output3DWrapper;
	class FramePoseStruct;
	class TrackableKeyFrameSearch;

	class TrackingThread;
	class OptimizationThread;
	class MappingThread;
	class ConstraintSearchThread;

	using std::unique_ptr;
	using std::shared_ptr;

class SlamSystem {

friend class IntegrationTest;
public:

	SlamSystem();

	SlamSystem( const SlamSystem&) = delete;
	SlamSystem& operator=(const SlamSystem&) = delete;

	~SlamSystem();

	// Creates a new SlamSystem, and passes over relevant configuration info
	SlamSystem *fullReset();

	// OLD API: tracks a frame.
	// first frame will return Identity = camToWord.
	// returns camToWord transformation of the tracked frame.
	// frameID needs to be monotonically increasing.
	//void trackFrame( Frame *newFrame ); //, bool blockUntilMapped );

	void nextImage( unsigned int id, const cv::Mat &img, const libvideoio::Camera &cam );
	void nextImageSet( const std::shared_ptr<ImageSet> &set );


	// finalizes the system, i.e. blocks and does all remaining loop-closures etc.
	void finalize();
	ThreadSynchronizer &finalized() { return _finalized; }

	/** Returns the current pose estimate. */
	Sophus::SE3d getCurrentPoseEstimate();

	Sophus::Sim3d getCurrentPoseEstimateScale();

	//==== KeyFrame maintenance functions ====
        Frame::SharedPtr &currentKeyFrame() { return depthMap()->currentKeyFrame(); }

	void changeKeyframe( const Frame::SharedPtr &frame, bool noCreate, bool force, float maxScore);
	void loadNewCurrentKeyframe( const Frame::SharedPtr &keyframeToLoad );
	void createNewCurrentKeyframe( const Frame::SharedPtr &newKeyframeCandidate );

	// void requestDepthMapScreenshot(const std::string& filename);

	// int findConstraintsForNewKeyFrames(Frame* newKeyFrame, bool forceParent=true, bool useFABMAP=true, float closeCandidatesTH=1.0);

	std::vector<FramePoseStruct::SharedPtr> getAllPoses();

	//=== Debugging output functions =====
	const shared_ptr<Output3DWrapper> &outputWrapper( void )       { return _outputWrapper; }
	void set3DOutputWrapper( Output3DWrapper* outputWrapper )      {	_outputWrapper.reset(outputWrapper); }
	void set3DOutputWrapper( const shared_ptr<Output3DWrapper> &outputWrapper) {	_outputWrapper = outputWrapper; }

	void publishPose(const Sophus::Sim3f &pose ) 	                 { if( _outputWrapper ) _outputWrapper->publishPose(pose);}
	void publishTrackedFrame( const Frame::SharedPtr &frame )      { if( _outputWrapper ) _outputWrapper->publishTrackedFrame( frame ); }
	void publishKeyframeGraph( void )                              { if( _outputWrapper ) _outputWrapper->publishKeyframeGraph( keyFrameGraph() ); }
	void publishKeyframe(  const Frame::SharedPtr &frame );
	void publishCurrentKeyframe();
	void publishPointCloud();
	void publishDepthImage( unsigned char* data  )                 { if( _outputWrapper ) _outputWrapper->updateDepthImage( data ); }

	void updateDisplayDepthMap();

	unique_ptr<TrackingThread> trackingThread;
	unique_ptr<OptimizationThread> optThread;
	unique_ptr<MappingThread> mapThread;
	unique_ptr<ConstraintSearchThread> constraintThread;

	// mutex to lock frame pose consistency. within a shared lock of this, *->getCamToWorld() is
	// GUARANTEED to give the same result each call, and to be compatible to each other.
	// locked exclusively during the pose-update by Mapping.
	boost::shared_mutex poseConsistencyMutex;

	const shared_ptr<KeyFrameGraph> &keyFrameGraph() { return _keyFrameGraph; };	  // has own locks
	shared_ptr<TrackableKeyFrameSearch> &trackableKeyFrameSearch() { return _trackableKeyFrameSearch; }

	unique_ptr<DepthMap> &depthMap() { return _depthMap; }


private:

	struct PerformanceData {
		MsRateAverage findReferences;
	} _perf;

	Timer timeLastUpdate;

	std::shared_ptr<Output3DWrapper> _outputWrapper;	// no lock required

	ThreadSynchronizer _finalized;
	bool _initialized;

	// ======= Functions =====

	void initialize( const std::shared_ptr<ImageSet> &set );
	void logPerformanceData();

	// == Shared "global" data structures ==

	std::shared_ptr<KeyFrameGraph> _keyFrameGraph;	  // has own locks
	//Frame::SharedPtr  _currentKeyFrame;

	std::shared_ptr<TrackableKeyFrameSearch> _trackableKeyFrameSearch;

	std::unique_ptr<DepthMap> _depthMap;

};

}
