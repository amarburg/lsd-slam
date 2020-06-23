/**
 * This file is part of LSD-SLAM.
 *
 * Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical
 * University of Munich) For more information see
 * <http://vision.in.tum.de/lsdslam>
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
#include <boost/thread/shared_mutex.hpp>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "opencv2/core/core.hpp"
#include "util/settings.h"

#include "IOWrapper/OutputIOWrapper.h"

#include "DataStructures/Frame.h"
#include "DataStructures/ImageSet.h"
#include "DataStructures/KeyFrame.h"

#include "DepthEstimation/DepthMap.h"

#include "util/Configuration.h"
#include "util/MovingAverage.h"
#include "util/SophusUtil.h"
#include "util/ThreadMutexObject.h"
#include "util/Timer.h"

#include "SlamSystem/TrackingThread.h"

#include "Tracking/Relocalizer.h"

namespace lsd_slam {

class KeyFrameGraph;
class Output3DWrapper;
class FramePoseStruct;
class TrackableKeyFrameSearch;

class TrackingThread;
class OptimizationThread;
class MappingThread;
class ConstraintSearchThread;

using std::shared_ptr;
using std::unique_ptr;

class SlamSystem {

friend class IntegrationTest;

public:
SlamSystem();

SlamSystem(const SlamSystem &) = delete;
SlamSystem &operator=(const SlamSystem &) = delete;

~SlamSystem();

// Creates a new SlamSystem, and passes over relevant configuration info
SlamSystem *fullReset();

void nextImage(unsigned int id, const cv::Mat &img,
               const libvideoio::Camera &cam);
void nextImageSet(const std::shared_ptr<ImageSet> &set);

// finalizes the system, i.e. blocks and does all remaining loop-closures etc.
void finalize();
ThreadSynchronizer &finalized() {
		return _finalized;
}

//==== KeyFrame maintenance functions ====
std::shared_ptr<KeyFrame> &currentKeyFrame() {
		return trackingThread()->currentKeyFrame();
}

std::shared_ptr<Frame> &currentFrame() {
		return trackingThread()->currentFrame();
}

// int findConstraintsForNewKeyFrames(Frame* newKeyFrame, bool
// forceParent=true, bool useFABMAP=true, float closeCandidatesTH=1.0);

std::vector<FramePoseStruct::SharedPtr> getAllPoses();

//=== Debugging output functions =====
void addOutputWrapper(const shared_ptr<OutputIOWrapper> &outputWrapper) {
		_outputWrappers.push_back(outputWrapper);
}

void publishPose(const Sophus::Sim3f &pose);
void publishStateEstimation( const Sophus::Sim3f &pose,
                             const Sophus::SE3f &motion,
                             double timeElapsed );

void publishTrackedFrame(const Frame::SharedPtr &frame,
                         SE3 frameToParentEstimate);
void publishKeyframeGraph(void);
void publishKeyframe(const Frame::SharedPtr &frame);
void publishCurrentKeyframe();
void publishCurrentFrame();
void publishDepthImage(unsigned char *data);

void updateDisplayDepthMap();

// mutex to lock frame pose consistency. within a shared lock of this,
// *->getCamToWorld() is GUARANTEED to give the same result each call, and to
// be compatible to each other. locked exclusively during the pose-update by
// Mapping.
boost::shared_mutex poseConsistencyMutex;

const shared_ptr<KeyFrameGraph> &keyFrameGraph() {
		return _keyFrameGraph;
};   // has own locks
shared_ptr<TrackableKeyFrameSearch> &trackableKeyFrameSearch() {
		return _trackableKeyFrameSearch;
}

unique_ptr<MappingThread> &mapThread() {
		return _mapThread;
}
unique_ptr<TrackingThread> &trackingThread() {
		return _trackingThread;
}
unique_ptr<OptimizationThread> &optThread() {
		return _optThread;
}
unique_ptr<ConstraintSearchThread> &constraintThread() {
		return _constraintThread;
}

private:
struct PerformanceData {
		MsRateAverage findReferences;
} _perf;

Timer timeLastUpdate;

std::list<std::shared_ptr<OutputIOWrapper> > _outputWrappers;

bool _initialized;
ThreadSynchronizer _finalized;

// ======= Functions =====
void logPerformanceData();
Eigen::Matrix4f frameToWorld();

//== Component threads
unique_ptr<TrackingThread> _trackingThread;
unique_ptr<OptimizationThread> _optThread;
unique_ptr<MappingThread> _mapThread;
unique_ptr<ConstraintSearchThread> _constraintThread;

// == Shared "global" data structures ==
std::shared_ptr<KeyFrameGraph> _keyFrameGraph;   // has own locks
std::shared_ptr<TrackableKeyFrameSearch> _trackableKeyFrameSearch;
};

} // namespace lsd_slam
