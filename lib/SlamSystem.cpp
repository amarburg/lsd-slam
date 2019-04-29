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

#include <memory>

#include "SlamSystem.h"

#include <boost/thread/shared_lock_guard.hpp>

#include <g3log/g3log.hpp>

#ifdef ANDROID
#include <android/log.h>
#endif

#include <opencv2/opencv.hpp>

#include "DataStructures/Frame.h"
#include "DataStructures/FrameMemory.h"
#include "GlobalMapping/KeyFrameGraph.h"
#include "GlobalMapping/TrackableKeyFrameSearch.h"
#include "util/globalFuncs.h"

#include "SlamSystem/ConstraintSearchThread.h"
#include "SlamSystem/MappingThread.h"
#include "SlamSystem/OptimizationThread.h"

using namespace lsd_slam;

SlamSystem::SlamSystem()
    : _perf(), _outputWrappers(), _finalized(), _initialized(false),
      //	_keyFrames(),
      _keyFrameGraph(new KeyFrameGraph),
      _trackableKeyFrameSearch(new TrackableKeyFrameSearch(_keyFrameGraph)) {

  // Because some of these rely on Conf(), need to explicitly call after
  // static initialization.  Is this true?
  const bool threaded = Conf().runRealTime;
  _optThread.reset(new OptimizationThread(*this, threaded));
  _mapThread.reset(new MappingThread(*this, threaded));
  _constraintThread.reset(new ConstraintSearchThread(*this, threaded));
  _trackingThread.reset(new TrackingThread(*this, threaded));

  timeLastUpdate.start();
}

SlamSystem::~SlamSystem() {
  // make sure no-one is waiting for something.
  LOG(INFO) << "... waiting for all threads to exit";

  _mapThread.reset();
  _constraintThread.reset();
  _optThread.reset();
  _trackingThread.reset();
  LOG(INFO) << "DONE waiting for all threads to exit";

  FrameMemory::getInstance().releaseBuffers();
}

SlamSystem *SlamSystem::fullReset(void) {
  SlamSystem *newSystem = new SlamSystem();
  for (auto &wrapper : _outputWrappers) {
    newSystem->addOutputWrapper(wrapper);
  }
  return newSystem;
}

void SlamSystem::finalize() {
  LOG(INFO) << "Finalizing Graph... adding final constraints!!";

  // Run this in the foreground
  _constraintThread->doFullReConstraintSearch();
  _constraintThread->fullReConstraintSearchComplete.wait();

  LOG(INFO) << "Finalizing Graph... optimizing!!";

  // This happens in the foreground
  // This will kick off a final map publication with the newly optimized offsets
  // (also in foreground)
  _optThread->doFinalOptimization();

  _optThread->finalOptimizationComplete.wait();
  _mapThread->optimizationUpdateMerged.wait();

  LOG(INFO) << "Done Finalizing Graph.!!";
  _finalized.notify();

  LOG(INFO) << "Done Finalizing Graph.!!";
  _finalized.notify();
}

// std::shared_ptr<KeyFrame> &SlamSystem::currentKeyFrame()

// Thin wrapper which turns a bare cv::Mat image into an ImageSe
void SlamSystem::nextImage(unsigned int id, const cv::Mat &img,
                           const libvideoio::Camera &cam) {
  nextImageSet(std::make_shared<ImageSet>(id, img, cam));
}

void SlamSystem::nextImageSet(const std::shared_ptr<ImageSet> &set) {
  if (!_initialized) {
    _mapThread->createFirstKeyFrame(set);
    _initialized = true;
    return;
  }

  _trackingThread->doTrackSet(set);

  logPerformanceData();
}

//=== Keyframe maintenance functions ====

// void SlamSystem::addKeyFrame( const KeyFrame::SharedPtr &keyframe )
// {
// 	_keyFrames.push_back( keyframe );
//
//
// 	keyFrameGraph()->idToKeyFrame.insert(std::make_pair(keyframe->id(),
// keyframe));
// }

// void SlamSystem::changeKeyframe( const Frame::SharedPtr &candidate, bool
// noCreate, bool force, float maxScore)
// {
// 	Frame::SharedPtr newReferenceKF(nullptr);
//
// 	if( Conf().doKFReActivation && Conf().SLAMEnabled )
// 	{
// 		Timer timer;
// 		newReferenceKF =
// trackableKeyFrameSearch()->findRePositionCandidate( candidate, maxScore );
// 		_perf.findReferences.update( timer );
// 	}
//
// 	if(newReferenceKF != 0) {
// 		LOG(INFO) << "Reloading existing key frame " <<
// newReferenceKF->id(); 		loadNewCurrentKeyframe(newReferenceKF);
// } else { 		if(force)
// 		{
// 			if(noCreate)
// 			{
// 				LOG(INFO) << "mapping is disabled & moved
// outside of known map. Starting Relocalizer!";
// trackingThread->setTrackingIsBad();
// 				//nextRelocIdx = -1; /// What does this do?
// 			}
// 			else
// 			{
// 				createNewCurrentKeyframe( candidate );
// 			}
// 		}
// 	}
// 	// createNewKeyFrame = false;
// }
//
// void SlamSystem::loadNewCurrentKeyframe( const Frame::SharedPtr
// &keyframeToLoad)
// {
// 	depthMap()->activateExistingKF(keyframeToLoad);
//
// 	LOG_IF(DEBUG, Conf().print.regularizeStatistics ) << "re-activate frame
// " << keyframeToLoad->id() << "!";
//
// 	// Not entirely sure why they're doing this lookup...
// 	//_currentKeyFrame =
// keyFrameGraph()->idToKeyFrame.find(keyframeToLoad->id())->second;
// 	//currentKeyFrame()->depthHasBeenUpdatedFlag = false;
// }
//
//
// void SlamSystem::createNewCurrentKeyframe( const Frame::SharedPtr
// &newKeyframeCandidate)
// {
// 	LOG_IF(INFO, Conf().print.threadingInfo) << "CREATE NEW KF " <<
// newKeyframeCandidate->id() << ", replacing " << currentKeyFrame()->id();
//
// 	if( Conf().SLAMEnabled)
// 	{
// 		boost::shared_lock_guard< boost::shared_mutex > lock(
// keyFrameGraph()->idToKeyFrameMutex );
// 		keyFrameGraph()->idToKeyFrame.insert(std::make_pair(newKeyframeCandidate->id(),
// newKeyframeCandidate));
// 	}
//
// 	// propagate & make new.
// 	depthMap()->createKeyFrame(newKeyframeCandidate);
// }

//===== Debugging output functions =====

void SlamSystem::logPerformanceData() {
  float sPassed = timeLastUpdate.reset();
  if (sPassed > 1.0f) {

    // LOGF(DEBUG, "Mapping: %3.1fms (%.1fHz); Track: %3.1fms (%.1fHz); Create:
    // %3.1fms (%.1fHz); FindRef: %3.1fms (%.1fHz); PermaTrk: %3.1fms (%.1fHz);
    // Opt: %3.1fms (%.1fHz); FindConst: %3.1fms (%.1fHz);\n",
    // 			depthMap()->perf().update.ms(),
    // depthMap()->perf().update.rate(),
    // _trackingThread->perf().track.ms(), _trackingThread->perf().track.rate(),
    // 			depthMap()->perf().create.ms()+depthMap()->perf().finalize.ms(),
    // depthMap()->perf().create.rate(),
    // _perf.findReferences.ms(), _perf.findReferences.rate(),
    // 0.0, 0.0,
    // 			//trackableKeyFrameSearch != 0 ?
    // trackableKeyFrameSearch->trackPermaRef.ms() : 0, trackableKeyFrameSearch
    // != 0 ? trackableKeyFrameSearch->trackPermaRef.rate() : 0,
    // 			_optThread->perf.ms(), _optThread->perf.rate(),
    // 			_constraintThread->perf().findConstraint.ms(),
    // _constraintThread->perf().findConstraint.rate() );
    //
    // depthMap()->logPerformanceData();
  }
}

void SlamSystem::updateDisplayDepthMap() {
  if (!Conf().displayDepthMap)
    return;

  const double scale = (bool)currentKeyFrame()
                           ? currentKeyFrame()->frame()->getCamToWorld().scale()
                           : 1.0;

  // debug plot depthmap
  char buf1[200] = "";
  char buf2[200] = "";

  if (Conf().onSceenInfoDisplay) {
    // snprintf(buf1,200,"Map: Upd %3.0fms (%2.0fHz); Trk %3.0fms (%2.0fHz); %d
    // / %d", 		depthMap()->perf().update.ms(),
    // depthMap()->perf().update.rate(),
    // _trackingThread->perf().track.ms(), _trackingThread->perf().track.rate(),
    // 		currentKeyFrame()->numFramesTrackedOnThis,
    // currentKeyFrame()->numMappedOnThis ); //,
    // (int)unmappedTrackedFrames().size());

    // snprintf(buf2,200,"dens %2.0f%%; good %2.0f%%; scale %2.2f; res %2.1f/;
    // usg %2.0f%%; Map: %d F, %d KF, %d E, %.1fm Pts",
    // 		100*currentKeyFrame->numPoints/(float)(Conf().slamImage.area()),
    // 		100*tracking_lastGoodPerBad,
    // 		scale,
    // 		tracking_lastResidual,
    // 		100*tracking_lastUsage,
    // 		(int)keyFrameGraph()->allFramePoses.size(),
    // 		keyFrameGraph()->totalVertices,
    // 		(int)keyFrameGraph()->edgesAll.size(),
    // 		1e-6 * (float)keyFrameGraph()->totalPoints);
  }

  currentKeyFrame()->depthMap()->plotDepthMap(buf1, buf2);

  CHECK(currentKeyFrame()->depthMap()->debugImages().depthImage().data != NULL);
  publishDepthImage(
      currentKeyFrame()->depthMap()->debugImages().depthImage().data);
}

//=== OutputWrapper functions ==

#define OUTPUT_FOR_EACH(func)                                                  \
  for (auto &wrapper : _outputWrappers) {                                      \
    wrapper->func;                                                             \
  }

void SlamSystem::publishPose(const Sophus::Sim3f &pose) {
  Eigen::Matrix3f R = pose.rotationMatrix();
  Eigen::Vector3f T = pose.translation();
  // R = q.toRotationMatrix().cast<float>();

  OUTPUT_FOR_EACH(publishPose(pose))
}

void SlamSystem::publishTrackedFrame(const Frame::SharedPtr &frame) {
  OUTPUT_FOR_EACH(publishTrackedFrame(frame))
}

void SlamSystem::publishKeyframeGraph(void) {
  OUTPUT_FOR_EACH(publishKeyframeGraph(keyFrameGraph()))
}

void SlamSystem::publishDepthImage(unsigned char *data) {
  OUTPUT_FOR_EACH(updateDepthImage(data))
}

void SlamSystem::publishKeyframe(const Frame::SharedPtr &frame) {
  OUTPUT_FOR_EACH(publishKeyframe(frame))
}

void SlamSystem::publishCurrentKeyframe() {
  if (currentKeyFrame()) {
    OUTPUT_FOR_EACH(publishKeyframe(currentKeyFrame()->frame()))
    OUTPUT_FOR_EACH(publishPointCloud(currentKeyFrame()->frame()))
  } else {
    LOG(DEBUG) << "No currentKeyframe, unable to publish";
  }
}

void SlamSystem::publishCurrentFrame() {
  if (currentKeyFrame()) {
    Eigen::Matrix4f G = frameToWorld();
    OUTPUT_FOR_EACH(publishFrame(currentKeyFrame()->frame(), G))
    // OUTPUT_FOR_EACH(publishPointCloud(currentKeyFrame()))
  } else {
    LOG(DEBUG) << "No currentKeyframe, unable to publish";
  }
}

Eigen::Matrix4f SlamSystem::frameToWorld() {
  SE3 FToKF = se3FromSim3(currentFrame()->getCamToWorld());
  Eigen::Matrix4f G_f_kf = Eigen::Matrix4f::Zero(4, 4);
  Eigen::Matrix3f R;
  Eigen::Vector3f T;
  Eigen::Quaterniond q;
  q.x() = FToKF.so3().unit_quaternion().x();
  q.y() = FToKF.so3().unit_quaternion().y();
  q.z() = FToKF.so3().unit_quaternion().z();
  q.w() = FToKF.so3().unit_quaternion().w();
  if (q.w() < 0) {
    q.x() *= -1;
    q.y() *= -1;
    q.z() *= -1;
    q.w() *= -1;
  }
  R = q.toRotationMatrix().cast<float>();
  T(0) = FToKF.translation()[0];
  T(1) = FToKF.translation()[1];
  T(2) = FToKF.translation()[2];
  G_f_kf.block<3, 3>(0, 0) = R;
  G_f_kf.block<3, 1>(0, 3) = T;
  G_f_kf(3, 3) = 1.0;

  SE3 KFToW = se3FromSim3(currentKeyFrame()->getCamToWorld());
  Eigen::Matrix4f G_kf_world = Eigen::Matrix4f::Zero(4, 4);
  q.x() = KFToW.so3().unit_quaternion().x();
  q.y() = KFToW.so3().unit_quaternion().y();
  q.z() = KFToW.so3().unit_quaternion().z();
  q.w() = KFToW.so3().unit_quaternion().w();
  if (q.w() < 0) {
    q.x() *= -1;
    q.y() *= -1;
    q.z() *= -1;
    q.w() *= -1;
  }
  R = q.toRotationMatrix().cast<float>();
  T(0) = KFToW.translation()[0];
  T(1) = KFToW.translation()[1];
  T(2) = KFToW.translation()[2];
  G_kf_world.block<3, 3>(0, 0) = R;
  G_kf_world.block<3, 1>(0, 3) = T;
  G_kf_world(3, 3) = 1.0;

  Eigen::Matrix4f G = G_kf_world * G_f_kf;

  return G;
}
