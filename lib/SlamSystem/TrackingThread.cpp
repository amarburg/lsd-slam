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

#include <boost/thread/shared_lock_guard.hpp>

#include "TrackingThread.h"

#include "SlamSystem.h"

#include "DataStructures/KeyFrame.h"
#include "GlobalMapping/KeyFrameGraph.h"
#include "GlobalMapping/TrackableKeyFrameSearch.h"
#include "Tracking/SE3Tracker.h"
#include "Tracking/TrackingReference.h"

#include "SlamSystem/MappingThread.h"

// for mkdir
#include <sys/stat.h>
#include <sys/types.h>

#include <g3log/g3log.hpp>

#ifdef ANDROID
#include <android/log.h>
#endif

#include "opencv2/opencv.hpp"

using namespace lsd_slam;

using active_object::Active;

TrackingThread::TrackingThread(SlamSystem &system, bool threaded)
    : _system(system), _perf(), _tracker(new SE3Tracker(Conf().slamImageSize)),
      _trackingIsGood(true), _newKeyFramePending(false),
      _latestGoodPoseCamToWorld(),
      _thread(threaded ? Active::createActive() : NULL),
      _currentFrame(nullptr) {
  // Do not use more than 4 levels for odometry tracking
  LOG(INFO) << "PYRAMID_LEVELS" << PYRAMID_LEVELS;
  for (int level = 4; level < PYRAMID_LEVELS; ++level)
    _tracker->settings.maxItsPerLvl[level] = 0;

  lastTrackingClosenessScore = 0;
}

TrackingThread::~TrackingThread() { ; }

void TrackingThread::trackSetImpl(const std::shared_ptr<ImageSet> &set) {
  LOG(WARNING) << "trackSetImpl_here";
  if (!_trackingIsGood) {
    // Prod mapping to check the relocalizer

    //!!TODO.Fix this
    if (!_system.mapThread()->relocalizer.isRunning)
      _system.mapThread()->relocalizer.start(
          _system.keyFrameGraph()->keyframesAll);

    _system.mapThread()->relocalizer.updateCurrentFrame(set->refFrame());
    bool relocResult = _system.mapThread()->relocalizer.waitResult(50);
    LOG(WARNING) << "relocResult " << relocResult;
    if (relocResult) {
      takeRelocalizeResult(_system.mapThread()->relocalizer.getResult());
    }
    //_system.mapThread()->pushDoIteration();

    return;
  }
  LOG(INFO) << "Pre tracking pose " << _latestGoodPoseCamToWorld.matrix3x4();

  // DO TRACKING & Show tracking result.
  LOG_IF(DEBUG, Conf().print.threadingInfo)
      << "TRACKING frame " << set->refFrame()->id() << " onto ref. "
      << _currentKeyFrame->id();

  SE3 frameToParentEstimate =
      se3FromSim3(_currentKeyFrame->pose()->getCamToWorld().inverse() *
                  _latestGoodPoseCamToWorld);

  LOG(DEBUG) << "frame to parent estimate\n: "
             << sim3FromSE3(frameToParentEstimate).matrix3x4();
  // Temp debug
  // Eigen::Matrix3f R;
  // Eigen::Quaterniond q;
  // q.x() = frameToReference_initialEstimate.so3().unit_quaternion().x();
  // q.y() = frameToReference_initialEstimate.so3().unit_quaternion().y();
  // q.z() = frameToReference_initialEstimate.so3().unit_quaternion().z();
  // q.w() = frameToReference_initialEstimate.so3().unit_quaternion().w();
  // if (q.w() < 0) {
  //   q.x() *= -1;
  //   q.y() *= -1;
  //   q.z() *= -1;
  //   q.w() *= -1;
  // }
  // R = q.toRotationMatrix().cast<float>();
  //
  // LOG(WARNING) << "Current frame pose estimate" << R;
  Timer timer;

  LOG(DEBUG) << "Tracking from " << set->id() << " against "
             << _currentKeyFrame->id() << "...";
  SE3 updatedFrameToParent = _tracker->trackFrame(
      _currentKeyFrame, set->refFrame(), frameToParentEstimate);
  LOG(DEBUG) << "   ... done. Tracking took " << timer.stop() * 1000 << " ms";
  _perf.track.update(timer);

  const bool doTrackImageSetFrames = true;
  if (doTrackImageSetFrames && set->size() > 1) {
    Frame::SharedPtr other = set->getFrame(1);
    Sophus::SE3d otherToRef = set->getSE3ToRef(1);

    //    Sophus::SE3d otherToParentEstimate = otherToRef *
    //    updatedParentToFrame.inverse();
    Sophus::SE3d otherToParentEstimate = otherToRef * updatedFrameToParent;

    LOG(DEBUG) << "Before tracking, estimate:\n"
               << otherToParentEstimate.matrix3x4();

    LOG(DEBUG) << "Tracking frame 1...";
    SE3 updatedOtherToParent =
        _tracker->trackFrame(_currentKeyFrame, other, otherToParentEstimate);

    LOG(DEBUG) << "After tracking, gets:\n" << updatedOtherToParent.matrix3x4();
  }
  if (Conf().doLeftRightStereo) {
    LOG(DEBUG) << "Propagating pose from refFrame to others in set";
    set->propagatePoseFromRefFrame();
  }

  tracking_lastResidual = _tracker->lastResidual;
  tracking_lastUsage = _tracker->pointUsage;
  LOG(INFO) << "manualTrackingLossIndicated " << manualTrackingLossIndicated;
  if (manualTrackingLossIndicated || _tracker->diverged ||
      (_system.keyFrameGraph()->keyframesAll.size() >
           INITIALIZATION_PHASE_COUNT &&
       !_tracker->trackingWasGood)) {
    LOGF(WARNING,
         "TRACKING LOST for frame %d (%1.2f%% good Points, which is %1.2f%% of "
         "available points; %s tracking; tracker has %s)!\n",
         set->refFrame()->id(), 100 * _tracker->_pctGoodPerTotal,
         100 * _tracker->_pctGoodPerGoodBad,
         _tracker->trackingWasGood ? "GOOD" : "BAD",
         _tracker->diverged ? "DIVERGED" : "NOT DIVERGED");

    //_trackingReference->invalidate();
    setTrackingIsBad();

    //!!TODO.  What does mapping thread do while tracking is bad?
    //_system.mapThread->pushDoIteration();
    manualTrackingLossIndicated = false;
    return;
  }
  std::chrono::duration<double, std::milli> duration =
      _latestTime - std::chrono::high_resolution_clock::now();

  double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                       std::chrono::system_clock::now() - _latestTime)
                       .count() /
                   1000.0;

  LOG(INFO) << "elapsed " << elapsed;

  Eigen::Matrix<float, 6, 1> motion =
      calculateVelocity(_latestGoodPoseCamToWorld.cast<float>(),
                        set->refFrame()->pose->getCamToWorld().cast<float>());
  motion /= elapsed;

  Eigen::Matrix<float, 6, 1> acceleration =
      calculateAcceleration(_latestGoodMotion, motion);
  acceleration /= elapsed;
  _latestGoodMotion = motion;

  _system.publishStateEstimation(motion, acceleration);

  LOG(INFO) << "motion: " << std::endl << motion;

  // SE3 _motion = se3FromSim3(set->refFrame()->pose->getCamToWorld()) -
  //               se3FromSim3(_latestGoodPoseCamToWorld);

  // TODO here's why the EKF addition will go
  _latestGoodPoseCamToWorld = set->refFrame()->pose->getCamToWorld();
  _latestTime = std::chrono::system_clock::now();

  LOG_IF(DEBUG, Conf().print.threadingInfo) << "Publishing tracked frame";
  _system.publishTrackedFrame(set->refFrame(), frameToParentEstimate);
  _system.publishPose(set->refFrame()->getCamToWorld().cast<float>());
  _currentFrame = set->refFrame();
  //
  LOG(INFO) << "Post tracking pose " << _latestGoodPoseCamToWorld.matrix3x4();

  // Keyframe selection
  LOG(INFO) << "Tracked " << set->id() << " against keyframe "
            << _currentKeyFrame->id();
  LOG_IF(INFO, Conf().print.threadingInfo)
      << _currentKeyFrame->numMappedOnThisTotal
      << " frames mapped on to keyframe " << _currentKeyFrame->id()
      << ", considering " << set->refFrame()->id() << " as new keyframe.";

  // Push to mapping before checking if its the new keyframe
  _system.mapThread()->doMapSet(_currentKeyFrame, set);
  LOG(DEBUG) << "_newKeyFramePending " << _newKeyFramePending;
  LOG(DEBUG) << "numMappedOnThisTotal "
             << _currentKeyFrame->numMappedOnThisTotal << " Min num"
             << MIN_NUM_MAPPED;
  if (!_newKeyFramePending &&
      _currentKeyFrame->numMappedOnThisTotal > MIN_NUM_MAPPED) {

    Sophus::Vector3d dist = updatedFrameToParent.translation() *
                            _currentKeyFrame->frame()->meanIdepth;
    float minVal = fmin(0.2f + _system.keyFrameGraph()->size() * 0.8f /
                                   INITIALIZATION_PHASE_COUNT,
                        1.0f);

    if (_system.keyFrameGraph()->size() < INITIALIZATION_PHASE_COUNT)
      minVal *= 0.7;

    lastTrackingClosenessScore =
        _system.trackableKeyFrameSearch()->getRefFrameScore(
            dist.dot(dist), _tracker->pointUsage);

    LOG(DEBUG) << "lastTrackingClosenessScore " << lastTrackingClosenessScore
               << " min val" << minVal;
    if (lastTrackingClosenessScore > minVal) {
      LOG(INFO) << "Telling mapping thread to make " << set->refFrame()->id()
                << " the new keyframe.";

      _newKeyFramePending = true;
      _system.mapThread()->doCreateNewKeyFrame(_currentKeyFrame, set);

      LOGF_IF(INFO, Conf().print.keyframeSelectionInfo,
              "SELECT KEYFRAME %d on %d! dist %.3f + usage %.3f = %.3f > 1\n",
              set->refFrame()->id(), set->refFrame()->trackingParent()->id(),
              dist.dot(dist), _tracker->pointUsage,
              _system.trackableKeyFrameSearch()->getRefFrameScore(
                  dist.dot(dist), _tracker->pointUsage));
    } else {
      LOGF_IF(INFO, Conf().print.keyframeSelectionInfo,
              "SKIPPD KEYFRAME %d on %d! dist %.3f + usage %.3f = %.3f > 1\n",
              set->refFrame()->id(), set->refFrame()->trackingParent()->id(),
              dist.dot(dist), _tracker->pointUsage,
              _system.trackableKeyFrameSearch()->getRefFrameScore(
                  dist.dot(dist), _tracker->pointUsage));
    }
  }

  LOG_IF(DEBUG, Conf().print.threadingInfo) << "Exiting trackFrame";
}

void TrackingThread::useNewKeyFrameImpl(const std::shared_ptr<KeyFrame> &kf) {
  LOG(DEBUG) << "Using " << kf->id() << " as new keyframe";
  _newKeyFramePending = false;
  _currentKeyFrame = kf;
}

Eigen::Matrix<float, 6, 1> TrackingThread::calculateVelocity(Sophus::Sim3f p1,
                                                             Sophus::Sim3f p2) {
  Eigen::Matrix3f R1 = p1.rotationMatrix();
  Eigen::Vector3f T1 = p1.translation();
  Eigen::Vector3f angles1 = R1.eulerAngles(2, 1, 0);

  Eigen::Matrix3f R2 = p2.rotationMatrix();
  Eigen::Vector3f T2 = p2.translation();
  Eigen::Vector3f angles2 = R2.eulerAngles(2, 1, 0);
  LOG(INFO) << "R2: " << std::endl << R2;
  LOG(INFO) << "angles2: " << std::endl << angles2;

  Eigen::Vector3f tanslation = T2 - T1;
  Eigen::Vector3f rotation;
  rotation(0) = angles2(2) - angles1(2);
  rotation(1) = angles2(1) - angles1(1);
  rotation(2) = angles2(0) - angles1(0);

  Eigen::Matrix<float, 6, 1> motion;
  motion.block<3, 1>(0, 0) = tanslation;
  motion.block<3, 1>(3, 0) = rotation;

  return motion;
}

Eigen::Matrix<float, 6, 1>
TrackingThread::calculateAcceleration(Eigen::Matrix<float, 6, 1> m1,
                                      Eigen::Matrix<float, 6, 1> m2) {

  return m2 - m1;
}

// n.b. this function will be called from the mapping thread.  Ensure
// locking is in place.

// TODO I don't think this is ever entered?
// Need to add pushUnmappedTrackedFrame for image set
void TrackingThread::takeRelocalizeResult(const RelocalizerResult &result) {
  LOG(WARNING) << "Entering takeRelocalizeResult";
  int succFrameID;
  SE3 succFrameToKF_init;
  std::shared_ptr<Frame> succFrame;

  _system.mapThread()->relocalizer.stop();
  //
  // //_trackingReference->importFrame(keyframe);
  // //_trackingReferenceFrameSharedPT = keyframe;

  _tracker->trackFrame(result.keyframe, result.successfulFrame,
                       result.successfulFrameToKeyframe);

  if (!_tracker->trackingWasGood ||
      _tracker->lastGoodCount() / (_tracker->lastGoodCount()) <
          1 - 0.75f * (1 - MIN_GOODPERGOODBAD_PIXEL)) {
    LOG(DEBUG) << "RELOCALIZATION FAILED BADLY! discarding result.";
    //_trackingReference->invalidate();
  } else {
    LOG(DEBUG) << "GOOD RELOCALIZATION!";
    // KeyFrame::SharedPtr kf(KeyFrame::Create(set));
    // KeyFrame::SharedPtr keyframe(KeyFrame::Create(result.successfulFrame));
    _currentKeyFrame = result.keyframe;
    //_currentFrame = result.successfulFrame;
    //_system.keyFrameGraph()->addKeyFrame(keyframe);

    _latestGoodPoseCamToWorld = _currentKeyFrame->pose()->getCamToWorld();

    // TODO commenting this out in the assumption I don't need it... need to
    // revist
    //_system.mapThread()->pushUnmappedTrackedFrame(result.successfulFrame);
    _newKeyFramePending = false;
    //_system.mapThread()->doCreateNewKeyFrame(_currentKeyFrame, set);
    // std::lock_guard<std::mutex> lock(currentKeyFrameMutex);
    // createNewKeyFrame = false;
    setTrackingIsGood();
  }
}
