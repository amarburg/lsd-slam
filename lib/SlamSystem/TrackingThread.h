/**
 * This file is derived from Jakob Engel's original LSD-SLAM code.
 * His original copyright notice follows:
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

#include "active_object/active.h"

#include "DataStructures/ImageSet.h"
#include "DataStructures/KeyFrame.h"
#include "Tracking/Relocalizer.h"
#include "util/MovingAverage.h"

// #include <Eigen/core>

#include <chrono>

namespace lsd_slam {

class SlamSystem;
class SE3Tracker;

typedef Eigen::Matrix<float, 7, 7> Matrix7x7;

class TrackingThread {
  friend class IntegrationTest;

public:
  // Delete these
  TrackingThread(const TrackingThread &) = delete;
  TrackingThread &operator=(const TrackingThread &) = delete;

  TrackingThread(SlamSystem &system, bool threaded);

  ~TrackingThread();

  //== Calls into the thread ==
  void doTrackSet(const std::shared_ptr<ImageSet> &set) {

    if (_thread) {
      _thread->send(std::bind(&TrackingThread::trackSetImpl, this, set));
    } else {
      trackSetImpl(set);
    }
  }

  void doUseNewKeyFrame(const std::shared_ptr<KeyFrame> &kf) {
    if (_thread) {
      _thread->send(std::bind(&TrackingThread::useNewKeyFrameImpl, this, kf));
    } else {
      useNewKeyFrameImpl(kf);
    }
  }

  KeyFrame::SharedPtr &currentKeyFrame(void) { return _currentKeyFrame; }
  Frame::SharedPtr &currentFrame(void) { return _currentFrame; }

  int findConstraintsForNewKeyFrames(Frame *newKeyFrame,
                                     bool forceParent = true,
                                     bool useFABMAP = true,
                                     float closeCandidatesTH = 1.0);

  // void changeKeyframe(std::shared_ptr<Frame> candidate, bool noCreate, bool
  // force, float maxScore);

  void takeRelocalizeResult(const RelocalizerResult &result);

  float lastTrackingClosenessScore;

  bool trackingIsGood(void) const { return _trackingIsGood; }
  bool setTrackingIsBad(void) {
    LOG(DEBUG) << "Setting tracking bad";
    return _trackingIsGood = false;
  }
  bool setTrackingIsGood(void) {
    LOG(DEBUG) << "Setting tracking good";
    return _trackingIsGood = true;
  }

  struct PerformanceData {
    MsRateAverage track;
  };

  PerformanceData perf() const { return _perf; }

private:
  SlamSystem &_system;
  PerformanceData _perf;

  std::unique_ptr<SE3Tracker> _tracker;

  // Thread Callbacks
  void trackSetImpl(const std::shared_ptr<ImageSet> &set);

  void useNewKeyFrameImpl(const std::shared_ptr<KeyFrame> &kf);

  Eigen::Matrix<float, 6, 1> calculateVelocity(Sophus::Sim3f p1,
                                               Sophus::Sim3f p2);
  Eigen::Matrix<float, 6, 1>
  calculateAcceleration(Eigen::Matrix<float, 6, 1> m1,
                        Eigen::Matrix<float, 6, 1> m2);

  bool _trackingIsGood;
  bool _newKeyFramePending;

  KeyFrame::SharedPtr _currentKeyFrame;
  Frame::SharedPtr _currentFrame;

  Sim3 _latestGoodPoseCamToWorld;
  Sim3 _latestGoodPoseFrameToParent;

  Eigen::Matrix<float, 6, 1> _latestGoodMotion;

  std::chrono::high_resolution_clock::time_point _latestTime =
      std::chrono::high_resolution_clock::now();

  // // ============= SHARED ENTITIES =============
  float tracking_lastResidual;
  float tracking_lastUsage;
  float tracking_lastGoodPerBad;
  float tracking_lastGoodPerTotal;

  std::unique_ptr<active_object::Active> _thread;
};

} // namespace lsd_slam
