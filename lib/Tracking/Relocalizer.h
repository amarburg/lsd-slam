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
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <stdio.h>

#include "boost/thread.hpp"
#include "util/Configuration.h"
#include "util/SophusUtil.h"
#include "util/settings.h"

#include "DataStructures/Frame.h"

namespace lsd_slam {

class Sim3Tracker;
class KeyFrame;

struct RelocalizerResult {

  // Null constructor means "no result"
  RelocalizerResult()
      : keyframe(nullptr), successfulFrame(nullptr), successfulFrameID(-1),
        successfulFrameToKeyframe(SE3()) {
    ;
  }

  RelocalizerResult(const std::shared_ptr<KeyFrame> &out_kf,
                    Frame::SharedPtr &f, int out_id, SE3 out_se3)
      : keyframe(out_kf), successfulFrame(f), successfulFrameID(out_id),
        successfulFrameToKeyframe(out_se3) {
    ;
  }

  std::shared_ptr<KeyFrame> keyframe;
  Frame::SharedPtr successfulFrame;
  int successfulFrameID;
  SE3 successfulFrameToKeyframe;
};

class Relocalizer {
public:
  Relocalizer();
  ~Relocalizer();

  void updateCurrentFrame(std::shared_ptr<Frame> currentFrame);
  void start(std::vector<std::shared_ptr<KeyFrame>> &allKeyframesList);
  void stop();

  bool waitResult(int milliseconds);
  RelocalizerResult
  getResult(); // Frame* &out_keyframe, std::shared_ptr<Frame> &frame, int
               // &out_successfulFrameID, SE3 &out_frameToKeyframe);

  bool isRunning;

private:
  // int w, h;
  // Eigen::Matrix3f K;
  boost::thread relocThreads[RELOCALIZE_THREADS];
  bool running[RELOCALIZE_THREADS];

  // locking & signalling structures
  std::mutex exMutex;
  std::condition_variable newCurrentFrameSignal;
  std::condition_variable resultReadySignal;

  // for rapid-checking
  std::deque<std::shared_ptr<KeyFrame>> KFForReloc;
  Frame::SharedPtr CurrentRelocFrame;
  int nextRelocIDX;
  int maxRelocIDX;
  bool continueRunning;

  // result!
  Frame::SharedPtr resultRelocFrame;
  bool hasResult;
  std::shared_ptr<KeyFrame> resultKF;
  int resultFrameID;
  SE3 resultFrameToKeyframe;

  void threadLoop(int idx);

  bool _printRelocalizationInfo;
};

} // namespace lsd_slam
