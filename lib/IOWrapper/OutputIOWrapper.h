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
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>

#include "DataStructures/Frame.h"
#include "GlobalMapping/KeyFrameGraph.h"
#include "util/SophusUtil.h"

namespace lsd_slam {

/**
 *
 */
class OutputIOWrapper {
public:
  virtual ~OutputIOWrapper(){};

  virtual void publishPose(const Sophus::Sim3f &pose) = 0;

  virtual void
  publishKeyframeGraph(const std::shared_ptr<KeyFrameGraph> &graph) = 0;

  virtual void publishPointCloud(const Frame::SharedPtr &kf) = 0;

  // publishes a keyframe. if that frame already exists, it is overwritten,
  // otherwise it is added.
  virtual void publishKeyframe(const Frame::SharedPtr &kf) = 0;
  virtual void publishFrame(const Frame::SharedPtr &kf,
                            const Eigen::MatrixXf G) = 0;

  virtual void publishFrame(const Frame::SharedPtr &kf, const Eigen::MatrixXf G,
                            const DepthMap::SharedPtr &depthMap) = 0;

  virtual void updateDepthImage(unsigned char *data) = 0;

  // published a tracked frame that did not become a keyframe (yet; i.e. has no
  // depth data)
  virtual void publishTrackedFrame(const Frame::SharedPtr &kf) = 0;

  virtual void publishTrackedFrame(const Frame::SharedPtr &frame,
                                   const Frame::SharedPtr &kf,
                                   const Eigen::MatrixXf G,
                                   const DepthMap::SharedPtr &depthMap,
                                   SE3 &frameToParentEstimate) = 0;

  // publishes graph and all constraints, as well as updated KF poses.
  virtual void
  publishTrajectory(std::vector<Eigen::Matrix<float, 3, 1>> trajectory,
                    std::string identifier) = 0;
  virtual void publishTrajectoryIncrement(Eigen::Matrix<float, 3, 1> pt,
                                          std::string identifier) = 0;

  virtual void publishDebugInfo(Eigen::Matrix<float, 20, 1> data) = 0;

  virtual void updateFrameNumber(int) = 0;
  virtual void updateLiveImage(const cv::Mat &img) = 0;
};
} // namespace lsd_slam
