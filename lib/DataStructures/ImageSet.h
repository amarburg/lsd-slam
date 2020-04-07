/**
 * This file is part of LSD-SLAM.
 *
 * Copyright 2019 Aaron Marburg <amarburg at uw dot edu >
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
#include <opencv2/core.hpp>
#include <vector>

#include "DataStructures/Frame.h"
#include "util/SophusUtil.h"
#include <libvideoio/types/Camera.h>

namespace lsd_slam {

// Relies on the lazy-initialize nature of Frame to be efficient
class ImageSet {
public:
  ImageSet() = delete;
  ImageSet(const ImageSet &) = delete;

  ImageSet(unsigned int id, const cv::Mat &img, const libvideoio::Camera &cam);
  ImageSet(unsigned int id, const cv::Mat &imgL, const cv::Mat &imgR,
           const libvideoio::Camera &camL, const libvideoio::Camera &camR,
           const unsigned int ref);
  ~ImageSet();

  Frame::SharedPtr &refFrame()                            { return _frames[_refFrame]; }

  Frame::SharedPtr &getFrame(const unsigned int frameNum) { return _frames[frameNum]; }

  void pushbackFrame(const cv::Mat &img, const libvideoio::Camera &cam);
  Sim3 getRefTransformation() { return _frames[_refFrame]->getCamToWorld(); }
  void setReferenceFrame(const unsigned int &frameNum) { _refFrame = frameNum; }
  unsigned int id() { return _frameId; }

  typedef std::shared_ptr<ImageSet> SharedPtr;

private:
  unsigned int _refFrame;
  unsigned int _frameId;
  std::vector<Frame::SharedPtr> _frames;
  std::vector<Sophus::SE3d> _se3FromFirst;
};

} // namespace lsd_slam
