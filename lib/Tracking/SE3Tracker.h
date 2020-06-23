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
#include "Tracking/LGSX.h"
#include "util/Configuration.h"
#include "util/DenseDepthTrackerSettings.h"
#include "util/EigenCoreInclude.h"
#include "util/SophusUtil.h"
#include <opencv2/core/core.hpp>

namespace lsd_slam {

template <int __LEVELS> class _TrackingRef;
typedef _TrackingRef<PYRAMID_LEVELS> TrackingReference;

class Frame;
class KeyFrame;

struct SE3TrackerDebugImages {
		SE3TrackerDebugImages() = delete;
		SE3TrackerDebugImages(const SE3TrackerDebugImages &) = delete;

		SE3TrackerDebugImages(const ImageSize &imgSize);

		// debug images
		cv::Mat debugImageResiduals;
		cv::Mat debugImageWeights;
		cv::Mat debugImageSecondFrame;
		cv::Mat debugImageOldImageSource;
		cv::Mat debugImageOldImageWarped;
};

class SE3Tracker {
public:
EIGEN_MAKE_ALIGNED_OPERATOR_NEW

DenseDepthTrackerSettings<PYRAMID_LEVELS> settings;

SE3Tracker(const SE3Tracker &) = delete;
SE3Tracker &operator=(const SE3Tracker &) = delete;

SE3Tracker(const ImageSize &sz);
~SE3Tracker();

// Trackes _frame_ onto _keyframe_.  Modifies _frame_
SE3 trackFrame(const std::shared_ptr<KeyFrame> &keyframe,
               const std::shared_ptr<Frame> &frame,
               const SE3 &frameToReference_initialEstimate);

SE3 trackFrame(const std::shared_ptr<KeyFrame> &keyframe,
               const std::shared_ptr<Frame> &frame,
               const SE3 &frameToReference_initialEstimate,
               const SE3 &frameToReference_odom);

float trackGradient(const std::shared_ptr<KeyFrame> &keyframe,
                    const std::shared_ptr<Frame> &frame,
                    const SE3 &frameToReference_odom, SE3 &gradientTracked,
                    bool &_trackingWasGood);

void updateTrack(const std::shared_ptr<KeyFrame> &keyframe,
                 const std::shared_ptr<Frame> &frame,
                 const SE3 frameToReference, float residual,
                 bool _trackingWasGood);

// This version does not recurse through levels of the pyramid,
// It only checks QUICK_KF_CHECK_LVL
SE3 trackFrameOnPermaref(const std::shared_ptr<KeyFrame> &reference,
                         const std::shared_ptr<Frame> &frame,
                         SE3 referenceToFrame);

// Calculates the percentage overlap between the two keyframes
float checkPermaRefOverlap(const std::shared_ptr<KeyFrame> &reference,
                           SE3 referenceToFrame);

float pointUsage;
float lastGoodCount() const {
		return _lastGoodCount;
}
float lastMeanRes;
float lastBadCount() const {
		return _lastBadCount;
}
float lastResidual;

float _pctGoodPerGoodBad;
float _pctGoodPerTotal;

float affineEstimation_a;
float affineEstimation_b;

bool diverged;
bool trackingWasGood;

private:
float _lastGoodCount;
float _lastBadCount;

const ImageSize &_imgSize;

float *buf_warped_residual;
float *buf_warped_dx;
float *buf_warped_dy;
float *buf_warped_x;
float *buf_warped_y;
float *buf_warped_z;

float *buf_d;
float *buf_idepthVar;
float *buf_weight_p;

int buf_warped_size;

SE3TrackerDebugImages _debugImages;

void calculateWarpUpdate(LGS6 &ls);

float calcWeightsAndResidual(const Sophus::SE3f &referenceToFrame);

float calcResidualAndBuffers(const Eigen::Vector3f *refPoint,
                             const Eigen::Vector2f *refColVar, int *idxBuf,
                             int refNum, const std::shared_ptr<Frame> &frame,
                             const Sophus::SE3f &referenceToFrame, int level,
                             bool plotResidual = false);

#if defined(ENABLE_SSE)
void calculateWarpUpdateSSE(LGS6 &ls);
float calcWeightsAndResidualSSE(const Sophus::SE3f &referenceToFrame);
float calcResidualAndBuffersSSE(const Eigen::Vector3f *refPoint,
                                const Eigen::Vector2f *refColVar, int *idxBuf,
                                int refNum,
                                const std::shared_ptr<Frame> &frame,
                                const Sophus::SE3f &referenceToFrame,
                                int level, bool plotResidual = false);
#endif

#if defined(ENABLE_NEON)
void calculateWarpUpdateNEON(LGS6 &ls);
float calcWeightsAndResidualNEON(const Sophus::SE3f &referenceToFrame);
float calcResidualAndBuffersNEON(const Eigen::Vector3f *refPoint,
                                 const Eigen::Vector2f *refColVar,
                                 int *idxBuf, int refNum,
                                 const std::shared_ptr<Frame> &frame,
                                 const Sophus::SE3f &referenceToFrame,
                                 int level, bool plotResidual = false);
#endif

void calcResidualAndBuffers_debugStart();
void calcResidualAndBuffers_debugFinish(int w);

// used for image saving
int iterationNumber;

float affineEstimation_a_lastIt;
float affineEstimation_b_lastIt;
};

} // namespace lsd_slam
