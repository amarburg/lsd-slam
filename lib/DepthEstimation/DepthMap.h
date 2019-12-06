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
#include "opencv2/core/core.hpp"
#include "util/Configuration.h"
#include "util/EigenCoreInclude.h"
#include "util/IndexThreadReduce.h"
#include "util/MovingAverage.h"
#include "util/SophusUtil.h"
#include "util/Timer.h"
#include "util/settings.h"

#include "DataStructures/Frame.h"
#include "DataStructures/ImageSet.h"
#include "DepthMapDebugImages.h"

namespace lsd_slam {

class DepthMapPixelHypothesis;
class KeyFrameGraph;

class KeyFrame;

/**
 * Keeps a detailed depth map (consisting of DepthMapPixelHypothesis) and does
 * stereo comparisons and regularization to update it.
 */
class DepthMap {
public:
  typedef std::shared_ptr<DepthMap> SharedPtr;

  // Delete default constructors
  DepthMap(const DepthMap &) = delete;
  DepthMap &operator=(const DepthMap &) = delete;

  DepthMap(const std::shared_ptr<Frame> &frame);
  DepthMap(const std::shared_ptr<ImageSet> &set);

  ~DepthMap();

  /** Resets everything. */
  void reset();

  const DepthMapDebugImages &debugImages() const { return _debugImages; }

  // This is the only debug plot which is triggered externally..
  void plotDepthMap(const char *buf1, const char *buf2);

  //== Initializers, required only when propagating from a previous keyframe
  void initializeFromFrame();
  void initializeFromSet();
  void initializeFromStereo();
  void initializeRandomly();
  // void initializefromStereo( const std::shared_ptr<ImageSet> &set);

  void propagateFrom(const DepthMap::SharedPtr &new_keyframe,
                     float &rescaleFactor);

  void finalize();

  // void activateExistingKF(const Frame::SharedPtr &kf);

  void activateExistingKF(const Frame::SharedPtr &kf);

  //==
  bool updateDepthFrom(const Frame::SharedPtr &frame, const bool _useDisparity);

  struct PerformanceData {
    PerformanceData(void) { ; }

    MsRateAverage update, create, finalize, observe, regularize, propagate,
        fillHoles, setDepth;
  };

  PerformanceData perf() const { return _perf; }
  void logPerformanceData();

  // Convenience accessors
  std::shared_ptr<Frame> &frame() { return _frame; }

  //
  DepthMapPixelHypothesis *hypothesisAt(const int x, const int y) {
    return currentDepthMap + x + y * Conf().slamImageSize.width;
  }

private:
  cv::Mat debugDepthImg;
  cv::Mat debugGradientImg;
  IndexThreadReduce threadReducer;

  bool useDisparity;

  Eigen::Vector3f trav_KF;

  PerformanceData _perf;
  DepthMapDebugImages _debugImages;

  // ============= parameter copies for convenience ===========================
  // these are just copies of the pointers given to this function, for
  // convenience. these are NOT managed by this object!
  std::shared_ptr<Frame> _frame;
  std::shared_ptr<ImageSet> _set;

  const float *activeKeyFrameImageData() { return frame()->image(0); }
  bool activeKeyFrameIsReactivated;

  // Frame::SharedPtr oldest_referenceFrame;
  // Frame::SharedPtr newest_referenceFrame;
  // std::vector< Frame::SharedPtr > referenceFrameByID;
  // int referenceFrameByID_offset;

  // ============= internally used buffers for intermediate calculations etc.
  // ============= for internal depth tracking, their memory is managed (created
  // & deleted) by this object.
  DepthMapPixelHypothesis *otherDepthMap;
  DepthMapPixelHypothesis *currentDepthMap;
  int *validityIntegralBuffer;

  // ============ internal functions
  // ================================================== does the line-stereo
  // seeking. takes a lot of parameters, because they all have been pre-computed
  // before.
  inline float doLineStereo(const float u, const float v, const float epxn,
                            const float epyn, const float min_idepth,
                            const float prior_idepth, float max_idepth,
                            const Frame *const referenceFrame,
                            const float *referenceFrameImage,
                            float &result_idepth, float &result_var,
                            float &result_eplLength, RunningStats *const stats);

  // Reset currentDepthMap by re-projecting is from activeKeyFrame to
  // new_keyframe
  void propagateDepthFromSet(const DepthMap::SharedPtr &new_keyframe,
                             float &rescaleFactor);
  void updateFromStereo();

  // This is a local state variable used to share data between the observeDepth*
  // functions.  Sucks, I know
  Frame::SharedPtr _observeFrame;
  void observeDepth(const Frame::SharedPtr &updateFrame);
  void observeDepthRow(int yMin, int yMax, RunningStats *stats);
  bool observeDepthCreate(const int &x, const int &y, const int &idx,
                          RunningStats *const &stats);
  bool createNewStereoDepthPoint(const int &x, const int &y, const int &idx,
                                 RunningStats *const &stats);
  bool observeDepthUpdate(const int &x, const int &y, const int &idx,
                          const float *keyFrameMaxGradBuf,
                          RunningStats *const &stats);
  bool makeAndCheckEPL(const int x, const int y, const Frame *const ref,
                       float *pepx, float *pepy, RunningStats *const stats);

  void regularizeDepthMap(bool removeOcclusion, int validityTH);
  template <bool removeOcclusions>
  void regularizeDepthMapRow(int validityTH, int yMin, int yMax,
                             RunningStats *stats);

  void buildRegIntegralBuffer();
  void buildRegIntegralBufferRow1(int yMin, int yMax, RunningStats *stats);
  void regularizeDepthMapFillHoles();
  void regularizeDepthMapFillHolesRow(int yMin, int yMax, RunningStats *stats);

  void resetCounters();

  // float clocksPropagate, clocksPropagateKF, clocksObserve, msObserve,
  // clocksReg1, clocksReg2, msReg1, msReg2, clocksFinalize;
};

} // namespace lsd_slam
