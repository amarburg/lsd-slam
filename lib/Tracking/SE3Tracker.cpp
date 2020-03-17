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

#include <sstream>
using std::stringstream;

#include "DataStructures/Frame.h"
#include "DataStructures/KeyFrame.h"
#include "IOWrapper/ImageDisplay.h"
#include "SE3Tracker.h"
#include "Tracking/LGSX.h"
#include "Tracking/TrackingReference.h"
#include "util/globalFuncs.h"
#include <opencv2/highgui/highgui.hpp>

namespace lsd_slam {

#if defined(ENABLE_NEON)
#define callOptimized(function, arguments) function##NEON arguments
#else
#if defined(ENABLE_SSE)
#error "SSE Enabled!"
#define callOptimized(function, arguments)                                     \
  (USESSE ? function##SSE arguments : function arguments)
#else
#define callOptimized(function, arguments) function arguments
#endif
#endif

SE3Tracker::SE3Tracker(const ImageSize &sz)
    : _pctGoodPerGoodBad(-1.0), _pctGoodPerTotal(-1.0), _lastGoodCount(0),
      _lastBadCount(0), _imgSize(sz), _debugImages(sz), settings() {

  const int area = _imgSize.area();

  buf_warped_residual = new float[area];
  buf_warped_dx = new float[area];
  buf_warped_dy = new float[area];
  buf_warped_x = new float[area];
  buf_warped_y = new float[area];
  buf_warped_z = new float[area];

  buf_d = new float[area];
  buf_idepthVar = new float[area];
  buf_weight_p = new float[area];

  buf_warped_size = 0;

  lastResidual = 0;
  iterationNumber = 0;
  pointUsage = 0;

  diverged = false;
}

SE3Tracker::~SE3Tracker() {

  delete[] buf_warped_residual;
  delete[] buf_warped_dx;
  delete[] buf_warped_dy;
  delete[] buf_warped_x;
  delete[] buf_warped_y;
  delete[] buf_warped_z;

  delete[] buf_d;
  delete[] buf_idepthVar;
  delete[] buf_weight_p;
}

// Calculates the percentage overlap between the two keyframes
float SE3Tracker::checkPermaRefOverlap(
    const std::shared_ptr<KeyFrame> &reference, SE3 referenceToFrameOrg) {
  Sophus::SE3f referenceToFrame = referenceToFrameOrg.cast<float>();
  // boost::unique_lock<boost::mutex> lock2 =
  // boost::unique_lock<boost::mutex>(reference->permaRef_mutex);
  const std::shared_ptr<Frame> &frame(reference->frame());
  const std::shared_ptr<TrackingReference> &ref(reference->trackingReference());

  const int w2 = frame->width(QUICK_KF_CHECK_LVL) - 1;
  const int h2 = frame->height(QUICK_KF_CHECK_LVL) - 1;
  const Eigen::Matrix3f KLvl = frame->K(QUICK_KF_CHECK_LVL);
  const float fx_l = KLvl(0, 0);
  const float fy_l = KLvl(1, 1);
  const float cx_l = KLvl(0, 2);
  const float cy_l = KLvl(1, 2);

  Eigen::Matrix3f rotMat = referenceToFrame.rotationMatrix();
  Eigen::Vector3f transVec = referenceToFrame.translation();

  const Eigen::Vector3f *refPoint_max =
      ref->posData[QUICK_KF_CHECK_LVL] + ref->numData[QUICK_KF_CHECK_LVL];
  const Eigen::Vector3f *refPoint = ref->posData[QUICK_KF_CHECK_LVL];

  float usageCount = 0;
  for (; refPoint < refPoint_max; refPoint++) {
    Eigen::Vector3f Wxp = rotMat * (*refPoint) + transVec;
    float u_new = (Wxp[0] / Wxp[2]) * fx_l + cx_l;
    float v_new = (Wxp[1] / Wxp[2]) * fy_l + cy_l;
    if ((u_new > 0 && v_new > 0 && u_new < w2 && v_new < h2)) {
      float depthChange = (*refPoint)[2] / Wxp[2];
      usageCount += depthChange < 1 ? depthChange : 1;
    }
  }

  pointUsage = usageCount / (float)ref->numData[QUICK_KF_CHECK_LVL];
  return pointUsage;
}

// tracks a frame.
// first_frame has depth, second_frame DOES NOT have depth.
SE3 SE3Tracker::trackFrameOnPermaref(const std::shared_ptr<KeyFrame> &reference,
                                     const std::shared_ptr<Frame> &frame,
                                     SE3 referenceToFrameOrg) {

  const std::shared_ptr<TrackingReference> &ref(reference->trackingReference());
  Sophus::SE3f referenceToFrame = referenceToFrameOrg.cast<float>();

  affineEstimation_a = 1;
  affineEstimation_b = 0;

  LGS6 ls;
  diverged = false;
  trackingWasGood = true;

  callOptimized(calcResidualAndBuffers,
                (ref->posData[QUICK_KF_CHECK_LVL],
                 ref->colorAndVarData[QUICK_KF_CHECK_LVL], 0,
                 ref->numData[QUICK_KF_CHECK_LVL], frame, referenceToFrame,
                 QUICK_KF_CHECK_LVL, false));
  if (buf_warped_size < MIN_GOODPERALL_PIXEL_ABSMIN *
                            (_imgSize.width >> QUICK_KF_CHECK_LVL) *
                            (_imgSize.height >> QUICK_KF_CHECK_LVL)) {
    diverged = true;
    trackingWasGood = false;
    return SE3();
  }
  if (useAffineLightningEstimation) {
    affineEstimation_a = affineEstimation_a_lastIt;
    affineEstimation_b = affineEstimation_b_lastIt;
  }
  float lastErr = callOptimized(calcWeightsAndResidual, (referenceToFrame));

  float LM_lambda = settings.lambdaInitialTestTrack;

  for (int iteration = 0; iteration < settings.maxItsTestTrack; iteration++) {
    callOptimized(calculateWarpUpdate, (ls));

    int incTry = 0;
    while (true) {
      // solve LS system with current lambda
      Vector6 b = -ls.b;
      Matrix6x6 A = ls.A;
      for (int i = 0; i < 6; i++)
        A(i, i) *= 1 + LM_lambda;
      Vector6 inc = A.ldlt().solve(b);
      incTry++;

      // apply increment. pretty sure this way round is correct, but hard to
      // test.
      Sophus::SE3f new_referenceToFrame =
          Sophus::SE3f::exp((inc)) * referenceToFrame;

      // re-evaluate residual
      callOptimized(calcResidualAndBuffers,
                    (ref->posData[QUICK_KF_CHECK_LVL],
                     ref->colorAndVarData[QUICK_KF_CHECK_LVL], 0,
                     ref->numData[QUICK_KF_CHECK_LVL], frame,
                     new_referenceToFrame, QUICK_KF_CHECK_LVL, false));
      if (buf_warped_size < MIN_GOODPERALL_PIXEL_ABSMIN *
                                (_imgSize.width >> QUICK_KF_CHECK_LVL) *
                                (_imgSize.height >> QUICK_KF_CHECK_LVL)) {
        diverged = true;
        trackingWasGood = false;
        return SE3();
      }
      float error =
          callOptimized(calcWeightsAndResidual, (new_referenceToFrame));

      // accept inc?
      if (error < lastErr) {
        // accept inc
        referenceToFrame = new_referenceToFrame;
        if (useAffineLightningEstimation) {
          affineEstimation_a = affineEstimation_a_lastIt;
          affineEstimation_b = affineEstimation_b_lastIt;
        }
        // converged?
        if (error / lastErr > settings.convergenceEpsTestTrack)
          iteration = settings.maxItsTestTrack;

        lastErr = error;

        if (LM_lambda <= 0.2)
          LM_lambda = 0;
        else
          LM_lambda *= settings.lambdaSuccessFac;

        break;
      } else {
        if (!(inc.dot(inc) > settings.stepSizeMinTestTrack)) {
          iteration = settings.maxItsTestTrack;
          break;
        }

        if (LM_lambda == 0)
          LM_lambda = 0.2;
        else
          LM_lambda *= std::pow(settings.lambdaFailFac, incTry);
      }
    }
  }

  lastResidual = lastErr;

  _pctGoodPerTotal = _lastGoodCount / (frame->width(QUICK_KF_CHECK_LVL) *
                                       frame->height(QUICK_KF_CHECK_LVL));
  _pctGoodPerGoodBad = _lastGoodCount / (_lastGoodCount + _lastBadCount);

  trackingWasGood = !diverged && _pctGoodPerTotal > MIN_GOODPERALL_PIXEL &&
                    _pctGoodPerGoodBad > MIN_GOODPERGOODBAD_PIXEL;

  return toSophus(referenceToFrame);
}

// tracks a frame.
SE3 SE3Tracker::trackFrame(const std::shared_ptr<KeyFrame> &keyframe,
                           const std::shared_ptr<Frame> &frame,
                           const SE3 &frameToReference_initialEstimate) {

  std::shared_ptr<TrackingReference> &reference(keyframe->trackingReference());

  boost::shared_lock<boost::shared_mutex> lock = frame->getActiveLock();
  diverged = false;
  trackingWasGood = true;
  affineEstimation_a = 1;
  affineEstimation_b = 0;

  if (saveAllTrackingStages) {
    saveAllTrackingStages = false;
    saveAllTrackingStagesInternal = true;
  }

  if (plotTrackingIterationInfo) {
    const float *frameImage = frame->image();
    for (int row = 0; row < _imgSize.height; ++row)
      for (int col = 0; col < _imgSize.height; ++col)
        setPixelInCvMat(&_debugImages.debugImageSecondFrame,
                        getGrayCvPixel(frameImage[col + row * _imgSize.width]),
                        col, row, 1);
  }

  // ============ track frame ============
  Sophus::SE3f referenceToFrame =
      frameToReference_initialEstimate.inverse().cast<float>();
  LGS6 ls;

  int numCalcResidualCalls[PYRAMID_LEVELS];
  int numCalcWarpUpdateCalls[PYRAMID_LEVELS];

  float last_residual = 0;
  int lowestLvl;

  for (int lvl = SE3TRACKING_MAX_LEVEL - 1; lvl >= SE3TRACKING_MIN_LEVEL;
       lvl--) {
    numCalcResidualCalls[lvl] = 0;
    numCalcWarpUpdateCalls[lvl] = 0;
    lowestLvl = lvl;

    reference->makePointCloud(lvl);

    LOG(INFO) << "Calculating initial residual on frame " << frame->id()
              << ", level " << lvl << " against reference frame "
              << reference->frameID() << " with " << reference->numData[lvl]
              << " points";
    callOptimized(
        calcResidualAndBuffers,
        (reference->posData[lvl], reference->colorAndVarData[lvl],
         SE3TRACKING_MIN_LEVEL == lvl ? reference->pointPosInXYGrid[lvl] : 0,
         reference->numData[lvl], frame, referenceToFrame, lvl,
         (plotTracking && lvl == SE3TRACKING_MIN_LEVEL)));

    if (buf_warped_size < MIN_GOODPERALL_PIXEL_ABSMIN *
                              (_imgSize.width >> lvl) *
                              (_imgSize.height >> lvl)) {
      diverged = true;
      trackingWasGood = false;
      LOG(INFO) << "Diverged at level " << lvl << "!  Only " << buf_warped_size
                << " pixel to track.";
      return SE3();
    }

    if (useAffineLightningEstimation) {
      affineEstimation_a = affineEstimation_a_lastIt;
      affineEstimation_b = affineEstimation_b_lastIt;
    }
    float lastErr = callOptimized(calcWeightsAndResidual, (referenceToFrame));

    numCalcResidualCalls[lvl]++;

    float LM_lambda = settings.lambdaInitial[lvl];

    for (int iteration = 0; iteration < settings.maxItsPerLvl[lvl];
         iteration++) {

      callOptimized(calculateWarpUpdate, (ls));

      numCalcWarpUpdateCalls[lvl]++;

      iterationNumber = iteration;

      int incTry = 0;
      while (true) {
        // solve LS system with current lambda
        Vector6 b = -ls.b;
        Matrix6x6 A = ls.A;
        for (int i = 0; i < 6; i++)
          A(i, i) *= 1 + LM_lambda;
        Vector6 inc = A.ldlt().solve(b);
        incTry++;

        // apply increment. pretty sure this way round is correct, but hard to
        // test.
        // Sophus::SE3f exp_increment = Sophus::SE3f::exp((inc));
        Sophus::SE3f new_referenceToFrame =
            Sophus::SE3f::exp((inc)) * referenceToFrame;
        // Sophus::SE3f new_referenceToFrame = referenceToFrame *
        // Sophus::SE3f::exp((inc));

        // re-evaluate residual
        callOptimized(calcResidualAndBuffers,
                      (reference->posData[lvl], reference->colorAndVarData[lvl],
                       SE3TRACKING_MIN_LEVEL == lvl
                           ? reference->pointPosInXYGrid[lvl]
                           : 0,
                       reference->numData[lvl], frame, new_referenceToFrame,
                       lvl, (plotTracking && lvl == SE3TRACKING_MIN_LEVEL)));

        if (buf_warped_size < MIN_GOODPERALL_PIXEL_ABSMIN *
                                  (_imgSize.width >> lvl) *
                                  (_imgSize.height >> lvl)) {
          diverged = true;
          trackingWasGood = false;
          LOG(INFO) << "Diverged at level " << lvl << " on iteration "
                    << iteration << "!  Only " << buf_warped_size
                    << " pixels to track.";
          return SE3();
        }

        float error =
            callOptimized(calcWeightsAndResidual, (new_referenceToFrame));
        numCalcResidualCalls[lvl]++;

        // accept inc?
        if (error < lastErr) {
          // accept inc
          referenceToFrame = new_referenceToFrame;
          if (useAffineLightningEstimation) {
            affineEstimation_a = affineEstimation_a_lastIt;
            affineEstimation_b = affineEstimation_b_lastIt;
          }

          LOGF_IF(DEBUG, Conf().print.trackingIterationInfo,
                  "(%d-%d): ACCEPTED increment of %f with lambda %.1f, "
                  "residual: %f > %f",
                  lvl, iteration, sqrt(inc.dot(inc)), LM_lambda, lastErr,
                  error);

          // converged?
          if (error / lastErr > settings.convergenceEps[lvl]) {
            LOGF_IF(DEBUG, Conf().print.trackingIterationInfo,
                    "(%d-%d): FINISHED pyramid level (last residual reduction "
                    "too small).",
                    lvl, iteration);
            iteration = settings.maxItsPerLvl[lvl];
          }

          last_residual = lastErr = error;

          if (LM_lambda <= 0.2)
            LM_lambda = 0;
          else
            LM_lambda *= settings.lambdaSuccessFac;

          break;
        } else {

          LOGF_IF(DEBUG, Conf().print.trackingIterationInfo,
                  "(%d-%d): REJECTED increment of %f with lambda %.1f, "
                  "(residual: %f < %f).",
                  lvl, iteration, sqrt(inc.dot(inc)), LM_lambda, lastErr,
                  error);

          if (!(inc.dot(inc) > settings.stepSizeMin[lvl])) {
            LOGF_IF(DEBUG, Conf().print.trackingIterationInfo,
                    "(%d-%d): FINISHED pyramid level (stepsize too small).",
                    lvl, iteration);

            iteration = settings.maxItsPerLvl[lvl];
            break;
          }

          if (LM_lambda == 0)
            LM_lambda = 0.2;
          else
            LM_lambda *= std::pow(settings.lambdaFailFac, incTry);
        }
      }
    }
  }

  if (plotTracking)
    Util::displayImage("TrackingResidual", _debugImages.debugImageResiduals,
                       false);

  if (Conf().print.trackingIterationInfo) {
    stringstream outstr;
    outstr << "SE3 Tracking: ";
    for (int lvl = PYRAMID_LEVELS - 1; lvl >= lowestLvl; lvl--) {
      outstr << "lvl " << lvl << ": " << numCalcResidualCalls[lvl] << " ("
             << numCalcWarpUpdateCalls[lvl] << "); ";
    }

    LOG(DEBUG) << outstr.str();
  }

  saveAllTrackingStagesInternal = false;

  lastResidual = last_residual;

  _pctGoodPerTotal = _lastGoodCount / (frame->width(SE3TRACKING_MIN_LEVEL) *
                                       frame->height(SE3TRACKING_MIN_LEVEL));
  _pctGoodPerGoodBad = _lastGoodCount / (_lastGoodCount + _lastBadCount);

  LOG_IF(DEBUG, Conf().print.trackingIterationInfo)
      << "lastGoodCount " << _lastGoodCount << " lastBadCount "
      << _lastBadCount;
  // LOG_IF(DEBUG, Conf().print.trackingIterationInfo ) <<
  // frame->width(SE3TRACKING_MIN_LEVEL) << " " <<
  // frame->height(SE3TRACKING_MIN_LEVEL); LOG_IF(DEBUG,
  // Conf().print.trackingIterationInfo ) << _pctGoodPerTotal << " " <<
  // _pctGoodPerGoodBad;

  trackingWasGood = !diverged && _pctGoodPerTotal > MIN_GOODPERALL_PIXEL &&
                    _pctGoodPerGoodBad > MIN_GOODPERGOODBAD_PIXEL;

  if (trackingWasGood)
    keyframe->numFramesTrackedOnThis++;

  frame->initialTrackedResidual = lastResidual / pointUsage;
  frame->pose->thisToParent_raw =
      sim3FromSE3(toSophus(referenceToFrame.inverse()), 1);
  frame->setTrackingParent(keyframe);
  return toSophus(referenceToFrame.inverse());
}

float SE3Tracker::calcWeightsAndResidual(const Sophus::SE3f &referenceToFrame) {
  float tx = referenceToFrame.translation()[0];
  float ty = referenceToFrame.translation()[1];
  float tz = referenceToFrame.translation()[2];

  float sumRes = 0;

  for (int i = 0; i < buf_warped_size; i++) {
    float px = *(buf_warped_x + i);                       // x'
    float py = *(buf_warped_y + i);                       // y'
    float pz = *(buf_warped_z + i);                       // z'
    float d = *(buf_d + i);                               // d
    float rp = *(buf_warped_residual + i);                // r_p
    float gx = *(buf_warped_dx + i);                      // \delta_x I
    float gy = *(buf_warped_dy + i);                      // \delta_y I
    float s = settings.var_weight * *(buf_idepthVar + i); // \sigma_d^2

    // calc dw/dd (first 2 components):
    float g0 = (tx * pz - tz * px) / (pz * pz * d);
    float g1 = (ty * pz - tz * py) / (pz * pz * d);

    // calc w_p
    float drpdd = gx * g0 + gy * g1; // ommitting the minus
    float w_p = 1.0f / ((cameraPixelNoise2) + s * drpdd * drpdd);

    float weighted_rp = fabs(rp * sqrtf(w_p));

    float wh = fabs(weighted_rp < (settings.huber_d / 2)
                        ? 1
                        : (settings.huber_d / 2) / weighted_rp);

    sumRes += wh * w_p * rp * rp;

    *(buf_weight_p + i) = wh * w_p;
  }

  return sumRes / buf_warped_size;
}

void SE3Tracker::calcResidualAndBuffers_debugStart() {
  if (plotTrackingIterationInfo || saveAllTrackingStagesInternal) {
    int other = saveAllTrackingStagesInternal ? 255 : 0;
    fillCvMat(&_debugImages.debugImageResiduals, cv::Vec3b(other, other, 255));
    fillCvMat(&_debugImages.debugImageWeights, cv::Vec3b(other, other, 255));
    fillCvMat(&_debugImages.debugImageOldImageSource,
              cv::Vec3b(other, other, 255));
    fillCvMat(&_debugImages.debugImageOldImageWarped,
              cv::Vec3b(other, other, 255));
  }
}

void SE3Tracker::calcResidualAndBuffers_debugFinish(int w) {
  if (plotTrackingIterationInfo) {
    Util::displayImage("Weights", _debugImages.debugImageWeights);
    Util::displayImage("second_frame", _debugImages.debugImageSecondFrame);
    Util::displayImage("Intensities of second_frame at transformed positions",
                       _debugImages.debugImageOldImageSource);
    Util::displayImage(
        "Intensities of second_frame at pointcloud in first_frame",
        _debugImages.debugImageOldImageWarped);
    Util::displayImage("Residuals", _debugImages.debugImageResiduals);
  }

  if (saveAllTrackingStagesInternal) {
    char charbuf[500];

    snprintf(charbuf, 500, "save/%sresidual-%d-%d.png", packagePath.c_str(), w,
             iterationNumber);
    cv::imwrite(charbuf, _debugImages.debugImageResiduals);

    snprintf(charbuf, 500, "save/%swarped-%d-%d.png", packagePath.c_str(), w,
             iterationNumber);
    cv::imwrite(charbuf, _debugImages.debugImageOldImageWarped);

    snprintf(charbuf, 500, "save/%sweights-%d-%d.png", packagePath.c_str(), w,
             iterationNumber);
    cv::imwrite(charbuf, _debugImages.debugImageWeights);

    printf("saved three images for lvl %d, iteration %d\n", w, iterationNumber);
  }
}

float SE3Tracker::calcResidualAndBuffers(const Eigen::Vector3f *refPoint,
                                         const Eigen::Vector2f *refColVar,
                                         int *idxBuf, int refNum,
                                         const std::shared_ptr<Frame> &frame,
                                         const Sophus::SE3f &referenceToFrame,
                                         int level, bool plotResidual) {
  calcResidualAndBuffers_debugStart();

  if (plotResidual)
    _debugImages.debugImageResiduals.setTo(0);

  int w = frame->width(level);
  int h = frame->height(level);
  Eigen::Matrix3f KLvl = frame->K(level);
  float fx_l = KLvl(0, 0);
  float fy_l = KLvl(1, 1);
  float cx_l = KLvl(0, 2);
  float cy_l = KLvl(1, 2);

  Eigen::Matrix3f rotMat = referenceToFrame.rotationMatrix();
  Eigen::Vector3f transVec = referenceToFrame.translation();

  const Eigen::Vector3f *refPoint_max = refPoint + refNum;

  const Eigen::Vector4f *frame_gradients = frame->gradients(level);

  int idx = 0;

  float sumResUnweighted = 0;

  bool *isGoodOutBuffer = idxBuf != 0 ? frame->refPixelWasGood() : 0;

  int goodCount = 0;
  int badCount = 0;

  float sumSignedRes = 0;

  float sxx = 0, syy = 0, sx = 0, sy = 0, sw = 0;

  float usageCount = 0;

  // LOG(DEBUG) << refNum << ": " << *refPoint << " < " << *refPoint_max;
  // LOG(DEBUG) << "Rotmat: " << rotMat;
  // LOG(DEBUG) << "transVec: " << transVec;

  int loop = 0;
  for (; refPoint < refPoint_max; refPoint++, refColVar++, idxBuf++, loop++) {

    Eigen::Vector3f Wxp = rotMat * (*refPoint) + transVec;
    float u_new = (Wxp[0] / Wxp[2]) * fx_l + cx_l;
    float v_new = (Wxp[1] / Wxp[2]) * fy_l + cy_l;

    // step 1a: coordinates have to be in image:
    // (inverse test to exclude NANs)
    if (!(u_new > 1 && v_new > 1 && u_new < w - 2 && v_new < h - 2)) {
      if (isGoodOutBuffer != 0)
        isGoodOutBuffer[*idxBuf] = false;

      //			LOG_IF(DEBUG, loop < 50) << "Ref point: " <<
      //(*refPoint)[0] << " " << (*refPoint)[1] << " " << (*refPoint)[2];
      //			LOG_IF(DEBUG, loop < 50) << "Wxp :" << Wxp[0] <<
      //" " << Wxp[1] << " " << Wxp[2] << " maps to " << u_new << " " << v_new;
      continue;
    }

    Eigen::Vector3f resInterp =
        getInterpolatedElement43(frame_gradients, u_new, v_new, w);

    float c1 = affineEstimation_a * (*refColVar)[0] + affineEstimation_b;
    float c2 = resInterp[2];
    float residual = c1 - c2;

    float weight = fabsf(residual) < 5.0f ? 1 : 5.0f / fabsf(residual);
    sxx += c1 * c1 * weight;
    syy += c2 * c2 * weight;
    sx += c1 * weight;
    sy += c2 * weight;
    sw += weight;

    bool isGood = residual * residual /
                      (MAX_DIFF_CONSTANT +
                       MAX_DIFF_GRAD_MULT * (resInterp[0] * resInterp[0] +
                                             resInterp[1] * resInterp[1])) <
                  1;

    if (isGoodOutBuffer != 0)
      isGoodOutBuffer[*idxBuf] = isGood;

    *(buf_warped_x + idx) = Wxp(0);
    *(buf_warped_y + idx) = Wxp(1);
    *(buf_warped_z + idx) = Wxp(2);

    *(buf_warped_dx + idx) = fx_l * resInterp[0];
    *(buf_warped_dy + idx) = fy_l * resInterp[1];
    *(buf_warped_residual + idx) = residual;

    *(buf_d + idx) = 1.0f / (*refPoint)[2];
    *(buf_idepthVar + idx) = (*refColVar)[1];
    idx++;

    if (isGood) {
      sumResUnweighted += residual * residual;
      sumSignedRes += residual;
      goodCount++;
    } else
      badCount++;

    float depthChange =
        (*refPoint)[2] / Wxp[2]; // if depth becomes larger: pixel becomes
                                 // "smaller", hence count it less.
    usageCount += depthChange < 1 ? depthChange : 1;

    // DEBUG STUFF
    if (plotTrackingIterationInfo || plotResidual) {
      // for debug plot only: find x,y again.
      // horribly inefficient, but who cares at this point...
      int width = _imgSize.width;
      Eigen::Vector3f point = KLvl * (*refPoint);
      int x = point[0] / point[2] + 0.5f;
      int y = point[1] / point[2] + 0.5f;

      if (plotTrackingIterationInfo) {
        setPixelInCvMat(&_debugImages.debugImageOldImageSource,
                        getGrayCvPixel((float)resInterp[2]), u_new + 0.5,
                        v_new + 0.5, (width / w));
        setPixelInCvMat(&_debugImages.debugImageOldImageWarped,
                        getGrayCvPixel((float)resInterp[2]), x, y, (width / w));
      }
      if (isGood)
        setPixelInCvMat(&_debugImages.debugImageResiduals,
                        getGrayCvPixel(residual + 128), x, y, (width / w));
      else
        setPixelInCvMat(&_debugImages.debugImageResiduals, cv::Vec3b(0, 0, 255),
                        x, y, (width / w));
    }
  }

  buf_warped_size = idx;

  pointUsage = usageCount / (float)refNum;
  _lastGoodCount = goodCount;
  _lastBadCount = badCount;
  lastMeanRes = sumSignedRes / goodCount;

  LOG_IF(DEBUG, Conf().print.trackingIterationInfo)
      << "loop: " << loop << " buf_warped_size = " << buf_warped_size
      << "; goodCount = " << goodCount << "; badCount = " << badCount;
  // if( buf_warped_size == 0 ) {
  // 		LOG(DEBUG) << "Trap!";
  // }

  affineEstimation_a_lastIt =
      sqrtf((syy - sy * sy / sw) / (sxx - sx * sx / sw));
  affineEstimation_b_lastIt = (sy - affineEstimation_a_lastIt * sx) / sw;

  calcResidualAndBuffers_debugFinish(w);

  return sumResUnweighted / goodCount;
}

void SE3Tracker::calculateWarpUpdate(LGS6 &ls) {
  //	weightEstimator.reset();
  //	weightEstimator.estimateDistribution(buf_warped_residual,
  // buf_warped_size); 	weightEstimator.calcWeights(buf_warped_residual,
  // buf_warped_weights, buf_warped_size);
  //
  ls.initialize(_imgSize.area());
  for (int i = 0; i < buf_warped_size; i++) {
    float px = *(buf_warped_x + i);
    float py = *(buf_warped_y + i);
    float pz = *(buf_warped_z + i);
    float r = *(buf_warped_residual + i);
    float gx = *(buf_warped_dx + i);
    float gy = *(buf_warped_dy + i);
    // step 3 + step 5 comp 6d error vector

    float z = 1.0f / pz;
    float z_sqr = 1.0f / (pz * pz);
    Vector6 v;
    v[0] = z * gx + 0;
    v[1] = 0 + z * gy;
    v[2] = (-px * z_sqr) * gx + (-py * z_sqr) * gy;
    v[3] = (-px * py * z_sqr) * gx + (-(1.0 + py * py * z_sqr)) * gy;
    v[4] = (1.0 + px * px * z_sqr) * gx + (px * py * z_sqr) * gy;
    v[5] = (-py * z) * gx + (px * z) * gy;

    // step 6: integrate into A and b:
    ls.update(v, r, *(buf_weight_p + i));
  }

  // solve ls
  ls.finish();
  // result = ls.A.ldlt().solve(ls.b);
}

//=== SE3TrackerDebugImages ==

SE3TrackerDebugImages::SE3TrackerDebugImages(const ImageSize &imgSize)
    : debugImageWeights(cv::Mat(imgSize.cvSize(), CV_8UC3)),
      debugImageResiduals(cv::Mat(imgSize.cvSize(), CV_8UC3)),
      debugImageSecondFrame(cv::Mat(imgSize.cvSize(), CV_8UC3)),
      debugImageOldImageWarped(cv::Mat(imgSize.cvSize(), CV_8UC3)),
      debugImageOldImageSource(cv::Mat(imgSize.cvSize(), CV_8UC3)) {
  ;
}

} // namespace lsd_slam
