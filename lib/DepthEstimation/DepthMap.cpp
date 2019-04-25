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

#include "DepthEstimation/DepthMap.h"

#include <fstream>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <stdio.h>

#include <g3log/g3log.hpp>

#include "DataStructures/Frame.h"
#include "DataStructures/KeyFrame.h"

#include "GlobalMapping/KeyFrameGraph.h"
#include "IOWrapper/ImageDisplay.h"
#include "util/globalFuncs.h"
#include "util/settings.h"

namespace lsd_slam {

// Constructor when unable to proagate from previous
DepthMap::DepthMap(const Frame::SharedPtr &frame)
    : _perf(), _debugImages(Conf().slamImageSize), _frame(frame), _set(nullptr),
      activeKeyFrameIsReactivated(false) {
  const size_t imgArea(Conf().slamImageSize.area());

  otherDepthMap = new DepthMapPixelHypothesis[imgArea];
  currentDepthMap = new DepthMapPixelHypothesis[imgArea];
  validityIntegralBuffer = new int[imgArea];

  reset();
}

DepthMap::DepthMap(const ImageSet::SharedPtr &set)
    : _perf(), _debugImages(Conf().slamImageSize), _set(set),
      _frame(set->refFrame()), activeKeyFrameIsReactivated(false) {
  const size_t imgArea(Conf().slamImageSize.area());

  otherDepthMap = new DepthMapPixelHypothesis[imgArea];
  currentDepthMap = new DepthMapPixelHypothesis[imgArea];
  validityIntegralBuffer = new int[imgArea];

  debugDepthImg = cv::Mat(Conf().slamImageSize.height,
                          Conf().slamImageSize.width, CV_32FC1);

  _meanIdepthRatio = 1;

  reset();
}

// // Constructor when propagation from an existing Depthmap
// DepthMap::DepthMap( const DepthMap::SharedPtr &other, const Frame::SharedPtr
// &keyframe ) : _perf(), 	_debugImages( Conf().slamImageSize ), _frame(
// keyframe
// ), 	activeKeyFrameIsReactivated = false;
// {
// 	const size_t imgArea( Conf().slamImageSize.area() );
//
// 	otherDepthMap = new DepthMapPixelHypothesis[imgArea];
// 	currentDepthMap = new DepthMapPixelHypothesis[imgArea];
// 	validityIntegralBuffer = new int[imgArea];
//
// 	propagateFrom( other );
//
// 	reset();
// }

// Convenience accessors.  Due to circular dependency between DepthMap
// and KeyFrame, these can't be inline in the header file

DepthMap::~DepthMap() {
  // if( (bool)activeKeyFrame )
  // 	activeKeyFramelock.unlock();

  delete[] otherDepthMap;
  delete[] currentDepthMap;

  delete[] validityIntegralBuffer;

  delete[] otherDepthMap;
  delete[] currentDepthMap;

  delete[] validityIntegralBuffer;
}

void DepthMap::reset() {
  for (DepthMapPixelHypothesis *pt =
           otherDepthMap + Conf().slamImageSize.area() - 1;
       pt >= otherDepthMap; pt--)
    pt->isValid = false;
  for (DepthMapPixelHypothesis *pt =
           currentDepthMap + Conf().slamImageSize.area() - 1;
       pt >= currentDepthMap; pt--)
    pt->isValid = false;
}

//=== Initialization function ==

void DepthMap::initializeFromFrame() {
  CHECK(_frame != nullptr) << "FRAME HAS NOT BEEN SET";
  if (frame()->hasIDepthBeenSet()) {
    LOG(INFO) << "Using initial Depth estimate in first frame.";
    initializeFromGTDepth();
  } else {
    LOG(INFO) << "Doing Stereo initialization!";
    initializeRandomly();
    // depthMap()->initializefromStereo(set);
  }
}

void DepthMap::initializeFromSet() {

  CHECK(_set != nullptr) << "SET HAS NOT BEEN SET";
  initializeFromStereo();
  // initializeRandomly();
}

void DepthMap::initializeFromStereo() {
  // TODO Initialize from stereo images
  // boost::shared_lock_guard<boost::shared_mutex> set_lock(_set->setMutex);
  int iDepthSize;

  iDepthSize = _set->disparity.iDepthSize;
  // std::cout << iDepthSize << " , "
  //           << Conf().slamImageSize.height * Conf().slamImageSize.width
  //           << std::endl;
  cv::Mat depthImg(Conf().slamImageSize.height, Conf().slamImageSize.width,
                   CV_32FC1);
  if (iDepthSize == Conf().slamImageSize.height * Conf().slamImageSize.width) {

    float *iDepth = _set->disparity.iDepth;
    uint8_t *iDepthValid = _set->disparity.iDepthValid;
    float iDepthMean = _set->disparity.iDepthMean;
    activeKeyFrameIsReactivated = false;

    const float *maxGradients = frame()->maxGradients();
    std::vector<float> idepth_vector;
    for (int y = 0; y < Conf().slamImageSize.height; y++) {
      for (int x = 0; x < Conf().slamImageSize.width; x++) {
        bool valid = *iDepthValid;
        int idx = x + y * Conf().slamImageSize.width;
        if (valid) {
          depthImg.at<float>(y, x) = *iDepth;
        }
        if (maxGradients[idx] > MIN_ABS_GRAD_CREATE && valid) {
          float idepth = *iDepth;
          // std::cout << "true: " << idepth << "random: "
          //           << iDepthMean + 1.0f * ((rand() % 100001) / 100000.0f)
          //           << std::endl;
          currentDepthMap[idx] = DepthMapPixelHypothesis(
              idepth, idepth, VAR_RANDOM_INIT_INITIAL, VAR_RANDOM_INIT_INITIAL,
              20, Conf().debugDisplay);
        }
        // else {
        //   if (maxGradients[idx] > MIN_ABS_GRAD_CREATE) {
        //     float r =
        //         (static_cast<float>(rand()) / static_cast<float>(RAND_MAX) -
        //          0.5) *
        //         2; // random from [-1,1]
        //
        //     float idepth = iDepthMean + iDepthMean / 2 * r;
        //     // std::cout << rand() % iDepthMean - iDepthMean / 2 <<
        //     // std::endl;
        //     // iDepthMean + iDepthMean * ((rand() % 100001) / 100000.0f);
        //     currentDepthMap[idx] = DepthMapPixelHypothesis(
        //         idepth, idepth, VAR_RANDOM_INIT_INITIAL,
        //         VAR_RANDOM_INIT_INITIAL, 20, Conf().debugDisplay);
        //   }
        //   // else {
        //   //   currentDepthMap[idx].isValid = false;
        //   //   currentDepthMap[idx].blacklisted = 0;
        //   // }
        // }
        iDepth++;
        iDepthValid++;
      }
    }
  }
  cv::imshow("depthImg", depthImg);
  cv::waitKey(1);

  // else {
  // LOG(WARNING) << "No disparity map found in time, initalizing randomly";
  // initializeRandomly();
  //}
}

void DepthMap::initializeRandomly() {
  activeKeyFrameIsReactivated = false;

  const float *maxGradients = frame()->maxGradients();

  for (int y = 1; y < (Conf().slamImageSize.height - 1); y++) {
    for (int x = 1; x < (Conf().slamImageSize.width - 1); x++) {
      int idx = x + y * Conf().slamImageSize.width;
      if (maxGradients[idx] > MIN_ABS_GRAD_CREATE) {
        float idepth = 0.5f + 1.0f * ((rand() % 100001) / 100000.0f);
        currentDepthMap[idx] = DepthMapPixelHypothesis(
            idepth, idepth, VAR_RANDOM_INIT_INITIAL, VAR_RANDOM_INIT_INITIAL,
            20, Conf().debugDisplay);
      } else {
        currentDepthMap[idx].isValid = false;
        currentDepthMap[idx].blacklisted = 0;
      }
    }
  }
}

void DepthMap::initializeFromGTDepth() {
  initializeRandomly();

  // TODO:   Disabled temporarily while re-plumbing DepthMap.   Should be
  // re-enabled

  // CHECK(new_frame->hasIDepthBeenSet());
  //
  // activeKeyFramelock = new_frame->getActiveLock();
  // activeKeyFrame = new_frame;
  // //activeKeyFrameImageData = activeKeyFrame->image(0);
  // activeKeyFrameIsReactivated = false;
  //
  // const float* idepth = new_frame->idepth();
  //
  //
  // float averageGTIDepthSum = 0;
  // int averageGTIDepthNum = 0;
  // for(int y=0;y<Conf().slamImageSize.height;y++)
  // {
  // 	for(int x=0;x<Conf().slamImageSize.width;x++)
  // 	{
  // 		float idepthValue = idepth[x+y*Conf().slamImageSize.width];
  // 		if(!isnanf(idepthValue) && idepthValue > 0)
  // 		{
  // 			averageGTIDepthSum += idepthValue;
  // 			averageGTIDepthNum ++;
  // 		}
  // 	}
  // }
  //
  //
  // for(int y=0;y<Conf().slamImageSize.height;y++)
  // {
  // 	for(int x=0;x<Conf().slamImageSize.width;x++)
  // 	{
  // 		int idx = x+y*Conf().slamImageSize.width;
  // 		float idepthValue = idepth[idx];
  //
  // 		if(!isnanf(idepthValue) && idepthValue > 0)
  // 		{
  // 			currentDepthMap[idx] = DepthMapPixelHypothesis(
  // 					idepthValue,
  // 					idepthValue,
  // 					VAR_GT_INIT_INITIAL,
  // 					VAR_GT_INIT_INITIAL,
  // 					20,
  // 				  Conf().debugDisplay );
  // 		}
  // 		else
  // 		{
  // 			currentDepthMap[idx].isValid = false;
  // 			currentDepthMap[idx].blacklisted = 0;
  // 		}
  // 	}
  // }
  //
  //
  // activeKeyFrame->setDepth(currentDepthMap);
}

//=== "Public interface" functions ==
bool DepthMap::updateDepthFrom(const Frame::SharedPtr &updateFrame) {
  // assert(isValid());

  // LOG(INFO) << "In DepthMap::updateKeyframe";

  Timer timeAll;

  // oldest_referenceFrame = referenceFrames.front();
  // newest_referenceFrame = referenceFrames.back();
  // referenceFrameByID.clear();
  // referenceFrameByID_offset = oldest_referenceFrame->id();
  //
  // for(std::shared_ptr<Frame> frame : referenceFrames)
  // {

  Sim3 refToKf;
  if (updateFrame->trackingParent()->id() == frame()->id())
    refToKf = updateFrame->pose->thisToParent_raw;
  else
    refToKf = frame()->getCamToWorld().inverse() * updateFrame->getCamToWorld();

  updateFrame->prepareForStereoWith(frame(), refToKf, 0);

  // while((int)referenceFrameByID.size() + referenceFrameByID_offset <=
  // frame->id()) 	referenceFrameByID.push_back(frame);

  //	}

  resetCounters();

  // if(plotStereoImages) _debugImages.plotUpdateKeyFrame( activeKeyFrame,
  // oldest_referenceFrame, newest_referenceFrame );

  {
    Timer time;
    observeDepth(updateFrame);
    _perf.observe.update(time);
  }

  // if(rand()%10==0)
  {
    Timer time;
    regularizeDepthMapFillHoles();
    _perf.fillHoles.update(time);
  }

  {
    Timer time;
    regularizeDepthMap(false, VAL_SUM_MIN_FOR_KEEP);
    _perf.regularize.update(time);
  }

  _perf.update.update(timeAll);

  if (plotStereoImages)
    _debugImages.displayUpdateKeyFrame();

  LOGF_IF(DEBUG, Conf().print.lineStereoStatistics,
          "ST: calls %6d, comp %6d, int %7d; good %6d (%.0f%%), neg %6d "
          "(%.0f%%); interp %6d / %6d / %6d\n",
          runningStats.num_stereo_calls, runningStats.num_stereo_comparisons,
          runningStats.num_pixelInterpolations,
          runningStats.num_stereo_successfull,
          100 * runningStats.num_stereo_successfull /
              (float)runningStats.num_stereo_calls,
          runningStats.num_stereo_negative,
          100 * runningStats.num_stereo_negative /
              (float)runningStats.num_stereo_successfull,
          runningStats.num_stereo_interpPre, runningStats.num_stereo_interpNone,
          runningStats.num_stereo_interpPost);

  LOGF_IF(DEBUG, Conf().print.lineStereoFails,
          "ST-ERR: oob %d (scale %d, inf %d, near %d); err %d (%d uncl; %d "
          "end; zro: %d btw, %d no, %d two; %d big)\n",
          runningStats.num_stereo_rescale_oob +
              runningStats.num_stereo_inf_oob +
              runningStats.num_stereo_near_oob,
          runningStats.num_stereo_rescale_oob, runningStats.num_stereo_inf_oob,
          runningStats.num_stereo_near_oob,
          runningStats.num_stereo_invalid_unclear_winner +
              runningStats.num_stereo_invalid_atEnd +
              runningStats.num_stereo_invalid_inexistantCrossing +
              runningStats.num_stereo_invalid_noCrossing +
              runningStats.num_stereo_invalid_twoCrossing +
              runningStats.num_stereo_invalid_bigErr,
          runningStats.num_stereo_invalid_unclear_winner,
          runningStats.num_stereo_invalid_atEnd,
          runningStats.num_stereo_invalid_inexistantCrossing,
          runningStats.num_stereo_invalid_noCrossing,
          runningStats.num_stereo_invalid_twoCrossing,
          runningStats.num_stereo_invalid_bigErr);

  return true;
}

// void DepthMap::invalidate()
// {
// 	if(activeKeyFrame==0) return;
// 	activeKeyFrame=0;
// 	activeKeyFramelock.unlock();
// }

void DepthMap::propagateFrom(const DepthMap::SharedPtr &other,
                             float &rescaleFactor, bool stereo_depth) {
  // assert(isValid());
  // assert(new_keyframe != nullptr);
  // assert(new_keyframe->hasTrackingParent());
  //
  // //boost::shared_lock<boost::shared_mutex> lock =
  // activeKeyFrame->getActiveLock(); boost::shared_lock<boost::shared_mutex>
  // lock2 = new_keyframe->getActiveLock(); std::lock_guard<std::mutex> lock(
  // new_keyframe->frameMutex );

  // TODO.  Check that my frame tracked on other's frame

  Timer timeAll;

  resetCounters();

  // rescaleFactor = _meanIdepthRatio;
  rescaleFactor = 1.0;

  // if(plotStereoImages)  _debugImages.plotNewKeyFrame( new_keyframe );

  // if( new_keyframe->hasIDepthBeenSet() )
  // 	LOG(INFO) << "Creating new KeyFrame but it already has depth
  // information";

  {
    Timer time;
    if (stereo_depth) {
      propagateDepthFromSet(other, rescaleFactor);
    } else {
      propagateDepthFrom(other, rescaleFactor);
    }
    _perf.propagate.update(time);
  }

  // activeKeyFrame = new_keyframe;
  // activeKeyFramelock = activeKeyFrame->getActiveLock();
  // //activeKeyFrameImageData = new_keyframe->image(0);
  // activeKeyFrameIsReactivated = false;

  {
    int validCount = 0;
    for (DepthMapPixelHypothesis *source = currentDepthMap;
         source < currentDepthMap + Conf().slamImageSize.area(); source++) {
      if (!source->isValid)
        continue;
      ++validCount;
    }
    LOG(DEBUG) << "After propagation, DepthMap has " << validCount
               << " valid points";
  }

  {
    Timer time;
    regularizeDepthMap(true, VAL_SUM_MIN_FOR_KEEP);
    _perf.regularize.update(time);
  }

  {
    Timer time;
    regularizeDepthMapFillHoles();
    _perf.fillHoles.update(time);
  }

  {
    Timer time;
    regularizeDepthMap(false, VAL_SUM_MIN_FOR_KEEP);
    _perf.regularize.update(time);
  }

  // make mean inverse depth be one.
  float sumIdepth = 0, numIdepth = 0;
  int validCount = 0;
  for (DepthMapPixelHypothesis *source = currentDepthMap;
       source < currentDepthMap + Conf().slamImageSize.area(); source++) {
    if (!source->isValid)
      continue;

    sumIdepth += source->idepth_smoothed;
    numIdepth++;
    ++validCount;
  }

  LOG(DEBUG) << "DepthMap has " << validCount << " valid points";

  // rescaleFactor = numIdepth / sumIdepth;
  LOG(INFO) << "disparity iDepthMean: " << _set->disparity.iDepthMean
            << " lsd iDepthMean: " << _meanIDepth
            << " depth ratio: " << _meanIdepthRatio;
  LOG(INFO) << "rescaling";
  // rescaleFactor = _meanIdepthRatio;
  rescaleFactor = 1.0;
  float rescaleFactor2 = rescaleFactor * rescaleFactor;
  uint8_t *iDepthValid = _set->disparity.iDepthValid;
  for (DepthMapPixelHypothesis *source = currentDepthMap;
       source < currentDepthMap + Conf().slamImageSize.area(); source++) {
    if (!source->isValid)
      continue;
    bool valid = *iDepthValid;
    if (!valid) {
      source->idepth *= rescaleFactor;
    }
    // RESCALE TO BE SAME SIZE AS Disparity
    // source->idepth *= _meanIdepthRatio;
    source->idepth_smoothed *= rescaleFactor;
    source->idepth_var *= rescaleFactor2;
    source->idepth_var_smoothed *= rescaleFactor2;

    iDepthValid++;
  }

  _perf.create.update(timeAll);

  if (plotStereoImages)
    _debugImages.displayNewKeyFrame();
}

void DepthMap::finalize() {
  //	assert(isValid());

  Timer timeAll;

  {
    Timer time;
    regularizeDepthMapFillHoles();
    _perf.fillHoles.update(time);
  }

  {
    Timer time;
    regularizeDepthMap(false, VAL_SUM_MIN_FOR_KEEP);
    _perf.regularize.update(time);
  }

  _perf.finalize.update(timeAll);
}

// void DepthMap::activateExistingKF( const Frame::SharedPtr &kf)
// {
// 	assert(kf->hasIDepthBeenSet());
//
// 	activeKeyFramelock = kf->getActiveLock();
// 	activeKeyFrame = kf;
//
// 	const float* idepth = activeKeyFrame->idepth_reAct();
// 	const float* idepthVar = activeKeyFrame->idepthVar_reAct();
// 	const unsigned char* validity = activeKeyFrame->validity_reAct();
//
// 	DepthMapPixelHypothesis* pt = currentDepthMap;
// 	activeKeyFrame->numMappedOnThis = 0;
// 	activeKeyFrame->numFramesTrackedOnThis = 0;
// 	//activeKeyFrameImageData = activeKeyFrame->image(0);
// 	activeKeyFrameIsReactivated = true;
//
// 	for(int y=0;y<Conf().slamImageSize.height;y++)
// 	{
// 		for(int x=0;x<Conf().slamImageSize.width;x++)
// 		{
// 			int idx = x + y*Conf().slamImageSize.width;
// 			if(*idepthVar > 0)
// 			{
// 				*pt = DepthMapPixelHypothesis(
// 						*idepth,
// 						*idepthVar,
// 						*validity,
// 					  Conf().debugDisplay );
// 			}
// 			else
// 			{
// 				currentDepthMap[idx].isValid = false;
// 				currentDepthMap[idx].blacklisted = (*idepthVar
// == -2) ? MIN_BLACKLIST-1 : 0;
// 			}
//
// 			idepth++;
// 			idepthVar++;
// 			validity++;
// 			pt++;
// 		}
// 	}
//
// 	regularizeDepthMap(false, VAL_SUM_MIN_FOR_KEEP);
// }

void DepthMap::resetCounters() {
  runningStats.num_stereo_comparisons = 0;
  runningStats.num_pixelInterpolations = 0;
  runningStats.num_stereo_calls = 0;

  runningStats.num_stereo_rescale_oob = 0;
  runningStats.num_stereo_inf_oob = 0;
  runningStats.num_stereo_near_oob = 0;
  runningStats.num_stereo_invalid_unclear_winner = 0;
  runningStats.num_stereo_invalid_atEnd = 0;
  runningStats.num_stereo_invalid_inexistantCrossing = 0;
  runningStats.num_stereo_invalid_twoCrossing = 0;
  runningStats.num_stereo_invalid_noCrossing = 0;
  runningStats.num_stereo_invalid_bigErr = 0;
  runningStats.num_stereo_interpPre = 0;
  runningStats.num_stereo_interpPost = 0;
  runningStats.num_stereo_interpNone = 0;
  runningStats.num_stereo_negative = 0;
  runningStats.num_stereo_successfull = 0;

  runningStats.num_observe_created = 0;
  runningStats.num_observe_create_attempted = 0;
  runningStats.num_observe_updated = 0;
  runningStats.num_observe_update_attempted = 0;
  runningStats.num_observe_skipped_small_epl = 0;
  runningStats.num_observe_skipped_small_epl_grad = 0;
  runningStats.num_observe_skipped_small_epl_angle = 0;
  runningStats.num_observe_transit_finalizing = 0;
  runningStats.num_observe_transit_idle_oob = 0;
  runningStats.num_observe_transit_idle_scale_angle = 0;
  runningStats.num_observe_trans_idle_exhausted = 0;
  runningStats.num_observe_inconsistent_finalizing = 0;
  runningStats.num_observe_inconsistent = 0;
  runningStats.num_observe_notfound_finalizing2 = 0;
  runningStats.num_observe_notfound_finalizing = 0;
  runningStats.num_observe_notfound = 0;
  runningStats.num_observe_skip_fail = 0;
  runningStats.num_observe_skip_oob = 0;
  runningStats.num_observe_good = 0;
  runningStats.num_observe_good_finalizing = 0;
  runningStats.num_observe_state_finalizing = 0;
  runningStats.num_observe_state_initializing = 0;
  runningStats.num_observe_skip_alreadyGood = 0;
  runningStats.num_observe_addSkip = 0;

  runningStats.num_observe_blacklisted = 0;
}

//=== Actual working functions ====
void DepthMap::observeDepth(const Frame::SharedPtr &updateFrame) {
  _observeFrame = updateFrame;
  debugDepthImg = cv::Mat::zeros(Conf().slamImageSize.height,
                                 Conf().slamImageSize.width, CV_32FC1);
  threadReducer.reduce(
      boost::bind(&DepthMap::observeDepthRow, this, _1, _2, _3), 3,
      Conf().slamImageSize.height - 3, 10);

  // cv::imshow("debugdepth", debugDepthImg);

  LOGF_IF(DEBUG, Conf().print.observeStatistics,
          "OBSERVE (%d): %d / %d created; %d / %d updated; %d skipped; %d "
          "init-blacklisted",
          frame()->id(), runningStats.num_observe_created,
          runningStats.num_observe_create_attempted,
          runningStats.num_observe_updated,
          runningStats.num_observe_update_attempted,
          runningStats.num_observe_skip_alreadyGood,
          runningStats.num_observe_blacklisted);

  LOGF_IF(DEBUG, Conf().print.observePurgeStatistics,
          "OBS-PRG (%d): Good: %d; inconsistent: %d; notfound: %d; oob: %d; "
          "failed: %d; addSkip: %d;",
          frame()->id(), runningStats.num_observe_good,
          runningStats.num_observe_inconsistent,
          runningStats.num_observe_notfound, runningStats.num_observe_skip_oob,
          runningStats.num_observe_skip_fail, runningStats.num_observe_addSkip);
}

void DepthMap::observeDepthRow(int yMin, int yMax, RunningStats *stats) {
  const float *keyFrameMaxGradBuf = frame()->maxGradients(0);

  int successes = 0;
  // debugDepthImg = cv::Mat::zeros(
  //     cv::Size(Conf().slamImageSize.height, Conf().slamImageSize.width),
  //     CV_32FC1);

  for (int y = yMin; y < yMax; y++)
    for (int x = 3; x < Conf().slamImageSize.width - 3; x++) {
      int idx = x + y * Conf().slamImageSize.width;
      DepthMapPixelHypothesis *target = currentDepthMap + idx;
      bool hasHypothesis = target->isValid;

      // ======== 1. check absolute grad =========
      if (hasHypothesis && keyFrameMaxGradBuf[idx] < MIN_ABS_GRAD_DECREASE) {
        target->isValid = false;
        continue;
      }

      if (keyFrameMaxGradBuf[idx] < MIN_ABS_GRAD_CREATE ||
          target->blacklisted < MIN_BLACKLIST)
        continue;

      bool success;
      if (!hasHypothesis)
        success = observeDepthCreate(x, y, idx, stats);
      else
        success = observeDepthUpdate(x, y, idx, keyFrameMaxGradBuf, stats);

      if (success)
        successes++;
    }
}

bool DepthMap::observeDepthCreate(const int &x, const int &y, const int &idx,
                                  RunningStats *const &stats) {
  DepthMapPixelHypothesis *target = currentDepthMap + idx;

  //	Frame::SharedPtr _observeFrame( activeKeyFrameIsReactivated ?
  // newest_referenceFrame : oldest_referenceFrame );

  if (_observeFrame->isTrackingParent(frame()->id())) {
    bool *wasGoodDuringTracking = _observeFrame->refPixelWasGoodNoCreate();
    if (wasGoodDuringTracking != 0 &&
        !wasGoodDuringTracking[(x >> SE3TRACKING_MIN_LEVEL) +
                               (Conf().slamImageSize.width >>
                                SE3TRACKING_MIN_LEVEL) *
                                   (y >> SE3TRACKING_MIN_LEVEL)]) {
      if (plotStereoImages)
        _debugImages.setHypothesisHandling(
            x, y, cv::Vec3b(255, 0, 0)); // BLUE for SKIPPED NOT GOOD TRACKED
      return false;
    }
  }

  float epx, epy;
  bool isGood = makeAndCheckEPL(x, y, _observeFrame.get(), &epx, &epy, stats);
  if (!isGood)
    return false;

  stats->num_observe_create_attempted++;

  float new_u = x;
  float new_v = y;
  float result_idepth, result_var, result_eplLength;
  float error =
      doLineStereo(new_u, new_v, epx, epy, 0.0f, 1.0f, 1.0f / MIN_DEPTH,
                   _observeFrame.get(), _observeFrame->image(0), result_idepth,
                   result_var, result_eplLength, stats);

  if (error == -3 || error == -2) {
    target->blacklisted--;
    stats->num_observe_blacklisted++;
  }

  if (error < 0 || result_var > MAX_VAR)
    return false;

  result_idepth = UNZERO(result_idepth);

  // add hypothesis
  // *target = DepthMapPixelHypothesis(result_idepth, result_var,
  //                                   VALIDITY_COUNTER_INITIAL_OBSERVE,
  //                                   Conf().debugDisplay);

  if (plotStereoImages)
    _debugImages.setHypothesisHandling(
        x, y, cv::Vec3b(255, 255, 255)); // white for GOT CREATED
  stats->num_observe_created++;

  return true;
}

bool DepthMap::observeDepthUpdate(const int &x, const int &y, const int &idx,
                                  const float *keyFrameMaxGradBuf,
                                  RunningStats *const &stats) {
  DepthMapPixelHypothesis *target = currentDepthMap + idx;
  //	Frame::SharedPtr refFrame;

  // TODO.  Not sure what's happening in this code block.
  // if(!activeKeyFrameIsReactivated)
  // {
  // 	if((int)target->nextStereoFrameMinID - referenceFrameByID_offset >=
  // (int)referenceFrameByID.size())
  // 	{
  // 		if(plotStereoImages) _debugImages.setHypothesisHandling(x,y,
  // cv::Vec3b(0,255,0));       // GREEN FOR skip
  //
  // 		stats->num_observe_skip_alreadyGood++;
  // 		return false;
  // 	}
  //
  // 	if((int)target->nextStereoFrameMinID - referenceFrameByID_offset < 0)
  // 		refFrame = oldest_referenceFrame;
  // 	else
  // 		refFrame = referenceFrameByID[(int)target->nextStereoFrameMinID
  // - referenceFrameByID_offset];
  // }
  // else
  // 	refFrame = newest_referenceFrame;

  if (_observeFrame->isTrackingParent(frame()->id())) {
    bool *wasGoodDuringTracking = _observeFrame->refPixelWasGoodNoCreate();
    if (wasGoodDuringTracking != 0 &&
        !wasGoodDuringTracking[(x >> SE3TRACKING_MIN_LEVEL) +
                               (Conf().slamImageSize.width >>
                                SE3TRACKING_MIN_LEVEL) *
                                   (y >> SE3TRACKING_MIN_LEVEL)]) {
      if (plotStereoImages)
        _debugImages.setHypothesisHandling(
            x, y, cv::Vec3b(255, 0, 0)); // BLUE for SKIPPED NOT GOOD TRACKED
      return false;
    }
  }

  float epx, epy;
  bool isGood = makeAndCheckEPL(x, y, _observeFrame.get(), &epx, &epy, stats);
  if (!isGood)
    return false;

  // which exact point to track, and where from.
  float sv = sqrt(target->idepth_var_smoothed);
  float min_idepth = target->idepth_smoothed - sv * STEREO_EPL_VAR_FAC;
  float max_idepth = target->idepth_smoothed + sv * STEREO_EPL_VAR_FAC;
  if (min_idepth < 0)
    min_idepth = 0;
  if (max_idepth > 1 / MIN_DEPTH)
    max_idepth = 1 / MIN_DEPTH;

  stats->num_observe_update_attempted++;

  float result_idepth, result_var, result_eplLength;

  float prior_idepth = target->idepth_smoothed;
  bool valid = false;
  // if (_set != nullptr) {
  //   float *iDepth = _set->disparity.iDepth;
  //   uint8_t *iDepthValid = _set->disparity.iDepthValid;
  //   iDepth += idx;
  //   iDepthValid += idx;
  //   bool valid = *iDepthValid;
  //   if (valid) {
  //     // if (0) {
  //     float prior_idepth = *iDepth;
  //     debugDepthImg.at<float>(y, x) = 1 / prior_idepth;
  //     // result_idepth = prior_idepth;
  //   } else {
  //     debugDepthImg.at<float>(y, x) = 0.0;
  //   }
  // }

  float error =
      doLineStereo(x, y, epx, epy, min_idepth, prior_idepth, max_idepth,
                   _observeFrame.get(), _observeFrame->image(0), result_idepth,
                   result_var, result_eplLength, stats);
  // if (_set != nullptr) {
  //   float *iDepth = _set->disparity.iDepth;
  //   uint8_t *iDepthValid = _set->disparity.iDepthValid;
  //   iDepth += idx;
  //   iDepthValid += idx;
  //   // valid = *iDepthValid;
  //   if (valid) {
  //     // std::cout << "valid" << std::endl;
  //     float disparty_depth = *iDepth;
  //     // std::cout << "result_idepth " << result_idepth << " disparty_depth "
  //     //          << disparty_depth << std::endl;
  //     result_idepth = disparty_depth; //(disparty_depth + result_idepth) / 2;
  //   }
  // }

  float diff = result_idepth - target->idepth_smoothed;

  // if oob: (really out of bounds)
  if (error == -1) {
    // do nothing, pixel got oob, but is still in bounds in original. I will
    // want to try again.
    stats->num_observe_skip_oob++;

    if (plotStereoImages)
      _debugImages.setHypothesisHandling(x, y,
                                         cv::Vec3b(0, 0, 255)); // RED FOR OOB
    return false;
  }

  // if just not good for stereo (e.g. some inf / nan occured; has
  // inconsistent minimum; ..)
  else if (error == -2) {
    stats->num_observe_skip_fail++;

    if (plotStereoImages)
      _debugImages.setHypothesisHandling(
          x, y, cv::Vec3b(255, 0, 255)); // PURPLE FOR NON-GOOD

    target->validity_counter -= VALIDITY_COUNTER_DEC;
    if (target->validity_counter < 0)
      target->validity_counter = 0;

    target->nextStereoFrameMinID = 0;

    target->idepth_var *= FAIL_VAR_INC_FAC;
    if (target->idepth_var > MAX_VAR) {
      target->isValid = false;
      target->blacklisted--;
    }
    return false;
  }

  // if not found (error too high)
  else if (error == -3) {
    stats->num_observe_notfound++;
    if (plotStereoImages)
      _debugImages.setHypothesisHandling(
          x, y, cv::Vec3b(0, 0, 0)); // BLACK FOR big not-found

    return false;
  }

  else if (error == -4) {
    if (plotStereoImages)
      _debugImages.setHypothesisHandling(
          x, y, cv::Vec3b(0, 0, 0)); // BLACK FOR big not-found
    return false;
  }

  // if inconsistent
  else if (DIFF_FAC_OBSERVE * diff * diff >
           result_var + target->idepth_var_smoothed) {
    stats->num_observe_inconsistent++;
    if (plotStereoImages)
      _debugImages.setHypothesisHandling(
          x, y, cv::Vec3b(255, 255, 0)); // Turkoise FOR big inconsistent

    target->idepth_var *= FAIL_VAR_INC_FAC;
    if (target->idepth_var > MAX_VAR)
      target->isValid = false;

    return false;
  }

  else {
    // one more successful observation!
    stats->num_observe_good++;
    stats->num_observe_updated++;

    // do textbook ekf update:
    // increase var by a little (prediction-uncertainty)
    float id_var = target->idepth_var * SUCC_VAR_INC_FAC;

    // update var with observation
    float w = result_var / (result_var + id_var);
    float new_idepth = (1 - w) * result_idepth + w * target->idepth;
    // target->idepth = UNZERO(new_idepth);

    if (valid) {
      target->idepth = UNZERO(result_idepth);
    }
    // else {
    //   float new_idepth = (1 - w) * result_idepth + w * target->idepth;
    //   target->idepth = UNZERO(new_idepth);
    // }

    // variance can only decrease from observation; never increase.
    id_var = id_var * w;
    if (id_var < target->idepth_var)
      target->idepth_var = id_var;

    // increase validity!
    target->validity_counter += VALIDITY_COUNTER_INC;
    float absGrad = keyFrameMaxGradBuf[idx];
    if (target->validity_counter >
        VALIDITY_COUNTER_MAX +
            absGrad * (VALIDITY_COUNTER_MAX_VARIABLE) / 255.0f)
      target->validity_counter =
          VALIDITY_COUNTER_MAX +
          absGrad * (VALIDITY_COUNTER_MAX_VARIABLE) / 255.0f;

    // increase Skip!
    if (result_eplLength < MIN_EPL_LENGTH_CROP) {
      //!!TODO.  Disabled this because I can't get to the statistics from
      //! here...
      // float inc = _parent->numFramesTrackedOnThis /
      // (float)(_parent->numMappedOnThis+5);
      float inc = 3;

      if (inc < 3)
        inc = 3;

      inc += ((int)(result_eplLength * 10000) % 2);

      stats->num_observe_addSkip++;

      if (result_eplLength < 0.5 * MIN_EPL_LENGTH_CROP)
        inc *= 3;

      target->nextStereoFrameMinID = _observeFrame->id() + inc;
    }

    if (plotStereoImages)
      _debugImages.setHypothesisHandling(
          x, y, cv::Vec3b(0, 255, 255)); // yellow for GOT UPDATED

    return true;
  }
}

bool DepthMap::makeAndCheckEPL(const int x, const int y, const Frame *const ref,
                               float *pepx, float *pepy,
                               RunningStats *const stats) {
  int idx = x + y * Conf().slamImageSize.width;

  const float fx = ref->fx(), fy = ref->fy(), cx = ref->cx(), cy = ref->cy();

  // ======= make epl ========
  // calculate the plane spanned by the two camera centers and the point
  // (x,y,1) intersect it with the keyframe's image plane (at depth=1)
  float epx = -fx * ref->thisToOther_t[0] + ref->thisToOther_t[2] * (x - cx);
  float epy = -fy * ref->thisToOther_t[1] + ref->thisToOther_t[2] * (y - cy);

  if (isnanf(epx + epy))
    return false;

  // ======== check epl length =========
  float eplLengthSquared = epx * epx + epy * epy;
  if (eplLengthSquared < MIN_EPL_LENGTH_SQUARED) {
    stats->num_observe_skipped_small_epl++;
    return false;
  }

  // ===== check epl-grad magnitude ======
  float gx =
      activeKeyFrameImageData()[idx + 1] - activeKeyFrameImageData()[idx - 1];
  float gy = activeKeyFrameImageData()[idx + Conf().slamImageSize.width] -
             activeKeyFrameImageData()[idx - Conf().slamImageSize.width];
  float eplGradSquared = gx * epx + gy * epy;
  eplGradSquared = eplGradSquared * eplGradSquared /
                   eplLengthSquared; // square and norm with epl-length

  if (eplGradSquared < MIN_EPL_GRAD_SQUARED) {
    stats->num_observe_skipped_small_epl_grad++;
    return false;
  }

  // ===== check epl-grad angle ======
  if (eplGradSquared / (gx * gx + gy * gy) < MIN_EPL_ANGLE_SQUARED) {
    stats->num_observe_skipped_small_epl_angle++;
    return false;
  }

  // ===== DONE - return "normalized" epl =====
  float fac = GRADIENT_SAMPLE_DIST / sqrt(eplLengthSquared);
  *pepx = epx * fac;
  *pepy = epy * fac;

  return true;
}
void DepthMap::propagateDepthFromSet(const DepthMap::SharedPtr &other,
                                     float &rescaleFactor) {
  CHECK(_set != nullptr) << "SET HAS NOT BEEN SET";
  LOG(WARNING) << "Entering Propagate Depth From Set";
  runningStats.num_prop_removed_out_of_bounds = 0;
  runningStats.num_prop_removed_colorDiff = 0;
  runningStats.num_prop_removed_validity = 0;
  runningStats.num_prop_grad_decreased = 0;
  runningStats.num_prop_color_decreased = 0;
  runningStats.num_prop_attempts = 0;
  runningStats.num_prop_occluded = 0;
  runningStats.num_prop_created = 0;
  runningStats.num_prop_merged = 0;
  runningStats.num_prop_source_invalid = 0;

  // LOGF_IF(WARNING, ( !new_keyframe->isTrackingParent( activeKeyFrame) ),
  // 		"propagating depth from current keyframe %d to new keyframe %d,
  // which was tracked on a different frame (%d).  While this should work, it
  // is not recommended.", 		activeKeyFrame->id(),
  // new_keyframe->id(), 		new_keyframe->trackingParent()->id());

  // wipe depthmap
  for (DepthMapPixelHypothesis *pt =
           currentDepthMap + Conf().slamImageSize.area() - 1;
       pt >= currentDepthMap; pt--) {
    pt->isValid = false;
    pt->blacklisted = 0;
  }

  // re-usable values.
  const SE3 oldToNew_SE3 =
      se3FromSim3(frame()->pose->thisToParent_raw).inverse();
  const Eigen::Vector3f trafoInv_t = oldToNew_SE3.translation().cast<float>();
  const Eigen::Matrix3f trafoInv_R =
      oldToNew_SE3.rotationMatrix().matrix().cast<float>();

  const bool *trackingWasGood = (frame()->isTrackingParent(other->frame()->id())
                                     ? frame()->refPixelWasGoodNoCreate()
                                     : nullptr);

  const float *activeKFImageData = other->frame()->image(0);
  const float *newKFMaxGrad = frame()->maxGradients(0);
  const float *newKFImageData = frame()->image(0);

  const float fx = frame()->fx(), fy = frame()->fy(), cx = frame()->cx(),
              cy = frame()->cy(), fxi = frame()->fxi(), fyi = frame()->fyi(),
              cxi = frame()->cxi(), cyi = frame()->cyi();

  /* plot
   */
  /*
 float *iDepthPlot = _set->disparity.iDepth;
 uint8_t *iDepthValidPlot = _set->disparity.iDepthValid;
 cv::Mat imgD(Conf().slamImageSize.height, Conf().slamImageSize.width,
 CV_32F); for (int y = 1; y < (Conf().slamImageSize.height - 1); y++) { for
 (int x = 1; x < (Conf().slamImageSize.width - 1); x++) {
     ++iDepthPlot;
     ++iDepthValidPlot;
     bool val = *iDepthValidPlot;
     float dep = *iDepthPlot;
     if (val) {
       imgD.at<float>(x, y) = dep;
     } else {
       imgD.at<float>(x, y) = 0;
     }
   }
 }
 cv::imshow("depth", imgD);
 cv::waitKey(1);
 */

  // go through all pixels of OLD image, propagating forwards.
  float *iDepth = _set->disparity.iDepth;
  uint8_t *iDepthValid = _set->disparity.iDepthValid;
  float dispartiyMapSum = 0;
  float iDepthSum = 0;
  float iDepthCount = 0;

  // cv::Mat depthImg2(Conf().slamImageSize.height, Conf().slamImageSize.width,
  //                   CV_32FC1);
  std::cout << "R: " << trafoInv_R << "T: " << trafoInv_t << std::endl;
  for (int y = 0; y < Conf().slamImageSize.height; y++)
    for (int x = 0; x < Conf().slamImageSize.width; x++) {
      const DepthMapPixelHypothesis *source = other->hypothesisAt(x, y);

      if (!source->isValid) {
        runningStats.num_prop_source_invalid++;
        continue;
      }

      runningStats.num_prop_attempts++;
      Eigen::Vector3f pn;
      bool valid = *iDepthValid;

      float id = source->idepth_smoothed;

      if (valid) {
        id = *iDepth;
      }
      float new_idepth = id;
      pn = (trafoInv_R * Eigen::Vector3f(x * fxi + cxi, y * fyi + cyi, 1.0f)) /
               id +
           trafoInv_t;
      // else {
      //   float current_depth = 1 / (*iDepth);
      //   // Eigen::Vector3f pn1 =
      //   //     (trafoInv_R *
      //   //      Eigen::Vector3f(x * fxi + cxi, y * fyi + cyi, current_depth))
      //   //      +
      //   //     trafoInv_t;
      //   Eigen::Vector3f pn1 =
      //       (trafoInv_R * Eigen::Vector3f(x * fxi + cxi, y * fyi +
      //       cyi, 1.0f)) /
      //           current_depth +
      //       trafoInv_t;
      //
      //   Eigen::Vector3f pn2 =
      //       (trafoInv_R * Eigen::Vector3f(x * fxi + cxi, y * fyi +
      //       cyi, 1.0f)) /
      //           source->idepth_smoothed +
      //       trafoInv_t;
      //   // pn = (pn1 + pn2) / 2;
      //   pn = pn1;
      // }
      // new_idepth = (1.0f / pn[2]);
      // if (valid) {
      //   iDepthCount++;
      //   iDepthSum += new_idepth;
      //   dispartiyMapSum += *iDepth;
      // }
      float u_new = pn[0] * new_idepth * fx + cx;
      float v_new = pn[1] * new_idepth * fy + cy;

      // check if still within image, if not: DROP.
      if (!(u_new > 2.1f && v_new > 2.1f &&
            u_new < Conf().slamImageSize.width - 3.1f &&
            v_new < Conf().slamImageSize.height - 3.1f)) {
        runningStats.num_prop_removed_out_of_bounds++;
        continue;
      }

      int newIDX = (int)(u_new + 0.5f) +
                   ((int)(v_new + 0.5f)) * Conf().slamImageSize.width;
      float destAbsGrad = newKFMaxGrad[newIDX];

      if (trackingWasGood) {
        if (!trackingWasGood[(x >> SE3TRACKING_MIN_LEVEL) +
                             (Conf().slamImageSize.width >>
                              SE3TRACKING_MIN_LEVEL) *
                                 (y >> SE3TRACKING_MIN_LEVEL)] ||
            destAbsGrad < MIN_ABS_GRAD_DECREASE) {
          runningStats.num_prop_removed_colorDiff++;
          continue;
        }
      } else {
        float sourceColor =
            activeKFImageData[x + y * Conf().slamImageSize.width];
        float destColor = getInterpolatedElement(newKFImageData, u_new, v_new,
                                                 Conf().slamImageSize.width);

        float residual = destColor - sourceColor;

        if (residual * residual /
                    (MAX_DIFF_CONSTANT +
                     MAX_DIFF_GRAD_MULT * destAbsGrad * destAbsGrad) >
                1.0f ||
            destAbsGrad < MIN_ABS_GRAD_DECREASE) {
          runningStats.num_prop_removed_colorDiff++;
          continue;
        }
      }

      DepthMapPixelHypothesis *targetBest = currentDepthMap + newIDX;

      // large idepth = point is near = large increase in variance.
      // small idepth = point is far = small increase in variance.
      float idepth_ratio_4 = new_idepth / source->idepth_smoothed;
      idepth_ratio_4 *= idepth_ratio_4;
      idepth_ratio_4 *= idepth_ratio_4;

      float new_var = idepth_ratio_4 * source->idepth_var;

      // check for occlusion
      if (targetBest->isValid) {
        // if they occlude one another, one gets removed.
        float diff = targetBest->idepth - new_idepth;
        if (DIFF_FAC_PROP_MERGE * diff * diff >
            new_var + targetBest->idepth_var) {
          if (new_idepth < targetBest->idepth) {
            runningStats.num_prop_occluded++;
            continue;
          } else {
            runningStats.num_prop_occluded++;
            targetBest->isValid = false;
          }
        }
      }

      if (!targetBest->isValid) {
        runningStats.num_prop_created++;

        *targetBest = DepthMapPixelHypothesis(
            new_idepth, new_var, source->validity_counter, Conf().debugDisplay);

      } else {
        runningStats.num_prop_merged++;

        // merge idepth ekf-style
        float w = new_var / (targetBest->idepth_var + new_var);
        float merged_new_idepth =
            w * targetBest->idepth + (1.0f - w) * new_idepth;

        // merge validity
        int merged_validity =
            source->validity_counter + targetBest->validity_counter;
        if (merged_validity >
            VALIDITY_COUNTER_MAX + (VALIDITY_COUNTER_MAX_VARIABLE))
          merged_validity =
              VALIDITY_COUNTER_MAX + (VALIDITY_COUNTER_MAX_VARIABLE);

        *targetBest = DepthMapPixelHypothesis(
            merged_new_idepth,
            1.0f / (1.0f / targetBest->idepth_var + 1.0f / new_var),
            merged_validity, Conf().debugDisplay);
      }
      iDepth++;
      iDepthValid++;
    }

  // cv::imshow("depthImg2", depthImg2);
  // cv::waitKey(1);

  _meanIDepth = iDepthSum / iDepthCount;
  float dispartiyMapIDepth = dispartiyMapSum / iDepthCount;
  if (_set != nullptr) {
    //_meanIdepthRatio = _set->disparity.iDepthMean / _meanIDepth;
    _meanIdepthRatio = dispartiyMapIDepth / _meanIDepth;
  } else {
    _meanIdepthRatio = 1;
  }
  // swap!
  //	std::swap(currentDepthMap, otherDepthMap);

  LOGF_IF(INFO, Conf().print.propagationStatistics,
          "PROPAGATE: %d invalid, %d: %d drop (%d oob, %d color); %d created; "
          "%d merged; %d occluded. %d col-dec, %d grad-dec.",
          runningStats.num_prop_source_invalid, runningStats.num_prop_attempts,
          runningStats.num_prop_removed_validity +
              runningStats.num_prop_removed_out_of_bounds +
              runningStats.num_prop_removed_colorDiff,
          runningStats.num_prop_removed_out_of_bounds,
          runningStats.num_prop_removed_colorDiff,
          runningStats.num_prop_created, runningStats.num_prop_merged,
          runningStats.num_prop_occluded, runningStats.num_prop_color_decreased,
          runningStats.num_prop_grad_decreased);
} // namespace lsd_slam

void DepthMap::propagateDepthFrom(const DepthMap::SharedPtr &other,
                                  float &rescaleFactor) {
  runningStats.num_prop_removed_out_of_bounds = 0;
  runningStats.num_prop_removed_colorDiff = 0;
  runningStats.num_prop_removed_validity = 0;
  runningStats.num_prop_grad_decreased = 0;
  runningStats.num_prop_color_decreased = 0;
  runningStats.num_prop_attempts = 0;
  runningStats.num_prop_occluded = 0;
  runningStats.num_prop_created = 0;
  runningStats.num_prop_merged = 0;
  runningStats.num_prop_source_invalid = 0;

  // LOGF_IF(WARNING, ( !new_keyframe->isTrackingParent( activeKeyFrame) ),
  // 		"propagating depth from current keyframe %d to new
  // keyframe %d, which was tracked on a different frame (%d).  While this
  // should work, it is not recommended.", 		activeKeyFrame->id(),
  // new_keyframe->id(), 		new_keyframe->trackingParent()->id());

  // wipe depthmap
  for (DepthMapPixelHypothesis *pt =
           currentDepthMap + Conf().slamImageSize.area() - 1;
       pt >= currentDepthMap; pt--) {
    pt->isValid = false;
    pt->blacklisted = 0;
  }

  // re-usable values.
  const SE3 oldToNew_SE3 =
      se3FromSim3(frame()->pose->thisToParent_raw).inverse();
  const Eigen::Vector3f trafoInv_t = oldToNew_SE3.translation().cast<float>();
  const Eigen::Matrix3f trafoInv_R =
      oldToNew_SE3.rotationMatrix().matrix().cast<float>();

  const bool *trackingWasGood = (frame()->isTrackingParent(other->frame()->id())
                                     ? frame()->refPixelWasGoodNoCreate()
                                     : nullptr);

  const float *activeKFImageData = other->frame()->image(0);
  const float *newKFMaxGrad = frame()->maxGradients(0);
  const float *newKFImageData = frame()->image(0);

  const float fx = frame()->fx(), fy = frame()->fy(), cx = frame()->cx(),
              cy = frame()->cy(), fxi = frame()->fxi(), fyi = frame()->fyi(),
              cxi = frame()->cxi(), cyi = frame()->cyi();

  // go through all pixels of OLD image, propagating forwards.
  for (int y = 0; y < Conf().slamImageSize.height; y++)
    for (int x = 0; x < Conf().slamImageSize.width; x++) {
      const DepthMapPixelHypothesis *source = other->hypothesisAt(x, y);

      if (!source->isValid) {
        runningStats.num_prop_source_invalid++;
        continue;
      }

      runningStats.num_prop_attempts++;
      Eigen::Vector3f pn =
          (trafoInv_R * Eigen::Vector3f(x * fxi + cxi, y * fyi + cyi, 1.0f)) /
              source->idepth_smoothed +
          trafoInv_t;
      float new_idepth;
      new_idepth = 1.0f / pn[2];

      float u_new = pn[0] * new_idepth * fx + cx;
      float v_new = pn[1] * new_idepth * fy + cy;

      // check if still within image, if not: DROP.
      if (!(u_new > 2.1f && v_new > 2.1f &&
            u_new < Conf().slamImageSize.width - 3.1f &&
            v_new < Conf().slamImageSize.height - 3.1f)) {
        runningStats.num_prop_removed_out_of_bounds++;
        continue;
      }

      int newIDX = (int)(u_new + 0.5f) +
                   ((int)(v_new + 0.5f)) * Conf().slamImageSize.width;
      float destAbsGrad = newKFMaxGrad[newIDX];

      if (trackingWasGood) {
        if (!trackingWasGood[(x >> SE3TRACKING_MIN_LEVEL) +
                             (Conf().slamImageSize.width >>
                              SE3TRACKING_MIN_LEVEL) *
                                 (y >> SE3TRACKING_MIN_LEVEL)] ||
            destAbsGrad < MIN_ABS_GRAD_DECREASE) {
          runningStats.num_prop_removed_colorDiff++;
          continue;
        }
      } else {
        float sourceColor =
            activeKFImageData[x + y * Conf().slamImageSize.width];
        float destColor = getInterpolatedElement(newKFImageData, u_new, v_new,
                                                 Conf().slamImageSize.width);

        float residual = destColor - sourceColor;

        if (residual * residual /
                    (MAX_DIFF_CONSTANT +
                     MAX_DIFF_GRAD_MULT * destAbsGrad * destAbsGrad) >
                1.0f ||
            destAbsGrad < MIN_ABS_GRAD_DECREASE) {
          runningStats.num_prop_removed_colorDiff++;
          continue;
        }
      }

      DepthMapPixelHypothesis *targetBest = currentDepthMap + newIDX;

      // large idepth = point is near = large increase in variance.
      // small idepth = point is far = small increase in variance.
      float idepth_ratio_4 = new_idepth / source->idepth_smoothed;
      idepth_ratio_4 *= idepth_ratio_4;
      idepth_ratio_4 *= idepth_ratio_4;

      float new_var = idepth_ratio_4 * source->idepth_var;

      // check for occlusion
      if (targetBest->isValid) {
        // if they occlude one another, one gets removed.
        float diff = targetBest->idepth - new_idepth;
        if (DIFF_FAC_PROP_MERGE * diff * diff >
            new_var + targetBest->idepth_var) {
          if (new_idepth < targetBest->idepth) {
            runningStats.num_prop_occluded++;
            continue;
          } else {
            runningStats.num_prop_occluded++;
            targetBest->isValid = false;
          }
        }
      }

      if (!targetBest->isValid) {
        runningStats.num_prop_created++;

        *targetBest = DepthMapPixelHypothesis(
            new_idepth, new_var, source->validity_counter, Conf().debugDisplay);

      } else {
        runningStats.num_prop_merged++;

        // merge idepth ekf-style
        float w = new_var / (targetBest->idepth_var + new_var);
        float merged_new_idepth =
            w * targetBest->idepth + (1.0f - w) * new_idepth;

        // merge validity
        int merged_validity =
            source->validity_counter + targetBest->validity_counter;
        if (merged_validity >
            VALIDITY_COUNTER_MAX + (VALIDITY_COUNTER_MAX_VARIABLE))
          merged_validity =
              VALIDITY_COUNTER_MAX + (VALIDITY_COUNTER_MAX_VARIABLE);

        *targetBest = DepthMapPixelHypothesis(
            merged_new_idepth,
            1.0f / (1.0f / targetBest->idepth_var + 1.0f / new_var),
            merged_validity, Conf().debugDisplay);
      }
    }

  // swap!
  //	std::swap(currentDepthMap, otherDepthMap);

  LOGF_IF(INFO, Conf().print.propagationStatistics,
          "PROPAGATE: %d invalid, %d: %d drop (%d oob, %d color); %d created; "
          "%d merged; %d occluded. %d col-dec, %d grad-dec.",
          runningStats.num_prop_source_invalid, runningStats.num_prop_attempts,
          runningStats.num_prop_removed_validity +
              runningStats.num_prop_removed_out_of_bounds +
              runningStats.num_prop_removed_colorDiff,
          runningStats.num_prop_removed_out_of_bounds,
          runningStats.num_prop_removed_colorDiff,
          runningStats.num_prop_created, runningStats.num_prop_merged,
          runningStats.num_prop_occluded, runningStats.num_prop_color_decreased,
          runningStats.num_prop_grad_decreased);
} // namespace lsd_slam

void DepthMap::regularizeDepthMapFillHoles() {

  buildRegIntegralBuffer();

  runningStats.num_reg_created = 0;

  memcpy(otherDepthMap, currentDepthMap,
         Conf().slamImageSize.area() * sizeof(DepthMapPixelHypothesis));
  threadReducer.reduce(
      boost::bind(&DepthMap::regularizeDepthMapFillHolesRow, this, _1, _2, _3),
      3, Conf().slamImageSize.height - 2, 10);
  LOGF_IF(INFO, Conf().print.fillHolesStatistics,
          "FillHoles (discreteDepth): %d created\n",
          runningStats.num_reg_created);
}

void DepthMap::regularizeDepthMapFillHolesRow(int yMin, int yMax,
                                              RunningStats *stats) {
  // =========== regularize fill holes
  const float *keyFrameMaxGradBuf = frame()->maxGradients(0);

  int width = Conf().slamImageSize.width;

  for (int y = yMin; y < yMax; y++) {
    for (int x = 3; x < Conf().slamImageSize.width - 2; x++) {
      int idx = x + y * width;
      DepthMapPixelHypothesis *dest = otherDepthMap + idx;
      if (dest->isValid)
        continue;
      if (keyFrameMaxGradBuf[idx] < MIN_ABS_GRAD_DECREASE)
        continue;

      int *io = validityIntegralBuffer + idx;
      int val = io[2 + 2 * width] - io[2 - 3 * width] - io[-3 + 2 * width] +
                io[-3 - 3 * width];

      if ((dest->blacklisted >= MIN_BLACKLIST &&
           val > VAL_SUM_MIN_FOR_CREATE) ||
          val > VAL_SUM_MIN_FOR_UNBLACKLIST) {
        float sumIdepthObs = 0, sumIVarObs = 0;
        int num = 0;

        DepthMapPixelHypothesis *s1max =
            otherDepthMap + (x - 2) + (y + 3) * width;
        for (DepthMapPixelHypothesis *s1 =
                 otherDepthMap + (x - 2) + (y - 2) * width;
             s1 < s1max; s1 += width)
          for (DepthMapPixelHypothesis *source = s1; source < s1 + 5;
               source++) {
            if (!source->isValid)
              continue;

            sumIdepthObs += source->idepth / source->idepth_var;
            sumIVarObs += 1.0f / source->idepth_var;
            num++;
          }

        float idepthObs = sumIdepthObs / sumIVarObs;
        idepthObs = UNZERO(idepthObs);

        currentDepthMap[idx] = DepthMapPixelHypothesis(
            idepthObs, VAR_RANDOM_INIT_INITIAL, 0, Conf().debugDisplay);

        stats->num_reg_created++;
      }
    }
  }
}

void DepthMap::buildRegIntegralBuffer() {
  threadReducer.reduce(
      boost::bind(&DepthMap::buildRegIntegralBufferRow1, this, _1, _2, _3), 0,
      Conf().slamImageSize.height);

  int *validityIntegralBufferPT = validityIntegralBuffer;
  int *validityIntegralBufferPT_T =
      validityIntegralBuffer + Conf().slamImageSize.width;

  int wh = Conf().slamImageSize.area();
  for (int idx = Conf().slamImageSize.width; idx < wh; idx++)
    *(validityIntegralBufferPT_T++) += *(validityIntegralBufferPT++);
}

void DepthMap::buildRegIntegralBufferRow1(int yMin, int yMax,
                                          RunningStats *stats) {
  // ============ build inegral buffers
  int *validityIntegralBufferPT =
      validityIntegralBuffer + yMin * Conf().slamImageSize.width;
  DepthMapPixelHypothesis *ptSrc =
      currentDepthMap + yMin * Conf().slamImageSize.width;
  for (int y = yMin; y < yMax; y++) {
    int validityIntegralBufferSUM = 0;

    for (int x = 0; x < Conf().slamImageSize.width; x++) {
      if (ptSrc->isValid)
        validityIntegralBufferSUM += ptSrc->validity_counter;

      *(validityIntegralBufferPT++) = validityIntegralBufferSUM;
      ptSrc++;
    }
  }
}

void DepthMap::regularizeDepthMap(bool removeOcclusions, int validityTH) {
  runningStats.num_reg_smeared = 0;
  runningStats.num_reg_total = 0;
  runningStats.num_reg_deleted_secondary = 0;
  runningStats.num_reg_deleted_occluded = 0;
  runningStats.num_reg_blacklisted = 0;
  runningStats.num_reg_setBlacklisted = 0;

  memcpy(otherDepthMap, currentDepthMap,
         Conf().slamImageSize.area() * sizeof(DepthMapPixelHypothesis));

  if (removeOcclusions)
    threadReducer.reduce(boost::bind(&DepthMap::regularizeDepthMapRow<true>,
                                     this, validityTH, _1, _2, _3),
                         2, Conf().slamImageSize.height - 2, 10);
  else
    threadReducer.reduce(boost::bind(&DepthMap::regularizeDepthMapRow<false>,
                                     this, validityTH, _1, _2, _3),
                         2, Conf().slamImageSize.height - 2, 10);

  LOGF_IF(INFO, Conf().print.regularizeStatistics,
          "REGULARIZE (%d): %d smeared; %d blacklisted /%d new); %d deleted; "
          "%d occluded; %d filled\n",
          frame()->id(), runningStats.num_reg_smeared,
          runningStats.num_reg_blacklisted, runningStats.num_reg_setBlacklisted,
          runningStats.num_reg_deleted_secondary,
          runningStats.num_reg_deleted_occluded, runningStats.num_reg_created);
}

template <bool removeOcclusions>
void DepthMap::regularizeDepthMapRow(int validityTH, int yMin, int yMax,
                                     RunningStats *stats) {
  const int regularize_radius = 2;

  const float regDistVar = REG_DIST_VAR;

  for (int y = yMin; y < yMax; y++) {
    for (int x = regularize_radius;
         x < (Conf().slamImageSize.width - regularize_radius); x++) {
      DepthMapPixelHypothesis *dest =
          currentDepthMap + x + y * Conf().slamImageSize.width;
      DepthMapPixelHypothesis *destRead =
          otherDepthMap + x + y * Conf().slamImageSize.width;

      // if isValid need to do better examination and then update.

      if (destRead->blacklisted < MIN_BLACKLIST)
        stats->num_reg_blacklisted++;

      if (!destRead->isValid)
        continue;

      float sum = 0, val_sum = 0, sumIvar = 0; //, min_varObs = 1e20;
      int numOccluding = 0, numNotOccluding = 0;

      for (int dx = -regularize_radius; dx <= regularize_radius; dx++)
        for (int dy = -regularize_radius; dy <= regularize_radius; dy++) {
          DepthMapPixelHypothesis *source =
              destRead + dx + dy * Conf().slamImageSize.width;

          if (!source->isValid)
            continue;
          //					stats->num_reg_total++;

          float diff = source->idepth - destRead->idepth;
          if (DIFF_FAC_SMOOTHING * diff * diff >
              source->idepth_var + destRead->idepth_var) {
            if (removeOcclusions) {
              if (source->idepth > destRead->idepth)
                numOccluding++;
            }
            continue;
          }

          val_sum += source->validity_counter;

          if (removeOcclusions)
            numNotOccluding++;

          float distFac = (float)(dx * dx + dy * dy) * regDistVar;
          float ivar = 1.0f / (source->idepth_var + distFac);

          sum += source->idepth * ivar;
          sumIvar += ivar;
        }

      if (val_sum < validityTH) {
        dest->isValid = false;
        stats->num_reg_deleted_secondary++;
        dest->blacklisted--;

        stats->num_reg_setBlacklisted++;
        continue;
      }

      if (removeOcclusions) {
        if (numOccluding > numNotOccluding) {
          dest->isValid = false;
          stats->num_reg_deleted_occluded++;

          continue;
        }
      }

      sum = sum / sumIvar;
      sum = UNZERO(sum);

      // update!
      dest->idepth_smoothed = sum;
      dest->idepth_var_smoothed = 1.0f / sumIvar;

      stats->num_reg_smeared++;
    }
  }
}
template void DepthMap::regularizeDepthMapRow<true>(int validityTH, int yMin,
                                                    int yMax,
                                                    RunningStats *stats);
template void DepthMap::regularizeDepthMapRow<false>(int validityTH, int yMin,
                                                     int yMax,
                                                     RunningStats *stats);

void DepthMap::logPerformanceData() {
  LOGF_IF(DEBUG, Conf().print.mappingTiming,
          "Upd %3.1fms (%.1fHz); Create %3.1fms (%.1fHz); Final %3.1fms "
          "(%.1fHz) // Obs %3.1fms (%.1fHz); Reg %3.1fms (%.1fHz); Prop "
          "%3.1fms (%.1fHz); Fill %3.1fms (%.1fHz); Set %3.1fms (%.1fHz)\n",
          _perf.update.ms(), _perf.update.rate(), _perf.create.ms(),
          _perf.create.rate(), _perf.finalize.ms(), _perf.finalize.rate(),
          _perf.observe.ms(), _perf.observe.rate(), _perf.regularize.ms(),
          _perf.regularize.rate(), _perf.propagate.ms(), _perf.propagate.rate(),
          _perf.fillHoles.ms(), _perf.fillHoles.rate(), _perf.setDepth.ms(),
          _perf.setDepth.rate());
}

// int DepthMap::debugPlotDepthMap()
// {
// 	if(activeKeyFrame == 0) return 1;
//
// 	cv::Mat keyFrameImage(activeKeyFrame->height(), activeKeyFrame->width(),
// CV_32F, const_cast<float*>(activeKeyFrameImageData())); 	cv::Mat
// keyFrameGray( keyFrameImage.size(), CV_8UC1 );
// keyFrameImage.convertTo(keyFrameGray, CV_8UC1);
// cv::cvtColor(keyFrameGray, debugImageDepth, CV_GRAY2RGB);
//
// 	// debug plot & publish sparse version?
// 	int refID = referenceFrameByID_offset;
//
//
// 	for(int y=0;y<(Conf().slamImageSize.height);y++)
// 		for(int x=0;x<(Conf().slamImageSize.width);x++)
// 		{
// 			int idx = x + y*Conf().slamImageSize.width;
//
// 			if(currentDepthMap[idx].blacklisted < MIN_BLACKLIST &&
// Conf().debugDisplay == 2)
// debugImageDepth.at<cv::Vec3b>(y,x) = cv::Vec3b(0,0,255);
//
// 			if(!currentDepthMap[idx].isValid) continue;
//
// 			cv::Vec3b color =
// currentDepthMap[idx].getVisualizationColor(refID);
// 			debugImageDepth.at<cv::Vec3b>(y,x) = color;
// 		}
//
//
// 	return 1;
// }

void DepthMap::plotDepthMap(const char *buf1, const char *buf2) {
  // TODO:  0 arg was referenceFrameByID_offset, but that doesn't exist
  // anymore
  _debugImages.debugPlotDepthMap(frame(), currentDepthMap, 0, buf1, buf2);
}

// find pixel in image (do stereo along epipolar line).
// mat: NEW image
// KinvP: point in OLD image (Kinv * (u_old, v_old, 1)), projected
// trafo: x_old = trafo * x_new; (from new to old image)
// realVal: descriptor in OLD image.
// returns: result_idepth : point depth in new camera's coordinate system
// returns: result_u/v : point's coordinates in new camera's coordinate
// system returns: idepth_var: (approximated) measurement variance of
// inverse depth of result_point_NEW returns error if sucessful; -1 if out
// of bounds, -2 if not found.
inline float DepthMap::doLineStereo(
    const float u, const float v, const float epxn, const float epyn,
    const float min_idepth, const float prior_idepth, float max_idepth,
    const Frame *const referenceFrame, const float *referenceFrameImage,
    float &result_idepth, float &result_var, float &result_eplLength,
    RunningStats *stats) {
  stats->num_stereo_calls++;

  int width = Conf().slamImageSize.width, height = Conf().slamImageSize.height;

  // calculate epipolar line start and end point in old image
  // TODO:  Converted from Conf().camrea to referenceFrame.  Not actually
  // sure that's the correct frame's K to be using (in the case where K
  // isn't constant)
  Eigen::Vector3f KinvP =
      Eigen::Vector3f(referenceFrame->fxi() * u + referenceFrame->cxi(),
                      referenceFrame->fyi() * v + referenceFrame->cyi(), 1.0f);
  Eigen::Vector3f pInf = referenceFrame->K_otherToThis_R * KinvP;
  Eigen::Vector3f pReal = pInf / prior_idepth + referenceFrame->K_otherToThis_t;

  float rescaleFactor = pReal[2] * prior_idepth;

  float firstX = u - 2 * epxn * rescaleFactor;
  float firstY = v - 2 * epyn * rescaleFactor;
  float lastX = u + 2 * epxn * rescaleFactor;
  float lastY = v + 2 * epyn * rescaleFactor;
  // width - 2 and height - 2 comes from the one-sided gradient calculation
  // at the bottom
  if (firstX <= 0 || firstX >= width - 2 || firstY <= 0 ||
      firstY >= height - 2 || lastX <= 0 || lastX >= width - 2 || lastY <= 0 ||
      lastY >= height - 2) {
    return -1;
  }

  if (!(rescaleFactor > 0.7f && rescaleFactor < 1.4f)) {
    stats->num_stereo_rescale_oob++;
    return -1;
  }

  // calculate values to search for
  const float *currentKeyFrameImageData = activeKeyFrameImageData();
  float realVal_p1 =
      getInterpolatedElement(currentKeyFrameImageData, u + epxn * rescaleFactor,
                             v + epyn * rescaleFactor, width);
  float realVal_m1 =
      getInterpolatedElement(currentKeyFrameImageData, u - epxn * rescaleFactor,
                             v - epyn * rescaleFactor, width);
  float realVal = getInterpolatedElement(currentKeyFrameImageData, u, v, width);
  float realVal_m2 = getInterpolatedElement(
      currentKeyFrameImageData, u - 2 * epxn * rescaleFactor,
      v - 2 * epyn * rescaleFactor, width);
  float realVal_p2 = getInterpolatedElement(
      currentKeyFrameImageData, u + 2 * epxn * rescaleFactor,
      v + 2 * epyn * rescaleFactor, width);

  //	if(referenceFrame->K_otherToThis_t[2] * max_idepth + pInf[2] <
  // 0.01)

  Eigen::Vector3f pClose = pInf + referenceFrame->K_otherToThis_t * max_idepth;
  // if the assumed close-point lies behind the
  // image, have to change that.
  if (pClose[2] < 0.001f) {
    max_idepth = (0.001f - pInf[2]) / referenceFrame->K_otherToThis_t[2];
    pClose = pInf + referenceFrame->K_otherToThis_t * max_idepth;
  }
  pClose =
      pClose / pClose[2]; // pos in new image of point (xy), assuming max_idepth

  Eigen::Vector3f pFar = pInf + referenceFrame->K_otherToThis_t * min_idepth;
  // if the assumed far-point lies behind the image or closter than the
  // near-point, we moved past the Point it and should stop.
  if (pFar[2] < 0.001f || max_idepth < min_idepth) {
    stats->num_stereo_inf_oob++;
    return -1;
  }
  pFar = pFar / pFar[2]; // pos in new image of point (xy), assuming min_idepth

  // check for nan due to eg division by zero.
  if (isnanf((float)(pFar[0] + pClose[0])))
    return -4;

  // calculate increments in which we will step through the epipolar line.
  // they are sampleDist (or half sample dist) long
  float incx = pClose[0] - pFar[0];
  float incy = pClose[1] - pFar[1];
  float eplLength = sqrt(incx * incx + incy * incy);
  if (!(eplLength > 0) || std::isinf(eplLength))
    return -4;

  if (eplLength > MAX_EPL_LENGTH_CROP) {
    pClose[0] = pFar[0] + incx * MAX_EPL_LENGTH_CROP / eplLength;
    pClose[1] = pFar[1] + incy * MAX_EPL_LENGTH_CROP / eplLength;
  }

  incx *= GRADIENT_SAMPLE_DIST / eplLength;
  incy *= GRADIENT_SAMPLE_DIST / eplLength;

  // extend one sample_dist to left & right.
  pFar[0] -= incx;
  pFar[1] -= incy;
  pClose[0] += incx;
  pClose[1] += incy;

  // make epl long enough (pad a little bit).
  if (eplLength < MIN_EPL_LENGTH_CROP) {
    float pad = (MIN_EPL_LENGTH_CROP - (eplLength)) / 2.0f;
    pFar[0] -= incx * pad;
    pFar[1] -= incy * pad;

    pClose[0] += incx * pad;
    pClose[1] += incy * pad;
  }

  // if inf point is outside of image: skip pixel.
  if (pFar[0] <= SAMPLE_POINT_TO_BORDER ||
      pFar[0] >= width - SAMPLE_POINT_TO_BORDER ||
      pFar[1] <= SAMPLE_POINT_TO_BORDER ||
      pFar[1] >= height - SAMPLE_POINT_TO_BORDER) {
    stats->num_stereo_inf_oob++;
    return -1;
  }

  // if near point is outside: move inside, and test length again.
  if (pClose[0] <= SAMPLE_POINT_TO_BORDER ||
      pClose[0] >= width - SAMPLE_POINT_TO_BORDER ||
      pClose[1] <= SAMPLE_POINT_TO_BORDER ||
      pClose[1] >= height - SAMPLE_POINT_TO_BORDER) {
    if (pClose[0] <= SAMPLE_POINT_TO_BORDER) {
      float toAdd = (SAMPLE_POINT_TO_BORDER - pClose[0]) / incx;
      pClose[0] += toAdd * incx;
      pClose[1] += toAdd * incy;
    } else if (pClose[0] >= width - SAMPLE_POINT_TO_BORDER) {
      float toAdd = (width - SAMPLE_POINT_TO_BORDER - pClose[0]) / incx;
      pClose[0] += toAdd * incx;
      pClose[1] += toAdd * incy;
    }

    if (pClose[1] <= SAMPLE_POINT_TO_BORDER) {
      float toAdd = (SAMPLE_POINT_TO_BORDER - pClose[1]) / incy;
      pClose[0] += toAdd * incx;
      pClose[1] += toAdd * incy;
    } else if (pClose[1] >= height - SAMPLE_POINT_TO_BORDER) {
      float toAdd = (height - SAMPLE_POINT_TO_BORDER - pClose[1]) / incy;
      pClose[0] += toAdd * incx;
      pClose[1] += toAdd * incy;
    }

    // get new epl length
    float fincx = pClose[0] - pFar[0];
    float fincy = pClose[1] - pFar[1];
    float newEplLength = sqrt(fincx * fincx + fincy * fincy);

    // test again
    if (pClose[0] <= SAMPLE_POINT_TO_BORDER ||
        pClose[0] >= width - SAMPLE_POINT_TO_BORDER ||
        pClose[1] <= SAMPLE_POINT_TO_BORDER ||
        pClose[1] >= height - SAMPLE_POINT_TO_BORDER || newEplLength < 8.0f) {
      stats->num_stereo_near_oob++;
      return -1;
    }
  }

  // from here on:
  // - pInf: search start-point
  // - p0: search end-point
  // - incx, incy: search steps in pixel
  // - eplLength, min_idepth, max_idepth: determines search-resolution, i.e.
  // the result's variance.

  float cpx = pFar[0];
  float cpy = pFar[1];

  float val_cp_m2 = getInterpolatedElement(
      referenceFrameImage, cpx - 2.0f * incx, cpy - 2.0f * incy, width);
  float val_cp_m1 = getInterpolatedElement(referenceFrameImage, cpx - incx,
                                           cpy - incy, width);
  float val_cp = getInterpolatedElement(referenceFrameImage, cpx, cpy, width);
  float val_cp_p1 = getInterpolatedElement(referenceFrameImage, cpx + incx,
                                           cpy + incy, width);
  float val_cp_p2;

  /*
   * Subsequent exact minimum is found the following way:
   * - assuming lin. interpolation, the gradient of Error at p1 (towards p2)
   * is given by dE1 = -2sum(e1*e1 - e1*e2) where e1 and e2 are summed over,
   * and are the residuals (not squared).
   *
   * - the gradient at p2 (coming from p1) is given by
   * 	 dE2 = +2sum(e2*e2 - e1*e2)
   *
   * - linear interpolation => gradient changes linearely; zero-crossing is
   * hence given by p1 + d*(p2-p1) with d = -dE1 / (-dE1 + dE2).
   *
   *
   *
   * => I for later exact min calculation, I need
   * sum(e_i*e_i),sum(e_{i-1}*e_{i-1}),sum(e_{i+1}*e_{i+1}) and sum(e_i *
   * e_{i-1}) and sum(e_i * e_{i+1}), where i is the respective winning
   * index.
   */

  // walk in equally sized steps, starting at depth=infinity.
  int loopCounter = 0;
  float best_match_x = -1;
  float best_match_y = -1;
  float best_match_err = 1e50;
  float second_best_match_err = 1e50;

  // best pre and post errors.
  float best_match_errPre = NAN, best_match_errPost = NAN,
        best_match_DiffErrPre = NAN, best_match_DiffErrPost = NAN;
  bool bestWasLastLoop = false;

  float eeLast = -1; // final error of last comp.

  // alternating intermediate vars
  float e1A = NAN, e1B = NAN, e2A = NAN, e2B = NAN, e3A = NAN, e3B = NAN,
        e4A = NAN, e4B = NAN, e5A = NAN, e5B = NAN;

  int loopCBest = -1, loopCSecond = -1;
  while (((incx < 0) == (cpx > pClose[0]) && (incy < 0) == (cpy > pClose[1])) ||
         loopCounter == 0) {
    // interpolate one new point
    val_cp_p2 = getInterpolatedElement(referenceFrameImage, cpx + 2 * incx,
                                       cpy + 2 * incy, width);

    // hacky but fast way to get error and differential error: switch
    // buffer variables for last loop.
    float ee = 0;
    if (loopCounter % 2 == 0) {
      // calc error and accumulate sums.
      e1A = val_cp_p2 - realVal_p2;
      ee += e1A * e1A;
      e2A = val_cp_p1 - realVal_p1;
      ee += e2A * e2A;
      e3A = val_cp - realVal;
      ee += e3A * e3A;
      e4A = val_cp_m1 - realVal_m1;
      ee += e4A * e4A;
      e5A = val_cp_m2 - realVal_m2;
      ee += e5A * e5A;
    } else {
      // calc error and accumulate sums.
      e1B = val_cp_p2 - realVal_p2;
      ee += e1B * e1B;
      e2B = val_cp_p1 - realVal_p1;
      ee += e2B * e2B;
      e3B = val_cp - realVal;
      ee += e3B * e3B;
      e4B = val_cp_m1 - realVal_m1;
      ee += e4B * e4B;
      e5B = val_cp_m2 - realVal_m2;
      ee += e5B * e5B;
    }

    // do I have a new winner??
    // if so: set.
    if (ee < best_match_err) {
      // put to second-best
      second_best_match_err = best_match_err;
      loopCSecond = loopCBest;

      // set best.
      best_match_err = ee;
      loopCBest = loopCounter;

      best_match_errPre = eeLast;
      best_match_DiffErrPre =
          e1A * e1B + e2A * e2B + e3A * e3B + e4A * e4B + e5A * e5B;
      best_match_errPost = -1;
      best_match_DiffErrPost = -1;

      best_match_x = cpx;
      best_match_y = cpy;
      bestWasLastLoop = true;
    }
    // otherwise: the last might be the current winner, in which case i
    // have to save these values.
    else {
      if (bestWasLastLoop) {
        best_match_errPost = ee;
        best_match_DiffErrPost =
            e1A * e1B + e2A * e2B + e3A * e3B + e4A * e4B + e5A * e5B;
        bestWasLastLoop = false;
      }

      // collect second-best:
      // just take the best of all that are NOT equal to current best.
      if (ee < second_best_match_err) {
        second_best_match_err = ee;
        loopCSecond = loopCounter;
      }
    }

    // shift everything one further.
    eeLast = ee;
    val_cp_m2 = val_cp_m1;
    val_cp_m1 = val_cp;
    val_cp = val_cp_p1;
    val_cp_p1 = val_cp_p2;

    stats->num_stereo_comparisons++;

    cpx += incx;
    cpy += incy;

    loopCounter++;
  }

  // if error too big, will return -3, otherwise -2.
  if (best_match_err > 4.0f * (float)MAX_ERROR_STEREO) {
    stats->num_stereo_invalid_bigErr++;
    return -3;
  }

  // check if clear enough winner
  if (abs(loopCBest - loopCSecond) > 1.0f &&
      MIN_DISTANCE_ERROR_STEREO * best_match_err > second_best_match_err) {
    stats->num_stereo_invalid_unclear_winner++;
    return -2;
  }

  bool didSubpixel = false;
  if (useSubpixelStereo) {
    // ================== compute exact match =========================
    // compute gradients (they are actually only half the real gradient)
    float gradPre_pre = -(best_match_errPre - best_match_DiffErrPre);
    float gradPre_this = +(best_match_err - best_match_DiffErrPre);
    float gradPost_this = -(best_match_err - best_match_DiffErrPost);
    float gradPost_post = +(best_match_errPost - best_match_DiffErrPost);

    // final decisions here.
    bool interpPost = false;
    bool interpPre = false;

    // if one is oob: return false.
    if (best_match_errPre < 0 || best_match_errPost < 0) {
      stats->num_stereo_invalid_atEnd++;
    } else if ((gradPost_this < 0) ^ (gradPre_this < 0)) {
      // - if zero-crossing occurs exactly in between (gradient
      // Inconsistent),

      // return exact pos, if both central gradients are small compared to
      // their counterpart.
      if ((gradPost_this * gradPost_this >
               0.1f * 0.1f * gradPost_post * gradPost_post ||
           gradPre_this * gradPre_this >
               0.1f * 0.1f * gradPre_pre * gradPre_pre))
        stats->num_stereo_invalid_inexistantCrossing++;
    } else if ((gradPre_pre < 0) ^ (gradPre_this < 0)) {
      // if pre has zero-crossing

      // if post has zero-crossing
      if ((gradPost_post < 0) ^ (gradPost_this < 0)) {
        stats->num_stereo_invalid_twoCrossing++;
      } else
        interpPre = true;
    } else if ((gradPost_post < 0) ^ (gradPost_this < 0)) {
      // if post has zero-crossing
      interpPost = true;
    } else {
      // if none has zero-crossing
      stats->num_stereo_invalid_noCrossing++;
    }

    // DO interpolation!
    // minimum occurs at zero-crossing of gradient, which is a straight
    // line
    // => easy to compute. the error at that point is also computed by
    // just integrating.
    if (interpPre) {
      float d = gradPre_this / (gradPre_this - gradPre_pre);
      best_match_x -= d * incx;
      best_match_y -= d * incy;
      best_match_err = best_match_err - 2 * d * gradPre_this -
                       (gradPre_pre - gradPre_this) * d * d;
      if (enablePrintDebugInfo)
        stats->num_stereo_interpPre++;
      didSubpixel = true;

    } else if (interpPost) {
      float d = gradPost_this / (gradPost_this - gradPost_post);
      best_match_x += d * incx;
      best_match_y += d * incy;
      best_match_err = best_match_err + 2 * d * gradPost_this +
                       (gradPost_post - gradPost_this) * d * d;
      stats->num_stereo_interpPost++;
      didSubpixel = true;
    } else {
      if (enablePrintDebugInfo)
        stats->num_stereo_interpNone++;
    }
  }

  // sampleDist is the distance in pixel at which the realVal's were
  // sampled
  float sampleDist = GRADIENT_SAMPLE_DIST * rescaleFactor;

  float gradAlongLine = 0;
  float tmp = realVal_p2 - realVal_p1;
  gradAlongLine += tmp * tmp;
  tmp = realVal_p1 - realVal;
  gradAlongLine += tmp * tmp;
  tmp = realVal - realVal_m1;
  gradAlongLine += tmp * tmp;
  tmp = realVal_m1 - realVal_m2;
  gradAlongLine += tmp * tmp;

  gradAlongLine /= sampleDist * sampleDist;

  // check if interpolated error is OK. use evil hack to allow more error
  // if there is a lot of gradient.
  if (best_match_err > (float)MAX_ERROR_STEREO + sqrtf(gradAlongLine) * 20) {
    stats->num_stereo_invalid_bigErr++;
    return -3;
  }

  // ================= calc depth (in KF) ====================
  // * KinvP = Kinv * (x,y,1); where x,y are pixel coordinates of point we
  // search for, in the KF.
  // * best_match_x = x-coordinate of found correspondence in the
  // reference frame.

  const float fxi = referenceFrame->fxi(), fyi = referenceFrame->fyi(),
              cxi = referenceFrame->cxi(), cyi = referenceFrame->cyi();

  float idnew_best_match; // depth in the new image
  float alpha; // d(idnew_best_match) / d(disparity in pixel) == conputed
               // inverse depth derived by the pixel-disparity.
  if (incx * incx > incy * incy) {
    float oldX = fxi * best_match_x + cxi;
    float nominator = (oldX * referenceFrame->otherToThis_t[2] -
                       referenceFrame->otherToThis_t[0]);
    float dot0 = KinvP.dot(referenceFrame->otherToThis_R_row0);
    float dot2 = KinvP.dot(referenceFrame->otherToThis_R_row2);

    idnew_best_match = (dot0 - oldX * dot2) / nominator;
    alpha = incx * fxi *
            (dot0 * referenceFrame->otherToThis_t[2] -
             dot2 * referenceFrame->otherToThis_t[0]) /
            (nominator * nominator);

  } else {
    float oldY = fyi * best_match_y + cyi;

    float nominator = (oldY * referenceFrame->otherToThis_t[2] -
                       referenceFrame->otherToThis_t[1]);
    float dot1 = KinvP.dot(referenceFrame->otherToThis_R_row1);
    float dot2 = KinvP.dot(referenceFrame->otherToThis_R_row2);

    idnew_best_match = (dot1 - oldY * dot2) / nominator;
    alpha = incy * fyi *
            (dot1 * referenceFrame->otherToThis_t[2] -
             dot2 * referenceFrame->otherToThis_t[1]) /
            (nominator * nominator);
  }

  if (idnew_best_match < 0) {
    stats->num_stereo_negative++;
    if (!allowNegativeIdepths)
      return -2;
  }

  stats->num_stereo_successfull++;

  // ================= calc var (in NEW image) ====================

  // calculate error from photometric noise
  float photoDispError =
      4.0f * cameraPixelNoise2 / (gradAlongLine + DIVISION_EPS);

  float trackingErrorFac =
      0.25f * (1.0f + referenceFrame->initialTrackedResidual);

  // calculate error from geometric noise (wrong camera pose /
  // calibration)
  Eigen::Vector2f gradsInterp =
      getInterpolatedElement42(frame()->gradients(0), u, v, width);
  float geoDispError =
      (gradsInterp[0] * epxn + gradsInterp[1] * epyn) + DIVISION_EPS;
  geoDispError =
      trackingErrorFac * trackingErrorFac *
      (gradsInterp[0] * gradsInterp[0] + gradsInterp[1] * gradsInterp[1]) /
      (geoDispError * geoDispError);

  // geoDispError *= (0.5 + 0.5 *result_idepth) * (0.5 + 0.5
  // *result_idepth);

  // final error consists of a small constant part (discretization error),
  // geometric and photometric error.
  result_var = alpha * alpha *
               ((didSubpixel ? 0.05f : 0.5f) * sampleDist * sampleDist +
                geoDispError + photoDispError); // square to make variance

  if (plotStereoImages) {
    if (rand() % 5 == 0) {
      // if(rand()%500 == 0)
      //	printf("geo: %f, photo: %f, alpha: %f\n", sqrt(geoDispError),
      // sqrt(photoDispError), alpha, sqrt(result_var));

      // int idDiff = (keyFrame->pyramidID - referenceFrame->id);
      // cv::Scalar color = cv::Scalar(0,0, 2*idDiff);// bw

      // cv::Scalar color = cv::Scalar(sqrt(result_var)*2000,
      // 255-sqrt(result_var)*2000, 0);// bw

      //			float eplLengthF =
      // std::min((float)MIN_EPL_LENGTH_CROP,(float)eplLength);
      // eplLengthF =
      // std::max((float)MAX_EPL_LENGTH_CROP,(float)eplLengthF);
      //
      //			float pixelDistFound =
      // sqrtf((float)((pReal[0]/pReal[2] -
      // best_match_x)*(pReal[0]/pReal[2] - best_match_x)
      //					+ (pReal[1]/pReal[2] -
      // best_match_y)*(pReal[1]/pReal[2] - best_match_y)));
      //
      float fac = best_match_err /
                  ((float)MAX_ERROR_STEREO + sqrtf(gradAlongLine) * 20);

      cv::Scalar color = cv::Scalar(255 * fac, 255 - 255 * fac, 0); // bw

      /*
      if(rescaleFactor > 1)
              color = cv::Scalar(500*(rescaleFactor-1),0,0);
      else
              color =
      cv::Scalar(0,500*(1-rescaleFactor),500*(1-rescaleFactor));
      */

      _debugImages.addStereoLine(cv::Point2f(pClose[0], pClose[1]),
                                 cv::Point2f(pFar[0], pFar[1]), color);
    }
  }

  result_idepth = idnew_best_match;

  result_eplLength = eplLength;

  return best_match_err;
}

} // namespace lsd_slam
