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

#include "Relocalizer.h"
#include "DataStructures/Frame.h"
#include "DataStructures/KeyFrame.h"
#include "IOWrapper/ImageDisplay.h"
#include "Tracking/SE3Tracker.h"

namespace lsd_slam {

Relocalizer::Relocalizer()
    : isRunning(false), KFForReloc(), nextRelocIDX(0), maxRelocIDX(0),
      continueRunning(false), hasResult(false), resultKF(nullptr),
      resultFrameID(0), resultFrameToKeyframe(SE3()),
      _printRelocalizationInfo(true) {
  for (int i = 0; i < RELOCALIZE_THREADS; i++)
    running[i] = false;
}

Relocalizer::~Relocalizer() { stop(); }

void Relocalizer::stop() {
  continueRunning = false;
  // exMutex.lock();
  // newCurrentFrameSignal.notify_all();
  // exMutex.unlock();
  for (int i = 0; i < RELOCALIZE_THREADS; i++) {
    relocThreads[i].join();
    running[i] = false;
  }
  isRunning = false;

  // KFForReloc.clear();
  // CurrentRelocFrame.reset();
}

void Relocalizer::updateCurrentFrame(std::shared_ptr<Frame> currentFrame) {
  // std::lock_guard<std::mutex> lock(exMutex);

  // if (hasResult)
  //   return;
  //
  this->CurrentRelocFrame = currentFrame;
  // //	int doneLast = KFForReloc.size() - (maxRelocIDX-nextRelocIDX);
  maxRelocIDX = nextRelocIDX + KFForReloc.size();
  // nextRelocIDX = 0;
  // newCurrentFrameSignal.notify_all();

  //	f("tried last on %d. set new current frame %d. trying %d to %d!\n",
  //			doneLast,
  //			currentFrame->id(), nextRelocIDX, maxRelocIDX);

  // Opening the window from deep within the stack was causing issues on OSX.
  // It was a good diagnostic but probably not the best UI choice anyway...
  // if (displayDepthMap)
  // 	Util::displayImage( "DebugWindow DEPTH", cv::Mat(currentFrame->height(),
  // currentFrame->width(), CV_32F, currentFrame->image())*(1/255.0f), false );
  //
  // int pressedKey = Util::waitKey(1);
  // handleKey(pressedKey);
}

void Relocalizer::start(std::vector<KeyFrame::SharedPtr> &allKeyframesList) {
  LOG(DEBUG) << "Starting ReLoc";
  // make KFForReloc List
  KFForReloc.clear();
  for (unsigned int k = 0; k < allKeyframesList.size(); k++) {
    // insert
    // const int ridx = rand() % (KFForReloc.size());
    // auto itr = KFForReloc.begin();
    // for (int i = 0; i < ridx; ++i) {
    //   ++itr;
    // }
    // std::cout << itr << std::endl;
    // KFForReloc.insert(itr, allKeyframesList[k]);
    LOG(DEBUG) << "Key frame pose. Id: " << allKeyframesList[k]->id()
               << " Pose: "
               << allKeyframesList[k]->pose()->getCamToWorld().matrix3x4();
    KFForReloc.push_back(allKeyframesList[k]);
  }
  nextRelocIDX = 0;
  maxRelocIDX = KFForReloc.size();

  hasResult = false;
  continueRunning = true;
  isRunning = true;

  // start threads
  for (int i = 0; i < RELOCALIZE_THREADS; i++) {
    relocThreads[i] = boost::thread(&Relocalizer::threadLoop, this, i);
    running[i] = true;
  }
}

bool Relocalizer::waitResult(int milliseconds) {
  // std::lock_guard<std::mutex> lock(exMutex);
  LOG(WARNING) << "has result " << hasResult;
  // if (hasResult)
  // return true;

  // !!TODO.  Change to C++11 condition variable
  // resultReadySignal.timed_wait(lock,
  // boost::posix_time::milliseconds(milliseconds));
  return hasResult;
}

RelocalizerResult Relocalizer::getResult(
    void) // Frame* &out_keyframe, std::shared_ptr<Frame> &frame, int
          // &out_successfulFrameID, SE3 &out_frameToKeyframe)
{
  std::lock_guard<std::mutex> lock(exMutex);
  if (hasResult) {
    return RelocalizerResult(resultKF, resultRelocFrame, resultFrameID,
                             resultFrameToKeyframe);
  } else {
    return RelocalizerResult();
  }
}

void Relocalizer::threadLoop(int idx) {
  if (!multiThreading && idx != 0)
    return;

  SE3Tracker *tracker = new SE3Tracker(Conf().slamImageSize);

  //	std::lock_guard<std::mutex> lock(exMutex);
  exMutex.lock();
  while (continueRunning) {
    // LOG(DEBUG) << "isRunning " << isRunning;
    // if got something: do it (unlock in the meantime)

    if (nextRelocIDX < maxRelocIDX) {

      KeyFrame::SharedPtr todo(KFForReloc[nextRelocIDX % KFForReloc.size()]);
      nextRelocIDX++;
      // if (todo->neighbors.size() <= 2)
      //   continue;

      std::shared_ptr<Frame> myRelocFrame = CurrentRelocFrame;

      exMutex.unlock();

      // initial Alignment
      SE3 todoToFrame =
          tracker->trackFrameOnPermaref(todo, myRelocFrame, SE3());

      // try neighbours
      float todoGoodVal = tracker->pointUsage * tracker->lastGoodCount() /
                          (tracker->lastGoodCount() + tracker->lastBadCount());
      // LOG(WARNING) << "todoGoodVal " << todoGoodVal << " " <<
      // relocalizationTH;
      // LOG(DEBUG) << "Relocalization thread";
      if (todoGoodVal > relocalizationTH) {
        int numGoodNeighbours = 0;
        int numBadNeighbours = 0;

        float bestNeightbourGoodVal = todoGoodVal;
        float bestNeighbourUsage = tracker->pointUsage;
        KeyFrame::SharedPtr bestKF(todo);
        SE3 bestKFToFrame = todoToFrame;
        for (auto nkf : todo->neighbors) {
          SE3 nkfToFrame_init =
              se3FromSim3((nkf->frame()->getCamToWorld().inverse() *
                           todo->frame()->getCamToWorld() *
                           sim3FromSE3(todoToFrame.inverse(), 1)))
                  .inverse();
          SE3 nkfToFrame =
              tracker->trackFrameOnPermaref(nkf, myRelocFrame, nkfToFrame_init);

          float goodVal = tracker->pointUsage * tracker->lastGoodCount() /
                          (tracker->lastGoodCount() + tracker->lastBadCount());
          if (goodVal > relocalizationTH * 0.8 &&
              (nkfToFrame * nkfToFrame_init.inverse()).log().norm() < 0.1)
            numGoodNeighbours++;
          else
            numBadNeighbours++;

          if (goodVal > bestNeightbourGoodVal) {
            bestNeightbourGoodVal = goodVal;
            bestKF = nkf;
            bestKFToFrame = nkfToFrame;
            bestNeighbourUsage = tracker->pointUsage;
          }
        }

        // LOG(DEBUG) << "num good numbers " << numGoodNeighbours << "num bad "
        //            << numBadNeighbours;

        if (numGoodNeighbours > numBadNeighbours || numGoodNeighbours >= 5) {
          if (_printRelocalizationInfo)
            LOGF(INFO,
                 "RELOCALIZED! frame %d on %d (bestNeighbour %d): good "
                 "%2.1f%%, usage %2.1f%%, GoodNeighbours %d / %d\n",
                 myRelocFrame->id(), todo->id(), bestKF->id(),
                 100 * bestNeightbourGoodVal, 100 * bestNeighbourUsage,
                 numGoodNeighbours, numGoodNeighbours + numBadNeighbours);

          // set everything to stop!
          continueRunning = false;
          exMutex.lock();
          resultRelocFrame = myRelocFrame;
          resultFrameID = myRelocFrame->id();
          resultKF = bestKF;
          resultFrameToKeyframe = bestKFToFrame.inverse();
          resultReadySignal.notify_all();
          hasResult = true;
          LOG(WARNING) << "has result " << hasResult;
          exMutex.unlock();
        } else {
          if (_printRelocalizationInfo)
            LOGF(INFO,
                 "FAILED RELOCALIZE! frame %d on %d (bestNeighbour %d): good "
                 "%2.1f%%, usage %2.1f%%, GoodNeighbours %d / %d\n",
                 myRelocFrame->id(), todo->id(), bestKF->id(),
                 100 * bestNeightbourGoodVal, 100 * bestNeighbourUsage,
                 numGoodNeighbours, numGoodNeighbours + numBadNeighbours);
        }
      }

      exMutex.lock();
    } else {
      // !!TODO.  Make this wok in C++11
      // newCurrentFrameSignal.wait(lock);
    }
  }
  exMutex.unlock();

  delete tracker;
}

} // namespace lsd_slam
