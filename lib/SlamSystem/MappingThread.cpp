
#include "MappingThread.h"

#include <g3log/g3log.hpp>

#include <boost/thread/shared_lock_guard.hpp>

#include "GlobalMapping/KeyFrameGraph.h"
#include "Tracking/TrackingReference.h"

#include "SlamSystem/ConstraintSearchThread.h"
#include "SlamSystem/OptimizationThread.h"
#include "SlamSystem/TrackingThread.h"

#include "SlamSystem.h"
#include "util/settings.h"

namespace lsd_slam {

using active_object::Active;

MappingThread::MappingThread(SlamSystem &system, bool threaded)
    : relocalizer(), _system(system),
      _thread(threaded ? Active::createActive() : NULL) {
  LOG(INFO) << "Started Mapping thread";
}

MappingThread::~MappingThread() {
  if (_thread)
    delete _thread.release();
  //	unmappedTrackedFrames.clear();
}

void MappingThread::mapSetImpl(const KeyFrame::SharedPtr &kf,
                               const ImageSet::SharedPtr &set) {
  // TODO.  Need code to handle when the queue is growing out of control...

  LOG(INFO) << "Mapping set " << set->id() << " onto KeyFrame " << kf->id();

  kf->updateDepthFrom(set);

  _system.updateDisplayDepthMap();
  //_system.publishCurrentKeyframe();
  _system.publishCurrentFrame();
}

// void MappingThread::createFirstKeyFrame(const Frame::SharedPtr &frame) {
//   LOG(WARNING) << "Making " << frame->id() << " into new keyframe!";
//
//   KeyFrame::SharedPtr kf(KeyFrame::Create(frame));
//   _system.keyFrameGraph()->addKeyFrame(kf);
//   _system.trackingThread()->doUseNewKeyFrame(kf);
// }

void MappingThread::createFirstKeyFrame(const ImageSet::SharedPtr &set) {
  LOG(WARNING) << "Making " << set->refFrame()->id() << " into new keyframe!";
  KeyFrame::SharedPtr kf(KeyFrame::Create(set));
  _system.keyFrameGraph()->addKeyFrame(kf);
  _system.trackingThread()->doUseNewKeyFrame(kf);
}

void MappingThread::updateCurrentSet(const ImageSet::SharedPtr &set) {
  LOG(WARNING) << "Making " << set->refFrame()->id() << " into new keyframe!";
  KeyFrame::SharedPtr kf(KeyFrame::Create(set));
  _system.keyFrameGraph()->addKeyFrame(kf);
  _system.trackingThread()->doUseNewKeyFrame(kf);
}

// void MappingThread::createNewKeyFrameImpl(
//     const KeyFrame::SharedPtr &currentKeyFrame, const Frame::SharedPtr
//     &frame) {
//   LOG(WARNING) << "Making " << frame->id() << " into new keyframe!";
//
//   CHECK(frame->isTrackingParent(currentKeyFrame))
//       << "New keyframe does not track on current keyframe!";
//
//   KeyFrame::SharedPtr kf(KeyFrame::PropagateAndCreate(currentKeyFrame,
//   frame)); _system.keyFrameGraph()->addKeyFrame(kf);
//   _system.trackingThread()->doUseNewKeyFrame(kf);
//   _system.constraintThread()->doCheckNewKeyFrame(kf);
// }

void MappingThread::createNewKeyFrameImplSet(
    const KeyFrame::SharedPtr &currentKeyFrame,
    const ImageSet::SharedPtr &set) {
  LOG(WARNING) << "Making " << set->refFrame()->id() << " into new keyframe!";
  // finishCurrentKeyframe(currentKeyFrame);
  currentKeyFrame->depthMap()->finalize();
  if (currentKeyFrame->frame()->idxInKeyframes < 0) {
    {
      boost::shared_lock_guard<boost::shared_mutex> kfLock(
          _system.keyFrameGraph()->keyframesAllMutex);
      currentKeyFrame->frame()->idxInKeyframes =
          _system.keyFrameGraph()->size();
      // _system.keyFrameGraph()->keyframesAll.push_back(kf);
    }
    // {
    //   boost::shared_lock_guard<boost::shared_mutex> kfLock(
    //       _system.keyFrameGraph()->newKeyFrameMutex);
    //   _system.keyFrameGraph()->newKeyFrames.push_back(kf);
    // }
  }

  CHECK(set->refFrame()->isTrackingParent(currentKeyFrame))
      << "New keyframe does not track on current keyframe!";

  KeyFrame::SharedPtr kf(KeyFrame::PropagateAndCreate(currentKeyFrame, set));

  _system.keyFrameGraph()->addKeyFrame(kf);
  _system.optThread()->doNewConstraint();
  _system.trackingThread()->doUseNewKeyFrame(kf);
  _system.constraintThread()->doCheckNewKeyFrame(kf);

  {
    boost::shared_lock_guard<boost::shared_mutex> kfLock(
        _system.keyFrameGraph()->keyframesAllMutex);
    _system.keyFrameGraph()->keyframesAll.push_back(kf);
  }

  // Publish outputs
  _system.publishKeyframeGraph();
  _system.publishCurrentKeyframe();
}

// TODO.  Not updated post-move_current_keyframe
void MappingThread::mergeOptimizationOffsetImpl() {
  LOG(DEBUG) << "Merging optimization offset";

  // lets us put the publishKeyframeGraph outside the mutex lock
  bool didUpdate = false;

  // if(_optThread->haveUnmergedOptimizationOffset())
  {
    boost::shared_lock_guard<boost::shared_mutex> pose_lock(
        _system.poseConsistencyMutex);
    boost::shared_lock_guard<boost::shared_mutex> kfLock(
        _system.keyFrameGraph()->keyframesAllMutex);

    for (unsigned int i = 0; i < _system.keyFrameGraph()->keyframesAll.size();
         i++)
      _system.keyFrameGraph()
          ->keyframesAll[i]
          ->frame()
          ->pose->applyPoseGraphOptResult();

    //_optThread->clearUnmergedOptimizationOffset();

    didUpdate = true;
  }

  if (didUpdate) {
    _system.publishKeyframeGraph();
  }

  optimizationUpdateMerged.notify();
}

void MappingThread::finishCurrentKeyframe(KeyFrame::SharedPtr &kf) {
  kf->depthMap()->finalize();
  if (kf->frame()->idxInKeyframes < 0) {
    {
      boost::shared_lock_guard<boost::shared_mutex> kfLock(
          _system.keyFrameGraph()->keyframesAllMutex);
      kf->frame()->idxInKeyframes = _system.keyFrameGraph()->size();
      // _system.keyFrameGraph()->keyframesAll.push_back(kf);
    }
    // {
    //   boost::shared_lock_guard<boost::shared_mutex> kfLock(
    //       _system.keyFrameGraph()->newKeyFrameMutex);
    //   _system.keyFrameGraph()->newKeyFrames.push_back(kf);
    // }
  }
}

} // namespace lsd_slam
