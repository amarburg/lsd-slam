

#pragma once

#include <memory>
#include <mutex>

#include <boost/thread/shared_mutex.hpp>

#include "active_object/active.h"

#include "util/MovingAverage.h"
#include "util/ThreadMutexObject.h"

#include "DataStructures/KeyFrame.h"

#include "DepthEstimation/DepthMap.h"
#include "Tracking/TrackingReference.h"

#include "Tracking/Relocalizer.h"

namespace lsd_slam {

class KeyFrameGraph;
class SlamSystem;

class MappingThread {
public:
  MappingThread(SlamSystem &system, bool threaded);
  ~MappingThread();

  //=== Callbacks into the thread ===

  // Create the first (uninitialized) keyframe
  // void createFirstKeyFrame(const Frame::SharedPtr &frame);
  void createFirstKeyFrame(const ImageSet::SharedPtr &set);

  // The core MapThread function, update the DepthMap with the contents of _set_
  void doMapSet(const KeyFrame::SharedPtr &kf, const ImageSet::SharedPtr &set) {
    if (_thread)
      _thread->send(std::bind(&MappingThread::mapSetImpl, this, kf, set));
    else
      mapSetImpl(kf, set);
  }
  // Create subsequent keyframes by depth map propagation
  // void doCreateNewKeyFrame(const KeyFrame::SharedPtr &keyframe,
  //                          const Frame::SharedPtr &frame) {
  //   if (_thread)
  //     _thread->send(std::bind(&MappingThread::createNewKeyFrameImpl, this,
  //                             keyframe, frame));
  //   else
  //     createNewKeyFrameImpl(keyframe, frame);
  // }

  void doCreateNewKeyFrame(const KeyFrame::SharedPtr &keyframe,
                           const ImageSet::SharedPtr &set) {
    if (_thread)
      _thread->send(std::bind(&MappingThread::createNewKeyFrameImplSet, this,
                              keyframe, set));
    else
      createNewKeyFrameImplSet(keyframe, set);
  }

  void doMergeOptimizationUpdate(void) {
    optimizationUpdateMerged.reset();
    if (_thread)
      _thread->send(
          std::bind(&MappingThread::mergeOptimizationOffsetImpl, this));
  }

  // Used during re-localization
  Relocalizer relocalizer;

  ThreadSynchronizer optimizationUpdateMerged;

private:
  SlamSystem &_system;

  // == Thread callbacks ==
  void mapSetImpl(const KeyFrame::SharedPtr &kf,
                  const ImageSet::SharedPtr &set);

  // void createNewKeyFrameImpl(const KeyFrame::SharedPtr &keyframe,
  //                            const Frame::SharedPtr &frame);

  void createNewKeyFrameImplSet(const KeyFrame::SharedPtr &keyframe,
                                const ImageSet::SharedPtr &set);

  // == Local functions ==

  void mergeOptimizationOffsetImpl();

  // == Local functions ==

  void addTimingSamples();

  void finishCurrentKeyframe(KeyFrame::SharedPtr &kf);
  void discardCurrentKeyframe();

  void debugDisplayDepthMap();

  std::unique_ptr<active_object::Active> _thread;
};

} // namespace lsd_slam
