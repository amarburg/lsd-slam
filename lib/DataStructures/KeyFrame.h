#pragma once

#include "DepthEstimation/DepthMap.h"
#include "Frame.h"
#include "Tracking/TrackingReference.h"

namespace lsd_slam {

class KeyFrame {
public:
  typedef std::shared_ptr<KeyFrame> SharedPtr;

  //== Creator methods.  Conventional constructor is hidden to ensure
  // all of the sub-objects are properly synchronized

  static KeyFrame::SharedPtr Create(const Frame::SharedPtr &frame);
  static KeyFrame::SharedPtr Create(const ImageSet::SharedPtr &set);
  static KeyFrame::SharedPtr PropagateAndCreate(const KeyFrame::SharedPtr &kf,
                                                const Frame::SharedPtr &frame);
  static KeyFrame::SharedPtr PropagateAndCreate(const KeyFrame::SharedPtr &kf,
                                                const ImageSet::SharedPtr &set);

  KeyFrame() = delete;
  KeyFrame(const KeyFrame &) = delete;

  //== Thin pass-throughs to Frame ==
  int id() const { return _frame->id(); }
  FramePoseStruct::SharedPtr pose() { return frame()->pose; }
  Sim3 getCamToWorld() { return pose()->getCamToWorld(); }

  //== Member accessors ==
  std::shared_ptr<Frame> &frame() { return _frame; }
  DepthMap::SharedPtr depthMap() { return _depthMap; }
  TrackingReference::SharedPtr &trackingReference() {
    return _trackingReference;
  }

  void setKeyFramePose(Sim3 pose) { _keyframePoseToWorld = pose; }
  Sim3 getKeyFramePose() { return _keyframePoseToWorld; }

  //== Depth maintenance functions ==
  void updateDepthFrom(const Frame::SharedPtr &frame);
  void updateDepthFrom(const ImageSet::SharedPtr &set);
  void syncDepthMapToFrame();
  void finalize();

  /** Pointers to all adjacent Frames in graph. empty for non-keyframes.*/
  std::unordered_set<KeyFrame::SharedPtr> neighbors;

  /** Multi-Map indicating for which other keyframes with which initialization
   * tracking failed.*/
  std::unordered_multimap<KeyFrame::SharedPtr, Sim3> trackingFailed;

  //== Meta statistics ==
  int numFramesTrackedOnThis;
  int numMappedOnThis;
  int numMappedOnThisTotal;

protected:
  // Constructors when not propagating from a previous keyframe

  KeyFrame(const Frame::SharedPtr &frame);
  KeyFrame(const ImageSet::SharedPtr &set);

private:
  Frame::SharedPtr _frame;
  DepthMap::SharedPtr _depthMap;
  TrackingReference::SharedPtr _trackingReference;
  Sim3 _keyframePoseToWorld;
};

} // namespace lsd_slam
