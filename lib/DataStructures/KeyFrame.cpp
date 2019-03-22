

#include "KeyFrame.h"
#include "Frame.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace lsd_slam {

//== Static functions for creation
KeyFrame::SharedPtr KeyFrame::Create(const Frame::SharedPtr &frame) {
  KeyFrame::SharedPtr kf(new KeyFrame(frame));

  kf->depthMap()->initializeFromFrame();
  kf->syncDepthMapToFrame();

  return kf;
}

KeyFrame::SharedPtr KeyFrame::Create(const ImageSet::SharedPtr &set) {
  KeyFrame::SharedPtr kf(new KeyFrame(set));
  LOG(INFO) << "Creating New Key Frame";
  kf->depthMap()->initializeFromSet();
  kf->syncDepthMapToFrame();

  return kf;
}

KeyFrame::SharedPtr
KeyFrame::PropagateAndCreate(const KeyFrame::SharedPtr &other,
                             const Frame::SharedPtr &frame) {
  LOG(INFO) << "Propagate Key Frame";
  KeyFrame::SharedPtr kf(new KeyFrame(frame));

  float rescaleFactor = 1.0;
  kf->depthMap()->propagateFrom(other->depthMap(), rescaleFactor, false);
  kf->syncDepthMapToFrame();

  kf->frame()->pose->setThisToParent(sim3FromSE3(
      se3FromSim3(kf->frame()->pose->thisToParent_raw), rescaleFactor));

  return kf;
}
KeyFrame::SharedPtr
KeyFrame::PropagateAndCreate(const KeyFrame::SharedPtr &other,
                             const ImageSet::SharedPtr &set) {
  LOG(INFO) << "Propagate Key Frame";
  KeyFrame::SharedPtr kf(new KeyFrame(set));

  float rescaleFactor = 1.0;
  kf->depthMap()->propagateFrom(other->depthMap(), rescaleFactor, true);
  kf->syncDepthMapToFrame();

  kf->frame()->pose->setThisToParent(sim3FromSE3(
      se3FromSim3(kf->frame()->pose->thisToParent_raw), rescaleFactor));

  return kf;
}

//=== Class functions ===

KeyFrame::KeyFrame(const Frame::SharedPtr &frame)
    : numFramesTrackedOnThis(0), numMappedOnThis(0), numMappedOnThisTotal(0),
      _frame(frame), _depthMap(new DepthMap(frame)),
      _trackingReference(new TrackingReference(frame)) {
  _frame->setDepth(_depthMap);
}

KeyFrame::KeyFrame(const ImageSet::SharedPtr &set)
    : numFramesTrackedOnThis(0), numMappedOnThis(0), numMappedOnThisTotal(0),
      _frame(set->refFrame()), _depthMap(new DepthMap(set)),
      _trackingReference(new TrackingReference(set->refFrame())) {
  _frame->setDepth(_depthMap);
}

void KeyFrame::updateDepthFrom(const Frame::SharedPtr &frame) {

  assert(frame->hasTrackingParent());

  if (frame->trackingParent()->id() != id()) {
    LOGF(WARNING,
         "updating keyframe %d with frame %d, which was tracked on a different "
         "keyframe (%d).  While this should work, it is not recommended.",
         id(), frame->id(), frame->trackingParent()->id());
  }

  if (!_depthMap->updateDepthFrom(frame)) {
    // TODO Handle error

    return;
  }

  syncDepthMapToFrame();

  numMappedOnThis++;
  numMappedOnThisTotal++;
}

void KeyFrame::updateDepthFrom(const ImageSet::SharedPtr &set) {

  assert(set->refFrame()->hasTrackingParent());

  if (set->refFrame()->trackingParent()->id() != id()) {
    LOGF(WARNING,
         "updating keyframe %d with frame %d, which was tracked on a different "
         "keyframe (%d).  While this should work, it is not recommended.",
         id(), set->refFrame()->id(), set->refFrame()->trackingParent()->id());
  }

  if (!_depthMap->updateDepthFrom(set->refFrame())) {
    // TODO Handle error

    return;
  }

  syncDepthMapToFrame();

  numMappedOnThis++;
  numMappedOnThisTotal++;
}

void KeyFrame::syncDepthMapToFrame() { _frame->setDepth(_depthMap); }

void KeyFrame::finalize() {
  depthMap()->finalize();

  syncDepthMapToFrame();
  frame()->calculateMeanInformation();
  // frame()->takeReActivationData(currentDepthMap);
}

} // namespace lsd_slam
