
#include <opencv2/core/core.hpp>

#include <g3log/g3log.hpp>            // Provides CHECK() macros

#ifdef USE_ZED
#include <zed/Camera.hpp>
#endif

#include "SophusUtil.h"

#include <libvideoio/types/ImageSize.h>
#include <libvideoio/types/Camera.h>

#pragma once

namespace lsd_slam {

  using libvideoio::Camera;
  using libvideoio::ImageSize;

  class Configuration;

  // Accessor for the global instance of Configuration...
  Configuration &Conf();


// Slow migration from the global settings.[h,cpp] model to a Configuration
// object.
class Configuration {
public:
  friend Configuration &Conf();

  // Does additional validation on sz
  const ImageSize &setSlamImageSize( const ImageSize &sz );
  ImageSize slamImageSize;
//  Camera camera;

  enum { NO_STEREO = 0, STEREO_ZED } doDepth;

  // If false, system will block while each new image is tracked and mapped
  bool runRealTime;

  bool stopOnFailedRead;
  bool SLAMEnabled;
  bool doKFReActivation;
  bool doMapping;
  bool continuousPCOutput;

  // settings variables
  // controlled via keystrokes
 int  debugDisplay;
 bool displayDepthMap;
 bool onSceenInfoDisplay;
 bool dumpMap;
 bool doFullReConstraintTrack;

 struct PrintConfiguration {
   PrintConfiguration();

   bool propagationStatistics;
   bool fillHolesStatistics;
   bool observeStatistics;
   bool observePurgeStatistics;
   bool regularizeStatistics;
   bool lineStereoStatistics;
   bool lineStereoFails;

   bool trackingIterationInfo;
   bool threadingInfo;
   //
   bool keyframeSelectionInfo;
   bool constraintSearchInfo;
   bool optimizationInfo;
   bool relocalizationInfo;
   //
   bool frameBuildDebugInfo;
   bool memoryDebugInfo;
   //
   bool mappingTiming;
   bool overallTiming;
 } print;

private:
  // Private constructor.  User shouldn't make their own copy of Configuration
  Configuration();


};

}
