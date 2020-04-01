
#include <opencv2/core/core.hpp>

#include <g3log/g3log.hpp> // Provides CHECK() macros

#ifdef USE_ZED
#include <zed/Camera.hpp>
#endif

#include "SophusUtil.h"

#include <libvideoio/types/Camera.h>
#include <libvideoio/types/ImageSize.h>

#pragma once

namespace lsd_slam {

using libvideoio::Camera;
using libvideoio::ImageSize;

class Configuration;

Configuration &Conf();

// Slow migration from the global settings.[h,cpp] model to a Configuration
// object.
class Configuration {
public:
  friend Configuration &Conf();

  // Does additional validation on sz
  const ImageSize &setSlamImageSize(const ImageSize &sz);
  ImageSize slamImageSize;
  //  Camera camera;

  enum { NO_STEREO = 0, STEREO_ZED } doDepth;

  bool doStereo;

  // If false, system will block while each new image is tracked and mapped
  bool runRealTime;

  bool stopOnFailedRead;
  bool SLAMEnabled;
  bool doKFReActivation;
  bool doMapping;
  bool continuousPCOutput;
  bool useEkf;
  bool syncDisparityImage;

  int refFrame;

  // settings variables
  // controlled via keystrokes
  int debugDisplay;
  bool displayDepthMap;
  bool displayInputFusedImage;
  bool displayInputImage;
  bool displayGradientMap;

  bool onSceenInfoDisplay;
  bool dumpMap;
  bool doFullReConstraintTrack;

  bool doSubpixelStereo;
  bool doLeftRightStereo;

  // Variables to control depth mapping
  float minVirtualBaselineLength;
  bool suppressLSDPoints;

  // Stereo and gradient calculations
  float minEplLengthCrop, maxEplLengthCrop;
  float gradientSampleDistance;

  // Image saturation
  bool doImageSaturation;
  double saturationAlpha;
  int saturationBeta;

  // ROS frames
  std::string lsdFrame;
  std::string globalFrame;
  bool useRectificationFrame;

  // GUI paramaters
  float pointcloudSize;
  bool printGUIinfo;
  float scale;
  bool setScale;
  float scaledTh;
  bool setscaledTh;
  float absTh;
  bool setabsTh;
  float nearSupport;
  bool setnearSupport;
  float sparisityFactor;
  bool setsparisityFactor;

  bool useVarianceFiltering;

  bool useVoxelFilter;
  float pclLeafSize;

  // Gradint creation paramaters
  float minAbsGradCreate;
  float minAbsGradDecrease;

  // image pre-processing
  bool doImageSharpen;
  int saturationKernelSize;

  float max_motion;

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

  struct PlotConfiguration {
    PlotConfiguration();

    int doWaitKey;
    bool debugStereo;
  } plot;

private:
  // Private constructor.  User shouldn't make their own copy of Configuration
  Configuration();
};

} // namespace lsd_slam
