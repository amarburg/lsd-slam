
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

// If false, system will block while each new image is tracked and mapped
bool runRealTime;

// LSD-SALM params
bool stopOnFailedRead;
bool SLAMEnabled;
bool doKFReActivation;
bool doMapping;
bool continuousPCOutput;

// Filtering Params
bool useEkf;           // To use EKF odom data, if provided
bool publishSetPose;   // Publish 'set_pose', i.e. odom estimate at loop clousre

// Stereo Params
bool doStereo;   // Stereo or Mono LSD-SLAM
int refFrame;    // Left (0) or right(1) image as refrence frame
bool doSubpixelStereo;
bool doLeftRightStereo;   // To propegate a new LSD-SLAM estimate with right
                          // image

// display variables
int debugDisplay;
bool displayDepthMap;
bool displayInputFusedImage;
bool displayInputImage;
bool displayGradientMap;

bool syncDisparityImage;   // To sync disprity image to left image or to have
                           // images sync

// Variables to control depth mapping
float minVirtualBaselineLength;   // Minimum motion between frames
bool suppressLSDPoints;   // To calculate LSD points or just use disparity
                          // estimates

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
bool onSceenInfoDisplay;
bool dumpMap;
bool doFullReConstraintTrack;
bool useVarianceFiltering;
bool useVoxelFilter;
float pclLeafSize;

float initalizationPhaseCount;

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
