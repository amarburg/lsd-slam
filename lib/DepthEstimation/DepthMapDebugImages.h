
#pragma once


#include <opencv2/core.hpp>

#include "libvideoio/types/ImageSize.h"

#include "DataStructures/Frame.h"
#include "DepthMapPixelHypothesis.h"

namespace lsd_slam {

class DepthMapDebugImages {
public:
  DepthMapDebugImages( const DepthMapDebugImages & ) = delete;

  DepthMapDebugImages( const libvideoio::ImageSize &imgSize );
  ~DepthMapDebugImages();


  // Accessors
  const cv::Mat &depthImage() const { return _debugImageDepth; }

  void plotUpdateKeyFrame( const Frame::SharedPtr &activeKeyFrame,
                          const Frame::SharedPtr &oldestReferenceFrame,
                          const Frame::SharedPtr &newestReferenceFrame );
  void displayUpdateKeyFrame();

  void plotNewKeyFrame( const Frame::SharedPtr &newKeyFrame );
  void displayNewKeyFrame();

  //
  void setHypothesisHandling( int x, int y, cv::Vec3b color );

  void addStereoLine( const cv::Point &a, const cv::Point &b, const cv::Scalar &color );

  // TODO.  First version is a short-term reversion to using DepthMapPixelHypothesis *
  // should purge DepthMapPixelHypothesis * from code base, then remove this
  // version of the function...
  int debugPlotDepthMap( const Frame::SharedPtr &activeKeyFrame, DepthMapPixelHypothesis *currentDepthMap, int refID, const char *buf1, const char *buf2 );
  int debugPlotDepthMap( const Frame::SharedPtr &activeKeyFrame, const DepthMapPixelHypothesisVector &currentDepthMap, int refID, const char *buf1, const char *buf2 );


protected:

  const libvideoio::ImageSize _imageSize;

	// ONLY for debugging, their memory is managed (created & deleted) by this object.
	cv::Mat _debugImageHypothesisHandling;
	cv::Mat _debugImageHypothesisPropagation;
	cv::Mat _debugImageStereoLines;
	cv::Mat _debugImageDepth;

};

}
