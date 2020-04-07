/**
* This file is part of LSD-SLAM.
*
* Copyright 2019 Aaron Marburg <amarburg@uw.edu>
* For more information see <http://vision.in.tum.de/lsdslam>
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

#pragma once

#include "libvideoio/types/Camera.h"
#include "libvideoio/types/ImageSize.h"

#include "util/settings.h"
#include "util/Configuration.h"


namespace lsd_slam {

  using libvideoio::Camera;
  using libvideoio::ImageSize;

  template< int __LEVELS >
	struct FrameData
	{
    const int Levels = __LEVELS;

		// Explicitly delete default and copy constructors
		FrameData() = delete;
		FrameData( const FrameData & ) = delete;

		FrameData( const Camera &camera, const ImageSize &slamImageSize, const unsigned char *data );
    FrameData( const Camera &camera, const ImageSize &slamImageSize, const float *data );

    ~FrameData();

		//int width[__LEVELS], height[__LEVELS];
    ImageSize imgSize[__LEVELS];
		Camera camera[__LEVELS];

		// Eigen::Matrix3f K[__LEVELS], KInv[__LEVELS];
		// float fx[__LEVELS], fy[__LEVELS], cx[__LEVELS], cy[__LEVELS];
		// float fxInv[__LEVELS], fyInv[__LEVELS], cxInv[__LEVELS], cyInv[__LEVELS];

		float* image[__LEVELS];
		bool imageValid[__LEVELS];

		Eigen::Vector4f* gradients[__LEVELS];
		bool gradientsValid[__LEVELS];

		float* maxGradients[__LEVELS];
		bool maxGradientsValid[__LEVELS];


		bool hasIDepthBeenSet;

		// negative depthvalues are actually allowed, so setting this to -1 does NOT invalidate the pixel's depth.
		// a pixel is valid iff idepthVar[i] > 0.
		float* idepth[__LEVELS];
		bool idepthValid[__LEVELS];

		// MUST contain -1 for invalid pixel (that dont have depth)!!
		float* idepthVar[__LEVELS];
		bool idepthVarValid[__LEVELS];

		// data needed for re-activating the frame. theoretically, this is all data the
		// frame contains.
		unsigned char* validity_reAct;
		float* idepth_reAct;
		float* idepthVar_reAct;
		bool reActivationDataValid;


		// data from initial tracking, indicating which pixels in the reference frame ware good or not.
		// deleted as soon as frame is used for mapping.
		bool* refPixelWasGood;


  protected:

    void commonInitialization( const Camera &cam, const ImageSize &slamImageSize );
	};

}

#include "FrameData_impl.h"
