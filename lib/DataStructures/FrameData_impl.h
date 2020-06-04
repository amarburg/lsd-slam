
#include "DataStructures/FrameMemory.h"

namespace lsd_slam {

template< int __LEVELS >
FrameData<__LEVELS>::FrameData( const Camera &cam, const ImageSize &slamImageSize, const unsigned char *img )
		:   hasIDepthBeenSet( false ),
		validity_reAct( 0 ),
		idepth_reAct( 0 ),
		idepthVar_reAct( 0 ),
		refPixelWasGood( 0 )
{


		commonInitialization( cam, slamImageSize );

		CHECK( (imgSize[0].width & (PYRAMID_DIVISOR-1)) == 0 ) << "Image width " << imgSize[0].width << " isn't divisible by " << PYRAMID_DIVISOR;
		CHECK( (imgSize[0].height & (PYRAMID_DIVISOR-1)) == 0 ) << "Image height " << imgSize[0].width << " isn't divisible by " << PYRAMID_DIVISOR;

		image[0] = FrameMemory::getInstance().getFloatBuffer(imgSize[0].area());

		float *pt = image[0];

		for(unsigned int i = 0; i < imgSize[0].area(); ++i, ++pt ) {
				// LOG(INFO) << imgSize[0].area();
				// LOG(INFO) << "pt: " << i << " " << pt;
				*pt = img[i];
		}
		imageValid[0] = true;
}


template< int __LEVELS >
FrameData<__LEVELS>::FrameData( const Camera &cam, const ImageSize &slamImageSize, const float *img )
		:   hasIDepthBeenSet( false ),
		validity_reAct( 0 ),
		idepth_reAct( 0 ),
		idepthVar_reAct( 0 ),
		refPixelWasGood( 0 )
{
		commonInitialization( cam, slamImageSize );

		CHECK( (imgSize[0].width & (PYRAMID_DIVISOR-1)) == 0 ) << "Image width " << imgSize[0].width << " isn't divisible by " << PYRAMID_DIVISOR;
		CHECK( (imgSize[0].height & (PYRAMID_DIVISOR-1)) == 0 ) << "Image height " << imgSize[0].width << " isn't divisible by " << PYRAMID_DIVISOR;

		image[0] = FrameMemory::getInstance().getFloatBuffer( imgSize[0].area() );
		memcpy(image[0], img, imgSize[0].area() * sizeof(float));
		imageValid[0] = true;
}

template< int __LEVELS >
void FrameData<__LEVELS>::commonInitialization( const Camera &cam, const ImageSize &slamImageSize )
{
		camera[0] = cam;

		for (int level = 0; level < __LEVELS; ++level)
		{
				imgSize[level] = ImageSize( slamImageSize.width >> level, slamImageSize.height >> level );

				imageValid[level] = false;
				gradientsValid[level] = false;
				maxGradientsValid[level] = false;
				idepthValid[level] = false;
				idepthVarValid[level] = false;

				image[level] = 0;
				gradients[level] = 0;
				maxGradients[level] = 0;
				idepth[level] = 0;
				idepthVar[level] = 0;
				reActivationDataValid = false;

//      refIDValid[level] = false;

				if (level > 0) camera[level] = camera[0].scale( 1.0f / (int)(1<<level) );
		}
}


template< int __LEVELS >
FrameData<__LEVELS>::~FrameData() {

		for (int level = 0; level < __LEVELS; ++level)
		{
				FrameMemory::getInstance().returnBuffer(image[level]);
				FrameMemory::getInstance().returnBuffer(reinterpret_cast<float*>(gradients[level]));
				FrameMemory::getInstance().returnBuffer(maxGradients[level]);
				FrameMemory::getInstance().returnBuffer(idepth[level]);
				FrameMemory::getInstance().returnBuffer(idepthVar[level]);
		}

		FrameMemory::getInstance().returnBuffer((float*)validity_reAct);
		FrameMemory::getInstance().returnBuffer(idepth_reAct);
		FrameMemory::getInstance().returnBuffer(idepthVar_reAct);
}

}
