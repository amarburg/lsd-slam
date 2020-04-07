
#include <gtest/gtest.h>

#include "libvideoio/types/Camera.h"
#include "libvideoio/types/ImageSize.h"

#include "DataStructures/FrameData.h"

#include "testimages.h"

const int TestFrameDataPyramidLevels = lsd_slam::PYRAMID_LEVELS;

TEST( FrameData, constructor )
{
  const libvideoio::Camera cam(1000,1000,320,240);
  const libvideoio::ImageSize sz( TestImageSize );

  lsd_slam::FrameData<TestFrameDataPyramidLevels> data( cam, sz, TestImage(0).data() );

  for( int i = 0; i < data.Levels; ++i ) {
    float scalar=1.0/pow(2,i);

    ASSERT_FLOAT_EQ( data.camera[i].fx,  cam.fx*scalar );
    ASSERT_FLOAT_EQ( data.camera[i].fy,  cam.fy*scalar );
    ASSERT_FLOAT_EQ( data.camera[i].cx,  (cam.cx+0.5)*scalar-0.5 ); /// TODO.  This is kindof wierd math
    ASSERT_FLOAT_EQ( data.camera[i].cy,  (cam.cy+0.5)*scalar-0.5 );

    ASSERT_EQ( data.imgSize[i].width, sz.width>>i );
    ASSERT_EQ( data.imgSize[i].height, sz.height>>i );

  }

}

TEST( FrameDataDeathTest, NotDivisibleBy16 ) {
  const libvideoio::Camera cam(1000,1000,320,240);

  // TODO.  Make the death test checks more constructive

  ASSERT_DEATH({
    const int width = 248;
    const int height = 256;
    lsd_slam::FrameData<TestFrameDataPyramidLevels> data( cam, libvideoio::ImageSize( width, height), TestImage(0).data() );
  }, ".*");  //"Image width \\d* isn't divisible by \\d*" );

  ASSERT_DEATH({
    const int width = 256;
    const int height = 2248;
    lsd_slam::FrameData<TestFrameDataPyramidLevels> data( cam, libvideoio::ImageSize( width, height), TestImage(0).data() );
  }, ".*" ); //"Image height \\d* isn't divisible by \\d*" );

}
