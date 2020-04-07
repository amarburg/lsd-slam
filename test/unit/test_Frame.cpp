
#include <gtest/gtest.h>

#include "libvideoio/types/Camera.h"
#include "libvideoio/types/ImageSize.h"

#include "DataStructures/Frame.h"

#include "testimages.h"

TEST( Frame, constructor )
{
  const libvideoio::Camera cam(1000,1000,320,240);
  const libvideoio::ImageSize sz(640,480);
  const int id = 123;
  const double timestamp = 1.5;

  std::vector<BYTE> img = TestImage( 0 );

  {
    lsd_slam::Frame frame( id, cam, sz, timestamp, img.data() );


  }
}


TEST( FrameDeathTest, NotDivisibleBy16 ) {
  const libvideoio::Camera cam(1000,1000,320,240);

  ASSERT_DEATH({
    const int width = 248;
    const int height = 256;
    lsd_slam::Frame frame( 1, cam, libvideoio::ImageSize( width, height), 0.0, (float *)nullptr );
  }, "std::bad_function_call");  //"Image width \\d* isn't divisible by \\d*" );

  ASSERT_DEATH({
    const int width = 256;
    const int height = 2248;
    lsd_slam::Frame frame( 1, cam, libvideoio::ImageSize( width, height), 0.0, (float *)nullptr );
  }, "std::bad_function_call" ); //"Image height \\d* isn't divisible by \\d*" );

}
