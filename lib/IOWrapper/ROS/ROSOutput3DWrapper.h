/**
* This file is part of LSD-SLAM.
*
* Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical University of Munich)
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

#include <ros/ros.h>
#include "IOWrapper/Output3DWrapper.h"


#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>


typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;


namespace lsd_slam
{


class Frame;
class KeyFrameGraph;

struct InputPointDense
{
	float idepth;
	float idepth_var;
	unsigned char color[4];
};

struct GraphConstraint
{
	int from;
	int to;
	float err;
};

struct GraphFramePose
{
	int id;
	float camToWorld[7];
};



/** Addition to LiveSLAMWrapper for ROS interoperability. */
class ROSOutput3DWrapper : public Output3DWrapper
{
public:

	// initializes cam-calib independent stuff
	ROSOutput3DWrapper(int width, int height);
	~ROSOutput3DWrapper();

	virtual void publishPose( const Sophus::Sim3f &pose );

	// publishes a keyframe. if that frame already existis, it is overwritten, otherwise it is added.
	virtual void publishKeyframe(const Frame::SharedPtr &kf);

	virtual void publishKeyframeGraph(const std::shared_ptr<KeyFrameGraph> &graph);

	//Publish pointcloud from graph
	virtual void publishPointCloud(const Frame::SharedPtr &kf);

	virtual void updateDepthImage(unsigned char * data);

	// published a tracked frame that did not become a keyframe (i.e. has no depth data)
	virtual void publishTrackedFrame(const Frame::SharedPtr &kf);

	// publishes graph and all constraints, as well as updated KF poses.
	virtual void publishTrajectory(std::vector<Eigen::Matrix<float, 3, 1>> trajectory, std::string identifier);

	virtual void publishTrajectoryIncrement(Eigen::Matrix<float, 3, 1> pt, std::string identifier);

	virtual void publishDebugInfo(Eigen::Matrix<float, 20, 1> data);


	int publishLvl;

private:
	int width, height;

	std::string liveframe_channel;
	ros::Publisher liveframe_publisher;

	std::string keyframe_channel;
	ros::Publisher keyframe_publisher;

	std::string graph_channel;
	ros::Publisher graph_publisher;

	std::string debugInfo_channel;
	ros::Publisher debugInfo_publisher;


	std::string pose_channel;
	ros::Publisher pose_publisher;

	std::string pointcloud_channel;
	ros::Publisher pointcloud_publisher;

	ros::NodeHandle nh_;

	PointCloud::Ptr pointcloudMsg;
};
}
