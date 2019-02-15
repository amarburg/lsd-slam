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

#include "ROSOutput3DWrapper.h"
#include "util/SophusUtil.h"
#include <ros/ros.h>
#include "util/settings.h"


#include "std_msgs/Float32MultiArray.h"
#include "lsd_slam_core/keyframeGraphMsg.h"
#include "lsd_slam_core/keyframeMsg.h"

#include "DataStructures/Frame.h"
#include "GlobalMapping/KeyFrameGraph.h"
#include "sophus/sim3.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "GlobalMapping/g2oTypeSim3Sophus.h"



namespace lsd_slam
{


ROSOutput3DWrapper::ROSOutput3DWrapper(int width, int height) : pointcloudMsg (new PointCloud )
{
	this->width = width;
	this->height = height;

	liveframe_channel = nh_.resolveName("lsd_slam/liveframes");
	liveframe_publisher = nh_.advertise<lsd_slam_core::keyframeMsg>(liveframe_channel,1);

	keyframe_channel = nh_.resolveName("lsd_slam/keyframes");
	keyframe_publisher = nh_.advertise<lsd_slam_core::keyframeMsg>(keyframe_channel,1);

	graph_channel = nh_.resolveName("lsd_slam/graph");
	graph_publisher = nh_.advertise<lsd_slam_core::keyframeGraphMsg>(graph_channel,1);

	debugInfo_channel = nh_.resolveName("lsd_slam/debug");
	debugInfo_publisher = nh_.advertise<std_msgs::Float32MultiArray>(debugInfo_channel,1);

	pose_channel = nh_.resolveName("lsd_slam/pose");
	pose_publisher = nh_.advertise<geometry_msgs::PoseStamped>(pose_channel,1);

	//pointcloud_channel = nh_.resolveName("lsd_slam/pointcloud");
	pointcloud_publisher = nh_.advertise<PointCloud> ("lsd_slam/pointcloud", 1);

	pointcloudMsg->header.frame_id = "map";



	//TODO what is this??
	publishLvl=0;
}

ROSOutput3DWrapper::~ROSOutput3DWrapper()
{
}


void ROSOutput3DWrapper::publishKeyframe(const Frame::SharedPtr &kf)
{
	lsd_slam_core::keyframeMsg fMsg;


	boost::shared_lock<boost::shared_mutex> lock = kf->getActiveLock();

	fMsg.id = kf->id();
	fMsg.time = kf->timestamp();
	fMsg.isKeyframe = true;

	int w = kf->width(publishLvl);
	int h = kf->height(publishLvl);

	memcpy(fMsg.camToWorld.data(),kf->getCamToWorld().cast<float>().data(),sizeof(float)*7);
	fMsg.fx = kf->fx(publishLvl);
	fMsg.fy = kf->fy(publishLvl);
	fMsg.cx = kf->cx(publishLvl);
	fMsg.cy = kf->cy(publishLvl);
	fMsg.width = w;
	fMsg.height = h;


	fMsg.pointcloud.resize(w*h*sizeof(InputPointDense));

	InputPointDense* pc = (InputPointDense*)fMsg.pointcloud.data();

	const float* idepth = kf->idepth(publishLvl);
	const float* idepthVar = kf->idepthVar(publishLvl);
	const float* color = kf->image(publishLvl);

	for(int idx=0;idx < w*h; idx++)
	{
		pc[idx].idepth = idepth[idx];
		pc[idx].idepth_var = idepthVar[idx];
		pc[idx].color[0] = color[idx];
		pc[idx].color[1] = color[idx];
		pc[idx].color[2] = color[idx];
		pc[idx].color[3] = color[idx];
	}

	keyframe_publisher.publish(fMsg);
}

void ROSOutput3DWrapper::publishTrackedFrame(const Frame::SharedPtr &kf)
{
	lsd_slam_core::keyframeMsg fMsg;


	fMsg.id = kf->id();
	fMsg.time = kf->timestamp();
	fMsg.isKeyframe = false;


	memcpy(fMsg.camToWorld.data(),kf->getCamToWorld().cast<float>().data(),sizeof(float)*7);
	fMsg.fx = kf->fx(publishLvl);
	fMsg.fy = kf->fy(publishLvl);
	fMsg.cx = kf->cx(publishLvl);
	fMsg.cy = kf->cy(publishLvl);
	fMsg.width = kf->width(publishLvl);
	fMsg.height = kf->height(publishLvl);

	fMsg.pointcloud.clear();

	liveframe_publisher.publish(fMsg);


	SE3 camToWorld = se3FromSim3(kf->getCamToWorld());

	geometry_msgs::PoseStamped pMsg;

	pMsg.pose.position.x = camToWorld.translation()[0];
	pMsg.pose.position.y = camToWorld.translation()[1];
	pMsg.pose.position.z = camToWorld.translation()[2];
	pMsg.pose.orientation.x = camToWorld.so3().unit_quaternion().x();
	pMsg.pose.orientation.y = camToWorld.so3().unit_quaternion().y();
	pMsg.pose.orientation.z = camToWorld.so3().unit_quaternion().z();
	pMsg.pose.orientation.w = camToWorld.so3().unit_quaternion().w();

	if (pMsg.pose.orientation.w < 0)
	{
		pMsg.pose.orientation.x *= -1;
		pMsg.pose.orientation.y *= -1;
		pMsg.pose.orientation.z *= -1;
		pMsg.pose.orientation.w *= -1;
	}

	pMsg.header.stamp = ros::Time(kf->timestamp());
	pMsg.header.frame_id = "map";
	pose_publisher.publish(pMsg);
}



void ROSOutput3DWrapper::publishKeyframeGraph(const std::shared_ptr<KeyFrameGraph> &graph)
{
	lsd_slam_core::keyframeGraphMsg gMsg;

	graph->edgesListsMutex.lock();
	gMsg.numConstraints = graph->edgesAll.size();
	gMsg.constraintsData.resize(gMsg.numConstraints * sizeof(GraphConstraint));
	GraphConstraint* constraintData = (GraphConstraint*)gMsg.constraintsData.data();
	for(unsigned int i=0;i<graph->edgesAll.size();i++)
	{
		constraintData[i].from = graph->edgesAll[i]->firstFrame->id();
		constraintData[i].to = graph->edgesAll[i]->secondFrame->id();
		Sophus::Vector7d err = graph->edgesAll[i]->edge->error();
		constraintData[i].err = sqrt(err.dot(err));
	}
	graph->edgesListsMutex.unlock();

	graph->keyframesAllMutex.lock_shared();
	gMsg.numFrames = graph->keyframesAll.size();
	gMsg.frameData.resize(gMsg.numFrames * sizeof(GraphFramePose));
	GraphFramePose* framePoseData = (GraphFramePose*)gMsg.frameData.data();
	for(unsigned int i=0;i<graph->keyframesAll.size();i++)
	{
		framePoseData[i].id = graph->keyframesAll[i]->id();
		memcpy(framePoseData[i].camToWorld, graph->keyframesAll[i]->getCamToWorld().cast<float>().data(),sizeof(float)*7);
	}
	graph->keyframesAllMutex.unlock_shared();

	graph_publisher.publish(gMsg);
}

void ROSOutput3DWrapper::publishPointCloud(const Frame::SharedPtr &kf)
{
	kf->getActiveLock();

	int w = kf->width(publishLvl);
	int h = kf->height(publishLvl);

	//get world transformation
	SE3 camToWorld = se3FromSim3(kf->getCamToWorld());
	Eigen::Matrix4f G = Eigen::Matrix4f::Zero(4,4);
	Eigen::Matrix3f R;
	Eigen::Vector3f T;
	Eigen::Quaterniond q;
	q.x() = camToWorld.so3().unit_quaternion().x();
	q.y() = camToWorld.so3().unit_quaternion().y();
	q.z() = camToWorld.so3().unit_quaternion().z();
	q.w() = camToWorld.so3().unit_quaternion().w();
	if (q.w() < 0)
	{
		q.x() *= -1;
		q.y() *= -1;
		q.z() *= -1;
		q.w() *= -1;
	}
	R = q.toRotationMatrix().cast<float>();
	T(0) = camToWorld.translation()[0];
	T(1) = camToWorld.translation()[1];
	T(2) = camToWorld.translation()[2];
	G.block<3,3>(0,0) = R;
	G.block<3,1>(0,3) = T;
	G(3,3) = 1.0;
	Eigen::Matrix3f Kinv = kf->Kinv(publishLvl);

	//update PointCloud
	int prevSize = pointcloudMsg->points.size();
	pointcloudMsg->points.resize(prevSize + w*h);
	const float* idepth = kf->idepth(publishLvl);
	const float* idepthVar = kf->idepthVar(publishLvl);
	const float* color = kf->image(publishLvl);

  int idx = prevSize - 1;

	for (int y=0;y<h;y++)
	{
		for (int x=0;x<w;x++)
		{
			idx += 1;
			float z = idepth[x*y];
			if (z > 0 && idx < pointcloudMsg->points.size())
			{
				//Get accurate scale, transform frame
				Eigen::Vector3f xImg;
				xImg << x,y,1;
				Eigen::Vector3f xWorldRelative = Kinv*xImg;
				Eigen::Vector4f xWorld;
				xWorld << xWorldRelative(0)/xWorldRelative(2),
									xWorldRelative(1)/xWorldRelative(2), z, 1.0;
				Eigen::Vector4f xWorldTrans = G*xWorld;
				//Add to pointcloud
				pointcloudMsg->points[idx].x = xWorldTrans(0)/xWorldTrans(3);
				pointcloudMsg->points[idx].y = xWorldTrans(1)/xWorldTrans(3);
				pointcloudMsg->points[idx].z = xWorldTrans(2)/xWorldTrans(3);//xWorldTrans(2);
				//TODO forgot we're grayscale...
				pointcloudMsg->points[idx].r = color[x*y];
				pointcloudMsg->points[idx].g = color[x*y];
				pointcloudMsg->points[idx].b = color[x*y];

			}

		}
	}
	/*
	PointCloud::Ptr pointcloudFiltered(new PointCloud);
	pcl::VoxelGrid<PointT> vox;
  vox.setInputCloud (pointcloudMsg);
  vox.setLeafSize (0.01f, 0.01f, 0.01f);
  vox.filter (*pointcloudFiltered);

	pointcloudFiltered->header.frame_id = "map";
	pointcloud_publisher.publish(pointcloudFiltered);
	*/

	/*
	graph->edgesListsMutex.lock();

	//Loop through all keyframes
	for (unsigned int i=0; i<graph->edgesAll.size();i++)
	{

		Frame::SharedPtr kf = graph->keyframesAll.at(i);



		std::cout << G << std::endl;





		//Increase the pointcloud by the new keyframe size
		//pointcloudMsg->points.resize(w*h + pointcloudMsg->points.size());


	}


  graph->edgesListsMutex.unlock();
	*/

}

void ROSOutput3DWrapper::publishTrajectory(std::vector<Eigen::Matrix<float, 3, 1>> trajectory, std::string identifier)
{
	// unimplemented ... do i need it?
}

void ROSOutput3DWrapper::publishTrajectoryIncrement(Eigen::Matrix<float, 3, 1> pt, std::string identifier)
{
	// unimplemented ... do i need it?
}

void ROSOutput3DWrapper::updateDepthImage(unsigned char * data)
{
	// unimplemented ... do i need it?
}

void ROSOutput3DWrapper::publishPose( const Sophus::Sim3f &pose )
{
	// unimplemented ... do i need it?
}

void ROSOutput3DWrapper::publishDebugInfo(Eigen::Matrix<float, 20, 1> data)
{
	std_msgs::Float32MultiArray msg;
	for(int i=0;i<20;i++)
		msg.data.push_back((float)(data[i]));

	debugInfo_publisher.publish(msg);
}

}
