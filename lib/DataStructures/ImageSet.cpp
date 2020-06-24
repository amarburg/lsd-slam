

#include "ImageSet.h"

namespace lsd_slam {

ImageSet::ImageSet(unsigned int id, const cv::Mat &img,
                   const libvideoio::Camera &cam)
		: _refFrame(0) {

		_frames.push_back(
				std::make_shared<Frame>(id, cam, img.size(), 0.0, img.data));
		_se3ToRef.push_back(Sophus::SE3d());
		_frameId = id;
		_odomEstimate = Eigen::Matrix<float, 6, 1>::Zero(6, 1);
}

ImageSet::ImageSet(unsigned int id, const std::vector<cv::Mat> &imgs,
                   const std::vector<libvideoio::Camera> &cams,
                   const unsigned int ref) {

		// LOG(INFO) << imgs.at(1).size();
		// LOG(INFO)<< "cams.at(0): " << cams.at(0).fx << " " << cams.at(0).fy;
		_frames.push_back(std::make_shared<Frame>(id, cams.at(0), imgs.at(0).size(),
		                                          0.0, imgs.at(0).data));
		_frames.push_back(std::make_shared<Frame>(id, cams.at(1), imgs.at(1).size(),
		                                          0.0, imgs.at(1).data));
		_se3ToRef.push_back(Sophus::SE3());
		_se3ToRef.push_back(Sophus::SE3());
		_frameId = id;
		_refFrame = ref;
}

ImageSet::~ImageSet() {
}

void ImageSet::addFrame(const cv::Mat &img, const libvideoio::Camera &cam,
                        const Sophus::SE3d &frameToRef) {
		_frames.push_back(
				std::make_shared<Frame>(_frameId, cam, img.size(), 0.0, img.data));
		_se3ToRef.push_back(frameToRef);
}
void ImageSet::setFrameToRef(const int frame, const Sophus::SE3d &frameToRef) {
		if (frame >= _se3ToRef.size())
				LOG(WARNING) << "Assigning out of bounds frame to ref";

		_se3ToRef.at(frame) = frameToRef;
}

void ImageSet::setRectificationMatrix(const Eigen::Matrix3f R) {
		refFrame()->setRectificationMatrix(R);
}

void ImageSet::setDisparityMap(float *_iDepth, uint8_t *_iDepthValid,
                               float _iDepthMean, int _size) {

		cv::Mat depthImg = cv::Mat::zeros(Conf().slamImageSize.height,
		                                  Conf().slamImageSize.width, CV_32FC1);
		disparity.iDepth = _iDepth;
		disparity.iDepthValid = _iDepthValid;
		disparity.iDepthSize = _size;
		disparity.iDepthMean = _iDepthMean;
}

void ImageSet::propagatePoseFromRefFrame() {
		boost::lock_guard<boost::shared_mutex> guard(setMutex);

		Sim3 refToParent = refFrame()->pose->thisToParent_raw;
		LOG(DEBUG) << "Current refToParent:\n" << refToParent.matrix3x4();

		const size_t sz = size();
		for (int i = 0; i < sz; ++i) {
				if (i == _refFrame)
						continue;

				Frame::SharedPtr thisFrame(_frames[i]);

				LOG(DEBUG) << "   Propagating pose from ref frame " << _refFrame
				           << " to sub-image " << i;
				thisFrame->pose->thisToParent_raw = sim3FromSE3(_se3ToRef[i]) * refToParent;

				LOG(DEBUG) << "   SetImage " << i << " to Parent:\n"
				           << thisFrame->pose->thisToParent_raw.matrix3x4();

				// LOG(WARNING) << sim3FromSE3(_se3ToRef[i]).matrix3x4();

				thisFrame->setTrackingParent(refFrame()->trackingParent());

				// Stopgap for now.   How to set a good value if the secondary frames
				// aren't running through the tracker
				thisFrame->initialTrackedResidual = refFrame()->initialTrackedResidual;
		}
}

void ImageSet::setOdomMotionEstimate(Eigen::Vector3f linear,
                                     Eigen::Vector3f angular) {
		_odomEstimate.block<3, 1>(0, 0) = linear;
		_odomEstimate.block<3, 1>(3, 0) = angular;
		_odomEstimateSet = true;
}

Sophus::SE3f ImageSet::getOdomEstimate() {
		Eigen::AngleAxisf rollAngle(_odomEstimate(3), Eigen::Vector3f::UnitX());
		Eigen::AngleAxisf pitchAngle(_odomEstimate(4), Eigen::Vector3f::UnitY());
		Eigen::AngleAxisf yawAngle(_odomEstimate(5), Eigen::Vector3f::UnitZ());

		Eigen::Quaternion<float> q = rollAngle * pitchAngle * yawAngle;

		Eigen::Matrix3f R = q.matrix();

		Eigen::Matrix4f G = Eigen::Matrix4f::Identity();
		G.block<3, 3>(0, 0) = R;
		G.block<3, 1>(0, 3) = _odomEstimate.block<3, 1>(0, 0);

		return Sophus::SE3f(G);
}

} // namespace lsd_slam
