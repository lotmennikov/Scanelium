#include "pcl_kinfu2.h"

#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>

using namespace cv;

PCL_Kinfu2::PCL_Kinfu2() {
	_tracker = NULL;
}

inline cv::Affine3f eigen2cv(Eigen::Affine3f eA) {
	cv::Matx<float, 3, 3> cvR;
	Eigen::Matrix3f eR = eA.rotation();
	Eigen::Vector3f et = eA.translation();
	for (int i = 0; i < 9; ++i)
		cvR(i / 3, i % 3) = eR(i / 3, i % 3);
	cv::Affine3f cvA = Affine3f::Identity();
	cvA = Affine3f(cvR, cv::Vec3f(et.x(), et.y(), et.z()));
	return cvA;
}

inline Eigen::Affine3f cv2eigen(cv::Affine3f cvA) {
	cv::Matx<float, 3, 3> cvR = cvA.rotation();
	cv::Vec3f cvt = cvA.translation();

	Eigen::Matrix3f eR;
	for (int i = 0; i < 9; ++i)
		eR(i / 3, i % 3) = cvR(i / 3, i % 3);

	Eigen::Affine3f eA = Eigen::Affine3f::Identity();
	eA = Eigen::Translation3f(Eigen::Vector3f(cvt[0], cvt[1], cvt[2]))*Eigen::Affine3f(eR);
	return eA;
}

bool PCL_Kinfu2::init(
	int dpt_width, int dpt_height,
	int rgb_width, int rgb_height,
	float fx, float fy,
	int grid_x, int grid_y, int grid_z,
	float size_x, float size_y, float size_z,
	Eigen::Affine3f pose) {

	this->dpt_height = dpt_height;
	this->dpt_width = dpt_width;
	this->rgb_height = rgb_height;
	this->rgb_width = rgb_width;

	this->angle_diff = 0, this->dist_diff = 0;
	this->_failed = false;
	//this->_has_image = false;
	//this->_had_reset = false;

	kfusion::KinFuParams kp = kfusion::KinFuParams::default_params();
	kp.cols = dpt_width; kp.rows = dpt_height;
	kp.intr = kfusion::Intr(fx, fy, (dpt_width - 1.0f) / 2.0f, (dpt_height - 1.0f) / 2.0f);
	
	kp.volume_dims = cv::Vec3i(grid_x, grid_y, grid_z);
	kp.volume_size = cv::Vec3f(size_x, size_y, size_z);
	kp.volume_pose = eigen2cv(pose).inv();
	kp.icp_pose_angle_thres = kf::deg2rad(30.0f);
	kp.icp_pose_dist_thres = 0.20f;
	kp.tsdf_trunc_dist = std::max(0.01f, 5.0f * (size_x / grid_x));
	kp.tsdf_max_weight = 128;

	_init_pose = pose;
	//kp.volume_pose = cv::Affine3f().translate(cv::Vec3f(-kp.volume_size[0] / 2, -kp.volume_size[1] / 2, 0.4f));

	if (_tracker != NULL) delete _tracker;
	_tracker = new kfusion::KinFu(kp);

	return true;
}

bool PCL_Kinfu2::update(const DepthMap& depth) {
	//_has_image = false;
	//_failed = false;
	//_had_reset = false;

	kfusion::cuda::Depth dp;
	kfusion::cuda::Image view_device_;
	cv::Mat view_host_;
			
	//initStart();
	dp.upload(depth, dpt_width * sizeof(unsigned short), dpt_height, dpt_width);
	
	_failed = !_tracker->operator()(dp, angle_diff, dist_diff);
	//printElapsed("Tick " + to_string(tick));

	if (!_failed) {
		int mode = 0;
		_tracker->renderImage(view_device_, mode);

		view_host_.create(view_device_.rows(), view_device_.cols(), CV_8UC4);
		view_device_.download(view_host_.ptr<void>(), view_host_.step);
		cv::cvtColor(view_host_, _rgb_view_host, CV_BGRA2RGB);
		//cv::imwrite("raycast.png", _rgb_view_host);
		
		return true;
	}
	else {
		//printf("Adiff: %f, Ddiff %f\n", acos(angle_diff) / 3.14159265f * 180.f, dist_diff);
		return false;
	}
}

bool PCL_Kinfu2::getImage(std::vector<unsigned char>& bits) {
	if (_tracker == NULL) return false;

	bits.resize(dpt_width * dpt_height * 3);
	int idx = 0;
	for (int y = 0; y < dpt_height; ++y) {
		cv::Vec3b* rowy = _rgb_view_host.ptr<cv::Vec3b>(y);
		for (int x = 0; x < dpt_width; ++x, idx+=3) {
			Vec3b pix = rowy[x];
			bits[idx + 0] = pix[0];
			bits[idx + 1] = pix[1];
			bits[idx + 2] = pix[2];
		}
	}
	return true;
}

bool PCL_Kinfu2::getPrediction(std::vector<unsigned short>&) {
	return false;
}

bool PCL_Kinfu2::reset() {
	if (_tracker != NULL) {
		_tracker->reset();
		_failed = false;
		//_had_reset = true;
		return true;
	} else
		return false;
}

/*
bool PCL_Kinfu2::hadReset() {
	return _had_reset;
}*/

Eigen::Affine3f PCL_Kinfu2::getPose() {
	if (_tracker == NULL) return Eigen::Affine3f();

	Affine3f pose = _tracker->getCameraPose();
	return _init_pose * cv2eigen(pose);
}

bool PCL_Kinfu2::extractMeshVBO(float*& buffer, int& num_points) {
	if (_tracker->extractMesh(buffer, num_points)) {
		return true;
	}
	return false;
}

bool PCL_Kinfu2::extractMeshIBO(float*& vertex_buffer, int& num_vertices, int*& index_buffer, int& num_indices) {
	if (_tracker->extractMesh(vertex_buffer, num_vertices, index_buffer, num_indices)) {
		return true;
	}

	return false;
}

PCL_Kinfu2::~PCL_Kinfu2() {
	if (_tracker != NULL) delete _tracker;
}