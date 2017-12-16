#pragma once
#include "kinfu_interface.h"

#include "kfusion\kinfu.hpp"

class PCL_Kinfu2 : public KinfuInterface {

	kfusion::KinFu * _tracker;
	int dpt_width, dpt_height;
	int rgb_width, rgb_height;

	cv::Mat _rgb_view_host;
	bool _failed;

	float angle_diff, dist_diff;
	//bool _has_image;
	//bool _had_reset;

	Eigen::Affine3f _init_pose;

public:
	PCL_Kinfu2();
	
	bool init(
		int dpt_width, int dpt_height,
		int rgb_width, int rgb_height,
		float fx, float fy,
		int dim_x, int dim_y, int dim_z,
		float size_x, float size_y, float size_z,
		Eigen::Affine3f pose);

	bool update(const DepthMap&);

	bool getImage(std::vector<unsigned char>&);

	bool getPrediction(std::vector<unsigned short>&);

	bool reset();

	float getPoseAngleDiff() const { return acos(angle_diff) / 3.14159265f * 180.f; }
	float getPoseDistDiff() const { return dist_diff; }

	Eigen::Affine3f getPose();

	bool extractMeshVBO(float*& buffer, int& size);

	bool extractMeshIBO(float*& vertex_buffer, int& num_vertices, int*& index_buffer, int& num_indices);

	~PCL_Kinfu2();
};