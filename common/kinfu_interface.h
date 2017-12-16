#pragma once
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "enums.h"

class KinfuInterface {

public:
	virtual bool init(int dpt_width, int dpt_height, 
					  int rgb_width, int rgb_height, 
					  float fx, float fy, 
					  int dim_x, int dim_y, int dim_z,
					  float size_x, float size_y, float size_z,
					  Eigen::Affine3f pose) = 0;

	virtual bool update(const DepthMap&) = 0;

	virtual bool getImage(std::vector<unsigned char>&) = 0;

	virtual bool getPrediction(std::vector<unsigned short>&) = 0;

	virtual bool reset() = 0;

	//virtual bool hadReset() = 0;

	virtual Eigen::Affine3f getPose() = 0;

	virtual bool extractMeshVBO(float*& buffer, int& size) = 0;
	virtual bool extractMeshIBO(float*& vertex_buffer, int& num_vertices, int*& index_buffer, int& num_indices) = 0;

	virtual ~KinfuInterface() {};
};