#pragma once

#include "kinfu_interface.h"
#include <pcl/gpu/kinfu/kinfu.h>
#include <pcl/gpu/kinfu/marching_cubes.h>
#include <vector>

class PCL_Kinfu : public KinfuInterface {


private:

	/*
	// kinfu
	pcl::gpu::KinfuTracker kinfu_;
	pcl::gpu::KinfuTracker::DepthMap depth_device_;
	pcl::gpu::KinfuTracker::View view_device_;
	std::vector<pcl::gpu::KinfuTracker::PixelRGB> view_host_;

	std::vector<pcl::gpu::KinfuTracker::PixelRGB> source_image_data_;
	std::vector<unsigned short> source_depth_data_;
	pcl::gpu::PtrStepSz<const unsigned short> depth_;
	pcl::gpu::PtrStepSz<const pcl::gpu::KinfuTracker::PixelRGB> rgb24_;

	pcl::gpu::MarchingCubes::Ptr marching_cubes_;
	pcl::gpu::DeviceArray<pcl::PointXYZ> triangles_buffer_device_;
	*/
public:

};