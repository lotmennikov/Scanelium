#ifndef CAMPARAM_H_LOT
#define CAMPARAM_H_LOT

#include "enums.h"
#include <cmath>

class CameraParams {

public:

	float focal_x, focal_y;
	float color_width, color_height;
	float depth_width, depth_height;
	bool camera_ratio_diff;

	CameraParams() {}

	CameraParams(const CameraParams& cp) : focal_x(cp.focal_x), focal_y(cp.focal_y), 
									color_width(cp.color_width), color_height(cp.color_height), 
									depth_width(cp.depth_width), depth_height(cp.depth_height), 
									camera_ratio_diff(cp.camera_ratio_diff) {}

	CameraParams(float fx, float fy, float cwidth, float cheight, float dwidth, float dheight) :
		focal_x(fx), focal_y(fy), color_width(cwidth), color_height(cheight), depth_width(dwidth), depth_height(dheight) {

		if (abs((640.0f / 480.0f) - (color_width / color_height)) > 0.000001f)
			camera_ratio_diff = true;
		else 
			camera_ratio_diff = false;
	}
};

#endif