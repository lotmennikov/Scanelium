#ifndef CAMPARAM_H_LOT
#define CAMPARAM_H_LOT

#include "enums.h"

class CameraParams {

public:

	float focal_x, focal_y;
	float color_width, color_height;
	float depth_width, depth_height;
	float snapshot_rate;
	bool camera_ratio_diff;

	CameraParams() {}

	CameraParams(const CameraParams& cp) : focal_x(cp.focal_x), focal_y(cp.focal_y), 
									color_width(cp.color_width), color_height(cp.color_height), 
									depth_width(cp.depth_width), depth_height(cp.depth_height), 
									snapshot_rate(cp.snapshot_rate), camera_ratio_diff(cp.camera_ratio_diff) {}

	CameraParams(float fx, float fy, float cwidth, float cheight, float dwidth, float dheight, float snapshotrate) :
		focal_x(fx), focal_y(fy), color_width(cwidth), color_height(cheight), depth_width(dwidth), depth_height(cheight), snapshot_rate(snapshotrate) {

		if (640.0 / 480.0 != color_width / color_height)
			camera_ratio_diff = true;
		else 
			camera_ratio_diff = false;
	}

	void setColorMode(ColorResolution res) {
		switch (res) {
			case COLOR_QVGA:
				color_width = 320;
				color_height= 240;
				break;
			case COLOR_VGA:
				color_width = 640;
				color_height= 480;
				break;
			case COLOR_SXGA:
				color_width = 1280;
				color_height= 1024;
				break;
			default:
				color_width = 640;
				color_height= 480;
				break;
		}
		if (640.0 / 480.0 != color_width / color_height)
			camera_ratio_diff = true;
		else 
			camera_ratio_diff = false;
	}

	void setDepthMode(DepthResolution res) {
		switch (res) {
			case DEPTH_QVGA:
				depth_width = 320;
				depth_height= 240;
				break;
			case DEPTH_VGA:
				depth_width = 640;
				depth_height= 480;
				break;
			default:
				depth_width = 640;
				depth_height= 480;
				break;
		}
	}
};

#endif