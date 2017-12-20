#include "structs.h"
#include <math.h>

float normal::getLen() {
	return sqrt(x*x + y*y + z*z);
}

iparams operator/(const iparams& ip, int val) {
	iparams ip2 = ip;
	ip2.width /= val;
	ip2.height /= val;
	ip2.fx /= val;
	ip2.fy /= val;
	ip2.cx = (ip2.width - 1.0f) / 2.0f;
	ip2.cy = (ip2.height - 1.f) / 2.0f;
	return ip2;
}

const int cam_res_depth_x[2] = { 320, 640 };
const int cam_res_depth_y[2] = { 240, 480 };

const int cam_res_color_x[3] = { 320, 640, 1280 };
const int cam_res_color_y[3] = { 240, 480, 1024 };