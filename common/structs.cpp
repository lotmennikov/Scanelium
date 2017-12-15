#include "structs.h"
#include <math.h>

float normal::getLen() {
	return sqrt(x*x + y*y + z*z);
}

const int cam_res_depth_x[2] = { 320, 640 };
const int cam_res_depth_y[2] = { 240, 480 };

const int cam_res_color_x[3] = { 320, 640, 1280 };
const int cam_res_color_y[3] = { 240, 480, 1024 };