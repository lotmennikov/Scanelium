#include <QtGui/qimage.h>
#include <QtGui/qcolor.h>
#include "structs.h"

const int kernelx[9] = {  3,  0,  -3, 
						 10,  0, -10, 
						  3,  0,  -3 };

const int kernely[9] = {  3,  10,  3, 
						  0,   0,  0, 
						 -3, -10, -3 };

const int offsetx[9] = { -1,  0,  1, 
						 -1,  0,  1, 
						 -1,  0,  1 };

const int offsety[9] = { -1, -1, -1, 
						  0,  0,  0, 
						  1,  1,  1 };

float* filterBW(QImage* img);
float* filterArrayScharrX(QImage* img);
float* filterArrayScharrY(QImage* img);

float* filterBWdepth(unsigned short* depth_buffer, int width, int height);
float* filterArrayScharr(float* img, int width, int height);
float* filterArrayScharrX(float* img, int width, int height);
float* filterArrayScharrY(float* img, int width, int height);

// compute depth discontinuities and assign weights to pixels
void computeWeightsFromDepth(float* depth, float*& weights, iparams depth_intr, float thres_grad = 0.025f);
