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

void filterBW(QImage* img, float* out);
void filterArrayScharrX(QImage* img, float* out);
void filterArrayScharrY(QImage* img, float* out);

void filterArrayScharr(float* img, float* out, int width, int height);
void filterArrayScharrX(float* img, float* out, int width, int height);
void filterArrayScharrY(float* img, float* out, int width, int height);

float* filterBW(QImage* img);
float* filterArrayScharrX(QImage* img);
float* filterArrayScharrY(QImage* img);

float* filterArrayScharr(float* img, int width, int height);
float* filterArrayScharrX(float* img, int width, int height);
float* filterArrayScharrY(float* img, int width, int height);

// compute depth discontinuities and assign weights to pixels
void computeWeightsFromDepth(float* depth, float* weights, iparams depth_intr, int max_dist = 5, float thres_grad = 0.025f);
