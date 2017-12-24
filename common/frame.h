#ifndef CAMERA_H_LOT
#define CAMERA_H_LOT

#include <QtGui/qimage.h>
#include <QtGui/qcolor.h>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

struct frame_params {
	int color_width;
	int color_height;

	int depth_width;
	int depth_height;

	float color_fx;
	float color_fy;
	
	float depth_fx;
	float depth_fy;
};

/** holds an image (with corresponding depth info) and corresponding camera pose */
class Frame {

public:
	frame_params fparams;
	QImage img;
	std::vector<unsigned short> depth;
	Eigen::Affine3f pose;

	double blureness;
	 
	void computeBlureness();
	
	~Frame();

};

#endif
