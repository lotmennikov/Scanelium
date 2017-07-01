#ifndef CAMERA_H_LOT
#define CAMERA_H_LOT

#include <QtGui/qimage.h>
#include <QtGui/qcolor.h>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

/** holds an image (with corresponding depth info) and corresponding camera pose */
class Camera {

public:
	QImage img;
	unsigned short* depth;
	bool depth_processed;

	Eigen::Affine3f pose;
	double blureness;
	 
	void computeBlureness();
	
	~Camera();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif
