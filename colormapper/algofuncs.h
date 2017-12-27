#ifndef ALGOFUNCS_H_LOT
#define ALGOFUNCS_H_LOT

#include "algoparams.h"
#include "structs.h"
#include "model.h"
#include <QtGui/qcolor.h>
#include <QtGui/qimage.h>

#include <Eigen/Core>
#include <Eigen/LU>

struct img_data {
	Eigen::Matrix4d TransM;

	iparams* ip;
	float** bw;
	float** scharrx;
	float** scharry;

	float* render;
	float* dpt_weights;
};


Eigen::Vector2d Ufunc(const Eigen::Vector4d& pt, iparams cip, bool check = true);

Eigen::Vector4d Gfunc(const Eigen::Vector4d& p, const Eigen::Matrix4d& Ti);

Eigen::Vector2d Ffunc(const Eigen::Vector2d& u, const Eigen::VectorXd& xvec, AlgoParams aparam);

double Fi(const Eigen::Vector2d& u, int indf, AlgoParams aparam);

Eigen::Matrix4d eTrotation(const Eigen::VectorXd& xvec, const Eigen::Matrix4d& Ti, int add = -1, double step = 0);

Eigen::Matrix<double, 4, 6> getJge(const Eigen::Vector4d& point, const Eigen::Matrix4d& Ti);

Eigen::Matrix<double, 2, 4> getJug(const Eigen::Vector4d& g, iparams cip);

Eigen::Matrix2d getJFu(const Eigen::Vector2d& u, const Eigen::VectorXd& xvec, AlgoParams aparam);

average_color computeColor(QImage* img,float pix_x,float pix_y);

float compute_value(float* img, float x, float y, iparams cip);

#endif
