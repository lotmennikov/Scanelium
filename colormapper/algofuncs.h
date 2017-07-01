#ifndef ALGOFUNCS_H_LOT
#define ALGOFUNCS_H_LOT


#include "camparam.h"
#include "algoparams.h"
#include "structs.h"
#include <QtGui/qcolor.h>
#include <QtGui/qimage.h>

#include <Eigen/Core>
#include <Eigen/LU>

Eigen::Vector2d Ufunc(const Eigen::Vector4d& pt, CameraParams camparams);

Eigen::Vector2d UfuncTrue(const Eigen::Vector4d& pt, CameraParams camparams);

Eigen::Vector4d Gfunc(const Eigen::Vector4d& p, const Eigen::Matrix4d& Ti);

Eigen::Vector2d Ffunc(const Eigen::Vector2d& u, const Eigen::VectorXd& xvec, AlgoParams aparam);

double Fi(const Eigen::Vector2d& u, int indf, AlgoParams aparam);

Eigen::Matrix4d eTrotation(const Eigen::VectorXd& xvec, const Eigen::Matrix4d& Ti, int add = -1, double step = 0);

Eigen::Matrix<double, 4, 6> getJge(const Eigen::Vector4d& point, const Eigen::Matrix4d& Ti, const Eigen::VectorXd xvec);

Eigen::Matrix<double, 2, 4> getJug(const Eigen::Vector4d& g, CameraParams cp);

Eigen::Matrix2d getJFu(const Eigen::Vector2d& u, const Eigen::VectorXd& xvec, AlgoParams aparam);

average_color computeColor(QImage* img,float pix_x,float pix_y);

float compute_value(float** img, float x, float y, CameraParams cp);

#endif
