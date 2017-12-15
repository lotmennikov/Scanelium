#pragma once

#include <qmatrix4x4.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

inline QMatrix4x4 toQtPose(Eigen::Matrix3f R, Eigen::Vector3f t) {
	QMatrix4x4 qpose;
	for (int j = 0; j < 9; ++j)
		qpose(j / 3, j % 3) = R(j / 3, j % 3);
	qpose(0, 3) = t(0); 	qpose(1, 3) = t(1); 	qpose(2, 3) = t(2); 	qpose(3, 3) = 1;
	return qpose;
}
