#pragma once

#include <qmatrix4x4.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <structs.h>

inline QMatrix4x4 toQtPose(Eigen::Matrix3f R, Eigen::Vector3f t) {
	QMatrix4x4 qpose;
	for (int j = 0; j < 9; ++j)
		qpose(j / 3, j % 3) = R(j / 3, j % 3);
	qpose(0, 3) = t(0); 	qpose(1, 3) = t(1); 	qpose(2, 3) = t(2); 	qpose(3, 3) = 1;
	return qpose;
}

inline QMatrix4x4 toQtPose(Eigen::Matrix4f T) {
	QMatrix4x4 qpose;
	for (int j = 0; j < 9; ++j)
		qpose(j / 3, j % 3) = T(j / 3, j % 3);
	qpose(0, 3) = T(0, 3); 	qpose(1, 3) = T(1, 3); 	qpose(2, 3) = T(2, 3); 	qpose(3, 3) = 1;
	return qpose;
}

inline QMatrix4x4 computeCamPose(rec_settings rs) {
	QMatrix4x4 pose = QMatrix4x4();
	switch (rs.camera_pose) {
	case CENTER:
		pose.translate(0, 0, 0);
		break;
	case CENTERFACE:
		pose.translate(0, 0, -0.4f - rs.volume_size / 2);
		break;
	case CUSTOM:
	{
		pose.translate(0, 0, -rs.camera_distance);
		QMatrix4x4 xrot = QMatrix4x4(); xrot.rotate(rs.camera_x_angle, QVector3D(1, 0, 0));
		QMatrix4x4 yrot = QMatrix4x4(); yrot.rotate(rs.camera_y_angle, QVector3D(0, 1, 0));
		pose = yrot * xrot * pose;
	}
		break;
	default:
		break;
	}

	QMatrix4x4 offset = QMatrix4x4(); offset.translate(rs.volume_size / 2, (rs.doubleY ? 2 : 1) * rs.volume_size / 2, rs.volume_size / 2);

	pose = offset * pose;
	return pose;
}

inline Eigen::Affine3f computeCamPoseE(rec_settings rs) {
	Eigen::Affine3f pose = Eigen::Affine3f();
	switch (rs.camera_pose) {
	case CENTER:
		pose = Eigen::Translation3f(0, 0, 0);
		break;
	case CENTERFACE:
		pose = Eigen::Translation3f(0, 0, -0.4f - rs.volume_size / 2);
		break;
	case CUSTOM:
	{
		pose = Eigen::Translation3f(0, 0, -rs.camera_distance);
		Eigen::Matrix3f xrot = (Eigen::Matrix3f)Eigen::AngleAxisf(rs.camera_x_angle * 3.14159265f / 180.0f, Eigen::Vector3f::UnitX());
		Eigen::Matrix3f yrot = (Eigen::Matrix3f)Eigen::AngleAxisf(rs.camera_y_angle * 3.14159265f / 180.0f, Eigen::Vector3f::UnitY());
		pose = Eigen::Affine3f(yrot * xrot) * pose;
	}
	break;
	default:
		break;
	}

	Eigen::Affine3f offset; offset = Eigen::Translation3f(rs.volume_size / 2, (rs.doubleY ? 2 : 1) * rs.volume_size / 2, rs.volume_size / 2);

	pose = offset * pose;
	return pose;
}

void saveFloatImg(float* data, int width, int height, std::string filename, float normalize = 1.0f);

void saveSignedFloatImg(float* data, int width, int height, std::string filename, float normalize = 1.0f);