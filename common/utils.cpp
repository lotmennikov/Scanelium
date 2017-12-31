#include "utils.h"
#include <qimage.h>
#include <random>
#include <iostream>

using namespace std;

struct f3 {
	float x;
	float y;
	float z;
	f3(float _x = 0, float _y = 0, float _z = 0) : x(_x), y(_y), z(_z) {}
};

void saveFloatImg(float* data, int width, int height, std::string name, float normalize) {
	uchar* img = new uchar[3 * width*height];
	for (int i = 0; i < width*height; ++i)
	{
		float val = std::max(0.0f, std::min(1.0f, data[i] / normalize));
		img[i * 3 + 0] = val * 255;
		img[i * 3 + 1] = val * 255;
		img[i * 3 + 2] = val * 255;
	}
	QImage qi = QImage(img, width, height, QImage::Format_RGB888).copy();
	qi.save(QString::fromStdString(name));
}

void saveSignedFloatImg(float* data, int width, int height, std::string name, float normalize) {
	uchar* img = new uchar[3 * width*height];
	for (int i = 0; i < width*height; ++i)
	{
		float val = std::max(0.0f, std::min(1.0f, 0.5f + data[i] / (2.0f * normalize)));
		img[i * 3 + 0] = val * 255;
		img[i * 3 + 1] = val * 255;
		img[i * 3 + 2] = val * 255;
	}
	QImage qi = QImage(img, width, height, QImage::Format_RGB888).copy();
	qi.save(QString::fromStdString(name));
}

inline void cross(f3 v0, f3 v1, f3 v2, f3& n) {
	float ax = (v0.x - v1.x);
	float bx = (v2.x - v1.x);
	float ay = (v0.y - v1.y);
	float by = (v2.y - v1.y);
	float az = (v0.z - v1.z);
	float bz = (v2.z - v1.z);
	n.x = ay*bz - az*by;
	n.y = az*bx - ax*bz;
	n.z = ax*by - ay*bx;
}


Eigen::Vector4f computeGroundPlane(std::vector<unsigned short> depth, iparams ip) {
	float divx = 1.0f / ip.fx;
	float divy = 1.0f / ip.fy;
	ushort maxdepth = 6000;

	f3* points = new f3[ip.width*ip.height];

	int valid_points = 0;
	int valid_normals = 0;

	int idx = 0;
	for (int v = 0; v < ip.height; ++v) {
		for (int u = 0; u < ip.width; ++u, ++idx) {

			unsigned short depth_val = depth[idx];
			if (depth_val > 0 && depth_val < maxdepth) {
				float z = depth_val * 0.001f;

				Eigen::Vector3f pt(((float)u - ip.cx) * z * divx,
								   ((float)v - ip.cy) * z * divy,
								   z);
				points[idx] = f3(pt.x(), pt.y(), pt.z());
				++valid_points;
			}
			else {
				points[idx] = f3(nanf(""), nanf(""), nanf(""));
			}
		}
	}

	std::vector<std::pair<f3, f3>> pn;

	f3* norms = new f3[ip.width * ip.height];
	idx = 0;
	for (int v = 0; v < ip.height; ++v) {
		for (int u = 0; u < ip.width; ++u, ++idx) {
			if (v > 0 && u > 0) {
				f3 p0 = points[v*ip.width + u];
				f3 p1 = points[(v-1)*ip.width + u];
				f3 p2 = points[v*ip.width + (u-1)];
				if (!isnan(p0.x) && !isnan(p1.x) && !isnan(p2.x)) {
					f3 n;
					cross(p1, p0, p2, n);
					Eigen::Vector3f nor(n.x, n.y, n.z); nor.normalize();
					n.x = nor.x(); n.y = nor.y(); n.z = nor.z();
					++valid_normals;
					if (nor.y() < 0) {
						pn.push_back(make_pair(p0, n));
					}
				}
			}
		}
	}
	if (pn.size() < 100) return Eigen::Vector4f(nanf(""), 0, 0, 0);

	int num_samples = 50;
	std::default_random_engine re;
	std::uniform_int_distribution<int> distr(0, pn.size() - 1);
	auto db = bind(distr, re);

	float max_dist = 0.02f;
	Eigen::Vector4f best_plane = Eigen::Vector4f(nanf(""), 0, 0, 0);

	float best_inl = 0;
	for (int i = 0; i < num_samples; ++i) {
		int i0 = db(); f3 p0 = pn[i0].first;
		int i1 = db(); f3 p1 = pn[i1].first;
		int i2 = db(); f3 p2 = pn[i2].first;
		f3 n;
		cross(p0, p1, p2, n);
		Eigen::Vector3f nor(n.x, n.y, n.z); nor.normalize();
		n.x = nor.x(); n.y = nor.y(); n.z = nor.z();
		if (n.y > 0) { n.x = -n.x; n.y = -n.y; n.z = -n.z; }
		float d = - (p0.x * n.x + p0.y * n.y + p0.z * n.z);

		int num_in = 0;
		for (int idx = 0; idx < pn.size(); ++idx) {
			f3 p = pn[idx].first;
			float dist = p.x * n.x + p.y * n.y + p.z * n.z + d;
			if (abs(dist) <= max_dist) {
				++num_in;
			}
		}
		if (num_in / (float)pn.size() > best_inl) {
			best_plane = Eigen::Vector4f(n.x, n.y, n.z, d);
			best_inl = num_in / (float)pn.size();
		}
	}

	if (best_inl < 0.20) return Eigen::Vector4f(nanf(""), 0, 0, 0);
	else {
		//std::cout << "Selected: " << pn.size() << ", " << 100 * pn.size() / valid_normals << "%, inl: " << (int)(100 * best_inl) << "% " << std::endl;
		return best_plane;
	}
}

bool rotateXZalignY(Eigen::Vector4f plane, int yangle, float& xangle, float& zangle) {
	
	Eigen::Vector3f n(plane.x(), plane.y(), plane.z());
	//Eigen::Vector3f ydir(0, -1, 0);

	Eigen::Vector3f curr_vec = n;
	xangle = 0;
	zangle = 0;

	if (curr_vec.y() <= 0) {
		float xy_norm = sqrtf(curr_vec.x()*curr_vec.x() + curr_vec.y() * curr_vec.y());
		float curr_z_cos = curr_vec.x() / xy_norm;
		if (abs(curr_z_cos) > 0.000000001f) {

			float curr_z_rot = -acos(curr_z_cos);
			float inv_z_rad = (-3.14159265f / 2.0f - curr_z_rot);

			Eigen::Matrix3f zrot = (Eigen::Matrix3f)Eigen::AngleAxisf(inv_z_rad, Eigen::Vector3f::UnitZ());
			zangle = 180.0f * inv_z_rad / 3.14159265f;

			curr_vec = (zrot*curr_vec).normalized();
		}
		float curr_x_cos = curr_vec.z();
		if (abs(curr_x_cos) > 0.000000001f) {
			float curr_x_rot = -acos(curr_x_cos);
			float inv_x_rad = -(-3.14159265f / 2.0f - curr_x_rot);

			Eigen::Matrix3f xrot = (Eigen::Matrix3f)Eigen::AngleAxisf(inv_x_rad, Eigen::Vector3f::UnitX());
			xangle = 180.0f * inv_x_rad / 3.14159265f;

			curr_vec = (xrot*curr_vec).normalized();
		}
		//std::cout << "xangle: " << xangle << ", zangle: " << zangle << "\n align2: \n" << curr_vec << std::endl;
		return true;
	}
	return false;
}