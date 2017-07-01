#ifndef STRUCTS_LOT
#define STRUCTS_LOT

#include "enums.h"
#include "math.h"
#include <stdint.h>

struct normal {
	float x;
	float y;
	float z;

	normal() : x(0), y(0), z(0) {}
	normal(float x, float y, float z) : x(x), y(y), z(z) {}

	normal operator+(const normal& n2) const {
		return normal(x + n2.x, y + n2.y, z + n2.z);
	}

	float getLen() {
		return sqrt(x*x + y*y + z*z);
	}

};

/** Average vertex color
*/
struct average_grey
{
	float bw;
	int count;

	average_grey() : bw(0.0), count(0) {}

	average_grey(float bw_) : bw(bw_), count(1) {}

	void setBW(float _bw) {
		bw = _bw;
		count = 0;
	}   

	void addBW(float _bw) {
		bw += _bw;
		count += 1;
	}   

	void clear() {
		bw = 0.0;
		count = 0;
	}

	void addBWFunc(float _bw, float _func) {
		bw += _bw * _func;

		count += _func;
	}

	void average() {
		if (count > 0) {
			bw /= (float)count;
			count = 1;
		}
		if (bw > 1.0) 
			bw = 1.0;
	}

};

struct point_bw {
	int index;
	float bw;
	float weight;
	point_bw() : bw(0), index(-1), weight(0.0) {}
	point_bw(int index_, float bw_, float weight_) : bw(bw_), index(index_), weight(weight_) {}
};

struct average_color
{
	double r;
	double g;
	double b;
	double count;

	average_color() : r(0), g(0), b(0), count(0) {}

	average_color(double r_, double g_, double b_) : r(r_), g(g_), b(b_), count(1) {}

	void setRGB(double _r, double _g, double _b) {
		r = _r;
		g = _g;
		b = _b;
		count = 0.0;
	}   

	void addRGB(double _r, double _g, double _b) {
		r += _r;
		g += _g;
		b += _b;
		count += 1;
	}   

	void addRGBFunc(double _r, double _g, double _b, double _func) {
		r += _r * _func;
		g += _g * _func;
		b += _b * _func;

		count += _func;
	}

	void average() {
		if (count > 0.0) {
			r /= count;
			g /= count;
			b /= count;
			count = 1.0;
		}
		if (r > 1) r = 1;
		if (g > 1) g = 1;
		if (b > 1) b = 1;
	}

	float tofloat() {
		int32_t colpix = ((int)(r*255) << 16) | ((int)(g*255) << 8) | ((int)(b*255));
		return *(float*)&colpix;
	}
};

#endif