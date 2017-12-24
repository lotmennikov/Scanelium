#include "filters.h"
#include <queue>

using namespace std;

float* filterArrayScharrX(QImage* img) {
	float* scharrX = new float[img->width() * img->height()];
	for (int x = 0; x < img->width(); ++x) {
		for (int y = 0; y < img->height(); ++y) {
			float wx = 0;
			for (int i = 0; i < 9; ++i) {
				if (x + offsetx[i] >=0 && x + offsetx[i] < img->width() && y + offsety[i] >= 0 && y + offsety[i] < img->height()) {
					QColor pix = QColor(img->pixel(x + offsetx[i], y + offsety[i]));
					wx = wx + ((float)pix.red()*0.3f + (float)pix.green()*0.5f + (float)pix.blue()*0.2f)*kernelx[i];
				}
			}
			wx /= 32.0f;

			scharrX[y * img->width() + x] = wx / 255.0f; 
		}
	}
	return scharrX;
}

float* filterArrayScharrY(QImage* img) {
	float* scharrY = new float[img->width()*img->height()];
	for (int x = 0; x < img->width(); ++x) {
		for (int y = 0; y < img->height(); ++y) {
			float wy = 0;
			for (int i = 0; i < 9; ++i) {
				if (x + offsetx[i] >=0 && x + offsetx[i] < img->width() && y + offsety[i] >= 0 && y + offsety[i] < img->height()) {
					QColor pix = QColor(img->pixel(x + offsetx[i], y + offsety[i]));
					wy = wy + ((float)pix.red()*0.3f + (float)pix.green()*0.5f + (float)pix.blue()*0.2f)*kernely[i];
				}
			}
			wy /= 32.0f;
			scharrY[y * img->width() + x] = wy / 255.0f;
		}
	}
	return scharrY;
}

float* filterArrayScharrX(float* img, int width, int height) {
	float* scharrX = new float[width*height];
	for (int x = 0; x < width; ++x) {
		for (int y = 0; y < height; ++y) {
			float wx = 0;
			for (int i = 0; i < 9; ++i) {
				if (x + offsetx[i] >=0 && x + offsetx[i] < width && y + offsety[i] >= 0 && y + offsety[i] < height) {
					wx += img[(y + offsety[i]) * width + (x + offsetx[i])]*kernelx[i];
				}
			}
			wx /= 16.0f;
			scharrX[y * width + x] = wx;
		}
	}
	return scharrX;
}

float* filterArrayScharrY(float* img, int width, int height) {
	float* scharrY = new float[width*height];
	for (int x = 0; x < width; ++x) {
		for (int y = 0; y < height; ++y) {
			float wy = 0;
			for (int i = 0; i < 9; ++i) {
				if (x + offsetx[i] >=0 && x + offsetx[i] < width && y + offsety[i] >= 0 && y + offsety[i] < height) {
					wy += img[(y + offsety[i]) * width + (x + offsetx[i])]*kernely[i];
				}
			}
			wy /= 16.0f;
			scharrY[y*width + x] = wy;
		}
	}
	return scharrY;
}

float* filterArrayScharr(float* img, int width, int height) {
	float* scharr = new float[width*height];
	for (int x = 0; x < width; ++x) {
		for (int y = 0; y < height; ++y) {
			float wx = 0;
			float wy = 0;
			for (int i = 0; i < 9; ++i) {
				if (x + offsetx[i] >=0 && x + offsetx[i] < width && y + offsety[i] >= 0 && y + offsety[i] < height) {
					wx += img[(y + offsety[i]) * width + (x + offsetx[i])] *kernelx[i];
					wy += img[(y + offsety[i]) * width + (x + offsetx[i])] *kernely[i];
				}
			}
			wx /= 16.0f;
			wy /= 16.0f;
            scharr[y*width+x] = sqrtf(wx*wx + wy*wy);
		}
	}
	return scharr;
}

float* filterBWdepth(unsigned short* depth_buffer, int width, int height) {
	float maxdep = 6000.0f;
	float* bw_dep = new float[width*height];
	for (int x = 0; x < width; ++x) {
		for (int y = 0; y < height; ++y) {
			bw_dep[y*width + x] = 1.0 - min(1.0f, ((float)depth_buffer[x + y*width]) / maxdep); 
		}
	}
	return bw_dep;
}

float* filterBW(QImage* img) {
	float* bw = new float[img->width()*img->height()];
	for (int x = 0; x < img->width(); ++x) {
		for (int y = 0; y < img->height(); ++y) {
			QColor pix = QColor(img->pixel(x, y));
			bw[y*img->width() + x] = ((float)pix.red()*0.3f + (float)pix.green()*0.5f + (float)pix.blue()*0.2f) / 255.0f;
		}
	}
	return bw;
}

void
computeWeightsFromDepth(float * depth, float*& weights, iparams dip, float thres_grad) {
	const float maxdepth = 6.000f;
	const int rad = 5;
	const int diam = rad * 2 + 1;

	float * conv_depth = filterArrayScharr(depth, dip.width, dip.height);

	float* weights_temp = new float[dip.width*dip.height];
	std::fill_n(weights_temp, dip.width*dip.height, 1.0f);

	for (int i = 0; i < dip.width*dip.height; ++i) {
		if (conv_depth[i] >= thres_grad)
			weights_temp[i] = 0;
		if (depth[i] == 0 || depth[i] >= maxdepth)
			weights_temp[i] = 0;
	}
	
	int idx = 0;
	for (int y = 0; y < dip.height; ++y) {
		for (int x = 0; x < dip.width; ++x, ++idx) {
			if (weights_temp[idx] == 0) {
				for (int i = 0; i < diam*diam; ++i) {
					int ux = i % diam - rad;
					int uy = i / diam - rad;
					
					if (x + ux < dip.width && x+ux >= 0 && y+uy < dip.height && y+uy >= 0 && ux*ux + uy*uy < rad * rad) {
						float weight = sqrtf((float)(ux*ux + uy*uy)) / (float)rad;
						if (weights_temp[(y + uy)*dip.width + (x + ux)] > weight)
							weights_temp[(y + uy)*dip.width + (x + ux)] = weight;
					}
				}
			}
		}
	}
	weights = weights_temp;
	delete[] conv_depth;
}