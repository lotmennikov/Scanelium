#include "filters.h"

using namespace std;

float** filterArrayScharrX(QImage* img) {
	float** scharrX = new float*[img->width()];
	for (int x = 0; x < img->width(); ++x) {
		scharrX[x] = new float[img->height()];
		for (int y = 0; y < img->height(); ++y) {
			float wx = 0;
			for (int i = 0; i < 9; ++i) {
				if (x + offsetx[i] >=0 && x + offsetx[i] < img->width() && y + offsety[i] >= 0 && y + offsety[i] < img->height()) {
					QColor pix = QColor(img->pixel(x + offsetx[i], y + offsety[i]));
					wx = wx + ((float)pix.red()*0.3f + (float)pix.green()*0.5f + (float)pix.blue()*0.2f)*kernelx[i];
				}
			}
			wx /= 32.0f;

			scharrX[x][y] = wx / 255.0f; 
		}
	}
	return scharrX;
}

float** filterArrayScharrY(QImage* img) {
	float** scharrY = new float*[img->width()];
	for (int x = 0; x < img->width(); ++x) {
		scharrY[x] = new float[img->height()];
		for (int y = 0; y < img->height(); ++y) {
			float wy = 0;
			for (int i = 0; i < 9; ++i) {
				if (x + offsetx[i] >=0 && x + offsetx[i] < img->width() && y + offsety[i] >= 0 && y + offsety[i] < img->height()) {
					QColor pix = QColor(img->pixel(x + offsetx[i], y + offsety[i]));
					wy = wy + ((float)pix.red()*0.3f + (float)pix.green()*0.5f + (float)pix.blue()*0.2f)*kernely[i];
				}
			}
			wy /= 32.0f;
			scharrY[x][y] = wy / 255.0f; 
		}
	}
	return scharrY;
}

float** filterArrayScharrX(float** img, int width, int height) {
	float** scharrX = new float*[width];
	for (int x = 0; x < width; ++x) {
		scharrX[x] = new float[height];
		for (int y = 0; y < height; ++y) {
			float wx = 0;
			for (int i = 0; i < 9; ++i) {
				if (x + offsetx[i] >=0 && x + offsetx[i] < width && y + offsety[i] >= 0 && y + offsety[i] < height) {
					wx += img[x + offsetx[i]][y + offsety[i]]*kernelx[i];
				}
			}
			wx /= 16.0f;
			scharrX[x][y] = wx;
		}
	}
	return scharrX;
}

float** filterArrayScharrY(float** img, int width, int height) {
	float** scharrY = new float*[width];
	for (int x = 0; x < width; ++x) {
		scharrY[x] = new float[height];
		for (int y = 0; y < height; ++y) {
			float wy = 0;
			for (int i = 0; i < 9; ++i) {
				if (x + offsetx[i] >=0 && x + offsetx[i] < width && y + offsety[i] >= 0 && y + offsety[i] < height) {
					wy += img[x + offsetx[i]][y + offsety[i]]*kernely[i];
				}
			}
			wy /= 16.0f;
			scharrY[x][y] = wy;
		}
	}
	return scharrY;
}

float** filterArrayScharr(float** img, int width, int height) {
	float** scharr = new float*[width];
	for (int x = 0; x < width; ++x) {
		scharr[x] = new float[height];
		for (int y = 0; y < height; ++y) {
			float wx = 0;
			float wy = 0;
			for (int i = 0; i < 9; ++i) {
				if (x + offsetx[i] >=0 && x + offsetx[i] < width && y + offsety[i] >= 0 && y + offsety[i] < height) {
					wx += img[x + offsetx[i]][y + offsety[i]]*kernelx[i];
					wy += img[x + offsetx[i]][y + offsety[i]]*kernely[i];
				}
			}
			wx /= 16.0f;
			wy /= 16.0f;
            scharr[x][y] = std::min(1.0f, sqrtf(wx*wx + wy*wy));
		}
	}
	return scharr;
}

float** filterBWdepth(unsigned short* depth_buffer, int width, int height) {
	float maxdep = 6000.0f;
	float** bw_dep = new float*[width];
	for (int x = 0; x < width; ++x) {
		bw_dep[x] = new float[height];
		for (int y = 0; y < height; ++y) {
			bw_dep[x][y] = 1.0 - min(1.0f, ((float)depth_buffer[x + y*(int)width]) / maxdep); 
		}
	}
	return bw_dep;
}

float** filterBW(QImage* img) {
	float** bw = new float*[img->width()];
	for (int x = 0; x < img->width(); ++x) {
		bw[x] = new float[img->height()];
		for (int y = 0; y < img->height(); ++y) {
			QColor pix = QColor(img->pixel(x, y));
			bw[x][y] = ((float)pix.red()*0.3f + (float)pix.green()*0.5f + (float)pix.blue()*0.2f) / 255.0f;
		}
	}
	return bw;
}
