#include "camera.h"

using namespace std;

struct blur_estimation_pixel {
	float bi;
	float bw;
	float bhor;
	float bver;

	blur_estimation_pixel() : bw(0), bi(0), bhor(0), bver(0) {}
};

void Camera::computeBlureness() {
	int width = img.width()/2;
	int height = img.height()/2;
	vector< vector<blur_estimation_pixel> > est(height, vector<blur_estimation_pixel>(width));

//		rgb8_image_t img_blur(width, height);
	for (int y = 0; y < height*2; ++y) {
		for (int x = 0; x < width*2; ++x) {
			QColor pix = QColor(img.pixel(x, y));
			est[y/2][x/2].bw += ((float)pix.red()*0.3f + (float)pix.green()*0.5f + (float)pix.blue()*0.2f) / 255.0f;
		}
	}

	for (int y = 0; y < height; ++y) 
		for (int x = 0; x < width; ++x) {
			est[y][x].bw /= 4;
//			view(img_blur)(x,y) = rgb8_pixel_t((int)(est[y][x].bw*255),(int)(est[y][x].bw*255),(int)(est[y][x].bw*255));
		}

	const int blur_len = 4;
	for (int y = 0; y < height; ++y) {
		for (int x = 0; x < width; ++x) {
			float divx = 0, divy = 0;
			for (int i = -blur_len; i <= blur_len; ++i) {
				if (x + i >= 0 && x + i < width) {
					est[y][x].bver += est[y][x+i].bw;
					++divx;
				}
				if (y + i >= 0 && y + i < height) {
					est[y][x].bhor += est[y+i][x].bw;
					++divy;
				}
			}
			est[y][x].bhor /= (float)divy;
			est[y][x].bver /= (float)divx;
//			view(img_blur)(x,y) = rgb8_pixel_t((int)(est[y][x].bhor*255),(int)(est[y][x].bhor*255),(int)(est[y][x].bhor*255));
		}
	}

//	png_write_view("./blured.png", view(img_blur));

	float sumFver = 0;
	float sumFhor = 0;

	float sumVver = 0;
	float sumVhor = 0;

	for (int x = 0; x < width; ++x) {
		for (int y = 1; y < height; ++y) {
			float dfhor = abs(est[y][x].bw   - est[y-1][x].bw);

			float dbhor = abs(est[y][x].bhor - est[y-1][x].bhor);
			
			float vhor = max(0.0f, dfhor - dbhor);
			if (x > 0) {
				sumVhor += vhor;
				sumFhor += dfhor;
			}
		}
	}

	for (int x = 1; x < width; ++x) {
		for (int y = 0; y < height; ++y) {
			float dfver = abs(est[y][x].bw   - est[y][x-1].bw);

			float dbver = abs(est[y][x].bver - est[y][x-1].bver);

			float vver = max(0.0f, dfver - dbver);
			if (y > 0) {
				sumVver += vver;
				sumFver += dfver;
			}
		}
	}
	
	float blur_ver = (sumFver - sumVver)/sumFver;
	float blur_hor = (sumFhor - sumVhor)/sumFhor;

	float result = max(blur_ver, blur_hor);
	this->blureness = result;
}

Camera::~Camera() {
	if (this->depth != 0)
		delete [] depth;
}