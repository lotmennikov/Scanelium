#include "utils.h"
#include <qimage.h>

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