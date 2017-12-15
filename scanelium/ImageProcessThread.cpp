#include "ImageProcessThread.h"
#include <iostream>
#include <Windows.h>

using namespace std;

ImageProcessThread::ImageProcessThread(QObject* parent) : QThread(parent) 
{
	best = NULL;
}

void ImageProcessThread::run() {
	best = NULL;
	stopped = false;
	while (!stopped) {
	
		queueMutex.lock();
		if (!toProcess.empty()) {
			Frame * cam = toProcess.front();
			toProcess.pop();
			queueMutex.unlock();

			cam->computeBlureness();

			queueMutex.lock();
			if (best == NULL ||
				best->blureness > cam->blureness)
				best = cam;
			else 
				delete cam;
			queueMutex.unlock();

		} else {
			queueMutex.unlock();
			Sleep(20);
		}
	}
	clear();
}

void ImageProcessThread::process(Frame* img) {
	if (stopped) {
		delete img;
		return;
	}
	queueMutex.lock();
	if (toProcess.size() < 2) {
		toProcess.push(img);
	} else {
		cout << "ImageProcess: ToProcess queue is too large (>2)" << endl;
	}
	queueMutex.unlock();
}

void ImageProcessThread::stop() {
	stopped = true;
}

Frame* ImageProcessThread::getBestCam() {
	queueMutex.lock();
	if (!toProcess.empty()) {
		while (!toProcess.empty()) {
			delete toProcess.front(); 
			toProcess.pop();
		}
	}

	if (best != NULL)
	{
		Frame* res = best;
		best = NULL;
		queueMutex.unlock();
		return res;
	} else  {
		queueMutex.unlock();
		cout << "ImageProcess: No good cameras" << endl;
		return NULL;
	}

}

void ImageProcessThread::clear() {
	queueMutex.lock();
	if (!toProcess.empty()) {
		while (!toProcess.empty()) {
			delete toProcess.front(); 
			toProcess.pop();
		}
	}
	if (best != NULL) {
		delete best;
		best = NULL;
	}
	queueMutex.unlock();
}
