#ifndef IMGPROCESS_H_LOT
#define IMGPROCESS_H_LOT

#include <QtCore\qthread.h>
#include <QtCore\qmutex.h>
#include <queue>
#include "camera.h"

class ImageProcessThread : public QThread {

	Q_OBJECT

	void run() Q_DECL_OVERRIDE;

	Camera* best;

	bool stopped;

	QMutex queueMutex;

public:
	ImageProcessThread(QObject* parent = 0);

	std::queue<Camera*> toProcess;

	void process(Camera* cam);

	Camera* getBestCam();

	void stop();

	void clear();
};

#endif