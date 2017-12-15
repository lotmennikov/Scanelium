#ifndef IMGPROCESS_H_LOT
#define IMGPROCESS_H_LOT

#include <QtCore\qthread.h>
#include <QtCore\qmutex.h>
#include <queue>
#include "frame.h"

class ImageProcessThread : public QThread {

	Q_OBJECT

	void run() Q_DECL_OVERRIDE;

	Frame* best;

	bool stopped;

	QMutex queueMutex;

public:
	ImageProcessThread(QObject* parent = 0);

	std::queue<Frame*> toProcess;

	void process(Frame* cam);

	Frame* getBestCam();

	void stop();

	void clear();
};

#endif