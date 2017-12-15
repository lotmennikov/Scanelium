#pragma once

#include "qobject.h"
#include "qmutex.h"
#include "qimage.h"
#include "qmatrix4x4.h"
#include "qelapsedtimer.h"

#include <vector>
#include <string>
#include <memory>

#include "kinfu_interface.h"
#include "structs.h"
#include "model.h"
#include "ImageProcessThread.h"

class Reconstructor : public QObject {

	Q_OBJECT

	KinfuInterface* _kf;
	ImageProcessThread* _ip;
	
	std::vector<uchar> last_rgb;
	std::vector<unsigned short> last_dpt;
	int last_processed_dpt;
	int last_processed_rgb;
	int current_dpt;
	int current_rgb;

	QMutex data_mutex;
	QElapsedTimer timer;

	rec_settings rec_set;
	cam_settings cam_set;
	frame_params _default_fparams;

	Eigen::Affine3f _init_pose;

	bool _started;
	bool _stop;
	bool _processing;

	bool _has_model;
	Model::Ptr _model;
	std::vector<Frame*> saved_frames;

	std::list<std::pair<int, std::vector<unsigned short>>> seq_depth;
	std::list<std::pair<int, QImage>> seq_image;
	std::list<std::pair<int, Eigen::Affine3f>> seq_pose;

	void checkSequences();
	void clearSequences();

public:
	Reconstructor();

	bool init(rec_settings, cam_settings);

	void newDepth(std::vector<unsigned short>, int);

	void newColor(QImage, int);

	~Reconstructor();

	bool isRunning();

	bool getModel(Model::Ptr& m);

public slots:

	bool start();

	bool reset();

	bool finish(bool extract = false);

private slots:

	void update();

signals:
	void ready();
	void error(std::string);
	void hadReset();
	void newRendering(QImage);
	void newPose(QMatrix4x4);
	void framesUpdate(int);
	void finished(bool);
};