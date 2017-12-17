#ifndef OPENNI2_H_LOT
#define OPENNI2_H_LOT

#include <OpenNI.h>
#include <qobject.h>
#include <qimage.h>
#include <vector>
#include <iostream>
#include "structs.h"

class ONICapture : public QObject {

	Q_OBJECT

	openni::Device* mDevice;
	openni::VideoStream mDepthStream;
	openni::VideoStream mColorStream;
	openni::VideoStream** mStreams;

	openni::VideoFrameRef m_depthFrame;
	openni::VideoFrameRef m_colorFrame;

	openni::Recorder* recorder;
	bool recorderStarted;

	enum { INIT_NONE = 0, INIT_DEVICE = 1, INIT_FILE = 2 };

	int _init_type;
	bool _started;
	bool _stop;
	bool _sync;

public:
	int * supportedColor;
	int * supportedDepth;

	ONICapture();

	~ONICapture();

	bool initDevice();
	bool initFile(std::string file);

	void setColorMode(int index);
	void setDepthMode(int index);

	void start();
	void run();
	void pause();
	void stop();
	void destroy();

	int waitFrame();
	int waitFrame(int stream);
	openni::VideoFrameRef getImage(int data_type);

	// == Param setters
	void syncTimestamp(bool sync);
	void setRegistration(bool reg);
	void setAutoWhiteBalanceAndExposure(bool on_off);
	//	void setAutoExposure(bool exp);


	// == Param getters
	int getDepthResolutionX();
	int getDepthResolutionY();
	int getColorResolutionX();
	int getColorResolutionY();

	int getAutoExposure();
	int getAutoWB();
	bool isRunning();
	bool isValid();
	// == Recorder

	bool initRecorder(std::string filename);
	bool startRec();
	bool isRecording();
	bool stopRec();

signals:
	void error(std::string err);
	void eof();

	void newDepth(std::vector<unsigned short>, int, int, int);
	void newColor(QImage, int, int, int);

};

#endif
