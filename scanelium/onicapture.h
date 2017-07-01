#ifndef OPENNI2_H_LOT
#define OPENNI2_H_LOT

#include <OpenNI.h>
#include "structs.h"

class ONICapture{
	openni::Device* mDevice;
	openni::VideoStream mDepthStream;
	openni::VideoStream mColorStream;
	openni::VideoStream** mStreams;

	openni::VideoFrameRef m_depthFrame;
	openni::VideoFrameRef m_colorFrame;

	openni::Recorder* recorder;
	bool recorderStarted;

public:
	int * supportedColor;
	int * supportedDepth;

	ONICapture();
	
	~ONICapture() {
		this->destroy();
        if (mDevice != NULL)
			delete mDevice;
	}

	bool init();

	void setColorMode(int index);
	void setDepthMode(int index);

	bool start();

	bool pause();

	bool stop();

	void destroy();

	int waitFrame();
	
	int waitFrame(int stream);
	openni::VideoFrameRef getImage(int data_type);

// == Param setters
	void setRegistration(bool reg);

	void setAutoWhiteBalanceAndExposure(bool on_off);
//	void setAutoExposure(bool exp);

	int getAutoExposure();
	int getAutoWB();

// == Param getters
	int getDepthResolutionX();
	int getDepthResolutionY();
	int getColorResolutionX();
	int getColorResolutionY();

// == Recorder

	bool initRecorder(const char* filename);
	bool startRec();
	bool isRecording();
	bool stopRec();


};

#endif
