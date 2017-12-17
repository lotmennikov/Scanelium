#include "onicapture.h"

using namespace openni;
using namespace std;

ONICapture::ONICapture() : QObject(NULL) {
	qDebug("Oni create");

	_init_type = INIT_NONE;
	_started = false;
	_stop = false;

	recorder = nullptr;
	recorderStarted = false;
	_sync = true;
}

bool ONICapture::initDevice() {
	qDebug("Oni::initDevice");

	openni::Status rc = openni::STATUS_OK;
	rc = openni::OpenNI::initialize();

	mDevice = new Device();
	const char* deviceURI = openni::ANY_DEVICE;


	printf("After initialization:\n%s\n", openni::OpenNI::getExtendedError());

	rc = mDevice->open(deviceURI);
	if (rc != openni::STATUS_OK)
	{
		printf("SimpleViewer: Device open failed:\n%s\n", openni::OpenNI::getExtendedError());
		openni::OpenNI::shutdown();
		return false;
	}

	int VGA_30_imode = 0, VGA_30_dmode = 0;

	supportedColor = new int[3]; // hard
	for (int i = 0; i < 3; ++i) supportedColor[i] = -1;
	
	supportedDepth = new int[2]; // hard
	for (int i = 0; i < 2; ++i) supportedDepth[i] = -1;

	rc = mColorStream.create(*mDevice, openni::SENSOR_COLOR);
	
	if (rc == openni::STATUS_OK)
	{
		const openni::Array<openni::VideoMode>* imodes = &mDevice->getSensorInfo(openni::SENSOR_COLOR)->getSupportedVideoModes();
		
		openni::PixelFormat currentpixel = mColorStream.getVideoMode().getPixelFormat();
		for (int i = 0; i < imodes->getSize(); ++i) {
			printf("Image: %dx%d, FPS: %d, Format: %d\n", (*imodes)[i].getResolutionX(), (*imodes)[i].getResolutionY(), (*imodes)[i].getFps(), (*imodes)[i].getPixelFormat()); 

			// selecting supported camera mode QVGA
			if ((*imodes)[i].getResolutionX() == 320 && (*imodes)[i].getResolutionY() == 240 &&
				(*imodes)[i].getFps() == 30 && (*imodes)[i].getPixelFormat() == currentpixel) {
				if (supportedColor[ColorResolution::COLOR_QVGA] == -1) {
					supportedColor[ColorResolution::COLOR_QVGA] = i;
					printf("COLOR QVGA: %d\n", i);
				}
				continue;
			}

			// selecting supported camera mode VGA
			if ((*imodes)[i].getResolutionX() == 640 && (*imodes)[i].getResolutionY() == 480 &&
				(*imodes)[i].getFps() == 30 && (*imodes)[i].getPixelFormat() == currentpixel) {
				if (supportedColor[ColorResolution::COLOR_VGA] == -1) {
					supportedColor[ColorResolution::COLOR_VGA] = i;
					printf("COLOR VGA: %d\n", i);
				}
				continue;
			}
			// selecting supported camera mode SXGA
			if ((*imodes)[i].getResolutionX() == 1280 && (*imodes)[i].getResolutionY() == 1024 &&
				(*imodes)[i].getFps() == 30 && (*imodes)[i].getPixelFormat() == currentpixel) {
				if (supportedColor[ColorResolution::COLOR_SXGA] == -1) {
					supportedColor[ColorResolution::COLOR_SXGA] = i;
					printf("COLOR SXGA: %d\n", i);
				}
				continue;
			}
//			if ((*imodes)[i].getResolutionX() == 1280 && (*imodes)[i].getResolutionY() == 1024 && 
//				(*imodes)[i].getFps() == 30 && (*imodes)[i].getPixelFormat() == currentpixel)
//			{	
//				VGA_30_imode = i;
//				break;
//			}
		}		
//		mColorStream.setVideoMode((*imodes)[VGA_30_imode]);//*/
	}
	else
	{
		printf("SimpleViewer: Couldn't find color stream:\n%s\n", openni::OpenNI::getExtendedError());
		return false;
	}

	rc = mDepthStream.create(*mDevice, openni::SENSOR_DEPTH);
	if (rc == openni::STATUS_OK)
	{
		const openni::Array<openni::VideoMode>* dmodes = &mDevice->getSensorInfo(openni::SENSOR_DEPTH)->getSupportedVideoModes();
	
		openni::PixelFormat currentpixel = mDepthStream.getVideoMode().getPixelFormat();
		for (int i = 0; i < dmodes->getSize(); ++i) {
			printf("Depth: %dx%d, FPS: %d, PixelF: %d\n", (*dmodes)[i].getResolutionX(), (*dmodes)[i].getResolutionY(), (*dmodes)[i].getFps(), (*dmodes)[i].getPixelFormat()); 
			// selecting supported camera mode
			if ((*dmodes)[i].getResolutionX() == 320 && (*dmodes)[i].getResolutionY() == 240 &&
				(*dmodes)[i].getFps() == 30 && (*dmodes)[i].getPixelFormat() == currentpixel) {
				if (supportedDepth[DepthResolution::DEPTH_QVGA] == -1) {
					supportedDepth[DepthResolution::DEPTH_QVGA] = i;
					printf("DEPTH QVGA: %d\n", i);
				}
				continue;
			}
			if ((*dmodes)[i].getResolutionX() == 640 && (*dmodes)[i].getResolutionY() == 480 &&
				(*dmodes)[i].getFps() == 30 && (*dmodes)[i].getPixelFormat() == currentpixel) {
				if (supportedDepth[DepthResolution::DEPTH_VGA] == -1) {
					supportedDepth[DepthResolution::DEPTH_VGA] = i;
					printf("DEPTH QVGA: %d\n", i);
				}
				continue;
			}

		}
//		mDepthStream.setVideoMode((*dmodes)[VGA_30_dmode]); //*/
	}
	else
	{
		printf("SimpleViewer: Couldn't find depth stream:\n%s\n", openni::OpenNI::getExtendedError());
		return false;
	}
	mColorStream.setMirroringEnabled(false);
	mDepthStream.setMirroringEnabled(false);

	mStreams = new VideoStream*[2];
	mStreams[0] = &mDepthStream;
	mStreams[1] = &mColorStream;

	_init_type = INIT_DEVICE;
	return true;
}

bool ONICapture::initFile(string file) {
	qDebug("Oni::initFile");

	openni::Status rc, rc2;

	// initialize and open file
	openni::OpenNI::initialize();
	mDevice = new Device();
	rc = mDevice->open(file.c_str());

	if (rc != openni::STATUS_OK) {
		printf("ONICapture: Cannot open file %s:\n%s\n", file.c_str(), openni::OpenNI::getExtendedError());
		openni::OpenNI::shutdown();
		return false;
	}

	rc  = mDepthStream.create(*mDevice, openni::SENSOR_DEPTH);
	rc2 = mColorStream.create(*mDevice, openni::SENSOR_COLOR);
	
	if (rc != openni::STATUS_OK || rc2 != openni::STATUS_OK) {
		printf("ONICapture: Cannot create streams :\n%s\n", openni::OpenNI::getExtendedError());
		openni::OpenNI::shutdown();
		return false;
	}

	mStreams = new VideoStream*[2];
	mStreams[0] = &mDepthStream;
	mStreams[1] = &mColorStream;
	mColorStream.setMirroringEnabled(false);
	mDepthStream.setMirroringEnabled(false);

	_init_type = INIT_FILE;
	return true;
}

void ONICapture::setColorMode(int index) {
	if (mColorStream.isValid()) {
		mColorStream.stop();
	}
	if (supportedColor[index] != -1) {
		mColorStream.setVideoMode(mDevice->getSensorInfo(openni::SENSOR_COLOR)->getSupportedVideoModes()[supportedColor[index]]);
		if (index != 2)
			_sync = true;
		else
			_sync = false;
	}
}

void ONICapture::setDepthMode(int index) {
	if (mDepthStream.isValid()) {
		mDepthStream.stop();
	}
	if (supportedDepth[index] != -1)
		mDepthStream.setVideoMode(mDevice->getSensorInfo(openni::SENSOR_DEPTH)->getSupportedVideoModes()[supportedDepth[index]]);
}

void ONICapture::start() { 
	qDebug("Oni start");
	if (_started) return;

	openni::Status rc;
	
	//setRegistration(true);
	//syncTimestamp(true);

	rc = mColorStream.start();
	if (rc != openni::STATUS_OK)
	{
		printf("SimpleViewer: Couldn't start color stream:\n%s\n", openni::OpenNI::getExtendedError());
		mColorStream.destroy();
	}

	rc = mDepthStream.start();
	if (rc != openni::STATUS_OK)
	{
		printf("SimpleViewer: Couldn't start depth stream:\n%s\n", openni::OpenNI::getExtendedError());
		mDepthStream.destroy();
	}

	setRegistration(true);
	syncTimestamp(_sync);
	setAutoWhiteBalanceAndExposure(true);

	if (!mColorStream.isValid() || !mDepthStream.isValid())
		emit error("Coudn't start stream");
	else
		run();
}

void ONICapture::run() {
	_stop = false;
	_started = true;

	int depth_counter = 0;
	int color_counter = 0;
	int max_depth_frames = 3;
	int max_color_frames = 1;

	bool check_frame_order = false;

	while (!_stop) {
		int index;
		if (!check_frame_order || (depth_counter < max_depth_frames && color_counter < max_color_frames))
			index = waitFrame();
		else if (depth_counter >= max_depth_frames)
			index = waitFrame(1);
		else if (color_counter >= max_color_frames)
			index = waitFrame(0);

		switch (index) {
		case -1:
			emit error("Stream wait error");
			_stop = true;
			break;
		case 0:
			// depth
			{
				VideoFrameRef frame_dpt = getImage(index);
				vector<unsigned short> depth;
				int width = this->getDepthResolutionX();
				int height = this->getDepthResolutionY();
				int frame_index = frame_dpt.getFrameIndex();
//				printf("Depth Frame: %d x %d\n", width, height);
				depth.resize(width * height);
//				printf("dpt: ds=%d, bs=%d\n", frame_dpt.getDataSize(), width*height * sizeof(unsigned short));
				memcpy(&depth[0], frame_dpt.getData(), frame_dpt.getDataSize());
				emit newDepth(depth, width, height, frame_index);
		
				qDebug(QString("Depth frame %1").arg(frame_index).toStdString().c_str());
				if (check_frame_order) {
					++depth_counter;
					color_counter = 0;
					qDebug(QString("Depth frame %3, counter %1-%2").arg(depth_counter).arg(color_counter).arg(frame_dpt.getFrameIndex()).toStdString().c_str());
				}
			}
			break;
		case 1:
			//color	
			{
				VideoFrameRef frame_rgb = getImage(index);
				int width = this->getColorResolutionX();
				int height = this->getColorResolutionY();
				int frame_index = _sync ? frame_rgb.getFrameIndex() : -1;
//				printf("Color Frame: %d x %d\n", width, height);

				uchar* rgb_buffer = new uchar[3 * width*height];
//				printf("rgb: ds=%d, bs=%d\n", frame_rgb.getDataSize(), 3 * width*height);
				memcpy(rgb_buffer, frame_rgb.getData(), frame_rgb.getDataSize());
				QImage img = QImage(rgb_buffer, width, height, QImage::Format_RGB888).copy();
				delete[] rgb_buffer;

				emit newColor(img, width, height, frame_index);

				qDebug(QString("Color frame %1").arg(frame_index).toStdString().c_str());
				if (check_frame_order) {
					++color_counter;
					depth_counter = 0;
					qDebug(QString("Color frame %3, counter %1-%2").arg(depth_counter).arg(color_counter).arg(frame_rgb.getFrameIndex()).toStdString().c_str());
				}
			}
			break;
		default:
			emit error("Unexpected stream index");
			_stop = true;
			break;
		}
	}
	_started = false;
}

void ONICapture::stop() {
	qDebug("Oni stop");
	if (_started) {
		_stop = true;
		while (_started) {
			Sleep(10);
		}
	}

	if (mColorStream.isValid())
		mColorStream.stop();
	if (mDepthStream.isValid())
		mDepthStream.stop();
	if (recorder != NULL)
		stopRec();
}

void ONICapture::destroy() {
	qDebug("Oni destroy");
	if (_started)
		stop();

	if (_init_type != INIT_NONE) {
		mDepthStream.destroy();
		mColorStream.destroy();
		openni::OpenNI::shutdown();

		if (recorder != NULL) stopRec();
	}
	_init_type = INIT_NONE;
}

ONICapture::~ONICapture() {
	qDebug("Oni delete");
	this->destroy();
	if (mDevice != NULL)
		delete mDevice;
}

bool ONICapture::isRunning() {
	return _started;
}

bool ONICapture::isValid() {
	return _init_type != 0;
}

void ONICapture::setRegistration(bool reg) {
	if (reg) {
		mDevice->setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
	} else {
		mDevice->setImageRegistrationMode(openni::IMAGE_REGISTRATION_OFF);
	}
}

void ONICapture::syncTimestamp(bool sync) {
	mDevice->setDepthColorSyncEnabled(sync);
}

void ONICapture::setAutoWhiteBalanceAndExposure(bool on_off)
{
	mColorStream.getCameraSettings()->setAutoWhiteBalanceEnabled(on_off);
	mColorStream.getCameraSettings()->setAutoExposureEnabled(on_off);
}

int ONICapture::getAutoWB() {
	if (mColorStream.isValid()) {
		return mColorStream.getCameraSettings()->getAutoWhiteBalanceEnabled();
	}
	return -1;
}

int ONICapture::getAutoExposure() {
	if (mColorStream.isValid()) {
		return mColorStream.getCameraSettings()->getAutoExposureEnabled();
	}
	return -1;
}

//  * * Frame getters * * 

int ONICapture::waitFrame() {
	int changedIndex;
	openni::Status rc = openni::OpenNI::waitForAnyStream(mStreams, 2, &changedIndex);
	if (rc != openni::STATUS_OK)
	{
		printf("Wait failed\n");
		return -1;
	}
	else return changedIndex;
}

int ONICapture::waitFrame(int stream) {
	int changedIndex;
	openni::Status rc;
	if (stream == 0) {
		rc = openni::OpenNI::waitForAnyStream(&mStreams[0], 1, &changedIndex);
	} else if (stream == 1) {
		rc = openni::OpenNI::waitForAnyStream(&mStreams[1], 1, &changedIndex);
	}
	changedIndex = stream;
	if (rc != openni::STATUS_OK)
	{
		printf("Wait failed\n");
		return -1;
	}
	else return changedIndex;
}

VideoFrameRef ONICapture::getImage(int changedIndex) {
	switch (changedIndex)
	{
	case 0:
		mDepthStream.readFrame(&m_depthFrame); 
		return m_depthFrame;
		break;
	case 1:
		mColorStream.readFrame(&m_colorFrame); 
		return m_colorFrame;
		break;
	default:
		printf("Error in wait\n");
		return m_depthFrame;
	}
}

// * * Resolutions * *

int ONICapture::getColorResolutionX() {
	if (mColorStream.isValid()) 
		return mColorStream.getVideoMode().getResolutionX();
	else
		return 0;
}

int ONICapture::getColorResolutionY() {
	if (mColorStream.isValid()) 
		return mColorStream.getVideoMode().getResolutionY();
	else
		return 0;
}

int ONICapture::getDepthResolutionX() {
	if (mDepthStream.isValid()) 
		return mDepthStream.getVideoMode().getResolutionX();
	else
		return 0;
}

int ONICapture::getDepthResolutionY() {
	if (mDepthStream.isValid()) 
		return mDepthStream.getVideoMode().getResolutionY();
	else
		return 0;
}

// == RECORDER ==

bool ONICapture::initRecorder(string filename) {
	if (recorder != NULL) stopRec();

	recorder = new Recorder();
	if (recorder->create(filename.c_str()) != 0) {
		printf("Failed to create ONI Recorder\n");
		return 0;
	} else 
		return 1;
}

bool ONICapture::startRec() {
	if (recorder->isValid()) {
		recorder->attach(mDepthStream);
		recorder->attach(mColorStream);

		recorder->start();
		recorderStarted = true;
		return 1;
	} else 
		return 0;
}

bool ONICapture::isRecording() {
	return recorderStarted;
}

bool ONICapture::stopRec() {
	if (recorder->isValid()) {
		recorder->stop();
		recorderStarted = false;
		recorder->destroy();
		
		delete recorder; recorder = NULL;
		return 1;
	} else 
		return 0;
}