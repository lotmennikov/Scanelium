#pragma once

#include <qobject.h>
#include <qthread.h>
#include <qstring.h>
#include <qimage.h>
#include <qvector3d.h>

#include <vector>
#include <string>

#include "reconstructor.h"
#include "onicapture.h"
#include "colormapper.h"

class Controller : public QObject {
	
	Q_OBJECT

private:
	QThread _threads[3];

	ProgramState _state;

	Reconstructor* _rc;
	ONICapture* _oni;
	ColorMapper* _cm;

	Model::Ptr _model;
	bool has_model;
	bool unsaved_model;

	QImage _color;
	std::vector<unsigned short> _depth;
	int depth_last_index;
	int color_last_index;

	// reconstruction parameters
	rec_settings _rec_set;
	// camera parameters
	cam_settings _cam_set;
	// color mapping parameters
	colormap_settings _col_set;

	// convert depth map to point cloud and pass to viewer
	void generateCloud();
	// internal program state switching
	void setState(ProgramState);
public:
	// in: renderer for colormapper
	Controller();
	~Controller();

	// prepare camera and threads
	void init();
	// current program state
	int getState();
	// has unsaved colored mesh model
	bool unsavedModel();
	// current focal length
	float getFocalX() { return _cam_set.fx; }
	// current snapshot rate
	int getSnapshotRate() { return _rec_set.snapshot_rate; }

public slots:

// UI slots
	// set camera resolution
	void setCamRes(int slot, int res_index);
	// set size of scene
	void setVolumeSize(float vsize);
	// set size of voxel grid
	void setGridSize(float gsize);
	// set initial camera pose
	void setCameraPose(int pose);
	// set params of custom pose
	void setCustomPose(float xangle, float yangle, float zdist);
	// double volume height
	void setDoubleY(bool);
	// record ONI
	void setRecording(bool);
	// record ONI, do not reconstruct
	void setRecordOnly(bool);
	// set number of colormap iterations
	void setNumColormapIterations(int);
	// set number of CPU threads
	void setNumColormapThreads(int);
	// inrease detalization (more mesh vertices)
	void setIncreaseModel(bool);
	// set camera focal lenghts
	void setFocalLength(float fx, float fy);
	// set rate of saving color images
	void setSnapshotRate(int rate);
	// allow UI tab switching
	bool switchTab(int index, bool confirmed = false);

	void startReconstruction();
	void stopReconstruction();
	void resetReconstruction();

	void startColormap();
	void softStopColormap();
	void stopColormap();
//	void continueColor();

	void openFile(QString filename);
	void saveFile(QString filename);

// oni slots
	void gotDepthMap(std::vector<unsigned short>, int, int, int);
	void gotColor(QImage, int, int, int);
	void oniEOF();
	void oniError(std::string);

// rec slots
	void recMessage(QString msg, int mark);
	void gotRendering(QImage img);
	void reconstructionFinished(bool has_model);

// colormap slots
	void colormapMessage(QString msg, int progress);
	void colormapFinished(bool);
	void renderFinished(bool res, std::vector<float> render);

signals:
	// ONI
	void oniStart();

	// REC
	void reconstructionStart();
	void reconstructionReset();
	void reconstructionFinish(bool extract_model);

	// COLORMAP
	void colormapStart();
	void renderRequest(QMatrix4x4 pose, iparams ip);

	// UI
	void askConfirmation(int index, int);
	void stateChanged(ProgramState);
	void errorBox(QString, QString);
	void showSoftStopColormap(bool);
	void showProgress(bool, int);
	
	//  UPDATES
	void statusUpdate(QString);
	void recSettingsUpdate(rec_settings);
	void cloudUpdate(QVector<QVector3D>, QVector<QVector3D>);
	void renderUpdate(QImage);
	void poseDiffUpdate(float, float);
	void meshUpdate(Model::Ptr);
	void poseUpdate(QMatrix4x4);
	void framesUpdate(int count);
	void colormapErrorUpdate(double, double);
};