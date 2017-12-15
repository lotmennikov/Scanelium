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
#include "camparam.h"

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

	rec_settings _rec_set;
	cam_settings _cam_set;
	colormap_settings _col_set;

	void generateCloud();
	void setState(ProgramState);

public:

	Controller();
	~Controller();

	void init();

	int getState();
	bool unsavedModel();
	float getFocalX() { return _cam_set.fx; }
	int getSnapshotRate() { return _rec_set.snapshot_rate; }

public slots:

// UI slots
	void setCamRes(int slot, int res_index);
	void setVolumeSize(float vsize);
	void setCameraPose(int pose);
	void setDoubleY(bool);
	void setRecording(bool);
	void setRecordOnly(bool);
	void setNumColormapIterations(int);
	void setNumColormapThreads(int);
	void setIncreaseModel(bool);
	
	void setFocalLength(float fx, float fy);
	void setSnapshotRate(int rate);

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
	void gotRendering(QImage img);
	void reconstructionFinished(bool has_model);

// colormap slots
	void colormapMessage(QString msg, int progress);
	void colormapFinished(bool);

signals:
	// ONI
	void oniStart();

	// REC
	void reconstructionStart();
	void reconstructionReset();
	void reconstructionFinish(bool extract_model);

	// COLORMAP
	void colormapStart();

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
	void meshUpdate(Model::Ptr);
	void poseUpdate(QMatrix4x4);
	void framesUpdate(int count);
	void colormapErrorUpdate(double, double);

	//void startCapture();
	//void stopCapture();
};