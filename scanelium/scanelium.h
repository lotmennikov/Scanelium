#ifndef SCANELIUM_H
#define SCANELIUM_H

#include <QMainWindow>
#include <QtCore/qthread.h>
#include <QtWidgets/qprogressbar.h>
#include "ui_scanelium.h"

#include "controller.h"

class Scanelium : public QMainWindow
{
	Q_OBJECT

public:
	Scanelium(QWidget *parent = 0);
	~Scanelium();

	void closeEvent(QCloseEvent*);

public slots:
	// refresh status at the bottom
	void refreshStatus(QString msg);
	// refresh progres at the bottom
	void showProgress(bool, int);
	// refresh status and progress (deprecated)
	void refreshStatusProgress(QString msg, int progress);
	// show error message box
	void showError(QString title, QString message);
	// show difference between poses (rec)
	void refreshPoseDiff(float angle_diff, float dist_diff);
	// show number of saved frames (rec)
	void refreshFramesNumber(int num);
	// refresh color mapping error (cm)
	void refreshResidualError(double initial, double current);

	// show save file dialog
	void saveDialog();
	// show open file dialog
	void openDialog();
	// confirm switching to other tab or closing
	bool confirmDialog(int switchIndex, int type);
	// open settings dialog
	void openSettings();
	// show program info
	void showInfo();

	void recordingBoxChecked(int checked);
	void tabIndexChanged(int index);	// from ui.scantab
	void stateChanged(int index);		// from controller

	void colorComboIndexChanged(int index);
	void depthComboIndexChanged(int index);
	void poseComboIndexChanged(int index);
	
	void sizeSliderChanged(int value);
	void iterationsSliderChanged(int value);
	void threadsSliderChanged(int value);
	void showSoftStopButton(bool);

private:
	Ui::ScaneliumClass ui;
	QProgressBar* statusProgress;

	Controller* _controller;
	
signals:

};

#endif // SCANELIUM_H
