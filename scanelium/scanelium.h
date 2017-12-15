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

	void refreshStatus(QString msg);
	void refreshStatusProgress(QString msg, int progress);
	void refreshResidualError(double initial, double current);
	void showError(QString title, QString message);

	void saveDialog();
	void openDialog();
	
	void confirmDialog(int switchIndex, int type);
	void openSettings();
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
	void incFramesCount(int count);
	void showSoftStopButton(bool);
	void showProgress(bool, int);

private:
	Ui::ScaneliumClass ui;
	QProgressBar* statusProgress;


	Controller* _controller;
	
signals:

};

#endif // SCANELIUM_H
