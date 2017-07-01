#ifndef SCANELIUM_H
#define SCANELIUM_H

#include <QMainWindow>
#include <QtCore/qthread.h>
#include <QtWidgets/qprogressbar.h>
#include "ui_scanelium.h"
#include "KinfuController.h"
#include "colormapper.h"

class Scanelium : public QMainWindow
{
	Q_OBJECT

public:
	Scanelium(QWidget *parent = 0);
	~Scanelium();
	public slots:
	void kinfu_begin();
	void kinfu_stop();
	void kinfu_reset();

	bool saveDialog();
	void save_file();
	void open_file();
	void open_settings();
	void show_info();

	void startColor();
	void softStopColor();
	void stopColor();

public slots:
	void meshReady(bool);
	void colorFinished(bool);
	void refreshStatus(QString msg);
	void refreshStatusProgress(QString msg, int progress);

	void scanTabIndexChanged(int index);

	void doubleYchecked(int checked);
	void recordingChecked(int checked);
	void recordOnlyChecked(int checked);
	void colorComboIndexChanged(int index);
	void depthComboIndexChanged(int index);

	void poseComboIndexChanged(int index);
	void sizeSliderChanged(int value);
	void iterationsSliderChanged(int value);
	void threadsSliderChanged(int value);
	void refreshResidualError(double initial, double current);
	void incImagesCount(int count);

	void error(std::string message);

private:
	Ui::ScaneliumClass ui;
	KinfuController* kinfuthread;
	ColorMapper* colorMapper;
	QProgressBar* statusProgress;

	bool unsaved_model;

//	pcl::PolygonMesh::Ptr mesh;

	ProgramState state;

signals:
	void startCapture();
	void beginKinfu();
	void stopCapture();
};

#endif // SCANELIUM_H
