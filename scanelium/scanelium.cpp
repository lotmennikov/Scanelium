#include "scanelium.h"
#include <QtCore/qthread.h>
#include <QtCore/qtimer.h>
#include <QtWidgets/qmessagebox.h>
#include <QtCore/qfile.h>
#include <QtWidgets/qfiledialog.h>

#include "settingsdialog.h"

using namespace std;

Scanelium::Scanelium(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);

	qRegisterMetaType<QVector<QVector3D>>("QVector<QVector3D>");
	qRegisterMetaType<std::vector<unsigned short>>("std::vector<unsigned short>");
	qRegisterMetaType<rec_settings>("rec_settings");
	qRegisterMetaType<Model::Ptr>("Model::Ptr");

	this->_controller = new Controller();
	connect(_controller, &Controller::recSettingsUpdate, ui.bigViewer, &glWidget::refreshRecSettings);
	connect(_controller, &Controller::cloudUpdate, ui.bigViewer, &glWidget::refreshCloud);
	connect(_controller, &Controller::renderUpdate, ui.bigViewer, &glWidget::refreshTexture);
	connect(_controller, &Controller::meshUpdate, ui.bigViewer, &glWidget::setPolygonMesh);
	connect(_controller, &Controller::poseUpdate, ui.bigViewer, &glWidget::newCameraPose);
	connect(_controller, &Controller::stateChanged, ui.bigViewer, &glWidget::stateChanged);

	connect(_controller, &Controller::statusUpdate, this, &Scanelium::refreshStatus);
	connect(_controller, &Controller::stateChanged, this, &Scanelium::stateChanged);
	connect(_controller, &Controller::framesUpdate, this, &Scanelium::refreshFramesNumber);
	connect(_controller, &Controller::poseDiffUpdate, this, &Scanelium::refreshPoseDiff);
	connect(_controller, &Controller::colormapErrorUpdate, this, &Scanelium::refreshResidualError);
	connect(_controller, &Controller::errorBox, this, &Scanelium::showError);
	connect(_controller, &Controller::askConfirmation, this, &Scanelium::confirmDialog);
	connect(_controller, &Controller::showSoftStopColormap, this, &Scanelium::showSoftStopButton);
	connect(_controller, &Controller::showProgress, this, &Scanelium::showProgress);

// interface signals
	connect(ui.doubleYcheckBox, &QCheckBox::stateChanged, _controller, &Controller::setDoubleY);
	connect(ui.recordCheckBox, &QCheckBox::stateChanged, this, &Scanelium::recordingBoxChecked);
	connect(ui.recordOnlyBox, &QCheckBox::stateChanged, _controller, &Controller::setRecordOnly);
	connect(ui.detailCheckBox, &QCheckBox::stateChanged, _controller, &Controller::setIncreaseModel);

	connect(ui.colorResCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(colorComboIndexChanged(int)));
	connect(ui.depthResCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(depthComboIndexChanged(int)));
	connect(ui.camposeCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(poseComboIndexChanged(int)));
	
	connect(ui.sizeSlider, &QSlider::valueChanged, this, &Scanelium::sizeSliderChanged);
	connect(ui.iterationsSlider, &QSlider::valueChanged, this, &Scanelium::iterationsSliderChanged);
	connect(ui.threadsSlider, &QSlider::valueChanged, this, &Scanelium::threadsSliderChanged);
	connect(ui.scanTab, &QToolBox::currentChanged, this, &Scanelium::tabIndexChanged);

	connect(ui.actionStart, &QAction::triggered, _controller, &Controller::startReconstruction);
	connect(ui.startButton, &QPushButton::clicked, _controller, &Controller::startReconstruction);
	connect(ui.actionReset, &QAction::triggered, _controller, &Controller::resetReconstruction);
	connect(ui.resetButton, &QPushButton::clicked, _controller, &Controller::resetReconstruction);
	connect(ui.actionStop,  &QAction::triggered, _controller, &Controller::stopReconstruction);
	connect(ui.finishButton,&QPushButton::clicked, _controller, &Controller::stopReconstruction);
	connect(ui.saveButton, &QPushButton::clicked, this, &Scanelium::saveDialog);

	connect(ui.actionSettings, &QAction::triggered, this, &Scanelium::openSettings);
	connect(ui.actionOpen, &QAction::triggered, this, &Scanelium::openDialog);
	connect(ui.actionSave, &QAction::triggered, this, &Scanelium::saveDialog);
	connect(ui.actionInfo, &QAction::triggered, this, &Scanelium::showInfo);

	connect(ui.colorButton, &QPushButton::released, _controller, &Controller::startColormap);
	connect(ui.finishButton, &QPushButton::released, _controller, &Controller::stopColormap);
	connect(ui.softStopButton, &QPushButton::released, _controller, &Controller::softStopColormap);
	ui.softStopButton->setVisible(false);

	statusProgress = new QProgressBar(NULL);
	statusProgress->setMaximumHeight(16);
	statusProgress->setMaximumWidth(200);
	statusProgress->setTextVisible(false);
	statusProgress->setMaximum(100);
	statusProgress->setMinimum(0);
	statusProgress->setValue(0);
	ui.statusBar->addPermanentWidget(statusProgress, 0);

	ui.scanTab->setCurrentIndex(0);

	QTimer::singleShot(0, _controller, &Controller::init);
}

Scanelium::~Scanelium()
{
	qDebug("Scanelium delete");
	delete _controller;
}

void Scanelium::saveDialog() {
	if (_controller->getState() != COLOR && _controller->getState() != FINAL) return;

	QString filename;
	if (_controller->getState() == COLOR)
		filename = QFileDialog::getSaveFileName(this, tr("Save"), ".", tr("Scanelium scans (*.scl);;PLY model (*.ply)"));
	else if (_controller->getState() == FINAL)
		filename = QFileDialog::getSaveFileName(this, tr("Save"), ".", tr("PLY model (*.ply);;Scanelium scans(*.scl)"));

	if (filename == "") return;
	else _controller->saveFile(filename);
}

void Scanelium::openDialog() {
	QString filename = QFileDialog::getOpenFileName(this, tr("Open Scanelium scan or ONI file"), "", tr("Scanelium scans (*.scl);; ONI file (*.oni)"));

	if (filename == "") return;
	else _controller->openFile(filename);
}

void Scanelium::closeEvent(QCloseEvent* e) {
	if (_controller->unsavedModel()) {
		if (!confirmDialog(-1, 0))
		{
			e->ignore();
			return;
		}
	}
	e->accept();
}

bool Scanelium::confirmDialog(int index, int type) {
	switch (type) {
	case 0: // save unsaved model dialog
	{
		QMessageBox* d_box =
			new QMessageBox(QString::fromLocal8Bit("Save?"),
				QString::fromLocal8Bit("Save model: "),
				QMessageBox::Information,
				QMessageBox::Yes,
				QMessageBox::No,
				QMessageBox::Cancel | QMessageBox::Escape);
		int res = d_box->exec();
		if (res == QMessageBox::Yes) {
			this->saveDialog();
			if (index != -1)
				_controller->switchTab(index, true);
			else
				return true;
		}
		else if (res == QMessageBox::No) {
			if (index != -1)
				_controller->switchTab(index, true);
			else
				return true;
		}
		else {
			return false;
		}
	}
		break;
	case 1:
		break;
	}
	return true;
}

void Scanelium::openSettings() {
	// TODO show error
	if (_controller->getState() != INIT) return;

	float old_focal = _controller->getFocalX();
	int old_rate = _controller->getSnapshotRate();

	SettingsDialog* dialog = new SettingsDialog(this, old_focal, old_rate);
	connect(dialog, &SettingsDialog::focalChanged, _controller, &Controller::setFocalLength);
	connect(dialog, &SettingsDialog::snapshotChanged, _controller, &Controller::setSnapshotRate);

	if (dialog->exec() == QDialog::Accepted) {
		_controller->setFocalLength(dialog->focal, dialog->focal);
		_controller->setSnapshotRate(dialog->snapshot_rate);
	}
	else {
		_controller->setFocalLength(old_focal, old_focal);
		_controller->setSnapshotRate(old_rate);
	}
	delete	dialog;
}

void Scanelium::showInfo() {
	QMessageBox* msgbox =
		new QMessageBox(QString::fromLocal8Bit("Info"),
			QString::fromLocal8Bit("Scanelium \n\nver 1.200 (160901/171216) "),
			QMessageBox::Information,
			QMessageBox::Ok, 0, 0);
	int res = msgbox->exec();
}

void Scanelium::showSoftStopButton(bool show) {
	ui.softStopButton->setVisible(show);
}

void Scanelium::showProgress(bool show, int val) {
	statusProgress->setVisible(show);
	if (show)
		statusProgress->setValue(val);
}

void Scanelium::refreshFramesNumber(int count) {
	ui.scanimagesLabel->setText(QString::fromLocal8Bit("Number of frames: %1").arg(count));
	ui.imagesLabel->setText(QString::fromLocal8Bit("Number of frames: %1").arg(count));
}

void Scanelium::recordingBoxChecked(int checked) {
	if (checked) {
		_controller->setRecording(true);
		_controller->setRecordOnly(ui.recordOnlyBox->isChecked());
		ui.recordOnlyBox->setEnabled(true);
	} else {
		_controller->setRecording(false);
		ui.recordOnlyBox->setEnabled(false);
	}
}

void Scanelium::refreshStatus(QString msg) {
	ui.statusBar->showMessage(msg);
	cout << "Status: " << msg.toStdString() << endl;
}

void Scanelium::refreshStatusProgress(QString msg, int progress) {
	statusProgress->setVisible(true);
	statusProgress->setValue(progress);
	ui.statusBar->showMessage(msg);
	cout << "Status: " << msg.toStdString() << endl;
}

void Scanelium::refreshResidualError(double initial, double current) {
	ui.residualLabel->setText(QString::fromLocal8Bit("Current std: %1").arg(current));
	ui.initialResidualLabel->setText(QString::fromLocal8Bit("Initial std: %1").arg(initial));
}

void Scanelium::refreshPoseDiff(float angle_diff, float dist_diff) {
	ui.angleLabel->setText(QString::fromLocal8Bit("Angle diff: %1").arg(angle_diff));
	ui.distLabel->setText(QString::fromLocal8Bit("Distance diff: %1").arg(dist_diff));
}

void Scanelium::showError(QString title, QString message) {
	
	QMessageBox* error_box =
		new QMessageBox(title, message,
					QMessageBox::Critical,
					QMessageBox::Ok, 0, 0);
	int n = error_box->exec(); 
	delete error_box; 
	if (n == QMessageBox::Ok) {}
}

void Scanelium::tabIndexChanged(int index) {
	if (!_controller->switchTab(index));
		ui.scanTab->setCurrentIndex(_controller->getState());
}

void Scanelium::stateChanged(int index) {
	qDebug(QString("Scanelium::stateChanged - %1").arg(index).toStdString().c_str());
	
	if (ui.scanTab->currentIndex() != index) {
		switch (index) {
		case ProgramState::INIT:
			ui.scanTab->setItemEnabled(0, true);
			ui.scanTab->setItemEnabled(1, false);
			ui.scanTab->setItemEnabled(2, false);
			ui.scanTab->setItemEnabled(3, false);
			break;
		case ProgramState::KINFU:
			ui.scanTab->setItemEnabled(0, true);
			ui.scanTab->setItemEnabled(1, true);
			ui.scanTab->setItemEnabled(2, false);
			ui.scanTab->setItemEnabled(3, false);
			break;
		case ProgramState::COLOR:
			ui.scanTab->setItemEnabled(0, true);
			ui.scanTab->setItemEnabled(1, true);
			ui.scanTab->setItemEnabled(2, true);
			ui.scanTab->setItemEnabled(3, false);
			break;
		case ProgramState::FINAL:
			ui.scanTab->setItemEnabled(0, true);
			ui.scanTab->setItemEnabled(1, true);
			ui.scanTab->setItemEnabled(2, true);
			ui.scanTab->setItemEnabled(3, true);
			break;
		}
		ui.scanTab->setCurrentIndex(index);
	}
}

void Scanelium::colorComboIndexChanged(int index) {
	_controller->setCamRes(1, index);
}

void Scanelium::depthComboIndexChanged(int index){
	_controller->setCamRes(0, index);
}

void Scanelium::poseComboIndexChanged(int index) {
	_controller->setCameraPose(index);
}

void Scanelium::sizeSliderChanged(int value) {
	float new_size = value / 10.0f;
	_controller->setVolumeSize(new_size);
	ui.sizeLabel->setText(QString::fromLocal8Bit("Size: %1m cube").arg(new_size));
}

void Scanelium::iterationsSliderChanged(int value) {
	int iterations = value;
	_controller->setNumColormapIterations(iterations);
	ui.iterationslabel->setText(QString::fromLocal8Bit("Iterations: %1").arg(value));
}

void Scanelium::threadsSliderChanged(int value) {
	int threads = value;
	_controller->setNumColormapThreads(threads);
	ui.threadsLabel->setText(QString::fromLocal8Bit("Threads: %1").arg(value));
}
