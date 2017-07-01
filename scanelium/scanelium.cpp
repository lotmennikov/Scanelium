#include "scanelium.h"
#include <QtCore/qthread.h>
#include <QtCore/qtimer.h>
#include <QtWidgets/qmessagebox.h>
#include <QtCore/qfile.h>
#include <QtWidgets/qfiledialog.h>

#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/ply_io.h>
#include "SettingsDialog.h"
#include "modelio.h"

using namespace std;

Scanelium::Scanelium(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);

	// kinfu and colormapper threads

	kinfuthread = new KinfuController();
	colorMapper = new ColorMapper();
	bool init = kinfuthread->init();
	if (!init) {
		QMessageBox* error_box =new QMessageBox(QString::fromLocal8Bit("Ошибка"),
						QString::fromLocal8Bit("Камера не подключена"),
						QMessageBox::Critical,
						QMessageBox::Ok, 0, 0);

		int n = error_box->exec(); 
		delete error_box; 

		QTimer::singleShot(0, this, SLOT(close()));
	} else {


		connect(kinfuthread, &KinfuController::capturedKinfu, ui.bigViewer, &glWidget::refreshTexture);
		connect(kinfuthread, &KinfuController::gotDepthCloud, ui.bigViewer, &glWidget::refreshCloud);
		connect(kinfuthread, &KinfuController::incImagesCount, this, &Scanelium::incImagesCount);

		connect(kinfuthread, &KinfuController::kinfuMessage, this, &Scanelium::refreshStatus);
		connect(kinfuthread, &KinfuController::gotMesh, this, &Scanelium::meshReady);

		connect(colorMapper, &ColorMapper::colorMapperMessage, this, &Scanelium::refreshStatusProgress);
		connect(colorMapper, &ColorMapper::colorFinished, this, &Scanelium::colorFinished);
		connect(colorMapper, &ColorMapper::refreshResidualError, this, &Scanelium::refreshResidualError);
		connect(colorMapper, &ColorMapper::colorFailed, this, &Scanelium::error);

		kinfuthread->setPriority(QThread::Priority::HighPriority);
		kinfuthread->startCapture();
		kinfuthread->start();

		colorMapper->setPriority(QThread::Priority::HighPriority);

		// interface signals
		connect(ui.startButton, &QPushButton::released, this, &Scanelium::kinfu_begin);
		connect(ui.resetButton, &QPushButton::released, this, &Scanelium::kinfu_reset);
		connect(ui.finishButton, &QPushButton::released, this, &Scanelium::kinfu_stop);
		connect(ui.saveButton, &QPushButton::released, this, &Scanelium::save_file);

		connect(ui.colorButton, &QPushButton::released, this, &Scanelium::startColor);
		connect(ui.softStopButton, &QPushButton::released, this, &Scanelium::softStopColor);
		connect(ui.colorResCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(colorComboIndexChanged(int)));
		connect(ui.depthResCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(depthComboIndexChanged(int)));
		connect(ui.camposeCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(poseComboIndexChanged(int)));
		connect(ui.sizeSlider, &QSlider::valueChanged, this, &Scanelium::sizeSliderChanged);
		connect(ui.doubleYcheckBox, &QCheckBox::stateChanged, this, &Scanelium::doubleYchecked);
		connect(ui.recordCheckBox, &QCheckBox::stateChanged, this, &Scanelium::recordingChecked);
		connect(ui.recordOnlyBox, &QCheckBox::stateChanged, this, &Scanelium::recordOnlyChecked);
		
		connect(ui.iterationsSlider, &QSlider::valueChanged, this, &Scanelium::iterationsSliderChanged);
		connect(ui.threadsSlider, &QSlider::valueChanged, this, &Scanelium::threadsSliderChanged);

		connect(ui.scanTab, &QToolBox::currentChanged, this, &Scanelium::scanTabIndexChanged);

		ui.softStopButton->setVisible(false);

		this->state = ProgramState::INIT;
	
		statusProgress = new QProgressBar(NULL);

		statusProgress->setMaximumHeight(16);
		statusProgress->setMaximumWidth(200);
		statusProgress->setTextVisible(false);
		statusProgress->setMaximum(100);
		statusProgress->setMinimum(0);
		statusProgress->setValue(0);
		ui.statusBar->addPermanentWidget(statusProgress, 0);

		ui.scanTab->setCurrentIndex(0);
	}
	unsaved_model = false;
}

Scanelium::~Scanelium()
{
	if (colorMapper->isRunning())
		colorMapper->hardStop();

	colorMapper->quit();
	colorMapper->wait();
	delete colorMapper;

	kinfuthread->destroy();
	kinfuthread->quit();
	kinfuthread->wait();
	delete kinfuthread;

}

void Scanelium::kinfu_begin() {
//	kinfuthread->initKinfu(1.0f, CameraPose::CENTERFACE);
	if (state == FINAL && unsaved_model) {
		if (!saveDialog()) return;
	}
	if (state == KINFU) { // часто путается
		kinfu_stop();
		return;
	}

	if (!kinfuthread->isRunning()) {
		kinfuthread->startCapture();
		kinfuthread->start();
	}
	kinfuthread->startKinfu();

	this->state = ProgramState::KINFU;
	ui.scansizeLabel->setText(ui.sizeLabel->text());
	ui.bigViewer->state = ProgramState::KINFU;
	ui.scanTab->setCurrentIndex(1);
}

void Scanelium::kinfu_stop() {
	if (state == KINFU) {
		if (!kinfuthread->record_only) {
			kinfuthread->stopKinfu(true);
			kinfuthread->stopCapture();
		}
		else 
			kinfuthread->stopKinfu(false);

	}
}

void Scanelium::kinfu_reset() {
	if (state == KINFU)
		kinfuthread->resetKinfu();
}

void Scanelium::incImagesCount(int count) {
	ui.scanimagesLabel->setText(QString::fromLocal8Bit("Количество изображений: %1").arg(count));
}

void Scanelium::startColor() {
	vector<Camera*> cameras = kinfuthread->cameras;
	if (cameras.size() > 0) {
		colorMapper->init(kinfuthread->camparams);
		pcl::PolygonMesh::Ptr cmesh = pcl::PolygonMesh::Ptr(new pcl::PolygonMesh(*kinfuthread->mesh));
		colorMapper->setMesh(cmesh);
		colorMapper->setIterations(this->ui.iterationsSlider->value());
		colorMapper->setThreadsNum(this->ui.threadsSlider->value());
		colorMapper->setDetalisation(this->ui.detailCheckBox->isChecked());
		colorMapper->setCameras(cameras);
		
		this->refreshStatus(QString::fromLocal8Bit("Вычисление цвета вершин..."));
		ui.softStopButton->setVisible(true);
		ui.residualLabel->setText(QString::fromLocal8Bit("Среднее отклонение: ---"));
		ui.initialResidualLabel->setText(QString::fromLocal8Bit("Исходное отклонение: ---"));

		colorMapper->start();
	}
}

void Scanelium::softStopColor() {
	if (colorMapper->isRunning()) {
		colorMapper->softStop();
	}
}

void Scanelium::stopColor() {
	colorMapper->hardStop();
	colorMapper->quit();
	colorMapper->wait();
}

bool Scanelium::saveDialog() {
	QMessageBox* d_box =
		new QMessageBox(QString::fromLocal8Bit("Сохранить?"),
					QString::fromLocal8Bit("Сохранить модель: "),
					QMessageBox::Information,
					QMessageBox::Yes,
                    QMessageBox::No,
                    QMessageBox::Cancel | QMessageBox::Escape);
	int res = d_box->exec();
	if (res == QMessageBox::Yes) {
		save_file();
		return true;
	} else if (res == QMessageBox::No) {
		return true;
	} else {
		return false;
	}
}

void Scanelium::save_file() {
	try {
		if (this->state == ProgramState::COLOR) {
			QString filename = QFileDialog::getSaveFileName(this, tr("Save Scanelium scan"), ".", tr("Scanelium scans (*.SCL)"));

			if (filename == "") return;
			refreshStatus(QString::fromLocal8Bit("Saving model..."));

			QString path = QFileInfo(filename).absolutePath();
			QString scan_name = QFileInfo(filename).baseName();

			QDir images_path = QDir(path + "/" + scan_name + "/");
			if (!images_path.exists()) images_path.mkdir(path + "/" + scan_name + "/"); 

			cout << "Saving mesh to to '" << scan_name.toStdString() << ".ply'... " << flush;
			pcl::io::savePLYFile(QString(path + "/" + scan_name + ".ply").toStdString(), *kinfuthread->mesh);		
			cout << "Done" << endl;

			refreshStatus(QString::fromLocal8Bit("Saving camera poses..."));

			ModelIO::saveModel(path, scan_name, kinfuthread->camparams, kinfuthread->cameras);

			refreshStatus(QString::fromLocal8Bit("Saved"));



		/*		if (kinfuthread->cameras.size() > 0) {

				for (int i = 0; i < kinfuthread->cameras.size(); ++i) {
					Camera* cam = kinfuthread->cameras[i];
				
					std::ofstream poseFile;
					poseFile.open (QString("%1d.txt").arg(i).toStdString());

					Vector3f teVecs(cam->pose.matrix()(0,3), cam->pose.matrix()(1,3), cam->pose.matrix()(2,3));  
					Matrix3f erreMats;
					for (int j = 0;j < 9; ++j)
						erreMats(j/3, j%3)= cam->pose.matrix()(j/3, j%3);

					poseFile << "TVector" << std::endl << teVecs << std::endl << std::endl 
								<< "RMatrix" << std::endl << erreMats << std::endl << std::endl 
								<< "Camera Intrinsics: focal height width" << std::endl << kinfuthread->FOCAL_LENGTH << " " << 1024 << " " << 1280 << std::endl << std::endl;
					poseFile.close ();
					
					cam->img.save(QString("%1d.png").arg(i));
				}

			}*/
		} 
		if (this->state == ProgramState::FINAL) {
			QString filename = QFileDialog::getSaveFileName(this, tr("Save polygon mesh file"), ".", tr("PLY Files (*.PLY)"));

			if (filename == "") return;
			refreshStatus(QString::fromLocal8Bit("Сохранение..."));

			cout << "Saving colored mesh to to " << filename.toStdString() << "... " << flush;
			pcl::io::savePLYFile(filename.toStdString(), *colorMapper->mesh);		
			cout << "Done" << endl;

			refreshStatus(QString::fromLocal8Bit("Сохранено"));

		}
		unsaved_model = false;
	} catch (...) {
		QMessageBox* error_box =new QMessageBox(QString::fromLocal8Bit("Ошибка"),
						QString::fromLocal8Bit("Ошибка при сохранении"),
						QMessageBox::Critical,
						QMessageBox::Ok, 0, 0);

		int n = error_box->exec(); 
		delete error_box; 
	}
}

void Scanelium::open_file() {
// read mesh from plyfile
    QString filename =  QFileDialog::getOpenFileName(this, tr("Open Scanelium scan file"), "", tr("Scanelium scans (*.SCL)"));

    if (filename == "") return;
	if (state != COLOR && state != FINAL)
		kinfuthread->stopCapture();

	try {
		refreshStatus(QString::fromLocal8Bit("Opening..."));

		QString path = QFileInfo(filename).absolutePath();
		QString scan_name = QFileInfo(filename).baseName();		

		kinfuthread->cameras = vector<Camera*>();
		
		ModelIO::openModel(filename, kinfuthread->camparams, kinfuthread->cameras);

		std::string mesh_name = scan_name.toStdString() + ".ply";
		printf("\nLoading mesh from file %s...\n", mesh_name.c_str());
		kinfuthread->mesh = pcl::PolygonMesh::Ptr(new pcl::PolygonMesh());
		pcl::io::loadPolygonFilePLY((path + "/" + QString::fromStdString(mesh_name)).toStdString(), *kinfuthread->mesh);

		meshReady(true);

	} catch (...) {
		QMessageBox* error_box =new QMessageBox(QString::fromLocal8Bit("Error"),
						QString::fromLocal8Bit("Error while opening file"),
						QMessageBox::Critical,
						QMessageBox::Ok, 0, 0);

		int n = error_box->exec(); 
		delete error_box; 
	}
}

void Scanelium::open_settings() {
	SettingsDialog* dialog = new SettingsDialog(this, kinfuthread->camparams.focal_x, kinfuthread->camparams.snapshot_rate);
	
	if (dialog->exec() == QDialog::Accepted) {
		kinfuthread->camparams.snapshot_rate = dialog->snapshot_rate;
		if (dialog->focal != kinfuthread->camparams.focal_x) {
			if (this->state == KINFU) {
				kinfuthread->stopKinfu(false);
				ui.scanTab->setCurrentIndex(0);
				this->state = ProgramState::INIT;
				ui.bigViewer->state = INIT;
			}
			kinfuthread->camparams.focal_x = kinfuthread->camparams.focal_y = dialog->focal;
		}
	}
	delete dialog;
}

void Scanelium::show_info() {
	QMessageBox* msgbox =
		new QMessageBox(QString::fromLocal8Bit("Информация"),
					QString::fromLocal8Bit("Scanelium \n\nВерсия 1.100 (160901) "),
					QMessageBox::Information,
					QMessageBox::Ok, 0, 0);
	int res = msgbox->exec();
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
	ui.residualLabel->setText(QString::fromLocal8Bit("Среднее отклонение: %1").arg(current));
	ui.initialResidualLabel->setText(QString::fromLocal8Bit("Исходное отклонение: %1").arg(initial));
}

void Scanelium::meshReady(bool isready) {
	if (isready) {
//		this->mesh = kinfuthread->mesh;
		kinfuthread->stopCapture();

		this->state = ProgramState::COLOR;
		ui.bigViewer->state = ProgramState::COLOR;
		ui.bigViewer->setPolygonMesh(kinfuthread->mesh, false);
		ui.scanTab->setCurrentIndex(2);
		this->refreshStatus(QString::fromLocal8Bit("Модель: %1 вершин, %2 полигонов").arg(kinfuthread->mesh->cloud.width).arg(kinfuthread->mesh->polygons.size()));
		this->statusProgress->setVisible(false);

		ui.imagesLabel->setText(QString::fromLocal8Bit("Количество снимков: %1").arg(kinfuthread->cameras.size()));
		ui.residualLabel->setText(QString::fromLocal8Bit("Среднее отклонение: ---"));
		ui.initialResidualLabel->setText(QString::fromLocal8Bit("Исходное отклонение: ---"));
	} else  {
		if (!kinfuthread->record_only) 
			this->refreshStatus(QString::fromLocal8Bit("Ошибка. Модель не получена."));
		
		this->state = ProgramState::INIT;
		ui.bigViewer->state = ProgramState::INIT;
		kinfuthread->startCapture();
		ui.scanTab->setCurrentIndex(0);
	}
}

void Scanelium::colorFinished(bool isready) {
	if (isready) {
//		this->mesh = colorMapper->mesh;
		ui.bigViewer->setPolygonMesh(colorMapper->mesh, true);
		this->state = ProgramState::FINAL;
		ui.bigViewer->state = ProgramState::FINAL;
		ui.softStopButton->setVisible(false);
		ui.residualLabel->setText(QString::fromLocal8Bit("Среднее отклонение: %1").arg(colorMapper->optimizedError));
		ui.initialResidualLabel->setText(QString::fromLocal8Bit("Исходное отклонение: %1").arg(colorMapper->initialError));
//		ui.scanTab->setCurrentIndex(3);
		this->refreshStatus("");
		statusProgress->setVisible(false);

		unsaved_model = true;
	} else  {
		this->refreshStatus(QString::fromLocal8Bit("Ошибка при вычислении"));
		this->state = ProgramState::INIT;
	}
}

void Scanelium::error(std::string message) {
	refreshStatus(QString("Color mapping failed: ") + QString::fromStdString(message));

	QMessageBox* error_box =
		new QMessageBox(QString::fromLocal8Bit("Ошибка"),
					QString::fromLocal8Bit("Ошибка при вычислении цвета: ") + QString::fromStdString(message),
					QMessageBox::Critical,
					QMessageBox::Ok, 0, 0);
            //        QMessageBox::No,
            //        QMessageBox::Cancel | QMessageBox::Escape);
	int n = error_box->exec(); 
	delete error_box; 
	if (n == QMessageBox::Ok)
	{}
}

void Scanelium::scanTabIndexChanged(int index) {
	switch (this->state) {
	case INIT:
		ui.scanTab->setCurrentIndex(0);
		break;
	case KINFU:
		if (index == 0) {
			kinfuthread->stopKinfu(false);
			ui.scanTab->setCurrentIndex(0);
			this->state = ProgramState::INIT;
			ui.bigViewer->state = INIT;
			refreshStatus("");
		} else {
			ui.scanTab->setCurrentIndex(1);
		}
		break;
	case COLOR:
		if (!colorMapper->isRunning()) {
			if (index == 0) {
				this->state = ProgramState::INIT;
				ui.bigViewer->state = INIT;
				kinfuthread->startCapture();
				kinfuthread->start();
				refreshStatus("");
			} else if (index == 1){
				kinfu_begin();
			}
		} else {
			ui.scanTab->setCurrentIndex(2);				
		}
		break;
	case FINAL: // есть модель, но она может заново расцвечиваться
		if (!colorMapper->isRunning()) {
			if (index == 0) {
				if (unsaved_model) {
					if (!saveDialog()) {
						ui.scanTab->setCurrentIndex(2);							
						return;
					}
				}
				this->state = ProgramState::INIT;
				ui.bigViewer->state = ProgramState::INIT; 

				kinfuthread->startCapture();
				kinfuthread->start();
				refreshStatus("");
			} else if (index == 1) {
				if (unsaved_model) {
					if (!saveDialog()) {
						ui.scanTab->setCurrentIndex(2);							
						return;
					}
				}

				kinfu_begin();
			} else if (index == 2) {
				this->state = ProgramState::COLOR;
				ui.bigViewer->state = ProgramState::COLOR; 
			}
		} else {
			ui.scanTab->setCurrentIndex(2);				
		}
		break;
	default:
		break;
	}

}

void Scanelium::colorComboIndexChanged(int index) {
	if (state == ProgramState::INIT) {
		kinfuthread->setColorVideoMode(index);
	}
}

void Scanelium::depthComboIndexChanged(int index){
	if (state == ProgramState::INIT) {
		kinfuthread->setDepthVideoMode(index);
	}
}

void Scanelium::poseComboIndexChanged(int index) {
	if (state == ProgramState::INIT) {
		switch(index) {
		case 0:
			ui.bigViewer->campose = CameraPose::CENTER;
			kinfuthread->setCameraPose(CameraPose::CENTER);
			break;
		case 1:
			ui.bigViewer->campose = CameraPose::CENTERFACE;
			kinfuthread->setCameraPose(CameraPose::CENTERFACE);
			break;
		default:
			ui.bigViewer->campose = CameraPose::CENTER;
			kinfuthread->setCameraPose(CameraPose::CENTER);
			break;
		}
	}
}

void Scanelium::sizeSliderChanged(int value) {
	if (state == ProgramState::INIT) {
		float newsize = value / 10.0f;
		kinfuthread->setVolumeSize(newsize);
		ui.bigViewer->volume_size = newsize;
		ui.sizeLabel->setText(QString::fromLocal8Bit("Размер: куб %1 м").arg(newsize));
	}
}

void Scanelium::doubleYchecked(int checked) {
	if (state == ProgramState::INIT) {
		kinfuthread->setDoubleY(checked);
		ui.bigViewer->doubleY = checked;
	}
}

void Scanelium::recordingChecked(int checked) {
	if (state == ProgramState::INIT) {
		kinfuthread->setRecording(checked);
		if (checked) {
			ui.recordOnlyBox->setEnabled(true); // TODO REPLACE
			kinfuthread->setRecordOnly(ui.recordOnlyBox->isChecked());
		} else {
			ui.recordOnlyBox->setEnabled(false); // TODO REPLACE
		}
	}
}

void Scanelium::recordOnlyChecked(int checked) {
	if (state == ProgramState::INIT) {
		kinfuthread->setRecordOnly(checked);
	}
}


void Scanelium::iterationsSliderChanged(int value) {
	int iterations = value;
	ui.iterationslabel->setText(QString::fromLocal8Bit("Количество итераций: %1").arg(value));
	this->colorMapper->setIterations(iterations);
}

void Scanelium::threadsSliderChanged(int value) {
	int threads = value;
	ui.threadsLabel->setText(QString::fromLocal8Bit("Количество потоков: %1").arg(value));
	this->colorMapper->setThreadsNum(threads);
}
