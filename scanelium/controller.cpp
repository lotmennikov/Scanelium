#include "controller.h"
#include "modelio.h"

#include <qfileinfo.h>
#include <qdatetime.h>

#include "utils.h"

Controller::Controller() : QObject(NULL) {
	_state = NONE;

	_cam_set.color_res = ColorResolution::COLOR_VGA;
	_cam_set.depth_res = DepthResolution::DEPTH_VGA;
	_cam_set.fx = 541.316f;
	_cam_set.fy = 541.316f;

	_rec_set.volume_size = 1.0f;
	_rec_set.grid_size = 256;
	_rec_set.doubleY = false;
	_rec_set.recording = false;
	_rec_set.recording_only = false;
	_rec_set.camera_pose = CameraPose::CENTERFACE;
	_rec_set.camera_distance = 0.9f;
	_rec_set.camera_x_angle = 0;
	_rec_set.camera_y_angle = 0;

	_rec_set.snapshot_rate = 1000;
	emit recSettingsUpdate(_rec_set);

	_col_set.increase_model = false;
	_col_set.num_iterations = 1;
	_col_set.num_threads = 4;

	has_model = false;
	unsaved_model = false;

	_oni = new ONICapture();
	connect(this, &Controller::oniStart, _oni, &ONICapture::start);
	connect(_oni, &ONICapture::newDepth, this, &Controller::gotDepthMap);
	connect(_oni, &ONICapture::newColor, this, &Controller::gotColor);
	connect(_oni, &ONICapture::eof, this, &Controller::oniEOF);
	connect(_oni, &ONICapture::error, this, &Controller::oniError);

	_rc = new Reconstructor();
	connect(this, &Controller::reconstructionStart, _rc, &Reconstructor::start);
	connect(this, &Controller::reconstructionReset, _rc, &Reconstructor::reset);
	connect(this, &Controller::reconstructionFinish, _rc, &Reconstructor::finish);
	connect(_rc, &Reconstructor::newRendering, this, &Controller::gotRendering);
	connect(_rc, &Reconstructor::newPose, this, &Controller::poseUpdate);
	connect(_rc, &Reconstructor::framesUpdate, this, &Controller::framesUpdate);
	connect(_rc, &Reconstructor::finished, this, &Controller::reconstructionFinished);
	connect(_rc, &Reconstructor::message, this, &Controller::recMessage);
	connect(_rc, &Reconstructor::diffUpdate, this, &Controller::poseDiffUpdate);

	_cm = new ColorMapper();
	connect(this, &Controller::colormapStart, _cm, &ColorMapper::start);
	connect(_cm, &ColorMapper::refreshResidualError, this, &Controller::colormapErrorUpdate);
	connect(_cm, &ColorMapper::message, this, &Controller::colormapMessage);
	connect(_cm, &ColorMapper::error, this, &Controller::errorBox);
	connect(_cm, &ColorMapper::finished, this, &Controller::colormapFinished);
	connect(_cm, &ColorMapper::renderRequest, this, &Controller::renderRequest);
	connect(this, &Controller::renderFinished, _cm, &ColorMapper::renderFinished);
}

void Controller::init() {
	qDebug("Controller::init");

	_threads[0].start();
	_oni->moveToThread(&_threads[0]);

	if (_oni->initDevice()) {
	} else {
		emit errorBox("Error", "Camera not found");
	}

	_threads[1].start();
	_rc->moveToThread(&_threads[1]);

	_threads[2].start();
	_cm->moveToThread(&_threads[2]);

	setState(ProgramState::INIT);

/*
	// kinfu and colormapper threads
	kinfuthread->setPriority(QThread::Priority::HighPriority);
	colorMapper->setPriority(QThread::Priority::HighPriority);
	*/
}

Controller::~Controller() {
	qDebug("Controller delete");

// delete oni
	if (_oni->isRunning())
		_oni->stop();
	this->_threads[0].quit();
	this->_threads[0].wait();
	delete _oni;

// delete reconstructor
	if (_rc->isRunning())
		_rc->finish(false);
	this->_threads[1].quit();
	this->_threads[1].wait();
	delete _rc;

// delete colormapper
	if (_cm->isRunning())
		_cm->hardStop(true);
	this->_threads[2].quit();
	this->_threads[2].wait();
	delete _cm;
}

int Controller::getState() {
	return _state;
}

bool Controller::unsavedModel() {
	qDebug("Controller unsavedModel");

	return unsaved_model;
}

void Controller::setState(ProgramState st) {
	qDebug("Controller::setState");

	if (this->_state != st) {
		this->_state = st;
		switch (_state) {
		case INIT:
			has_model = false;
			unsaved_model = false;
			if (_oni->isValid() && !_oni->isRunning()) {
				_oni->setDepthMode(_cam_set.depth_res);
				_oni->setColorMode(_cam_set.color_res);

				color_last_index = -1;
				depth_last_index = -1;
				emit oniStart();
			}
			break;
		case KINFU:
			has_model = false;
			unsaved_model = false;
			if (_oni->isValid() && !_oni->isRunning()) {
				_oni->setDepthMode(_cam_set.depth_res);
				_oni->setColorMode(_cam_set.color_res);

				color_last_index = -1;
				depth_last_index = -1;
				emit oniStart();
			}
			if (!_rec_set.recording_only && !_rc->isRunning()) {
				_rc->init(_rec_set, _cam_set);
				_oni->setAutoWhiteBalanceAndExposure(false);
				emit reconstructionStart();
			} 
			if (_rec_set.recording && !_oni->isRecording()) {
				_oni->initRecorder(QDateTime::currentDateTime().toString("yyyy-MM-dd-hh-mm-ss").toStdString() + ".oni");
				_oni->setAutoWhiteBalanceAndExposure(false);
				_oni->startRec();
			}
			break;
		case COLOR: // TODO
			if (_rc->isRunning()) _rc->finish();
			if (_oni->isValid() && _oni->isRunning()) _oni->stop(); // this stops recording too
			break;
		default:
			break;
		}

		emit stateChanged(st);
	}
}

void Controller::generateCloud() {
	if (depth_last_index == -1) return;

	QVector<QVector3D> pnts;
	QVector<QVector3D> colors;

	float divx = 1.0f / _cam_set.fx;
	float divy = 1.0f / _cam_set.fy;

	int color_width = cam_res_color_x[_cam_set.color_res];
	int color_height = cam_res_color_y[_cam_set.color_res];

	int width = cam_res_depth_x[_cam_set.depth_res];
	int height = cam_res_depth_y[_cam_set.depth_res];

	if (width != 640) {
		divx *= 640.0f / width;
		divy *= 640.0f / width;
	}

	float cx = ((float)width - 1) / 2.0f;
	float cy = ((float)height - 1) / 2.0f;

	pnts.reserve(width*height);
	colors.reserve(width*height);

	int depth_idx = 0;
	ushort maxdepth = 6000;

	ushort fardepth = (ushort)((int)(1 << 16) - 10);

	bool has_color = (color_last_index != -1);

	float volume_size = _rec_set.volume_size;
	bool doubleY = _rec_set.doubleY;
	float vs2 = volume_size / 2.0f;
	float depth_to_color = (color_width / (float)width);
	float depth_to_color_shiftY = 
		abs(depth_to_color - (color_height / (float)height)) > 0.00001 ?
		(color_height - (float)height*depth_to_color) / 2.0 - 3.0 : 
		0.0;

	QMatrix4x4 pose = computeCamPose(_rec_set);

	for (int v = 0; v < height; ++v)
	{
		for (int u = 0; u < width; ++u, ++depth_idx)
		{

			unsigned short depth_val = _depth[depth_idx];
			if (depth_val != 0 && depth_val < maxdepth)
			{
				float z = depth_val * 0.001f;
				QVector3D pt(((float)u - cx) * z * divx,
					((float)v - cx) * z * divy,
					z);

				float colval = (1.0f - ((float)(depth_val) / (float)(maxdepth)))*0.8f;
				float green = colval;

				QVector3D pt2 = pose * pt;

				if (pt2.x() > 0 && pt2.x() <= volume_size && pt2.y() > 0 && pt2.y() <= volume_size * (doubleY ? 2 : 1) && pt2.z() > 0 && pt2.z() <= volume_size)
					green = 1.0f;

				QVector3D clr(colval, green, colval);
				if (green == 1.0f) {
					if (has_color) {
						int coordx = (int)floor((float)u * depth_to_color);
						int coordy = (int)floor((float)v * depth_to_color + depth_to_color_shiftY);
						if (coordx < color_width && coordy < color_height) {
							uchar* bits = _color.bits();
							clr.setX(((float)bits[3 * (coordy*color_width + coordx) + 0]) / 255.0f);
							clr.setY(((float)bits[3 * (coordy*color_width + coordx) + 1]) / 255.0f);
							clr.setZ(((float)bits[3 * (coordy*color_width + coordx) + 2]) / 255.0f);
						}
					}
				}

				colors.push_back(clr);
				pnts.push_back(pt);
			}
		}
	}
	emit cloudUpdate(pnts, colors);
}

// ===========
//  ONI SLOTS
// ===========

void Controller::gotDepthMap(std::vector<unsigned short> depth, int width, int height, int index) {
	if (width != cam_res_depth_x[_cam_set.depth_res] || height != cam_res_depth_y[_cam_set.depth_res])
		return;

	_depth = depth;
	depth_last_index = index;
	switch (_state) {
	case INIT:
		if (depth_last_index == color_last_index || depth_last_index > color_last_index + 1)
			generateCloud();
		break;
	case KINFU:
		if (_rc->isRunning()) {
			_rc->newDepth(_depth, index);
		}
		else if (_rec_set.recording_only && _rec_set.recording && _oni->isRecording()) {
			float max_depth = 6000.0f;

			uchar* img_bits = new uchar[3 * width*height];
			for (int i = 0; i < width*height; ++i) {
				img_bits[3 * i + 0] = 255 * (1 - depth[i] / max_depth);
				img_bits[3 * i + 1] = 255 * (1 - depth[i] / max_depth);
				img_bits[3 * i + 2] = 255 * (1 - depth[i] / max_depth);
			}
			QImage rend = QImage(img_bits, width, height, QImage::Format_RGB888).copy();
			delete[] img_bits;

			gotRendering(rend);
		}
		break;
	default:
		break;
	}
}

void Controller::gotColor(QImage img, int width, int height, int index) {
	if (width != cam_res_color_x[_cam_set.color_res] || height != cam_res_color_y[_cam_set.color_res])
		return;

	this->_color = img;
	color_last_index = index;

	if (color_last_index == -1)
		color_last_index = depth_last_index;

	switch (_state) {
	case INIT:
	{
		if (color_last_index == depth_last_index || color_last_index > depth_last_index + 1)
			generateCloud();
	}
	break;

	case KINFU:
	{
		if (_rc->isRunning()) {
			_rc->newColor(_color, index);
		}
	}
	break;
	
	default:
		break;
	}
}

void Controller::oniEOF() {
	switch (_state) {
	case INIT:
		break;
	case KINFU:
		break;
	default:
		break;
	}
}

void Controller::oniError(std::string err) {
	emit errorBox(QString("ONI Error"), QString::fromStdString(err));
}

// ===========
//  REC SLOTS
// ===========

void Controller::startReconstruction() {
	switch (_state) {
	case INIT:
		setState(KINFU);
		break;
	case KINFU: // on/off are combined
		stopReconstruction();
		return;
		break;
	case COLOR: case FINAL:
		if (unsaved_model)
			emit askConfirmation(KINFU, 0);
		else
			setState(KINFU);
		break;
	}
}

void Controller::stopReconstruction() {
	if (_state == KINFU) {
		if (_rc->isRunning()) {
			emit reconstructionFinish(true);
		}
		if (_rec_set.recording && _oni->isRecording()) _oni->stopRec();
		if (_oni->isRunning()) _oni->stop();

		if (_rec_set.recording_only) setState(INIT);
	}
}

void Controller::resetReconstruction() {
	if (_state == KINFU) {
		if (_rc->isRunning())
			emit reconstructionReset();
	}
		
}

void Controller::recMessage(QString msg, int mark) {
	emit statusUpdate(msg);
}

void Controller::gotRendering(QImage render) {
	int width = cam_res_depth_x[_cam_set.depth_res];
	int height= cam_res_depth_y[_cam_set.depth_res];

	uchar* pnt_kinfu = render.bits();
	for (int i = 0; i < width*height; ++i) {
		if (_depth[i] == 0) {
			pnt_kinfu[i * 3] = 255;
			pnt_kinfu[i * 3 + 1] = (uchar)(pnt_kinfu[i * 3 + 1]);
			pnt_kinfu[i * 3 + 2] = (uchar)(pnt_kinfu[i * 3 + 2]);
		}
	}

	emit renderUpdate(render);
}

void Controller::reconstructionFinished(bool has_model) {
	qDebug("Controller::reconstructionFinished");

	if (has_model && _rc->getModel(_model)) {
		this->has_model = true;
		emit meshUpdate(_model);
		emit statusUpdate(QString::fromLocal8Bit("Mesh model: %1 vertices, %2 triangles").arg(_model->getVertsSize()).arg(_model->getIndsSize() / 3));

		setState(COLOR);
	}
	else {
		emit errorBox("Reconstruction failed", "No mesh received");
		setState(INIT);
	}
}

// ===============
//   COLOR SLOTS
// ===============

void Controller::startColormap() {
	if (_cm->isRunning()) return;

	if (!has_model || this->_model->getFramesSize() == 0) return;

	_cm->init(this->_model, this->_col_set);

	emit statusUpdate(QString::fromLocal8Bit("Color mapping..."));
	emit colormapErrorUpdate(std::nan(""), std::nan(""));

	emit colormapStart();

	emit showSoftStopColormap(true);
	emit showProgress(true, 0);
}

void Controller::softStopColormap() {
	if (_cm->isRunning()) {
		_cm->softStop();
	}
}

void Controller::stopColormap() {
	if (_cm->isRunning())
		_cm->hardStop();
}

void Controller::colormapMessage(QString msg, int progress) {
	emit statusUpdate(msg);
	emit showProgress(true, progress);
}

void Controller::colormapFinished(bool ok) {

	if (ok) {
		setState(FINAL);
		emit meshUpdate(_model);
		unsaved_model = true;
		emit statusUpdate(QString::fromLocal8Bit("Color mapping finished"));
	}
	else {
		emit statusUpdate(QString::fromLocal8Bit("Color mapping failed"));
	}
	emit showSoftStopColormap(false);
	emit showProgress(false, 0);
}

void Controller::renderFinished(bool res, std::vector<float> dpt) {
	_cm->renderFinished(res, dpt);
}

// ================
//  PARAMS, UI, IO
// ================

void Controller::setCamRes(int slot, int index) {
	qDebug(QString("Controller setCamRes %1, %2").arg(slot).arg(index).toStdString().c_str());
	if (_oni->isValid()) {
		if (_oni->isRunning())
			_oni->stop();

		if (slot == 0) {// depth 
			_oni->setDepthMode(index);
			_cam_set.depth_res = index;
		}
		if (slot == 1) {
			_oni->setColorMode(index);
			_cam_set.color_res = index;
		}

		color_last_index = -1;
		depth_last_index = -1;

		emit oniStart();
	}
}

void Controller::setVolumeSize(float vsize) {
	if (_state != ProgramState::INIT) return;

	if (vsize >= 0.5f && vsize <= 5.0f) {
		_rec_set.volume_size = vsize;
		emit recSettingsUpdate(_rec_set);
	}
}

void Controller::setGridSize(float gsize) {
	if (_state != ProgramState::INIT) return;

	if (gsize >= 64 && gsize <= 512) {
		_rec_set.grid_size = gsize;
	}
}


void Controller::setCameraPose(int pose) {
	if (_state != ProgramState::INIT) return;

	if (pose == CameraPose::CENTER || pose == CameraPose::CENTERFACE || pose == CameraPose::CUSTOM) {
		_rec_set.camera_pose = pose;
		emit recSettingsUpdate(_rec_set);
	}
	else
		return;
}

void Controller::setCustomPose(float xangle, float yangle, float zdist) {
	if (_state != ProgramState::INIT) return;

	if (_rec_set.camera_pose == CameraPose::CUSTOM) {
		_rec_set.camera_distance = zdist;
		_rec_set.camera_x_angle = xangle;
		_rec_set.camera_y_angle = yangle;

		emit recSettingsUpdate(_rec_set);
	}
}

void Controller::setDoubleY(bool dy) {
	if (_state != ProgramState::INIT) return;
	
	this->_rec_set.doubleY = dy;
	emit recSettingsUpdate(this->_rec_set);
}

void Controller::setRecording(bool rec) {
	if (_state != ProgramState::INIT) return;
	
	_rec_set.recording = rec;
}

void Controller::setRecordOnly(bool ronly) {
	if (_state != ProgramState::INIT) return;
	
	_rec_set.recording_only = ronly;
}

void Controller::setNumColormapIterations(int num) {
	if (_state != ProgramState::COLOR) return;
	this->_col_set.num_iterations = num;
	if (_cm->isRunning()) _cm->setIterations(num);
}

void Controller::setNumColormapThreads(int num) {
	if (_state != ProgramState::COLOR) return;
	this->_col_set.num_threads = num;
}

void Controller::setIncreaseModel(bool inc) {
	if (_state != ProgramState::COLOR) return;
	_col_set.increase_model = inc;
}

void Controller::setFocalLength(float fx, float fy) {
	if ((this->_state == NONE || this->_state == INIT) && fx > 0 && fy > 0) {
		_cam_set.fx = fx, _cam_set.fy = fy;
	}
}

void Controller::setSnapshotRate(int rate) {
	if ((this->_state == NONE || this->_state == INIT)  && rate >= 0) {
		_rec_set.snapshot_rate = rate;
	}
}

bool Controller::switchTab(int index, bool confirmed) {
	if (index == _state)
		return true;

	switch (_state) {
	case INIT:
		return false;
		break;
	case KINFU:
		if (index == 0) {
			if (_rc->isRunning()) _rc->finish(false);
			if (_oni->isRecording()) _oni->stopRec();
			setState(INIT);
			return true;
		}
		else {
			return false;
		}
		break;
	case COLOR:
		if (index == 3) return false;

		if (_cm->isRunning()) {
			if (!confirmed) {
				emit askConfirmation(index, 1);
				return false;
			}
			else {
				_cm->hardStop(true);
				confirmed = false;
			}
		}
		if (unsaved_model && !confirmed) {
			emit askConfirmation(index, 0);
			return false;
		}
		else {
			switch (index) {
			case 0:
				setState(INIT);
				return true;
				break;
			case 1:
				setState(KINFU);
				return true;
				break;
			default:
				return false;
				break;
			}
		}
		break;
	case FINAL: // model may be colored again
		switch (index) {
		case 0:
			if (unsaved_model && !confirmed) {
				emit askConfirmation(index, 0);
				return false;
			}
			else {
				setState(INIT);
				return true;
			}
			break;
		case 1:
			if (unsaved_model && !confirmed) {
				emit askConfirmation(index, 0);
				return false;
			}
			else {
				setState(KINFU);
				return true;
			}
			break;
		case 2:
			setState(COLOR);
			return true;
			break;
		}
	default:
		return false;
		break;
	}
}

void Controller::saveFile(QString filename) {
	QFileInfo f_info(filename);
	if (f_info.suffix().toLower().compare("scl") == 0) { // Save as an SCL scan
		emit statusUpdate(QString("Saving..."));
		if (ModelIO::saveSCL(filename, _model)) {
			emit statusUpdate("Saved");
			unsaved_model = false;
		}
		else {
			emit statusUpdate("Failed");

			emit errorBox("Save file", ModelIO::last_error);
		}
	}
	if (f_info.suffix().toLower().compare("ply") == 0) { // Save as a PLY file
		emit statusUpdate(QString("Saving..."));
		if (ModelIO::saveMesh(filename, _model)) {
			unsaved_model = false;
			emit statusUpdate("Saved");
		}
		else {
			emit statusUpdate("Failed");

			emit errorBox("Save file", ModelIO::last_error);
		}
	}
}

void Controller::openFile(QString filename) {
	QFileInfo f_info(filename);
	if (f_info.suffix().toLower().compare("scl") == 0) {
		emit statusUpdate(QString("Opening..."));
		if (ModelIO::openSCL(filename, _model)) {
			has_model = true;
			unsaved_model = false;

			emit statusUpdate("Loaded scl file");
			emit meshUpdate(_model);
			emit framesUpdate(_model->getFramesSize());
			setState(COLOR);
		} else {
			emit statusUpdate("Failed");

			emit errorBox("Open file", ModelIO::last_error);
		}
	} 
	if (f_info.suffix().toLower().compare("oni") == 0) {
		// TODO
	}
}
