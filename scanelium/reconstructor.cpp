#include "reconstructor.h"
#include "pcl_kinfu2.h"
#include "utils.h"

#if defined(_WIN32) || defined(_WIN64)
	#include <Windows.h>
#endif

#include <Eigen\Core>
#include <Eigen\Geometry>

using namespace Eigen;
using namespace std;

Reconstructor::Reconstructor() : QObject(NULL) {
	qDebug("Reconstructor create");

	_kf = NULL;
	_ip = NULL;

	_started = false;
	_stop = false;
	_has_model = false;
	_processing = false;

	connect(this, &Reconstructor::ready, this, &Reconstructor::update, Qt::QueuedConnection);
}

Reconstructor::~Reconstructor() {
	qDebug("Reconstructor delete");
	
	if (_started) finish(false);

	clearSequences();

	if (_kf != NULL) delete _kf;

	if (_ip != NULL) {
		if (_ip->isRunning())
		{
			_ip->stop();
			_ip->quit();
			_ip->wait();
		}
		delete _ip;
	}
}

bool Reconstructor::init(rec_settings rec_set, cam_settings cam_set) {
	qDebug("Reconstructor::init");

	if (_kf == NULL) 
		_kf = new PCL_Kinfu2();
	if (_ip == NULL)
		_ip = new ImageProcessThread();

	
	_has_model = false;

	this->rec_set = rec_set;
	this->cam_set = cam_set;

	Eigen::Vector3f v_size(rec_set.volume_size, rec_set.volume_size * (rec_set.doubleY ? 2 : 1), rec_set.volume_size);

	_init_pose = Affine3f::Identity();
	switch (rec_set.camera_pose) {
	case CENTER:
	{
		Eigen::Vector3f t = v_size * 0.5f;
		_init_pose = Eigen::Translation3f(t);
	}
	break;
	case CENTERFACE:
	{
		Eigen::Matrix3f R = Eigen::Matrix3f::Identity();   // * AngleAxisf( pcl::deg2rad(-30.f), Vector3f::UnitX());
		Eigen::Vector3f t = v_size * 0.5f - Vector3f(0, 0, v_size(2) / 2 + rec_set.camera_distance);

		_init_pose.rotate(R);
		_init_pose.translate(t);
	}
	break;
	/*
	case CENTEREDGE:
	{
		// TODO проверить
		Eigen::Matrix3f R = Eigen::Matrix3f::Identity() * Eigen::AngleAxisf(pcl::deg2rad(45.f), Vector3f::UnitX());
		Eigen::Vector3f t = Vector3f(v_size(0) / 2, 0, v_size(2) / 2);

		pose = Eigen::Translation3f(t) * Eigen::AngleAxisf(R);
	}
	break;
	case VERTEX:
	{
		Eigen::Vector3f t = Eigen::Vector3f::Zero();
		pose = Eigen::Translation3f(t);
	}
	break;*/
	}
	cout << "init T" << endl;
	cout << _init_pose.translation() << endl;

	float focal_ratio_color = cam_res_color_x[cam_set.color_res] / 640.0f;
	_default_fparams.color_width  = cam_res_color_x[cam_set.color_res];
	_default_fparams.color_height = cam_res_color_y[cam_set.color_res];
	_default_fparams.color_fx = cam_set.fx * focal_ratio_color;
	_default_fparams.color_fy = cam_set.fy * focal_ratio_color;

	float focal_ratio_depth = cam_res_depth_x[cam_set.depth_res] / 640.0f;
	_default_fparams.depth_width  = cam_res_depth_x[cam_set.depth_res];
	_default_fparams.depth_height = cam_res_depth_y[cam_set.depth_res];
	_default_fparams.depth_fx = cam_set.fx * focal_ratio_depth;
	_default_fparams.depth_fy = cam_set.fy * focal_ratio_depth;

	return 
		_kf->init(
		_default_fparams.depth_width, _default_fparams.depth_height,
		_default_fparams.color_width, _default_fparams.color_height,
		_default_fparams.depth_fx, _default_fparams.depth_fy,
		256, 256, 256, 
		v_size.x(), v_size.y(), v_size.z(),
		_init_pose);
}

bool Reconstructor::start() {
	qDebug("Reconstructor::start");

	last_processed_dpt = -1;
	last_processed_rgb = -1;
	current_dpt = -1;
	current_rgb = -1;

	_stop = false;
	_started = true;

	timer.start();
	_ip->clear();
	if (!_ip->isRunning())
		_ip->start();

	emit ready();
	return true;
}

bool Reconstructor::isRunning() {
	return _started;
}

void Reconstructor::update() {
	qDebug("Reconstructor::update");

	_processing = true;
	if (_started) {
		if (!_stop) {
			if (current_dpt > last_processed_dpt) {
				qDebug(QString("Recontructor - process frame %1").arg(current_dpt).toStdString().c_str());
				data_mutex.lock();
				auto dpt = this->last_dpt;
				last_processed_dpt = current_dpt;
				data_mutex.unlock();
				if (_kf->update(&dpt[0])) {
					emit message("Tracking...", 0);
					//if (_kf->hadReset())
					//	emit hadReset();

					Affine3f pose;
					pose = _kf->getPose();
					emit newPose(toQtPose(pose.rotation(), pose.translation()));


					float angle_diff = ((PCL_Kinfu2*)_kf)->getPoseAngleDiff();
					float dist_diff = ((PCL_Kinfu2*)_kf)->getPoseDistDiff();
					emit diffUpdate(angle_diff, dist_diff);

					data_mutex.lock();
					seq_pose.push_back(make_pair(last_processed_dpt, pose));
					data_mutex.unlock();

					//emit newPose(toQtPose(pose.rotation(), pose.translation()));

					vector<uchar> render_bits;
					if (_kf->getImage(render_bits)) {
						QImage render;
						render = QImage(&render_bits[0], cam_res_depth_x[cam_set.depth_res], cam_res_depth_y[cam_set.depth_res], QImage::Format_RGB888).copy();
						//qDebug("Reconstructor - got img");
						//render.save("raycast2.png");
						emit newRendering(render);
					}
					else qDebug("Reconstructor - no img");

					checkSequences();
					if (timer.elapsed() > rec_set.snapshot_rate) {
						Frame* bestFrame = _ip->getBestCam();
						if (bestFrame != NULL) {
							saved_frames.push_back(bestFrame);

							emit framesUpdate(saved_frames.size());
							emit newPose(toQtPose(bestFrame->pose.rotation(), bestFrame->pose.translation()));
						}
						timer.restart();
					}
				}
				else {
					float angle_diff = ((PCL_Kinfu2*)_kf)->getPoseAngleDiff();
					float dist_diff = ((PCL_Kinfu2*)_kf)->getPoseDistDiff();
					emit diffUpdate(angle_diff, dist_diff);

					qDebug("Reconstructor - processing failed");
					emit message("Tracking failed!", 2);
				}
				qDebug("Recontructor - fin processing");
			}
			else {
				qDebug("Recontructor - no frame");
				Sleep(1);
			}
			emit ready();
		}
		if (_stop) _started = false;
	}
	_processing = false;
}

bool Reconstructor::reset() {
	qDebug("Reconstructor::reset");

	if (_started) {

		_kf->reset();
		_ip->clear();

		timer.restart();
		last_processed_dpt = -1;
		last_processed_rgb = -1;
		current_dpt = -1;
		current_rgb = -1;
		clearSequences();

		while (!saved_frames.empty()) {
			delete saved_frames.back();
			saved_frames.pop_back();
		}
		emit framesUpdate(saved_frames.size());

		return true;
	}
	else
		return false;
}

bool Reconstructor::finish(bool extract) {
	qDebug("Reconstructor::finish");

	if (_started) {
		_stop = true;
		if (_processing) {
			while (_started) {
				Sleep(10);
			}
		}
		else _started = false;
		_ip->stop();
	}
	clearSequences();

	_has_model = false;
	if (extract) {
		qDebug("Reconstructor::finish getting model");
		
		//float* vbo; int num_vertices;
		//_has_model = _kf->extractMeshVBO(vbo, num_vertices);
		
		float* vbo; int num_vertices; int* ibo; int num_indices;
		_has_model = _kf->extractMeshIBO(vbo, num_vertices, ibo, num_indices);

		if (_has_model) {
			_model = Model::Ptr(new Model());
			//_model->setIBO(vbo, num_vertices);
			_model->setIBO(vbo, num_vertices, ibo, num_indices);
			qDebug(QString("Got model: %1 vertices, %2 indices").arg(num_vertices).arg(num_indices).toStdString().c_str());

			delete[] vbo;
			delete[] ibo;

			for (int i = 0; i < saved_frames.size(); ++i) {
				_model->addFrame(saved_frames[i]);
			}
			saved_frames.clear();
		}
	}

	while (!saved_frames.empty()) {
		delete saved_frames.back();
		saved_frames.pop_back();
	}
		
	qDebug("Reconstructor::finish finish");

	emit finished(_has_model);
	return true;
}

void Reconstructor::checkSequences() {
	data_mutex.lock();

	while (seq_depth.size() > 30) seq_depth.pop_front();
	while (seq_image.size() > 30) seq_image.pop_front();
	while (seq_pose.size() > 30) seq_pose.pop_front();

	while (seq_depth.size() > 0 && seq_image.size() > 0 && seq_pose.size() > 0) {
		int ind_depth = seq_depth.front().first;
		int ind_image = seq_image.front().first;
		int ind_pose = seq_pose.front().first;
		if (ind_depth == ind_image && ind_image == ind_pose) {
			// create new frame
			Frame* f = new Frame();
			f->img = seq_image.front().second;
			f->depth = seq_depth.front().second;
			f->pose = seq_pose.front().second;
			f->fparams = _default_fparams;

			// process frame
			_ip->process(f);
			
			// pop all parts
			seq_depth.pop_front();
			seq_image.pop_front();
			seq_pose.pop_front();
		}
		else {
			int min_ind = min(ind_depth, min(ind_pose, ind_image));
			if (ind_depth == min_ind) seq_depth.pop_front();
			if (ind_image == min_ind) seq_image.pop_front();
			if (ind_pose == min_ind) seq_pose.pop_front();
		}
	}

	data_mutex.unlock();
}

void Reconstructor::clearSequences() {
	data_mutex.lock();

	seq_depth.clear();
	seq_image.clear();
	seq_pose.clear();

	data_mutex.unlock();
}

bool Reconstructor::getModel(Model::Ptr& m_ptr) {
	if (_has_model) {
		m_ptr = _model;
		return true;
	}
	else return false;
}

void Reconstructor::newDepth(std::vector<unsigned short> new_dpt, int frame_index) {
	data_mutex.lock();

	last_dpt = new_dpt;
	current_dpt = frame_index;

	seq_depth.push_back(make_pair(current_dpt, last_dpt));

	data_mutex.unlock();
}

void Reconstructor::newColor(QImage new_rgb, int frame_index) {
	data_mutex.lock();

	new_rgb = new_rgb.convertToFormat(QImage::Format_RGB888);
	last_rgb = std::vector<uchar>(new_rgb.bits(), new_rgb.bits() + (new_rgb.width()*new_rgb.height() * 3));
	current_rgb = frame_index;

	seq_image.push_back(make_pair(current_rgb, new_rgb));
	
	data_mutex.unlock();
}