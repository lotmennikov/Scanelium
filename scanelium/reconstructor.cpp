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
	_finish_frames = false;

	this->rec_set = rec_set;
	this->cam_set = cam_set;

	Eigen::Vector3f v_size(rec_set.volume_size, rec_set.volume_size * (rec_set.doubleY ? 2 : 1), rec_set.volume_size);

	_init_pose = computeCamPoseE(rec_set);

	//cout << "init T" << endl;
	//cout << _init_pose.translation() << endl;

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
		rec_set.grid_size, rec_set.grid_size, rec_set.grid_size,
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

	if (rec_set.each_frame) frame_counter = 0; else timer.restart();

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

	data_mutex.lock();
	if (_finish_frames && seq_depth.empty())
	{
		data_mutex.unlock();
		_started = false;
		finish(true);
		return;
	}
	data_mutex.unlock();

	_processing = true;
	if (_started) {
		if (!_stop) {
			if ((rec_set.each_frame && !seq_depth.empty() && seq_depth.front().first > last_processed_dpt) || 
				(!rec_set.each_frame && current_dpt > last_processed_dpt)) {
				
				data_mutex.lock();
				vector<unsigned short> dpt;
				if (rec_set.each_frame) {
					dpt = seq_depth.front().second;
					last_processed_dpt = seq_depth.front().first;
				} else {
					dpt = this->last_dpt;
					last_processed_dpt = current_dpt;
				}
				data_mutex.unlock();
				qDebug(QString("Recontructor - process frame %1").arg(last_processed_dpt).toStdString().c_str());

				if (rec_set.each_frame) ++frame_counter;

				if (_kf->update(&dpt[0])) {
					emit message("Tracking...", 0);

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
					if ((rec_set.each_frame && (1000 * frame_counter) / cam_set.depth_framerate >= rec_set.snapshot_rate) || 
						(!rec_set.each_frame && timer.elapsed() >= rec_set.snapshot_rate)) {
						Frame* bestFrame = _ip->getBestCam();
						if (bestFrame != NULL) {
							saved_frames.push_back(bestFrame);

							emit framesUpdate(saved_frames.size());
							emit newPose(toQtPose(bestFrame->pose.rotation(), bestFrame->pose.translation()));
						}
						if (rec_set.each_frame) frame_counter = 0; 
						else timer.restart();
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
				checkSequences();
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

		if (rec_set.each_frame) frame_counter = 0; else timer.restart();

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

void Reconstructor::finishAllFrames() {
	_finish_frames = true;
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

	if (!rec_set.each_frame) while (seq_depth.size() > 30) seq_depth.pop_front();
	if (!rec_set.each_frame) while (seq_image.size() > 30) seq_image.pop_front();
	while (seq_pose.size() > 30) seq_pose.pop_front();

	while (seq_depth.size() > 0 && seq_image.size() > 0 && seq_pose.size() > 0) {
		int ind_image = seq_image.front().first;
		
		if (ind_image == -1) { // this image cannot be associated with any frame, so it is associated with the latest processed depth
			int ind_pose = seq_pose.back().first; // take latest processed pose and find its depth
			while (!seq_depth.empty() && seq_depth.front().first != ind_pose) seq_depth.pop_front(); // remove all previous frames
			if (seq_depth.empty()) {
				qDebug("Could not find depth frame in the list?..");
				break; // something went wrong
			}
			//int ind_depth = seq_depth.front().first;
			// assumed here (ind_depth == ind_pose) 

			// create new frame
			Frame* f = new Frame();
			f->img = seq_image.front().second;
			f->depth = seq_depth.front().second;
			f->pose = seq_pose.front().second;
			f->fparams = _default_fparams;

			// process frame
			_ip->process(f);

			// pop all parts
			while (!seq_pose.empty()) seq_pose.pop_front();	// all previous frames are deprecated
			seq_depth.pop_front();							// but depth is still needed
			seq_image.pop_front();
		}
		else {
			int ind_depth = seq_depth.front().first;
			int ind_pose = seq_pose.front().first;
			if (ind_depth == ind_pose && ind_image == ind_pose) {
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
				int min_ind = min(min(ind_depth, ind_pose), ind_image);
				if (ind_depth == min_ind && (!rec_set.each_frame || ind_pose == ind_depth)) {
					qDebug(QString("Pop depth %1").arg(seq_depth.front().first).toStdString().c_str());
					seq_depth.pop_front();
				}
				if (ind_image == min_ind) seq_image.pop_front();
				if (ind_pose == min_ind) seq_pose.pop_front();
			}
		}
	}

	// prevent dead lock
	if (rec_set.each_frame && _finish_frames && seq_image.empty()) {
		while (!seq_pose.empty() &&
			!seq_depth.empty() &&
			seq_depth.front().first <= seq_pose.front().first) {

			if (seq_depth.front().first == seq_pose.front().first)
				seq_pose.pop_front();
			seq_depth.pop_front();

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

void Reconstructor::clearModel() {
	_has_model = false;
	_model.reset();
	clearSequences();
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
	if (frame_index != -1) {
		current_rgb = frame_index;
		seq_image.push_back(make_pair(current_rgb, new_rgb));
	}
	else {
		if (this->rec_set.each_frame) {
			current_rgb = current_dpt;
			// insert if it comes after depth,  and there is no frame with the same index
			if (current_rgb != -1 && (seq_image.empty() || seq_image.back().first != current_rgb)) 
				seq_image.push_back(make_pair(current_rgb, new_rgb));
		}
		else {
			current_rgb = -1;
			while (!seq_image.empty()) seq_image.pop_back(); // only one must stay
			seq_image.push_back(make_pair(-1, new_rgb));
		}
	}
	
	data_mutex.unlock();
}