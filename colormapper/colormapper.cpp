#include "colormapper.h"
#include <fstream>
#include <Windows.h>
#include <thread>
#include "utils.h"
#include "filters.h"

using namespace std;

vector<average_grey> ColorMapper::comp_bw;
vector<average_color> ColorMapper::avgcolors;
vector<normal> ColorMapper::point_normals;
Eigen::SparseMatrix<double>* ColorMapper::bigIdentity;


void callZhk(ColorMapper* cm) {
	cm->mapColorsZhouKoltun();
}

ColorMapper::ColorMapper() : QObject(NULL) 
{
	iteration_count = 1;
	_started = false;
	_stop = false;
	camerathreads_num = 4;

	zhk_thread = NULL;
	mesh_vertices = NULL;
	mesh_triangles = NULL;
}

ColorMapper::~ColorMapper(void) {
	qDebug("ColorMapper delete");

	if (_started) hardStop(true);

	if (zhk_thread != NULL) {
		zhk_thread->join();
		delete zhk_thread;
	}
	if (mesh_vertices != NULL) delete mesh_vertices;
	if (mesh_triangles != NULL) delete mesh_triangles;
}

void ColorMapper::init(Model::Ptr model, colormap_settings set) {
	if (_started) return;

	//increase_model = false;
	this->_model = model;
	this->_col_set = set;
	this->increase_model = _col_set.increase_model;
	this->camerathreads_num = _col_set.num_threads;
	this->iteration_count = _col_set.num_iterations;
}

// may be changed during execution
void ColorMapper::setIterations(int it_count) {
//	if (!started)
		this->iteration_count = it_count;
}

// RUN / STOP

void ColorMapper::start() {
	if (_started) return;

	_stop = false;
	end_iterations = false;
	_started = true;

	run();
}

bool ColorMapper::isRunning() {
	return _started;
}

// make current iteration the last
void ColorMapper::softStop() {
	if (!_started) return;

	end_iterations = true;
}

// terminate execution and all threads
void ColorMapper::hardStop(bool wait) {
	if (!_started) return;

	_stop = true;

	if (wait) {
		while (_started) {
			Sleep(10);
		}
	}
}

void ColorMapper::cancelThreads() {
	if (camera_threads.size() > 0) {
		while (!free_threads.empty())
			free_threads.pop();
		for (int i = 0; i < camera_threads.size(); ++i) {
			if (camera_threads[i]->isRunning()) {
				camera_threads[i]->quit();
				camera_threads[i]->wait();
			}
			delete camera_threads[i];
		}
	}
}

void ColorMapper::run() {
	if (!_started) return;
	if (_model->getFramesSize() > 0) {
		//	if (camerathread_mutex)

		if (mesh_vertices == NULL) mesh_vertices = new vector<Model::PointXYZ>();
		if (mesh_triangles == NULL) mesh_triangles = new vector<Model::Triangle>();

		_model->copyVertices(*mesh_vertices);
		_model->copyTriangles(*mesh_triangles);
		_model->copyFrames(this->_cameras);
		camera_count = this->_cameras.size();
		// computing colors
		point_color = vector<Model::ColorRGB>(mesh_vertices->size());

		// TEST
	//	increase_model = false;

		zhk_thread = new std::thread(callZhk, this);
	}
	else {
		emit error("Colormapper", "No images");
		_started = false;
	}
}

// ALGORITHM

void 
ColorMapper::increaseVertexCount() {
	
  new_polygons.clear();
  new_polygons.reserve(mesh_triangles->size() * 4);
  new_polygons.resize(mesh_triangles->size ());

  copy(mesh_triangles->begin(), mesh_triangles->end(), new_polygons.begin());

  printf("Increasing vertex number....\n");
  float nx, ny, nz;

  int newp_ind = mesh_vertices->size();
  
  mesh_vertices->reserve((mesh_vertices->size() * 4) / 3 + 1);
//  triangles.polygons.reserve(triangles.polygons.size() * 2);
  vector< vector<pair<int, int> > > npoints(oldpoints);
  edge_points.clear();
  edge_points.reserve(oldpoints*3);

  int tr_count = new_polygons.size();
  int newindex = oldpoints; // current new point index;
  for (int i = 0; i < tr_count; ++i) {
	 // сделать 3 точки и 4 тр-ка

	  Model::Triangle vert = new_polygons[i];
	  int ind0 = vert.p[0];
	  int ind1 = vert.p[1];
	  int ind2 = vert.p[2];

	  int newpi0, newpi1, newpi2;
	  int found;
// first
	  found = 0;
	  if (ind0 < ind1) {
		  for (int j = 0; j < npoints[ind0].size(); ++j) if (npoints[ind0][j].first == ind1) { found = npoints[ind0][j].second; break; }
	  } else {
		  for (int j = 0; j < npoints[ind1].size(); ++j) if (npoints[ind1][j].first == ind0) { found = npoints[ind1][j].second; break; }
	  }
	  if (!found) {
		  Model::PointXYZ newp0;
		  newp0.x = (mesh_vertices->operator[](ind0).x + mesh_vertices->operator[](ind1).x) / 2;
		  newp0.y = (mesh_vertices->operator[](ind0).y + mesh_vertices->operator[](ind1).y) / 2;
		  newp0.z = (mesh_vertices->operator[](ind0).z + mesh_vertices->operator[](ind1).z) / 2;
	  
		  mesh_vertices->push_back(newp0);
		  edge_points.push_back(make_pair(ind0, ind1));
		  newpi0 = newindex;
		  ++newindex;

		  if (ind0 < ind1)
			  npoints[ind0].push_back(make_pair(ind1, newpi0));
		  else
			  npoints[ind1].push_back(make_pair(ind0, newpi0));

	  } else newpi0 = found;
// second
	  found = 0;
	  if (ind1 < ind2) {
		  for (int j = 0; j < npoints[ind1].size(); ++j) if (npoints[ind1][j].first == ind2) { found = npoints[ind1][j].second; break; }
	  } else {
		  for (int j = 0; j < npoints[ind2].size(); ++j) if (npoints[ind2][j].first == ind1) { found = npoints[ind2][j].second; break; }
	  }
	  if (!found) {
		  Model::PointXYZ newp1;
		  newp1.x = (mesh_vertices->operator[](ind1).x + mesh_vertices->operator[](ind2).x) / 2;
		  newp1.y = (mesh_vertices->operator[](ind1).y + mesh_vertices->operator[](ind2).y) / 2;
		  newp1.z = (mesh_vertices->operator[](ind1).z + mesh_vertices->operator[](ind2).z) / 2;
	  
		  mesh_vertices->push_back(newp1);
		  edge_points.push_back(make_pair(ind1, ind2));
		  newpi1 = newindex;
		  ++newindex;

		  if (ind1 < ind2)
			  npoints[ind1].push_back(make_pair(ind2, newpi1));
		  else
			  npoints[ind2].push_back(make_pair(ind1, newpi1));

	  } else newpi1 = found;
// third
	  found = 0;
	  if (ind2 < ind0) {
		  for (int j = 0; j < npoints[ind2].size(); ++j) if (npoints[ind2][j].first == ind0) { found = npoints[ind2][j].second; break; }
	  } else {
		  for (int j = 0; j < npoints[ind0].size(); ++j) if (npoints[ind0][j].first == ind2) { found = npoints[ind0][j].second; break; }
	  }
	  if (!found) {
		  Model::PointXYZ newp2;
		  newp2.x = (mesh_vertices->operator[](ind2).x + mesh_vertices->operator[](ind0).x) / 2;
		  newp2.y = (mesh_vertices->operator[](ind2).y + mesh_vertices->operator[](ind0).y) / 2;
		  newp2.z = (mesh_vertices->operator[](ind2).z + mesh_vertices->operator[](ind0).z) / 2;
	  
		  mesh_vertices->push_back(newp2);
		  edge_points.push_back(make_pair(ind2, ind0));
		  newpi2 = newindex;
		  ++newindex;

		  if (ind2 < ind0)
			  npoints[ind2].push_back(make_pair(ind0, newpi2));
		  else
			  npoints[ind0].push_back(make_pair(ind2, newpi2));

	  } else newpi2 = found;
// --------

	  Model::Triangle newt;
	  newt.p[0] = newpi2;
	  newt.p[1] = ind0;
	  newt.p[2] = newpi0;
	  new_polygons[i] = newt;

	  newt.p[0] = newpi0;
	  newt.p[1] = ind1;
	  newt.p[2] = newpi1;
	  new_polygons.push_back(newt);

	  newt.p[0] = newpi1;
	  newt.p[1] = ind2;
	  newt.p[2] = newpi2;
	  new_polygons.push_back(newt);

	  newt.p[0] = newpi0;
	  newt.p[1] = newpi1;
	  newt.p[2] = newpi2;
	  new_polygons.push_back(newt);
  }

  newpoints = mesh_vertices->size();
  newtriangles = new_polygons.size();
}

void 
ColorMapper::computePointNormals() {
	// COMPUTING NORMALS
	point_normals.clear();
	point_normals.resize(mesh_vertices->size());
	auto it = mesh_triangles->begin();
	for (int idx_face = 0; it != mesh_triangles->end(); ++it, ++idx_face) //static_cast<int> (mesh.tex_polygons[current_cam].size ()); ++idx_face)
	{
		float A = mesh_vertices->operator[](it->p[0]).y * (mesh_vertices->operator[](it->p[1]).z - mesh_vertices->operator[](it->p[2]).z) + mesh_vertices->operator[](it->p[1]).y * (mesh_vertices->operator[](it->p[2]).z - mesh_vertices->operator[](it->p[0]).z) + mesh_vertices->operator[](it->p[2]).y * (mesh_vertices->operator[](it->p[0]).z - mesh_vertices->operator[](it->p[1]).z);
		float B = mesh_vertices->operator[](it->p[0]).z * (mesh_vertices->operator[](it->p[1]).x - mesh_vertices->operator[](it->p[2]).x) + mesh_vertices->operator[](it->p[1]).z * (mesh_vertices->operator[](it->p[2]).x - mesh_vertices->operator[](it->p[0]).x) + mesh_vertices->operator[](it->p[2]).z * (mesh_vertices->operator[](it->p[0]).x - mesh_vertices->operator[](it->p[1]).x);
		float C = mesh_vertices->operator[](it->p[0]).x * (mesh_vertices->operator[](it->p[1]).y - mesh_vertices->operator[](it->p[2]).y) + mesh_vertices->operator[](it->p[1]).x * (mesh_vertices->operator[](it->p[2]).y - mesh_vertices->operator[](it->p[0]).y) + mesh_vertices->operator[](it->p[2]).x * (mesh_vertices->operator[](it->p[0]).y - mesh_vertices->operator[](it->p[1]).y);

		point_normals[it->p[0]].x += A;
		point_normals[it->p[0]].y += B;
		point_normals[it->p[0]].z += C;

		point_normals[it->p[1]].x += A;
		point_normals[it->p[1]].y += B;
		point_normals[it->p[1]].z += C;

		point_normals[it->p[2]].x += A;
		point_normals[it->p[2]].y += B;
		point_normals[it->p[2]].z += C;
	}
}


void
ColorMapper::renderPose(vector<float>& dpt, Eigen::Matrix4f pose, iparams ip) {
	render_return = false;
	emit renderRequest(toQtPose(pose), ip);

	while (true) {
		render_mutex.lock();
		if (render_return) break;
		render_mutex.unlock();
		Sleep(1);
	}
	render_mutex.unlock();

	dpt = render_result;
}

void
ColorMapper::mapColorsZhouKoltun() {
	cout << "entered mapColorsZhouKoltun\n";

	// setup camera and algorithm params
	frame_params fp = _cameras[0]->fparams;
	float fx_crat = 640.0f / fp.color_width;
	iparams color_iparams(fp.color_fx, fp.color_fy, fp.color_width, fp.color_height);

	AlgoParams algoparams;
	double stepx = (color_iparams.width - 1) / (double)(algoparams.gridsizex - 1);
	double stepy = (color_iparams.height - 1) / (double)(algoparams.gridsizey - 1);
	algoparams = AlgoParams(stepx, stepy);

	// imgs for each camera
	avgcolors.clear();
	bw_images.clear();
	scharrx_images.clear();
	scharry_images.clear();
	camdata.clear(); camdata.resize(camera_count);

	comp_bw.resize(mesh_vertices->size());
	camera_point_inds.resize(camera_count);

	std::vector<std::vector<float>> dpt_render;

	// point normals
	computePointNormals();

	for (int current_cam = 0; current_cam < camera_count; ++current_cam)
	{
		Eigen::Matrix4d Mtcam = _cameras[current_cam]->pose.matrix().cast<double>(); Mtcam = Mtcam.inverse().eval();

		camdata[current_cam] = new img_data();
		camdata[current_cam]->TransM = Mtcam;

		camera_point_inds[current_cam] = new vector<point_bw>();

		{ // rendering
			vector<float> dpt;
			renderPose(dpt, _cameras[current_cam]->pose.matrix(), color_iparams);
			dpt_render.push_back(dpt);
			camdata[current_cam]->render = &dpt_render.back()[0];

			computeWeightsFromDepth(camdata[current_cam]->render, camdata[current_cam]->dpt_weights, color_iparams);

			// TEST
			if (false) {
				uchar* img = new uchar[3 * color_iparams.width*color_iparams.height];
				for (int i = 0; i < color_iparams.width*color_iparams.height; ++i)
				{
					float val = camdata[current_cam]->dpt_weights[i];
					img[i * 3] = val * 255;
					img[i * 3 + 1] = val * 255;
					img[i * 3 + 2] = val * 255;
				}
				QImage ni2 = QImage(img, color_iparams.width, color_iparams.height, QImage::Format_RGB888).copy();
				ni2.save(QString("test-depth2.png"));
			}
		}

	}

	// -====== IMAGE PYRAMID ======-
	int num_levels = 3;
	int down = 1;
	vector<AlgoParams> algoparams_lvl;
	vector<iparams> cip_lvl;

	for (int level = 0; level < num_levels; ++level) {
		vector<float*> bwim, schxim, schyim;
		AlgoParams ap = algoparams;
		iparams cip = color_iparams;

		cip = cip / down;

		ap.stepx = (cip.width - 1) / (double)(algoparams.gridsizex - 1);
		ap.stepy = (cip.height - 1) / (double)(algoparams.gridsizey - 1);

		for (int current_cam = 0; current_cam < camera_count; ++current_cam)
		{
			// filtering
			QImage img = QImage(_cameras[current_cam]->img.scaled(cip.width, cip.height, Qt::KeepAspectRatio, Qt::SmoothTransformation));
			//	if (current_cam == 0) {
			//		img.save(QString("scale%1.png").arg(level));
			//	}

			float* bw_img = filterBW(&img);
			float* scharrx_img = filterArrayScharrX(bw_img, cip.width, cip.height);
			float* scharry_img = filterArrayScharrY(bw_img, cip.width, cip.height);

			bwim.push_back(bw_img);
			schxim.push_back(scharrx_img);
			schyim.push_back(scharry_img);

			//			camera_point_inds[current_cam] = new vector<point_bw>();
						//	processCamera(current_cam);	
		} // 'end for (cameras)
		bw_images.push_back(bwim);
		scharrx_images.push_back(schxim);
		scharry_images.push_back(schyim);
		algoparams_lvl.push_back(ap);
		cip_lvl.push_back(cip);
		down <<= 1;
	}
	current_lvl = 0;

	for (int current_cam = 0; current_cam < camera_count; ++current_cam) {
		camdata[current_cam]->ip = cip_lvl[current_lvl];

		camdata[current_cam]->bw = bw_images[current_lvl][current_cam];
		camdata[current_cam]->scharrx = scharrx_images[current_lvl][current_cam];
		camdata[current_cam]->scharry = scharry_images[current_lvl][current_cam];
	}

	// prepare threads
	for (int i = 0; i < camerathreads_num; ++i) {
		CameraThread* thread = new CameraThread();
		thread->threadId = i;
		connect(thread, &CameraThread::finished, this, &ColorMapper::cameraTaskFinished);
		connect(thread, &CameraThread::error, this, &ColorMapper::cameraTaskError);
		camera_threads.push_back(thread);
	}

	// *********** begin multithread
	multithread(PREPROCESS, algoparams_lvl[current_lvl], cip_lvl[current_lvl]);
	if (_stop) { cancelThreads(); _started = false; emit finished(false);  return; }

	// ***********   end multithread

	printf("camera_points:\n");
	for (int i = 0; i < camera_count; ++i) {
		printf("#%d: %d points\n", i, (int)(camera_point_inds[i]->size()));
	}

	// =======================
	// = = ALGORITHM BEGIN = =
	// =======================

	//	int iteration_count = 50;

	using namespace Eigen;
	initialError = 1;
	double PrevResid = 1;
	int resid_lower = 0;

	double lambda = 1.0f;// 0.5f*algoparams.stepx;

	x.resize(camera_count);

	bigIdentity = new Eigen::SparseMatrix<double>(algoparams.matrixdim, algoparams.matrixdim);
	bigIdentity->reserve(Eigen::VectorXi::Constant(algoparams.matrixdim, 1));
	for (int i = algoparams.dof6; i < algoparams.matrixdim; ++i)
		bigIdentity->insert(i, i) = lambda;


	for (int i = 0; i < camera_count; ++i) {
		x[i] = new Eigen::VectorXd();
		*x[i] = Eigen::VectorXd::Zero(algoparams.matrixdim, 1);
	}


	// ITERATIONS <===================================================
		//current_lvl = 0;
	cout << "Pyramid level: " << current_lvl << endl;

	cout << "iterations count " << iteration_count << endl;

	for (iteration = 0; iteration < iteration_count; ++iteration) {
		if (end_iterations) break;
		if (_stop) { cancelThreads(); _started = false; emit finished(false); return; }

		printf("**** Iteration #%d ****\n\n", iteration);

		emit message(QString::fromLocal8Bit("Iteration %1").arg(iteration + 1), 0);// (100.0f*(float)iteration)/iteration_count);

		// Average C(p)
		for (int i = 0; i < comp_bw.size(); ++i)
			comp_bw[i].average();
		// Вычисление отклонений
		double E = 0.0;
		int cnt_e = 0;
		for (int i = 0; i < camera_count; ++i) {
			for (int j = 0; j < camera_point_inds[i]->size(); ++j) {
				double g = camera_point_inds[i]->at(j).bw;
				if (g >= 0) {
					double r = comp_bw[camera_point_inds[i]->at(j).index].bw - g;
					E += r*r;
					++cnt_e;
				}
			}
		}
		printf("Average Residual Error: %lf , sqrt: %lf\n", E / (double)cnt_e, sqrt(E / (double)cnt_e));
		if (iteration == 0)
			initialError = sqrt(E / (double)cnt_e);
		if (PrevResid >= sqrt(E / (double)cnt_e))
			resid_lower = 0;
		else
			resid_lower += 1;

		PrevResid = sqrt(E / (double)cnt_e);

		if (resid_lower > 2) break;

		emit refreshResidualError(initialError, PrevResid);

		// *  MULTITHREADING	
		multithread(CameraTask::ALGORITHM, algoparams_lvl[current_lvl], cip_lvl[current_lvl]);
		if (_stop) { cancelThreads(); _started = false; emit finished(false); return; }
		// * END MULTITHREADING

		cout << endl;
		// Compute C(p) again
		for (int ip = 0; ip < comp_bw.size(); ++ip) comp_bw[ip].clear();
		for (int currentcam = 0; currentcam < camera_point_inds.size(); ++currentcam) {
			auto it = camera_point_inds[currentcam]->begin();
			for (; it != camera_point_inds[currentcam]->end(); it++) {
				if ((*it).bw != -1) {
					comp_bw[it->index].addBW(it->bw);
					//	(*it).bw =  val;
				}
			}
		}
		cout << "iteration end" << endl;
		//		emit iterationFinished(iteration);
		if (iteration % 10 == 9 && current_lvl > 0) {
			double diffx = algoparams_lvl[current_lvl].stepx / algoparams_lvl[current_lvl - 1].stepy;
			double diffy = algoparams_lvl[current_lvl].stepy / algoparams_lvl[current_lvl - 1].stepy;

			for (int cam = 0; cam < camera_count; ++cam) {
				for (int xdim = algoparams.dof6; xdim < algoparams.matrixdim; ++xdim) {
					if (xdim % 2 == 0) (*x[cam])(xdim) *= diffx;
					else (*x[cam])(xdim) *= diffy;
				}
			}
			current_lvl--;
			resid_lower = 0;
			cout << "Level down: " << current_lvl << endl;
		}
	}

	// =====================
	// = = ALGORITHM END = =
	// =====================


		// Average C(p)
	for (int i = 0; i < comp_bw.size(); ++i)
		comp_bw[i].average();
	// Вычисление отклонений
	double E = 0.0;
	int cnt_e = 0;
	for (int i = 0; i < camera_count; ++i) {
		for (int j = 0; j < camera_point_inds[i]->size(); ++j) {
			double g = camera_point_inds[i]->at(j).bw;
			if (g >= 0.0) {
				double r = comp_bw[camera_point_inds[i]->at(j).index].bw - g;
				E += r*r;
				++cnt_e;
			}
		}
	}
	printf("Average Residual Error: %lf , sqrt: %lf\n", E / (double)cnt_e, sqrt(E / (double)cnt_e));
	if (iteration_count == 0)
		initialError = sqrt(E / (double)cnt_e);

	printf("\nInitial Residual Error: %lf , sqrt: %lf\n", initialError*initialError, initialError);

	optimizedError = sqrt(E / (double)cnt_e);

	// увеличение количества вершин и полигонов
	oldpoints = mesh_vertices->size();
	oldtriangles = mesh_vertices->size();
	if (increase_model) {

		// subdividing triangles
		increaseVertexCount();
		cout << "vertex count increased" << endl;
		point_normals.resize(mesh_vertices->size());
		point_color.resize(mesh_vertices->size());

		auto itnewp = mesh_vertices->begin() + oldpoints;
		for (int i = oldpoints; itnewp != mesh_vertices->end(); ++itnewp, ++i)
			point_normals[i] = point_normals[i] +
			point_normals[edge_points[i - oldpoints].first] +
			point_normals[edge_points[i - oldpoints].second];

		cout << "new normals computed" << endl;
	}

	avgcolors.resize(mesh_vertices->size());


// REPEAT DEPTH RENDERING
	for (int current_cam = 0; current_cam < camera_count; ++current_cam)
	{
		Matrix4f new_pose = camdata[current_cam]->TransM.inverse().cast<float>();
		vector<float> dpt;
		renderPose(dpt, new_pose, color_iparams);
		dpt_render[current_cam] = dpt;
		camdata[current_cam]->render = &dpt_render[current_cam][0];

		delete[] camdata[current_cam]->dpt_weights; // old weights
		computeWeightsFromDepth(camdata[current_cam]->render, camdata[current_cam]->dpt_weights, color_iparams);
	}

// *  MULTITHREADING		
	multithread(CameraTask::POSTPROCESS, algoparams, color_iparams);
	if (_stop) { cancelThreads(); _started = false; emit finished(false); return; }
// * END MULTITHREADING
	/*
	ofstream fout;
	fout.open("xvec.txt");
	for (int ab = 0; ab < algoparams.matrixdim; ++ab) {
		fout << (*x[0])(ab) << ' ';
	}
	fout.close();*/

	for (int i = 0; i < point_color.size(); ++i) {
		if (avgcolors[i].count == 0) {
			avgcolors[i].setRGB(0.3,0.3,0.3);
		} else {
			avgcolors[i].average();
		}
		point_color[i].r = avgcolors[i].r;
		point_color[i].g = avgcolors[i].g;
		point_color[i].b = avgcolors[i].b;

	}

// очистка
	avgcolors.clear();
	avgcolors.reserve(1);
	point_normals.clear();
	point_normals.reserve(1);

// garbage collecting
	for (int i = 0; i < camera_count; ++i) {
		delete x[i];
		delete camera_point_inds[i];
		for (int lvl = 0; lvl < num_levels; ++lvl) {
			delete [] scharrx_images[lvl][i];
			delete [] scharry_images[lvl][i];
			delete [] bw_images[lvl][i];
		}
		delete[] camdata[i]->dpt_weights;
		delete camdata[i];
	}

	delete bigIdentity;

	for (int i = 0; i < camerathreads_num; ++i) 
		delete camera_threads[i];
	camera_threads.clear();
	while (!free_threads.empty()) free_threads.pop();

	// add

	if (!_stop) {
		if (increase_model) {
			_model->setIBO((float*)&mesh_vertices->operator[](0), mesh_vertices->size(), (int*)&new_polygons[0], newtriangles * 3);
		}
		// combining in a new model
		_model->setColors((float*)&point_color[0], point_color.size());

		delete mesh_vertices; mesh_vertices = NULL;
		delete mesh_triangles; mesh_triangles = NULL;

		// mesh ready
		emit finished(true);
	}
	else {
		cancelThreads();
		emit finished(false);
	}
	
	_started = false;

}// 'end ZhouKoltun

void 
ColorMapper::multithread(CameraTask task, AlgoParams ap, iparams cip) {
	camerathread_mutex.lock();

	while (!free_threads.empty())
		free_threads.pop();
	for (int i = 0; i < camerathreads_num; ++i)
		free_threads.push(i);

	camerathread_mutex.unlock();
	try {
		int currentcam = 0;
		processed_count = 0;

		// Process all cameras
		while (currentcam < camera_count) {

			camerathread_mutex.lock();
			if (!free_threads.empty()) {
				CameraThread* thread = camera_threads[free_threads.front()];
				free_threads.pop();

				thread->task = task;
				thread->aparam = ap;
				thread->currentcam = currentcam;
				thread->cam = _cameras[currentcam];
				thread->camdata = camdata[currentcam];
				thread->mesh_vertices = mesh_vertices;

				if (task == PREPROCESS) {
					thread->mesh_triangles = mesh_triangles;
					thread->camera_point_inds = camera_point_inds[currentcam];
				}
				if (task == ALGORITHM) {
					thread->camera_point_inds = camera_point_inds[currentcam];
					thread->x = x[currentcam];
				}
				if (task == POSTPROCESS) {
					thread->mesh_triangles = mesh_triangles;
					thread->x = x[currentcam];
					thread->processing_points_size = oldpoints;
					thread->edge_points = &edge_points;
				}

				cout << "Thread " << thread->threadId << " is processing camera " << currentcam << endl;

				switch (task) {
				case PREPROCESS:
					emit message(QString::fromLocal8Bit("Preprocessing %1/%2").arg(currentcam + 1).arg(camera_count), (int)((100.0f*(float)currentcam) / camera_count));
					break;
				case ALGORITHM:
					emit message(QString::fromLocal8Bit("Iteration %1 - Image %2").arg(iteration + 1).arg(currentcam), 100.0f * (float)currentcam / camera_count);
					break;
				case POSTPROCESS:
					emit message(QString::fromLocal8Bit("Postprocessing %1/%2").arg(currentcam + 1).arg(camera_count), (int)((100.0f*(float)currentcam) / camera_count));
					break;
				}

				currentcam++;

				thread->start();
			}
			camerathread_mutex.unlock();

			Sleep(10);
			if (_stop) return;
		}

		//waiting...
		while (processed_count < camera_count) {
			Sleep(10);
			if (_stop) return;
		}
	}
	catch (...) {
		cout << "Exception in ColorMapper" << endl;
		hardStop();
		emit error("Colormapper", "Unexpected Error in ColorMapper: PostProcessCamera");
		return;
	}
}

// SLOTS

void 
ColorMapper::cameraTaskFinished(int camera, int thread, bool success) {
	camerathread_mutex.lock();
	if (success) {

	} else {
		cout << "Computation failed for camera " << camera << " in thread " << thread << " !" << endl;
	}
	if (camera_threads[thread]->isRunning()) {
        while (!camera_threads[thread]->isFinished()) {}
			Sleep(5);
	}
	free_threads.push(thread);
	processed_count++;
	camerathread_mutex.unlock();
}

void 
ColorMapper::cameraTaskError(int thread, string msg) {
	hardStop();
	emit error("Colormapper: cameraTask", QString::fromStdString(msg));
}

void
ColorMapper::renderFinished(bool res, vector<float> dpt) {
	render_mutex.lock();
	render_result = dpt;
	render_return = true;
	render_mutex.unlock();
}