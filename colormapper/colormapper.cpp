#include "colormapper.h"
#include <fstream>
#include <Windows.h>
#include <thread>

using namespace std;
using namespace pcl;

//vector<bool> ColorMapper::failCamera;
AlgoParams ColorMapper::algoparams;

vector<average_grey> ColorMapper::comp_bw;
vector<average_color> ColorMapper::avgcolors;
vector<normal> ColorMapper::point_normals;
Eigen::SparseMatrix<double>* ColorMapper::bigIdentity;


void callZhk(ColorMapper* cm) {
	cm->mapColorsZhouKoltun();
}

ColorMapper::ColorMapper(QObject* parent) : QObject(parent) 
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

	increase_model = false;
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

//void ColorMapper::setModel(Model::Ptr model) {
//	if (!_started)
//		this->_model = model;
//}
// could not be changed during execution
//void ColorMapper::setThreadsNum(int threads) {
//	if (!_started)
//		this->camerathreads_num = threads;
//}
//void ColorMapper::setDetalisation(bool det) {
//	if (!_started)
//		increase_model = det;
//}

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

void 
ColorMapper::increaseVertexCount() {
	
 // std::vector< pcl::Vertices> polygons;

  new_polygons.clear();
  new_polygons.reserve(mesh_triangles->size() * 4);
  new_polygons.resize(mesh_triangles->size ());

  copy(mesh_triangles->begin(), mesh_triangles->end(), new_polygons.begin());

  PCL_INFO ("Increasing vertex number....\n");
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

//  toPCLPointCloud2(*cloud, triangles.cloud);
//  triangles.polygons = polygons;
}

void
ColorMapper::mapColorsZhouKoltun() {
	cout << "entered mapColorsZhouKoltun\n";

//	bool camera_ratio_diff;
//	if (640.0 / 480.0 != camparams.color_width / camparams.color_height)
//		camera_ratio_diff = true;
//	else 
//		camera_ratio_diff = false;
	frame_params fp = _cameras[0]->fparams;
	float fx_crat = 640.0f / fp.color_width;
	CameraParams camparams(fp.color_fx*fx_crat, fp.color_fy*fx_crat, fp.color_width, fp.color_height, fp.depth_width, fp.depth_height);

	double stepx = (camparams.color_width-1) / (double)(algoparams.gridsizex - 1);
	double stepy = (camparams.color_height-1) / (double)(algoparams.gridsizey - 1);
	algoparams = AlgoParams(stepx, stepy);

	avgcolors.clear();

	TransM.clear();
	bw_images.clear();
	scharrx_images.clear();
	scharry_images.clear();

	comp_bw.resize(mesh_vertices->size());
	camera_point_inds.resize(camera_count);

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
		
	for (int current_cam = 0; current_cam < camera_count; ++current_cam)
	{
		TransM.push_back(new Eigen::Matrix4d(_cameras[current_cam]->pose.matrix().cast<double>()));
		camera_point_inds[current_cam] = new vector<point_bw>();
		if (!_cameras[current_cam]->depth_processed) {
			computeDepthDiscont(&_cameras[current_cam]->depth[0], camparams);
			_cameras[current_cam]->depth_processed = true;
		}
		//	processCamera(current_cam);	
	} // 'end for (cameras)

// -====== IMAGE PYRAMID ======-
	int num_levels = 3;
	int down = 1;
	vector<CameraParams> camparams_lvl;
	vector<AlgoParams> algoparams_lvl;

	for (int level = 0; level < num_levels; ++level) {
		vector<float**> bwim, schxim, schyim;
		CameraParams cp = camparams;
		AlgoParams ap = algoparams;
		
		cp.color_width/=down;
		cp.color_height/=down;

		ap.stepx = (cp.color_width -1) / (double)(algoparams.gridsizex - 1);
		ap.stepy = (cp.color_height-1) / (double)(algoparams.gridsizey - 1);

		for (int current_cam = 0; current_cam < camera_count; ++current_cam)
		{
//			TransM.push_back(new Eigen::Matrix4d(cameras[current_cam]->pose.matrix().cast<double>()));

	// filtering
			QImage img = QImage(_cameras[current_cam]->img.scaled(cp.color_width, cp.color_height, Qt::KeepAspectRatio, Qt::SmoothTransformation));
		//	if (current_cam == 0) {
		//		img.save(QString("scale%1.png").arg(level));
		//	}

			float** bw_img = filterBW(&img);	
			float** scharrx_img = filterArrayScharrX(bw_img, cp.color_width, cp.color_height);
			float** scharry_img = filterArrayScharrY(bw_img, cp.color_width, cp.color_height);

			bwim.push_back(bw_img);
			schxim.push_back(scharrx_img);
			schyim.push_back(scharry_img);

//			camera_point_inds[current_cam] = new vector<point_bw>();
			//	processCamera(current_cam);	
		} // 'end for (cameras)
		bw_images.push_back(bwim);
		scharrx_images.push_back(schxim);
		scharry_images.push_back(schyim);
		camparams_lvl.push_back(cp);
		algoparams_lvl.push_back(ap);
		down <<=1;
	}	
	int current_lvl = 0;

// * подготовка многопоточи€
	for (int i = 0; i < camerathreads_num; ++i) {
		CameraThread* thread = new CameraThread();
		thread->threadId = i;
		connect(thread, &CameraThread::finished, this, &ColorMapper::cameraTaskFinished);
		connect(thread, &CameraThread::error, this, &ColorMapper::cameraTaskError);
		camera_threads.push_back(thread);
	}
// *********** многопоточие

	camerathread_mutex.lock();
		
	while (!free_threads.empty()) 
		free_threads.pop();
	for (int i = 0; i < camerathreads_num; ++i)
		free_threads.push(i);
		
	camerathread_mutex.unlock();
	try {
		int currentcam = 0;
		processed_count = 0;
		// == » вот тут будем строить алгоритм
		while (currentcam < camera_count) {

				camerathread_mutex.lock();
				if (!free_threads.empty()) {
					CameraThread* thread = camera_threads[free_threads.front()];
					free_threads.pop();

					thread->task = CameraTask::PREPROCESS;
					thread->cp = camparams_lvl[current_lvl];
					thread->aparam = algoparams_lvl[current_lvl];
					thread->mesh_triangles = mesh_triangles;
					thread->mesh_vertices = mesh_vertices;
					thread->camera_point_inds = camera_point_inds[currentcam];
					thread->TransM = TransM[currentcam];
					thread->bw = bw_images[current_lvl][currentcam];
					thread->cam = _cameras[currentcam];

					thread->currentcam = currentcam;

					cout << "Thread " << thread->threadId << " is processing camera " << currentcam << endl;
				
					emit message(QString::fromLocal8Bit("Preprocessing %1/%2").arg(currentcam+1).arg(camera_count), (int)((100.0f*(float)currentcam)/camera_count));

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
	} catch (...) {
		cout << "Exception in ColorMapper" << endl;
		hardStop();
		emit error("Colormap", "Unexpected Error in ColorMapper: PreProcessCamera");
		return;
	}

// *********** конец многопоточи€

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
	
	double lambda = 10.0f;// 0.5f*algoparams.stepx;
	
	vector<Eigen::VectorXd*> x(camera_count); 
	
	bigIdentity = new Eigen::SparseMatrix<double>(algoparams.matrixdim, algoparams.matrixdim);
	bigIdentity->reserve(Eigen::VectorXi::Constant(algoparams.matrixdim,1));
	for (int i = algoparams.dof6; i < algoparams.matrixdim; ++i) 
		bigIdentity->insert(i,i) = lambda;
	

	for (int i = 0; i < camera_count; ++i) {
		x[i] = new Eigen::VectorXd();
		*x[i] = Eigen::VectorXd::Zero(algoparams.matrixdim, 1);
	}


// ITERATIONS <===================================================
	//current_lvl = 0;
	cout << "Pyramid level: " << current_lvl << endl;

	cout << "iterations count " << iteration_count << endl;
	for (int iteration = 0; iteration < iteration_count; ++iteration) {
		printf("**** Iteration #%d ****\n\n", iteration);
		if (_stop) return;
		emit message(QString::fromLocal8Bit("Iteration %1").arg(iteration+1), 0);// (100.0f*(float)iteration)/iteration_count);

		// Average C(p)
		for (int i = 0; i< comp_bw.size(); ++i) 
			comp_bw[i].average();
		// ¬ычисление отклонений
		double E = 0.0;
		int cnt_e = 0;
		for (int i = 0; i < camera_count; ++i) {
				for (int j = 0;j < camera_point_inds[i]->size(); ++j) {
					double g = camera_point_inds[i]->at(j).bw;
					if (g >= 0) {
						double r = comp_bw[camera_point_inds[i]->at(j).index].bw - g; 
						E += r*r;
						++cnt_e;
					}
				}
		}
		printf("Average Residual Error: %lf , sqrt: %lf\n", E / (double)cnt_e,  sqrt(E / (double)cnt_e) );
		if (iteration == 0) 
			initialError = sqrt(E / (double)cnt_e);
		if (PrevResid >= sqrt(E / (double)cnt_e)) 
			resid_lower = 0;
		else 
			resid_lower+=1;
		
		PrevResid = sqrt(E / (double)cnt_e);

		if (resid_lower > 2) break;
	
		emit refreshResidualError(initialError, PrevResid);

//		vector<VectorXd*> dx;
// solution to the linear system ()
// * not multithreading
/*		for (int currentcam = 0; currentcam < camera_count; ++currentcam) {

			try {
				emit colorMapperMessage(QString("Iteration %1 - Camera %2").arg(iteration+1).arg(currentcam), 100.0f * (float)currentcam / camera_count);


				Eigen::VectorXd res = VectorXd::Zero(camera_point_inds[currentcam]->size());            // residuals
		
				Eigen::SparseMatrix<double> JrTs(ColorMapper::matrixdim, camera_point_inds[currentcam]->size());
				JrTs.reserve(VectorXi::Constant(camera_point_inds[currentcam]->size(), ColorMapper::dof6 + 8));
		//		ident_mutex.unlock();

				Matrix<double, 1, 2> Gru;                             // Scharr gradient(u)
				Matrix<double, 2, 2> JFu;
				Matrix<double, 2, 4> Jug;                             // Jacobian u(g)
				Matrix<double, 4, 6> Jge;                             // Jacobian g(e) (e == dx)

				vector<point_bw>::iterator it = camera_point_inds[currentcam]->begin();
				for (int ipoint = 0; it != camera_point_inds[currentcam]->end(); ++ipoint, it++) {
					Eigen::Vector4d pnt;
					pnt(0) = mesh_cloud->points[(*it).index].x;
					pnt(1) = mesh_cloud->points[(*it).index].y;
					pnt(2) = mesh_cloud->points[(*it).index].z;
					pnt(3) = 1;

			// * compute r
					int indexp = (*it).index;

					if ((*it).bw >= 0) {
						res(ipoint) = comp_bw[indexp].bw - (*it).bw;
				
		
			// * compute Jge  \/ g(e(al, be, ga, a, b, c))
						Jge = getJge(pnt, *TransM[currentcam], *x[currentcam]) ;
	
			// * compute Jug  \/
						Eigen::Vector4d initG = Gfunc(pnt, eTrotation(*x[currentcam], *TransM[currentcam]));
						Jug = getJug(initG);

			// * compute JFu  \/
						Eigen::Vector2d initU = Ufunc(initG);

						JFu = getJFu(initU, *x[currentcam]);
					
			// * compute Gru  \/
						Eigen::Vector2d initF = Ffunc(initU, *x[currentcam]);

						double step = 1;
						if (initF(0) >= 0 && initF(1) >= 0) {
							Gru(0, 0) = -compute_value(scharrx_images[currentcam], initF(0), initF(1));
							Gru(0, 1) = -compute_value(scharry_images[currentcam], initF(0), initF(1));
						} else {
							Gru(0,0) = Gru(0,1) = 0;
							res(ipoint) = 0;
						}
			// * compute Jr   \/
						MatrixXd row6 = - ((((Gru)*JFu) * Jug) * Jge);			
						for (int i = 0; i < dof6; ++i) JrTs.insert(i, ipoint) = row6(0, i);						

			// * compute F part
						int indf = floor(initU(0) / stepx + eps) + floor(initU(1) / stepy + eps)*gridsizex;
						double 
						fi = Fi(initU,   indf);
						JrTs.insert(dof6 +indf*2, ipoint)   = -fi * Gru(0, 0);  // left up
						JrTs.insert(dof6 +indf*2+1, ipoint) = -fi * Gru(0, 1); 
						if (indf%gridsizex + 1 < gridsizex) {
							fi = Fi(initU,    indf+1);
							JrTs.insert(dof6 +(indf+1)*2  , ipoint) = -fi * Gru(0, 0); // right up
							JrTs.insert(dof6 +(indf+1)*2+1, ipoint) = -fi * Gru(0, 1); 
						}
						if (indf/gridsizex + 1 < gridsizey) {
							fi = Fi(initU,    indf+gridsizex);
							JrTs.insert(dof6 +(indf+gridsizex)*2  ,ipoint) = -fi * Gru(0, 0); // left down
							JrTs.insert(dof6 +(indf+gridsizex)*2+1,ipoint) = -fi * Gru(0, 1); 
							if (indf%gridsizex + 1 < gridsizex) {
								fi = Fi(initU,    indf+gridsizex+1);
								JrTs.insert(dof6 +(indf+gridsizex+1)*2  ,ipoint) = -fi * Gru(0, 0); // right down
								JrTs.insert(dof6 +(indf+gridsizex+1)*2+1,ipoint) = -fi * Gru(0, 1); 
							}
						} 
					} else res(ipoint) = 0;
			// * final Jr Row
				}

				cout  << " solve matrices" << endl; 

			// * compute JrT  \/
			// *              \/
			// * solve (JrT*Jr)*dx = (-JrT*r)

				cout << "matrix A" << endl;

				SparseMatrix<double> A = JrTs*JrTs.transpose();
				A = A + (*bigIdentity);

				cout << "matrix B" << endl;
				MatrixXd B = -JrTs*res;		
			
				cout << "cholesky" << endl;
				Eigen::SimplicialCholesky< SparseMatrix<double> > chol(A);
				cout << "solving" << endl;
			// * Finally solve the system
				dx[currentcam] = new VectorXd(chol.solve(B));
		
				cout << currentcam << ' ';
			} catch (const std::exception& ex) {
				cout << "Camera "<< currentcam << " exception " << ex.what() << endl;
				dx[currentcam] = new VectorXd(VectorXd::Zero(matrixdim));
			} catch (const std::string& ex) {
				cout << "Camera "<< currentcam << " exception " << ex << endl;
				dx[currentcam] = new VectorXd(VectorXd::Zero(matrixdim));
			} catch (...) {
				cout << "Camera "<< currentcam << " exception" << endl;;
				dx[currentcam] = new VectorXd(VectorXd::Zero(matrixdim));
				//throw "Exception in thread!";
			}
		}



		*/
		
// *  MULTITHREADING		
		camerathread_mutex.lock();
		
		while (!free_threads.empty()) 
			free_threads.pop();
		for (int i = 0; i < camerathreads_num; ++i)
			free_threads.push(i);
		
		camerathread_mutex.unlock();
		try {
			int currentcam = 0;
			processed_count = 0;
			// == » вот тут будем строить алгоритм
			while (currentcam < camera_count) {

					camerathread_mutex.lock();
					if (!free_threads.empty()) {
						CameraThread* thread = camera_threads[free_threads.front()];
						free_threads.pop();

						thread->task = CameraTask::ALGORITHM;
						thread->cp = camparams_lvl[current_lvl];
						thread->aparam = algoparams_lvl[current_lvl];
						thread->mesh_vertices = mesh_vertices;
						thread->scharrx = scharrx_images[current_lvl][currentcam];
						thread->scharry = scharry_images[current_lvl][currentcam];
						thread->bw = bw_images[current_lvl][currentcam];
						thread->x = x[currentcam];
						thread->TransM = TransM[currentcam];
						thread->camera_point_inds = camera_point_inds[currentcam];

						thread->currentcam = currentcam;

						cout << "Thread " << thread->threadId << " is processing camera " << currentcam << endl;
				

						emit message(QString::fromLocal8Bit("Iteration %1 - Image %2").arg(iteration+1).arg(currentcam), 100.0f * (float)currentcam / camera_count);
						
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
		} catch (...) {
			cout << "Exception in ColorMapper" << endl;
			hardStop();
			emit error("Colormapper", "Unexpected Error in ColorMapper");
			return;
		}
// * END MULTITHREADING

		cout << endl;
		// Compute C(p) again
		for (int ip = 0; ip < comp_bw.size(); ++ip) comp_bw[ip].clear();
		for (int currentcam = 0; currentcam < camera_point_inds.size(); ++currentcam) {
		/*		*x[currentcam] += *dx[currentcam];
				Matrix4d transf = eTrotation(*x[currentcam], *TransM[currentcam]);*/
				auto it = camera_point_inds[currentcam]->begin();
				for (;it != camera_point_inds[currentcam]->end(); it++) {
				/*	Vector4d pnt;
					pnt(0) = mesh_cloud->points[(*it).index].x;
					pnt(1) = mesh_cloud->points[(*it).index].y;
					pnt(2) = mesh_cloud->points[(*it).index].z;
					pnt(3) = 1;

					Vector4d initG = Gfunc(pnt, transf);
					Vector2d initU = Ufunc(initG);
					Vector2d initF = Ffunc(initU, *x[currentcam]);
					if (initF(0) < 0 || initF(1) < 0 || 
						initF(0) > camparams.color_width - 1 || initF(1) > camparams.color_height - 1)  {
					//	cout << "Bad coordinates : " << initU << endl;
						(*it).bw = -1;
					//	system("pause");
					} else {
						float val = compute_value(bw_images[currentcam], initF(0), initF(1));*/
					if ((*it).bw != -1) {
						comp_bw[it->index].addBW(it->bw);
					//	(*it).bw =  val;
					}		
				}
		}
		cout << "iteration end" << endl;
//		emit iterationFinished(iteration);
		if (iteration % 10 == 9 && current_lvl > 0) {
			double diffx = algoparams_lvl[current_lvl].stepx / algoparams_lvl[current_lvl-1].stepy;
			double diffy = algoparams_lvl[current_lvl].stepy / algoparams_lvl[current_lvl-1].stepy;

			for (int cam = 0; cam < camera_count; ++cam) {
				for (int xdim = algoparams.dof6; xdim < algoparams.matrixdim; ++xdim) {
					if (xdim%2==0) (*x[cam])(xdim) *= diffx;
					else (*x[cam])(xdim) *= diffy;
				}
			}
			current_lvl--;
			resid_lower = 0;
			cout << "Level down: " << current_lvl << endl;
		}

		if (end_iterations) break;
		if (_stop) return;
	}

// =====================
// = = ALGORITHM END = =
// =====================


	// Average C(p)
	for (int i = 0; i < comp_bw.size(); ++i) 
		comp_bw[i].average();
	// ¬ычисление отклонений
	double E = 0.0;
	int cnt_e = 0;
	for (int i = 0; i < camera_count; ++i) {
		for (int j = 0;j < camera_point_inds[i]->size(); ++j) {
			double g = camera_point_inds[i]->at(j).bw;
			if (g >= 0.0) {
				double r = comp_bw[camera_point_inds[i]->at(j).index].bw - g; 
				E += r*r;
				++cnt_e;
			}
		}
	}
	printf("Average Residual Error: %lf , sqrt: %lf\n", E / (double)cnt_e,  sqrt(E / (double)cnt_e) );
	if (iteration_count == 0)
		initialError = sqrt(E / (double)cnt_e);
	
	printf("\nInitial Residual Error: %lf , sqrt: %lf\n", initialError*initialError, initialError);

	optimizedError = sqrt(E / (double)cnt_e);
	
	
	/*
// * final colors *
	std::vector<average_color> avgcolors(mesh_cloud->points.size());
	for (int currentcam = 0; currentcam < camera_point_inds.size(); ++currentcam) {
			Matrix4d transf = eTrotation(*x[currentcam], *TransM[currentcam]);
			vector<point_bw>::iterator it = camera_point_inds[currentcam]->begin();
			for (;it != camera_point_inds[currentcam]->end(); it++) {
				Vector4d pnt;
				pnt(0) = mesh_cloud->points[(*it).index].x;
				pnt(1) = mesh_cloud->points[(*it).index].y;
				pnt(2) = mesh_cloud->points[(*it).index].z;
				pnt(3) = 1;

				Vector4d initG = Gfunc(pnt, transf);
				Vector2d initU = Ufunc(initG);
				Vector2d initF = Ffunc(initU, *x[currentcam]);
				// если не вылезает
				if (!(initF(0) < 0 || initF(1) < 0 || 
					  initF(0) > camparams.color_width - 1 || initF(1) > camparams.color_height - 1))  {
					average_color clr = computeColor(&cameras[currentcam]->img, initF(0), initF(1));
					avgcolors[(*it).index].addRGBFunc(clr.r, clr.g, clr.b, it->weight);
				}		
			}
	} */

// увеличение количества вершин и полигонов
	oldpoints = mesh_vertices->size();
	oldtriangles = mesh_vertices->size();
	if (increase_model) {
			
		// subdividing triangles
		increaseVertexCount();
		cout << "vertex count increased" << endl;
		point_normals.resize(mesh_vertices->size());
		point_color.resize(mesh_vertices->size());

		auto itnewp = mesh_vertices->begin()+oldpoints;
		for (int i = oldpoints; itnewp != mesh_vertices->end(); ++itnewp, ++i) 
			point_normals[i] = point_normals[i] + 
							   point_normals[edge_points[i-oldpoints].first] + 
							   point_normals[edge_points[i-oldpoints].second];

		cout << "new normals computed" << endl;
	}

	avgcolors.resize(mesh_vertices->size());
	
// * многопоточие:
// *  MULTITHREADING		

	camerathread_mutex.lock();
		
	while (!free_threads.empty()) 
		free_threads.pop();
	for (int i = 0; i < camerathreads_num; ++i)
		free_threads.push(i);
		
	camerathread_mutex.unlock();
	try {
		int currentcam = 0;
		processed_count = 0;
		// == » вот тут будем строить алгоритм
		while (currentcam < camera_count) {

				camerathread_mutex.lock();
				if (!free_threads.empty()) {
					CameraThread* thread = camera_threads[free_threads.front()];
					free_threads.pop();

					thread->task = CameraTask::POSTPROCESS;
					thread->cp = camparams;
					thread->aparam = algoparams;
					thread->mesh_triangles = mesh_triangles;
					thread->mesh_vertices = mesh_vertices;
					thread->x = x[currentcam];
					thread->TransM = TransM[currentcam];
					thread->cam = _cameras[currentcam];
					thread->processing_points_size = oldpoints;
					thread->edge_points = &edge_points;
					thread->currentcam = currentcam;

					cout << "Thread " << thread->threadId << " is processing camera " << currentcam << endl;
				
					emit message(QString::fromLocal8Bit("Postprocessing %1/%2").arg(currentcam+1).arg(camera_count), (int)((100.0f*(float)currentcam)/camera_count));

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
	} catch (...) {
		cout << "Exception in ColorMapper" << endl;
		hardStop();
		emit error("Colormapper", "Unexpected Error in ColorMapper: PostProcessCamera");
		return;
	}
// * END MULTITHREADING
	/*
	ofstream fout;
	fout.open("xvec.txt");
	for (int ab = 0; ab < algoparams.matrixdim; ++ab) {
		fout << (*x[0])(ab) << ' ';
	}
	fout.close();*/
// * конец многопоточи€
	
//	for (int currentcam = 0; currentcam < camera_point_inds.size(); ++currentcam) {
//
//		postProcessCamera(currentcam, *x[currentcam]);
//	}

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
		delete TransM[i];
		delete x[i];
		delete camera_point_inds[i];
		for (int lvl = 0; lvl < num_levels; ++lvl) {
			for (int j = 0; j < camparams_lvl[lvl].color_width; ++j) delete [] scharrx_images[lvl][i][j];
			delete [] scharrx_images[lvl][i];
			for (int j = 0; j < camparams_lvl[lvl].color_width; ++j) delete [] scharry_images[lvl][i][j];
			delete [] scharry_images[lvl][i];
			for (int j = 0; j < camparams_lvl[lvl].color_width; ++j) delete [] bw_images[lvl][i][j];
			delete bw_images[lvl][i];
		}
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
ColorMapper::cameraTaskFinished(int camera, int thread, bool success) {
	camerathread_mutex.lock();
	if (success) {

	} else {
		cout << "Computation failed for camera " << camera << " in thread " << thread << " !" << endl;
	}
	if (camera_threads[thread]->isRunning()) {
        while (!camera_threads[thread]->isFinished()) {}
			Sleep(50);
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
