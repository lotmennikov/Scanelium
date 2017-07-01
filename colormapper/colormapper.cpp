#include "colormapper.h"
#include <fstream>
#include <Windows.h>

using namespace std;
using namespace pcl;

//vector<bool> ColorMapper::failCamera;

CameraParams ColorMapper::camparams;			
AlgoParams ColorMapper::algoparams;

vector<average_grey> ColorMapper::comp_bw;
vector<average_color> ColorMapper::avgcolors;
vector<normal> ColorMapper::point_normals;
Eigen::SparseMatrix<double>* ColorMapper::bigIdentity;

ColorMapper::ColorMapper(QObject* parent) : QThread(parent) 
{
	iteration_count = 1;
	started = false;
	stop = false;
	camerathreads_num = 4;

}

ColorMapper::~ColorMapper(void) {
}

void ColorMapper::init(CameraParams cparams) {
	camparams = cparams;
	increase_model = false;
}

void ColorMapper::setMesh(PolygonMesh::Ptr mesh) {
	if (!started)
		this->mesh = mesh;
}

void ColorMapper::setCameras(vector<Camera*> cameras) {
	if (!started) {
		this->cameras = cameras;
		this->camera_count = cameras.size();
	}
}

// may be changed during execution
void ColorMapper::setIterations(int it_count) {
//	if (!started)
		this->iteration_count = it_count;
}

// could not be changed during execution
void ColorMapper::setThreadsNum(int threads) {
	if (!started)
		this->camerathreads_num = threads;
}

void ColorMapper::setDetalisation(bool det) {
	if (!started)
		increase_model = det;
}

// make current iteration the last
void ColorMapper::softStop() {
	end_iterations = true;
}

// terminate execution and all threads
void ColorMapper::hardStop() {
	stop = true;
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
	started = false;
}

void ColorMapper::run() {
	if (started) return;
	started = true;
	stop = false;
	end_iterations = false;
//	if (camerathread_mutex)
	mesh_cloud = pcl::PointCloud<PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(mesh->cloud, *mesh_cloud);

	// computing colors
	point_color =  PointCloud<RGB>::Ptr (new PointCloud<RGB>);
	point_color->points.resize(mesh_cloud->size());

	// TEST
//	increase_model = false;

	mapColorsZhouKoltun();

	if (increase_model) {
		mesh->polygons = new_polygons;
	}

	// combining in a new model
	PointCloud<PointXYZRGB>::Ptr colorcloud(new PointCloud<PointXYZRGB>());
	pcl::copyPointCloud(*mesh_cloud, *colorcloud);      
	for (size_t i = 0; i < point_color->size(); ++i)
		colorcloud->points[i].rgba = point_color->points[i].rgba;

	toPCLPointCloud2(*colorcloud, mesh->cloud);
	// mesh ready
	
	started = false;
	
	emit colorFinished(true);
}

void 
ColorMapper::increaseVertexCount() {
	
 // std::vector< pcl::Vertices> polygons;

  new_polygons.clear();
  new_polygons.reserve(mesh->polygons.size() * 4);
  new_polygons.resize(mesh->polygons.size ());

  copy(mesh->polygons.begin(), mesh->polygons.end(), new_polygons.begin());

  PCL_INFO ("Increasing vertex number....\n");
  float nx, ny, nz;

  int newp_ind = mesh_cloud->size();
  
  mesh_cloud->points.reserve((mesh_cloud->points.size() * 4) / 3 + 1);
//  triangles.polygons.reserve(triangles.polygons.size() * 2);
  vector< vector<pair<int, int> > > npoints(oldpoints);
  edge_points.clear();
  edge_points.reserve(oldpoints*3);

  int tr_count = new_polygons.size();
  int newindex = oldpoints; // current new point index;
  for (int i = 0; i < tr_count; ++i) {
	 // сделать 3 точки и 4 тр-ка

	  pcl::Vertices vert = new_polygons[i];
	  int ind0 = vert.vertices[0];
	  int ind1 = vert.vertices[1];
	  int ind2 = vert.vertices[2];

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
		  pcl::PointXYZ newp0 = pcl::PointXYZ((mesh_cloud->points[ind0].x + mesh_cloud->points[ind1].x)/2, 
											  (mesh_cloud->points[ind0].y + mesh_cloud->points[ind1].y)/2,
											  (mesh_cloud->points[ind0].z + mesh_cloud->points[ind1].z)/2);	     
	  
		  mesh_cloud->points.push_back(newp0);
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
		  pcl::PointXYZ newp1 = pcl::PointXYZ((mesh_cloud->points[ind1].x + mesh_cloud->points[ind2].x)/2, 
											  (mesh_cloud->points[ind1].y + mesh_cloud->points[ind2].y)/2,
											  (mesh_cloud->points[ind1].z + mesh_cloud->points[ind2].z)/2);	     
	  
		  mesh_cloud->points.push_back(newp1);
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
		  pcl::PointXYZ newp2 = pcl::PointXYZ((mesh_cloud->points[ind2].x + mesh_cloud->points[ind0].x)/2, 
											  (mesh_cloud->points[ind2].y + mesh_cloud->points[ind0].y)/2,
											  (mesh_cloud->points[ind2].z + mesh_cloud->points[ind0].z)/2);	     
	  
		  mesh_cloud->points.push_back(newp2);
		  edge_points.push_back(make_pair(ind2, ind0));
		  newpi2 = newindex;
		  ++newindex;

		  if (ind2 < ind0)
			  npoints[ind2].push_back(make_pair(ind0, newpi2));
		  else
			  npoints[ind0].push_back(make_pair(ind2, newpi2));

	  } else newpi2 = found;
// --------

	  pcl::Vertices newt;
	  newt.vertices.push_back(newpi2);
	  newt.vertices.push_back(ind0);
	  newt.vertices.push_back(newpi0);
	  new_polygons[i] = pcl::Vertices(newt);

	  newt.vertices.clear();
	  newt.vertices.push_back(newpi0);
	  newt.vertices.push_back(ind1);
	  newt.vertices.push_back(newpi1);
	  new_polygons.push_back(newt);

	  newt.vertices.clear();
	  newt.vertices.push_back(newpi1);
	  newt.vertices.push_back(ind2);
	  newt.vertices.push_back(newpi2);
	  new_polygons.push_back(newt);

	  newt.vertices.clear();
	  newt.vertices.push_back(newpi0);
	  newt.vertices.push_back(newpi1);
	  newt.vertices.push_back(newpi2);
	  new_polygons.push_back(newt);
  }
  mesh_cloud->width = mesh_cloud->points.size();

  newpoints = mesh_cloud->points.size();
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

	double stepx = (camparams.color_width-1) / (double)(algoparams.gridsizex - 1);
	double stepy = (camparams.color_height-1) / (double)(algoparams.gridsizey - 1);
	algoparams = AlgoParams(stepx, stepy);

	avgcolors.clear();

	TransM.clear();
	bw_images.clear();
	scharrx_images.clear();
	scharry_images.clear();

	comp_bw.resize(mesh_cloud->points.size());
	camera_point_inds.resize(camera_count);

	// COMPUTING NORMALS
	point_normals.clear();
	point_normals.resize(mesh_cloud->points.size());
    auto it = mesh->polygons.begin();
	for (int idx_face = 0; it != mesh->polygons.end(); ++it, ++idx_face) //static_cast<int> (mesh.tex_polygons[current_cam].size ()); ++idx_face)
	{
		float A = mesh_cloud->points[it->vertices[0]].y * (mesh_cloud->points[it->vertices[1]].z - mesh_cloud->points[it->vertices[2]].z) + mesh_cloud->points[it->vertices[1]].y * (mesh_cloud->points[it->vertices[2]].z - mesh_cloud->points[it->vertices[0]].z) + mesh_cloud->points[it->vertices[2]].y * (mesh_cloud->points[it->vertices[0]].z - mesh_cloud->points[it->vertices[1]].z); 
		float B = mesh_cloud->points[it->vertices[0]].z * (mesh_cloud->points[it->vertices[1]].x - mesh_cloud->points[it->vertices[2]].x) + mesh_cloud->points[it->vertices[1]].z * (mesh_cloud->points[it->vertices[2]].x - mesh_cloud->points[it->vertices[0]].x) + mesh_cloud->points[it->vertices[2]].z * (mesh_cloud->points[it->vertices[0]].x - mesh_cloud->points[it->vertices[1]].x); 
		float C = mesh_cloud->points[it->vertices[0]].x * (mesh_cloud->points[it->vertices[1]].y - mesh_cloud->points[it->vertices[2]].y) + mesh_cloud->points[it->vertices[1]].x * (mesh_cloud->points[it->vertices[2]].y - mesh_cloud->points[it->vertices[0]].y) + mesh_cloud->points[it->vertices[2]].x * (mesh_cloud->points[it->vertices[0]].y - mesh_cloud->points[it->vertices[1]].y);

		point_normals[it->vertices[0]].x += A;
		point_normals[it->vertices[0]].y += B;
		point_normals[it->vertices[0]].z += C;

		point_normals[it->vertices[1]].x += A;
		point_normals[it->vertices[1]].y += B;
		point_normals[it->vertices[1]].z += C;

		point_normals[it->vertices[2]].x += A;
		point_normals[it->vertices[2]].y += B;
		point_normals[it->vertices[2]].z += C;
	}
		
	for (int current_cam = 0; current_cam < camera_count; ++current_cam)
	{
		TransM.push_back(new Eigen::Matrix4d(cameras[current_cam]->pose.matrix().cast<double>()));
		camera_point_inds[current_cam] = new vector<point_bw>();
		if (!cameras[current_cam]->depth_processed) {
			computeDepthDiscont(cameras[current_cam]->depth);
			cameras[current_cam]->depth_processed = true;
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
			QImage img = QImage(cameras[current_cam]->img.scaled(cp.color_width, cp.color_height, Qt::KeepAspectRatio, Qt::SmoothTransformation));
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

// * подготовка многопоточия
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
		// == И вот тут будем строить алгоритм
		while (currentcam < camera_count) {

				camerathread_mutex.lock();
				if (!free_threads.empty()) {
					CameraThread* thread = camera_threads[free_threads.front()];
					free_threads.pop();

					thread->task = CameraTask::PREPROCESS;
					thread->cp = camparams_lvl[current_lvl];
					thread->aparam = algoparams_lvl[current_lvl];
					thread->cloud = mesh_cloud;
					thread->mesh = mesh;
					thread->camera_point_inds = camera_point_inds[currentcam];
					thread->TransM = TransM[currentcam];
					thread->bw = bw_images[current_lvl][currentcam];
					thread->cam = cameras[currentcam];

					thread->currentcam = currentcam;

					cout << "Thread " << thread->threadId << " is processing camera " << currentcam << endl;
				
					emit colorMapperMessage(QString::fromLocal8Bit("Предобработка %1/%2").arg(currentcam+1).arg(camera_count), (int)((100.0f*(float)currentcam)/camera_count));

					currentcam++;
				
					thread->start();				
				}
				camerathread_mutex.unlock();

				Sleep(10);
				if (stop) return;
		}

		//waiting...
		while (processed_count < camera_count) {
			Sleep(10);
			if (stop) return;
		}
	} catch (...) {
		cout << "Exception in ColorMapper" << endl;
		hardStop();
		emit colorFailed("Unexpected Error in ColorMapper: PreProcessCamera");
		return;
	}

// *********** конец многопоточия

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
		if (stop) return;
		emit colorMapperMessage(QString::fromLocal8Bit("Итерация %1").arg(iteration+1), 0);// (100.0f*(float)iteration)/iteration_count);

		// Average C(p)
		for (int i = 0; i< comp_bw.size(); ++i) 
			comp_bw[i].average();
		// Вычисление отклонений
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
			// == И вот тут будем строить алгоритм
			while (currentcam < camera_count) {

					camerathread_mutex.lock();
					if (!free_threads.empty()) {
						CameraThread* thread = camera_threads[free_threads.front()];
						free_threads.pop();

						thread->task = CameraTask::ALGORITHM;
						thread->cp = camparams_lvl[current_lvl];
						thread->aparam = algoparams_lvl[current_lvl];
						thread->cloud = mesh_cloud;
						thread->scharrx = scharrx_images[current_lvl][currentcam];
						thread->scharry = scharry_images[current_lvl][currentcam];
						thread->bw = bw_images[current_lvl][currentcam];
						thread->x = x[currentcam];
						thread->TransM = TransM[currentcam];
						thread->camera_point_inds = camera_point_inds[currentcam];

						thread->currentcam = currentcam;

						cout << "Thread " << thread->threadId << " is processing camera " << currentcam << endl;
				

						emit colorMapperMessage(QString::fromLocal8Bit("Итерация %1 - Изображение %2").arg(iteration+1).arg(currentcam), 100.0f * (float)currentcam / camera_count);
						
						currentcam++;
				
						thread->start();				
					}
					camerathread_mutex.unlock();

					Sleep(10);
					if (stop) return;
			}

			//waiting...
			while (processed_count < camera_count) {
				Sleep(10);
				if (stop) return;
			}
		} catch (...) {
			cout << "Exception in ColorMapper" << endl;
			hardStop();
			emit colorFailed("Unexpected Error in ColorMapper");
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
		if (stop) return;
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
	oldpoints = mesh_cloud->points.size();
	oldtriangles = mesh->polygons.size();
	if (increase_model) {
			
		// subdividing triangles
		increaseVertexCount();
		cout << "vertex count increased" << endl;
		point_normals.resize(mesh_cloud->points.size());
		point_color->points.resize(mesh_cloud->points.size());

		auto itnewp = mesh_cloud->points.begin()+oldpoints;
		for (int i = oldpoints; itnewp != mesh_cloud->points.end(); ++itnewp, ++i) 
			point_normals[i] = point_normals[i] + 
							   point_normals[edge_points[i-oldpoints].first] + 
							   point_normals[edge_points[i-oldpoints].second];

		cout << "new normals computed" << endl;
	}

	avgcolors.resize(mesh_cloud->points.size());
	
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
		// == И вот тут будем строить алгоритм
		while (currentcam < camera_count) {

				camerathread_mutex.lock();
				if (!free_threads.empty()) {
					CameraThread* thread = camera_threads[free_threads.front()];
					free_threads.pop();

					thread->task = CameraTask::POSTPROCESS;
					thread->cp = camparams;
					thread->aparam = algoparams;
					thread->cloud = mesh_cloud;
					thread->mesh = mesh;
					thread->x = x[currentcam];
					thread->TransM = TransM[currentcam];
					thread->cam = cameras[currentcam];
					thread->processing_points_size = oldpoints;
					thread->edge_points = &edge_points;
					thread->currentcam = currentcam;

					cout << "Thread " << thread->threadId << " is processing camera " << currentcam << endl;
				
					emit colorMapperMessage(QString::fromLocal8Bit("Постобработка %1/%2").arg(currentcam+1).arg(camera_count), (int)((100.0f*(float)currentcam)/camera_count));

					currentcam++;
				
					thread->start();				
				}
				camerathread_mutex.unlock();

				Sleep(10);
				if (stop) return;
		}

		//waiting...
		while (processed_count < camera_count) {
			Sleep(10);
			if (stop) return;
		}
	} catch (...) {
		cout << "Exception in ColorMapper" << endl;
		hardStop();
		emit colorFailed("Unexpected Error in ColorMapper: PostProcessCamera");
		return;
	}
// * END MULTITHREADING
	ofstream fout;
	fout.open("xvec.txt");
	for (int ab = 0; ab < algoparams.matrixdim; ++ab) {
		fout << (*x[0])(ab) << ' ';
	}
	fout.close();
// * конец многопоточия
	
//	for (int currentcam = 0; currentcam < camera_point_inds.size(); ++currentcam) {
//
//		postProcessCamera(currentcam, *x[currentcam]);
//	}

	for (int i = 0; i < point_color->points.size(); ++i) {
		if (avgcolors[i].count == 0) {
			avgcolors[i].setRGB(0.3,0.3,0.3);
		} else {
			avgcolors[i].average();
		}
		point_color->points[i].rgb = avgcolors[i].tofloat();//avgcolors[i].tofloat();
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
	emit colorFailed(msg);
}

bool
ColorMapper::getPointUVCoordinates(const PointXYZ &pt, pcl::PointXY &UV_coordinates, CameraParams cp)
{
  if (pt.z > 0)
  {
    // compute image center and dimension
    // focal length is only for this resolution
	double sizeX = 640;
    double sizeY = 480;

    double cx, cy;

    cx = sizeX / 2.0-0.5;
    cy = sizeY / 2.0 - 0.5;

    double focal_x, focal_y; 
	focal_x = cp.focal_x;
	focal_y = cp.focal_y;

    // project point on camera's image plane
	UV_coordinates.x = static_cast<float> ((focal_x * (pt.x / pt.z) + cx) / (sizeX-1) * (cp.color_width-1)); //horizontal
	UV_coordinates.y = static_cast<float> ((focal_y * (pt.y / pt.z) + cy) / (sizeY) * (sizeY*cp.color_width/sizeX)); //vertical

	if (cp.camera_ratio_diff)
		UV_coordinates.y = UV_coordinates.y + (cp.color_height - sizeY*(cp.color_width/sizeX))/2.0 - 2.8;

/*
	UV_coordinates.x = static_cast<float> ((focal_x * (pt.x / pt.z) + cx) / (sizeX-1)); //horizontal
	UV_coordinates.y = 1.0 - static_cast<float> ((focal_y * (pt.y / pt.z) + cy) / (sizeY)); //vertical

	double ratio = (sizeY * cam.width / sizeX) / cam.height;
	double addrat = (1.0/ratio - 1.0) / 2.0;
	UV_coordinates.y = 1.0 - (UV_coordinates.y*ratio + addrat); 

	UV_coordinates.x *= (cam.width-1);
	UV_coordinates.y *= cam.height;
*/
    // point is visible!
	if (UV_coordinates.x >= 0.0 && UV_coordinates.x <= cp.color_width-1.0 && UV_coordinates.y >= 0.0 && UV_coordinates.y <= cp.color_height-1.0){
		return (true); // point was visible by the camera
	}
  }

  // point is NOT visible by the camera
  UV_coordinates.x = -1.0f;
  UV_coordinates.y = -1.0f;
  return (false); // point was not visible by the camera
}

bool
ColorMapper::mapUVtoDepth(const pcl::PointXY &uv, unsigned short* depth_buffer, CameraParams cp) {
	if (uv.x >= 5 && uv.x <= cp.color_width-6 && uv.y >= 5 && uv.y <= cp.color_height-6) {
	    float kofw = cp.depth_width / cp.color_width;
		float kofh = cp.depth_width / cp.color_width;
		int x = uv.x* kofw;
		int y = (cp.camera_ratio_diff ? ( uv.y - (cp.color_height-cp.depth_height/kofw)/2 )*kofw : uv.y * kofh); 
		
		//cout << uv.x <<' '<< uv.y <<' ' << x << ' ' << y << endl;

		if (x >= 0 && y >= 0 && x < cp.depth_width && y < cp.depth_height) {
			unsigned short val = depth_buffer[y*(int)cp.depth_width + x];
			if (val == 0) return false;
			else return true;

		} else return false;

	} else return false;
}

void 
ColorMapper::computeDepthDiscont(unsigned short * dbuffer) {
	float ** bw = filterBWdepth(dbuffer, camparams.depth_width, camparams.depth_height);
	float ** conv_depth = filterArrayScharr(bw, camparams.depth_width, camparams.depth_height);

	int delta = 3;

	for (int i = 0; i < camparams.depth_width; ++i) delete [] bw[i];
	delete [] bw;

// TEST
//	uchar *img = new uchar[(int)camparams.depth_width*(int)camparams.depth_height*3];

	queue<int> q_nulls;
	for (int i = 0; i < camparams.depth_width*camparams.depth_height; ++i) {

// Test		
/*		int val = min(1.0f,16.0f*conv_depth[i % (int)camparams.depth_width][i / (int)camparams.depth_width]) * 255.0f;
		img[i*3  ]=val;
		img[i*3+1]=val;
		img[i*3+2]=val;*/
		
		if (16.0f*conv_depth[i % (int)camparams.depth_width][i / (int)camparams.depth_width] >= 0.5f) 
			dbuffer[i] = 0;
		
		if (dbuffer[i] == 0)
			q_nulls.push(i);
	}

// Test
//	QImage ni = QImage(img, camparams.depth_width, camparams.depth_height, QImage::Format_RGB888);
//	ni.save(QString("test-depth.png"));

	while (!q_nulls.empty()) {
		int ind = q_nulls.front();
		q_nulls.pop();

		int x = ind % (int)camparams.depth_width;
		int y = ind / (int)camparams.depth_width;
		for (int i = -delta; i <= delta; ++i)
			for (int j = -delta; j <= delta; ++j) {
				if (sqrt((float)i*i+(float)j*j) <= delta &&  x+i >= 0 && x+i <= camparams.depth_width-1 && y+j >= 0 && y+j <= camparams.depth_height-1) 
					dbuffer[(x+i)+(y+j)*(int)camparams.depth_width] = 0;
			}
	}

	for (int i = 0; i < camparams.depth_width; ++i) delete [] conv_depth[i];
	delete [] conv_depth;

// TEST
/*	conv_depth = filterBWdepth(dbuffer, camparams.depth_width, camparams.depth_height);
	for (int i = 0; i < camparams.depth_width*camparams.depth_height; ++i) {
		int val = min(1.0f,conv_depth[i % (int)camparams.depth_width][i / (int)camparams.depth_width]) * 255.0f;
		img[i*3  ]=val;
		img[i*3+1]=val;
		img[i*3+2]=val;
	}
	QImage ni2 = QImage(img, camparams.depth_width, camparams.depth_height, QImage::Format_RGB888);
	ni2.save(QString("test-depth2.png"));
	for (int i = 0; i < camparams.depth_width; ++i) delete [] conv_depth[i];
	delete [] conv_depth;*/
}

void
ColorMapper::getTriangleCircumcscribedCircleCentroid ( const pcl::PointXY &p1, const pcl::PointXY &p2, const pcl::PointXY &p3, pcl::PointXY &circumcenter, double &radius)
{
  // compute centroid's coordinates (translate back to original coordinates)
  circumcenter.x = static_cast<float> (p1.x + p2.x + p3.x ) / 3;
  circumcenter.y = static_cast<float> (p1.y + p2.y + p3.y ) / 3;
  double r1 = (circumcenter.x - p1.x) * (circumcenter.x - p1.x) + (circumcenter.y - p1.y) * (circumcenter.y - p1.y)  ;
  double r2 = (circumcenter.x - p2.x) * (circumcenter.x - p2.x) + (circumcenter.y - p2.y) * (circumcenter.y - p2.y)  ;
  double r3 = (circumcenter.x - p3.x) * (circumcenter.x - p3.x) + (circumcenter.y - p3.y) * (circumcenter.y - p3.y)  ;

  // radius
  radius = std::sqrt( std::max( r1, std::max( r2, r3) )) ;
}

bool
ColorMapper::checkPointInsideTriangle(const pcl::PointXY &p1, const pcl::PointXY &p2, const pcl::PointXY &p3, const pcl::PointXY &pt)
{
   // Compute vectors
   Eigen::Vector2d v0, v1, v2;
   v0(0) = p3.x - p1.x; v0(1) = p3.y - p1.y; // v0= C - A
   v1(0) = p2.x - p1.x; v1(1) = p2.y - p1.y; // v1= B - A
   v2(0) = pt.x - p1.x; v2(1) = pt.y - p1.y; // v2= P - A

   // Compute dot products
   double dot00 = v0.dot(v0); // dot00 = dot(v0, v0)
   double dot01 = v0.dot(v1); // dot01 = dot(v0, v1)
   double dot02 = v0.dot(v2); // dot02 = dot(v0, v2)
   double dot11 = v1.dot(v1); // dot11 = dot(v1, v1)
   double dot12 = v1.dot(v2); // dot12 = dot(v1, v2)

   // Compute barycentric coordinates
   double invDenom = 1.0 / (dot00*dot11 - dot01*dot01);
   double u = (dot11*dot02 - dot01*dot12) * invDenom;
   double v = (dot00*dot12 - dot01*dot02) * invDenom;

   // Check if point is in triangle
   return ((u >= 0) && (v >= 0) && (u + v < 1));
}

/*
Eigen::Matrix<double, 2, 4> ColorMapper::getJug(const Eigen::Vector4d& g) {
	double step = 0.0000001;
	Eigen::Matrix<double, 2, 4> Jug;
	Eigen::Vector4d gmod(g);
	Eigen::Vector2d initU = ColorMapper::Ufunc(g);
	gmod(0) += step;
	Jug.col(0) = ColorMapper::UfuncTrue(gmod) - initU;
	gmod(0) -= step; gmod(1) += step;
	Jug.col(1) = ColorMapper::UfuncTrue(gmod) - initU;
	gmod(1) -= step; gmod(2) += step;
	Jug.col(2) = ColorMapper::UfuncTrue(gmod) - initU;
	gmod(2) -= step; gmod(3) += step;
	Jug.col(3) = ColorMapper::UfuncTrue(gmod) - initU;
		  
	return Jug / step;
}

Eigen::Matrix<double, 4, 6> ColorMapper::getJge(const Eigen::Vector4d& point, const Eigen::Matrix4d& Ti, const Eigen::VectorXd xvec) {
	Eigen::Matrix<double, 4, 6> Jge;
	double step = 0.0000001;
	Eigen::Vector4d initG = Gfunc(point, ColorMapper::eTrotation(xvec, Ti));
	Jge.col(0) = ColorMapper::Gfunc(point, ColorMapper::eTrotation(xvec, Ti, 0, step)) - initG;
	Jge.col(1) = ColorMapper::Gfunc(point, ColorMapper::eTrotation(xvec, Ti, 1, step)) - initG;
	Jge.col(2) = ColorMapper::Gfunc(point, ColorMapper::eTrotation(xvec, Ti, 2, step)) - initG;
	Jge.col(3) = ColorMapper::Gfunc(point, ColorMapper::eTrotation(xvec, Ti, 3, step)) - initG;
	Jge.col(4) = ColorMapper::Gfunc(point, ColorMapper::eTrotation(xvec, Ti, 4, step)) - initG;
	Jge.col(5) = ColorMapper::Gfunc(point, ColorMapper::eTrotation(xvec, Ti, 5, step)) - initG;
		
	return Jge / step;
}

Eigen::Matrix2d ColorMapper::getJFu(const Eigen::Vector2d& u, const Eigen::VectorXd& xvec) {
	double step = 0.001;
	Eigen::Matrix2d JFu = Eigen::Matrix2d::Zero();
	Eigen::Vector2d ux = u, uy = u;
	ux(0) += step;
	uy(1) += step;
	int indf = floor(u(0) / ColorMapper::stepx + eps) + floor(u(1)/ColorMapper::stepy + eps) * ColorMapper::gridsizex; 
	for (int i = 0; i < 9; ++i) {
		int nindf = indf + (i/3)*ColorMapper::gridsizex + i%3; 
		if (nindf < ColorMapper::gridsizex*ColorMapper::gridsizey) {
			// fx * d0
			JFu(0,0) += xvec(ColorMapper::dof6+nindf*2) * (ColorMapper::Fi(ux, nindf) - ColorMapper::Fi(u, nindf)) / step;
			JFu(0,1) += xvec(ColorMapper::dof6+nindf*2) * (ColorMapper::Fi(uy, nindf) - ColorMapper::Fi(u, nindf)) / step;
			// fy * d0
			JFu(1,0) += xvec(ColorMapper::dof6+nindf*2+1) * (ColorMapper::Fi(ux, nindf) - ColorMapper::Fi(u, nindf)) / step;
			JFu(1,1) += xvec(ColorMapper::dof6+nindf*2+1) * (ColorMapper::Fi(uy, nindf) - ColorMapper::Fi(u, nindf)) / step;
		}
	}
	JFu(0,0) += 1;
	JFu(1,1) += 1;
	return JFu;
}
*/  
