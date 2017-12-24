#include "camerathread.h"
#include "colormapper.h"

QMutex CameraThread::ident_mutex;

using namespace Eigen;
using namespace std;

void CameraThread::run() {
	failed = false;
	locked = false;
	switch (task) {
	case ALGORITHM:
		computedx();
		break;
	case PREPROCESS:
		preProcessCamera();
		break;
	case POSTPROCESS:
		postProcessCamera();
		break;
	}
	if (failed) return;

	emit finished(currentcam, threadId, true);
}

void CameraThread::computedx() {

	int dof6 = 6;

// Vector - C(p) - mx1
	try {
		Eigen::VectorXd res = VectorXd::Zero(camera_point_inds->size());            // residuals

		//ident_mutex.lock(); locked = true; 
		Eigen::SparseMatrix<double> JrTs(aparam.matrixdim, camera_point_inds->size());
		JrTs.reserve(VectorXi::Constant(camera_point_inds->size(), aparam.dof6 + 8));
//		locked = false; ident_mutex.unlock(); 


		Matrix<double, 1, 2> Gru;                             // Scharr gradient(u)
		Matrix<double, 2, 2> JFu;
		Matrix<double, 2, 4> Jug;                             // Jacobian u(g)
		Matrix<double, 4, 6> Jge;                             // Jacobian g(e) (e == dx)

		vector<point_bw>::iterator it = camera_point_inds->begin();
		for (int ipoint = 0; it != camera_point_inds->end(); ++ipoint, it++) {
			Eigen::Vector4d pnt;
			pnt(0) = mesh_vertices->operator[]((*it).index).x;
			pnt(1) = mesh_vertices->operator[]((*it).index).y;
			pnt(2) = mesh_vertices->operator[]((*it).index).z;
			pnt(3) = 1;

	// * compute r
			int indexp = (*it).index;

			if ((*it).bw >= 0) {
				res(ipoint) = ColorMapper::comp_bw[indexp].bw - (*it).bw;
				
		
	// * compute Jge  \/ g(e(al, be, ga, a, b, c))
				Jge = getJge(pnt, camdata->TransM);

	// * compute Jug  \/
				Eigen::Vector4d initG = Gfunc(pnt, camdata->TransM);

				Jug = getJug(initG, camdata->ip);

	// * compute JFu  \/
				Eigen::Vector2d initU = Ufunc(initG, camdata->ip); // checked projection

				JFu = getJFu(initU, *x, aparam); // 0 if bad projection

	// * compute Gru  \/
				Eigen::Vector2d initF = Ffunc(initU, *x, aparam); // nan if bad projection

				double step = 1;
				if (initF(0) >= 0 && initF(1) >= 0) {
					Gru(0, 0) = -compute_value(camdata->scharrx, initF(0), initF(1), camdata->ip);
					Gru(0, 1) = -compute_value(camdata->scharry, initF(0), initF(1), camdata->ip);
				} else {
					Gru(0,0) = Gru(0,1) = 0;
					res(ipoint) = 0;
				}
	// * compute Jr   \/
				MatrixXd row6 = - ((((Gru)*JFu) * Jug) * Jge);			
				for (int i = 0; i < dof6; ++i) JrTs.coeffRef(i, ipoint) = row6(0, i);						

	// * compute F part
				if (!isnan(initU.x())) { // only of projection is valid
					int indf = floor(initU(0) / aparam.stepx + eps) + floor(initU(1) / aparam.stepy + eps)*aparam.gridsizex;
					double
						fi = Fi(initU, indf, aparam);
					JrTs.coeffRef(dof6 + indf * 2, ipoint) = -fi * Gru(0, 0);  // left up
					JrTs.coeffRef(dof6 + indf * 2 + 1, ipoint) = -fi * Gru(0, 1);
					if (indf%aparam.gridsizex + 1 < aparam.gridsizex) {
						fi = Fi(initU, indf + 1, aparam);
						JrTs.coeffRef(dof6 + (indf + 1) * 2, ipoint) = -fi * Gru(0, 0); // right up
						JrTs.coeffRef(dof6 + (indf + 1) * 2 + 1, ipoint) = -fi * Gru(0, 1);
					}
					if (indf / aparam.gridsizex + 1 < aparam.gridsizey) {
						fi = Fi(initU, indf + aparam.gridsizex, aparam);
						JrTs.coeffRef(dof6 + (indf + aparam.gridsizex) * 2, ipoint) = -fi * Gru(0, 0); // left down
						JrTs.coeffRef(dof6 + (indf + aparam.gridsizex) * 2 + 1, ipoint) = -fi * Gru(0, 1);
						if (indf%aparam.gridsizex + 1 < aparam.gridsizex) {
							fi = Fi(initU, indf + aparam.gridsizex + 1, aparam);
							JrTs.coeffRef(dof6 + (indf + aparam.gridsizex + 1) * 2, ipoint) = -fi * Gru(0, 0); // right down
							JrTs.coeffRef(dof6 + (indf + aparam.gridsizex + 1) * 2 + 1, ipoint) = -fi * Gru(0, 1);
						}
					}
				}
			} else res(ipoint) = 0;
	// * final Jr Row
		}
	// * compute JrT  \/
	// *              \/
	// * solve (JrT*Jr)*dx = (-JrT*r)

		//ident_mutex.lock(); locked = true;
		cout << "<" << threadId << "s>" << endl;
		SparseMatrix<double>* A = new SparseMatrix<double>(JrTs*JrTs.transpose());
		(*A) = (*A) + (*ColorMapper::bigIdentity);

		MatrixXd B = -(JrTs*res);		
		Eigen::SimplicialCholesky< SparseMatrix<double> > *chol = new SimplicialCholesky< SparseMatrix<double> >(*A);

	// * Finally solve the system
		VectorXd dx = chol->solve(B);

		delete chol;
		delete A;
		cout << "</" << threadId << "s>" << endl;
		
		//locked = false; ident_mutex.unlock();

	// COMPUTE NEW BW
		*x += dx;
		for (int i = 0; i < 6; ++i) (*x)(i) = 0; // transformation is updated independently

		camdata->TransM = eTrotation(dx, camdata->TransM);
		Matrix4d transf = camdata->TransM;


		it = camera_point_inds->begin();
		for (;it != camera_point_inds->end(); it++) {
			Vector4d pnt;
			pnt(0) = mesh_vertices->operator[]((*it).index).x;
			pnt(1) = mesh_vertices->operator[]((*it).index).y;
			pnt(2) = mesh_vertices->operator[]((*it).index).z;
			pnt(3) = 1;

			Vector4d initG = Gfunc(pnt, transf);
			Vector2d initU = Ufunc(initG, camdata->ip);
			Vector2d initF = Ffunc(initU, *x, aparam);
			if (initF(0) >= 0 && initF(1) >= 0 && initF(0) <= camdata->ip.width - 1 && initF(1) <= camdata->ip.height - 1)  {
				it->bw = compute_value(camdata->bw, initF(0), initF(1), camdata->ip);
			} else {
				//	cout << "Bad coordinates : " << initU << endl;
				it->bw = -1;
				//	system("pause");
			}		
		}
	} catch (const std::exception& ex) {
		cout << "Thread "<< threadId << " exception " << ex.what() << endl;
		if 	(locked)
			ident_mutex.unlock();
		failed = true;
		emit error(threadId, ex.what());
	} catch (const std::string& ex) {
		cout << "Thread "<< threadId << " exception " << ex << endl;
		if 	(locked)
			ident_mutex.unlock();
		failed = true;
		emit error(threadId, ex);
	} catch (...) {
		cout << "Thread "<< threadId << "exception" << endl;;
		if 	(locked)
			ident_mutex.unlock();
		failed = true;
		emit error(threadId, "Exception in CameraThread");
	}
}

inline float interp(float* data, float ux, float uy, iparams ip) {
	float ux_l = floorf(ux);
	float uy_l = floorf(uy);
	float ux_m = ux_l + 1.0f;
	float uy_m = uy_l + 1.0f;

	if (ux_m > ip.width - 1 || uy_m > ip.height - 1)
		return data[(int)(uy_l*ip.width + ux_l)];

	float val00 = data[(int)(uy_l*ip.width + ux_l)];
	float val01 = data[(int)(uy_m*ip.width + ux_l)];
	float val10 = data[(int)(uy_l*ip.width + ux_m)];
	float val11 = data[(int)(uy_m*ip.width + ux_m)];

	return
		val00 * (ux_m - ux) * (uy_m - uy) +
		val01 * (ux_m - ux) * (uy - uy_l) +
		val10 * (ux - ux_l) * (uy_m - uy) +
		val11 * (ux - ux_l) * (uy - uy_l);
}

inline void computeProjections(Matrix4d cam_pose_inv, vector<Model::PointXYZ>* mesh_vertices, int num_points, vector<Vector2d>& projections, vector<bool>& visible, float* depth, iparams cip) {
	Vector2d nan_point(nan(""), nan(""));
	Vector2d proj;

	float camera_z;

	int ind = 0;
	for (; ind < num_points; ++ind) {
		Model::PointXYZ mp = mesh_vertices->at(ind);
		Vector4d p = Vector4d(mp.x, mp.y, mp.z, 1.0f);
		Vector4d p_cam;

		p_cam = Gfunc(p, cam_pose_inv);
		proj = Ufunc(p_cam, cip);


		camera_z = p_cam.z();

		if (isnan(proj.x())) {
			projections[ind] = nan_point;
			visible[ind] = false;
		}
		else {
			projections[ind] = proj;

			int ux = (int)roundf(proj.x());
			int uy = (int)roundf(proj.y());
			float depth_rendered = interp(depth, proj.x(), proj.y(), cip);//camdata->render[uy*cip.width + ux];
			float diff = camera_z - depth_rendered;

			if (diff < 0.001f)
				visible[ind] = true;
			else
				visible[ind] = false;
		}
	}
}

void CameraThread::postProcessCamera() {
	iparams cip = camdata->ip;

	vector<Vector2d> projections(processing_points_size);
	vector<bool> visible_render(processing_points_size);

	Matrix4d cam_pose_inv = camdata->TransM;
	computeProjections(cam_pose_inv, mesh_vertices, processing_points_size, projections, visible_render, camdata->render, camdata->ip);

	Matrix4d cam_pose = cam_pose_inv.inverse();
	normal camera;
	camera.x = cam_pose(0, 3);
	camera.y = cam_pose(1, 3);
	camera.z = cam_pose(2, 3);

//increasing vertex count
	Vector2d nan_point(nan(""), nan(""));

	if (mesh_vertices->size() > processing_points_size) {
		projections.resize(mesh_vertices->size());
		visible_render.resize(mesh_vertices->size());
		auto itp = mesh_vertices->begin() + processing_points_size;
		auto ip = projections.begin() + processing_points_size;
		auto ep = edge_points->begin();
		for (int ind = processing_points_size; itp != mesh_vertices->end(); ++ip, ++itp, ++ind, ++ep) {
			if (visible_render[ep->first] && visible_render[ep->second]) {
				Vector4d pnt(itp->x, itp->y, itp->z, 1);

				Vector4d gp = Gfunc(pnt, cam_pose_inv);
				Vector2d proj = Ufunc(gp, cip);
				
				if (isnan(proj.x())) {
					projections[ind] = nan_point;
					visible_render[ind] = false;
				}
				else {
					projections[ind] = proj;
					visible_render[ind] = true;
				}
			} else (*ip) = nan_point;
		}
	}
// projections size expanded

	ident_mutex.lock(); // for safe avgcolors inc

// adding color to each point
	//uchar* img_bits = new uchar[3 * cip.width*cip.height];
	//memset(img_bits, 0, 3 * cip.width*cip.height * sizeof(uchar));
	
	auto ip = projections.begin();
	for (int i = 0; ip != projections.end(); ++ip, ++i) { 

		Vector2d u = (*ip);
		if (visible_render[i]) {
			float dpt_weight = interp(camdata->dpt_weights, u.x(), u.y(), cip);
			Vector2d uf = Ffunc(u, *x, aparam);

			if (uf.x() >= 5.0 && uf.x() <= cip.width - 6.0 &&
				uf.y() >= 5.0 && uf.y() <= cip.height - 6.0) {

				normal tocam;
				tocam.x = camera.x - mesh_vertices->operator[](i).x;
				tocam.y = camera.y - mesh_vertices->operator[](i).y;
				tocam.z = camera.z - mesh_vertices->operator[](i).z;

				normal pointnormal = ColorMapper::point_normals[i];

				float weight = max(0.0f, (pointnormal.x*tocam.x + pointnormal.y*tocam.y + pointnormal.z*tocam.z) / (pointnormal.getLen() * tocam.getLen()));

				weight /= tocam.getLen()*tocam.getLen();

				if (dpt_weight < 0.001f) dpt_weight = 0.001f;
				if (dpt_weight < 1.0f) dpt_weight = dpt_weight*dpt_weight*0.1f;

				weight *= dpt_weight;

				if (weight > 0) {
					average_color clr = computeColor(&cam->img, uf.x(), uf.y());
					ColorMapper::avgcolors[i].addRGBFunc(clr.r, clr.g, clr.b, weight);
				}

				// img
			}

			/*
			int ux = (int)roundf(u.x());
			int uy = (int)roundf(u.y());
			img_bits[3 * (uy*cip.width + ux) + 0] = 255 * (1.0f - dpt_weight);
			img_bits[3 * (uy*cip.width + ux) + 1] = 255 * visible_render[i] * (dpt_weight);
			img_bits[3 * (uy*cip.width + ux) + 2] = 0;
			*/
		}
	} // 'end add color

	//QImage img_errdpt = QImage(img_bits, cip.width, cip.height, QImage::Format_RGB888).copy();
	//img_errdpt.save(QString::fromStdString("img_errdpt" + to_string(this->currentcam) + ".png"));
	//delete[] img_bits;

	ident_mutex.unlock();
}

void CameraThread::preProcessCamera() {
	iparams cip = camdata->ip;

	vector<Vector2d> projections(mesh_vertices->size());
	vector<bool> visible_render(mesh_vertices->size());

	Matrix4d cam_pose_inv = camdata->TransM;
	computeProjections(cam_pose_inv, mesh_vertices, mesh_vertices->size(), projections, visible_render, camdata->render, camdata->ip);
	   
	// adding color to each point
	normal camera;
	Matrix4d pose_corr = camdata->TransM.inverse();
	camera.x = pose_corr(0,3);
	camera.y = pose_corr(1,3);
	camera.z = pose_corr(2,3);

	ident_mutex.lock(); // process each camera sequentially

	cout << "Thread " << threadId << " preprocess finish - computing camera_point_inds" << endl;
	vector<point_bw> temp_points;
	temp_points.reserve(mesh_vertices->size());

	uchar* img_bits = new uchar[3 * cip.width*cip.height];
	memset(img_bits, 0, 3 * cip.width*cip.height * sizeof(uchar));

	for (int i = 0; i < projections.size(); ++i) { 

		Vector2d u = projections[i];
		if (u.x() >= 8.0 && u.x() <= cip.width - 9.0 &&
			u.y() >= 8.0 && u.y() <= cip.height - 9.0 &&
			visible_render[i]) {

			normal tocam;
			tocam.x = camera.x- mesh_vertices->operator[](i).x;
			tocam.y = camera.y- mesh_vertices->operator[](i).y;
			tocam.z = camera.z- mesh_vertices->operator[](i).z;

			normal pointnormal = ColorMapper::point_normals[i];
			
			float dpt_weight = interp(camdata->dpt_weights, u.x(), u.y(), cip);

			float weight = max(0.0f, (pointnormal.x*tocam.x + pointnormal.y*tocam.y + pointnormal.z*tocam.z) / (pointnormal.getLen() * tocam.getLen()));
			if (dpt_weight > 0.5 && weight > 0.1) {
				float val = compute_value(camdata->bw, u.x(), u.y(), cip);
				ColorMapper::comp_bw[i].addBW(val);

				temp_points.push_back(point_bw(i, val, weight));
			}

			// img
			int ux = (int)roundf(u.x());
			int uy = (int)roundf(u.y());
			img_bits[3 * (uy*cip.width + ux) + 0] = 255 * (1.0f - dpt_weight);
			img_bits[3 * (uy*cip.width + ux) + 1] = 255 * visible_render[i] * (dpt_weight);
			img_bits[3 * (uy*cip.width + ux) + 2] = 0;
		}
	} // 'end add color

	//QImage img_errdpt = QImage(img_bits, cip.width, cip.height, QImage::Format_RGB888).copy();
	//img_errdpt.save("img_errdpt.png");
	delete[] img_bits;

	camera_point_inds->resize(temp_points.size());
	std::copy(temp_points.begin(), temp_points.end(), camera_point_inds->begin());

	ident_mutex.unlock();
}
