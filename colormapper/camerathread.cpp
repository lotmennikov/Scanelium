#include "camerathread.h"
#include "colormapper.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>

QMutex CameraThread::ident_mutex;

using namespace Eigen;
using namespace pcl;
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
//		cout << threadId << " init" << endl;

		Eigen::VectorXd res = VectorXd::Zero(camera_point_inds->size());            // residuals

		//ident_mutex.lock(); locked = true; 
		Eigen::SparseMatrix<double> JrTs(aparam.matrixdim, camera_point_inds->size());
		JrTs.reserve(VectorXi::Constant(camera_point_inds->size(), aparam.dof6 + 8));
//		locked = false; ident_mutex.unlock(); 


		Matrix<double, 1, 2> Gru;                             // Scharr gradient(u)
		Matrix<double, 2, 2> JFu;
		Matrix<double, 2, 4> Jug;                             // Jacobian u(g)
		Matrix<double, 4, 6> Jge;                             // Jacobian g(e) (e == dx)

//		cout << threadId << " points" << endl;

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
				Jge = getJge(pnt, *TransM);

	// * compute Jug  \/
				Eigen::Vector4d initG = Gfunc(pnt, *TransM);

				Jug = getJug(initG, cip);

	// * compute JFu  \/
				Eigen::Vector2d initU = Ufunc(initG, cip); // checked projection

				JFu = getJFu(initU, *x, aparam); // 0 if bad projection

	// * compute Gru  \/
				Eigen::Vector2d initF = Ffunc(initU, *x, aparam); // nan if bad projection

				double step = 1;
				if (initF(0) >= 0 && initF(1) >= 0) {
					Gru(0, 0) = -compute_value(scharrx, initF(0), initF(1), cip);
					Gru(0, 1) = -compute_value(scharry, initF(0), initF(1), cip);
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
		cout << threadId << " solving >";
	//	cout << threadId << " matrix A" << endl;
		SparseMatrix<double>* A = new SparseMatrix<double>(JrTs*JrTs.transpose());
		(*A) = (*A) + (*ColorMapper::bigIdentity);

	//	cout << threadId << " matrix B" << endl;
		MatrixXd B = -(JrTs*res);		
	//	cout << threadId << " cholesky" << endl;
		Eigen::SimplicialCholesky< SparseMatrix<double> > *chol = new SimplicialCholesky< SparseMatrix<double> >(*A);

		// * Finally solve the system
		VectorXd dx = chol->solve(B);


		delete chol;
		delete A;
		cout << "<" << endl;
		
		
		//locked = false; ident_mutex.unlock();

		// COMPUTE NEW BW
		*x += dx;
		
		//Matrix4d transf = eTrotation(*x, *TransM);
		*TransM = eTrotation(dx, *TransM);
		Matrix4d transf = *TransM;

		for (int i = 0; i < 6; ++i) (*x)(i) = 0;

		it = camera_point_inds->begin();
		for (;it != camera_point_inds->end(); it++) {
			Vector4d pnt;
			pnt(0) = mesh_vertices->operator[]((*it).index).x;
			pnt(1) = mesh_vertices->operator[]((*it).index).y;
			pnt(2) = mesh_vertices->operator[]((*it).index).z;
			pnt(3) = 1;

			Vector4d initG = Gfunc(pnt, transf);
			Vector2d initU = Ufunc(initG, cip);
			Vector2d initF = Ffunc(initU, *x, aparam);
			if (initF(0) >= 0 && initF(1) >= 0 && initF(0) <= cip.width - 1 && initF(1) <= cip.height - 1)  {
				it->bw = compute_value(bw, initF(0), initF(1), cip);
			} else {
				//	cout << "Bad coordinates : " << initU << endl;
				it->bw = -1;
				//	system("pause");
			}		
		}
	//	cout << currentcam << ' ';
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

inline Eigen::Vector2f p2_to_v2e(const pcl::PointXY& p) {
	return Eigen::Vector2f(p.x, p.y);
}

inline pcl::PointXY v2e_to_p2(const Eigen::Vector2f& v) {
	pcl::PointXY p; p.x = v.x(); p.y = v.y();
	return p;
}

// TODO replace with simple OpenGL rendering
inline void kdtree_visibility_check(pcl::PointCloud<pcl::PointXY>::Ptr projections, std::vector<Model::Triangle>* mesh_triangles, const vector<float>& camera_z) {
	std::vector<bool> visibility;
	visibility.resize(mesh_triangles->size());

	pcl::PointXY nan_point;
	nan_point.x = -1.0;
	nan_point.y = -1.0;

	// removing completely unseen faces
	int cpt_invisible = 0;
	auto it = mesh_triangles->begin();
	for (int idx_face = 0; it != mesh_triangles->end(); ++it, ++idx_face)
	{
		if (projections->points[it->p[0]].x >= 0.0 &&
			projections->points[it->p[1]].x >= 0.0 &&
			projections->points[it->p[2]].x >= 0.0)
		{
			visibility[idx_face] = true;
		}
		else {
			visibility[idx_face] = false;
			cpt_invisible++;
		}
	}

	if (visibility.size() - cpt_invisible != 0)
	{
		//create kdtree
		pcl::KdTreeFLANN<pcl::PointXY> kdtree;
		kdtree.setInputCloud(projections);

		std::vector<int> idxNeighbors;
		std::vector<float> neighborsSquaredDistance;
		// project all faces
		it = mesh_triangles->begin();
		for (int idx_face = 0; it != mesh_triangles->end(); ++it, ++idx_face)
		{

			if (!visibility[idx_face])
			{
				// no need to check
				continue;
			}

			pcl::PointXY uv_coord1 = projections->points[it->p[0]];
			pcl::PointXY uv_coord2 = projections->points[it->p[1]];
			pcl::PointXY uv_coord3 = projections->points[it->p[2]];

			if (uv_coord1.x >= 0.0 && uv_coord2.x >= 0.0 && uv_coord3.x >= 0.0)
			{
				//face is in the camera's FOV
				//get its circumsribed circle
				double radius;
				pcl::PointXY center;
				// getTriangleCircumcenterAndSize (uv_coord1, uv_coord2, uv_coord3, center, radius);
				if ((uv_coord1.x == uv_coord2.x && uv_coord1.y == uv_coord2.y) &&
					(uv_coord2.x == uv_coord3.x && uv_coord2.y == uv_coord3.y)) {
					center = uv_coord1;
					radius = 0.00001;
				}
				else {
					Eigen::Vector2f ecenter;
					getTriangleCircumcscribedCircleCentroid(p2_to_v2e(uv_coord1), p2_to_v2e(uv_coord2), p2_to_v2e(uv_coord3), ecenter, radius); // this function yields faster results than getTriangleCircumcenterAndSize
					center = v2e_to_p2(ecenter);
				}
				// get points inside circ.circle
				if (kdtree.radiusSearch(center, radius, idxNeighbors, neighborsSquaredDistance) > 0)
				{
					// for each neighbor
					for (size_t i = 0; i < idxNeighbors.size(); ++i)
					{
						if (std::max(camera_z[it->p[0]],
							std::max(camera_z[it->p[1]],
								camera_z[it->p[2]]))
							< camera_z[idxNeighbors[i]])
						{
							// neighbor is farther than all the face's points. Check if it falls into the triangle
							if (checkPointInsideTriangle(p2_to_v2e(uv_coord1), p2_to_v2e(uv_coord2), p2_to_v2e(uv_coord3), p2_to_v2e(projections->points[idxNeighbors[i]])))
							{
								projections->points[idxNeighbors[i]] = nan_point;
							}
						}
					}
				}
			}
		}
	}
	// end' occluding
}

void CameraThread::postProcessCamera() {

	// transform mesh into camera's frame

	pcl::PointCloud<pcl::PointXY>::Ptr projections(new pcl::PointCloud<pcl::PointXY>);
	projections->points.resize(processing_points_size); // old size //cloud->points.size()); 

	Matrix4d transf = *TransM;

	pcl::PointXY nan_point;
	nan_point.x = -1.0;
	nan_point.y = -1.0;

	{
		vector<float> camera_z(mesh_vertices->size()); // new size

		auto itp = mesh_vertices->begin();
		auto ip = projections->points.begin();
		for (int ind = 0; ind < processing_points_size; ++ip, ++itp, ++ind) { // itp != cloud->points.end()

			Vector4d pnt;
			pnt(0) = itp->x;
			pnt(1) = itp->y;
			pnt(2) = itp->z;
			pnt(3) = 1;

			Vector4d initG = Gfunc(pnt, transf);
			Vector2d initU = Ufunc(initG, cip);
			Vector2d initF = Ffunc(initU, *x, aparam);

			camera_z[ind] = initG(2);

			if (initF(0) >= 0 && initF(1) >= 0 && initF(0) <= cip.width - 1 && initF(1) <= cip.height - 1) {
				ip->x = initF(0);
				ip->y = initF(1);
			} else {
				(*ip) = nan_point;
			}
		}

		kdtree_visibility_check(projections, mesh_triangles, camera_z);
	}
// adding color to each point
	normal camera;
	Matrix4d pose_corr = TransM->inverse();
	camera.x = pose_corr(0, 3);
	camera.y = pose_corr(1, 3);
	camera.z = pose_corr(2, 3);


//increasing vertex count
	if (mesh_vertices->size() > processing_points_size) {
		projections->points.resize(mesh_vertices->size());
		
		auto itp = mesh_vertices->begin() + processing_points_size;
		auto ip = projections->points.begin() + processing_points_size;
		auto ep = edge_points->begin();
		for (int ind = processing_points_size; itp != mesh_vertices->end(); ++ip, ++itp, ++ind, ++ep) {
			if (projections->points[ep->first].x >= 0 && projections->points[ep->second].x >= 0) {
				Vector4d pnt;
				pnt(0) = itp->x;
				pnt(1) = itp->y;
				pnt(2) = itp->z;
				pnt(3) = 1;

				Vector4d initG = Gfunc(pnt, transf);
				Vector2d initU = Ufunc(initG, cip);
				Vector2d initF = Ffunc(initU, *x, aparam);

				if (initF(0) >= 0 && initF(1) >= 0 && initF(0) <= cip.width - 1 && initF(1) <= cip.height - 1) {
					ip->x = initF(0);
					ip->y = initF(1);
				}
				else {
					(*ip) = nan_point;
				}
			} else (*ip) = nan_point;
		}
	}
// projections size expanded

	ident_mutex.lock(); // for safe avgcolors inc

	auto ip = projections->points.begin();
	for (int i = 0; ip != projections->points.end(); ++ip, ++i) { 

		pcl::PointXY uv_coord1 = (*ip);
		if (uv_coord1.x >= 5.0 && uv_coord1.x <= cp.color_width - 6.0 &&
			uv_coord1.y >= 5.0 && uv_coord1.y <= cp.color_height - 6.0) {

			normal tocam;
			tocam.x = camera.x- mesh_vertices->operator[](i).x;
			tocam.y = camera.y- mesh_vertices->operator[](i).y;
			tocam.z = camera.z- mesh_vertices->operator[](i).z;

			normal pointnormal = ColorMapper::point_normals[i];
			
			float weight = (pointnormal.x*tocam.x + pointnormal.y*tocam.y + pointnormal.z*tocam.z) / (pointnormal.getLen() * tocam.getLen());
			
			weight /= tocam.getLen()*tocam.getLen();

			if (!mapUVtoDepth(p2_to_v2e(uv_coord1), &cam->depth[0], cp))
				weight *= 0.001;

			if (weight > 0) {
				average_color clr = computeColor(&cam->img, uv_coord1.x, uv_coord1.y);
				ColorMapper::avgcolors[i].addRGBFunc(clr.r, clr.g, clr.b, weight);
			}

		}
	} // 'end add color
	ident_mutex.unlock();
}

void CameraThread::preProcessCamera() {

	pcl::PointCloud<pcl::PointXY>::Ptr projections (new pcl::PointCloud<pcl::PointXY>);
	projections->points.resize(mesh_vertices->size());

	float width = cip.width;
	float height = cip.height;

	pcl::PointXY nan_point;
	nan_point.x = -1.0;
	nan_point.y = -1.0;

	{
		// transform mesh into camera's frame
		//pcl::PointCloud<PointXYZ>::Ptr camera_cloud (new pcl::PointCloud<PointXYZ>);
		//pcl::transformPointCloud (*cloud, *camera_cloud, cam->pose.inverse ());
		vector<float> camera_z(mesh_vertices->size());

		Matrix4d cam_pose_inv = *TransM;

		Eigen::Vector2d proj;

		int ind = 0;
		for (auto it_m = mesh_vertices->begin(); it_m != mesh_vertices->end(); ++it_m, ++ind) {
			Vector4d p(it_m->x, it_m->y, it_m->z, 1.0f);
			Vector4d p_cam = cam_pose_inv*p;
			camera_z[ind] = p_cam.z();

			proj = Ufunc(p_cam, cip);
			if (isnan(proj.x()))
				projections->points[ind] = nan_point;
			else {
				projections->points[ind].x = proj.x(); 
				projections->points[ind].y = proj.y();
			}
		}
	   
		kdtree_visibility_check(projections, mesh_triangles, camera_z);
	}
	// adding color to each point
	normal camera;
	Matrix4d pose_corr = TransM->inverse();
	camera.x = pose_corr(0,3);
	camera.y = pose_corr(1,3);
	camera.z = pose_corr(2,3);

	ident_mutex.lock(); // process each camera sequentially

	cout << "Thread " << threadId << " preprocess finish - computing camera_point_inds" << endl;
	vector<point_bw> temp_points;
	temp_points.reserve(mesh_vertices->size());

	for (int i = 0; i < projections->points.size(); ++i) { 

		pcl::PointXY uv_coord1 = projections->points[i];
		if (uv_coord1.x >= 8.0 && uv_coord1.x <= width - 9.0 &&
			uv_coord1.y >= 8.0 && uv_coord1.y <= height - 9.0 &&
			
			mapUVtoDepth(p2_to_v2e(uv_coord1), &cam->depth[0], cp)) {
			
			normal tocam;
			tocam.x = camera.x- mesh_vertices->operator[](i).x;
			tocam.y = camera.y- mesh_vertices->operator[](i).y;
			tocam.z = camera.z- mesh_vertices->operator[](i).z;

			normal pointnormal = ColorMapper::point_normals[i];
			
			float weight = (pointnormal.x*tocam.x + pointnormal.y*tocam.y + pointnormal.z*tocam.z) / (pointnormal.getLen() * tocam.getLen());
			if (weight > 0.1) {
				float val = compute_value(bw, uv_coord1.x, uv_coord1.y, cip);
				ColorMapper::comp_bw[i].addBW(val);

				temp_points.push_back(point_bw(i, val, weight));
			}

		}
	} // 'end add color

	camera_point_inds->resize(temp_points.size());
	copy(temp_points.begin(), temp_points.end(), camera_point_inds->begin());

	ident_mutex.unlock();
}
