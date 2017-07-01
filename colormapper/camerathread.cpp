#include "camerathread.h"
#include "colormapper.h"

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

//		ident_mutex.lock(); locked = true; 
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
			pnt(0) = cloud->points[(*it).index].x;
			pnt(1) = cloud->points[(*it).index].y;
			pnt(2) = cloud->points[(*it).index].z;
			pnt(3) = 1;

	// * compute r
			int indexp = (*it).index;

			if ((*it).bw >= 0) {
				res(ipoint) = ColorMapper::comp_bw[indexp].bw - (*it).bw;
				
		
	// * compute Jge  \/ g(e(al, be, ga, a, b, c))
				Jge = getJge(pnt, *TransM, *x) ;
	
	// * compute Jug  \/
				Eigen::Vector4d initG = Gfunc(pnt, eTrotation(*x, *TransM));
				Jug = getJug(initG, cp);

	// * compute JFu  \/
				Eigen::Vector2d initU = Ufunc(initG,cp);

				JFu = getJFu(initU, *x, aparam);
					
	// * compute Gru  \/
				Eigen::Vector2d initF = Ffunc(initU, *x, aparam);

				double step = 1;
				if (initF(0) >= 0 && initF(1) >= 0) {
					Gru(0, 0) = -compute_value(scharrx, initF(0), initF(1), cp);
					Gru(0, 1) = -compute_value(scharry, initF(0), initF(1), cp);
				} else {
					Gru(0,0) = Gru(0,1) = 0;
					res(ipoint) = 0;
				}
	// * compute Jr   \/
				MatrixXd row6 = - ((((Gru)*JFu) * Jug) * Jge);			
				for (int i = 0; i < dof6; ++i) JrTs.coeffRef(i, ipoint) = row6(0, i);						

	// * compute F part
				int indf = floor(initU(0) / aparam.stepx + eps) + floor(initU(1) / aparam.stepy + eps)*aparam.gridsizex;
				double 
				fi = Fi(initU, indf, aparam);
				JrTs.coeffRef(dof6 +indf*2, ipoint)   = -fi * Gru(0, 0);  // left up
				JrTs.coeffRef(dof6 +indf*2+1, ipoint) = -fi * Gru(0, 1); 
				if (indf%aparam.gridsizex + 1 < aparam.gridsizex) {
					fi = Fi(initU,    indf+1, aparam);
					JrTs.coeffRef(dof6 +(indf+1)*2  , ipoint) = -fi * Gru(0, 0); // right up
					JrTs.coeffRef(dof6 +(indf+1)*2+1, ipoint) = -fi * Gru(0, 1); 
				}
				if (indf/aparam.gridsizex + 1 < aparam.gridsizey) {
					fi = Fi(initU,    indf+aparam.gridsizex, aparam);
					JrTs.coeffRef(dof6 +(indf+aparam.gridsizex)*2  ,ipoint) = -fi * Gru(0, 0); // left down
					JrTs.coeffRef(dof6 +(indf+aparam.gridsizex)*2+1,ipoint) = -fi * Gru(0, 1); 
					if (indf%aparam.gridsizex + 1 < aparam.gridsizex) {
						fi = Fi(initU,    indf+aparam.gridsizex+1, aparam);
						JrTs.coeffRef(dof6 +(indf+aparam.gridsizex+1)*2  ,ipoint) = -fi * Gru(0, 0); // right down
						JrTs.coeffRef(dof6 +(indf+aparam.gridsizex+1)*2+1,ipoint) = -fi * Gru(0, 1); 
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
		Matrix4d transf = eTrotation(*x, *TransM);
		it = camera_point_inds->begin();
		for (;it != camera_point_inds->end(); it++) {
			Vector4d pnt;
			pnt(0) = cloud->points[(*it).index].x;
			pnt(1) = cloud->points[(*it).index].y;
			pnt(2) = cloud->points[(*it).index].z;
			pnt(3) = 1;

			Vector4d initG = Gfunc(pnt, transf);
			Vector2d initU = Ufunc(initG, cp);
			Vector2d initF = Ffunc(initU, *x, aparam);
			if (initF(0) < 0 || initF(1) < 0 || 
				initF(0) > cp.color_width - 1 || initF(1) > cp.color_height - 1)  {
					//	cout << "Bad coordinates : " << initU << endl;
				it->bw = -1;
			//	system("pause");
			} else { 
				it->bw =  compute_value(bw, initF(0), initF(1), cp);
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

void CameraThread::postProcessCamera() {

// transform mesh into camera's frame
	vector<float> camera_z(cloud->points.size()); // new size
	    
	std::vector<bool> visibility;
	visibility.resize (mesh->polygons.size ());

	pcl::PointCloud<pcl::PointXY>::Ptr projections (new pcl::PointCloud<pcl::PointXY>);
	projections->points.resize(processing_points_size); // old size //cloud->points.size()); 

	pcl::PointXY nan_point;
	nan_point.x = -1.0; 
	nan_point.y = -1.0; 

	Matrix4d transf = eTrotation(*x, *TransM);

	auto itp = cloud->points.begin();
	auto ip = projections->points.begin();
	for (int ind = 0; ind < processing_points_size; ++ip, ++itp, ++ind) { // itp != cloud->points.end()

		Vector4d pnt;
		pnt(0) = itp->x;
		pnt(1) = itp->y;
		pnt(2) = itp->z;
		pnt(3) = 1;

		Vector4d initG = Gfunc(pnt, transf);
		Vector2d initU = Ufunc(initG, cp);
		Vector2d initF = Ffunc(initU, *x, aparam);

		camera_z[ind] = initG(2);

		if (initF(0) < 0 && initF(1) < 0 &&
			initF(0) > cp.color_width-1 && initF(1) > cp.color_height-1) {
				(*ip) = nan_point;
		} else {
			ip->x = initF(0);
			ip->y = initF(1);
		}
	}

// removing completely unseen faces
	int cpt_invisible=0;
	auto it = mesh->polygons.begin();
	for (int idx_face = 0; it != mesh->polygons.end(); ++it, ++idx_face) //static_cast<int> (mesh.tex_polygons[current_cam].size ()); ++idx_face)
	{
		if (projections->points[it->vertices[0]].x >= 0.0 &&
			projections->points[it->vertices[1]].x >= 0.0 &&
			projections->points[it->vertices[2]].x >= 0.0)
		{
			visibility[idx_face] = true; 
		} else {
			visibility[idx_face] = false;
			cpt_invisible++;
		}
	}

	if (visibility.size() - cpt_invisible !=0)
	{
		//create kdtree
		pcl::KdTreeFLANN<pcl::PointXY> kdtree;
		kdtree.setInputCloud (projections);

		std::vector<int> idxNeighbors;
		std::vector<float> neighborsSquaredDistance;
			// project all faces
		it = mesh->polygons.begin();
		for (int idx_face = 0; it != mesh->polygons.end(); ++it, ++idx_face)
		{

			if (!visibility[idx_face])
			{
				// no need to check
				continue;
			}

			pcl::PointXY uv_coord1 = projections->points[it->vertices[0]];
			pcl::PointXY uv_coord2 = projections->points[it->vertices[1]];
			pcl::PointXY uv_coord3 = projections->points[it->vertices[2]];
			
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
				} else
					ColorMapper::getTriangleCircumcscribedCircleCentroid(uv_coord1, uv_coord2, uv_coord3, center, radius); // this function yields faster results than getTriangleCircumcenterAndSize

				// get points inside circ.circle
				if (kdtree.radiusSearch (center, radius, idxNeighbors, neighborsSquaredDistance) > 0 )
				{
					// for each neighbor
					for (size_t i = 0; i < idxNeighbors.size (); ++i)
					{
						if (std::max (camera_z[it->vertices[0]],
							std::max (camera_z[it->vertices[1]], 
									  camera_z[it->vertices[2]]))
									< camera_z[idxNeighbors[i]])
						{
							// neighbor is farther than all the face's points. Check if it falls into the triangle
							if (ColorMapper::checkPointInsideTriangle(uv_coord1, uv_coord2, uv_coord3, projections->points[idxNeighbors[i]]))
							{
								projections->points[idxNeighbors[i]] = nan_point;
							}
						}
					}
				}
			}
		}
	} // end' occluding

// adding color to each point
	normal camera;
	camera.x = (*TransM)(0,3);
	camera.y = (*TransM)(1,3);
	camera.z = (*TransM)(2,3);


//increasing vertex count
	if (cloud->points.size() > processing_points_size) {
		projections->points.resize(cloud->points.size());
		
		itp = cloud->points.begin() + processing_points_size;
		ip = projections->points.begin() + processing_points_size;
		auto ep = edge_points->begin();
		for (int ind = processing_points_size; itp != cloud->points.end(); ++ip, ++itp, ++ind, ++ep) {
			if (projections->points[ep->first].x >= 0 && projections->points[ep->second].x >= 0) {
				Vector4d pnt;
				pnt(0) = itp->x;
				pnt(1) = itp->y;
				pnt(2) = itp->z;
				pnt(3) = 1;

				Vector4d initG = Gfunc(pnt, transf);
				Vector2d initU = Ufunc(initG, cp);
				Vector2d initF = Ffunc(initU, *x, aparam);

				if (initF(0) < 0 && initF(1) < 0 &&
					initF(0) > cp.color_width-1 && initF(1) > cp.color_height-1) {
						(*ip) = nan_point;
				} else {
					ip->x = initF(0);
					ip->y = initF(1);
				}
			} else (*ip) = nan_point; 
		}
	}
// projections size expanded

	ident_mutex.lock(); // for save avgcolors inc

	ip = projections->points.begin();
	for (int i = 0; ip != projections->points.end(); ++ip, ++i) { 

		pcl::PointXY uv_coord1 = (*ip);
		if (uv_coord1.x >= 5.0 && uv_coord1.x <= cp.color_width - 6.0 &&
			uv_coord1.y >= 5.0 && uv_coord1.y <= cp.color_height - 6.0) {

			normal tocam;
			tocam.x = camera.x-cloud->points[i].x;
			tocam.y = camera.y-cloud->points[i].y;
			tocam.z = camera.z-cloud->points[i].z;

			normal pointnormal = ColorMapper::point_normals[i];
			
			float weight = (pointnormal.x*tocam.x + pointnormal.y*tocam.y + pointnormal.z*tocam.z) / (pointnormal.getLen() * tocam.getLen());
			
			weight /= tocam.getLen()*tocam.getLen();

			if (!ColorMapper::mapUVtoDepth(uv_coord1, cam->depth, cp))
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
	projections->points.resize(cloud->points.size());

	float width = cp.color_width;
	float height = cp.color_height;
	
	{
		// transform mesh into camera's frame
		pcl::PointCloud<PointXYZ>::Ptr camera_cloud (new pcl::PointCloud<PointXYZ>);
		pcl::transformPointCloud (*cloud, *camera_cloud, cam->pose.inverse ());

	    
		std::vector<bool> visibility;
		visibility.resize (mesh->polygons.size ());

		pcl::PointXY nan_point;
		nan_point.x = -1.0; 
		nan_point.y = -1.0; 

		for (int ip = 0; ip < camera_cloud->points.size(); ++ip) {
			ColorMapper::getPointUVCoordinates(camera_cloud->points[ip], projections->points[ip], cp);
		}
		// removing completely unseen faces
		int cpt_invisible=0;
		auto it = mesh->polygons.begin();
		for (int idx_face = 0; it != mesh->polygons.end(); ++it, ++idx_face) //static_cast<int> (mesh.tex_polygons[current_cam].size ()); ++idx_face)
		{
			if (projections->points[it->vertices[0]].x >= 0.0 &&
				projections->points[it->vertices[1]].x >= 0.0 &&
				projections->points[it->vertices[2]].x >= 0.0)
			{
				visibility[idx_face] = true; 
			} else {
				visibility[idx_face] = false;
				cpt_invisible++;
			}
		}

		if (visibility.size() - cpt_invisible !=0)
		{
			//create kdtree
			pcl::KdTreeFLANN<pcl::PointXY> kdtree;
			kdtree.setInputCloud (projections);

			std::vector<int> idxNeighbors;
			std::vector<float> neighborsSquaredDistance;
				// project all faces
			it = mesh->polygons.begin();
			for (int idx_face = 0; it != mesh->polygons.end(); ++it, ++idx_face)
			{

				if (!visibility[idx_face])
				{
					// no need to check
					continue;
				}

				pcl::PointXY uv_coord1 = projections->points[it->vertices[0]];
				pcl::PointXY uv_coord2 = projections->points[it->vertices[1]];
				pcl::PointXY uv_coord3 = projections->points[it->vertices[2]];
			
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
					} else
						ColorMapper::getTriangleCircumcscribedCircleCentroid(uv_coord1, uv_coord2, uv_coord3, center, radius); // this function yields faster results than getTriangleCircumcenterAndSize

					// get points inside circ.circle
					if (kdtree.radiusSearch (center, radius, idxNeighbors, neighborsSquaredDistance) > 0 )
					{
						// for each neighbor
						for (size_t i = 0; i < idxNeighbors.size (); ++i)
						{
							if (std::max (camera_cloud->points[it->vertices[0]].z,
								std::max (camera_cloud->points[it->vertices[1]].z, 
											camera_cloud->points[it->vertices[2]].z))
										< camera_cloud->points[idxNeighbors[i]].z)
							{
								// neighbor is farther than all the face's points. Check if it falls into the triangle
								if (ColorMapper::checkPointInsideTriangle(uv_coord1, uv_coord2, uv_coord3, projections->points[idxNeighbors[i]]))
								{
									projections->points[idxNeighbors[i]] = nan_point;
								}
							}
						}
					}
				}
			}
		} // end' occluding
	}
	// adding color to each point
	normal camera;
	camera.x = (*TransM)(0,3);
	camera.y = (*TransM)(1,3);
	camera.z = (*TransM)(2,3);

	ident_mutex.lock(); // память экономим и comp_bw аккуратно вычисляем
	cout << "Thread " << threadId << " preprocess finish - computing camera_point_inds" << endl;
	vector<point_bw> temp_points;
	temp_points.reserve(cloud->points.size());

	for (int i = 0; i < projections->points.size(); ++i) { 

		pcl::PointXY uv_coord1 = projections->points[i];
		if (uv_coord1.x >= 8.0 && uv_coord1.x <= width - 9.0 &&
			uv_coord1.y >= 8.0 && uv_coord1.y <= height - 9.0 &&
			
			ColorMapper::mapUVtoDepth(uv_coord1, cam->depth, cp)) {
			
			normal tocam;
			tocam.x = camera.x-cloud->points[i].x;
			tocam.y = camera.y-cloud->points[i].y;
			tocam.z = camera.z-cloud->points[i].z;

			normal pointnormal = ColorMapper::point_normals[i];
			
			float weight = (pointnormal.x*tocam.x + pointnormal.y*tocam.y + pointnormal.z*tocam.z) / (pointnormal.getLen() * tocam.getLen());
			if (weight > 0.1) {
				float val = compute_value(bw, uv_coord1.x, uv_coord1.y, cp);
				ColorMapper::comp_bw[i].addBW(val);

				temp_points.push_back(point_bw(i, val, weight));
			}

		}
	} // 'end add color

	camera_point_inds->resize(temp_points.size());
	copy(temp_points.begin(), temp_points.end(), camera_point_inds->begin());

	ident_mutex.unlock();
}
