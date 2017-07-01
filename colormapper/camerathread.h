#ifndef CAMERATHREAD_H_LOT
#define CAMERATHREAD_H_LOT

#include <QtCore/qthread.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>

#include <QtCore/qmutex.h>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>

#include "structs.h"
#include "camera.h"
#include "camparam.h"
#include "algoparams.h"
#include "algofuncs.h"

class CameraThread : public QThread
{
	Q_OBJECT

	void run() Q_DECL_OVERRIDE;

	static QMutex ident_mutex;
	bool locked;
	bool failed;
public:
	int threadId;
	CameraTask task;
	CameraParams cp;
	AlgoParams aparam;
// in algorithm
	int currentcam;
	Eigen::Matrix4d* TransM;
	float** scharrx;
	float** scharry;
	float** bw;
	Eigen::VectorXd* x;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	std::vector<point_bw>* camera_point_inds;
// in postProcess
	// currentcam;
	// *TransM
	// *x
	// cloud
	pcl::PolygonMesh::Ptr mesh;
	Camera * cam;
	std::vector< std::pair<int, int> >* edge_points;
	int processing_points_size;
// in preProcess
	// currentcam
	// cam
	// *TransM
	// bw
	// mesh
	// cloud
	// camera_point_inds
// out
//	VectorXd* dx;

private:

	void computedx();

	void postProcessCamera();

	void preProcessCamera();

signals:

	void started();
	void finished(int currentcam, int thread, bool success);
	void error(int thread, std::string msg);
};

#endif
