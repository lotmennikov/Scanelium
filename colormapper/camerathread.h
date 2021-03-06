#ifndef CAMERATHREAD_H_LOT
#define CAMERATHREAD_H_LOT

#include <QtCore/qthread.h>
#include <QtCore/qmutex.h>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>

#include "structs.h"
#include "frame.h"
#include "algoparams.h"
#include "algofuncs.h"
#include "model.h"

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
	AlgoParams aparam;
// in algorithm
	int currentcam;
	int current_lvl;
	img_data* camdata;

	Eigen::VectorXd* x;
	std::vector<Model::PointXYZ>* mesh_vertices;
	std::vector<Model::Triangle>* mesh_triangles;

	std::vector<point_bw>* camera_point_inds;
// in postProcess
	// currentcam;
	// camdata
	// *x
	// vertices
	// triangles
	Frame * cam;
	std::vector< std::pair<int, int> >* edge_points;
	int processing_points_size;
// in preProcess
	// currentcam
	// camdata
	// mesh
	// cloud
	// camera_point_inds
// out
//	VectorXd* dx;

private:

	void computedx();

	void postProcessCamera();

	void preProcessCamera();

	void computePointBW();
signals:

	void started();
	void finished(int currentcam, int thread, bool success);
	void error(int thread, std::string msg);
};

#endif
