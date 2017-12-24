#ifndef COLORMAPPER_H_LOT
#define COLORMAPPER_H_LOT

#include <iostream>
#include <vector>
#include <QtCore/qthread.h>
#include <QtCore/qmutex.h>
#include <queue>
#include <qmatrix4x4.h>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>

#include "frame.h"
#include "structs.h"
#include "camerathread.h"

#include "model.h"
#include <thread>

#include "colormapper_global.h"

class COLORMAPPER_EXPORT ColorMapper : public QObject
{
	Q_OBJECT

private:

// running
	bool _started;
	bool _stop;
	bool end_iterations;
	
// rendering
	QMutex render_mutex;
	bool render_return;
	std::vector<float> render_result;

// threads
	QMutex camerathread_mutex;
	std::vector<CameraThread*> camera_threads;
	std::queue<int> free_threads;
	int camerathreads_num;

	int camera_count;
	int processed_count;

// input
	Model::Ptr _model;
	colormap_settings _col_set;
	int iteration_count;
	bool increase_model;

// inside algorithm
	int current_lvl;
	int iteration;
	std::vector<img_data*> camdata;
	std::thread* zhk_thread;

	std::vector<Frame*> _cameras;
	std::vector<Model::PointXYZ>* mesh_vertices;
	std::vector<Model::Triangle>* mesh_triangles;

	std::vector<Eigen::VectorXd*> x;
// output
	double initialError;
	double optimizedError;
	std::vector<Model::ColorRGB> point_color;

// increasing vertex count
	int oldpoints;
	int newpoints;

	int oldtriangles;
	int newtriangles;

	std::vector<Model::Triangle> new_polygons;
	std::vector< std::pair<int, int> > edge_points;

// methods
	void run();
	void cancelThreads();
	void increaseVertexCount();
	void multithread(CameraTask task, AlgoParams ap, iparams cip);
	void computePointNormals();
	void renderPose(std::vector<float>& dpt, Eigen::Matrix4f pose, iparams ip);
public:
	void mapColorsZhouKoltun();

	// TODO remove static
	static std::vector<average_grey> comp_bw;
	static std::vector<average_color> avgcolors;
	static std::vector<normal> point_normals;
	static Eigen::SparseMatrix<double>* bigIdentity;

	std::vector<std::vector<float*> > bw_images;
	std::vector<std::vector<float*> > scharrx_images;
	std::vector<std::vector<float*> > scharry_images;

	std::vector<std::vector<point_bw>*> camera_point_inds;
public:

	ColorMapper();
	~ColorMapper(void);

	void init(Model::Ptr model, colormap_settings set);
	void setIterations(int iteration_count);
	//void setThreadsNum(int threads);
	//void setDetalisation(bool det);

	void start();
	void softStop();
	void hardStop(bool wait = false);

	bool isRunning();

public slots:
	void cameraTaskFinished(int camera, int thread, bool success);
	void cameraTaskError(int thread, std::string msg);
	void renderFinished(bool res, std::vector<float> dpt);

signals:
//	void iterationFinished(int iteration);
	void renderRequest(QMatrix4x4 pose, iparams ip);

	void message(QString message, int progress);
	void refreshResidualError(double initial, double current);
	void finished(bool);
	void error(QString title, QString message);
};


#endif
