#ifndef COLORMAPPER_H_LOT
#define COLORMAPPER_H_LOT

#include <iostream>
#include <vector>
#include <QtCore/qthread.h>
#include <QtCore/qmutex.h>
#include <queue>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>

#include "frame.h"
#include "structs.h"
#include "camerathread.h"
#include "filters.h"

#include "model.h"
#include <thread>
#include "renderer.h"

//const double eps = 0.000000001;

#include "colormapper_global.h"

class COLORMAPPER_EXPORT ColorMapper : public QObject
{
	Q_OBJECT

private:
	Renderer* renderer; // produces mesh depth maps

	bool _started;
	bool _stop;
	bool end_iterations;

	std::vector<Frame*> _cameras;
	std::vector<Model::PointXYZ>* mesh_vertices;
	std::vector<Model::Triangle>* mesh_triangles;

	std::vector<Model::ColorRGB> point_color;

	QMutex camerathread_mutex;
	std::vector<CameraThread*> camera_threads;
	std::queue<int> free_threads;

	int camera_count;
	int processed_count;

	// input
	Model::Ptr _model;
	colormap_settings _col_set;
	int iteration_count;
	bool increase_model; // new
	int camerathreads_num;
	int current_lvl;
	int iteration;
	std::vector<Eigen::VectorXd*> x;

	void 
	increaseVertexCount();

	// all for increasing vertex count
	int oldpoints;
	int newpoints;

	int oldtriangles;
	int newtriangles;

	std::vector<Model::Triangle> new_polygons;
	std::vector< std::pair<int, int> > edge_points;

	double initialError;
	double optimizedError;

	void run();
	void cancelThreads();

	void multithread(CameraTask task, AlgoParams ap, iparams cip, CameraParams cp);

public:
	
	std::thread* zhk_thread;
	void
		mapColorsZhouKoltun();

	

	static AlgoParams algoparams;
//	static float camwidth;
//	static float camheight;
//	static float dwidth;
//	static float dheight;
//	static float focal_length;
//	static bool camera_ratio_diff;
//	static double stepx;
//	static double stepy;
//	static const int gridsizex = 11;
//	static const int gridsizey = 9;
//	static const int dof6 = 6;
//	static const int matrixdim = dof6 + gridsizex*gridsizey*2;

	static std::vector<average_grey> comp_bw;
	static std::vector<average_color> avgcolors;
	static std::vector<normal> point_normals;
	//	static vector<Eigen::VectorXd*> dx;
	static Eigen::SparseMatrix<double>* bigIdentity;

	std::vector<Eigen::Matrix4d*> TransM;
	std::vector<std::vector<float**> > bw_images;
	std::vector<std::vector<float**> > scharrx_images;
	std::vector<std::vector<float**> > scharry_images;

	std::vector<std::vector<point_bw>*> camera_point_inds;
public:

	ColorMapper(Renderer*);
	~ColorMapper(void);

	void init(Model::Ptr model, colormap_settings set);
	//void setModel(Model::Ptr model);
	//void setCameras(std::vector<Frame*> cameras);
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

signals:
//	void iterationFinished(int iteration);

	void message(QString message, int progress);
	void refreshResidualError(double initial, double current);
	void finished(bool);
	void error(QString title, QString message);
};


#endif
