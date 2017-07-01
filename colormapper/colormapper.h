#ifndef COLORMAPPER_H_LOT
#define COLORMAPPER_H_LOT

#include <iostream>
#include <vector>
#include <QtCore/qthread.h>
#include <QtCore/qmutex.h>
//#include <qimage.h>
//#include <qcolor.h>
#include <queue>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>

#include <pcl/conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "camera.h"
#include "structs.h"
#include "camerathread.h"
#include "filters.h"

//const double eps = 0.000000001;

#include "colormapper_global.h"

class COLORMAPPER_EXPORT ColorMapper : public QThread
{
	Q_OBJECT

	void run() Q_DECL_OVERRIDE;

private:

	std::vector<Camera*> cameras;

	pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_cloud;
	pcl::PointCloud<pcl::RGB>::Ptr point_color;

	QMutex camerathread_mutex;
	int camerathreads_num;
	std::vector<CameraThread*> camera_threads;
	std::queue<int> free_threads;

	bool started;
	bool stop;
	bool end_iterations;

	bool increase_model; // new

	int camera_count;
	int processed_count;

	int iteration_count;

	void
	mapColorsZhouKoltun();

	void 
	increaseVertexCount();

	// all for increasing vertex count
	int oldpoints;
	int newpoints;

	int oldtriangles;
	int newtriangles;

	std::vector< pcl::Vertices> new_polygons;
	std::vector< std::pair<int, int> > edge_points;
//	void 
//	processCamera(int currentcam);

//	void 
//	postProcessCamera(int currentcam, const Eigen::VectorXd& x);

public:
	static CameraParams camparams;
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

	static bool
	getPointUVCoordinates (const pcl::PointXYZ &pt, pcl::PointXY &UV_coordinates, CameraParams cp);

	static bool
	mapUVtoDepth(const pcl::PointXY &uv, unsigned short* depth_buffer, CameraParams cp);

	// find depth discontinueties on bw depth map
	void 
	computeDepthDiscont(unsigned short * dbuffer);

	static bool
	checkPointInsideTriangle (const pcl::PointXY &p1, const pcl::PointXY &p2, const pcl::PointXY &p3, const pcl::PointXY &pt);

	static void 
	getTriangleCircumcscribedCircleCentroid ( const pcl::PointXY &p1, const pcl::PointXY &p2, const pcl::PointXY &p3, pcl::PointXY &circumcenter, double &radius);

//	static 
//	Eigen::Vector2d Ufunc(const Eigen::Vector4d& pt);
//	static
//	Eigen::Vector2d UfuncTrue(const Eigen::Vector4d& pt);
//	static 
//	Eigen::Vector4d Gfunc(const Eigen::Vector4d& p, const Eigen::Matrix4d& Ti);
//	static 
//	Eigen::Vector2d Ffunc(const Eigen::Vector2d& u, const Eigen::VectorXd& xvec);
//	static
//	double Fi(const Eigen::Vector2d& u, int indf);
//	static 
//	Eigen::Matrix4d eTrotation(const Eigen::VectorXd& xvec, const Eigen::Matrix4d& Ti, int add = -1, double step = 0);
/*
	inline 
	Eigen::Matrix<double, 4, 6> getJge(const Eigen::Vector4d& point, const Eigen::Matrix4d& Ti, const Eigen::VectorXd xvec);

	inline 
	Eigen::Matrix<double, 2, 4> getJug(const Eigen::Vector4d& g);

	inline 
	Eigen::Matrix2d getJFu(const Eigen::Vector2d& u, const Eigen::VectorXd& xvec);
*/	

public:
	double initialError;
	double optimizedError;

	pcl::PolygonMesh::Ptr mesh;

	ColorMapper(QObject* parent = 0);
	~ColorMapper(void);

	void init(CameraParams cparam);
	void setMesh(pcl::PolygonMesh::Ptr mesh);
	void setCameras(std::vector<Camera*> cameras);
	void setIterations(int iteration_count);
	void setThreadsNum(int threads);
	void setDetalisation(bool det);
	void softStop();
	void hardStop();

public slots:
	void cameraTaskFinished(int camera, int thread, bool success);
	void cameraTaskError(int thread, std::string msg);
signals:
//	void iterationFinished(int iteration);

	void colorMapperMessage(QString message, int progress);
	void refreshResidualError(double initial, double current);
	void colorFinished(bool);
	void colorFailed(std::string message);
};


#endif
