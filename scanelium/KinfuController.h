#ifndef KINFUCONTROLLER_H_LOT
#define KINFUCONTROLLER_H_LOT

#include <qimage.h>
#include <qvector.h>
#include <qvector3d.h>
#include <QtCore/qthread.h>
#include <QtCore/qelapsedtimer.h>

#include <pcl/gpu/kinfu/kinfu.h>
#include <pcl/gpu/kinfu/raycaster.h>
#include <pcl/gpu/kinfu/marching_cubes.h>
#include <pcl/gpu/containers/initialization.h>
#include <pcl/kdtree/kdtree_flann.h>


#include <pcl/common/angles.h>
#include <pcl/common/time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/io/ply_io.h>

#include <pcl/exceptions.h>

#include "onicapture.h"
#include "camera.h"
#include "ImageProcessThread.h"
#include "structs.h"
#include "camparam.h"


class KinfuController : public QThread
{
	Q_OBJECT

	void run() Q_DECL_OVERRIDE;

private:
	QMutex capture_mutex;
	ONICapture* capture_;
	ImageProcessThread* imgProcess;

	int* supportedColor;
	int* supportedDepth;

	QMutex start_stop;
	bool started;
	bool stopped;
	bool pre_kinfu;
	bool kinfu_on;

// kinfu
	pcl::gpu::KinfuTracker kinfu_;
	pcl::gpu::KinfuTracker::DepthMap depth_device_;
	pcl::gpu::KinfuTracker::View view_device_;
	std::vector<pcl::gpu::KinfuTracker::PixelRGB> view_host_;

	std::vector<pcl::gpu::KinfuTracker::PixelRGB> source_image_data_;
	std::vector<unsigned short> source_depth_data_;
	pcl::gpu::PtrStepSz<const unsigned short> depth_;
	pcl::gpu::PtrStepSz<const pcl::gpu::KinfuTracker::PixelRGB> rgb24_;

	pcl::gpu::MarchingCubes::Ptr marching_cubes_;
	pcl::gpu::DeviceArray<pcl::PointXYZ> triangles_buffer_device_;

	float volume_size;
	bool doubleY;
	CameraPose camera_pose;
	bool recording;

	bool registration;
	bool icp;
	bool visualization;
	bool enable_texture_extraction;
	bool getmesh;

	int num_depth;
	int num_color;

	void execute(//const PtrStepSz<const unsigned short>& depth, 
				 //const PtrStepSz<const KinfuTracker::PixelRGB>& rgb24,
				 bool has_depth, 
				 bool has_color);
	
	bool captureImage(int);

	void checkImage();


public:

//	static float FOCAL_LENGTH;
//	static int camWidth;
//	static int camHeight;
//	static int dWidth;
//	static int dHeight;
//	static int snapshot_rate;

	static CameraParams camparams;

	pcl::PolygonMesh::Ptr mesh;

	QImage depth;
	QImage color;

	std::vector<Camera*> cameras;
//	QVector<QVector3D> pnts;
//	QVector<QVector3D> colors;

	KinfuController(QObject* parent = 0);
	~KinfuController(void);

	int frame_counter_;

	QElapsedTimer timer;

	int time_ms_;

	bool init();
	void destroy();

	bool pose_ready;

	bool record_only;

	static void cleanImageBuffer(void*);

public slots:
	void startCapture();
	
	void setColorVideoMode(int col_res);
	void setDepthVideoMode(int dep_res);

	void setVolumeSize(float size);
	void setDoubleY(bool doubleY_);
	void setCameraPose(CameraPose);
	void setRecording(bool rec);
	void setRecordOnly(bool rec);
	
	void initKinfu();
	void startKinfu();
	void resetKinfu();
	void stopKinfu(bool getmesh);
	void getMesh();
	void getDepthCloud();
	void getRGBDOverlap();

	void stopCapture();

signals:
	void capturedDepth(const QImage& img);
	void capturedColor(const QImage& img);
	void capturedKinfu(const QImage& img);

	void kinfuMessage(QString msg);

	void incImagesCount(int count);
	void gotMesh(bool);
	void gotDepthCloud(QVector<QVector3D>*, QVector<QVector3D>*);

};


#endif
