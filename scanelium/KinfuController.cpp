#include "KinfuController.h"
#include <qdatetime.h>
#include <qstandardpaths.h>
#include <qdir.h>

//float KinfuController::FOCAL_LENGTH = 541.316f; // подсмотрено //535.002f;//570.34f;
//int KinfuController::camWidth = 640;
//int KinfuController::camHeight = 480;
//int KinfuController::dWidth = 640;
//int KinfuController::dHeight = 480;
//int KinfuController::snapshot_rate = 2000;

CameraParams KinfuController::camparams = CameraParams(541.316f, 541.316f,	// focal_lengths
													   640, 480,			// color 
													   640, 480,			// depth
													   2000);				// snapshot_rate
																

using namespace std;
using namespace pcl;
using namespace pcl::gpu;
using namespace Eigen;
using namespace openni;

struct SampledScopeTime : public StopWatch
{          
  enum { EACH = 33 };
  SampledScopeTime(int& time_ms) : time_ms_(time_ms) {}
  ~SampledScopeTime()
  {
    static int i_ = 0;
    static boost::posix_time::ptime starttime_ = boost::posix_time::microsec_clock::local_time();
    time_ms_ += getTime ();
    if (i_ % EACH == 0 && i_)
    {
      boost::posix_time::ptime endtime_ = boost::posix_time::microsec_clock::local_time();
      cout << "Average frame time = " << time_ms_ / EACH << "ms ( " << 1000.f * EACH / time_ms_ << "fps )"
           << "( real: " << 1000.f * EACH / (endtime_-starttime_).total_milliseconds() << "fps )"  << endl;
      time_ms_ = 0;
      starttime_ = endtime_;
    }
    ++i_;
  }
private:    
  int& time_ms_;
};

KinfuController::KinfuController(QObject* parent) : QThread(parent) 
{

	capture_mutex.lock();
	start_stop.lock();

	started = false;
	stopped = true;
	kinfu_on = false;
	pre_kinfu = false;

	this->capture_ = new ONICapture();
	this->imgProcess = new ImageProcessThread();

	start_stop.unlock();
	capture_mutex.unlock();
  
	int device = 0;
//  pc::parse_argument (argc, argv, "-gpu", device);
	pcl::gpu::setDevice (device);
	pcl::gpu::printShortCudaDeviceInfo (device);

	volume_size = 1.0f;
	camera_pose = CameraPose::CENTERFACE;
	doubleY = false;
	recording = false;
	record_only = false;

	frame_counter_ = 0;
	time_ms_ = 0;
	icp = true;
	visualization = true;
	registration = true;
	enable_texture_extraction = true;
	getmesh = false;
}

/* KinfuController initialization - camera */
bool KinfuController::init() {
	capture_mutex.lock();

	bool sc = capture_->init();
	pre_kinfu = true;

	capture_mutex.unlock();
	
	if (sc) {
		this->supportedColor = capture_->supportedColor;
		this->supportedDepth = capture_->supportedDepth;
		//capture_->setColorMode(ColorResolution::COLOR_SXGA);
		//camWidth = 1280; camHeight = 1024;

		capture_->setColorMode(ColorResolution::COLOR_VGA);
		camparams.setColorMode(ColorResolution::COLOR_VGA);
		
		capture_->setDepthMode(DepthResolution::DEPTH_VGA);
		camparams.setDepthMode(DepthResolution::DEPTH_VGA);
	}

	return sc;
}

void KinfuController::setColorVideoMode(int res) {
	if (started){
		capture_mutex.lock();
		capture_->stop();

		capture_->setColorMode(res);
		camparams.setColorMode((ColorResolution)res);

		capture_->start();
		capture_mutex.unlock();
	} else {
		capture_->setColorMode(res);
		camparams.setColorMode((ColorResolution)res);
	}
}

void KinfuController::setDepthVideoMode(int res) {
	if (started){
		capture_mutex.lock();
		capture_->stop();

		capture_->setDepthMode(res);
		capture_->setDepthMode((DepthResolution)res);

		capture_->start();	
		capture_mutex.unlock();
	} else {
		capture_->setDepthMode(res);
		camparams.setDepthMode((DepthResolution)res);
	}
}

void KinfuController::startCapture() {
	if (!started) {
		start_stop.lock();	  // **<
		started = true;
		stopped = false;
		kinfu_on = false;
		pre_kinfu = true;
		start_stop.unlock();  // >**

		capture_mutex.lock(); // *<

//		assert (camparams.color_width == capture_->getColorResolutionX && camparams.color_height == capture_->getColorResolutionY);
//		assert (camparams.depth_width == capture_->getDepthResolutionX && camparams.depth_height == capture_->getDepthResolutionY);

		capture_->start();
		capture_->setRegistration(true);

//		if (capture_->getAutoWB() != 1)
		capture_->setAutoWhiteBalanceAndExposure(true);
//		if (capture_->getAutoExposure() != 1)
//			capture_->setAutoExposure(false);

		capture_mutex.unlock(); // >*
	}
}

void KinfuController::run() {
	if (!started) startCapture();
	timer.start();
	while (started) {
		capture_mutex.lock(); // *<
		int has_data = capture_->waitFrame(); // data_ready_cond_.timed_wait (lock, boost::posix_time::millisec(50));        
		bool skip = false;
		if (kinfu_on) {
			if (has_data == 0) {
				num_depth ++;
				num_color = 0;
				if (num_depth > 10 && num_depth < 20) 
					skip = true;
			} else {
				num_color++;
				num_depth = 0;
				if (num_color > 2) 
					skip = true;
			}
		}
		if (!skip && captureImage(has_data)) {

			if (pre_kinfu && has_data == 0) {
				getDepthCloud();
			}
			if (kinfu_on) {
				if (!record_only) {
					if (has_data == 1 && pose_ready) {
						checkImage();
	//					capture_->setAutoWhiteBalance(false);
	//					capture_->setAutoExposure(false);
						//cout << capture_->getAutoWB() << " " << capture_->getAutoExposure() << endl;
					}
					if (timer.elapsed() > camparams.snapshot_rate) {
					/*	Camera* cand_camera = imgProcess->getBestCam();
						if (cand_camera != NULL){
							cameras.push_back(cand_camera);
							cand_camera = NULL;
							emit incImagesCount(cameras.size());
						}
						timer.restart();*/
					}
					this->execute(has_data == 0, has_data == 1);
				} else {
					if (has_data == 0) 
						getRGBDOverlap();
				}
			}
		}
		capture_mutex.unlock(); // >*
		if (getmesh) {
			//saveCameras();
			getMesh();
//			stopCapture();
		}	
		//Sleep(100);
	}
	
}

void KinfuController::execute(bool has_depth, 
							  bool has_color) {        
    bool has_image = false;
      
	frame_counter_++;

	if (has_depth) {
		depth_device_.upload (depth_.data, depth_.step, depth_.rows, depth_.cols);  
        SampledScopeTime fps(time_ms_);
		has_image = kinfu_ (depth_device_);                  
    }

    if (has_image)
    {
		pose_ready = true;

		kinfu_.getImage (view_device_);
		int cols;
		view_device_.download (view_host_, cols);

		int width = view_device_.cols(); //pture_->getDepthResolutionX();
		int height = view_device_.rows(); //capture_->getDepthResolutionY();
		// small image interface correction
		unsigned short * pnt_depth = (unsigned short*)depth_.data;
		uchar* pnt_kinfu = (uchar*)&view_host_[0];
		for (int i = 0; i < width*height; ++i) {
			if (pnt_depth[i] == 0) {
				pnt_kinfu[i*3] = 255;// (uchar)((int)pnt_kinfu[i*3] + 255) / 2;
				pnt_kinfu[i*3+1] = (uchar)(pnt_kinfu[i*3+1]);
				pnt_kinfu[i*3+2] = (uchar)(pnt_kinfu[i*3+2]);
			}
		}

		QImage kinfu_image = QImage((uchar*)&view_host_[0], width, height, QImage::Format_RGB888);

		emit capturedKinfu(kinfu_image);
    }    
}

void KinfuController::checkImage() {
	Camera *cam = new Camera();
	
	cam->pose = kinfu_.getCameraPose();
	
	int width = capture_->getColorResolutionX();
	int height = capture_->getColorResolutionY();
	int imsize = width * height * 3;

	uchar* rgb_buffer = new uchar[imsize];
	memcpy(rgb_buffer, (uchar*)&source_image_data_[0], imsize*sizeof(uchar));
	cam->img = QImage(rgb_buffer, width, height, QImage::Format_RGB888, cleanImageBuffer, rgb_buffer);
	cam->depth = new unsigned short[depth_.cols*depth_.rows];
	cam->depth_processed = false;
	memcpy(cam->depth, depth_.data, depth_.cols*depth_.rows*sizeof(unsigned short)); 
	
	cameras.push_back(cam);
	emit incImagesCount(cameras.size());
	//imgProcess->process(cam);
}

bool KinfuController::captureImage(int datatype) {
	VideoFrameRef img = capture_->getImage(datatype);
	switch(datatype) {
	case 0:// DEPTH
		{
			int width = capture_->getDepthResolutionX();
			int height = capture_->getDepthResolutionY();
			depth_.cols = width;
			depth_.rows = height;
			depth_.step = depth_.cols * depth_.elemSize();
			source_depth_data_.resize(depth_.cols * depth_.rows);
	// copying ---		
			memcpy (&source_depth_data_[0], img.getData(), img.getDataSize());
			depth_.data = &source_depth_data_[0];
		}
		return true;
		break;
	case 1:// IMAGE
		{
			int width = capture_->getColorResolutionX();
			int height = capture_->getColorResolutionY();
			rgb24_.cols = width;
			rgb24_.rows = height;
			rgb24_.step = rgb24_.cols*rgb24_.elemSize();
			source_image_data_.resize(rgb24_.cols*rgb24_.rows);
	// copying ---
			uchar* rgb_buffer = (uchar*)&source_image_data_[0];
			memcpy(rgb_buffer, img.getData(), img.getDataSize());
			rgb24_.data = &source_image_data_[0];
		}
		return true;
		break;
	default:
		printf("Unexpected capture error");
		return false;
	}
}

void KinfuController::cleanImageBuffer(void * p) {
	delete [] (uchar*)p;
}

void KinfuController::setCameraPose(CameraPose pose) {
	this->camera_pose = pose;
}

void KinfuController::setVolumeSize(float size) {
	this->volume_size = size;
}

void KinfuController::setDoubleY(bool doubleY_) {
	this->doubleY = doubleY_;
}

void KinfuController::setRecording(bool rec) {
	this->recording = rec;
	if (!rec) this->record_only = false;
}

void KinfuController::setRecordOnly(bool rec) {
	this->record_only = rec;
}

void KinfuController::initKinfu() {

	int depWidth = capture_->getDepthResolutionX();
	int depHeight = capture_->getDepthResolutionY();


	kinfu_on = false;
	getmesh = false;
    kinfu_ = KinfuTracker(depHeight, depWidth, (doubleY ? 2: 1));
	Eigen::Vector3f v_size = Vector3f(volume_size, (doubleY ? volume_size*2 : volume_size), volume_size );    
    kinfu_.volume().setSize (v_size);

	Eigen::Affine3f pose;

	switch (camera_pose) {
	case CENTER:
		{
			Eigen::Vector3f t = v_size * 0.5f;
			pose = Eigen::Translation3f (t);
		}
		break;
	case CENTERFACE:
		{
			Eigen::Matrix3f R = Eigen::Matrix3f::Identity ();   // * AngleAxisf( pcl::deg2rad(-30.f), Vector3f::UnitX());
			Eigen::Vector3f t = v_size * 0.5f - Vector3f (0, 0, v_size (2) / 2 + camera_distance);

			pose = Eigen::Translation3f (t) * Eigen::AngleAxisf (R);
		}
		break;
	case CENTEREDGE:
		{
			// TODO проверить
			Eigen::Matrix3f R = Eigen::Matrix3f::Identity() * Eigen::AngleAxisf( pcl::deg2rad(45.f), Vector3f::UnitX());
			Eigen::Vector3f t = Vector3f (v_size(0)/2, 0, v_size (2)/2);

			pose = Eigen::Translation3f (t) * Eigen::AngleAxisf (R);
		}
		break;
	case VERTEX:
		{
			Eigen::Vector3f t = Eigen::Vector3f::Zero();
			pose = Eigen::Translation3f (t);
		}
		break;
	}
    kinfu_.setInitalCameraPose (pose);
    kinfu_.volume().setTsdfTruncDist (0.030f);    
    kinfu_.setIcpCorespFilteringParams (0.1f, sin ( pcl::deg2rad(20.f) ));
    //kinfu_.setDepthTruncationForICP(5.f//meters);
    kinfu_.setCameraMovementThreshold(0.001f);
	if (depWidth != 640)
		kinfu_.setDepthIntrinsics(camparams.focal_x * ((float)depWidth / 640.0f), camparams.focal_y * ((float)depWidth / 640.0f));
	else
		kinfu_.setDepthIntrinsics(camparams.focal_x, camparams.focal_y);	
}

void KinfuController::startKinfu() {
	if (!started) {
		startCapture();
		pre_kinfu = true;
//		return;
	}

	capture_mutex.lock();

	initKinfu();	
	pose_ready = false;
	num_depth = 0;
	num_color = 0;
	kinfu_.reset();
	capture_->setRegistration(true);

	capture_->setAutoWhiteBalanceAndExposure(false);

	timer.restart();
	if (!imgProcess->isRunning()) {
		imgProcess->start();		
	}
	imgProcess->clear();
	// clear arrays
	for (int i = 0; i < cameras.size(); ++i) 
		delete cameras[i];	
	cameras.clear();
	emit incImagesCount(cameras.size());

	pre_kinfu = false;
	kinfu_on = true;	

	if (recording) {
		QString filename = QStandardPaths::writableLocation(QStandardPaths::HomeLocation) + "/Scanelium/" + "oni" + QDateTime::currentDateTime().toString() + ".oni";
		printf(filename.toStdString().c_str());
		printf("\n");
		capture_->initRecorder("160913.oni");
		capture_->startRec();
	}

	emit kinfuMessage(QString::fromLocal8Bit("Reconstructing..."));
	capture_mutex.unlock();
}

void KinfuController::resetKinfu() {
	if (!started) return;
	if (!kinfu_on) {
		startKinfu();
		return;
	}

	capture_mutex.lock();
	emit kinfuMessage(QString::fromLocal8Bit("Restarting..."));
	pose_ready = false;
	num_depth = 0;
	num_color = 0;
	// reset kinfu
	kinfu_.reset();
	timer.restart();
	imgProcess->clear();
	// clear arrays
	for (int i = 0; i < cameras.size(); ++i) 
		delete cameras[i];	
	cameras.clear();
	emit incImagesCount(cameras.size());

	emit kinfuMessage(QString::fromLocal8Bit("Reconstructing..."));
	capture_mutex.unlock();
}

void KinfuController::stopKinfu(bool get_mesh) {
	if (!kinfu_on) return;

	if (kinfu_on && get_mesh) 
		getmesh = true;
	imgProcess->stop();
	capture_mutex.lock();
	kinfu_on = false;

	if (recording) {
		if (capture_->isRecording()) 
			capture_->stopRec();
	}

	if (!get_mesh) {
		pre_kinfu = true;
		capture_->setAutoWhiteBalanceAndExposure(true);
//		capture_->setAutoExposure(true);

		emit gotMesh(false);
	}
	capture_mutex.unlock();

}

void KinfuController::getMesh() {
	if (getmesh) {
		capture_mutex.lock();

		emit kinfuMessage(QString::fromLocal8Bit("Получение модели..."));

		std::cout << "\nGetting mesh... " << flush;

		if (!marching_cubes_)
		  marching_cubes_ = MarchingCubes::Ptr( new MarchingCubes() );

		DeviceArray<PointXYZ> triangles_device = marching_cubes_->run(kinfu_.volume(), triangles_buffer_device_);    

		if (!triangles_device.empty()) {
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
			cloud->width  = (int)triangles_device.size();
			cloud->height = 1;
			triangles_device.download(cloud->points);

			mesh = PolygonMesh::Ptr( new pcl::PolygonMesh()); 
      
			mesh->polygons.resize (triangles_device.size() / 3);
			for (size_t i = 0; i < mesh->polygons.size (); ++i)
			{
				pcl::Vertices v;
				v.vertices.push_back(i*3+0);
				v.vertices.push_back(i*3+2);
				v.vertices.push_back(i*3+1);              
				mesh->polygons[i] = v;
			}    

			std::cout << "Done.  Triangles number: " << triangles_device.size() / MarchingCubes::POINTS_PER_TRIANGLE / 1000 << "K" << endl;

// * * Simplification * *
			emit kinfuMessage(QString(QString::fromLocal8Bit("Преобразование модели...")));
			
			std::cout << "Simplification check" << endl;

			// find close points
			// create kdtree
			pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
			kdtree.setInputCloud (cloud);

			std::vector<int> repl_index(cloud->points.size(), -1);
			std::vector<int> new_index;
			new_index.resize(cloud->points.size());

			std::vector<int> idxNeighbors;
			std::vector<float> neighborsSquaredDistance;
 
			float radius = 0.0002; // less then 0.2 mm

			int cnt_unique = 0;
			for (int i = 0; i < cloud->points.size(); ++i) {
				if (repl_index[i] == -1) {
					repl_index[i] = i;
					cnt_unique++;
					if (kdtree.radiusSearch (cloud->points[i], radius, idxNeighbors, neighborsSquaredDistance) > 0 )
					{
						bool same_found = false; int same_ind = -1;
						for (int j = 0; j < idxNeighbors.size(); ++j) {
							if (repl_index[idxNeighbors[j]] == -1) {
								repl_index[idxNeighbors[j]] = i;
							} else {
								if (i != idxNeighbors[j]) {
									printf("Unobvious case: same point %d - %d \n", i, idxNeighbors[j]);
									same_found = true; same_ind = repl_index[idxNeighbors[j]]; break;
								
									for (int jj = 0; jj < j; ++jj) {
										if (repl_index[idxNeighbors[jj]] == i) {
											repl_index[idxNeighbors[jj]] = same_ind;
										}
									}
									repl_index[i] = same_ind;
									cnt_unique--;
									break; // no need to view the rest of neighbors
								}
							}
						}
					}
				}
			}

			printf("Possible point reduction: %d to %d\n", cloud->points.size(), cnt_unique);
			if (cloud->points.size() != cnt_unique) {
				printf("Reduction starts...\n");
				//new indices
				pcl::PointCloud<pcl::PointXYZ>::Ptr reduced_cloud (new pcl::PointCloud<pcl::PointXYZ>);
				reduced_cloud->points.reserve(cnt_unique + 1);
				int curr_ind = 0;
				for (int i = 0; i < cloud->points.size(); ++i) {
					if (repl_index[i] == i) {
						reduced_cloud->points.push_back(cloud->points[i]);
						new_index[i] = curr_ind++;
					}
				}
				// polygons with new indices
				std::vector<pcl::Vertices> polygons;
				polygons.reserve(mesh->polygons.size());
				auto plg = mesh->polygons.begin();
				for (; plg != mesh->polygons.end(); ++plg) {
					Vertices vert = (*plg);
					vert.vertices[0] = new_index[repl_index[vert.vertices[0]]];
					vert.vertices[1] = new_index[repl_index[vert.vertices[1]]];
					vert.vertices[2] = new_index[repl_index[vert.vertices[2]]];
					// throw out unpolygoned
					if (vert.vertices[0] == vert.vertices[1] || vert.vertices[1] == vert.vertices[2] || vert.vertices[0] == vert.vertices[2]) 
						continue;
					polygons.push_back(vert);
				}

				mesh->polygons = polygons;
				toPCLPointCloud2(*reduced_cloud, mesh->cloud);
				emit kinfuMessage(QString::fromLocal8Bit("Готово"));

				repl_index.clear();
				new_index.clear();
			} else {
				toPCLPointCloud2(*cloud, mesh->cloud);
			}
		}
// * * end Simplification * *
		getmesh = false;
		capture_mutex.unlock();
		stopCapture();
		emit gotMesh(true);
	} else {
		emit gotMesh(false);
	}
}

void KinfuController::getDepthCloud() {

//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZRGB>);

	QVector<QVector3D>* pnts = new QVector<QVector3D>();
	QVector<QVector3D>* colors = new QVector<QVector3D>();

	float divx = 1.0f / camparams.focal_x;
	float divy = 1.0f / camparams.focal_y;
	if (capture_->getDepthResolutionX() != 640) {
		divx *= 640.0f / capture_->getDepthResolutionX();
		divy *= 640.0f / capture_->getDepthResolutionX();
	}

	int width = depth_.cols;
	int height = depth_.rows;

	float cx = ((float)width - 1) / 2.0f;
	float cy = ((float)height - 1) / 2.0f;

	pnts->reserve(width*height);
	colors->reserve(width*height);

	int depth_idx = 0;
//	ushort mindepth = (ushort)((int)(1<<16)-1);
	ushort maxdepth = 6000;

	ushort fardepth = (ushort)((int)(1<<16)-10);
/*
	for (int i = 0; i < height*width; ++i) {
		if (depth_[i] != 0 && depth_[i] < fardepth){
			if (depth_[i] < mindepth)
				mindepth = depth_[i];
			if (depth_[i] > maxdepth)
				maxdepth = depth_[i];
		}
	}*/
	float vs2 = volume_size / 2.0f;
	float depth_to_color = (capture_->getColorResolutionX() / (float)width);
	bool has_img = source_image_data_.size() == capture_->getColorResolutionX() * capture_->getColorResolutionY();
	float depth_to_color_shiftY = abs(depth_to_color - (capture_->getColorResolutionY() / (float)height)) > 0.00001 ?
		(camparams.color_height - (float)height*depth_to_color) / 2.0 - 3.0 : 0.0;

	for (int v = 0; v < height; ++v)
	{
		for (int u = 0; u < width; ++u, ++depth_idx)
		{
			pcl::PointXYZRGB pt;

			if (depth_[depth_idx] != 0 && depth_[depth_idx] < maxdepth)
			{
				float z = depth_[depth_idx] * 0.001f;
				QVector3D pt(((float)u - cx) * z * divx, 
							 ((float)v - cx) * z * divy, 
							  z);
				float colval = (1.0f - ((float)(depth_[depth_idx]) / (float)(maxdepth)))*0.8f;
				float green = colval;
					switch (camera_pose) {
					case CENTER:
						{
							if (abs(pt.x()) <= vs2 && abs(pt.y()) <= vs2 * (doubleY ? 2 : 1) && abs(pt.z()) <= vs2)
								green = 1.0f;
						}
						break;
					case CENTERFACE:
						{
							if (abs(pt.x()) <= vs2 && abs(pt.y()) <= vs2 * (doubleY ? 2 : 1) && (pt.z() - camera_distance >= 0) && (pt.z() - camera_distance <= volume_size))
								green = 1.0f;
						}
						break; 
					default :
						break;
					}	
				QVector3D clr(colval, green, colval);
				if (green == 1.0f) {
					if (has_img) {
						int coordx = (int)floor((float)u * depth_to_color);
						int coordy = (int)floor((float)v * depth_to_color + depth_to_color_shiftY);
						if (coordx < camparams.color_width && coordy < camparams.color_height) {
							clr.setX(((float)source_image_data_[coordy*camparams.color_width + coordx].r)/255.0);
							clr.setY(((float)source_image_data_[coordy*camparams.color_width + coordx].g)/255.0);
							clr.setZ(((float)source_image_data_[coordy*camparams.color_width + coordx].b)/255.0);
						}
					}
				}
//				pt.r = 250;
//				pt.g = 250;
//				pt.b = 50;
				colors->push_back(clr);
				pnts->push_back(pt);
			}
		}
	}
	emit gotDepthCloud(pnts, colors);
}

void KinfuController::getRGBDOverlap() {

	float divx = 1.0f / camparams.focal_x;
	float divy = 1.0f / camparams.focal_y;
	if (capture_->getDepthResolutionX() != 640) {
		divx *= 640.0f / capture_->getDepthResolutionX();
		divy *= 640.0f / capture_->getDepthResolutionX();
	}

	int width = depth_.cols;
	int height = depth_.rows;

	float cx = ((float)width - 1) / 2.0f;
	float cy = ((float)height - 1) / 2.0f;

//	int depth_idx = 0;
	ushort maxdepth = 6000;

//	ushort fardepth = (ushort)((int)(1<<16)-10);


//	float vs2 = volume_size / 2.0f;
	float depth_to_color = (capture_->getColorResolutionX() / (float)width);
	bool has_img = source_image_data_.size() == capture_->getColorResolutionX() * capture_->getColorResolutionY();
	float depth_to_color_shiftY = abs(depth_to_color - (capture_->getColorResolutionY() / (float)height)) > 0.00001 ?
		(camparams.color_height - (float)height*depth_to_color) / 2.0 - 3.0 : 0.0;


	unsigned short * pnt_depth = (unsigned short*)depth_.data;
	
	uchar* pnt_kinfu = new uchar[width*height*3];//(uchar*)&rgbd_vec[0];
	int i = 0;
	for (int v = 0; v < height; ++v) {
	for (int u = 0; u < width; ++u, ++i) {	
		if (pnt_depth[i] > 0 && pnt_depth[i] < maxdepth) {
			float colval = (1.0f - ((float)(pnt_depth[i]) / (float)(maxdepth)))*0.8f;	
			
			pnt_kinfu[i*3] =   (uchar)(floor(colval*255));
			pnt_kinfu[i*3+1] = (uchar)(floor(colval*255));
			pnt_kinfu[i*3+2] = (uchar)(floor(colval*255));
			if (has_img) {
				int coordx = (int)floor((float)u * depth_to_color);
				int coordy = (int)floor((float)v * depth_to_color + depth_to_color_shiftY);
				if (coordx < camparams.color_width && coordy < camparams.color_height) {
					pnt_kinfu[i*3] =   (uchar)min((uint)255, (uint)(source_image_data_[coordy*camparams.color_width + coordx].r*0.7f) + (uint)(pnt_kinfu[i*3]*0.5));
					pnt_kinfu[i*3+1] = (uchar)min((uint)255, (uint)(source_image_data_[coordy*camparams.color_width + coordx].g*0.7f) + (uint)(pnt_kinfu[i*3+1]*0.5));
					pnt_kinfu[i*3+2] = (uchar)min((uint)255, (uint)(source_image_data_[coordy*camparams.color_width + coordx].b*0.7f));
				}
			}
		}
	} }

//	for (int v = 0; v < height; ++v) {
//		for (int u = 0; u < width; ++u, ++depth_idx) {
//			if (depth_[depth_idx] != 0 && depth_[depth_idx] < maxdepth)	{
//				float z = depth_[depth_idx] * 0.001f;
//				float colval = (1.0f - ((float)(depth_[depth_idx]) / (float)(maxdepth)))*0.8f;			
//			}
//		}
//	}

	QImage kinfu_image = QImage(pnt_kinfu, width, height, QImage::Format_RGB888, cleanImageBuffer, pnt_kinfu);

	emit capturedKinfu(kinfu_image);

}

void KinfuController::stopCapture() {
	if (kinfu_on) stopKinfu(false);

	if (!stopped) {
		start_stop.lock();
		pre_kinfu = false;
		started = false;
		stopped = true;
		start_stop.unlock();
		
		capture_mutex.lock();
		capture_->stop();
		capture_mutex.unlock();
	}
}

void KinfuController::destroy() {
	if (started || !stopped) 
		stopCapture();
	capture_mutex.lock();
	capture_->destroy();
	if (imgProcess->isRunning())
	{
		imgProcess->stop();
		imgProcess->quit();
		imgProcess->wait();
	}
	capture_mutex.unlock();
}

KinfuController::~KinfuController(void)
{
	delete imgProcess;
	delete capture_;
}
