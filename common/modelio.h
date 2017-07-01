#ifndef MODELIO_H_LOT
#define MODELIO_H_LOT

#include <QtCore/qstring.h>

#include <vector>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "camera.h"
#include "camparam.h"

#include "common_global.h"

class ModelIO {

public:
	static bool saveModel(QString path, QString scan_name, CameraParams camparams, std::vector<Camera*> cameras);
	
	static bool openModel(QString filename, CameraParams& camparams, std::vector<Camera*>& cameras);

private:
//	static bool saveModelSCL1();
	static bool saveModelSCL2(QString path, QString scan_name, CameraParams camparams, std::vector<Camera*> cameras);
	static bool saveModelSCL3(QString path, QString scan_name, CameraParams camparams, std::vector<Camera*> cameras);

	static bool openModelSCL1(QString filename, CameraParams& camparams, std::vector<Camera*>& cameras);
	static bool openModelSCL2(QString filename, CameraParams& camparams, std::vector<Camera*>& cameras);
	static bool openModelSCL3(QString filename, CameraParams& camparams, std::vector<Camera*>& cameras);

};



#endif
