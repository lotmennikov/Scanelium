#ifndef MODELIO_H_LOT
#define MODELIO_H_LOT

#include <QtCore/qstring.h>

#include <vector>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "frame.h"
#include "model.h"

#include "common_global.h"

class ModelIO {

public:
	// save mesh (ply)
	static bool saveMesh(QString filename, Model::Ptr model);

	// save scl scan (SCL)
	static bool saveSCL(QString filename, Model::Ptr model);

	// open scl scan (SCL)
	static bool openSCL(QString filename, Model::Ptr& model);

	// error
	static QString last_error;

private:
	// open mesh (ply)
	static bool openMesh(QString filename, Model::Ptr model);

	static bool writeDPT2(std::string file, std::vector<unsigned short> depth, int width, int height);
	static bool loadDPT2(std::string file, std::vector<unsigned short>& depth, int& width, int& height);

	static bool openSCL2(QString filename, Model::Ptr& model);
	static bool openSCL3(QString filename, Model::Ptr& model);

	static bool saveSCL3(QString filename, Model::Ptr model);
};



#endif
