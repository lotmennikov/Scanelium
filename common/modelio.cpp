#include "modelio.h"
#include <iostream>
#include <fstream>
#include <QtCore/qstring.h>
#include <QtCore/qfileinfo.h>

using namespace std;
using namespace Eigen;

bool ModelIO::openModel(QString filename, CameraParams& camparams, vector<Camera*>& cameras) {
	return openModelSCL2(filename, camparams, cameras);
}

bool ModelIO::openModelSCL1(QString filename, CameraParams& camparams, vector<Camera*>& cameras) { return false; }

bool ModelIO::openModelSCL2(QString filename, CameraParams& camparams, vector<Camera*>& cameras) {
	
	QString imgpath = QFileInfo(filename).absolutePath() + "/" + QFileInfo(filename).baseName();

	ifstream fin;
    fin.open(filename.toStdString().c_str());

	int num_mesh; string mesh_name; // unused
	int camera_count;

	float focal_length;
	int cwidth;	int cheight;
	int dwidth = 640; int dheight = 480;

	int ver;
	string camera_name,vers;
	getline(fin, vers);
	if (vers.compare("SCL1") == 0) ver = 1;
	else if (vers.compare("SCL2") == 0) {
		ver = 2;
		cout << "FILE VER2" << endl;
	}
	else throw "Unsupported file format";

	fin >> num_mesh >> mesh_name >> camera_count >> focal_length >> cwidth >> cheight;
		
	if (ver == 2)
		fin >> dwidth >> dheight;

	camparams.focal_x = camparams.focal_y = focal_length;
	camparams.color_width = cwidth; camparams.color_height = cheight;
	camparams.depth_width = dwidth; camparams.depth_height = dheight;
	camparams.snapshot_rate = 2000;

	cout << "Loading cameras..." << endl;

//		QMessageBox* error_box =new QMessageBox(QString::fromLocal8Bit("Check"),
//			QString::fromStdString("%1 " + mesh_name + " %2 %3 %4 %5").arg(num_mesh).arg(camera_count).arg(focal_length).arg(width).arg(height),
//						QMessageBox::Critical,
//						QMessageBox::Ok, 0, 0);
//		int n = error_box->exec(); 
//		delete error_box; 
	//char * meshfile;
	//meshfile = "mesh.ply";
//	vector<Camera*> cameras;
//	int ind = 0;
	vector<string> cam_names(camera_count);
	for (int i = 0; i < camera_count; ++i) 
		fin >> cam_names[i];
	fin.close();

	for (int i = 0; i < camera_count; ++i) {
		string camera_name = cam_names[i];
		cout << "Loading camera #" << i << "..." << endl; 
		QString cam_png = imgpath + "/" + QString::fromStdString(camera_name) + ".png";
		cout << cam_png.toStdString() << endl;
		//		while (QFile(QString("%1.txt").arg(ind)).exists()) {
		Camera* cam = new Camera();
			
		cam->img.load(cam_png);

		ifstream camfin;
        camfin.open((imgpath.toStdString() + "/"+ camera_name + ".txt").c_str());
		Matrix4f posematrix;
		float val;
		for (int j = 0; j < 16; ++j)
		{
			camfin >> val;
			posematrix(j/4, j%4) = val;
		}
		camfin.close();
		cam->pose = Affine3f(posematrix);

		// V2 -
		if (ver == 2) {
            camfin.open((imgpath.toStdString() + "/"+ camera_name + ".dpt").c_str());
			cam->depth = new unsigned short[dheight*dwidth];
			cam->depth_processed = false;
			for (int j = 0; j < dwidth*dheight; ++j)  {
				unsigned short vv;
				camfin >> cam->depth[j];
			}
//				fin.read((char*)(cam->depth), dwidth*dheight*sizeof(unsigned short));
			camfin.close();
		}
		// - V2

		cameras.push_back(cam);
	}

	return true;
}

bool ModelIO::saveModel(QString path, QString scan_name, CameraParams camparams, vector<Camera*> cameras) {
	return saveModelSCL2(path, scan_name, camparams, cameras);
}

bool ModelIO::saveModelSCL2(QString path, QString scan_name, CameraParams camparams, vector<Camera*> cameras) {
	QString filename = path + "/" + scan_name + ".scl";

	ofstream fout;
    fout.open(filename.toStdString().c_str());
	// V2 - 
	fout << "SCL2" << endl;
	// - V2
	// V1 -
	fout	<< 1  << endl  // mesh num
			<< scan_name.toStdString()	<< ".ply" << endl // meshname
			<< cameras.size()			<< endl // cameras number
			<< camparams.focal_x		<< ' '	// focal length
			<< camparams.color_width	<< ' '	// color widht
			<< camparams.color_height	<< ' '	// color height 
	// - V1
	// V2 - 
			<< camparams.depth_width	<< ' '
			<< camparams.depth_height	<< endl;
	// - V2
	for (int i = 0; i < cameras.size(); ++i) {
		fout << QString("%1").arg(i).toStdString() << endl;
	}
	fout.close();


// ==** Cameras **==
	cout << "saving cameras" << endl;
	QString imgpath = path + "/" + scan_name + "/";
	if (cameras.size() > 0) {
		for (int i = 0; i < cameras.size(); ++i) {
			/*
			// * test * depth capture TODO remove it
			if (camWidth == dWidth && camHeight == dHeight) { 
				for (int j=0; j< depth_.rows*depth_.cols; j++) {
					if (cameras[i]->depth[j] == 0) {
						cameras[i]->img.setPixel(j % depth_.cols, (int)(j / depth_.cols), qRgb(0,0,0));
					}
				}
			}

			// * test * end
			*/

			cameras[i]->img.save(imgpath + QString("%1.png").arg(i));
			ofstream poseout, camout;
            poseout.open((imgpath + QString("%1.txt").arg(i)).toStdString().c_str());
			poseout << cameras[i]->pose.matrix() << endl;
			poseout.close();

            camout.open((imgpath + QString("%1.dpt").arg(i)).toStdString().c_str());
			for (int j = 0; j < camparams.depth_width*camparams.depth_height; ++j) camout << cameras[i]->depth[j] << ' ';
//			camout.write(reinterpret_cast<char*>(cameras[i]->depth), dWidth*camparams.depth_height*sizeof(unsigned short));
			camout.close();
		}
	}
		
	return true;
}

bool ModelIO::saveModelSCL3(QString path, QString scan_name, CameraParams camparams, vector<Camera*> cameras) {
// UNDER CONSTRUCTION
	return false;
}
