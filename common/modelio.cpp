#include "modelio.h"
#include <iostream>
#include <fstream>
#include <QtCore/qstring.h>
#include <QtCore/qfileinfo.h>
#include <qdir.h>

using namespace std;
using namespace Eigen;

QString ModelIO::last_error = "";

bool ModelIO::openMesh(QString filename, Model::Ptr model) {
	float* verts = NULL; 
	float* clrs = NULL;
	int* inds = NULL;
	bool has_color = false;

	int num_vertices; int num_indices;
	int num_triangles;
	ifstream fin;
	fin.open(filename.toStdString());
	if (fin.is_open()) {
		string line;
		QStringList list;
		getline(fin, line);			// ply
		if (line.substr(0, 3).compare("ply") == 0) {
			getline(fin, line);		// format
			getline(fin, line);		// comment
			getline(fin, line);		// element
			list = QString::fromStdString(line).split(" ");
			num_vertices = list.at(2).toInt();
			getline(fin, line);		// property float x
			getline(fin, line);		// property float y
			getline(fin, line);		// property float z
			getline(fin, line);		// ?
			list = QString::fromStdString(line).split(" ");
			if (list.at(0).compare(QString::fromStdString("property")) == 0) {  // property uchar r
				has_color = true;
				getline(fin, line);		// property uchar g
				getline(fin, line);		// property uchar b
				getline(fin, line);		// element
			}
			list = QString::fromStdString(line).split(" "); // element face
			num_triangles = list.at(2).toInt();
			getline(fin, line);		// property list ...
			getline(fin, line);		// end_header

			verts = new float[4 * num_vertices];
			if (has_color)
				clrs = new float[3 * num_vertices];
			for (int i = 0; i < num_vertices; ++i) {
				fin >> verts[4 * i + 0] >> verts[4 * i + 1] >> verts[4 * i + 2];
				verts[4 * i + 3] = 1.0f;
				if (has_color) {
					int r, g, b;
					fin >> r >> g >> b;
					clrs[3 * i + 0] = r / 255.0f;
					clrs[3 * i + 1] = g / 255.0f;
					clrs[3 * i + 2] = b / 255.0f;
				}
			}

			inds = new int[num_triangles * 3];
			for (int i = 0; i < num_triangles; ++i) {
				int num_p = 0;
				fin >> num_p >> inds[3 * i + 0] >> inds[3 * i + 1] >> inds[3 * i + 2];
			}
			num_indices = num_triangles * 3;

			
			model->setIBO(verts, num_vertices, inds, num_indices);
			delete[] verts;
			delete[] inds;
			if (has_color) {
				model->setColors(clrs, num_vertices);
				delete[] clrs;
			}
			fin.close();
			return true;
		}
		else {
			last_error = "Not a ply file";
			fin.close();
			return false;
		}
	}
	else {
		last_error = "Could not open mesh file";
		return false;
	}
}

bool ModelIO::saveMesh(QString filename, Model::Ptr model) {
	try {
		std::string file = filename.toStdString();
		ofstream fout;
		fout.open(file);
		//if (fout.is_open()) {
			int num_vertices;
			int num_indices;
			bool has_color;
			float* clrs;

			num_vertices = model->getVertsSize();
			num_indices = model->getIndsSize();
			
			has_color = model->hasColor();
			//if (has_color) model->getColors(clrs);

			fout << "ply" << endl;
			fout << "format ascii 1.0" << endl;
			fout << "comment Created in Scanelium" << endl;
			fout << "element vertex " << num_vertices << endl;
			fout << "property float x" << endl;
			fout << "property float y" << endl;
			fout << "property float z" << endl;
			
			if (has_color) {
				fout << "property uchar red" << endl;
				fout << "property uchar green" << endl;
				fout << "property uchar blue" << endl;
			}
			fout << "element face " << num_indices / 3 << endl;
			fout << "property list uchar int vertex_indices" << endl;
			fout << "end_header";
			
			for (auto it = model->verts_begin(); it != model->verts_end(); ++it) {
				fout << endl;
				Model::PointXYZ p = *it;
				fout 
					<< p.x << " " 
					<< p.y << " "
					<< p.z;
				
				if (has_color) 
					fout 
					<< " " << (int)round(it.getColor().r * 255) 
					<< " " << (int)round(it.getColor().g * 255)
					<< " " << (int)round(it.getColor().b * 255);
			}

			for (auto it = model->tri_begin(); it != model->tri_end(); ++it) {
				Model::Triangle t = *it;
				fout << endl;
				fout << 3 << ' ' << t.p[0] << ' ' << t.p[1] << ' ' << t.p[2];
			}
			fout.close();
			return true;
		//}
		//else {
		//	last_error = "Cannot open file";
		//	return false;
		//}
	}
	catch (...) {
		last_error = "Unknown error";
		return false;
	}
}

bool ModelIO::openSCL(QString filename, Model::Ptr& model) {
	try {
		ifstream fin;
		fin.open(filename.toStdString());
		
		string vers; int ver;
		getline(fin, vers);
		if (vers.substr(0, 4).compare("SCL3") == 0) {
			ver = 3;
			std::cout << "FILE SCL VER3" << endl;
			fin.close();
			return openSCL3(filename, model);
		}
		else if (vers.compare("SCL2") == 0) {
			ver = 2;
			std::cout << "FILE SCL VER2" << endl;
			fin.close();
			return openSCL2(filename, model);
		}
		else {
			last_error = "Unsupported file format";
			fin.close();
			return false;
		}
	}
	catch (...) {
		last_error = "Unknown error";
		return false;
	}
}

bool ModelIO::saveSCL(QString filename, Model::Ptr model) {
	return saveSCL3(filename, model);
}

// PRIVATE

bool ModelIO::writeDPT2(std::string file, vector<unsigned short> depth, int width, int height) {
	ofstream fout;
	fout.open(file, ios::binary);
	if (fout.is_open()) {
		fout.write("DPT2", 4);
		fout.write((char*)&width, 4);
		fout.write((char*)&height, 4);
		fout.write((char*)&depth[0], width*height*sizeof(ushort));
		fout.close();
		return true;
	}
	else
		return false;
}

bool ModelIO::loadDPT2(std::string file, std::vector<unsigned short>& dst, int& width, int& height) {
	ifstream fin;
	fin.open(file, ios::binary);
	if (fin.is_open()) {
		char * ch = new char[5]; ch[4] = 0;
		fin.read(ch, 4);
		fin.read(ch, 4);	width = *(int*)ch;
		fin.read(ch, 4);	height = *(int*)ch;
		delete[] ch;
		int n = width * height;
		dst.resize(n);
		fin.read((char*)&dst[0], n * sizeof(ushort));
		fin.close();
		return true;
	}
	else return false;
}

bool ModelIO::openSCL2(QString filename, Model::Ptr& model) {
//  < read SCL info
	int num_mesh, num_frames;
	string vers, mesh_name_str;
	ifstream fin;
	fin.open(filename.toStdString());
	getline(fin, vers);
	cout << "FILE VER2" << endl;
	QString imgpath = QFileInfo(filename).absolutePath() + "/" + QFileInfo(filename).baseName();

	float focal_length;
	int cwidth;	int cheight;
	int dwidth = 640; int dheight = 480;

	fin >> num_mesh >> mesh_name_str >> num_frames >> focal_length >> cwidth >> cheight;
	fin >> dwidth >> dheight;

	frame_params fparams;
	fparams.color_width = cwidth; fparams.color_height= cheight;
	fparams.depth_width = dwidth; fparams.depth_height= dheight;
	float focal_ratio_depth = dwidth / 640.0f;
	float focal_ratio_color = cwidth / 640.0f;
	fparams.color_fx = focal_length * focal_ratio_color;
	fparams.color_fy = focal_length * focal_ratio_color;
	fparams.depth_fx = focal_length * focal_ratio_depth;
	fparams.depth_fy = focal_length * focal_ratio_depth;

	vector<string> frame_names(num_frames);
	for (int i = 0; i < num_frames; ++i)
		fin >> frame_names[i];
	fin.close();
	// >

	QFileInfo file_info(filename);
	QString scan_name = QFileInfo(filename).baseName(); //QString::fromStdString(scan_name_str);
	QString data_path = file_info.absolutePath() + "/" + scan_name + "/";
	QString ply_file = file_info.absolutePath() + "/" + QString::fromStdString(mesh_name_str);

	model = Model::Ptr(new Model());

	std::cout << "Loading ply..." << endl;
	if (!openMesh(ply_file, model)) {
		return false;
	}

	std::cout << "Loading cameras..." << endl;
	for (int i = 0; i < num_frames; ++i) {
		string frame_name = frame_names[i];
		cout << "Loading frame #" << i << "..." << endl;

		QString frame_txt = data_path + QString::fromStdString(frame_name) + ".txt";
		QString frame_png = data_path + QString::fromStdString(frame_name) + ".png";
		QString frame_dpt = data_path + QString::fromStdString(frame_name) + ".dpt";
		Frame* frame = new Frame();

		// load frame info
		ifstream frame_fin;
		frame_fin.open(frame_txt.toStdString());
		Matrix4f posematrix;
		float val;
		for (int j = 0; j < 16; ++j)
		{
			frame_fin >> val;
			posematrix(j / 4, j % 4) = val;
		}
		frame_fin.close();
		frame->pose = Affine3f(posematrix);
		frame->fparams = fparams;

		// load frame img
		frame->img.load(frame_png);

		// load depth
		ifstream camfin;
		camfin.open(frame_dpt.toStdString());
		frame->depth = vector<unsigned short>(dheight*dwidth);
		for (int j = 0; j < dwidth*dheight; ++j) {
			unsigned short vv;
			camfin >> frame->depth[j];
		}
		camfin.close();

		// add
		model->addFrame(frame);
	}
	return true;
}

bool ModelIO::openSCL3(QString filename, Model::Ptr& model) {
//  < read SCL info
	int num_frames;
	string vers, scan_name_str;
	ifstream fin;
	fin.open(filename.toStdString());
	getline(fin, vers);
	fin >> scan_name_str >> num_frames;
	vector<string> frame_names(num_frames);
	for (int i = 0; i < num_frames; ++i)
		fin >> frame_names[i];
	fin.close();
// >

	QFileInfo file_info(filename);
	QString scan_name = QString::fromStdString(scan_name_str);
	QString data_path = file_info.absolutePath() + "/" + scan_name + "/";
	QString ply_file = data_path + QString::fromStdString(scan_name_str + ".ply");
	
	model = Model::Ptr(new Model());
	
	std::cout << "Loading ply..." << endl;
	if (!openMesh(ply_file, model)) {
		return false;
	}
	
	std::cout << "Loading cameras..." << endl;
	for (int i = 0; i < num_frames; ++i) {
		string frame_name = frame_names[i];
		cout << "Loading frame #" << i << "..." << endl;

		QString frame_txt = data_path + QString::fromStdString(frame_name) + ".txt";
		QString frame_png = data_path + QString::fromStdString(frame_name) + ".png";
		QString frame_dpt = data_path + QString::fromStdString(frame_name) + ".dpt";
		Frame* frame = new Frame();

		// load frame info
		ifstream frame_fin;
		frame_fin.open(frame_txt.toStdString());
		Matrix4f posematrix;
		frame_params fparams;
		frame_fin
			>> fparams.color_width >> fparams.color_height >> fparams.color_fx >> fparams.color_fy
			>> fparams.depth_width >> fparams.depth_height >> fparams.depth_fx >> fparams.depth_fy;
		float val;
		for (int j = 0; j < 16; ++j)
		{
			frame_fin >> val;
			posematrix(j / 4, j % 4) = val;
		}
		frame_fin.close();
		frame->pose = Affine3f(posematrix);
		frame->fparams = fparams;

		// load frame img
		frame->img.load(frame_png);

		// load depth
		int width, height;
		if (loadDPT2(frame_dpt.toStdString(), frame->depth, width, height)) {
			assert(width == fparams.depth_width && height == fparams.depth_height);
		}
		else {
			last_error = "LoadDPT2 failed";
			return false;
		}

		// add
		model->addFrame(frame);
	}
}

bool ModelIO::saveSCL3(QString filename, Model::Ptr model) {
	QFileInfo file_info(filename);
	QString scan_name = file_info.baseName();
	QString data_path = file_info.absolutePath() + "/" + scan_name + "/";

	QString scl_file = file_info.absoluteFilePath();
	QString ply_file = data_path + scan_name + ".ply";
	QString ply_short = scan_name + ".ply";

	cout << "scl file: " << ply_file.toStdString() << endl;
	cout << "data dir: " << data_path.toStdString() << endl;
	cout << "ply file: " << ply_file.toStdString() << endl;

	try {
		QDir data_dir = QDir(data_path);
		if (!data_dir.exists()) QDir().mkdir(data_path);

		if (!saveMesh(ply_file, model))
			return false;

		ofstream fout;
		fout.open(scl_file.toStdString().c_str());

		fout << "SCL3" << endl;
		fout
			<< scan_name.toStdString() << endl  // meshname
			<< model->getFramesSize() << endl; // cameras number

		int num_frames = model->getFramesSize();
		for (int i = 0; i < num_frames; ++i) {
			fout << QString("%1").arg(i).toStdString() << endl;
		}
		fout.close();

		// ==** Cameras **==
		cout << "saving cameras" << endl;
		if (num_frames > 0) {
			for (int i = 0; i < num_frames; ++i) {
				Frame* frame = model->getFrame(i);

				frame->img.save(data_path + QString("%1.png").arg(i));

				ofstream poseout;
				poseout.open((data_path + QString("%1.txt").arg(i)).toStdString().c_str());
				frame_params fparams = frame->fparams;
				poseout << fparams.color_width << ' ' << fparams.color_height << ' ' << fparams.color_fx << ' ' << fparams.color_fy << endl;
				poseout << fparams.depth_width << ' ' << fparams.depth_height << ' ' << fparams.depth_fx << ' ' << fparams.depth_fy << endl;
				poseout << frame->pose.matrix() << endl;
				poseout.close();

				QString dpt_file = data_path + QString("%1.dpt").arg(i);
				writeDPT2(dpt_file.toStdString(), frame->depth, fparams.depth_width, fparams.depth_height);
			}
		}
		return true;
	}
	catch (...) {
		last_error = "Unknown error";
		return false;
	}
}

// ===== DEPRECATED =====

/*
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
*/