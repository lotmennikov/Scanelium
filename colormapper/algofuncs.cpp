#include "algofuncs.h"
#include "filters.h"
#include <queue>

Eigen::Vector2d Ufunc(const Eigen::Vector4d& pt, CameraParams camparams) {
	Eigen::Vector2d u;
	if (pt(2) > 0)
	{
		// compute image center and dimension
		double sizeX = 640; // default focus
		double sizeY = 480;

		double cx, cy;

		cx = sizeX / 2.0-0.5;
		cy = sizeY / 2.0-0.5;

		double focal_x, focal_y; 

		focal_x = camparams.focal_x;
		focal_y = camparams.focal_y;

		// project point on camera's image plane
		u(0) = ((focal_x * (pt(0) / pt(2)) + cx) / (sizeX-1) * (camparams.color_width-1)); //horizontal
		u(1) = ((focal_y * (pt(1) / pt(2)) + cy) / (sizeY) * (sizeY*camparams.color_width/sizeX)); //vertical
			
		if (camparams.camera_ratio_diff)
			u(1) = u(1) + (camparams.color_height - sizeY*(camparams.color_width/sizeX))/2.0 - 2.8;

		// point is visible
		if (u(0) >= 0.0 && u(0) <= camparams.color_width-1.0 && u(1) >= 0.0 && u(1) <= camparams.color_height-1.0){
			return u; 
		}
	}
	u(0) = -1.0f;
	u(1) = -1.0f;
	return u;
}

//returns value even if out of border
Eigen::Vector2d UfuncTrue(const Eigen::Vector4d& pt, CameraParams camparams) {
	Eigen::Vector2d u;
	if (pt(2) > 0)
	{
		// compute image center and dimension
		double sizeX = 640; // default focus
		double sizeY = 480;

		double cx, cy;

		cx = sizeX / 2.0-0.5;
		cy = sizeY / 2.0-0.5;

		double focal_x, focal_y; 

		focal_x = camparams.focal_x;
		focal_y = camparams.focal_y;

		// project point on camera's image plane
		u(0) = ((focal_x * (pt(0) / pt(2)) + cx) / (sizeX-1) * (camparams.color_width-1)); //horizontal
		u(1) = ((focal_y * (pt(1) / pt(2)) + cy) / (sizeY) * (sizeY*camparams.color_width/sizeX)); //vertical

		if (camparams.camera_ratio_diff)
			u(1) = u(1) + (camparams.color_height - sizeY*(camparams.color_width/sizeX))/2.0 - 2.8;

		// point is visible
		//if (u(0) >= 8.0 && u(0) < cam.width-8.0 && u(1) >= 8.0 && u(1) < cam.height-8.0){
			return u; 
	//	}
	}
	u(0) = -1.0;
	u(1) = -1.0;
	return u;
}


Eigen::Vector4d Gfunc(const Eigen::Vector4d& p, const Eigen::Matrix4d& Ti) {
	return Ti.inverse()*p;
}

Eigen::Vector2d Ffunc(const Eigen::Vector2d& u, const Eigen::VectorXd& xvec, AlgoParams aparam) {
	if (u(0) < 0 || u(1) < 0) {
		return u;
	}
	Eigen::Vector2d F = u;
	int ind = floor(u(1) / aparam.stepy + eps) * aparam.gridsizex + floor(u(0) / aparam.stepx + eps);
	double difx = (u(0) - (ind%aparam.gridsizex)*aparam.stepx) / aparam.stepx;
	double dify = (u(1) - (ind/aparam.gridsizex)*aparam.stepy) / aparam.stepy;
	F = F + (1.0f-difx)*(1.0f-dify)*Eigen::Vector2d(xvec(aparam.dof6+ ind					 *2), xvec(aparam.dof6+ ind					   *2+1)) + // left up
			(     difx)*(1.0f-dify)*Eigen::Vector2d(xvec(aparam.dof6+(ind+1)			     *2), xvec(aparam.dof6+(ind+1)				   *2+1)) + // right up
			(1.0f-difx)*(     dify)*Eigen::Vector2d(xvec(aparam.dof6+(ind+aparam.gridsizex)  *2), xvec(aparam.dof6+(ind+aparam.gridsizex)*2  +1)) + // left down
			(     difx)*(     dify)*Eigen::Vector2d(xvec(aparam.dof6+(ind+aparam.gridsizex+1)*2), xvec(aparam.dof6+(ind+aparam.gridsizex+1)*2+1));  // right down
	return F;
}

Eigen::Matrix4d eTrotation(const Eigen::VectorXd& xvec, const Eigen::Matrix4d& Ti, int add, double step) {
	Eigen::Matrix<double, 6, 1> corxvec = xvec.middleRows(0, 6);
	if (add != -1) corxvec(add) += step;
		
//		Eigen::Matrix4d newTi = Eigen::Matrix4d(Ti);
//		newTi(0, 3) = newTi(1,3) = newTi(2,3) = 0;

	Eigen::Matrix4d eTrot;
	eTrot(0, 0) = eTrot(1, 1) = eTrot(2, 2) = eTrot(3, 3) = 1;
	eTrot(3, 0) = eTrot(3, 1) = eTrot(3, 2) = 0;
	eTrot(0, 3) =  corxvec(3); eTrot(1, 3) = corxvec(4); eTrot(2, 3) = corxvec(5);
	eTrot(0, 1) = -corxvec(2); eTrot(1, 0) = corxvec(2);
	eTrot(0, 2) =  corxvec(1); eTrot(2, 0) =-corxvec(1);
	eTrot(1, 2) = -corxvec(0); eTrot(2, 1) = corxvec(0);
//		newTi = eTrot*newTi;
//		newTi(0, 3) = Ti(0, 3) + corxvec(3);
//		newTi(1, 3) = Ti(1, 3) + corxvec(4);
//		newTi(2, 3) = Ti(2, 3) + corxvec(5);

	return eTrot * Ti;
}

double Fi(const Eigen::Vector2d& u, int indf, AlgoParams aparam) {
	double indfx = (indf % aparam.gridsizex)*aparam.stepx;
	double indfy = (indf / aparam.gridsizex)*aparam.stepy;
	if (abs(u(0)-indfx) < aparam.stepx && abs(u(1)-indfy) < aparam.stepy) {
		return (1.0 - abs(u(0) - indfx)/aparam.stepx) * (1.0 - abs(u(1) - indfy)/aparam.stepy);
	} else return 0;
}

Eigen::Matrix<double, 2, 4> getJug(const Eigen::Vector4d& g, CameraParams cp) {
	double step = 0.0000001;
	Eigen::Matrix<double, 2, 4> Jug;
	Eigen::Vector4d gmod(g);
	Eigen::Vector2d initU = Ufunc(g,cp);
	gmod(0) += step;
	Jug.col(0) = UfuncTrue(gmod, cp) - initU;
	gmod(0) -= step; gmod(1) += step;
	Jug.col(1) = UfuncTrue(gmod, cp) - initU;
	gmod(1) -= step; gmod(2) += step;
	Jug.col(2) = UfuncTrue(gmod, cp) - initU;
	gmod(2) -= step; gmod(3) += step;
	Jug.col(3) = UfuncTrue(gmod, cp) - initU;
		  
	return Jug / step;
}

Eigen::Matrix<double, 4, 6> getJge(const Eigen::Vector4d& point, const Eigen::Matrix4d& Ti, const Eigen::VectorXd xvec) {
	Eigen::Matrix<double, 4, 6> Jge;
	double step = 0.0000001;
	Eigen::Vector4d initG = Gfunc(point, eTrotation(xvec, Ti));
	Jge.col(0) = Gfunc(point, eTrotation(xvec, Ti, 0, step)) - initG;
	Jge.col(1) = Gfunc(point, eTrotation(xvec, Ti, 1, step)) - initG;
	Jge.col(2) = Gfunc(point, eTrotation(xvec, Ti, 2, step)) - initG;
	Jge.col(3) = Gfunc(point, eTrotation(xvec, Ti, 3, step)) - initG;
	Jge.col(4) = Gfunc(point, eTrotation(xvec, Ti, 4, step)) - initG;
	Jge.col(5) = Gfunc(point, eTrotation(xvec, Ti, 5, step)) - initG;
		
	return Jge / step;
}

Eigen::Matrix2d getJFu(const Eigen::Vector2d& u, const Eigen::VectorXd& xvec, AlgoParams ap) {
	double step = 0.001;
	Eigen::Matrix2d JFu = Eigen::Matrix2d::Zero();
	Eigen::Vector2d ux = u, uy = u;
	ux(0) += step;
	uy(1) += step;
	int indf = floor(u(0) / ap.stepx + eps) + floor(u(1)/ap.stepy + eps) * ap.gridsizex; 
	for (int i = 0; i < 9; ++i) {
		int nindf = indf + (i/3)*ap.gridsizex + i%3; 
		if (nindf < ap.gridsizex*ap.gridsizey) {
			// fx * d0
			JFu(0,0) += xvec(ap.dof6+nindf*2) * (Fi(ux, nindf, ap) - Fi(u, nindf, ap)) / step;
			JFu(0,1) += xvec(ap.dof6+nindf*2) * (Fi(uy, nindf, ap) - Fi(u, nindf, ap)) / step;
			// fy * d0
			JFu(1,0) += xvec(ap.dof6+nindf*2+1) * (Fi(ux, nindf, ap) - Fi(u, nindf, ap)) / step;
			JFu(1,1) += xvec(ap.dof6+nindf*2+1) * (Fi(uy, nindf, ap) - Fi(u, nindf, ap)) / step;
		}
	}
	JFu(0,0) += 1;
	JFu(1,1) += 1;
	return JFu;
}
	  
average_color computeColor(QImage* img, float pix_x,float pix_y) {
	QColor pix = QColor(img->pixel(pix_x, pix_y));
	if ((int)pix_x + 1 < img->width() && (int)pix_y + 1 < img->height()) {				
		average_color avg;
					
		avg.addRGBFunc(pix.red()/255.0, pix.green()/255.0, pix.blue()/255.0, (((int)pix_x + 1 - pix_x) * ((int)pix_y + 1 - pix_y)));
					
		pix = QColor(img->pixel((int)pix_x + 1, (int)pix_y));
		avg.addRGBFunc(pix.red()/255.0, pix.green()/255.0, pix.blue()/255.0, ((pix_x     - (int)pix_x) * ((int)pix_y + 1 - pix_y)));

		pix = QColor(img->pixel((int)pix_x, (int)pix_y + 1));
		avg.addRGBFunc(pix.red()/255.0, pix.green()/255.0, pix.blue()/255.0, (((int)pix_x + 1 - pix_x) * (pix_y -     (int)pix_y)));

		pix = QColor(img->pixel((int)pix_x + 1, (int)pix_y + 1));
		avg.addRGBFunc(pix.red()/255.0, pix.green()/255.0, pix.blue()/255.0, ((pix_x -     (int)pix_x) * (pix_y -     (int)pix_y)));

		avg.average();
		return avg;
	} else {
		return average_color(pix.red()/255, pix.green()/255, pix.blue()/255);
	}
}

float compute_value(float** img, float x, float y, CameraParams camparams) {
	float result = 0.0f;
	if (x < 0 || y < 0 || x > camparams.color_width - 1 || y > camparams.color_height - 1)
		return 0;

	float difx = x - floor(x); 
	float dify = y - floor(y); 
	if (floor(x) + 1 < camparams.color_width && floor(y) + 1 < camparams.color_height) {				
		average_grey avg;
					
		result += img[(int)floor(x+eps)  ][(int)floor(y+eps)  ] * (1.0f - difx) * (1.0f - dify);
		result += img[(int)floor(x+eps)+1][(int)floor(y+eps)  ] * (       difx) * (1.0f - dify);
		result += img[(int)floor(x+eps)  ][(int)floor(y+eps)+1] * (1.0f - difx) * (       dify);
		result += img[(int)floor(x+eps)+1][(int)floor(y+eps)+1] * (       difx) * (       dify);
					
		return result;
	} else {
		return img[(int)floor(x)][(int)floor(y)];
	}
}

// FROM COLORMAPPER

bool
getPointUVCoordinates(const Model::PointXYZ &pt, Eigen::Vector2d &UV_coordinates, CameraParams cp)
{
	if (pt.z > 0)
	{
		// compute image center and dimension
		// focal length is only for this resolution
		double sizeX = 640;
		double sizeY = 480;

		double cx, cy;

		cx = sizeX / 2.0 - 0.5;
		cy = sizeY / 2.0 - 0.5;

		double focal_x, focal_y;
		focal_x = cp.focal_x;
		focal_y = cp.focal_y;

		// project point on camera's image plane
		UV_coordinates(0) = static_cast<float> ((focal_x * (pt.x / pt.z) + cx) / (sizeX - 1) * (cp.color_width - 1)); //horizontal
		UV_coordinates(1) = static_cast<float> ((focal_y * (pt.y / pt.z) + cy) / (sizeY) * (sizeY*cp.color_width / sizeX)); //vertical

		if (cp.camera_ratio_diff)
			UV_coordinates(1) = UV_coordinates(1) + (cp.color_height - sizeY*(cp.color_width / sizeX)) / 2.0 - 2.8;

		// point is visible!
		if (UV_coordinates(0) >= 0.0 && UV_coordinates(0) <= cp.color_width - 1.0 && UV_coordinates(1) >= 0.0 && UV_coordinates(1) <= cp.color_height - 1.0) {
			return (true); // point was visible by the camera
		}
	}

	// point is NOT visible by the camera
	UV_coordinates(0) = -1.0f;
	UV_coordinates(1) = -1.0f;
	return (false); // point was not visible by the camera
}

bool
mapUVtoDepth(const Eigen::Vector2f &uv, unsigned short* depth_buffer, CameraParams cp) {
	if (uv.x() >= 5 && uv.x() <= cp.color_width - 6 && uv.y() >= 5 && uv.y() <= cp.color_height - 6) {
		float kofw = cp.depth_width / cp.color_width;
		float kofh = cp.depth_width / cp.color_width;
		int x = uv.x()* kofw;
		int y = (cp.camera_ratio_diff ? (uv.y() - (cp.color_height - cp.depth_height / kofw) / 2)*kofw : uv.y() * kofh);

		//cout << uv.x() <<' '<< uv.y() <<' ' << x << ' ' << y << endl;

		if (x >= 0 && y >= 0 && x < cp.depth_width && y < cp.depth_height) {
			unsigned short val = depth_buffer[y*(int)cp.depth_width + x];
			if (val == 0) return false;
			else return true;

		}
		else return false;

	}
	else return false;
}

void
computeDepthDiscont(unsigned short * dbuffer, CameraParams camparams) {
	float ** bw = filterBWdepth(dbuffer, camparams.depth_width, camparams.depth_height);
	float ** conv_depth = filterArrayScharr(bw, camparams.depth_width, camparams.depth_height);

	int delta = 3;

	for (int i = 0; i < camparams.depth_width; ++i) delete[] bw[i];
	delete[] bw;

	// TEST
	//	uchar *img = new uchar[(int)camparams.depth_width*(int)camparams.depth_height*3];

	std::queue<int> q_nulls;
	for (int i = 0; i < camparams.depth_width*camparams.depth_height; ++i) {

		// Test		
		/*		int val = min(1.0f,16.0f*conv_depth[i % (int)camparams.depth_width][i / (int)camparams.depth_width]) * 255.0f;
		img[i*3  ]=val;
		img[i*3+1]=val;
		img[i*3+2]=val;*/

		if (16.0f*conv_depth[i % (int)camparams.depth_width][i / (int)camparams.depth_width] >= 0.5f)
			dbuffer[i] = 0;

		if (dbuffer[i] == 0)
			q_nulls.push(i);
	}

	// Test
	//	QImage ni = QImage(img, camparams.depth_width, camparams.depth_height, QImage::Format_RGB888);
	//	ni.save(QString("test-depth.png"));

	while (!q_nulls.empty()) {
		int ind = q_nulls.front();
		q_nulls.pop();

		int x = ind % (int)camparams.depth_width;
		int y = ind / (int)camparams.depth_width;
		for (int i = -delta; i <= delta; ++i)
			for (int j = -delta; j <= delta; ++j) {
				if (sqrt((float)i*i + (float)j*j) <= delta &&  x + i >= 0 && x + i <= camparams.depth_width - 1 && y + j >= 0 && y + j <= camparams.depth_height - 1)
					dbuffer[(x + i) + (y + j)*(int)camparams.depth_width] = 0;
			}
	}

	for (int i = 0; i < camparams.depth_width; ++i) delete[] conv_depth[i];
	delete[] conv_depth;

	// TEST
	/*	conv_depth = filterBWdepth(dbuffer, camparams.depth_width, camparams.depth_height);
	for (int i = 0; i < camparams.depth_width*camparams.depth_height; ++i) {
	int val = min(1.0f,conv_depth[i % (int)camparams.depth_width][i / (int)camparams.depth_width]) * 255.0f;
	img[i*3  ]=val;
	img[i*3+1]=val;
	img[i*3+2]=val;
	}
	QImage ni2 = QImage(img, camparams.depth_width, camparams.depth_height, QImage::Format_RGB888);
	ni2.save(QString("test-depth2.png"));
	for (int i = 0; i < camparams.depth_width; ++i) delete [] conv_depth[i];
	delete [] conv_depth;*/
}

void
getTriangleCircumcscribedCircleCentroid(const Eigen::Vector2f &p1, const Eigen::Vector2f &p2, const Eigen::Vector2f &p3, Eigen::Vector2f &circumcenter, double &radius)
{
	// compute centroid's coordinates (translate back to original coordinates)
	circumcenter.x() = static_cast<float> (p1.x() + p2.x() + p3.x()) / 3;
	circumcenter.y() = static_cast<float> (p1.y() + p2.y() + p3.y()) / 3;
	double r1 = (circumcenter.x() - p1.x()) * (circumcenter.x() - p1.x()) + (circumcenter.y() - p1.y()) * (circumcenter.y() - p1.y());
	double r2 = (circumcenter.x() - p2.x()) * (circumcenter.x() - p2.x()) + (circumcenter.y() - p2.y()) * (circumcenter.y() - p2.y());
	double r3 = (circumcenter.x() - p3.x()) * (circumcenter.x() - p3.x()) + (circumcenter.y() - p3.y()) * (circumcenter.y() - p3.y());

	// radius
	radius = std::sqrt(std::max(r1, std::max(r2, r3)));
}

bool
checkPointInsideTriangle(const Eigen::Vector2f &p1, const Eigen::Vector2f &p2, const Eigen::Vector2f &p3, const Eigen::Vector2f &pt)
{
	// Compute vectors
	Eigen::Vector2d v0, v1, v2;
	v0(0) = p3.x() - p1.x(); v0(1) = p3.y() - p1.y(); // v0= C - A
	v1(0) = p2.x() - p1.x(); v1(1) = p2.y() - p1.y(); // v1= B - A
	v2(0) = pt.x() - p1.x(); v2(1) = pt.y() - p1.y(); // v2= P - A

	// Compute dot products
	double dot00 = v0.dot(v0); // dot00 = dot(v0, v0)
	double dot01 = v0.dot(v1); // dot01 = dot(v0, v1)
	double dot02 = v0.dot(v2); // dot02 = dot(v0, v2)
	double dot11 = v1.dot(v1); // dot11 = dot(v1, v1)
	double dot12 = v1.dot(v2); // dot12 = dot(v1, v2)

							   // Compute barycentric coordinates
	double invDenom = 1.0 / (dot00*dot11 - dot01*dot01);
	double u = (dot11*dot02 - dot01*dot12) * invDenom;
	double v = (dot00*dot12 - dot01*dot02) * invDenom;

	// Check if point is in triangle
	return ((u >= 0) && (v >= 0) && (u + v < 1));
}
