#ifndef ALGOPARAMS_H_LOT
#define ALGOPARAMS_H_LOT

struct AlgoParams {

public :
	double stepx;
	double stepy;

	static const int gridsizex = 11;
	static const int gridsizey = 9;
	static const int dof6 = 6;
	static const int matrixdim = dof6 + gridsizex*gridsizey*2;

	bool img_correction;

	AlgoParams(bool use_img_correction = false, int width = 640, int height = 480) {
		img_correction = use_img_correction;
		stepx = (width - 1) / (double)(gridsizex - 1);
		stepy = (height - 1) / (double)(gridsizey - 1);
	}

};

#endif