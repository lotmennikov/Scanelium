#ifndef ALGOPARAMS_H_LOT
#define ALGOPARAMS_H_LOT


const double eps = 0.000000001;


class AlgoParams {

public :
	double stepx;
	double stepy;

	static const int gridsizex = 11;
	static const int gridsizey = 9;
	static const int dof6 = 6;
	static const int matrixdim = dof6 + gridsizex*gridsizey*2;

	AlgoParams() {};

	AlgoParams(double sx, double sy) : stepx(sx), stepy(sy) {} 

};

#endif