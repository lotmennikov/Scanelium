#ifndef ENUMS_LOT
#define ENUMS_LOT

/** Commonly used enums */

enum ProgramState { NONE = -1, INIT = 0, KINFU = 1, COLOR = 2, FINAL = 3 }; 

enum CameraPose { CENTER, CENTERFACE, CENTEREDGE, VERTEX };

enum DepthResolution { DEPTH_QVGA = 0, DEPTH_VGA = 1 };

enum ColorResolution { COLOR_QVGA = 0, COLOR_VGA = 1, COLOR_SXGA = 2 };

enum CameraTask { PREPROCESS = 0, ALGORITHM = 1, POSTPROCESS = 2 };

typedef unsigned short* DepthMap;

typedef unsigned char* ImageRGB;

#endif