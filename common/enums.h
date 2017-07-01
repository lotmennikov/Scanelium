#ifndef ENUMS_LOT
#define ENUMS_LOT

/** Commonly used enums */
const float camera_distance = 0.4f;

enum ProgramState { INIT, KINFU, COLOR, FINAL }; 

enum CameraPose { CENTER, CENTERFACE, CENTEREDGE, VERTEX };

enum DepthResolution { DEPTH_QVGA = 0, DEPTH_VGA = 1 };

enum ColorResolution { COLOR_QVGA = 0, COLOR_VGA = 1, COLOR_SXGA = 2 };

enum CameraTask { PREPROCESS = 0, ALGORITHM = 1, POSTPROCESS = 2 };

#endif