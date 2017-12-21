#pragma once

#include "structs.h"
#include <vector>
#include <qmatrix4x4.h>

/** Mesh renderer interface */
class Renderer {

public:

	virtual bool render(QMatrix4x4 pose, iparams params, std::vector<float>& depth) = 0;

};