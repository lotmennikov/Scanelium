#include "model.h"

Model::Model() {
	init_type = INIT_NONE;
	has_colors = false;

	vertices = NULL;
	indices = NULL;
	_colors = NULL;
	num_vertices = 0;
	num_indices = 0;
}

Model::Model(const Model& m) {
	init_type = m.init_type;
	if (init_type == INIT_NONE) {
		vertices = NULL;
		indices = NULL;
		_colors = NULL;
		num_vertices = 0;
		num_indices = 0;
	}
	else {
		num_vertices = m.num_vertices;
		vertices = new float[4 * num_vertices];
		std::copy(m.vertices, m.vertices + 4 * num_vertices, vertices);

		num_indices = m.num_indices;
		indices = new int[num_indices];
		std::copy(m.indices, m.indices + num_indices, indices);

		has_colors = m.has_colors;
		if (has_colors) {
			_colors = new float[3*num_vertices];
			std::copy(m._colors, m._colors + (3*num_vertices), _colors);
		}
		else
			_colors = NULL;
	}

	frames.clear();
	for (int i = 0; i < m.frames.size(); ++i) {
		frames.push_back(new Frame(*m.frames[i]));
	}

}

void Model::setVBO(float* vertices, int num) {
	init_type = INIT_VBO;
	this->num_vertices = num;
	this->vertices = new float[4 * num_vertices];
	std::copy(vertices, vertices + 4 * num_vertices, this->vertices);

	this->num_indices = num;
	indices = new int[num_indices];
	for (int i = 0; i < num_indices; ++i) indices[i] = i;
}

void Model::setIBO(float* vertices, int num_vertices, int* indices, int num_indices) {
	init_type = INIT_IBO;
	this->num_vertices = num_vertices;
	this->vertices = new float[4 * num_vertices];
	std::copy(vertices, vertices + 4 * num_vertices, this->vertices);

	this->num_indices = num_indices;
	this->indices = new int[num_indices];
	std::copy(indices, indices + num_indices, this->indices);
}

void Model::setColors(float* colors, int num_colors) {
	assert(num_vertices == num_colors);

	this->_colors = new float[3 * num_vertices];
	std::copy(colors, colors + (3 * num_vertices), _colors);

	has_colors = true;
}

bool Model::hasColor() const { return has_colors; }
bool Model::isIBO() const { return init_type == INIT_IBO; };
bool Model::isVBO() const { return init_type == INIT_VBO; };

int Model::getVertsSize() const {
	return num_vertices;
}

int Model::getIndsSize() const {
	return num_indices;
}

int Model::getFramesSize() const { return frames.size(); }

Frame* Model::getFrame(int num) const { return frames[num]; }

void Model::copyVertices(std::vector<Model::PointXYZ>& verts) {
	verts.resize(num_vertices);
	std::copy((Model::PointXYZ*)vertices, (Model::PointXYZ*)vertices + num_vertices, verts.begin());
}

void Model::copyTriangles(std::vector<Model::Triangle>& trigs) {
	trigs.resize(num_indices/3);
	std::copy((Model::Triangle*)indices, (Model::Triangle*)indices + num_indices/3, trigs.begin());
}

void Model::copyFrames(std::vector<Frame*>& cframes) {
	cframes.resize(this->frames.size());
	std::copy(this->frames.begin(), this->frames.end(), cframes.begin());
}

void Model::addFrame(Frame* f) {
	frames.push_back(f);
}

Model::~Model() {
	if (init_type != INIT_NONE) {
		if (vertices != NULL) delete[] vertices;
		if (indices != NULL) delete[] indices;
		if (has_colors && _colors != NULL) delete[] _colors;
	}
	if (!frames.empty())
		while (!frames.empty()) {
			delete frames.back();
			frames.pop_back();
		}
}