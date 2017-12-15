#include "model.h"

// point iterator

Model::point_iterator::point_iterator(int ind, float* verts, float* colors) {
	this->index = ind;
	this->verts = verts;
	this->colors = colors;
}

void Model::point_iterator::operator++() {
	this->index++;
}

bool Model::point_iterator::operator!=(const Model::point_iterator& phit) {
	return this->index != phit.index;
}

Model::PointXYZ Model::point_iterator::operator*() {
	PointXYZ p = *(PointXYZ*)(&verts[this->index * 4]);
	return p;
}

Model::ColorRGB Model::point_iterator::getColor() {
	if (colors == NULL) {
		ColorRGB c; c.r = c.g = c.b = 0;
		return c;
	} else 
		return *(ColorRGB*)(&colors[this->index * 3]);
}

Model::point_iterator Model::verts_begin() {
	return Model::point_iterator(0, vertices, (has_colors ? _colors : NULL));
}

Model::point_iterator Model::verts_end() {
	return Model::point_iterator(num_vertices, vertices, (has_colors ? _colors : NULL));
}

// triangle iterator

Model::triangle_iterator::triangle_iterator(int ind, int* inds) {
	this->index = ind;
	this->inds = inds;
}

void Model::triangle_iterator::operator++() {
	this->index++;
}

bool Model::triangle_iterator::operator!=(const Model::triangle_iterator& phit) {
	return this->index != phit.index;
}

Model::Triangle  Model::triangle_iterator::operator*() {
	Triangle p = *(Triangle*)(&inds[this->index * 3]);
	return p;
}

Model::triangle_iterator Model::tri_begin() {
	return Model::triangle_iterator(0, indices);
}

Model::triangle_iterator Model::tri_end() {
	return Model::triangle_iterator(num_indices / 3, indices);
}