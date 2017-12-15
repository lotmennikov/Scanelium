#pragma once
#include <iostream>
#include <memory>

#include "frame.h"

class Model {
	int init_type;

	float* vertices;
	float* _colors;
	int* indices;

	int num_vertices;
	int num_indices;
	bool has_colors;
	
	std::vector<Frame*> frames;
public:
	
	struct PointXYZ {
		float x; float y; float z; float w;
		PointXYZ(float _x = 0, float _y = 0, float _z = 0, float _w = 1) : x(_x), y(_y), z(_z), w(_w) {}
	};

	struct Triangle {
		int p[3];
	};

	struct ColorRGB {
		float r, g, b;
	};
	
	static const int FLOATS_PER_POINT = 4;

	enum { INIT_NONE, INIT_VBO, INIT_IBO };

	typedef std::shared_ptr<Model> Ptr;

	Model();
	Model(const Model&);

	void setVBO(float* vertices, int num);
	void setIBO(float* vertices, int num_vertices, int* indices, int num_indices);
	void setColors(float* colors, int num);

	bool hasColor() const;
	bool isIBO() const;
	bool isVBO() const;

	int getVertsSize() const;
	int getIndsSize() const;
	int getFramesSize() const;
	Frame* getFrame(int num) const;

	void copyVertices(std::vector<PointXYZ>& verts);
	void copyTriangles(std::vector<Triangle>& trigs);
	void copyFrames(std::vector<Frame*>& cframes);

	void addFrame(Frame*);

	~Model();

// iterators

	class point_iterator {
		friend class Model;
		float* verts;
		float* colors;

		int index;	
		point_iterator(int it, float* verts, float* colors = NULL);
	public:
		void operator++();
		bool operator!=(const point_iterator& phit);
		PointXYZ operator*();
		ColorRGB getColor();
	};

	class triangle_iterator {
		friend class Model;
		int* inds;
		int index;
		triangle_iterator(int it, int* inds);
	public:
		void operator++();
		bool operator!=(const triangle_iterator& phit);
		Triangle operator*();
	};

	point_iterator verts_begin();
	point_iterator verts_end();

	triangle_iterator tri_begin();
	triangle_iterator tri_end();

};