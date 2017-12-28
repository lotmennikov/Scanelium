#ifndef GLWIDGET_H_LOT
#define GLWIDGET_H_LOT

#include <QtOpenGL>
#include <QGLWidget>
#include <QtCore/qmutex.h>
#include <QMouseEvent>
#include <QKeyEvent>
#include <QWheelEvent>
#include <QPlainTextEdit>
#include <QOpenGLVertexArrayObject>
#include <qopenglfunctions.h>
#include <qopenglbuffer.h>
#include <qvector3d.h>
#include <qmatrix4x4.h>
#include <QtOpenGL/QGLShaderProgram>

#include <memory>

#include "structs.h"
#include "model.h"

typedef struct {
	float vertex[3];
	float color[3];
} color_vertex;

typedef color_vertex cn_vertex;

class glWidget : public QGLWidget, protected QOpenGLFunctions
{
	Q_OBJECT
private:
	QMutex texture_mutex;
	QMutex data_mutex;

	ProgramState state;
	rec_settings _rec_set;

// Programs
    QGLShaderProgram textureProgram;
    QGLShaderProgram lineProgram;
	QGLShaderProgram normalProgram;
	QGLShaderProgram colorProgram;
	QGLShaderProgram depthProgram;

	QGLFramebufferObject* depthFrameBuffer;
// Buffers
	QOpenGLBuffer gridBuffer;
	QOpenGLBuffer indBuffer;

	QOpenGLBuffer meshBuffer;
	QOpenGLBuffer meshindBuffer;

	QOpenGLBuffer camBuffer;
	QOpenGLBuffer camindBuffer;

// Scene rotation
    bool movebegin;
    bool angleChanged;
	float angle1, angle2;
	float distance;
	QVector2D lastPosition;
	QVector3D camLook;

	bool initialized;
	bool painting;
	bool draw_color;

    QMatrix4x4 pMatrix;
	QMatrix4x4 mMatrix;
	QMatrix4x4 vMatrix;

// Data
	QVector<float> points;
	QVector<unsigned int> inds;

	QVector<color_vertex> mesh_points;
	QVector<unsigned int> mesh_inds;

	QVector<QVector3D> cloud_points;
	QVector<QVector3D> cloud_colors;

	QVector<QVector3D> imgQuad;
	QVector<QVector2D> imgTexCoord;

	GLuint texture;
	QPixmap img;

	QVector3D offset;
	QVector<QMatrix4x4> camposes;

	float cam_ratio, cam_size;
	QVector<QVector<QVector3D>> camgrids;

	QVector<float> cam_points;
	QVector<unsigned int> cam_inds;


	void drawGrid();
	void drawFrame();
	void drawCamera(QMatrix4x4 pose, QColor color, int num = -1);
	void drawCloud();
	void drawMesh();

	void computeQuadVertices();

public:
    explicit glWidget(QWidget *parent = 0);
	~glWidget();
    
	void resizeGL(int, int);
    void initializeGL();
    void paintGL();

    void mouseMoveEvent(QMouseEvent *);
    void mousePressEvent(QMouseEvent *);
    void mouseReleaseEvent(QMouseEvent *);
    void wheelEvent(QWheelEvent *);

signals:
	void renderFinished(bool res, std::vector<float> dpt);

public slots :
	void stateChanged(ProgramState);
	void refreshRecSettings(rec_settings);
	void refreshTexture(const QImage& img);
	void refreshCloud(QVector<QVector3D>, QVector<QVector3D>);
	void setPolygonMesh(Model::Ptr);

	void newCameraPose(QMatrix4x4);
	void setCameraImgGrid(std::vector<std::vector<float>> grids, int grid_x, int grid_y);
	// renderer
	void render(QMatrix4x4 pose, iparams params);
};

#endif // GLWIDGET_H_LOT
