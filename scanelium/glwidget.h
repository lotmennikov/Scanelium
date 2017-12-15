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
/*
typedef struct {
	float vertex[3];
	float normal[3];
} normal_vertex;
*/
class glWidget : public QGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT
private:

    QGLShaderProgram textureProgram;
    QGLShaderProgram lineProgram;
	QGLShaderProgram normalProgram;
	QGLShaderProgram colorProgram;

	QOpenGLVertexArrayObject* mVaoGrid;
	QOpenGLBuffer gridBuffer;
	QOpenGLBuffer indBuffer;

	QOpenGLVertexArrayObject* mVaoMesh;
	QOpenGLBuffer meshBuffer;
	QOpenGLBuffer meshindBuffer;

    bool movebegin;

    bool angleChanged;

	bool initialized;
	bool painting;
	bool draw_color;

    float angle1, angle2;
    float distance;

    QVector2D lastPosition;

    QVector3D camLook;

    QMatrix4x4 pMatrix;
	QMatrix4x4 mMatrix;
	QMatrix4x4 vMatrix;

	QColor col;

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

//	QVector<color_vertex> vertices_;
//	QVector<unsigned int> indices_;

//	QPlainTextEdit* log;
	QVector3D offset;
	QVector<QMatrix4x4> camposes;
	QVector<float> cam_points;
	QVector<unsigned int> cam_inds;
	QOpenGLVertexArrayObject* mVaoCam;
	QOpenGLBuffer camBuffer;
	QOpenGLBuffer camindBuffer;
	void drawCamera(QMatrix4x4 pose, QColor color);

public:
    explicit glWidget(QWidget *parent = 0);

	static QMutex texture_mutex;
	QMutex cloud_mutex;

	ProgramState state;
	rec_settings _rec_set;

	~glWidget();
    
	void resizeGL(int, int);
    void initializeGL();
    void paintGL();

	void computeQuadVertices();
//	void setLog(QPlainTextEdit*);
//	bool resetBuffers(QVector<color_vertex>, QVector<unsigned int>); 

    void mouseMoveEvent(QMouseEvent *);
    void mousePressEvent(QMouseEvent *);
    void mouseReleaseEvent(QMouseEvent *);

    void wheelEvent(QWheelEvent *);

signals:

public slots :
	void stateChanged(ProgramState);
	void refreshRecSettings(rec_settings);
	void refreshTexture(const QImage& img);
	void refreshCloud(QVector<QVector3D>, QVector<QVector3D>);
	void setPolygonMesh(Model::Ptr);

	void newCameraPose(QMatrix4x4);
};

#endif // GLWIDGET_H_LOT
