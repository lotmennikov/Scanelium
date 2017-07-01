#include "glwidget.h"

QMutex glWidget::texture_mutex;

glWidget::glWidget(QWidget *parent) :
    QGLWidget(parent)
{
	mVaoGrid = NULL;
	mVaoMesh = NULL;


	camLook = QVector3D(0,0,0);
    angle1 = 180.0f;
    angle2 = 0.0f;
    distance  = 2.0f;
    angleChanged = false;
	movebegin = false;
	initialized = false;
	painting = false;
	col = QColor(50, 0, 160);

	volume_size = 1;
	doubleY = false;
	campose = CameraPose::CENTERFACE;

	points = QVector<float>();
	points.reserve(8*3);
	float cube_size = 0.5f;//half edge length
	points << -cube_size << -cube_size << -cube_size
		   << -cube_size << -cube_size <<  cube_size
		   <<  cube_size << -cube_size << -cube_size
		   <<  cube_size << -cube_size <<  cube_size
		   << -cube_size <<  cube_size << -cube_size
		   << -cube_size <<  cube_size <<  cube_size
		   <<  cube_size <<  cube_size << -cube_size
		   <<  cube_size <<  cube_size <<  cube_size;
	inds.reserve(12*2);
	inds << 0 << 1 << 0 << 2 << 1 << 3 << 2 << 3
		 << 4 << 5 << 4 << 6 << 5 << 7 << 6 << 7
		 << 0 << 4 << 1 << 5 << 2 << 6 << 3 << 7;

	imgQuad << QVector3D(0,0,1) 
			<< QVector3D(0,1,1)
			<< QVector3D(1,1,1)
			<< QVector3D(1,0,1);
	imgTexCoord << QVector2D(0, 0)
				<< QVector2D(0, 1)
				<< QVector2D(1, 1)
				<< QVector2D(1, 0);

	img = QPixmap(QSize(640, 480));
	img.fill(QColor(Qt::green));

	state = ProgramState::INIT;
}

//void glWidget::remakeVolumeCube(int size) {
//	this->volume_size = size;
//
//}

void glWidget::resizeGL(int width, int height) {
    if (height == 0)
        height = 1;

    pMatrix.setToIdentity();
    pMatrix.perspective(60.0, (float)width / (float)height, 0.001, 1000);

    glViewport(0,0, width,height);
}

void glWidget::initializeGL() {

	initializeOpenGLFunctions();
    
	qglClearColor(col);
    glEnable(GL_DEPTH_TEST);
//	glEnable(GL_CULL_FACE);

	normalProgram.addShaderFromSourceFile(QGLShader::Vertex, ":/shader/normalVertexShader.vsh");
	normalProgram.addShaderFromSourceFile(QGLShader::Fragment, ":/shader/normalFragmentShader.fsh");
	normalProgram.link();

	colorProgram.addShaderFromSourceFile(QGLShader::Vertex, ":/shader/colorVertexShader.vsh");
	colorProgram.addShaderFromSourceFile(QGLShader::Fragment, ":/shader/colorFragmentShader.fsh");
	colorProgram.link();

	textureProgram.addShaderFromSourceFile(QGLShader::Vertex, ":/shader/textureVertexShader.vsh");
	textureProgram.addShaderFromSourceFile(QGLShader::Fragment, ":/shader/textureFragmentShader.fsh");
	textureProgram.link();
	textureProgram.bind();

	texture = bindTexture(img, GL_TEXTURE_2D);
	textureProgram.release();

	lineProgram.addShaderFromSourceFile(QGLShader::Vertex, ":/shader/simpleVertexShader.vsh");
	lineProgram.addShaderFromSourceFile(QGLShader::Fragment, ":/shader/simpleFragmentShader.fsh");
	lineProgram.link();

	lineProgram.bind();
	
	mVaoGrid = new QOpenGLVertexArrayObject(this);
	mVaoGrid->create();
	mVaoGrid->bind();

	gridBuffer.create();
    gridBuffer.setUsagePattern( QOpenGLBuffer::StreamDraw );
	gridBuffer.bind();
	gridBuffer.allocate(points.constData(), points.size()*sizeof(float));
//	lineProgram.enableAttributeArray("vertex");
//	lineProgram.setAttributeBuffer("vertex", GL_FLOAT, 0, 3);
	gridBuffer.release();

	indBuffer = QOpenGLBuffer(QOpenGLBuffer::IndexBuffer);
	indBuffer.create();
    indBuffer.setUsagePattern( QOpenGLBuffer::StreamDraw);
	indBuffer.bind();
	indBuffer.allocate(inds.constData(), inds.size()*sizeof(unsigned int));
	indBuffer.release();
	mVaoGrid->release();
	
	lineProgram.release();

}
/*
bool glWidget::resetBuffers(QVector<color_vertex> vert, QVector<unsigned int> ind) {
//	QGLFunctions::glDisableVertexAttribArray(VERTICES);
//	
	
	while (painting) {}

	initialized = false;

	this->vertices_ = QVector<color_vertex>(vert);
	this->indices_ = QVector<unsigned int>(ind);

	colorProgram.bind();

 

	mVao = new QOpenGLVertexArrayObject( this );
    mVao->create();
    mVao->bind();

    // Setup VBOs and IBO (use QOpenGLBuffer to buffer data,
    // specify format, usage hint etc). These will be
    // remembered by the currently bound VAO
    mVertex.create();
    mVertex.setUsagePattern( QOpenGLBuffer::StreamDraw );
    mVertex.bind();
	mVertex.allocate(vertices_.constData(), vertices_.size()*sizeof(color_vertex));
	colorProgram.enableAttributeArray("vertex");
    colorProgram.setAttributeBuffer("vertex", GL_FLOAT, 0, 3, sizeof(color_vertex));
	
	colorProgram.enableAttributeArray("color");
    colorProgram.setAttributeBuffer("color", GL_FLOAT, sizeof(float)*3, 3, sizeof(color_vertex));

	mIndex = QOpenGLBuffer(QOpenGLBuffer::IndexBuffer);
	mIndex.create();
	mIndex.setUsagePattern(QOpenGLBuffer::StreamDraw);
	mIndex.bind();
	mIndex.allocate(indices_.constData(), indices_.size()*sizeof(unsigned int));

	mVao->release();

	colorProgram.release();

	initialized = true;

	this->updateGL();

	return true;
}
*/

void glWidget::computeQuadVertices() {
	//QMatrix4x4 coords;
	QVector4D c0 = pMatrix.inverted() * QVector4D(-1, -1, 0, 1);
	QVector4D c1 = pMatrix.inverted() * QVector4D(-1, 1, 0, 1); 
	QVector4D c2 = pMatrix.inverted() * QVector4D(1, 1, 0, 1);
	QVector4D c3 = pMatrix.inverted() * QVector4D(1, -1, 0, 1);
	

	imgQuad[0] = c0.toVector3D(); // c0.w;
	imgQuad[1] = c1.toVector3D(); // c1.w;
	imgQuad[2] = c2.toVector3D(); // c2.w;
	imgQuad[3] = c3.toVector3D(); // c3.w;


}


void glWidget::paintGL() {

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	painting = true;

	mMatrix = QMatrix4x4();
	vMatrix = QMatrix4x4();
	QMatrix4x4 camTransform;
	camTransform.rotate(angle1, 0, 1, 0);
	camTransform.rotate(angle2, 1, 0, 0);

//	mMatrix.translate(0.5, -0.5, -0.5);

	QVector3D cameraPosition = camTransform * (QVector3D(0, 0, distance)) + camLook;
	QVector3D cameraUpDirection = camTransform * QVector3D(0,1,0);

	vMatrix.lookAt(cameraPosition, camLook, cameraUpDirection);	
	
	switch (state) {
	case INIT:
		{
			// cube
			lineProgram.bind();

			mVaoGrid->bind();
			gridBuffer.bind();
			indBuffer.bind();
			mMatrix = QMatrix4x4();
			mMatrix.scale(volume_size, (doubleY ? volume_size*2 : volume_size), volume_size);

			lineProgram.setUniformValue("mvpMatrix", pMatrix * vMatrix * mMatrix);
			lineProgram.setUniformValue("color", QColor(255, 255, 255));
			lineProgram.setAttributeBuffer("vertex", GL_FLOAT, 0, 3);
			lineProgram.enableAttributeArray("vertex");
			glDrawElements(GL_LINES, inds.size(), GL_UNSIGNED_INT, 0);
			lineProgram.disableAttributeArray("vertex");
	
			indBuffer.release();
			gridBuffer.release();
			mVaoGrid->release();
			lineProgram.release();
			
			if (cloud_points.size() > 0) {
				cloud_mutex.lock();
				
				mMatrix = QMatrix4x4();
				mMatrix(0,0) = -1;
				mMatrix(1,1) = -1;
				switch (campose) {
				case CENTER:
					mMatrix.translate(0, 0, 0);
					break;
				case CENTERFACE:
					mMatrix.translate(0, 0, -volume_size / 2 - camera_distance);
					break;
				default:
					break;
				}

				colorProgram.bind();
				colorProgram.setUniformValue("mvpMatrix", pMatrix * vMatrix * mMatrix);
				colorProgram.setAttributeArray("vertex", cloud_points.constData());
				colorProgram.setAttributeArray("color", cloud_colors.constData());
				colorProgram.enableAttributeArray("vertex");
				colorProgram.enableAttributeArray("color");
				glDrawArrays(GL_POINTS, 0, cloud_points.size());
				colorProgram.disableAttributeArray("color");
				colorProgram.disableAttributeArray("vertex");
				colorProgram.release();

				cloud_mutex.unlock();
			}
		}
		break;
	case KINFU:
		{
			computeQuadVertices();

			// front img
			textureProgram.bind();

			glBindTexture(GL_TEXTURE_2D, this->texture);

			textureProgram.setUniformValue("mvpMatrix", pMatrix);

			textureProgram.setAttributeArray("vertex", imgQuad.constData());
			textureProgram.setAttributeArray("texcoord", imgTexCoord.constData());
			textureProgram.enableAttributeArray("vertex");
			textureProgram.enableAttributeArray("texcoord");
			glDrawArrays(GL_QUADS, 0, imgQuad.size());
			textureProgram.disableAttributeArray("texcoord");
			textureProgram.disableAttributeArray("vertex");

			textureProgram.release();
		}
		break;
	case COLOR:
		{
			if (initialized) {
				cloud_mutex.lock();

				normalProgram.bind();

				mVaoMesh->bind();
				meshBuffer.bind();
				meshindBuffer.bind();

				mMatrix = QMatrix4x4();
				mMatrix(0,0) = -1;
				mMatrix(1,1) = -1;
				mMatrix.translate(-volume_size / 2, -volume_size / 2 * (doubleY ? 2 : 1), -volume_size / 2);

				normalProgram.setUniformValue("mvpMatrix", pMatrix * vMatrix * mMatrix);
				normalProgram.setAttributeBuffer("vertex", GL_FLOAT, 0, 3, sizeof(color_vertex));
				normalProgram.enableAttributeArray("vertex");
				normalProgram.setAttributeBuffer("normal", GL_FLOAT, sizeof(float)*3, 3, sizeof(color_vertex));
				normalProgram.enableAttributeArray("normal");
				glDrawElements(GL_TRIANGLES, mesh_inds.size(), GL_UNSIGNED_INT, 0);
				normalProgram.disableAttributeArray("vertex");
				normalProgram.disableAttributeArray("normal");
	
				meshindBuffer.release();
				meshBuffer.release();
				mVaoMesh->release();

				normalProgram.release();
			
				cloud_mutex.unlock();
			}
		}
		break;
	case FINAL:
		{
			if (initialized) {
				cloud_mutex.lock();

				colorProgram.bind();

				mVaoMesh->bind();
				meshBuffer.bind();
				meshindBuffer.bind();

				mMatrix = QMatrix4x4();
				mMatrix(0,0) = -1;
				mMatrix(1,1) = -1;
				mMatrix.translate(-volume_size / 2, -volume_size / 2 * (doubleY ? 2 : 1), -volume_size / 2);

				colorProgram.setUniformValue("mvpMatrix", pMatrix * vMatrix * mMatrix);
				colorProgram.setAttributeBuffer("vertex", GL_FLOAT, 0, 3, sizeof(color_vertex));
				colorProgram.enableAttributeArray("vertex");
				colorProgram.setAttributeBuffer("color", GL_FLOAT, sizeof(float)*3, 3, sizeof(color_vertex));
				colorProgram.enableAttributeArray("color");
				glDrawElements(GL_TRIANGLES, mesh_inds.size(), GL_UNSIGNED_INT, 0);
				colorProgram.disableAttributeArray("vertex");
	
				meshindBuffer.release();
				meshBuffer.release();
				mVaoMesh->release();

				colorProgram.release();
			
				cloud_mutex.unlock();
			}
		}
		break;
	default:
		break;
	}

	angleChanged = false;

	painting = false;
}

glWidget::~glWidget() {
	if (mVaoGrid != NULL) {
		mVaoGrid->bind();
		mVaoGrid->destroy();
		mVaoGrid->release();
	}
	delete mVaoGrid;
}

void glWidget::mousePressEvent(QMouseEvent * event) {
    lastPosition = QVector2D(event->x(), event->y());
    movebegin = true;

}

void glWidget::mouseMoveEvent(QMouseEvent * event) {
	if (movebegin) {
		angle1 = angle1 + (lastPosition.x() - event->x());
		angle2 = angle2 + (lastPosition.y() - event->y());
		lastPosition = QVector2D(event->x(), event->y());
		angleChanged = true;
		this->updateGL();
	} else {
		while (painting) {}
		float mx = event->x();
		float my = event->y();
		float mz = 0;
		glReadPixels( mx, my, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &mz );
		
		//if (log != NULL && initialized)
			//log->insertPlainText(QString::number(mx) + " " + QString::number(my) + " " + QString::number(mz) + "\n");
	}
}

void glWidget::mouseReleaseEvent(QMouseEvent *) {
    movebegin = false;
}

void glWidget::wheelEvent(QWheelEvent *event) {
    int delta = event->delta();
    if (event->orientation() == Qt::Vertical) {
        if (delta > 0) {
            if (distance > 0.1) distance *=0.9f;
        } else {
            distance *=1.1f;
        }
        this->updateGL();
    }
}

void glWidget::refreshTexture(const QImage& img) {

//	int i = 0;
//	i++;
//	img.save(QString("depth.png"));

	
//	this->img.detach();
	this->img = QPixmap::fromImage(img);
	this->deleteTexture(this->texture);
	this->texture = this->bindTexture(this->img, GL_TEXTURE_2D);
	
	
	this->updateGL();
}

void glWidget::refreshCloud(QVector<QVector3D>* pnts, QVector<QVector3D>* clrs) {
	cloud_mutex.lock();
	this->cloud_points = (*pnts);
	this->cloud_colors = (*clrs);
	delete pnts;
	delete clrs;
	cloud_mutex.unlock();
	this->updateGL();
}

void glWidget::setPolygonMesh(pcl::PolygonMesh::Ptr mesh, bool color) {
	cloud_mutex.lock();
	initialized = false;

// init arrays
	mesh_points.clear();
	mesh_inds.clear();

	std::vector<int> face_per_point;

	if (color) {
		pcl::PointCloud<pcl::PointXYZRGB> cloud;
		pcl::fromPCLPointCloud2(mesh->cloud, cloud);
		mesh_points.reserve(cloud.points.size());

		for (int i = 0; i < cloud.points.size(); ++i) {
			color_vertex vert;
			vert.vertex[0] = cloud.points[i].x;
			vert.vertex[1] = cloud.points[i].y;
			vert.vertex[2] = cloud.points[i].z;

			vert.color[0] = cloud.points[i].r / 255.0f;
			vert.color[1] = cloud.points[i].g / 255.0f;
			vert.color[2] = cloud.points[i].b / 255.0f;

			mesh_points.push_back(vert);
		}
	} else {
		pcl::PointCloud<pcl::PointXYZ> cloud;
		pcl::fromPCLPointCloud2(mesh->cloud, cloud);
		mesh_points.reserve(cloud.points.size());
		face_per_point.resize(cloud.points.size(), 0);

		for (int i = 0; i < cloud.points.size(); ++i) {
			color_vertex vert;
			vert.vertex[0] = cloud.points[i].x;
			vert.vertex[1] = cloud.points[i].y;
			vert.vertex[2] = cloud.points[i].z;

			vert.color[0] = 0.0f;
			vert.color[1] = 0.0f;
			vert.color[2] = 0.0f;

			mesh_points.push_back(vert);
		}
	}
	mesh_inds.reserve(mesh->polygons.size()*3);
	auto it = mesh->polygons.begin();
	for (; it != mesh->polygons.end(); ++it) {
		uint ind0 = (*it).vertices[0];
		uint ind1 = (*it).vertices[1];
		uint ind2 = (*it).vertices[2];

		mesh_inds.push_back(ind0);
		mesh_inds.push_back(ind1);
		mesh_inds.push_back(ind2);

		if (!color) {
			float A = mesh_points[ind0].vertex[1] * (mesh_points[ind1].vertex[2] - mesh_points[ind2].vertex[2]) + mesh_points[ind1].vertex[1] * (mesh_points[ind2].vertex[2] - mesh_points[ind0].vertex[2]) + mesh_points[ind2].vertex[1] * (mesh_points[ind0].vertex[2] - mesh_points[ind1].vertex[2]); 
			float B = mesh_points[ind0].vertex[2] * (mesh_points[ind1].vertex[0] - mesh_points[ind2].vertex[0]) + mesh_points[ind1].vertex[2] * (mesh_points[ind2].vertex[0] - mesh_points[ind0].vertex[0]) + mesh_points[ind2].vertex[2] * (mesh_points[ind0].vertex[0] - mesh_points[ind1].vertex[0]); 
			float C = mesh_points[ind0].vertex[0] * (mesh_points[ind1].vertex[1] - mesh_points[ind2].vertex[1]) + mesh_points[ind1].vertex[0] * (mesh_points[ind2].vertex[1] - mesh_points[ind0].vertex[1]) + mesh_points[ind2].vertex[0] * (mesh_points[ind0].vertex[1] - mesh_points[ind1].vertex[1]);

			mesh_points[ind0].color[0] += A;
			mesh_points[ind0].color[1] += B;
			mesh_points[ind0].color[2] += C;
			face_per_point[ind0]++;
			
			mesh_points[ind1].color[0] += A;
			mesh_points[ind1].color[1] += B;
			mesh_points[ind1].color[2] += C;
			face_per_point[ind1]++;
			
			mesh_points[ind2].color[0] += A;
			mesh_points[ind2].color[1] += B;
			mesh_points[ind2].color[2] += C;
			face_per_point[ind2]++;
		}
	}
	if (!color) {
		for (int i = 0; i < face_per_point.size(); ++i)
		{
			if (face_per_point[i] > 0) {
				mesh_points[i].color[0] /= face_per_point[i];
				mesh_points[i].color[1] /= face_per_point[i];
				mesh_points[i].color[2] /= face_per_point[i];
			}
		}
	}
// ----------
	if (mVaoMesh != NULL) 
		delete mVaoMesh;

	mVaoMesh = new QOpenGLVertexArrayObject( this );
    mVaoMesh->create();
    mVaoMesh->bind();

    // Setup VBOs and IBO (use QOpenGLBuffer to buffer data,
    // specify format, usage hint etc). These will be
    // remembered by the currently bound VAO
	meshBuffer.create();
    meshBuffer.setUsagePattern( QOpenGLBuffer::StreamDraw );
    meshBuffer.bind();
	meshBuffer.allocate(mesh_points.constData(), mesh_points.size()*sizeof(color_vertex));
	meshBuffer.release();
//	colorProgram.enableAttributeArray("vertex");
//    colorProgram.setAttributeBuffer("vertex", GL_FLOAT, 0, 3, sizeof(color_vertex));
	
//	colorProgram.enableAttributeArray("color");
//    colorProgram.setAttributeBuffer("color", GL_FLOAT, sizeof(float)*3, 3, sizeof(color_vertex));

	meshindBuffer = QOpenGLBuffer(QOpenGLBuffer::IndexBuffer);
	meshindBuffer.create();
	meshindBuffer.setUsagePattern(QOpenGLBuffer::StreamDraw);
	meshindBuffer.bind();
	meshindBuffer.allocate(mesh_inds.constData(),mesh_inds.size()*sizeof(unsigned int));
	meshindBuffer.release();

	mVaoMesh->release();

//	colorProgram.release();
	initialized = true;
	cloud_mutex.unlock();
	if (color) {
		this->state = FINAL;
	} else {
		this->state = COLOR;
	}
	this->updateGL();
}