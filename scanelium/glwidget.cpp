#include "glwidget.h"
#include "utils.h"


#define printGlError(a) if (a != GL_NO_ERROR) printf("GL Error %d\n", a);


glWidget::glWidget(QWidget *parent) :
    QGLWidget(parent)
{
	camLook = QVector3D(0,0,0);
    angle1 = 180.0f;
    angle2 = 0.0f;
    distance  = 2.0f;
    angleChanged = false;
	movebegin = false;
	initialized = false;
	draw_color = false;
	painting = false;

	_rec_set.volume_size = 1;
	_rec_set.doubleY = false;
	_rec_set.camera_pose = CameraPose::CENTERFACE;
	_rec_set.camera_distance = 0.4f;

// camera points
	camposes = QVector<QMatrix4x4>();
	cam_points = QVector<float>();
	cam_points.reserve(5 * 3);

	float cam_ratio = 640.f / 480.f;//half edge length
	float cam_size = 0.05f;
	cam_points << 0 << 0 << 0
		<< -cam_ratio * cam_size << -cam_size << cam_size*1.5f
		<< -cam_ratio * cam_size << cam_size << cam_size*1.5f
		<< cam_ratio * cam_size << -cam_size << cam_size*1.5f
		<< cam_ratio * cam_size << cam_size << cam_size*1.5f;
	cam_inds.reserve(8 * 2);
	cam_inds << 0 << 1 << 0 << 2 << 0 << 3 << 0 << 4
		<< 1 << 2 << 2 << 4 << 1 << 3 << 3 << 4;


// cube points
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

	depthFrameBuffer = NULL;
}

glWidget::~glWidget() {
	if (depthFrameBuffer != NULL) delete depthFrameBuffer;
}

void glWidget::resizeGL(int width, int height) {
    if (height == 0)
        height = 1;

    pMatrix.setToIdentity();
    pMatrix.perspective(60.0, (float)width / (float)height, 0.001, 1000);

    glViewport(0,0, width,height);
}

void glWidget::initializeGL() {

	initializeOpenGLFunctions();
    
	qglClearColor(QColor(50, 0, 160));
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

	texture = bindTexture(img, GL_TEXTURE_2D);
	textureProgram.release();

	lineProgram.addShaderFromSourceFile(QGLShader::Vertex, ":/shader/simpleVertexShader.vsh");
	lineProgram.addShaderFromSourceFile(QGLShader::Fragment, ":/shader/simpleFragmentShader.fsh");
	lineProgram.link();

	depthProgram.addShaderFromSourceFile(QGLShader::Vertex, ":/shader/depth.vsh");
	depthProgram.addShaderFromSourceFile(QGLShader::Fragment, ":/shader/depth.fsh");
	depthProgram.link();

// camera buffers
	camBuffer = QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
	camBuffer.create();
	camBuffer.setUsagePattern(QOpenGLBuffer::StreamDraw);
	camBuffer.bind();
	camBuffer.allocate(cam_points.constData(), cam_points.size() * sizeof(float));
	camBuffer.release();

	camindBuffer = QOpenGLBuffer(QOpenGLBuffer::IndexBuffer);
	camindBuffer.create();
	camindBuffer.setUsagePattern(QOpenGLBuffer::StreamDraw);
	camindBuffer.bind();
	camindBuffer.allocate(cam_inds.constData(), cam_inds.size() * sizeof(unsigned int));
	camindBuffer.release();

// grid buffers

	gridBuffer = QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
	gridBuffer.create();
    gridBuffer.setUsagePattern( QOpenGLBuffer::StreamDraw );
	gridBuffer.bind();
	gridBuffer.allocate(points.constData(), points.size()*sizeof(float));
	gridBuffer.release();

	indBuffer = QOpenGLBuffer(QOpenGLBuffer::IndexBuffer);
	indBuffer.create();
    indBuffer.setUsagePattern( QOpenGLBuffer::StreamDraw);
	indBuffer.bind();
	indBuffer.allocate(inds.constData(), inds.size()*sizeof(unsigned int));
	indBuffer.release();

	glPointSize(2.0f);

	depthFrameBuffer = new QGLFramebufferObject(QSize(640, 480), QGLFramebufferObject::Depth, GL_TEXTURE_2D, GL_R32F);
}

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

// DRAW

void glWidget::paintGL() {
	qglClearColor(QColor(50, 0, 160));

	//glDisable(GL_CULL_FACE);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	painting = true;

	mMatrix = QMatrix4x4();
	vMatrix = QMatrix4x4();
	QMatrix4x4 camTransform;
	camTransform.rotate(angle1, 0, 1, 0);
	camTransform.rotate(angle2, 1, 0, 0);

	QVector3D cameraPosition = camTransform * (QVector3D(0, 0, distance)) + camLook;
	QVector3D cameraUpDirection = camTransform * QVector3D(0,1,0);

	vMatrix.lookAt(cameraPosition, camLook, cameraUpDirection);	

	switch (state) {
	case INIT:
		drawGrid();
		drawCloud();
		break;
	case KINFU:
		drawFrame();
		break;
	case COLOR: case FINAL:
		drawMesh();
		break;
	default:
		break;
	}

	angleChanged = false;

	painting = false;
}

void glWidget::drawGrid() {
	// cube
	lineProgram.bind();

	gridBuffer.bind();
	indBuffer.bind();
	mMatrix = QMatrix4x4();
	mMatrix.scale(_rec_set.volume_size, (_rec_set.doubleY ? _rec_set.volume_size * 2 : _rec_set.volume_size), _rec_set.volume_size);

	lineProgram.setUniformValue("mvpMatrix", pMatrix * vMatrix * mMatrix);
	lineProgram.setUniformValue("color", QColor(255, 255, 255));
	lineProgram.setAttributeBuffer("vertex", GL_FLOAT, 0, 3);
	lineProgram.enableAttributeArray("vertex");
	glDrawElements(GL_LINES, inds.size(), GL_UNSIGNED_INT, 0);
	lineProgram.disableAttributeArray("vertex");

	indBuffer.release();
	gridBuffer.release();
	lineProgram.release();
}

void glWidget::drawCamera(QMatrix4x4 pose, QColor color) {
	lineProgram.bind();

	camBuffer.bind();
	camindBuffer.bind();

	QMatrix4x4 mcorrMatrix = QMatrix4x4();
	mcorrMatrix(0, 0) = -1;
	mcorrMatrix(1, 1) = -1;
	QMatrix4x4 mOffsetMatrix = QMatrix4x4(); mOffsetMatrix.translate(offset);	
	mMatrix = mcorrMatrix * mOffsetMatrix*pose;

	lineProgram.setUniformValue("mvpMatrix", pMatrix * vMatrix * mMatrix);
	lineProgram.setUniformValue("color", color);
	lineProgram.setAttributeBuffer("vertex", GL_FLOAT, 0, 3);
	lineProgram.enableAttributeArray("vertex");
	glDrawElements(GL_LINES, cam_inds.size(), GL_UNSIGNED_INT, 0);
	lineProgram.disableAttributeArray("vertex");

	camindBuffer.release();
	camBuffer.release();
	lineProgram.release();

}

void glWidget::drawCloud() {
	if (cloud_points.size() > 0) {
		cloud_mutex.lock();

		QMatrix4x4 mcorrMatrix = QMatrix4x4();
		mcorrMatrix(0, 0) = -1;
		mcorrMatrix(1, 1) = -1;
		QMatrix4x4 mOffsetMatrix = QMatrix4x4();
		offset = QVector3D(-_rec_set.volume_size / 2, -_rec_set.volume_size / 2 * (_rec_set.doubleY ? 2 : 1), -_rec_set.volume_size / 2);
		mOffsetMatrix.translate(offset);

		QMatrix4x4 mPoseMatrix = computeCamPose(_rec_set);

		mMatrix = mcorrMatrix * mOffsetMatrix*mPoseMatrix;
		drawCamera(mPoseMatrix, QColor(255, 255, 0));

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

void glWidget::drawFrame() {

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

void glWidget::drawMesh() {
	if (initialized) {
		cloud_mutex.lock();

		if (!draw_color) {

			normalProgram.bind();

			meshBuffer.bind();
			meshindBuffer.bind();

			offset = QVector3D(-_rec_set.volume_size / 2, -_rec_set.volume_size / 2 * (_rec_set.doubleY ? 2 : 1), -_rec_set.volume_size / 2);

			QMatrix4x4 mcorrMatrix = QMatrix4x4();
			mcorrMatrix(0, 0) = -1;
			mcorrMatrix(1, 1) = -1;
			QMatrix4x4 mOffsetMatrix = QMatrix4x4(); mOffsetMatrix.translate(offset);
			mMatrix = mcorrMatrix * mOffsetMatrix;

			normalProgram.setUniformValue("mvpMatrix", pMatrix * vMatrix * mMatrix);
			normalProgram.setAttributeBuffer("vertex", GL_FLOAT, 0, 3, sizeof(color_vertex));
			normalProgram.enableAttributeArray("vertex");
			normalProgram.setAttributeBuffer("normal", GL_FLOAT, sizeof(float) * 3, 3, sizeof(color_vertex));
			normalProgram.enableAttributeArray("normal");
			glDrawElements(GL_TRIANGLES, mesh_inds.size(), GL_UNSIGNED_INT, 0);
			normalProgram.disableAttributeArray("vertex");
			normalProgram.disableAttributeArray("normal");

			meshindBuffer.release();
			meshBuffer.release();

			normalProgram.release();
		}
		else {
			colorProgram.bind();

			meshBuffer.bind();
			meshindBuffer.bind();

			QMatrix4x4 mcorrMatrix = QMatrix4x4();
			mcorrMatrix(0, 0) = -1;
			mcorrMatrix(1, 1) = -1;
			QMatrix4x4 mOffsetMatrix = QMatrix4x4(); mOffsetMatrix.translate(offset);
			mMatrix = mcorrMatrix * mOffsetMatrix;

			colorProgram.setUniformValue("mvpMatrix", pMatrix * vMatrix * mMatrix);
			colorProgram.setAttributeBuffer("vertex", GL_FLOAT, 0, 3, sizeof(color_vertex));
			colorProgram.enableAttributeArray("vertex");
			colorProgram.setAttributeBuffer("color", GL_FLOAT, sizeof(float) * 3, 3, sizeof(color_vertex));
			colorProgram.enableAttributeArray("color");
			glDrawElements(GL_TRIANGLES, mesh_inds.size(), GL_UNSIGNED_INT, 0);
			colorProgram.disableAttributeArray("vertex");

			meshindBuffer.release();
			meshBuffer.release();

			colorProgram.release();

		}

		for (int i = 0; i < camposes.size(); ++i)
			drawCamera(camposes[i], QColor(255, 255, 255));

		cloud_mutex.unlock();
	}
}

// FRAMEBUFFER RENDERING

void glWidget::render(QMatrix4x4 pose, iparams ip) {
	if (!initialized) {
		emit renderFinished(false, std::vector<float>());
		return;
	}

	if (depthFrameBuffer->width() != ip.width || depthFrameBuffer->height() != ip.height) {
		printf("gl: Remake framebuffer %dx%d\n", ip.width, ip.height);
		delete depthFrameBuffer;
		depthFrameBuffer = new QGLFramebufferObject(QSize(ip.width, ip.height), QGLFramebufferObject::Depth, GL_TEXTURE_2D, GL_R32F);
	}


	depthFrameBuffer->bind();
	glViewport(0, 0, ip.width, ip.height);
	qglClearColor(QColor(0, 0, 0));
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//glDisable(GL_BLEND);
	glDepthMask(GL_TRUE);

	glPointSize(1.0f);
	QMatrix4x4 vdMatrix;
	vdMatrix.lookAt(QVector3D(-0, -0, -0.0f), QVector3D(-0, -0, 1), QVector3D(0, 1, 0));

	QMatrix4x4 pdMatrix;
	float fov_r = 2 * atan(ip.height / (float)(2.0f * ip.fy));
	float fov_d = fov_r * 180.0f / 3.1415926535f;
	pdMatrix.perspective(fov_d, (float)ip.width / (float)ip.height, 0.1f, 5.f);

	QMatrix4x4 CoordM;
	CoordM(0, 0) = -1;
	CoordM(1, 1) = -1;

	//glDisable(GL_CULL_FACE);
	glEnable(GL_CULL_FACE);

	depthProgram.bind();

	meshBuffer.bind();
	meshindBuffer.bind();

	mMatrix = QMatrix4x4();
	mMatrix = pose.inverted();

	depthProgram.setUniformValue("pMatrix", pdMatrix);
	depthProgram.setUniformValue("vmMatrix", vdMatrix * CoordM * mMatrix);
	depthProgram.setAttributeBuffer("vertex", GL_FLOAT, 0, 3, sizeof(color_vertex));
	depthProgram.enableAttributeArray("vertex");
	glDrawElements(GL_TRIANGLES, mesh_inds.size(), GL_UNSIGNED_INT, 0);
	depthProgram.disableAttributeArray("vertex");

	meshindBuffer.release();
	meshBuffer.release();

	depthProgram.release();

	//glEnable(GL_CULL_FACE);


	float* pixels = new float[ip.width*ip.height];
	memset(pixels, 0, ip.width*ip.height * sizeof(float));

	glBindTexture(GL_TEXTURE_2D, depthFrameBuffer->texture());
	glGetTexImage(GL_TEXTURE_2D, 0, GL_RED, GL_FLOAT, pixels);
	printGlError(glGetError());

	glPointSize(2.0f);
	depthFrameBuffer->release();

	glViewport(0, 0, this->width(), this->height());
	glDisable(GL_CULL_FACE);

	std::vector<float> dpt; dpt.resize(ip.width*ip.height);
	for (int y = 0; y < ip.height; ++y)
		for (int x = 0; x < ip.width; ++x)
			dpt[y*ip.width + x] = -pixels[((ip.height - 1 - y)*ip.width + x)];
	delete[] pixels;

	emit renderFinished(true, dpt);
}

// MOUSE

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

// SLOTS

void glWidget::stateChanged(ProgramState st) {
	qDebug("glWidget::stateChanged");

	this->state = st;
	this->updateGL();
}

void glWidget::refreshRecSettings(rec_settings rc) {
	this->_rec_set = rc;

	this->updateGL();
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

void glWidget::refreshCloud(QVector<QVector3D> pnts, QVector<QVector3D> clrs) {
	cloud_mutex.lock();
	this->cloud_points = pnts;
	this->cloud_colors = clrs;

	cloud_mutex.unlock();
	this->updateGL();
}

void cross(cn_vertex v0, cn_vertex v1, cn_vertex v2, float& nx, float& ny, float& nz) {
	float ax = (v0.vertex[0] - v1.vertex[0]);
	float bx = (v2.vertex[0] - v1.vertex[0]);
	float ay = (v0.vertex[1] - v1.vertex[1]);
	float by = (v2.vertex[1] - v1.vertex[1]);
	float az = (v0.vertex[2] - v1.vertex[2]);
	float bz = (v2.vertex[2] - v1.vertex[2]);
	nx = ay*bz - az*by;
	ny = az*bx - ax*bz;
	nz = ax*by - ay*bx;
}

void glWidget::setPolygonMesh(Model::Ptr model) {
	qDebug("glWidget::setPolygonMesh");

	cloud_mutex.lock();
	initialized = false;

// init arrays
	mesh_points.clear();
	mesh_inds.clear();

	std::vector<int> face_per_point;

	bool has_color = model->hasColor();

	if (model->isVBO()) {
		int points_size;
		points_size = model->getVertsSize();
		
		mesh_points.reserve(points_size);

		color_vertex v0, v1, v2;
		int i = 0;
		for (auto it = model->verts_begin(); it != model->verts_end(); ++it, ++i) {
			Model::PointXYZ p = *it;

			color_vertex vert;
			vert.vertex[0] = p.x;
			vert.vertex[1] = p.y;
			vert.vertex[2] = p.z;

			vert.color[0] = vert.color[1] = vert.color[2] = 0.0f;
			//vert.normal[0] = vert.normal[1] = vert.normal[2] = 0.0f;

			mesh_points.push_back(vert);
			mesh_inds.push_back(i);

			if (i % 3 == 0)
				v0 = vert;
			if (i % 3 == 1)
				v1 = vert;
			if (i % 3 == 2) {
				v2 = vert;
				float x, y, z;
				cross(v0, v1, v2, x, y, z);
				mesh_points[i].color[0] = x;
				mesh_points[i].color[1] = y;
				mesh_points[i].color[2] = z;

				mesh_points[i - 1].color[0] = x;
				mesh_points[i - 1].color[1] = y;
				mesh_points[i - 1].color[2] = z;

				mesh_points[i - 2].color[0] = x;
				mesh_points[i - 2].color[1] = y;
				mesh_points[i - 2].color[2] = z;
			}
		}
	} else if (model->isIBO()) {
		int points_size;
		points_size = model->getVertsSize();
		int indices_size;
		indices_size = model->getIndsSize();

		mesh_points.reserve(points_size);
		mesh_inds.reserve(indices_size);

		std::vector<int> face_per_point;
		face_per_point.resize(points_size);

		cn_vertex v0, v1, v2;

		int i = 0;
		for (auto it = model->verts_begin(); it != model->verts_end(); ++it, ++i) {
			Model::PointXYZ p = *it;

			cn_vertex vert;
			vert.vertex[0] = p.x;
			vert.vertex[1] = p.y;
			vert.vertex[2] = p.z;

			if (!has_color) {
				vert.color[0] = 0;
				vert.color[1] = 0;
				vert.color[2] = 0;
			}
			else {
				vert.color[0] = it.getColor().r;
				vert.color[1] = it.getColor().g;
				vert.color[2] = it.getColor().b;
			}
			//vert.normal[0] = vert.normal[1] = vert.normal[2] = 0.0f;

			mesh_points.push_back(vert);
			face_per_point[i] = 0;
		}
		for (auto it = model->tri_begin(); it!=model->tri_end(); ++it) {
			Model::Triangle t = *it;

			int i0 = t.p[0], i1 = t.p[1], i2 = t.p[2];
			mesh_inds.push_back(i0);
			mesh_inds.push_back(i1);
			mesh_inds.push_back(i2);
			if (!has_color) {
				float nx, ny, nz;
				cross(mesh_points[i0], mesh_points[i1], mesh_points[i2], nx, ny, nz);
				mesh_points[i0].color[0] -= nx;
				mesh_points[i0].color[1] -= ny;
				mesh_points[i0].color[2] -= nz;
				face_per_point[i0] += 1;

				mesh_points[i1].color[0] -= nx;
				mesh_points[i1].color[1] -= ny;
				mesh_points[i1].color[2] -= nz;
				face_per_point[i1] += 1;

				mesh_points[i2].color[0] -= nx;
				mesh_points[i2].color[1] -= ny;
				mesh_points[i2].color[2] -= nz;
				face_per_point[i2] += 1;
			}
		}
		if (!has_color) {
			for (i = 0; i < mesh_points.size(); ++i) {
				if (face_per_point[i] > 0) {
					mesh_points[i].color[0] /= face_per_point[i];
					mesh_points[i].color[1] /= face_per_point[i];
					mesh_points[i].color[2] /= face_per_point[i];
				}
			}
		}
	}
// ----------
	meshBuffer = QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
	meshBuffer.create();
    meshBuffer.setUsagePattern( QOpenGLBuffer::StreamDraw );
    meshBuffer.bind();
	meshBuffer.allocate(mesh_points.constData(), mesh_points.size()*sizeof(color_vertex));
	meshBuffer.release();

	meshindBuffer = QOpenGLBuffer(QOpenGLBuffer::IndexBuffer);
	meshindBuffer.create();
	meshindBuffer.setUsagePattern(QOpenGLBuffer::StreamDraw);
	meshindBuffer.bind();
	meshindBuffer.allocate(mesh_inds.constData(), mesh_inds.size()*sizeof(unsigned int));
	meshindBuffer.release();

	initialized = true;
	draw_color = has_color;
	
	camposes.clear();
	cloud_mutex.unlock();
	for (int i = 0; i < model->getFramesSize(); ++i)
		newCameraPose(toQtPose(model->getFrame(i)->pose.rotation(), model->getFrame(i)->pose.translation()));
	
	this->updateGL();
}

void glWidget::newCameraPose(QMatrix4x4 pose) {
	cloud_mutex.lock();

	camposes.push_back(pose);

	cloud_mutex.unlock();
}