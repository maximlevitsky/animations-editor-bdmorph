
#include <stdio.h>
#include <assert.h>
#include <QtOpenGL>
#include "ThumbnailRenderer.h"
#include "MeshModel.h"

ThumbnailRenderer::ThumbnailRenderer(QWidget* parent,
		QGLWidget* shareWidget) :
		QGLWidget(parent, shareWidget), textureRef(0)
{
	makeCurrent();


	fbo = new QGLFramebufferObject(128,128);
	bool result = fbo->bind();

	if (result == false)
		printf("Failed to bind thumbnail FBO");

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    glHint(GL_POLYGON_SMOOTH_HINT, GL_DONT_CARE);
    glEnable(GL_POLYGON_SMOOTH);
}

QImage ThumbnailRenderer::renderThumbnail(MeshModel* model)
{
	makeCurrent();
	assert(model);

	int width = 128;
	int height = 98;

    glViewport(0,30,128, 98);
    glClearColor(1.,1.,1., 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    BBOX b = model->getActualBBox();

    double ratio = std::max(b.width() / width, b.height() / height) * 1.1;
    double neededWidth = ratio * width;
    double neededHeight = ratio * height;

    /* Setup projection */
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    glOrtho(b.center().x - neededWidth/2  , b.center().x + neededWidth/2, b.center().y - neededHeight/2 ,
    		b.center().y + neededHeight/2, 0, 1);

    /* Setup texture */
    glEnable(GL_TEXTURE_2D);

    model->renderFaces();
	return fbo->toImage();
}

void ThumbnailRenderer::onTextureChanged(GLuint texture)
{
	textureRef = texture;
	makeCurrent();
	glBindTexture(GL_TEXTURE_2D, textureRef);
}
