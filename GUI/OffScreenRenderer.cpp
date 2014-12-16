
#include <stdio.h>
#include <assert.h>
#include <QtOpenGL>
#include "OffScreenRenderer.h"
#include "MeshModel.h"

OffScreenRenderer::OffScreenRenderer(QWidget* parent, QGLWidget* shareWidget, int width, int height) :
		QGLWidget(parent, shareWidget) ,
		height(height),
		width(width)
{
	makeCurrent();
	fbo = new QGLFramebufferObject(width,height);
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
    /* Setup texture */
    glEnable(GL_TEXTURE_2D);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

/*****************************************************************************************************/

void OffScreenRenderer::renderToImage(MeshModel* model, QImage &out, int stripeSize, double scale)
{
	setupTransform(model,false,stripeSize,scale);
	makeCurrent();
    model->renderFaces();
    out = fbo->toImage();
}

/*****************************************************************************************************/
void OffScreenRenderer::setupTransform(MeshModel* model,bool flip,int stripeSize,double scale)
{
	int height1 = height - stripeSize;
	makeCurrent();

    glViewport(0,stripeSize,width, height - stripeSize);
    glClearColor(1.,1.,1., 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    BBOX b = model->getActualBBox();

    double ratio = std::max(b.width() / width, b.height() / height1) / scale;
    double neededWidth = ratio * width;
    double neededHeight = ratio * height1;

    /* Setup projection */
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    if (flip)
    	neededHeight *= -1;

    glOrtho(b.center().x - neededWidth/2  , b.center().x + neededWidth/2, b.center().y - neededHeight/2 ,
    		b.center().y + neededHeight/2, 0, 1);

}

/*****************************************************************************************************/
void OffScreenRenderer::renderBGRA(MeshModel* model, void* out)
{
	makeCurrent();
    glClearColor(1.,1.,1., 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
	model->renderFaces();
	glReadPixels(0, 0, width, height, GL_BGRA_EXT, GL_UNSIGNED_BYTE, (GLvoid*)out);
}

