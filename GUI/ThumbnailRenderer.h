#ifndef THUMBNAILRENDERER_H_
#define THUMBNAILRENDERER_H_

#include <QtOpenGL>
#include <QGLWidget>
#include <QWidget>
#include <QGLFramebufferObject>
#include <QImage>
#include "Model.h"

class ThumbnailRenderer : public QGLWidget
{
	Q_OBJECT
public:
	ThumbnailRenderer(QWidget* parent, QGLWidget* shareWidget);
	virtual ~ThumbnailRenderer() {}

public slots:
	void onTextureChanged(GLuint texture);
public:
	QImage renderThumbnail(MeshModel* model);

private:
	GLuint textureRef;
	QGLFramebufferObject *fbo;
};



#endif /* THUMBNAILRENDERER_H_ */
