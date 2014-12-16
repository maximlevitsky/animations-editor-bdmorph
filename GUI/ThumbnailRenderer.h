#ifndef THUMBNAILRENDERER_H_
#define THUMBNAILRENDERER_H_

#include <QGLWidget>
#include <QWidget>
#include <QGLFramebufferObject>
#include <QImage>

class MeshModel;
class ThumbnailRenderer : public QGLWidget
{
	Q_OBJECT
public:
	ThumbnailRenderer(QWidget* parent, QGLWidget* shareWidget);
	virtual ~ThumbnailRenderer() {}
	QImage renderThumbnail(MeshModel* model);
	QGLFramebufferObject *fbo;
};

#endif /* THUMBNAILRENDERER_H_ */
