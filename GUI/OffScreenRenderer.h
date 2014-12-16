#ifndef THUMBNAILRENDERER_H_
#define THUMBNAILRENDERER_H_

#include <QGLWidget>
#include <QWidget>
#include <QGLFramebufferObject>
#include <QImage>

class MeshModel;
class OffScreenRenderer : public QGLWidget
{
	Q_OBJECT
public:
	OffScreenRenderer(QWidget* parent, QGLWidget* shareWidget);
	virtual ~OffScreenRenderer() {}
	QImage renderThumbnail(MeshModel* model);
	QGLFramebufferObject *fbo;
};

#endif /* THUMBNAILRENDERER_H_ */
