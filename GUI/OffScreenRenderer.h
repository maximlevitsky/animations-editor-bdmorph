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
	OffScreenRenderer(QWidget* parent, QGLWidget* shareWidget, int width,int height);
	virtual ~OffScreenRenderer();

	/* Slow simple render */
	void renderToQImage(MeshModel* model, QImage& out, int stripSize, double scale);

	/* Fast render for video */
	void setupTransform(MeshModel* model,bool vertFlip,int stripSize, double scale);
	void renderToBufferBGRA(MeshModel* model, void* out);

	void setTexture(QPixmap &texture);

private:
	int width;
	int height;
	QGLFramebufferObject *fbo;
	GLuint textureRef;
};

#endif /* THUMBNAILRENDERER_H_ */
