#ifndef DEFORMATIONSCENE_H
#define DEFORMATIONSCENE_H

#include <QLabel>
#include <QTime>
#include <QKeyEvent>
#include <QSpinBox>
#include <QSlider>
#include <QImage>
#include <qgl.h>
#include <QGLWidget>
#include <set>
#include <QCheckBox>
#include <qwidget.h>

#define ZOOM_FACTOR 1.2

#include "Model.h"
#include "KVFModel.h"
#include "VideoModel.h"

class MainScene : public QGLWidget
{
    Q_OBJECT
public:
    MainScene(QWidget* parent);
    ~MainScene() {}
        
public slots:
	/* these slots are connected to side panel to manipulate the view*/
	void undoModel();
	void redoModel();
	void reuseVF();
    void resetPoints();

	void drawVFModeChanged(bool m);
	void drawOrigVFModeChanged(bool m);
	void pinModeChanged(bool m);

	void changeAlpha(int i);
	void changeWireframe(int i);

	void clearPins();
    void saveLog();
    void runLog();

    void resetTransform();
    
    /* these slots are connected to bottom animation panel + main window */
    void onFrameSwitched(MeshModel* model);
    void onVideoModelLoaded(VideoModel* model);

    void setTexture(QPixmap& texture);
    void resetTexture();

signals:
	void modelEdited(KVFModel* model);

private:
    /* transformations*/
	int closestIndex(QPointF pos);
    void zoom(double factor) {modelWidth *= factor;}
    void move(QPointF direction) {modelLocation += direction;}
    void moveLeft() {move(QPointF(-10,0));}
    void moveRight() {move(QPointF(10,0));}
    void moveUp() {move(QPointF(0,10));}
    void moveDown() {move(QPointF(0,-10 ));}
	void zoomIn()  {zoom(ZOOM_FACTOR);}
    void zoomOut() {zoom(1./ZOOM_FACTOR);}
	Point2 screenToModel(QPointF v);

    /* Events */
    bool event(QEvent *);
	void mouseMoveEvent (QMouseEvent * event);
	void mousePressEvent ( QMouseEvent * event );
	void mouseReleaseEvent(QMouseEvent * event);
	void wheelEvent (QWheelEvent * event );
	void keyPressEvent ( QKeyEvent * event );
	//void keyReleaseEvent ( QKeyEvent * event );
    bool touchEvent(QTouchEvent* te);
    void paintGL();
    void resizeGL(int w, int h);
    void initializeGL();

private:

    KVFModel *editModel;
    MeshModel* renderModel;

    /* Model state */
    QPointF modelLocation;
    float modelWidth;
    std::vector<int> selectedVertices;
    bool pinMode;
    bool multitouchMode;

    /* GL state */
    unsigned int texHandle;
    int wireframeTransparency;
    GLuint textureRef;

    /* Input state */
    QPointF lastMousePos;
    std::map<int, QPointF> touchPointLocations;
    std::map<int, int> touchToVertex;

};

#endif // DEFORMATIONSCENE_H
