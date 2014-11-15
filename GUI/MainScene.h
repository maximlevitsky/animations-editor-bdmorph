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

class MainScene : public QGLWidget
{
    Q_OBJECT
public:
    MainScene(QWidget* parent);
    ~MainScene() { if (model) delete model; }

public slots:
	void loadModel();
	void saveModel();
	void undoModel();
	void redoModel();
    void resetPoints();

    void chooseTexture();
	void resetTexture();

	void zoomIn()  {zoom(ZOOM_FACTOR);}
    void zoomOut() {zoom(1./ZOOM_FACTOR);}

    void moveLeft() {move(QPointF(-10,0));}
    void moveRight() {move(QPointF(10,0));}
    void moveUp() {move(QPointF(0,10));}
    void moveDown() {move(QPointF(0,-10 ));}

	void changeAlpha(int i);
	void drawModeChanged(bool m);
	void changeWireframe(int i);
	void clearPins();

    void saveLog();
    void runLog();

private:
    /* transformations*/
	int closestIndex(QPointF pos);
    void zoom(double factor) {modelWidth *= factor;}
    void move(QPointF direction) {modelLocation += direction;}
	Vector2D<double> screenToModelVec(QPointF v);

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

private:
    /* Loaded model
     * TODO: Here we will store all keyframes, BDFFrames, etc later*/
    MeshModel *model;
    KVFModel *keyframeModel;

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
