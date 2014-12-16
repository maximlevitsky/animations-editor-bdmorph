#ifndef DEFORMATIONSCENE_H
#define DEFORMATIONSCENE_H

#include <vector>
#include <map>
#include <QGLWidget>
#include <QPointF>
#include <QTouchEvent>
#include "ProgramState.h"
#include "vector2d.h"
#include "utils.h"

#define ZOOM_FACTOR 1.2

class MeshModel;
class KVFModel;
class VideoModel;
class OutlineModel;
class ProgramState;

class EditorWindow : public QGLWidget
{
    Q_OBJECT
public:
    EditorWindow(QWidget* parent);
    virtual ~EditorWindow() {}
        
public slots:
	void programStateUpdated(int flags, void *param);
	void programStateCreated(ProgramState* state) { programstate = state; }

private:
    /* transformations*/
	void zoom(double factor);
	void move(QPointF direction);
    void moveLeft() {move(QPointF(-10,0));}
    void moveRight() {move(QPointF(10,0));}
    void moveUp() {move(QPointF(0,10));}
    void moveDown() {move(QPointF(0,-10 ));}
	void zoomIn()  {zoom(ZOOM_FACTOR);}
    void zoomOut() {zoom(1./ZOOM_FACTOR);}

    /* Events */
    bool event(QEvent *);
	void mouseMoveEvent (QMouseEvent * event);
	void mousePressEvent ( QMouseEvent * event );
	void mouseReleaseEvent(QMouseEvent * event);
	void wheelEvent (QWheelEvent * event );
	void keyPressEvent ( QKeyEvent * event );
    bool touchEvent(QTouchEvent* te);
    void paintGL();
    void resizeGL(int w, int h);
    void initializeGL();
private:

    ProgramState* programstate;

    /* Model state */
    QPointF modelLocation;
    double modelWidth;
    std::vector<Vertex> selectedVertices;

    /* Input state */
    QPointF lastMousePos;
    std::map<int, QPointF> touchPointLocations;
    std::map<int, int> touchToVertex;


    bool mouseMoved;
    bool mouseLeft;

    double getRadius(MeshModel *model);
	Point2 screenToModel(MeshModel *model, QPointF v);

};
#endif // DEFORMATIONSCENE_H
