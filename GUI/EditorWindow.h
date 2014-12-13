#ifndef DEFORMATIONSCENE_H
#define DEFORMATIONSCENE_H

#include <vector>
#include <map>
#include <QGLWidget>
#include <QPointF>
#include <QTouchEvent>
#include "vector2d.h"
#include "Utils.h"

#define ZOOM_FACTOR 1.2

class MeshModel;
class KVFModel;
class VideoModel;
class OutlineModel;

class EditorWindow : public QGLWidget
{
    Q_OBJECT
public:
    EditorWindow(QWidget* parent);
    virtual ~EditorWindow() {}
        
public slots:
	/* these slots are connected to side panel to manipulate the view*/
	void onUndoModel();
	void onRedoModel();
	void onReuseVF();
    void onResetPoints();

	void onDrawVFModeChanged(bool m);
	void onDrawOrigVFModeChanged(bool m);
	void onPinModeChanged(bool m);
	void onShowSelectionChanged(bool m);

	void onChangeAlpha(int i);
	void onChangeWireframe(int i);

	void onClearPins();
    void onSaveLog();
    void onRunLog();

    void onResetTransform();
    
    /* these slots are connected to bottom animation panel + main window */
    void onFrameSwitched(MeshModel* model);
    void onVideoModelLoaded(VideoModel* model);

    void onTextureChanged(GLuint textureRef);

	void onAnimationStarted();
	void onAnimationStopped();

signals:
	void modelEdited(MeshModel* model);
	void selectionChanged(int selectedVertex, int selectedFace);
	void FPSUpdated(double msec);

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

    KVFModel *kvfModel;
    OutlineModel* outlineModel;
    MeshModel* renderModel;

    /* Model state */
    QPointF modelLocation;
    double modelWidth;
    std::vector<Vertex> selectedVertices;
    bool pinMode;
    bool multitouchMode;

    /* GL state */
    unsigned int texHandle;
    int wireframeTransparency;
    GLuint textureRef;
    Vertex hoveredVertex;
    int hoveredFace;

    /* Input state */
    QPointF lastMousePos;
    std::map<int, QPointF> touchPointLocations;
    std::map<int, int> touchToVertex;

    bool drawVF;
    bool drawVFOrig;
    bool disableEdit;
    bool showSelection;

    bool mouseMoved;
    bool mouseLeft;

    double getRadius();

};
#endif // DEFORMATIONSCENE_H
