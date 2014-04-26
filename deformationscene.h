#ifndef DEFORMATIONSCENE_H
#define DEFORMATIONSCENE_H

#include <QGraphicsScene>
#include <QLabel>
#include <QTime>
#include <QKeyEvent>
#include <QSpinBox>
#include <QSlider>
//#include <QComboBox>
#include <QImage>
#include <qgl.h>
#include <QGLWidget>
#include <set>
#include <QCheckBox>

#include "logspiral.h"
using namespace std;

#define ZOOM_FACTOR 1.2

#include "model2d.h"

class DeformationScene : public QGraphicsScene
{
    Q_OBJECT

public:
    void setPointsToRender(vector<int> &p) {
        pointsToRender = p;
        if (p.size() > 0) oldVertices = p;
    }

    DeformationScene(QGLWidget *w);
    ~DeformationScene() { if (model) delete model; }
    void drawBackground(QPainter *painter, const QRectF &rect);
    void zoom(double factor) {modelWidth *= factor;}
    void move(QPointF direction) {modelLocation += direction;}
    int closestTangent(QPointF pos);
    void updateLogSpiral(int which);
    int closestIndex(QPointF pos) {
        if (!model) return -1;

        pos.setY(height()-pos.y()-1);
        pos -= modelLocation;
        pos *= model->getWidth() / modelWidth;
        pos += QPointF(model->getMinX(),model->getMinY());

        // large radius because my fingers are "fleshy" according to my piano prof
        return model->getClosestVertex(Point2D<double>(pos.x(),pos.y()),5*model->getWidth()/modelWidth);
    }

    Vector2D<double> screenToModelVec(QPointF v) {
        v *= model->getWidth() / modelWidth;
        return Vector2D<double>(v.x(), -v.y());
    }

    void displaceMesh(vector<int> indices, vector<Vector2D<double> > displacements) {
        for (set<int>::iterator it = pinned.begin(); it != pinned.end(); ++it) {
            indices.push_back(*it);
            displacements.push_back(Vector2D<double>(0,0));
        }

        if (multitouchMode->isChecked()) { 
			model->displaceMesh(indices,displacements,alpha);
			logDisplacements.push_back(displacements);
			logIndices.push_back(indices);
			logAlphas.push_back(alpha);

			model->addUndoAction(indices,displacements,alpha);
		}
    }

public slots:
	void undoModel();
	void redoModel();
	void loadImage();
    void loadModel();
    void saveModel();
    void chooseTexture();
	void removeTexture();
    void zoomIn() {zoom(ZOOM_FACTOR);}
    void zoomOut() {zoom(1./ZOOM_FACTOR);}
    void moveLeft() {move(QPointF(-10,0));}
    void moveRight() {move(QPointF(10,0));}
    void moveUp() {move(QPointF(0,10));}
    void moveDown() {move(QPointF(0,-10 ));}
    void changeAlpha(int i) { alpha = (double)i / 100 * 2; }
	void changeBrush(int i) { brush = (double)i; }
    void modeChanged(bool m);
    void drawModeChanged(bool m) { if (model) model->changeDrawMode(m); }
	
	void changeWireframe(int i) {
		wireframe = i;
		if (model) model->setWireframeTrans((double)i/100); 
	}

	bool isPinModeChecked() { return pinMode->isChecked(); }
	bool isMultitouchModeChecked() { return multitouchMode->isChecked(); }
    void clearPins() { pinned.clear(); }

    void resetPoints();
    void restorePoints() {
        model->copyPositions(*origModel);
    }

    void saveLog();
    void runLog();

protected:
	void wheelEvent(QGraphicsSceneWheelEvent *event);
    void mousePressEvent(QGraphicsSceneMouseEvent *event);
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);
    void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
    void keyPressEvent(QKeyEvent *e);
    void keyReleaseEvent(QKeyEvent *e);

private:
    QGLWidget *glWidget;
    QDialog *createDialog(const QString &windowTitle) const;
    QWidget *undoButton, *redoButton, *imageButton, *modelButton, *chooseTextureButton, *removeTextureButton, *clearButton, *loadGeometry, *saveButton, *resetButton;
    QSlider *alphaSlider, *brushSlider, *wireframeSlider;
	QLabel *meshLabel;
    QCheckBox *animationMode, *multitouchMode, *drawVectorField, *pinMode;

    Model2D<double> *model, *origModel;
    QPointF modelLocation;
    float modelWidth;
    int selectedIndex;

    set<int> pinned;
    bool shiftDown, controlDown;
    double alpha;
	double brush;
	int wireframe;

    unsigned int texHandle;

    QPointF mousePos;

    vector<int> pointsToRender;

    GLuint textureRef;

    vector< vector<int> > constraintVerts;
    vector< vector< Vector2D<double> > > constraintTangents;
    vector< vector< Point2D<double> > > constraintTargets;
    vector< LogSpiral<double> > curSpirals;
    int curFrame;
    vector<int> oldVertices;

    vector< vector< Vector2D<double> > > logDisplacements;
    vector< vector<int> > logIndices;
    vector< double > logAlphas;

};

#endif // DEFORMATIONSCENE_H
