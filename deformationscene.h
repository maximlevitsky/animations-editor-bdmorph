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

#define ZOOM_FACTOR 1.2

#include "model2d.h"

class DeformationScene : public QGraphicsScene
{
    Q_OBJECT

public:
    void setPointsToRender(std::vector<int> &p) {
        pointsToRender = p;
        if (p.size() > 0) oldVertices = p;
    }

    DeformationScene(QGLWidget *w);
    ~DeformationScene() { if (model) delete model; }
    void drawBackground(QPainter *painter, const QRectF &rect);
    void zoom(double factor) {modelWidth *= factor;}
    void move(QPointF direction) {modelLocation += direction;}
    void updateLogSpiral(int which);
	int closestIndex(QPointF pos);

	Vector2D<double> screenToModelVec(QPointF v);

	void displaceMesh(std::vector<int> indices,
			std::vector<Vector2D<double> > displacements);

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
	void restorePoints();

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
    QCheckBox *multitouchMode, *drawVectorField, *pinMode;

    Model2D *model, *origModel;
    QPointF modelLocation;
    float modelWidth;
    int selectedIndex;

    std::set<int> pinned;
    double alpha;
	double brush;
	int wireframe;

    unsigned int texHandle;

    QPointF mousePos;

    std::vector<int> pointsToRender;

    GLuint textureRef;

    std::vector< std::vector<int> > constraintVerts;
    std::vector<int> oldVertices;
    std::vector< std::vector< Vector2D<double> > > logDisplacements;
    std::vector<std::vector<int> > logIndices;
    std::vector< double > logAlphas;

};

#endif // DEFORMATIONSCENE_H
