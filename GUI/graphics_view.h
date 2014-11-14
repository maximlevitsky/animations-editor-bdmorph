#ifndef GRAPHICS_VIEW_H
#define GRAPHICS_VIEW_H

#include <QtGui>
#include <QGLWidget>
#include <vector>
#include <set>
#include "deformationscene.h"

class GraphicsView : public QGraphicsView
{
public:
    GraphicsView(DeformationScene *e) : deformationScene(e) {
        setAttribute(Qt::WA_AcceptTouchEvents);
        setWindowTitle(tr("2D Near-Isometric Deformations"));
    }
protected:
	void resizeEvent(QResizeEvent* event);
	bool viewportEvent(QEvent* event);
	void touchEvent(QTouchEvent* te);
private:
    std::map<int, QPointF> idToLocation;
    std::map<int, int> touchToVertex;
    DeformationScene *deformationScene;
};

#endif
