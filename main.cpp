#include "simplesparsematrix.h"
#include "logspiral.h"

#include <amd.h>

#include <QtGui>
#include <QGLWidget>
#include <fstream>
#include <vector>
#include <set>
#include <map>
using namespace std;

#include "deformationscene.h"

class GraphicsView : public QGraphicsView {
public:
    GraphicsView(DeformationScene *e) : deformationScene(e) {
        setAttribute(Qt::WA_AcceptTouchEvents);
        setWindowTitle(tr("2D Near-Isometric Deformations"));
    }

protected:
    void resizeEvent(QResizeEvent *event) {
        if (scene())
            scene()->setSceneRect(QRect(QPoint(0, 0), event->size()));
        QGraphicsView::resizeEvent(event);
    }

    bool viewportEvent(QEvent *event) {
        switch (event->type()) {
             case QEvent::TouchBegin:
             case QEvent::TouchUpdate:
             case QEvent::TouchEnd:

            touchEvent(static_cast<QTouchEvent*>(event));
            break;

        default : break;
            }
        return QGraphicsView::viewportEvent(event);
    }

    void touchEvent(QTouchEvent *te) {
        QList<QTouchEvent::TouchPoint> points = te->touchPoints();


		if (deformationScene->isPinModeChecked()) return; //if pin mode is checked, directed to mousepress event. Yeah, Qt is weird like that
		if (!deformationScene->isMultitouchModeChecked() && points.count()==2) { //zoom only in normal mode
			QTouchEvent::TouchPoint p0 = points.first();
			QTouchEvent::TouchPoint p1 = points.last();
			QLineF line1(p0.startPos(),p1.startPos());
			QLineF line2(p0.pos(),p1.pos());
			qreal scaleFactor = line2.length()/line1.length();
			deformationScene->zoom(scaleFactor+(1-scaleFactor)/1.05);
			return;
		}


		//if (te->type() == QEvent::TouchBegin) qWarning("Begin!");
        if (te->type() == QEvent::TouchEnd) {
            idToLocation.clear();
            vector<int> v;
            deformationScene->setPointsToRender(v);
            return;
        }

	
        // fix set of pins
        set<int> ids;
        for (int i = 0; i < points.size(); i++) {
            QTouchEvent::TouchPoint p = points[i];
            if (idToLocation.count(p.id()) == 0) { //what the hell is idToLocation? :(
                idToLocation[p.id()] = p.pos();
                touchToVertex[p.id()] = deformationScene->closestIndex(p.pos());
            }
            ids.insert(p.id());
        }

        map<int, QPointF>::iterator it = idToLocation.begin();
        set<int> toRemove;
        while (it != idToLocation.end()){
            if (ids.count(it->first) == 0)
                toRemove.insert(it->first);
            ++it;
        }

        for (set<int>::iterator iter = toRemove.begin(); iter != toRemove.end(); ++iter)
            idToLocation.erase(*iter);

        // figure out if anything moved significantly
        double maxDistance = 0;
        for (int i = 0; i < points.size(); i++) {
            QPointF loc = points[i].pos();
            QPointF old = idToLocation[points[i].id()];
            QPointF diff = old-loc;
            double d = sqrt(diff.x()*diff.x() + diff.y()*diff.y());

            if (d > maxDistance) maxDistance = d;
        }

        if (maxDistance >= 1) { // moved more than one pixel!
            vector<int> ids;
            vector< Vector2D<double> > displacements;

            for (int i = 0; i < points.size(); i++) {
                QPointF displacement = points[i].pos() - idToLocation[points[i].id()];
                if (touchToVertex[points[i].id()] == -1) continue;
                ids.push_back(touchToVertex[points[i].id()]);
                displacements.push_back(deformationScene->screenToModelVec(displacement));
                idToLocation[points[i].id()] = points[i].pos();
            }

            if (ids.size() > 0)
                deformationScene->displaceMesh(ids, displacements);

        }

        vector<int> v;
        for (map<int,QPointF>::iterator it = idToLocation.begin(); it != idToLocation.end(); ++it)
            v.push_back(touchToVertex[it->first]);
        deformationScene->setPointsToRender(v);
    }

    map<int, QPointF> idToLocation;
    map<int, int> touchToVertex;
    DeformationScene *deformationScene;
};

void myMessageOutput(QtMsgType type, const char *msg) {
    switch (type) {
    case QtDebugMsg:
//            fprintf(stderr, "Debug: %s\n", msg);
        break;
    case QtWarningMsg:
//            fprintf(stderr, "Warning: %s\n", msg);
        break;
    case QtCriticalMsg:
//            fprintf(stderr, "Critical: %s\n", msg);
        break;
    case QtFatalMsg:
//            fprintf(stderr, "Fatal: %s\n", msg);
        abort();
    }
}

int main(int argc, char **argv) {
    //qInstallMsgHandler(myMessageOutput);

    QApplication app(argc, argv);

    QGLWidget *qgl = new QGLWidget(QGLFormat(QGL::SampleBuffers));
    qgl->setAttribute(Qt::WA_AcceptTouchEvents);

    DeformationScene *scene = new DeformationScene(qgl);

    GraphicsView view(scene);
    view.setViewport(qgl);
    view.setViewportUpdateMode(QGraphicsView::FullViewportUpdate);
    view.setScene(scene);
    view.show();

    view.resize(800, 600);

    return app.exec();
}
