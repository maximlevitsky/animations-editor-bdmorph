#include <QtGui>
#include <QGLWidget>
#include <vector>
#include <set>
#include "graphics_view.h"

/******************************************************************************************************************************/
void GraphicsView::resizeEvent(QResizeEvent* event)
{
	if (scene())
		scene()->setSceneRect(QRect(QPoint(0, 0), event->size()));
	QGraphicsView::resizeEvent(event);
}

/******************************************************************************************************************************/
bool GraphicsView::viewportEvent(QEvent* event)
{
	switch (event->type()) {
	case QEvent::TouchBegin:
	case QEvent::TouchUpdate:
	case QEvent::TouchEnd:
		touchEvent(static_cast<QTouchEvent*>(event));
		break;
	default:
		break;
	}
	return QGraphicsView::viewportEvent(event);
}

/******************************************************************************************************************************/
void GraphicsView::touchEvent(QTouchEvent* te)
{
	QList<QTouchEvent::TouchPoint> points = te->touchPoints();
	if (deformationScene->isPinModeChecked())
		return; //if pin mode is checked, directed to mousepress event.

	if (!deformationScene->isMultitouchModeChecked() && points.count() == 2) {
		//zoom only in normal mode
		QTouchEvent::TouchPoint p0 = points.first();
		QTouchEvent::TouchPoint p1 = points.last();
		QLineF line1(p0.startPos(), p1.startPos());
		QLineF line2(p0.pos(), p1.pos());
		qreal scaleFactor = line2.length() / line1.length();
		deformationScene->zoom(scaleFactor + (1 - scaleFactor) / 1.05);
		return;
	}

	//if (te->type() == QEvent::TouchBegin) qWarning("Begin!");
	if (te->type() == QEvent::TouchEnd) {
		idToLocation.clear();
		std::vector<int> v;
		deformationScene->setPointsToRender(v);
		return;
	}

	// fix set of pins
	std::set<int> ids;
	for (int i = 0; i < points.size(); i++) {
		QTouchEvent::TouchPoint p = points[i];
		if (idToLocation.count(p.id()) == 0) {
			idToLocation[p.id()] = p.pos();
			touchToVertex[p.id()] = deformationScene->closestIndex(p.pos());
		}
		ids.insert(p.id());
	}

	std::map<int, QPointF>::iterator it = idToLocation.begin();
	std::set<int> toRemove;
	while (it != idToLocation.end()) {
		if (ids.count(it->first) == 0)
			toRemove.insert(it->first);

		++it;
	}

	for (std::set<int>::iterator iter = toRemove.begin();iter != toRemove.end(); ++iter)
		idToLocation.erase(*iter);

	// figure out if anything moved significantly
	double maxDistance = 0;

	for (int i = 0; i < points.size(); i++) {
		QPointF loc = points[i].pos();
		QPointF old = idToLocation[points[i].id()];
		QPointF diff = old - loc;
		double d = sqrt(diff.x() * diff.x() + diff.y() * diff.y());
		if (d > maxDistance)
			maxDistance = d;
	}

	if (maxDistance >= 1) {
		// moved more than one pixel!
		std::vector<int> ids;
		std::vector<Vector2D<double> > displacements;
		for (int i = 0; i < points.size(); i++) {
			QPointF displacement = points[i].pos()
					- idToLocation[points[i].id()];
			if (touchToVertex[points[i].id()] == -1)
				continue;

			ids.push_back(touchToVertex[points[i].id()]);
			displacements.push_back(
					deformationScene->screenToModelVec(displacement));
			idToLocation[points[i].id()] = points[i].pos();
		}
		if (ids.size() > 0)
			deformationScene->displaceMesh(ids, displacements);
	}

	std::vector<int> v;
	for (std::map<int, QPointF>::iterator it = idToLocation.begin();
			it != idToLocation.end(); ++it)
		v.push_back(touchToVertex[it->first]);
	deformationScene->setPointsToRender(v);
}
