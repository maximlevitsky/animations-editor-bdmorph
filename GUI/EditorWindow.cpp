
#include <iostream>
#include <fstream>
#include <ctime>
#include <stdio.h>
#include <unistd.h>
#include <QtGui>
#include <QtOpenGL>

#include "EditorWindow.h"
#include "MeshModel.h"
#include "KVFModel.h"
#include "OutlineModel.h"
#include "VideoModel.h"
#include <assert.h>

using std::max;
using std::min;


/******************************************************************************************************************************/
EditorWindow::EditorWindow(QWidget* parent) :
			kvfModel(NULL), renderModel(NULL), outlineModel(NULL) ,wireframeTransparency(0),
			QGLWidget(QGLFormat(QGL::SampleBuffers), parent),
			pinMode(false),
			multitouchMode(false),
			drawVF(false),
			drawVFOrig(false),
			disableEdit(false),
			hoveredVertex(-1),
			hoveredFace(-1),
			showSelection(false)
{
    setAttribute(Qt::WA_AcceptTouchEvents);
    setAttribute(Qt::WA_StaticContents);
    setFocusPolicy(Qt::WheelFocus);
    setMouseTracking(true);
    setContextMenuPolicy(Qt::ActionsContextMenu);
}

/******************************************************************************************************************************/
void  EditorWindow::onResetPoints()
{
	if (disableEdit || !renderModel) return;
	renderModel->historyReset();
	emit modelEdited(renderModel);
	repaint();

	/* TODO: updat alpha in GUI */
}
/******************************************************************************************************************************/
void EditorWindow::onUndoModel()
{
	if (disableEdit || !renderModel) return;
	renderModel->historyUndo();
	repaint();
	emit modelEdited(renderModel);

	/* TODO: updat alpha in GUI */
}

/******************************************************************************************************************************/
void EditorWindow::onRedoModel()
{
	if (disableEdit || !renderModel) return;
	renderModel->historyRedo();
	repaint();
	emit modelEdited(renderModel);

	/* TODO: updat alpha in GUI */
}

/******************************************************************************************************************************/
void EditorWindow::onSaveLog()
{
	if ( !kvfModel || disableEdit) return;
    QString filename = QFileDialog::getSaveFileName(this, tr("Choose file"), QString(), QLatin1String("*.txt"));
    if (filename == "") return;

    std::ofstream outfile(filename.toAscii());
    kvfModel->historySaveToFile(outfile);
}
/******************************************************************************************************************************/
void EditorWindow::onRunLog()
{
	if ( !kvfModel || disableEdit) return;
    QString filename = QFileDialog::getOpenFileName(this, tr("Choose log"), QString(), QLatin1String("*.txt"));
    if (filename == "") return;
    std::ifstream infile(filename.toAscii());

    printf("STARTING log replay\n");
    TimeMeasurment t;

    int numSteps;
    infile >> numSteps;

    /* TODO: rework and update alpha in GIU*/

    for (int step = 0; step < numSteps; step++)
    {
    	kvfModel->historyLoadFromFile(infile);
        repaint();
        emit modelEdited(kvfModel);
    }

    printf("DONE WITH log replay (took %f msec)\n", t.measure_msec());
    kvfModel->historySnapshot();

}

/******************************************************************************************************************************/
void EditorWindow::onChangeAlpha(int i)
{
	if ( !kvfModel || disableEdit) return;
	kvfModel->setAlpha((double) (i) / 100 * 8);
}

/******************************************************************************************************************************/
void EditorWindow::onDrawVFModeChanged(bool m)
{
	drawVF = m;
	repaint();
}

/******************************************************************************************************************************/
void EditorWindow::onDrawOrigVFModeChanged(bool m)
{
	drawVFOrig = m;
	repaint();
}

void EditorWindow::onShowSelectionChanged(bool m)
{
	showSelection = m;
	if (showSelection) {
		setMouseTracking(true);
	} else {
		setMouseTracking(false);
		hoveredFace = -1;
		hoveredVertex = -1;
		emit selectionChanged(-1,-1);
	}
	repaint();
}

/******************************************************************************************************************************/
void EditorWindow::onReuseVF()
{
	if ( !kvfModel || disableEdit) return;
	kvfModel->applyVF();
	emit modelEdited(kvfModel);
	repaint();
}

/******************************************************************************************************************************/
void EditorWindow::onPinModeChanged(bool m)
{
	pinMode = m;
}
/******************************************************************************************************************************/
void EditorWindow::onChangeWireframe(int i)
{
	wireframeTransparency = i;
	repaint();
}

/******************************************************************************************************************************/
void EditorWindow::onClearPins()
{
	if (kvfModel)
		kvfModel->clearPins();
	repaint();
}
/******************************************************************************************************************************/

void EditorWindow::onResetTransform()
{
	if (!renderModel) return;

	double maxZoomX = width() / renderModel->getWidth();
	double maxZoomY = height() / renderModel->getHeight();

	double maxZoom = std::min(maxZoomX,maxZoomY);
	modelWidth = renderModel->getWidth() * maxZoom * 0.5;
	modelLocation = QPointF(0, 0);
	repaint();
}

/******************************************************************************************************************************/
void EditorWindow::onFrameSwitched(MeshModel* model)
{
	hoveredVertex = -1;
	hoveredFace = -1;
	renderModel = model;
	kvfModel = dynamic_cast<KVFModel*>(model);
	outlineModel = dynamic_cast<OutlineModel*>(model);

	if (outlineModel)
		onResetTransform();
	repaint();
}

/******************************************************************************************************************************/

void EditorWindow::onVideoModelLoaded(VideoModel* model)
{
	if (!model)
	{
		renderModel = NULL;
		kvfModel = NULL;
		outlineModel = NULL;
	} else
	{
		renderModel = model;
		onResetTransform();
	}
}

/******************************************************************************************************************************/

void EditorWindow::onTextureChanged(GLuint texture)
{
	textureRef = texture;
	makeCurrent();
	glBindTexture(GL_TEXTURE_2D, textureRef);
	repaint();
}

/******************************************************************************************************************************/

void EditorWindow::initializeGL()
{
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);
    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_POLYGON_SMOOTH);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

/******************************************************************************************************************************/
void EditorWindow::resizeGL(int w, int h)
{
	repaint();
}
/******************************************************************************************************************************/
void EditorWindow::paintGL()
{
    glViewport(0,0,(GLint)width(), (GLint)height());
    glClearColor(1.,1.,1., 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (!renderModel)
    	return;

    double ratio = (renderModel->getWidth()) / modelWidth;

    double centerX      = (-ratio * modelLocation.x()) + renderModel->center.x;
    double centerY      = (-ratio * modelLocation.y()) + renderModel->center.y;
    double neededWidth  = ratio * width();
    double neededHeight = ratio * height();

    /* Setup projection */
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(centerX - neededWidth/2  , centerX + neededWidth/2, centerY - neededHeight/2 , centerY + neededHeight/2, 0, 1);

	/* render the model*/
	renderModel->renderFaces();
	if (wireframeTransparency) {
		glColor4f(0,0,0,(double)wireframeTransparency/100.0);
		renderModel->renderWireframe();
	}

	/* Some models will render stuff on top, like pined vertexes, extra wireframes, etc*/
	renderModel->renderOverlay(ratio);


	/* Render hovered vertices */
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	if (hoveredFace != -1)
	{
		glColor4f(0,0,1,0.5);
		renderModel->renderFace(hoveredFace);
	}

	if (hoveredVertex != -1)
	{
		glColor3f(0,0,1);
		renderModel->renderVertex(hoveredVertex, ratio);
	}

	/* render selected vertices */
	glColor3f(1,0,0);
	for (unsigned int i = 0; i < selectedVertices.size(); i++)
		renderModel->renderVertex(selectedVertices[i], ratio);


	/* render VF of KVF model */
	if (kvfModel)
	{
		if (drawVFOrig)
			kvfModel->renderVFOrig();
		if (drawVF)
			kvfModel->renderVF();
	}
}

/******************************************************************************************************************************/
bool EditorWindow::event(QEvent *event)
{
	switch (event->type()) {
	case QEvent::TouchBegin:
	case QEvent::TouchUpdate:
	case QEvent::TouchEnd:
		return touchEvent(static_cast<QTouchEvent*>(event));
	default:
		return QWidget::event(event);
	}
}

/******************************************************************************************************************************/
bool EditorWindow::touchEvent(QTouchEvent* te)
{
	if (pinMode)
		return false; //if pin mode is checked, directed to mousepress event.

	if (!kvfModel || disableEdit)
		return false;

	QList<QTouchEvent::TouchPoint> touchPoints = te->touchPoints();

	if (!multitouchMode && touchPoints.count() == 2) {
		//zoom only in normal mode
		QTouchEvent::TouchPoint p0 = touchPoints.first();
		QTouchEvent::TouchPoint p1 = touchPoints.last();
		QLineF line1(p0.startPos(), p1.startPos());
		QLineF line2(p0.pos(), p1.pos());
		qreal scaleFactor = line2.length() / line1.length();
		zoom(scaleFactor + (1 - scaleFactor) / 1.05);
		return true;
	}

	if (te->type() == QEvent::TouchEnd)
	{
		touchPointLocations.clear();
		selectedVertices.clear();
		return true;
	}

	// fix set of pins
	std::set<int> ids;

	for (int i = 0; i < touchPoints.size(); i++)
	{
		QTouchEvent::TouchPoint p = touchPoints[i];
		if (touchPointLocations.count(p.id()) == 0) {
			touchPointLocations[p.id()] = p.pos();
			touchToVertex[p.id()] = kvfModel->getClosestVertex(screenToModel(p.pos()));
		}
		ids.insert(p.id());
	}

	std::map<int, QPointF>::iterator it = touchPointLocations.begin();
	std::set<int> toRemove;
	while (it != touchPointLocations.end()) {
		if (ids.count(it->first) == 0)
			toRemove.insert(it->first);
		++it;
	}

	for (std::set<int>::iterator iter = toRemove.begin();iter != toRemove.end(); ++iter)
		touchPointLocations.erase(*iter);

	// figure out if anything moved significantly
	double maxDistance = 0;

	for (int i = 0; i < touchPoints.size(); i++)
	{
		QPointF loc = touchPoints[i].pos();
		QPointF old = touchPointLocations[touchPoints[i].id()];
		QPointF diff = old - loc;
		double d = sqrt(diff.x() * diff.x() + diff.y() * diff.y());
		if (d > maxDistance)
			maxDistance = d;
	}

	if (maxDistance >= 1)
	{
		// moved more than one pixel!
		std::set<DisplacedVertex> disps;

		for (int i = 0; i < touchPoints.size(); i++)
		{
			if (touchToVertex[touchPoints[i].id()] == -1)
				continue;

			Vector2 displacement = screenToModel(touchPoints[i].pos()) - screenToModel(touchPointLocations[touchPoints[i].id()]);

			disps.insert(DisplacedVertex(touchToVertex[touchPoints[i].id()], displacement));
			touchPointLocations[touchPoints[i].id()] = touchPoints[i].pos();
		}
		if (disps.size() > 0) {
			kvfModel->calculateVF(disps);

			if (!drawVF && !drawVFOrig)
				kvfModel->applyVFLogSpiral();

			emit modelEdited(kvfModel);
		}
	}

	selectedVertices.clear();
	for (std::map<int, QPointF>::iterator it = touchPointLocations.begin();it != touchPointLocations.end(); ++it)
		selectedVertices.push_back(touchToVertex[it->first]);

	return true;
}
/******************************************************************************************************************************/
void EditorWindow::keyPressEvent(QKeyEvent *e)
{
	if (disableEdit)
		return;

    switch(e->key()) {
    case Qt::Key_Up:
    	moveUp();
    	break;
    case Qt::Key_Down:
    	moveDown();
    	break;
    case Qt::Key_Left:
    	moveLeft();
    	break;
    case Qt::Key_Right:
    	moveRight();
    	break;
    case Qt::Key_Plus:
    case Qt::Key_Equal:
    	zoomIn();
    	break;
    case Qt::Key_Minus:
    case Qt::Key_Underscore:
    	zoomOut();
    	break;
    case Qt::Key_V:
    	onReuseVF();
    	break;
    }
    update();
}

/******************************************************************************************************************************/
void EditorWindow::mousePressEvent(QMouseEvent *event)
{
	mouseMoved = false;
	mouseLeft = event->buttons() & Qt::LeftButton;


    QPointF pos = event->pos(); // (0,0) is upper left
    pos.setY(height()-pos.y()-1);
    lastMousePos = pos;

	if (!renderModel || disableEdit)
		return;

    setCursor(Qt::BlankCursor);

    Point2 modelPos = screenToModel(pos);

	if ((QApplication::keyboardModifiers() == Qt::NoModifier) && !pinMode && kvfModel)
	{
		Vertex v = kvfModel->getClosestVertex(modelPos);
		/* Select this vertex */
		selectedVertices.clear();
		selectedVertices.push_back(v);
		repaint();
		return;
	}

	if (event->buttons() & Qt::LeftButton)
		renderModel->mousePressAction(modelPos,getRadius());

	repaint();
	return;
}
/******************************************************************************************************************************/
void EditorWindow::mouseMoveEvent(QMouseEvent *event)
{
	mouseLeft = event->buttons() & Qt::LeftButton;

	if (!renderModel)
		return;

    Qt::KeyboardModifiers mods =  QApplication::keyboardModifiers();

	QPointF oldPos = lastMousePos;
    QPointF curPos = event->pos();
    curPos.setY(height()-curPos.y()-1);

    lastMousePos = curPos;

    /* Plain mouse move */
    if (event->buttons() == Qt::NoButton && showSelection)
    {
		hoveredVertex = renderModel->getClosestVertex(screenToModel(curPos));
		hoveredFace = renderModel->getFaceUnderPoint(screenToModel(curPos));
		emit selectionChanged(hoveredVertex,hoveredFace);
		repaint();
		return;
    }

    /* Right button pressed - always mouse move */
	if ((event->buttons() & Qt::RightButton))
	{
        QPointF diff = curPos - oldPos;
		move(QPointF(diff.x(),diff.y()));
		repaint();
		return;
	}

	/* Left button pressed - edit */
	if ((event->buttons() & Qt::LeftButton))
    {
		/* See if we can deform the mesh */
		if (!disableEdit && !(mods & Qt::ShiftModifier) && !pinMode && kvfModel && kvfModel->getPinnedVertexes().size() != 0 )
		{
			QPointF diff = curPos - oldPos;
			Vertex selectedVertex = selectedVertices[0];
			diff *= kvfModel->getWidth() / modelWidth;
			if (diff.rx() == 0 && diff.ry() == 0)
				return;

			std::cout << "Mouse move delta:(" << diff.rx() << "," << diff.ry() << ")" << std::endl;

			TimeMeasurment t;

			std::set<DisplacedVertex> disps;
			disps.insert(DisplacedVertex(selectedVertex, Vector2(diff.x(),diff.y())));
			kvfModel->calculateVF(disps);

			if (!drawVF && !drawVFOrig)
				kvfModel->applyVFLogSpiral();

			emit modelEdited(kvfModel);
			repaint();

			double msec = t.measure_msec();
			printf("KVF: Total frame time (with rendering): %f msec (%f FPS)\n\n", msec, 1000.0 / msec);
			return;
		}

		if (!renderModel->moveAction(screenToModel(oldPos), screenToModel(curPos), getRadius())) {
			/* If all failed, move the model */
			QPointF diff = curPos - oldPos;
			move(QPointF(diff.x(),diff.y()));
			repaint();
			return;
		}

		mouseMoved = true;
		repaint();
    }
}

/******************************************************************************************************************************/
void EditorWindow::mouseReleaseEvent(QMouseEvent *event)
{
    setCursor(Qt::ArrowCursor);
    selectedVertices.clear();
    mouseMoved = false;
    if (!renderModel)
    	return;

    renderModel->historySnapshot();
    QPointF curPos = event->pos();
    curPos.setY(height()-curPos.y()-1);
    renderModel->mouseReleaseAction(screenToModel(curPos),mouseMoved, getRadius(), !mouseLeft);
    update();
}

/******************************************************************************************************************************/
void EditorWindow::wheelEvent(QWheelEvent *event)
{
	if (!renderModel)
		return;

	QPointF pos = event->pos();
	pos.setY(height()-pos.y()-1);

	Point2 posBefore = screenToModel(pos);
	zoom(event->delta() > 0 ? ZOOM_FACTOR : 1 / ZOOM_FACTOR);
	Point2 posAfter = screenToModel(pos);

	Vector2 diff = (posAfter-posBefore) * (modelWidth / renderModel->getWidth());
	QPointF m = QPointF(diff.x,diff.y);
	move(m);

	update();
}

/******************************************************************************************************************************/
void EditorWindow::zoom(double factor)
{
	modelWidth *= factor;
}

/******************************************************************************************************************************/
void EditorWindow::move(QPointF direction)
{
	modelLocation += direction;
}

/******************************************************************************************************************************/
Point2 EditorWindow::screenToModel(QPointF pos)
{
	pos -= modelLocation;
	pos -= QPointF((double)width()/2,(double)height()/2);
	pos *= renderModel->getWidth() / modelWidth;
	pos.rx() += renderModel->center.x;
	pos.ry() += renderModel->center.y;
	return Point2(pos.x(), pos.y());
}


double EditorWindow::getRadius()
{
	Point2 modelPos = screenToModel(QPointF(0,0));
	Point2 modelPos2 = screenToModel(QPointF(0,5));
	return modelPos.distance(modelPos2);
}
/******************************************************************************************************************************/


void EditorWindow::onAnimationStarted()
{
	disableEdit = true;
}

void EditorWindow::onAnimationStopped()
{
	disableEdit = false;
}
