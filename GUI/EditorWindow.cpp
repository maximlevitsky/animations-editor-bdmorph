
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
#include "VideoModel.h"
#include <assert.h>

using std::max;
using std::min;


/******************************************************************************************************************************/
EditorWindow::EditorWindow(QWidget* parent) :
			editModel(NULL), renderModel(NULL), wireframeTransparency(0),
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
void  EditorWindow::resetPoints()
{
	if ( !editModel || disableEdit) return;
	editModel->resetDeformations();
	emit modelEdited(editModel);
	repaint();
}
/******************************************************************************************************************************/
void EditorWindow::undoModel()
{
	if ( !editModel || disableEdit) return;
	editModel->historyUndo();
	emit modelEdited(editModel);
	repaint();
}

/******************************************************************************************************************************/
void EditorWindow::redoModel()
{
	if ( !editModel || disableEdit) return;
	editModel->historyRedo();
	emit modelEdited(editModel);
	repaint();
}

/******************************************************************************************************************************/
void EditorWindow::saveLog()
{
	if ( !editModel || disableEdit) return;
    QString filename = QFileDialog::getSaveFileName(this, tr("Choose file"), QString(), QLatin1String("*.txt"));
    if (filename == "") return;

    std::ofstream outfile(filename.toAscii());
    editModel->historySaveToFile(outfile);
}
/******************************************************************************************************************************/
void EditorWindow::runLog()
{
	if ( !editModel || disableEdit) return;
    QString filename = QFileDialog::getOpenFileName(this, tr("Choose log"), QString(), QLatin1String("*.txt"));
    if (filename == "") return;
    std::ifstream infile(filename.toAscii());

    printf("STARTING log replay\n");
    TimeMeasurment t;

    int numSteps;
    infile >> numSteps;

    for (int step = 0; step < numSteps; step++)
    {
    	editModel->historyLoadFromFile(infile);
        repaint();
        emit modelEdited(editModel);
    }

    printf("DONE WITH log replay (took %d msec)\n", t.measure_msec());

}

/******************************************************************************************************************************/
void EditorWindow::changeAlpha(int i)
{
	if ( !editModel || disableEdit) return;
	editModel->setAlpha((double) (i) / 100 * 2);
}

/******************************************************************************************************************************/
void EditorWindow::drawVFModeChanged(bool m)
{
	drawVF = m;
	repaint();
}

/******************************************************************************************************************************/
void EditorWindow::drawOrigVFModeChanged(bool m)
{
	drawVFOrig = m;
	repaint();
}

void EditorWindow::showSelectionChanged(bool m)
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
void EditorWindow::reuseVF()
{
	if ( !editModel || disableEdit) return;
	editModel->applyVF();
	emit modelEdited(editModel);
	repaint();
}

/******************************************************************************************************************************/
void EditorWindow::pinModeChanged(bool m)
{
	pinMode = m;
}
/******************************************************************************************************************************/
void EditorWindow::changeWireframe(int i)
{
	wireframeTransparency = i;
	repaint();
}

/******************************************************************************************************************************/
void EditorWindow::clearPins()
{
	if (editModel)
		editModel->clearPins();
	repaint();
}
/******************************************************************************************************************************/

void EditorWindow::resetTransform()
{
	if (disableEdit) return;
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
	renderModel = model;
	editModel = dynamic_cast<KVFModel*>(model);
	repaint();
}

/******************************************************************************************************************************/

void EditorWindow::onVideoModelLoaded(VideoModel* model)
{
	if (!model)
	{
		renderModel = NULL;
		editModel = NULL;
	} else
	{
		renderModel = model;
		resetTransform();
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
    glHint(GL_LINE_SMOOTH_HINT, GL_DONT_CARE);
    glHint(GL_POLYGON_SMOOTH_HINT, GL_DONT_CARE);
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

    double centerX      = -ratio * modelLocation.x();
    double centerY      = -ratio * modelLocation.y();
    double neededWidth  = ratio * width();
    double neededHeight = ratio * height();

    /* Setup projection */
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(centerX - neededWidth/2  , centerX + neededWidth/2, centerY - neededHeight/2 , centerY + neededHeight/2, 0, 1);

    //glBindTexture(GL_TEXTURE_2D, textureRef);

	/* render the model*/
	renderModel->render((double)wireframeTransparency/100);

	/* render selected vertices */
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

	glColor3f(1,0,0);
	for (unsigned int i = 0; i < selectedVertices.size(); i++)
		renderModel->renderVertex(selectedVertices[i], ratio);

	/* render pinned vertices */
	if (editModel)
	{
		glColor3f(1,1,0);
		for (auto it = editModel->getPinnedVertexes().begin(); it != editModel->getPinnedVertexes().end(); it++)
			editModel->renderVertex(*it, ratio);

		if (drawVFOrig)
			editModel->renderVFOrig();
		if (drawVF)
			editModel->renderVF();
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

	if (!editModel || disableEdit)
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
			touchToVertex[p.id()] = editModel->getClosestVertex(screenToModel(p.pos()));
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
			editModel->calculateVF(disps);

			if (!drawVF && !drawVFOrig)
				editModel->applyVFLogSpiral();

			emit modelEdited(editModel);
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
    	reuseVF();
    	break;
    }
    update();
}
/******************************************************************************************************************************/
void EditorWindow::mousePressEvent(QMouseEvent *event)
{
	if (disableEdit || !editModel)
		return;

    setCursor(Qt::BlankCursor);

    QPointF pos = event->pos(); // (0,0) is upper left
    pos.setY(height()-pos.y()-1);
    lastMousePos = pos;

    Vertex v = editModel->getClosestVertex(screenToModel(pos));
    if (v == -1) return;

	if ((QApplication::keyboardModifiers() & Qt::ShiftModifier) || pinMode)
	{
		editModel->togglePinVertex(v);
		repaint();

	} else
	{
		/* Select this vertex */
		selectedVertices.clear();
		selectedVertices.push_back(v);
		repaint();
	}
}
/******************************************************************************************************************************/
void EditorWindow::mouseMoveEvent(QMouseEvent *event)
{
    Qt::KeyboardModifiers mods =  QApplication::keyboardModifiers();

	if (disableEdit || !editModel || multitouchMode)
		return;

	QPointF oldPos = lastMousePos;
    QPointF curPos = event->pos();
    curPos.setY(height()-curPos.y()-1);

    lastMousePos = curPos;

	if ((event->buttons() & Qt::RightButton))
	{
        QPointF diff = curPos - oldPos;
		move(QPointF(diff.x(),diff.y()));
		repaint();
	}
	else if ((event->buttons() & Qt::LeftButton) && (!(mods & Qt::ShiftModifier)) && !pinMode && selectedVertices.size())
    {
		QPointF diff = curPos - oldPos;
		Vertex selectedVertex = selectedVertices[0];
		diff *= editModel->getWidth() / modelWidth;

		if (diff.rx() == 0 && diff.ry() == 0)
			return;

		std::cout << "Mouse move delta:(" << diff.rx() << "," << diff.ry() << ")" << std::endl;

		std::set<DisplacedVertex> disps;
		disps.insert(DisplacedVertex(selectedVertex, Vector2D<double>(diff.x(),diff.y())));
		editModel->calculateVF(disps);

		if (!drawVF && !drawVFOrig)
			editModel->applyVFLogSpiral();

		emit modelEdited(editModel);
    	repaint();
    }
	else if (event->buttons() == Qt::NoButton && showSelection)
	{
		hoveredVertex = editModel->getClosestVertex(screenToModel(curPos));
		hoveredFace = editModel->getFaceUnderPoint(screenToModel(curPos));
		emit selectionChanged(hoveredVertex,hoveredFace);
		repaint();
	}
}

/******************************************************************************************************************************/
void EditorWindow::mouseReleaseEvent(QMouseEvent *event)
{
    setCursor(Qt::ArrowCursor);
    selectedVertices.clear();
    update();
}

/******************************************************************************************************************************/
void EditorWindow::wheelEvent(QWheelEvent *event)
{
	if (disableEdit)
		return;

	zoom(event->delta() > 0 ? ZOOM_FACTOR : 1 / ZOOM_FACTOR);
	update();
}

/******************************************************************************************************************************/
Point2 EditorWindow::screenToModel(QPointF pos)
{
	pos -= modelLocation;
	pos -= QPointF((double)width()/2,(double)height()/2);
	pos *= renderModel->getWidth() / modelWidth;
	return Point2(pos.x(), pos.y());
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
