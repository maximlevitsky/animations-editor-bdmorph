#include "MainScene.h"

#define TANGENT_WIDTH 20
#include <QtGui>
#include <QtOpenGL>
#include <QPalette>
#include <iostream>
#include <fstream>
#include <QGesture>
#include <ctime>
#include <stdio.h>
#include <unistd.h>
#include "MainWindow.h"
#include <assert.h>

using std::max;
using std::min;


/******************************************************************************************************************************/
MainScene::MainScene(QWidget* parent) :
			editModel(NULL), renderModel(NULL), wireframeTransparency(0),
			QGLWidget(QGLFormat(QGL::SampleBuffers), parent),
			pinMode(false),
			multitouchMode(false),
			drawVF(false),
			drawVFOrig(false)
{
    setAttribute(Qt::WA_AcceptTouchEvents);
    setAttribute(Qt::WA_StaticContents);
    setFocusPolicy(Qt::WheelFocus);
}

/******************************************************************************************************************************/
void  MainScene::resetPoints()
{
	if ( !editModel) return;
	editModel->resetDeformations();
	emit modelEdited(editModel);
	repaint();
}
/******************************************************************************************************************************/
void MainScene::undoModel()
{
	if ( !editModel) return;
	editModel->historyUndo();
	emit modelEdited(editModel);
	repaint();
}

/******************************************************************************************************************************/
void MainScene::redoModel()
{
	if ( !editModel) return;
	editModel->historyRedo();
	emit modelEdited(editModel);
	repaint();
}

/******************************************************************************************************************************/
void MainScene::saveLog()
{
	if ( !editModel) return;
    QString filename = QFileDialog::getSaveFileName(this, tr("Choose file"), QString(), QLatin1String("*.txt"));
    if (filename == "") return;

    std::ofstream outfile(filename.toAscii());
    editModel->historySaveToFile(outfile);
}
/******************************************************************************************************************************/
void MainScene::runLog()
{
	if ( !editModel) return;
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
    }

    printf("DONE WITH log replay (took %d msec)\n", t.measure_msec());
	emit modelEdited(editModel);
}

/******************************************************************************************************************************/
void MainScene::changeAlpha(int i)
{
	if (editModel)
		editModel->setAlpha((double) (i) / 100 * 2);
}

/******************************************************************************************************************************/
void MainScene::drawVFModeChanged(bool m)
{
	drawVF = m;
	repaint();
}

/******************************************************************************************************************************/
void MainScene::drawOrigVFModeChanged(bool m)
{
	drawVFOrig = m;
	repaint();
}

/******************************************************************************************************************************/
void MainScene::reuseVF()
{
	if (editModel) {
		editModel->applyVF();
		emit modelEdited(editModel);
		repaint();
	}
}

/******************************************************************************************************************************/
void MainScene::pinModeChanged(bool m)
{
	pinMode = m;
}
/******************************************************************************************************************************/
void MainScene::changeWireframe(int i)
{
	wireframeTransparency = i;
	repaint();
}

/******************************************************************************************************************************/
void MainScene::clearPins()
{
	if (editModel)
		editModel->clearPins();
	repaint();
}
/******************************************************************************************************************************/

void MainScene::resetTransform()
{
	if (!renderModel) return;
	double maxZoomX = width() / renderModel->getWidth();
	double maxZoomY = height() / renderModel->getHeight();

	double maxZoom = std::min(maxZoomX,maxZoomY) * 0.9;
	modelWidth = renderModel->getWidth() * maxZoom;
	modelLocation = QPointF(0, 0);
	repaint();
}

/******************************************************************************************************************************/
void MainScene::onFrameSwitched(MeshModel* model)
{
	renderModel = model;
	editModel = dynamic_cast<KVFModel*>(model);
	repaint();
}

/******************************************************************************************************************************/

void MainScene::onVideoModelLoaded(VideoModel* model)
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

void MainScene::onTextureChanged(GLuint texture)
{
	textureRef = texture;
	makeCurrent();
	glBindTexture(GL_TEXTURE_2D, textureRef);
	repaint();
}

/******************************************************************************************************************************/

void MainScene::initializeGL()
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
void MainScene::resizeGL(int w, int h)
{
	repaint();
}
/******************************************************************************************************************************/
void MainScene::paintGL()
{
    glViewport(0,0,(GLint)width(), (GLint)height());
    glClearColor(1.,1.,1., 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (!renderModel)
    	return;

    double ratio = (renderModel->getWidth()) / modelWidth;
    double centerX = renderModel->getCenterX() - modelLocation.x() * ratio;
    double centerY = renderModel->getCenterY() - modelLocation.y() * ratio;
    double neededWidth = ratio * width();
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

	glColor4f(1,0,0,1);
	for (int i = 0; i < selectedVertices.size(); i++)
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
bool MainScene::event(QEvent *event)
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
bool MainScene::touchEvent(QTouchEvent* te)
{
	if (pinMode)
		return false; //if pin mode is checked, directed to mousepress event.

	if (!editModel)
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
			touchToVertex[p.id()] = closestIndex(p.pos());
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
void MainScene::keyPressEvent(QKeyEvent *e)
{
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
void MainScene::mousePressEvent(QMouseEvent *event)
{
    setCursor(Qt::BlankCursor);
    QPointF pos = event->pos(); // (0,0) is upper left
    pos.setY(height()-pos.y()-1);

    QPointF origPos = pos;
    lastMousePos = event->pos();
    if (!editModel) return;

    Vertex v = closestIndex(pos);
    if (v == -1) return;

	if ((QApplication::keyboardModifiers() & Qt::ShiftModifier) || pinMode)
	{
		editModel->togglePinVertex(v);

	} else
	{
		/* Select this vertex */
		selectedVertices.clear();
		selectedVertices.push_back(v);
	}
}
/******************************************************************************************************************************/
void MainScene::mouseMoveEvent(QMouseEvent *event)
{
    Qt::KeyboardModifiers mods =  QApplication::keyboardModifiers();

	QPointF oldPos = lastMousePos;
    QPointF curPos = event->pos();
    lastMousePos = curPos;

    if (multitouchMode)
    	return;

	if ((event->buttons() & Qt::RightButton))
	{
        QPointF diff = curPos - oldPos;
		move(QPointF(diff.x(),-diff.y()));
	} 

    if ((event->buttons() & Qt::LeftButton) && (!(mods & Qt::ShiftModifier)) && !pinMode && selectedVertices.size())
    {

    	if (editModel) {
			QPointF diff = curPos - oldPos;
			Vertex selectedVertex = selectedVertices[0];
			diff *= editModel->getWidth() / modelWidth;

			std::cout << "Mouse move delta:" << diff.rx() << "-" << diff.ry() << std::endl;

			if (diff.rx() == 0 && diff.ry() == 0)
				return;

			std::set<DisplacedVertex> disps;
			disps.insert(DisplacedVertex(selectedVertex, Vector2D<double>(diff.x(),-diff.y())));
			editModel->calculateVF(disps);

			if (!drawVF && !drawVFOrig)
				editModel->applyVFLogSpiral();

			emit modelEdited(editModel);
    	}
        update();
    }
    repaint();
}

/******************************************************************************************************************************/
void MainScene::mouseReleaseEvent(QMouseEvent *event)
{
    setCursor(Qt::ArrowCursor);
    selectedVertices.clear();
    update();
}

/******************************************************************************************************************************/
void MainScene::wheelEvent(QWheelEvent *event)
{
	zoom(event->delta() > 0 ? ZOOM_FACTOR : 1 / ZOOM_FACTOR);
	update();
}

/******************************************************************************************************************************/
int MainScene::closestIndex(QPointF pos)
{
	if (!renderModel)
		return -1;
	return renderModel->getClosestVertex(screenToModel(pos));
}

/******************************************************************************************************************************/
Point2 MainScene::screenToModel(QPointF pos)
{
	pos -= modelLocation;
	pos -= QPointF((double)width()/2,(double)height()/2);
	pos *= renderModel->getWidth() / modelWidth;
	pos += QPointF(renderModel->getCenterX(), renderModel->getCenterY());

	return Point2(pos.x(), pos.y());
}
/******************************************************************************************************************************/
