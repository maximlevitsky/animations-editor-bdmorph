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

using std::max;
using std::min;


/******************************************************************************************************************************/
MainScene::MainScene(QWidget* parent) :
			model(NULL), currentKeyFrame(NULL), wireframeTransparency(0),
			QGLWidget(QGLFormat(QGL::SampleBuffers), parent),
			pinMode(false),
			multitouchMode(false)
{
    setAttribute(Qt::WA_AcceptTouchEvents);
    setAttribute(Qt::WA_StaticContents);
    setFocusPolicy(Qt::WheelFocus);
}

/******************************************************************************************************************************/
void MainScene::loadModel()
{
    QString filename = QFileDialog::getOpenFileName(0, tr("Choose model"), QString(), QLatin1String("*.off *.obj"));
    if (filename == "") return;

    qWarning("Deleting");
    if (currentKeyFrame) delete currentKeyFrame;
    if (model) delete model;

    qWarning("Making a new model");

    model = new MeshModel(filename);
	currentKeyFrame = new KVFModel(model);

	resetTexture();
    resetTransform();

    MainWindow* mainWindow = (MainWindow*)parentWidget();
	mainWindow->setStatusBarStatistics(currentKeyFrame->getNumVertices(), currentKeyFrame->getNumFaces());
    repaint();
}
/******************************************************************************************************************************/

void MainScene::saveModel()
{
	if ( !currentKeyFrame) return;
    QString filename = QFileDialog::getSaveFileName(0, tr("Choose file"), QString(), QLatin1String("*.off *.obj"));

    if ( filename == "" || (!(currentKeyFrame)) ) return;
    std::ofstream outfile(filename.toAscii());

	if (filename.endsWith("off"))
	{
		outfile << "OFF\n";
		outfile << currentKeyFrame->getNumVertices() << ' ' << currentKeyFrame->getNumFaces() << " 0\n"; // don't bother counting edges
		currentKeyFrame->saveVertices(outfile,filename);
		currentKeyFrame->saveFaces(outfile,filename);
	}
    
	if (filename.endsWith("obj"))
	{
		currentKeyFrame->saveVertices(outfile,filename);
		currentKeyFrame->saveTextureUVs(outfile,filename);
		currentKeyFrame->saveFaces(outfile,filename);
	}
}

/******************************************************************************************************************************/
void MainScene::chooseTexture()
{
    QString filename = QFileDialog::getOpenFileName(0, tr("Choose image"), QString(), QLatin1String("*.png *.jpg *.bmp"));

    glEnable(GL_TEXTURE_2D);
    if (filename == NULL)
    {
		resetTexture();
	}
    else {
    	makeCurrent();
		textureRef = bindTexture(QPixmap(filename),GL_TEXTURE_2D);
	}
    repaint();
}

/******************************************************************************************************************************/
void MainScene::resetTexture()
{
    makeCurrent();
	glEnable(GL_TEXTURE_2D);
	QPixmap texture = QPixmap(16,16);
    texture.fill(QColor(200,200,255));
    textureRef = bindTexture(texture);
    repaint();
}

/******************************************************************************************************************************/
void  MainScene::resetPoints()
{
	if ( !currentKeyFrame) return;
	currentKeyFrame->resetDeformations();
	repaint();
}
/******************************************************************************************************************************/
void MainScene::undoModel()
{
	if ( !currentKeyFrame) return;
	currentKeyFrame->historyUndo();
	repaint();
}

/******************************************************************************************************************************/
void MainScene::redoModel()
{
	if ( !currentKeyFrame) return;
	currentKeyFrame->historyRedo();
	repaint();
}

/******************************************************************************************************************************/
void MainScene::saveLog()
{
	if ( !currentKeyFrame) return;
    QString filename = QFileDialog::getSaveFileName(this, tr("Choose file"), QString(), QLatin1String("*.txt"));
    if (filename == "") return;

    std::ofstream outfile(filename.toAscii());
    currentKeyFrame->historySaveToFile(outfile);
}
/******************************************************************************************************************************/
void MainScene::runLog()
{
	if ( !currentKeyFrame) return;
    QString filename = QFileDialog::getOpenFileName(this, tr("Choose log"), QString(), QLatin1String("*.txt"));
    if (filename == "") return;
    std::ifstream infile(filename.toAscii());

    printf("STARTING log replay\n");
    TimeMeasurment t;

    int numSteps;
    infile >> numSteps;

    for (int step = 0; step < numSteps; step++)
    {
    	currentKeyFrame->historyLoadFromFile(infile);
        repaint();
    }

    printf("DONE WITH log replay (took %d msec)\n", t.measure_msec());
}

/******************************************************************************************************************************/
void MainScene::changeAlpha(int i)
{
	if (currentKeyFrame)
		currentKeyFrame->setAlpha((double) (i) / 100 * 2);
}

/******************************************************************************************************************************/
void MainScene::drawVFModeChanged(bool m)
{
	if (currentKeyFrame)
		currentKeyFrame->setDrawVFMode(m);
	repaint();
}

/******************************************************************************************************************************/
void MainScene::drawOrigVFModeChanged(bool m)
{
	if (currentKeyFrame)
		currentKeyFrame->setDrawOrigVFMode(m);
	repaint();
}

/******************************************************************************************************************************/
void MainScene::reuseVF()
{
	if (currentKeyFrame)
		currentKeyFrame->reuseVF();
	repaint();
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
	if (currentKeyFrame)
		currentKeyFrame->clearPins();
	repaint();
}
/******************************************************************************************************************************/

void MainScene::resetTransform()
{
	double maxZoomX = width() / currentKeyFrame->getWidth();
	double maxZoomY = height() / currentKeyFrame->getHeight();

	double maxZoom = std::min(maxZoomX,maxZoomY) * 0.9;
	modelWidth = currentKeyFrame->getWidth() * maxZoom;
	modelLocation = QPointF(0, 0);
	repaint();
}

/******************************************************************************************************************************/

void MainScene::initializeGL()
{
    glEnable(GL_TEXTURE_2D);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    glHint(GL_LINE_SMOOTH_HINT, GL_DONT_CARE);
    glHint(GL_POLYGON_SMOOTH_HINT, GL_DONT_CARE);

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

    if (!currentKeyFrame)
    	return;

    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_POLYGON_SMOOTH);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glLineWidth(1.5);

    double ratio = (currentKeyFrame->getWidth()) / modelWidth;
    double centerX = currentKeyFrame->getCenterX() - modelLocation.x() * ratio;
    double centerY = currentKeyFrame->getCenterY() - modelLocation.y() * ratio;
    double neededWidth = ratio * width();
    double neededHeight = ratio * height();

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(centerX - neededWidth/2  , centerX + neededWidth/2, centerY - neededHeight/2 , centerY + neededHeight/2, 0, 1);
    glMatrixMode(GL_MODELVIEW);
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, textureRef);

	/* render the model*/
	currentKeyFrame->render((double)wireframeTransparency/100);

	/* render the VF*/
	currentKeyFrame->renderVF();

	/* render selected vertices */
	glColor3f(1,0,0);
	for (int i = 0; i < selectedVertices.size(); i++)
		currentKeyFrame->renderVertex(selectedVertices[i], ratio);

	/* render pinned vertices */
	glColor3f(1,1,0);
	for (auto it = currentKeyFrame->getPinnedVertexes().begin(); it != currentKeyFrame->getPinnedVertexes().end(); it++)
		currentKeyFrame->renderVertex(*it, ratio);
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
		if (disps.size() > 0)
			currentKeyFrame->displaceMesh(disps);
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
    if (!currentKeyFrame) return;

    Vertex v = closestIndex(pos);
    if (v == -1) return;

	if ((QApplication::keyboardModifiers() & Qt::ShiftModifier) || pinMode)
	{
		currentKeyFrame->togglePinVertex(v);

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
    if (!currentKeyFrame) return;
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
        QPointF diff = curPos - oldPos;

        Vertex selectedVertex = selectedVertices[0];
        diff *= currentKeyFrame->getWidth() / modelWidth;

        std::cout << "Mouse move delta:" << diff.rx() << "-" << diff.ry() << std::endl;

        if (diff.rx() == 0 && diff.ry() == 0)
        	return;

        std::set<DisplacedVertex> disps;
        disps.insert(DisplacedVertex(selectedVertex, Vector2D<double>(diff.x(),-diff.y())));
		currentKeyFrame->displaceMesh(disps);
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
	if (!currentKeyFrame)
		return -1;
	return currentKeyFrame->getClosestVertex(screenToModel(pos));
}

/******************************************************************************************************************************/
Point2 MainScene::screenToModel(QPointF pos)
{
	pos -= modelLocation;
	pos -= QPointF((double)width()/2,(double)height()/2);
	pos *= currentKeyFrame->getWidth() / modelWidth;
	pos += QPointF(currentKeyFrame->getCenterX(), currentKeyFrame->getCenterY());

	return Point2(pos.x(), pos.y());
}
/******************************************************************************************************************************/
