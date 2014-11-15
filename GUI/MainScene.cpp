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

using std::max;
using std::min;

/******************************************************************************************************************************/
MainScene::MainScene(QWidget* parent) :
			model(NULL), keyframeModel(NULL), wireframeTransparency(0),
			QGLWidget(QGLFormat(QGL::SampleBuffers), parent),
			pinMode(false),
			multitouchMode(false)
{
    setAttribute(Qt::WA_AcceptTouchEvents);
    setAttribute(Qt::WA_StaticContents);
    setFocusPolicy(Qt::WheelFocus);

    makeCurrent();
    glEnable(GL_TEXTURE_2D);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    QPixmap texture = QPixmap(16,16);
    texture.fill(QColor(200,200,255));
    textureRef = bindTexture(texture);
}

/******************************************************************************************************************************/
void MainScene::loadModel()
{
    QString filename = QFileDialog::getOpenFileName(0, tr("Choose model"), QString(), QLatin1String("*.off *.obj"));
    if (filename == "") return;

    qWarning("Deleting");
    if (keyframeModel) delete keyframeModel;
    if (model) delete model;

    qWarning("Making a new model");

    model = new MeshModel(filename);

	keyframeModel = new KVFModel(model);
    keyframeModel->setWireframeTrans((double)wireframeTransparency/100);


	QPixmap texture = QPixmap(16,16);
    texture.fill(QColor(200,200,255));
    textureRef = bindTexture(texture);
    qWarning("Done loading");

	modelWidth = keyframeModel->getWidth() * 200;
	modelLocation = QPointF(width()/2, height()/2);

    QString verticesNum = QString::number(keyframeModel->getNumVertices());
	QString facesNum = QString::number(keyframeModel->getNumFaces());

	/* TODO: show facenum/vertexnum in statusbar*/
    repaint();
}

void MainScene::saveModel()
{
    QString filename = QFileDialog::getSaveFileName(0, tr("Choose file"), QString(), QLatin1String("*.off *.obj"));

    if ( filename == "" || (!(keyframeModel)) ) return;
    std::ofstream outfile(filename.toAscii());

	if (filename.endsWith("off"))
	{
		outfile << "OFF\n";
		outfile << keyframeModel->getNumVertices() << ' ' << keyframeModel->getNumFaces() << " 0\n"; // don't bother counting edges
		keyframeModel->saveVertices(outfile,filename);
		keyframeModel->saveFaces(outfile,filename);
	}
    
	if (filename.endsWith("obj"))
	{
		keyframeModel->saveVertices(outfile,filename);
		keyframeModel->saveTextureUVs(outfile,filename);
		keyframeModel->saveFaces(outfile,filename);
	}
}

/******************************************************************************************************************************/
void MainScene::chooseTexture()
{
    makeCurrent();
    QString filename = QFileDialog::getOpenFileName(0, tr("Choose image"), QString(), QLatin1String("*.png *.jpg *.bmp"));

    glEnable(GL_TEXTURE_2D);
    if (filename == NULL) {
		resetTexture();
	}
    else {
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
	keyframeModel->copyPositions(*model);
	keyframeModel->historyReset();
	repaint();
}
/******************************************************************************************************************************/
void MainScene::undoModel()
{
	keyframeModel->historyUndo();
	repaint();
}

/******************************************************************************************************************************/
void MainScene::redoModel()
{
	if ( !keyframeModel) return;
	keyframeModel->historyRedo();
	repaint();
}

/******************************************************************************************************************************/
void MainScene::saveLog()
{
    QString filename = QFileDialog::getSaveFileName(0, tr("Choose file"), QString(), QLatin1String("*.txt"));
    if (filename == "") return;

    std::ofstream outfile(filename.toAscii());
    keyframeModel->historySaveToFile(outfile);
}
/******************************************************************************************************************************/
void MainScene::runLog()
{
    QString filename = QFileDialog::getOpenFileName(0, tr("Choose log"), QString(), QLatin1String("*.txt"));
    if (filename == "") return;
    std::ifstream infile(filename.toAscii());

    printf("STARTING log replay\n");

    int numSteps;
    infile >> numSteps;

    for (int step = 0; step < numSteps; step++)
    {
    	keyframeModel->historyLoadFromFile(infile);
        update();
        repaint();
        usleep(500);
    }

    printf("DONE WITH RUN\n");
}

/******************************************************************************************************************************/
void MainScene::changeAlpha(int i) {
	if (keyframeModel)
		keyframeModel->setAlpha((double) (i) / 100 * 2);
}

/******************************************************************************************************************************/
void MainScene::drawModeChanged(bool m) {
	if (keyframeModel)
		keyframeModel->setDrawVFMode(m);

	update();
	repaint();
}

/******************************************************************************************************************************/
void MainScene::changeWireframe(int i) {
	if (keyframeModel)
		keyframeModel->setWireframeTrans((double) (i) / 100);

	wireframeTransparency = i;

	update();
	repaint();
}

/******************************************************************************************************************************/
void MainScene::clearPins()
{
	if (keyframeModel)
		keyframeModel->clearPins();

	update();
	repaint();
}

/******************************************************************************************************************************/
void MainScene::paintGL()
{
    glViewport(0,0,(GLint)width(), (GLint)height());

    glHint(GL_LINE_SMOOTH_HINT, GL_DONT_CARE);
    glHint(GL_POLYGON_SMOOTH_HINT, GL_DONT_CARE);

    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_POLYGON_SMOOTH);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glClearColor(1.,1.,1., 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (keyframeModel)
    {
        glEnable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, textureRef);

        /* TODO: move projection dummy settings from model, here */

    	/* render the model*/
        keyframeModel->render(modelLocation.x(),modelLocation.y(),modelWidth,width(),height());

        /* render the VF*/
        keyframeModel->renderVF();

        /* render selected vertices */
        glColor3f(1,0,0);
        for (int i = 0; i < selectedVertices.size(); i++)
        	keyframeModel->renderVertex(modelLocation.x(),modelLocation.y(),modelWidth,width(),height(),selectedVertices[i]);

        /* render pinned vertices */
        glColor3f(1,1,0);
        for (auto it = keyframeModel->getPinnedVertexes().begin(); it != keyframeModel->getPinnedVertexes().end(); it++)
        	keyframeModel->renderVertex(modelLocation.x(),modelLocation.y(),modelWidth,width(),height(),*it);
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
		std::vector<DisplacedVertex> disps;
		std::vector<Vector2D<double> > displacements;

		for (int i = 0; i < touchPoints.size(); i++)
		{
			QPointF displacement = touchPoints[i].pos()
					- touchPointLocations[touchPoints[i].id()];
			if (touchToVertex[touchPoints[i].id()] == -1)
				continue;

			disps.push_back(DisplacedVertex(touchToVertex[touchPoints[i].id()], screenToModelVec(displacement)));
			touchPointLocations[touchPoints[i].id()] = touchPoints[i].pos();
		}
		if (disps.size() > 0)
			keyframeModel->displaceMesh(disps);
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

    if (!keyframeModel) return;

	pos -= modelLocation;
	pos *= keyframeModel->getWidth() / modelWidth;
	pos += QPointF(keyframeModel->getMinX(),keyframeModel->getMinY());
	Qt::KeyboardModifiers mods =  QApplication::keyboardModifiers();


	if ((mods & Qt::ShiftModifier) || pinMode) {
		/* pin/unpin vertexes */
		 Vertex v = keyframeModel->getClosestVertex(Point2D<double>(pos.x(),pos.y()));
		if (v != -1)
			keyframeModel->togglePinVertex(v);

	} else {
		/* Select this vertex */
		selectedVertices.clear();
		Vertex v = keyframeModel->getClosestVertex(Point2D<double>(pos.x(),pos.y()));
		if (v != -1)
			selectedVertices.push_back(v);
	}

	event->accept();
}
/******************************************************************************************************************************/
void MainScene::mouseMoveEvent(QMouseEvent *event)
{
    if (!keyframeModel) return;
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
        diff *= keyframeModel->getWidth() / modelWidth;

        std::cout << "Mouse move delta:" << diff.rx() << "-" << diff.ry() << std::endl;

        if (diff.rx() == 0 && diff.ry() == 0)
        	return;

        std::vector<DisplacedVertex> disps;
        disps.push_back(DisplacedVertex(selectedVertex, Vector2D<double>(diff.x(),-diff.y())));
		keyframeModel->displaceMesh(disps);
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
	if (!keyframeModel)
		return -1;

	pos.setY(height() - pos.y() - 1);
	pos -= modelLocation;
	pos *= keyframeModel->getWidth() / modelWidth;
	pos += QPointF(keyframeModel->getMinX(), keyframeModel->getMinY());
	return keyframeModel->getClosestVertex(Point2D<double>(pos.x(), pos.y()));
}

/******************************************************************************************************************************/
Vector2D<double> MainScene::screenToModelVec(QPointF v)
{
	v *= keyframeModel->getWidth() / modelWidth;
	return Vector2D<double>(v.x(), -v.y());
}
/******************************************************************************************************************************/
