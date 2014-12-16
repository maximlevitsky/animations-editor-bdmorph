
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

/******************************************************************************************************************************/
EditorWindow::EditorWindow(QWidget* parent) :
			programstate(NULL),
			QGLWidget(QGLFormat(QGL::SampleBuffers), parent)
{
    setAttribute(Qt::WA_AcceptTouchEvents);
    setAttribute(Qt::WA_StaticContents);
    setFocusPolicy(Qt::WheelFocus);
    setMouseTracking(true);
    setContextMenuPolicy(Qt::ActionsContextMenu);
}
/******************************************************************************************************************************/
void EditorWindow::programStateUpdated(int flags, void *param)
{
	if (!programstate) return;
	MeshModel *currentModel = programstate->currentModel;
	bool need_repaint = false;

	if (flags & (ProgramState::CURRENT_MODEL_CHANGED))
	{
		programstate->selectedFace = -1;
		programstate->selectedFace = -1;
		need_repaint = true;
	}

	if (flags & (ProgramState::KEYFRAME_EDITED | ProgramState::ANIMATION_STEPPED)) {
		need_repaint = true;
	}

	if (flags & ProgramState::EDIT_SETTINGS_CHANGED)
	{
		setMouseTracking(programstate->showSelection);
		programstate->selectedFace = -1;
		programstate->selectedFace = -1;
		need_repaint = true;
	}

	if (flags & ProgramState::TEXTURE_CHANGED) {
		makeCurrent();
		glBindTexture(GL_TEXTURE_2D, programstate->textureRef);
		need_repaint = true;
	}

	if (flags & ProgramState::TRANSFORM_RESET) {
		if (currentModel)  {
			double maxZoomX = width() / currentModel->getWidth();
			double maxZoomY = height() / currentModel->getHeight();
			double maxZoom = std::min(maxZoomX,maxZoomY);
			modelWidth = currentModel->getWidth() * maxZoom * 0.5;
			modelLocation = QPointF(0, 0);
			need_repaint = true;
		}
	}

	if (need_repaint)
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

	if (!programstate) return;
	MeshModel *renderModel = programstate->currentModel;

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
	if (programstate->wireframeTransparency)
	{
		glColor4f(0,0,0,programstate->wireframeTransparency);
		renderModel->renderWireframe();
	}

	/* Some models will render stuff on top, like pined vertexes, extra wireframes, etc*/
	renderModel->renderOverlay(ratio);


	/* Render hovered vertices */
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	if (programstate->selectedFace != -1)
	{
		glColor4f(0,0,1,0.5);
		renderModel->renderFace(programstate->selectedFace);
	}

	if (programstate->selectedVertex != -1)
	{
		glColor3f(0,0,1);
		renderModel->renderVertex(programstate->selectedVertex, ratio);
	}

	/* render selected vertices */
	glColor3f(1,0,0);
	for (unsigned int i = 0; i < selectedVertices.size(); i++)
		renderModel->renderVertex(selectedVertices[i], ratio);


	/* render VF of KVF model */
	KVFModel* kvfModel = dynamic_cast<KVFModel*>(renderModel);
	if (kvfModel)
	{
		if (programstate->showVForig)
			kvfModel->renderVFOrig();
		if (programstate->showVF)
			kvfModel->renderVF();
	}

	BDMORPHModel* bdmodel = dynamic_cast<BDMORPHModel*>(renderModel);

	if (bdmodel) {
		if (programstate->showBDmorphEdge)
			bdmodel->renderInitialEdge(ratio);

		if (programstate->showBDmorphOrigMesh && bdmodel->modela ) {
			glColor4f(1,0,0,programstate->wireframeTransparency);
			bdmodel->modela->renderWireframe();
		}
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
	if (!programstate) return false;
	if (programstate->pinMode) return false;
	if (programstate->getCurrentMode() != ProgramState::PROGRAM_MODE_DEFORMATIONS) return false;

	KVFModel *kvfModel = dynamic_cast<KVFModel*>(programstate->currentModel);
    if (!kvfModel) return false;

	QList<QTouchEvent::TouchPoint> touchPoints = te->touchPoints();
	if (!programstate->multitouchMode && touchPoints.count() == 2)
	{
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
			touchToVertex[p.id()] = kvfModel->getClosestVertex(screenToModel(kvfModel,p.pos()));
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

			Vector2 displacement = screenToModel(kvfModel,touchPoints[i].pos()) -
					screenToModel(kvfModel,touchPointLocations[touchPoints[i].id()]);

			disps.insert(DisplacedVertex(touchToVertex[touchPoints[i].id()], displacement));
			touchPointLocations[touchPoints[i].id()] = touchPoints[i].pos();
		}
		if (disps.size() > 0)
		{
			kvfModel->calculateVF(disps);

			if (!programstate->showVF && !programstate->showVForig)
				kvfModel->applyVFLogSpiral();

			programstate->FPS = kvfModel->create_msec;
			programstate->onUpdateModel();
			programstate->updateStatistics();
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
	if (!programstate) return;
	KVFModel *kvfModel = dynamic_cast<KVFModel*>(programstate->currentModel);

    switch(e->key()) {
    case Qt::Key_Up:
    	moveUp();
    	repaint();
    	break;
    case Qt::Key_Down:
    	moveDown();
    	repaint();
    	break;
    case Qt::Key_Left:
    	moveLeft();
    	repaint();
    	break;
    case Qt::Key_Right:
    	moveRight();
    	repaint();
    	break;
    case Qt::Key_Plus:
    case Qt::Key_Equal:
    	zoomIn();
    	repaint();
    	break;
    case Qt::Key_Minus:
    case Qt::Key_Underscore:
    	zoomOut();
    	repaint();
    	break;
    case Qt::Key_V:
    	if (programstate->getCurrentMode() != ProgramState::PROGRAM_MODE_DEFORMATIONS) break;
    	if (kvfModel) kvfModel->applyVF();
    	programstate->onUpdateModel();
    	break;
    }
}

/******************************************************************************************************************************/
void EditorWindow::mousePressEvent(QMouseEvent *event)
{
	mouseMoved = false;
	mouseLeft = event->buttons() & Qt::LeftButton;
    QPointF pos = event->pos(); // (0,0) is upper left
    pos.setY(height()-pos.y()-1);
    lastMousePos = pos;

	if (!programstate) return;
	MeshModel *model = programstate->currentModel;
	if (!model) return;

    setCursor(Qt::BlankCursor);
    Point2 modelPos = screenToModel(model,pos);

	if ((QApplication::keyboardModifiers() == Qt::NoModifier) &&
			!programstate->pinMode && programstate->getCurrentMode() == ProgramState::PROGRAM_MODE_DEFORMATIONS)
	{
		Vertex v = model->getClosestVertex(modelPos);
		/* Select this vertex */
		selectedVertices.clear();
		selectedVertices.push_back(v);
		repaint();
		return;
	}

	if (event->buttons() & Qt::LeftButton)
		model->mousePressAction(modelPos,getRadius(model));

	repaint();
	return;
}
/******************************************************************************************************************************/
void EditorWindow::mouseMoveEvent(QMouseEvent *event)
{
	mouseLeft = event->buttons() & Qt::LeftButton;

	if (!programstate) return;
	MeshModel *model = programstate->currentModel;
	KVFModel *kvfModel = dynamic_cast<KVFModel*>(programstate->currentModel);
	if (!model) return;

    Qt::KeyboardModifiers mods =  QApplication::keyboardModifiers();

	QPointF oldPos = lastMousePos;
    QPointF curPos = event->pos();
    curPos.setY(height()-curPos.y()-1);

    lastMousePos = curPos;

    /* Plain mouse move */
    if (event->buttons() == Qt::NoButton && programstate->showSelection)
    {
		programstate->selectedVertex = model->getClosestVertex(screenToModel(model,curPos));
		programstate->selectedFace = model->getFaceUnderPoint(screenToModel(model,curPos));
		programstate->updateStatistics();
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
		bool deformMode =
				programstate->getCurrentMode() == ProgramState::PROGRAM_MODE_DEFORMATIONS
				&& kvfModel && kvfModel->getPinnedVertexes().size() &&
				!programstate->pinMode && !(mods & Qt::ShiftModifier);

		if (deformMode)
		{
			QPointF diff = curPos - oldPos;
			Vertex selectedVertex = selectedVertices[0];
			diff *= kvfModel->getWidth() / modelWidth;
			if (diff.rx() == 0 && diff.ry() == 0)
				return;

			std::cout << "Mouse move delta:(" << diff.rx() << "," << diff.ry() << ")" << std::endl;

			std::set<DisplacedVertex> disps;
			disps.insert(DisplacedVertex(selectedVertex, Vector2(diff.x(),diff.y())));
			kvfModel->calculateVF(disps);

			if (!programstate->showVF && !programstate->showVForig)
				kvfModel->applyVFLogSpiral();

			programstate->FPS = kvfModel->create_msec;
			programstate->onUpdateModel();
			programstate->updateStatistics();
			repaint();
			return;
		}

		if (!model->moveAction(screenToModel(model,oldPos), screenToModel(model,curPos), getRadius(model)))
		{
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

	if (!programstate) return;
	MeshModel *model = programstate->currentModel;
	if (!model) return;

	model->historySnapshot();
    QPointF curPos = event->pos();
    curPos.setY(height()-curPos.y()-1);
    model->mouseReleaseAction(screenToModel(model,curPos),mouseMoved, getRadius(model), !mouseLeft);
    update();
}

/******************************************************************************************************************************/
void EditorWindow::wheelEvent(QWheelEvent *event)
{
	if (!programstate) return;
	MeshModel *model = programstate->currentModel;
	if (!model) return;

	QPointF pos = event->pos();
	pos.setY(height()-pos.y()-1);

	Point2 posBefore = screenToModel(model,pos);
	zoom(event->delta() > 0 ? ZOOM_FACTOR : 1 / ZOOM_FACTOR);
	Point2 posAfter = screenToModel(model,pos);

	Vector2 diff = (posAfter-posBefore) * (modelWidth / model->getWidth());
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
Point2 EditorWindow::screenToModel(MeshModel *model,QPointF pos)
{
	assert(model);
	pos -= modelLocation;
	pos -= QPointF((double)width()/2,(double)height()/2);
	pos *= model->getWidth() / modelWidth;
	pos.rx() += model->center.x;
	pos.ry() += model->center.y;
	return Point2(pos.x(), pos.y());
}


double EditorWindow::getRadius(MeshModel *model)
{
	Point2 modelPos = screenToModel(model,QPointF(0,0));
	Point2 modelPos2 = screenToModel(model,QPointF(0,5));
	return modelPos.distance(modelPos2);
}
/******************************************************************************************************************************/
