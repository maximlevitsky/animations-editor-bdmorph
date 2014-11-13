#include "deformationscene.h"

#define TANGENT_WIDTH 20
#include <QtGui>
#include <QtOpenGL>
#include <QPalette>
#include <iostream>
#include <fstream>
#include <QGesture>
#include <ctime>
using namespace std;
#include <stdio.h>

/******************************************************************************************************************************/
DeformationScene::DeformationScene(QGLWidget *w) :
			model(NULL), origModel(NULL), selectedIndex(-1), alpha(.5), brush(10), wireframe(0), glWidget(w)
{
    glWidget->setAttribute(Qt::WA_AcceptTouchEvents);
    glWidget->setAttribute(Qt::WA_StaticContents);

    glWidget->makeCurrent();
    glEnable(GL_TEXTURE_2D);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

    QPixmap texture = QPixmap(16,16);
    texture.fill(QColor(200,200,255));

    textureRef = glWidget->bindTexture(texture);

	QPalette plt;
    plt.setColor(QPalette::WindowText, Qt::white);
    QWidget *deformBox = createDialog(tr("Deformation Tools"));
	undoButton = new QPushButton(tr("Undo"));
	connect(undoButton, SIGNAL(clicked()), this, SLOT(undoModel()));
	redoButton = new QPushButton(tr("Redo"));
	connect(redoButton, SIGNAL(clicked()), this, SLOT(redoModel()));
	pinMode = new QCheckBox("Pin mode");
	pinMode->setPalette(plt);
	connect(pinMode, SIGNAL(toggled(bool)), this, SLOT(setCheckState(bool)));
	multitouchMode = new QCheckBox("Multitouch mode");
	multitouchMode->setPalette(plt);
	connect(multitouchMode, SIGNAL(toggled(bool)), this, SLOT(setCheckState(bool)));
	brushSlider = new QSlider();
    brushSlider->setOrientation(Qt::Horizontal);
    brushSlider->setRange(1, 100);
    brushSlider->setValue((int)(brush));
    connect(brushSlider, SIGNAL(valueChanged(int)), this, SLOT(changeBrush(int)));
    alphaSlider = new QSlider();
    alphaSlider->setOrientation(Qt::Horizontal);
    alphaSlider->setRange(0, 100);
    alphaSlider->setValue((int)(alpha / 2 * 100));
    connect(alphaSlider, SIGNAL(valueChanged(int)), this, SLOT(changeAlpha(int)));
	clearButton = new QPushButton(tr("Clear pins"));
	connect(clearButton, SIGNAL(clicked()), this, SLOT(clearPins()));
    resetButton = new QPushButton(tr("Reset points"));
	connect(resetButton, SIGNAL(clicked()), this, SLOT(restorePoints()));
	deformBox->layout()->addWidget(undoButton);
	deformBox->layout()->addWidget(redoButton);
	deformBox->layout()->addWidget(pinMode);
	deformBox->layout()->addWidget(multitouchMode);
	deformBox->layout()->addWidget(new QLabel("<font color=white>Brush (1 to 100):</font>"));
    deformBox->layout()->addWidget(brushSlider);
    deformBox->resize(100,50);
    deformBox->layout()->addWidget(new QLabel("<font color=white>Alpha (0 to 2):</font>"));
    deformBox->layout()->addWidget(alphaSlider);
    deformBox->resize(100,50);
	deformBox->layout()->addWidget(clearButton);
	deformBox->layout()->addWidget(resetButton);
    addWidget(deformBox);

	QWidget *effectBox = createDialog(tr("Wireframe"));
	drawVectorField = new QCheckBox("Draw VF");
	drawVectorField->setPalette(plt);
    wireframeSlider = new QSlider();
    wireframeSlider->setOrientation(Qt::Horizontal);
    wireframeSlider->setRange(0, 100);
    wireframeSlider->setValue(0);
	connect(drawVectorField, SIGNAL(toggled(bool)), this, SLOT(drawModeChanged(bool)));
	connect(wireframeSlider, SIGNAL(valueChanged(int)), this, SLOT(changeWireframe(int)));
	meshLabel = new QLabel("<font color=yellow>#Vertices: -<br>#Faces: -</font>");
	effectBox->layout()->addWidget(new QLabel("<font color=white>Wireframe (0 to 100):</font>"));
	effectBox->layout()->addWidget(wireframeSlider);
	effectBox->resize(100,50);
	effectBox->layout()->addWidget(meshLabel);
	effectBox->layout()->addWidget(drawVectorField);
	addWidget(effectBox);

    QWidget *controls = createDialog(tr("Controls"));
	chooseTextureButton = new QPushButton(tr("Load texture"));
    removeTextureButton = new QPushButton(tr("Remove texture"));
	imageButton = new QPushButton(tr("Load image"));
    modelButton = new QPushButton(tr("Load model"));
    saveButton = new QPushButton(tr("Save model"));
    loadGeometry = new QPushButton(tr("Load geometry"));
	connect(chooseTextureButton, SIGNAL(clicked()), this, SLOT(chooseTexture()));
	connect(removeTextureButton, SIGNAL(clicked()), this, SLOT(removeTexture()));
	connect(imageButton, SIGNAL(clicked()), this, SLOT(loadImage()));
	connect(modelButton, SIGNAL(clicked()), this, SLOT(loadModel()));
    connect(saveButton, SIGNAL(clicked()), this, SLOT(saveModel()));
    connect(loadGeometry, SIGNAL(clicked()), this, SLOT(resetPoints()));
	controls->layout()->addWidget(modelButton);
	controls->layout()->addWidget(saveButton);
	controls->layout()->addWidget(imageButton);
    controls->layout()->addWidget(chooseTextureButton);
    controls->layout()->addWidget(removeTextureButton);
    controls->layout()->addWidget(loadGeometry);
    addWidget(controls);

    QPointF pos(10, 10);
    foreach (QGraphicsItem *item, items()) {
        item->setFlag(QGraphicsItem::ItemIsMovable);
        item->setCacheMode(QGraphicsItem::DeviceCoordinateCache);

        const QRectF rect = item->boundingRect();
        item->setPos(pos.x() - rect.x(), pos.y() - rect.y());
        pos += QPointF(0, 10 + rect.height());
    }
}

/******************************************************************************************************************************/

QDialog *DeformationScene::createDialog(const QString &windowTitle) const
{
    QDialog *dialog = new QDialog(0, Qt::CustomizeWindowHint | Qt::WindowTitleHint);

    dialog->setStyleSheet("QDialog { background-color: black; }");
    dialog->setWindowOpacity(0.8);
    dialog->setWindowTitle(windowTitle);
    dialog->setLayout(new QVBoxLayout);

    return dialog;
}

/******************************************************************************************************************************/

void DeformationScene::undoModel()
{
	if ( !model) return;
	model->undoDeform(logIndices,logDisplacements,logAlphas);
	glWidget->repaint();
}


void DeformationScene::redoModel()
{
	if ( !model) return;
	model->redoDeform(logIndices,logDisplacements,logAlphas);
	glWidget->repaint();
}

/******************************************************************************************************************************/

void DeformationScene::loadImage()
{
	QString filename = QFileDialog::getOpenFileName(0, tr("Choose model"), QString(), QLatin1String("*.png *.jpg *.bmp"));
	QImage image(filename);
	if (image.isNull()) return;

	image = image.scaled(350,350,Qt::KeepAspectRatio);
	image = image.mirrored(false,true);
	qWarning("width: %d, height: %d", image.width(), image.height()); 

	vector<QPointF> V; //vertices
	vector<pair<int,int> > E; //segments
	int Vmap[354][354] = {{0}}; //added stroke 2 in each side of the map
	int stroke = 2;
	for (int i=stroke; i<image.width()+stroke; i++) {
		for (int j=stroke; j<image.height()+stroke; j++) {
			if ((filename.endsWith("png") && qAlpha(image.pixel(i-stroke,j-stroke)) >= 250) ||
				((filename.endsWith("jpg") || filename.endsWith("bmp")) && qGray(image.pixel(i-stroke,j-stroke)) <= 250)) {
				for (int k=i-stroke; k<=i+stroke; k++) {
					for (int l=j-stroke; l<=j+stroke; l++) {
						Vmap[k][l] = -1;
					}
				}
			}
		}
	}

	int count = 0;
	for (int i=0; i<image.width()+2*stroke; i++) {
		for (int j=0; j<image.height()+2*stroke; j++) {
			if (Vmap[i][j] == -1) {
				if (Vmap[max(0,i-1)][j] == 0 || Vmap[i][max(0,j-1)] == 0 || 
					Vmap[min(image.width()+2*stroke-1,i+1)][j] == 0 || Vmap[i][min(image.height()+2*stroke-1,j+1)] == 0 ||
					Vmap[max(0,i-1)][max(0,j-1)] == 0 || Vmap[max(0,i-1)][min(image.height()+2*stroke-1,j+1)] == 0 ||
					Vmap[min(image.width()+2*stroke-1,i+1)][max(0,j-1)] == 0 || Vmap[min(image.width()+2*stroke-1,i+1)][min(image.height()+2*stroke-1,j+1)] == 0) {
						count++;
						Vmap[i][j] = count;
						V.push_back(QPointF(i,j));
				}
			}
		}
	}

	for (int i=0; i<image.width()+2*stroke; i++) {
		for (int j=0; j<image.height()+2*stroke; j++) {
			if (j+1 < image.height()+2*stroke && Vmap[i][j] > 0 && Vmap[i][j+1] > 0) {
				E.push_back(make_pair(Vmap[i][j],Vmap[i][j+1]));
			}
			if (i+1 < image.width()+2*stroke && Vmap[i][j] > 0 && Vmap[i+1][j] > 0) {
				E.push_back(make_pair(Vmap[i][j],Vmap[i+1][j]));
			}
		}
	}

	qWarning("\n#vertices: %d, #segments: %d", (int)V.size(), (int)E.size());

	ofstream outfile("temp.poly");
	//vertices
	outfile << V.size() << " 2 0 1" << endl;
	for (int i=1; i<=V.size(); i++) {
		outfile << i << ' ' << V[i-1].x()/(image.width()+stroke) << ' ' << V[i-1].y()/(image.height()+stroke) << " 1 " << endl;
	}
	//segments
	outfile << E.size() << " 1" << endl;
	for (int i=1; i<=E.size(); i++) {
		outfile << i << ' ' << E[i-1].first << ' ' << E[i-1].second << " 1 " << endl;
	}
	//holes
	outfile << '0' << endl;

	system("cc -O -o triangle triangle.c -lm");
	system("triangle -pqDgPNE temp");

}


void DeformationScene::saveModel()
{
    QString filename = QFileDialog::getSaveFileName(0, tr("Choose file"), QString(), QLatin1String("*.off *.obj"));

    if ( filename == "" || (!(model)) ) return;
	ofstream outfile(filename.toAscii());

	if (filename.endsWith("off"))
	{
		outfile << "OFF\n";
		outfile << model->getNumVertices() << ' ' << model->getNumFaces() << " 0\n"; // don't bother counting edges
		model->saveVertices(outfile,filename);
		model->saveFaces(outfile,filename);
	}
    
	if (filename.endsWith("obj"))
	{
		model->saveVertices(outfile,filename);
		origModel->saveTextureUVs(outfile,filename);
		model->saveFaces(outfile,filename);
	}
}

void DeformationScene::loadModel()
{
    pinned.clear();
    QString filename = QFileDialog::getOpenFileName(0, tr("Choose model"), QString(), QLatin1String("*.off *.obj"));

    if (filename == "") return;

    qWarning("Deleting");
    if (model) delete model;
    if (origModel) delete origModel;
	
    qWarning("Making a new model");
    model = new Model2D(filename);
	model->setWireframeTrans((double)wireframe/100);
//	model->setWireframeTrans(wireframeSlider->value());
	model->changeDrawMode(drawVectorField->isChecked());
	QPixmap texture = QPixmap(16,16);
    texture.fill(QColor(200,200,255));
    textureRef = glWidget->bindTexture(texture);
    origModel = new Model2D(filename);
    qWarning("Done loading");

	modelWidth = model->getWidth() * 200;
	modelLocation = QPointF(width()/2, height()/2);

    QString verticesNum = QString::number(model->getNumVertices());
	QString facesNum = QString::number(model->getNumFaces());
	QString meshText = "<font color=yellow>#Vertices: " + verticesNum;
	meshText += "<br>#Faces: ";
	meshText += facesNum;
	meshText += "</font>";
	meshLabel->setText(meshText); 

    glWidget->repaint();
}


/******************************************************************************************************************************/

void DeformationScene::resetPoints()
{
    QString filename = QFileDialog::getOpenFileName(0, tr("Choose model"), QString(), QLatin1String("*.off *.obj"));

    if (filename == "") return;

    if (model) model->replacePoints(filename);

	modelWidth = model->getWidth() * 200;
	modelLocation = QPointF(width()/2, height()/2);
    glWidget->repaint();
}



void DeformationScene::saveLog()
{
    QString filename = QFileDialog::getSaveFileName(0, tr("Choose file"), QString(), QLatin1String("*.txt"));

    if (filename == "") return;

    ofstream outfile(filename.toAscii());

    outfile << logIndices.size() << endl;
    for (unsigned int i = 0; i < logIndices.size(); i++) {
        outfile << logAlphas[i] << endl;
        outfile << logDisplacements[i].size() << endl;
        for (unsigned int j = 0; j < logDisplacements[i].size(); j++)
            outfile << logIndices[i][j] << ' ' << logDisplacements[i][j][0] << ' ' << logDisplacements[i][j][1] << endl;
    }
}

void DeformationScene::runLog()
{
    QString filename = QFileDialog::getOpenFileName(0, tr("Choose log"), QString(), QLatin1String("*.txt"));
    if (filename == "") return;
    ifstream infile(filename.toAscii());

    qWarning("STARTING RUN");

    int numSteps;
    infile >> numSteps;

    for (int i = 0; i < numSteps; i++) {
        double alpha;
        infile >> alpha;

        int numDisplacements;
        infile >> numDisplacements;

        vector< Vector2D<double> > displacements(numDisplacements);
        vector<int> indices(numDisplacements);

        for (int j = 0; j < numDisplacements; j++)
            infile >> indices[j] >> displacements[j][0] >> displacements[j][1];

        model->displaceMesh(indices,displacements,alpha);
        update();
        glWidget->repaint();

        qWarning("ALPHA: %g", alpha);
        for (int j = 0; j < displacements.size(); j++)
            qWarning("%d: %g %g", indices[j], displacements[j][0], displacements[j][1]);
    }

    qWarning("DONE WITH RUN");
}

/******************************************************************************************************************************/


void DeformationScene::chooseTexture()
{
    glWidget->makeCurrent();
    QString filename = QFileDialog::getOpenFileName(0, tr("Choose image"), QString(), QLatin1String("*.png *.jpg *.bmp"));
    glEnable(GL_TEXTURE_2D);
	if (filename == NULL) {
		QPixmap texture = QPixmap(16,16);
		texture.fill(QColor(200,200,255));
		textureRef = glWidget->bindTexture(texture);
	}
    else {
		textureRef = glWidget->bindTexture(QPixmap(filename),GL_TEXTURE_2D);
	}
    glWidget->repaint();
}

void DeformationScene::removeTexture()
{
    glWidget->makeCurrent();
	glEnable(GL_TEXTURE_2D);
	QPixmap texture = QPixmap(16,16);
    texture.fill(QColor(200,200,255));
    textureRef = glWidget->bindTexture(texture);
   // glEnable(GL_TEXTURE_2D);
    glWidget->repaint();
}

/******************************************************************************************************************************/

void DeformationScene::modeChanged(bool m)
{
    if (!origModel) return;

    if (!m) { // go back to deformation mode
        // restore original positions
        model->copyPositions(*origModel);
    } else
    {
        // restore original positions
        model->copyPositions(*origModel);
        constraintVerts.resize(1); // only one frame
        constraintVerts[0].resize(0);
    }
}


void DeformationScene::keyReleaseEvent(QKeyEvent *e)
{
    switch(e->key()) {
    default:
    	QGraphicsScene::keyPressEvent(e);
    	break;
    }
}

void DeformationScene::keyPressEvent(QKeyEvent *e)
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
    case Qt::Key_D:
    	model->reuseVF();
    	break;
    case Qt::Key_L:
        logDisplacements.resize(0);
        logIndices.resize(0);
        logAlphas.resize(0);
        break;
    case Qt::Key_R:
    	runLog();
    	break;
    case Qt::Key_S:
    	saveLog();
    	break;
    default:
    	QGraphicsScene::keyPressEvent(e);
    }

    update();
}

/******************************************************************************************************************************/

void DeformationScene::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    glWidget->setCursor(Qt::BlankCursor);
    QGraphicsScene::mousePressEvent(event);

    if (event->isAccepted()) return;

    QPointF pos = event->scenePos(); // (0,0) is upper left
    pos.setY(height()-pos.y()-1);

    QPointF origPos = pos;

    if (model)
    {
        pos -= modelLocation;
        pos *= model->getWidth() / modelWidth;
        pos += QPointF(model->getMinX(),model->getMinY());

        Qt::KeyboardModifiers mods =  QApplication::keyboardModifiers();

        if ((mods & Qt::ShiftModifier) || pinMode->isChecked())
        {
            int i = model->getClosestVertex(Point2D<double>(pos.x(),pos.y()),brush*model->getWidth()/modelWidth);

            if (i == -1)
            	return;

            if (pinned.count(i))
            	pinned.erase(i);
            else
            	pinned.insert(i);
        } else
        {
            int vertex = model->getClosestVertex(Point2D<double>(pos.x(),pos.y()),brush*model->getWidth()/modelWidth);
            oldVertices.resize(0);
            oldVertices.push_back(vertex);

			if (!multitouchMode->isChecked())
				selectedIndex = vertex;
        }
    }

    event->accept();
}

void DeformationScene::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
    QGraphicsScene::mouseMoveEvent(event);

    if (!model) return;
    if (event->isAccepted()) return;

    Qt::KeyboardModifiers mods =  QApplication::keyboardModifiers();

	if ((event->buttons() & Qt::RightButton) && selectedIndex >= 0)
	{
		mousePos = event->scenePos();
        mousePos.setY(height()-mousePos.y()-1);
        mousePos -= modelLocation;
        mousePos *= model->getWidth() / modelWidth;
        mousePos += QPointF(model->getMinX(),model->getMinY());

		QPointF oldPos = event->lastScenePos();
        QPointF curPos = event->scenePos();
        QPointF diff = curPos - oldPos;
		move(QPointF(diff.x(),-diff.y()));
	} 

    if ((event->buttons() & Qt::LeftButton) && (!(mods & Qt::ShiftModifier)) && !pinMode->isChecked() && selectedIndex >= 0)
    {
        mousePos = event->scenePos();
        mousePos.setY(height()-mousePos.y()-1);
        mousePos -= modelLocation;
        mousePos *= model->getWidth() / modelWidth;
        mousePos += QPointF(model->getMinX(),model->getMinY());

        QPointF oldPos = event->lastScenePos();
        QPointF curPos = event->scenePos();
        QPointF diff = curPos - oldPos;

        diff *= model->getWidth() / modelWidth;

        std::cout << "Mouse move delta:" << diff.rx() << "-" << diff.ry() << std::endl;

        if (diff.rx() == 0 && diff.ry() == 0)
        	return;


        vector<int> indices;
		vector< Vector2D<double> > displacements;
		indices.push_back(selectedIndex);
		displacements.push_back(Vector2D<double>(diff.x(),-diff.y()));

		for (set<int>::iterator it = pinned.begin(); it != pinned.end(); ++it)
		{
			indices.push_back(*it);
			displacements.push_back(Vector2D<double>(0,0));
		}

		if (!multitouchMode->isChecked())
		{
			model->displaceMesh(indices,displacements,alpha);
			logDisplacements.push_back(displacements);
			logIndices.push_back(indices);
			logAlphas.push_back(alpha);

			model->addUndoAction(indices,displacements,alpha);
		}

        event->accept();
        update();
    }

    glWidget->repaint();
}


void DeformationScene::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
    glWidget->setCursor(Qt::ArrowCursor);

    QGraphicsScene::mouseReleaseEvent(event);
    if (event->isAccepted()) return;
    selectedIndex = -1;
    update();
}


void DeformationScene::wheelEvent(QGraphicsSceneWheelEvent *event)
{
	QGraphicsScene::wheelEvent(event);
	if (event->isAccepted()) return;
	zoom(event->delta() > 0 ? ZOOM_FACTOR : 1 / ZOOM_FACTOR);
}

/******************************************************************************************************************************/


void DeformationScene::drawBackground(QPainter *painter, const QRectF &)
{
	glWidget->makeCurrent();

    //glHint(GL_POINT_SMOOTH_HINT, GL_DONT_CARE);
    glHint(GL_LINE_SMOOTH_HINT, GL_DONT_CARE);
    glHint(GL_POLYGON_SMOOTH_HINT, GL_DONT_CARE);

    glEnable(GL_LINE_SMOOTH);
    //glEnable(GL_POINT_SMOOTH);
    glEnable(GL_POLYGON_SMOOTH);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glClearColor(1.,1.,1., 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (model)
    {
        glEnable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, textureRef);
        model->render(modelLocation.x(),modelLocation.y(),modelWidth,width(),height());
        glColor3f(1,0,0);

        if (selectedIndex != -1)
                model->renderSelectedVertex(modelLocation.x(),modelLocation.y(),modelWidth,width(),height(),selectedIndex);

        for (int i = 0; i < pointsToRender.size(); i++)
			model->renderSelectedVertex(modelLocation.x(),modelLocation.y(),modelWidth,width(),height(),pointsToRender[i]);


        glColor3f(1,1,0); //highlighted anchors
        for (set<int>::iterator it = pinned.begin(); it != pinned.end(); ++it)
            model->renderSelectedVertex(modelLocation.x(),modelLocation.y(),modelWidth,width(),height(),*it);

        if (drawVectorField->isChecked())
            for (unsigned int i = 0; i < oldVertices.size(); i++)
                model->renderSelectedVertex(modelLocation.x(),modelLocation.y(),modelWidth,width(),height(),oldVertices[i]);
    }
}

/******************************************************************************************************************************/


int DeformationScene::closestIndex(QPointF pos)
{
	if (!model)
		return -1;

	pos.setY(height() - pos.y() - 1);
	pos -= modelLocation;
	pos *= model->getWidth() / modelWidth;
	pos += QPointF(model->getMinX(), model->getMinY());
	// large radius because my fingers are "fleshy" according to my piano prof
	return model->getClosestVertex(Point2D<double>(pos.x(), pos.y()), 5 * model->getWidth() / modelWidth);
}

Vector2D<double> DeformationScene::screenToModelVec(QPointF v)
{
	v *= model->getWidth() / modelWidth;
	return Vector2D<double>(v.x(), -v.y());
}

void DeformationScene::displaceMesh(vector<int> indices,vector<Vector2D<double> > displacements)
{
	for (set<int>::iterator it = pinned.begin(); it != pinned.end(); ++it)
	{
		indices.push_back(*it);
		displacements.push_back(Vector2D<double>(0, 0));
	}

	if (multitouchMode->isChecked()) {
		model->displaceMesh(indices, displacements, alpha);
		logDisplacements.push_back(displacements);
		logIndices.push_back(indices);
		logAlphas.push_back(alpha);
		model->addUndoAction(indices, displacements, alpha);
	}
}

void DeformationScene::restorePoints()
{
	model->copyPositions(*origModel);
}
