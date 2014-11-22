


#include <QDockWidget>
#include <QFileDialog>
#include <QtOpenGL>
#include <QString>
#include <fstream>

#include "MainWindow.h"
#include "MainScene.h"
#include "SidePanel.h"
#include "ThumbnailRenderer.h"
#include "AnimationPanel.h"
#include "Utils.h"
#include "VideoModel.h"

/*****************************************************************************************************/
MainWindow::MainWindow() : model(NULL), currentFrameModel(NULL)
{
	setupUi(this);

	/* setup GL view */
	mainScene = new MainScene(this);
	setCentralWidget(mainScene);

	/* setup side panel */
	sidePanel = new SidePanel(this);
	connect_(sidePanel->btnLoadModel, clicked(),this, loadModel());
	connect_(sidePanel->btnResetTexture, clicked(),this, resetTexture());
	connect_(sidePanel->btnLoadTexture, clicked(),this, chooseTexture());

	connect_(sidePanel->btnResetMesh, clicked(), mainScene, resetPoints());
	connect_(sidePanel->btnResetPins, clicked(),mainScene, clearPins());
	connect_(sidePanel->btnResetTransform, clicked(),mainScene, resetTransform());
	connect_(sidePanel->btnSaveLog, clicked(),mainScene,saveLog());
	connect_(sidePanel->btnReplayLog, clicked(),mainScene, runLog());
	connect_(sidePanel->btnUndo, clicked(),mainScene, undoModel());
	connect_(sidePanel->btnRedo, clicked(),mainScene, redoModel());

	connect_(sidePanel->chkPinMode, clicked(bool), mainScene, pinModeChanged(bool));
	connect_(sidePanel->chkVFMode, clicked(bool), mainScene, drawVFModeChanged(bool));
	connect_(sidePanel->chkVFOrigMode, clicked(bool), mainScene, drawOrigVFModeChanged(bool));
	connect_(sidePanel->btnApplyVF, clicked(), mainScene, reuseVF());

	connect_(sidePanel->sliderWireframeTransparency, valueChanged(int), mainScene, changeWireframe(int));
	connect_(sidePanel->sliderAlpha, valueChanged(int), mainScene, changeAlpha(int));
	sidePanel->sliderAlpha->setValue(20);
	addDockWidget(Qt::RightDockWidgetArea, sidePanel);

	/* setup animation panel */
	animationPanel = new AnimationPanel(this);
	animationPanel->setTitleBarWidget(new QWidget(this));
	addDockWidget(Qt::BottomDockWidgetArea, animationPanel);

	/* setup statusbar*/
	statusBar()->showMessage(tr("Ready"));
	lblVertexCount = new QLabel(this);
	lblVertexCount->setFrameStyle(QFrame::Panel | QFrame::Sunken);
	statusBar()->addPermanentWidget(lblVertexCount);
	lblFacesCount = new QLabel(this);
	lblFacesCount->setFrameStyle(QFrame::Panel | QFrame::Sunken);
	statusBar()->addPermanentWidget(lblFacesCount);
	lblFPS = new QLabel(this);
	lblFPS->setFrameStyle(QFrame::Panel | QFrame::Sunken);
	statusBar()->addPermanentWidget(lblFPS);

	thumbnailRender = new ThumbnailRenderer(NULL, mainScene);
	animationPanel->setThumbailRenderer(thumbnailRender);

	/* main scene will listen on model loads and keyframes switches*/
	connect_(this, videoModelLoaded(VideoModel*), mainScene, onVideoModelLoaded(VideoModel*));
	connect_(this, frameSwitched(MeshModel*), mainScene, onFrameSwitched(MeshModel*));
	connect_(this, textureChanged(GLuint), mainScene, onTextureChanged(GLuint));

	/* Animation panel will listen to 88.0FM....*/
	connect_(this, videoModelLoaded(VideoModel*), animationPanel, onVideoModelLoaded(VideoModel*));
	connect_(this, frameSwitched(MeshModel*), animationPanel, onFrameSwitched(MeshModel*));
	connect_(mainScene, modelEdited(KVFModel*), animationPanel, onFrameEdited(KVFModel*));
	connect_(this, textureChanged(GLuint), thumbnailRender, onTextureChanged(GLuint));
	connect_(this, textureChanged(GLuint), animationPanel, onTextureChanged(GLuint));

	/* we listen to main scene for edit events */
	connect_(mainScene, modelEdited(KVFModel*), this, onModelUpdate(KVFModel*));
	/* we listen on keyframe switches */
	connect_(this, frameSwitched(MeshModel*), this, onFrameSwitched(MeshModel*));
	connect_(animationPanel, frameSelectionChanged(MeshModel*), this, onEditBoxNewFrameSelected(MeshModel*));


	clearStatusBar();
}

/*****************************************************************************************************/
MainWindow::~MainWindow()
{
	delete model;
	delete thumbnailRender;
	delete mainScene;
	delete animationPanel;
	delete sidePanel;
}

/*****************************************************************************************************/
void MainWindow::setStatusBarStatistics(int vertexCount, int facesCount)
{
	QString str;
	str.sprintf("Vertexes: %d", vertexCount);
	lblVertexCount->setText(str);

	str.sprintf("Faces: %d", facesCount);
	lblFacesCount->setText(str);

	lblVertexCount->show();
	lblFacesCount->show();
}

/*****************************************************************************************************/
void MainWindow::setStatusBarFPS(int msec)
{
	QString str;
	str.sprintf("%d FPS", 1000/msec);
	lblFPS->setText(str);
	lblFPS->show();
}

/*****************************************************************************************************/
void MainWindow::clearStatusBar()
{
	lblFPS->hide();
	lblFacesCount->hide();
	lblVertexCount->hide();
}

/*****************************************************************************************************/
void MainWindow::loadModel()
{
    QString filename = QFileDialog::getOpenFileName(0, tr("Choose model"), QString(), QLatin1String("*.off *.obj"));
    if (filename == "") return;

    /* unload current model*/
    emit frameSwitched(NULL);
    emit videoModelLoaded(NULL);
    delete model;
    resetTexture();

    /* load new model */
    model = new VideoModel(filename.toStdString());
    emit videoModelLoaded(model);
    emit frameSwitched(model->keyframe(0));

    /* set these statistics - will only change when loading new model */
    setStatusBarStatistics(model->getNumVertices(), model->getNumFaces());
}

/*****************************************************************************************************/
void MainWindow::saveModel()
{
	if ( !currentFrameModel) return;

    QString filename = QFileDialog::getSaveFileName(0, tr("Choose file"), QString(), QLatin1String("*.off *.obj"));
    if ( filename == "")
    	return;

    std::ofstream outfile(filename.toAscii());
	if (filename.endsWith("off"))
	{
		outfile << "OFF\n";
		outfile << currentFrameModel->getNumVertices() << ' ' << currentFrameModel->getNumFaces() << " 0\n"; // don't bother counting edges
		currentFrameModel->saveVertices(outfile,filename);
		currentFrameModel->saveFaces(outfile,filename);
	}

	if (filename.endsWith("obj"))
	{
		currentFrameModel->saveVertices(outfile,filename);
		currentFrameModel->saveTextureUVs(outfile,filename);
		currentFrameModel->saveFaces(outfile,filename);
	}
}

/*****************************************************************************************************/
void MainWindow::chooseTexture()
{
    QString filename = QFileDialog::getOpenFileName(0, tr("Choose image"), QString(), QLatin1String("*.png *.jpg *.bmp"));
    if (filename == NULL)
		return;

	QPixmap pix(filename);
	textureRef = mainScene->bindTexture(pix,GL_TEXTURE_2D);
	emit textureChanged(textureRef);
}

/*****************************************************************************************************/
void MainWindow::resetTexture()
{
	QPixmap texture = QPixmap(16,16);
	texture.fill(QColor(200,200,255));
	textureRef = mainScene->bindTexture(texture);
	emit textureChanged(textureRef);
}

/*****************************************************************************************************/
void MainWindow::onModelUpdate(KVFModel* model)
{
	if (!model) return;

	/* called each time user edits model in the editor */
	if (model->lastVFCalcTime)
		setStatusBarFPS(model->lastVFCalcTime);
}

/*****************************************************************************************************/
void MainWindow::onEditBoxNewFrameSelected(MeshModel* model)
{
	emit frameSwitched(model);
}

/*****************************************************************************************************/
void MainWindow::onFrameSwitched(MeshModel* model)
{
	currentFrameModel = model;
}
