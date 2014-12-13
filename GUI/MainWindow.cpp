


#include <QDockWidget>
#include <QFileDialog>
#include <QtOpenGL>
#include <QString>
#include <fstream>

#include "MainWindow.h"
#include "EditorWindow.h"
#include "SidePanel.h"
#include "ThumbnailRenderer.h"
#include "AnimationPanel.h"
#include "Utils.h"
#include "VideoModel.h"
#include "BDMORPH.h"
#include "OutlineModel.h"

/*****************************************************************************************************/
MainWindow::MainWindow() : videoModel(NULL), currentFrameModel(NULL), outlineModel(NULL)
{
	setupUi(this);

	/* setup GL view */
	editorWindow = new EditorWindow(this);
	setCentralWidget(editorWindow);

	/* setup side panel */
	sidePanel = new SidePanel(this);


	sidePanel->sliderAlpha->setValue(5);
	addDockWidget(Qt::RightDockWidgetArea, sidePanel);

	/* setup animation panel */
	animationPanel = new AnimationPanel(this);
	animationPanel->setTitleBarWidget(new QWidget(this));
	addDockWidget(Qt::BottomDockWidgetArea, animationPanel);

	/* setup statusbar*/
	statusBar()->showMessage(tr("Ready"));
	lblVertexCount = new QLabel(this);
	lblVertexCount->setFrameStyle(QFrame::Panel | QFrame::Sunken);

	lblFacesCount = new QLabel(this);
	lblFacesCount->setFrameStyle(QFrame::Panel | QFrame::Sunken);

	lblFPS = new QLabel(this);
	lblFPS->setFrameStyle(QFrame::Panel | QFrame::Sunken);

	lblSelectedFace = new QLabel(this);
	lblSelectedFace->setFrameStyle(QFrame::Panel | QFrame::Sunken);

	lblSelectedVertex = new QLabel(this);
	lblSelectedVertex->setFrameStyle(QFrame::Panel | QFrame::Sunken);

	statusBar()->addPermanentWidget(lblFacesCount);
	statusBar()->addPermanentWidget(lblVertexCount);
	statusBar()->addPermanentWidget(lblSelectedVertex);
	statusBar()->addPermanentWidget(lblSelectedFace);
	statusBar()->addPermanentWidget(lblFPS);

	thumbnailRender = new ThumbnailRenderer(NULL, editorWindow);
	animationPanel->setThumbailRenderer(thumbnailRender);

	/* --------------------------------------------------------------------------------*/
	/* editor window will listen on ECO90 FM */
	connect_(this, videoModelLoaded(VideoModel*), 			editorWindow, onVideoModelLoaded(VideoModel*));
	connect_(this, frameSwitched(MeshModel*), 				editorWindow, onFrameSwitched(MeshModel*));
	connect_(this, textureChanged(GLuint), 					editorWindow, onTextureChanged(GLuint));
	connect_(&animationPanel->animationThread, started(),
															editorWindow, onAnimationStarted());
	connect_(&animationPanel->animationThread, finished(),
															editorWindow, onAnimationStopped());
	connect_(sidePanel->btnResetMesh, clicked(), 			editorWindow, onResetPoints());
	connect_(sidePanel->btnResetPins, clicked(),			editorWindow, onClearPins());
	connect_(sidePanel->btnResetTransform, clicked(),		editorWindow, onResetTransform());
	connect_(sidePanel->btnUndo, clicked(),					editorWindow, onUndoModel());
	connect_(sidePanel->btnRedo, clicked(),					editorWindow, onRedoModel());
	connect_(sidePanel->chkPinMode, clicked(bool), 			editorWindow, onPinModeChanged(bool));
	connect_(sidePanel->chkShowSelection, clicked(bool), 	editorWindow, onShowSelectionChanged(bool));
	connect_(sidePanel->sliderWireframeTransparency, valueChanged(int),
															editorWindow, onChangeWireframe(int));
	connect_(sidePanel->sliderAlpha, valueChanged(int), 	editorWindow, onChangeAlpha(int));
	connect_(actionUndo, triggered(), 						editorWindow, onUndoModel());
	connect_(actionRedo, triggered(), 						editorWindow, onRedoModel());
	connect_(actionReset_model, triggered(), 				editorWindow, onResetPoints());
	connect_(actionReset_pins, triggered(), 				editorWindow, onClearPins());
	connect_(actionReplay_log, triggered(), 				editorWindow, onRunLog());
	connect_(actionSave_log, triggered(), 					editorWindow, onSaveLog());
	connect_(action_show_VF, toggled(bool), 				editorWindow, onDrawVFModeChanged(bool));
	connect_(action_show_orig_VF, toggled(bool), 			editorWindow, onDrawOrigVFModeChanged(bool));
	connect_(actionReapply_VF,triggered(), 					editorWindow, onReuseVF());
	/* --------------------------------------------------------------------------------*/
	/* Animation panel will listen to 88.0FM....*/
	connect_(this, videoModelLoaded(VideoModel*), 			animationPanel, onVideoModelLoaded(VideoModel*));
	connect_(this, frameSwitched(MeshModel*), 				animationPanel, onFrameSwitched(MeshModel*));
	connect_(this, textureChanged(GLuint), 					thumbnailRender, onTextureChanged(GLuint));
	connect_(this, textureChanged(GLuint), 					animationPanel, onTextureChanged(GLuint));

	connect_(editorWindow, modelEdited(MeshModel*), 		animationPanel, onFrameEdited(MeshModel*));
	connect_(actionNew_keyframe, triggered(), 				animationPanel, onCloneKeyFramePressed());
	connect_(actionDelete_keyframe, triggered(), 			animationPanel, onDeleteKeyframe());
	connect_(actionPlay, triggered(), 						animationPanel, onPlayPauseButtonPressed());
	connect_(actionRewind, triggered(), 					animationPanel, onBackwardButton());
	connect_(&animationPanel->animationThread, started(),   animationPanel, onAnimationStarted());
	connect_(&animationPanel->animationThread, finished(),  animationPanel, onAnimationStopped());

	/* --------------------------------------------------------------------------------*/
	/* Side panel will listen to Radius 100 FM*/
	connect_(this, frameSwitched(MeshModel*), 				sidePanel, onFrameSwitched(MeshModel*));
	connect_(&animationPanel->animationThread, started(),
															sidePanel, onAnimationStarted());
	connect_(&animationPanel->animationThread, finished(),
															sidePanel, onAnimationStopped());
	/* --------------------------------------------------------------------------------*/
	/* we listen to everyone  */
	connect_(editorWindow, selectionChanged(int,int), 		this, onEditorSelectionChanged(int,int));
	connect_(editorWindow, FPSUpdated(double), 				this, onFPSUpdated(double));
	connect_(animationPanel, FPSUpdated(double), 			this, onFPSUpdated(double));
	connect_(sidePanel, meshCreationRequest(int), 			this, onMeshCreationRequest(int));
	connect_(animationPanel, frameSwitched(MeshModel*),		this, onFrameSwitchListener(MeshModel*));
	connect_(sidePanel->btnLoadModel, clicked(),			this, onLoadModel());
	connect_(sidePanel->btnResetTexture, clicked(),			this, onResetTexture());
	connect_(sidePanel->btnLoadTexture, clicked(),			this, onChooseTexture());
	connect_(sidePanel->btnNewModel, clicked(), 			this, onCreateOutlineModel());
	connect_(sidePanel->btnSaveModel, clicked(), 			this, onSaveModel());
	connect_(sidePanel->btnUnloadModel, clicked(), 			this, onUnloadModel());
	connect_(actionNew_model, triggered(), 					this, onCreateOutlineModel());
	connect_(actionLoad_mesh, triggered(), 					this, onLoadModel());
	connect_(actionSave_model, triggered(), 				this, onSaveModel());
	connect_(actionUnload_model, triggered(), 				this, onUnloadModel());
	connect_(actionLoad_texture, triggered(), 				this, onChooseTexture());
	connect_(actionReset_texture, triggered(), 				this, onResetTexture());
	connect_(actionSave_screenshot, triggered(), 			this, onSaveScreenShot());
	connect_(actionSide_panel, toggled(bool), 				this, onSidePanelMenuShowHide(bool));
	connect_(actionAnimation_panel, toggled(bool), 			this, onAnimationPanelMenuShowHide(bool));
	connect_(actionAbout, triggered(),						this, onAbout());
	connect_(actionExit, triggered(), 						this, close());
	connect_(actionTest_animations, triggered(), 			this, onInterpolationTest());
	/* --------------------------------------------------------------------------------*/
	/* We listen to ourselves */
	connect_(this, frameSwitched(MeshModel*), 				this, onFrameSwitched(MeshModel*));
	connect_(this, videoModelLoaded(VideoModel*), 			this, onVideoModelLoaded(VideoModel*));

	/* --------------------------------------------------------------------------------*/
	animationPanel->btnRepeat->setAction(actionLoop);
	sidePanel->chkPinMode->setAction(actionPin_edit_mode);
	actionSide_panel->setChecked(true);
	actionAnimation_panel->setChecked(true);

	/* --------------------------------------------------------------------------------*/
	clearStatusBar();
	emit frameSwitched(NULL);
	emit videoModelLoaded(NULL);
	show();
}

/*****************************************************************************************************/
MainWindow::~MainWindow()
{
	delete videoModel;
	delete thumbnailRender;
	delete editorWindow;
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
void MainWindow::clearStatusBar()
{
	lblFPS->hide();
	lblFacesCount->hide();
	lblVertexCount->hide();
	lblSelectedFace->hide();
	lblSelectedVertex->hide();
}

/*****************************************************************************************************/
void MainWindow::onLoadModel()
{
    QString filename = QFileDialog::getOpenFileName(this, tr("Choose model"), QString(), QLatin1String("*.off *.obj *.poly"));
    if (filename == "") return;

    QFileInfo fileinfo(filename);

    if (!fileinfo.exists()) {
    	QMessageBox::warning(this, "Error","File doesn't exist");
    	return;
    }

    onUnloadModel();
    onResetTexture();

    if (filename.endsWith(".poly"))
    {
    	if (outlineModel)
    		delete outlineModel;
    	outlineModel = new OutlineModel();

    	bool result = outlineModel->loadFromFile(filename.toStdString());

    	if (!result) {
    		delete outlineModel;
    		outlineModel = NULL;
    		return;
    	}
		emit videoModelLoaded(NULL);
    	emit frameSwitched(outlineModel);
    } else
    {
		/* load new model */
		videoModel = new VideoModel();

		bool result = videoModel->loadFromFile(filename.toStdString());
		if (result == false) {
			QMessageBox::warning(this, "Error", "Can't load mesh");
			delete videoModel;
			videoModel = NULL;
			return;
		}

		result = videoModel->initialize();
		if (result == false)
		{
			QMessageBox::warning(this, "Error", "Problem initializing animations");
			delete videoModel;
			videoModel = NULL;
			return;
		}
		emit videoModelLoaded(videoModel);
		emit frameSwitched(videoModel->getKeyframeByIndex(1));
    }


    /* Try to load texture */
    int lastPoint = filename.lastIndexOf(".");
    QString fileNameNoExt = filename.left(lastPoint);

    fileinfo.setFile(fileNameNoExt + ".png");
    if (fileinfo.exists())
    	setTexture(fileNameNoExt + ".png");
}

/*****************************************************************************************************/

void MainWindow::onCreateOutlineModel()
{

    QString filename = QFileDialog::getOpenFileName(this, tr("Choose image for new model"), QString(), QLatin1String("*.png *.jpg *.bmp"));
    if (filename == NULL)
		return;

	onUnloadModel();

	if (outlineModel)
		delete outlineModel;
	outlineModel = new OutlineModel();

	emit frameSwitched(outlineModel);
    setTexture(filename);
}

/*****************************************************************************************************/
void MainWindow::onSaveModel()
{
	if ( !currentFrameModel) return;
    QString filename = QFileDialog::getSaveFileName(this, tr("Choose file"), QString(), QLatin1String("*.off *.obj"));
    if ( filename == "") return;
    currentFrameModel->saveToFile(filename.toStdString());
}

void MainWindow::onUnloadModel()
{
    /* unload current model*/
    emit frameSwitched(NULL);
    emit videoModelLoaded(NULL);
    delete videoModel;
    videoModel = NULL;
    clearStatusBar();
    emit frameSwitched(outlineModel);
}

/*****************************************************************************************************/
void MainWindow::onChooseTexture()
{
    QString filename = QFileDialog::getOpenFileName(this, tr("Choose image"), QString(), QLatin1String("*.png *.jpg *.bmp"));
    if (filename == NULL)
		return;

    setTexture(filename);
}

/*****************************************************************************************************/
void MainWindow::onEditorSelectionChanged(int selectedVertex, int selectedFace)
{
	QString str;

	if (selectedVertex != -1) {
		lblSelectedVertex->show();
		str.sprintf("Vertex: %d", selectedVertex);
		lblSelectedVertex->setText(str);
	} else
		lblSelectedVertex->hide();

	if (selectedFace != -1) {
		lblSelectedFace->show();
		str.sprintf("Face: %d", selectedFace);
		lblSelectedFace->setText(str);
	} else
		lblSelectedFace->hide();

}
/*****************************************************************************************************/
void MainWindow::onResetTexture()
{
	QPixmap texture = QPixmap(16,16);
	texture.fill(QColor(200,200,255));
	textureRef = editorWindow->bindTexture(texture);
	emit textureChanged(textureRef);
}

/*****************************************************************************************************/
void MainWindow::onFPSUpdated(double msec)
{
	QString str;

	if (msec >0 ) {
		str.sprintf("%4.2f FPS", 1000.0/msec);
		lblFPS->setText(str);
		lblFPS->show();
	} else {
		lblFPS->setText("FAILURE");
	}
}

/*****************************************************************************************************/
void MainWindow::onFrameSwitchListener(MeshModel* model)
{
	emit frameSwitched(model);
}

/*****************************************************************************************************/
void MainWindow::onFrameSwitched(MeshModel* model)
{
	currentFrameModel = model;

	bool isKVFModel = dynamic_cast<KVFModel*>(model) != NULL;
	bool isBDMORPH = dynamic_cast<BDMORPHModel*>(model) != NULL;
	//bool isOutlineModel = dynamic_cast<OutlineModel*>(model) != NULL;
	actionAnimation_panel->setVisible(isKVFModel||isBDMORPH);
}

void MainWindow::onVideoModelLoaded(VideoModel* model)
{
	animationPanel->setVisible(model != NULL);
}
/*****************************************************************************************************/
void MainWindow::onSidePanelMenuShowHide(bool checked)
{
	if (checked)
	{
		sidePanel->show();
	} else
		sidePanel->hide();
}

/*****************************************************************************************************/
void MainWindow::onAnimationPanelMenuShowHide(bool checked)
{
	if (checked)
		animationPanel->show();
	else
		animationPanel->hide();
}
/*****************************************************************************************************/
void MainWindow::onSaveScreenShot()
{
	/*TODO */
}
void MainWindow::onSaveVideo()
{
	/*TODO */
}


void MainWindow::onAbout()
{
	/* TODO*/
}

/*****************************************************************************************************/

void MainWindow::onInterpolationTest()
{
	if (!videoModel || !currentFrameModel)
		return;

	videoModel->pFrame->interpolate_frame(videoModel,currentFrameModel,0.5);
	emit frameSwitched(videoModel->pFrame);
}

/*****************************************************************************************************/

void MainWindow::setTexture(QString filename)
{
	if (!currentFrameModel)
		return;

	texture.load(filename);

	if (texture.height() !=texture.width()  && !currentFrameModel->hasTextureMapping())
	{
		int size = std::max(texture.height(),texture.width());
		QPixmap scaledTexture(size,size);
		QPainter painter(&scaledTexture);

		int x = (size - texture.width())/2;
		int y = (size - texture.height())/2;

		painter.eraseRect(0,0,size,size);
		painter.drawPixmap(x,y,texture);

		textureRef = editorWindow->bindTexture(scaledTexture,GL_TEXTURE_2D);
	} else
		textureRef = editorWindow->bindTexture(texture,GL_TEXTURE_2D);

	emit textureChanged(textureRef);
}

/*****************************************************************************************************/

void MainWindow::onMeshCreationRequest(int requestedDensity)
{
	if (videoModel || !outlineModel)
		return;

	videoModel = new VideoModel();
	bool result = outlineModel->createMesh(videoModel, requestedDensity);

	if (result == false) {
    	QMessageBox::warning(this, "Error","Problem on creating the mesh, check if outline is valid");
		delete videoModel;
		videoModel = NULL;
		return;
	}

    result = videoModel->initialize();
    if (result == false)
    {
    	QMessageBox::warning(this, "Error", "Problem initializing animations. Mesh is not connected probably");
    	delete videoModel;
    	videoModel = NULL;
    	return;
    }

    emit videoModelLoaded(videoModel);
    emit frameSwitched(videoModel->getKeyframeByIndex(1));
    setStatusBarStatistics(videoModel->getNumVertices(), videoModel->getNumFaces());
}
