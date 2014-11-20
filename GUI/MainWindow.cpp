
#include "MainWindow.h"
#include "MainScene.h"
#include "SidePanel.h"
#include "AnimationPanel.h"
#include <QDockWidget>
#include "Utils.h"

/*****************************************************************************************************/
MainWindow::MainWindow()
{
	setupUi(this);
	mainScene = new MainScene(this);
	setCentralWidget(mainScene);


	/* create side panel and connect it to us */
	sidePanel = new SidePanel(this);
	connect_(sidePanel->btnLoadModel, clicked(),mainScene, loadModel());
	connect_(sidePanel->btnLoadTexture, clicked(),mainScene, chooseTexture());
	connect_(sidePanel->btnResetMesh, clicked(),mainScene, resetPoints());
	connect_(sidePanel->btnResetTexture, clicked(),mainScene, resetTexture());

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


	animationPanel = new AnimationPanel(this);
	animationPanel->setTitleBarWidget(new QWidget(this));
	addDockWidget(Qt::BottomDockWidgetArea, animationPanel);
	addDockWidget(Qt::RightDockWidgetArea, sidePanel);

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

}

/*****************************************************************************************************/
void MainWindow::setStatusBarStatistics(int vertexCount, int facesCount)
{
	QString str;
	str.sprintf("Vertexes: %d", vertexCount);
	lblVertexCount->setText(str);

	str.sprintf("Faces: %d", facesCount);
	lblFacesCount->setText(str);
}

/*****************************************************************************************************/
void MainWindow::setRenderTimeStatistics(int timeMsec)
{
	QString str;
	str.sprintf("Render time: %d ms (%d FPS)", timeMsec, 1000/timeMsec);
}

/*****************************************************************************************************/
MainWindow::~MainWindow()
{
}
