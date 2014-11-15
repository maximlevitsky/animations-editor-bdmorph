
#include "MainWindow.h"
#include "MainScene.h"
#include "SidePanel.h"
#include "AnimationPanel.h"
#include <QDockWidget>

MainWindow::MainWindow()
{
	setupUi(this);
	mainScene = new MainScene(this);
	setCentralWidget(mainScene);


	/* create side panel and connect it to us */
	sidePanel = new SidePanel(this);
	connect(sidePanel->btnLoadModel, SIGNAL(clicked()), mainScene, SLOT(loadModel()));
	connect(sidePanel->btnLoadTexture, SIGNAL(clicked()), mainScene, SLOT(chooseTexture()));
	connect(sidePanel->btnResetMesh, SIGNAL(clicked()), mainScene, SLOT(resetPoints()));
	connect(sidePanel->btnResetTexture, SIGNAL(clicked()), mainScene, SLOT(resetTexture()));

	connect(sidePanel->btnResetPins, SIGNAL(clicked()), mainScene, SLOT(clearPins()));
	connect(sidePanel->btnResetTransform, SIGNAL(clicked()), mainScene, SLOT(resetTransform()));
	connect(sidePanel->btnSaveLog, SIGNAL(clicked()), mainScene, SLOT(saveLog()));
	connect(sidePanel->btnReplayLog, SIGNAL(clicked()), mainScene, SLOT(runLog()));
	connect(sidePanel->btnUndo, SIGNAL(clicked()), mainScene, SLOT(undoModel()));
	connect(sidePanel->btnRedo, SIGNAL(clicked()), mainScene, SLOT(redoModel()));



	connect(sidePanel->sliderWireframeTransparency, SIGNAL(valueChanged(int)), mainScene, SLOT(changeWireframe(int)));
	connect(sidePanel->sliderAlpha, SIGNAL(valueChanged(int)), mainScene, SLOT(changeAlpha(int)));
	sidePanel->sliderAlpha->setValue(20);


	animationPanel = new AnimationPanel(this);
	animationPanel->setTitleBarWidget(new QWidget(this));
	addDockWidget(Qt::BottomDockWidgetArea, animationPanel);
	addDockWidget(Qt::RightDockWidgetArea, sidePanel);

}

MainWindow::~MainWindow()
{
}
