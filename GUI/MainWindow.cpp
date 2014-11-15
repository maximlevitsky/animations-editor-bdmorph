
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

	sidePanel = new SidePanel(this);
	addDockWidget(Qt::RightDockWidgetArea, sidePanel);

	animationPanel = new AnimationPanel(this);
	animationPanel->setTitleBarWidget(new QWidget(this));
	addDockWidget(Qt::BottomDockWidgetArea, animationPanel);

	//connect(pinMode, SIGNAL(toggled(bool)), this, SLOT(setCheckState(bool)));
	connect(sidePanel->btnLoadModel, SIGNAL(clicked()), mainScene, SLOT(loadModel()));
	connect(sidePanel->btnLoadTexture, SIGNAL(clicked()), mainScene, SLOT(chooseTexture()));

	connect(sidePanel->sliderWireframeTransparency, SIGNAL(valueChanged(int)), mainScene, SLOT(changeWireframe(int)));
}

MainWindow::~MainWindow()
{
}
