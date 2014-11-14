
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
}

MainWindow::~MainWindow()
{
}
