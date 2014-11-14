#ifndef MAINWINDOW_H
#define MAINWINDOW_H


#include <QMainWindow>
#include "ui_MainWindow.h"

#include <QMouseEvent>
#include <QWheelEvent>
#include <QKeyEvent>

class SidePanel;
class AnimationPanel;
class MainScene;

class MainWindow : public QMainWindow, public Ui_MainWindow
{
	Q_OBJECT
public:
	MainWindow();
	virtual ~MainWindow();
private:
	SidePanel* sidePanel;
	AnimationPanel* animationPanel;
	MainScene *mainScene;
};

#endif
