#ifndef MAINWINDOW_H
#define MAINWINDOW_H


#include <QMainWindow>
#include "ui_MainWindow.h"

#include <QMouseEvent>
#include <QWheelEvent>
#include <QKeyEvent>
#include <QLabel>

class SidePanel;
class AnimationPanel;
class MainScene;

class MainWindow : public QMainWindow, public Ui_MainWindow
{
	Q_OBJECT
public:
	MainWindow();
	virtual ~MainWindow();

	void setStatusBarStatistics(int vertexCount, int facesCount);
	void setRenderTimeStatistics(int timeMsec);

private:
	SidePanel* sidePanel;
	AnimationPanel* animationPanel;
	MainScene *mainScene;

	QLabel* lblVertexCount;
	QLabel* lblFacesCount;
	QLabel* lblFPS;


};

#endif
