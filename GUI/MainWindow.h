#ifndef MAINWINDOW_H
#define MAINWINDOW_H


#include <QMainWindow>
#include <QtOpenGL>
#include "ui_MainWindow.h"
#include "ProgramState.h"

class SidePanel;
class AnimationPanel;
class EditorWindow;
class OffScreenRenderer;
class VideoModel;
class MeshModel;
class KVFModel;
class QLabel;
class BDMORPHModel;
class OutlineModel;
class QProgressBar;

class MainWindow : public QMainWindow, public Ui_MainWindow
{
	Q_OBJECT
public:
	MainWindow();
	virtual ~MainWindow();
public slots:
	void programStateUpdated(int flags, void *param);
	void onSaveScreenShot();
	void onSaveVideo();
	void onAbout();
	void onInterpolationTest();
	void onToggleDebugConsole(bool on);
	void onNewProject();
private:
	SidePanel* sidePanel;
	AnimationPanel* animationPanel;
	EditorWindow *editorWindow;

	QLabel* lblMode;
	QLabel* lblVertexCount;
	QLabel* lblFacesCount;
	QLabel* lblFPS;
	QLabel* lblSelectedVertex;
	QLabel* lblSelectedFace;

	QFrame *progressFrame;
	QProgressBar *progressIndicator;
	QPushButton *cancelButton;

	ProgramState* programstate;
};

#endif
