#ifndef MAINWINDOW_H
#define MAINWINDOW_H


#include <QMainWindow>
#include <QtOpenGL>
#include "ui_MainWindow.h"

class SidePanel;
class AnimationPanel;
class EditorWindow;
class ThumbnailRenderer;
class VideoModel;
class MeshModel;
class KVFModel;
class QLabel;
class BDMORPHModel;
class OutlineModel;

class MainWindow : public QMainWindow, public Ui_MainWindow
{
	Q_OBJECT
public:
	MainWindow();
	virtual ~MainWindow();

	void setStatusBarStatistics(int vertexCount, int facesCount);
	void clearStatusBar();

public slots:
	void onLoadModel();
	void onUnloadModel();
	void onSaveModel();
    void onChooseTexture();
	void onResetTexture();

	void onFrameSwitched(MeshModel* model);
	void onAnimationpanelNewFrameSelected(MeshModel* model);

	void onSidePanelMenuShowHide(bool checked);
	void onAnimationPanelMenuShowHide(bool checked);

	void onSaveScreenShot();
	void onSaveVideo();
	void onAbout();

	void onEditorSelectionChanged(int selectedVertex, int selectedFace);
	void onInterpolationTest();
	void onFPSUpdated(double msec);

	void onCreateOutlineModel();
	void onMeshCreationRequest(int requestedDensity);

signals:
	void videoModelLoaded(VideoModel* model);
	void frameSwitched(MeshModel* model);
	void textureChanged(GLuint textureRef);
private:
	SidePanel* sidePanel;
	AnimationPanel* animationPanel;
	EditorWindow *mainScene;

	QLabel* lblVertexCount;
	QLabel* lblFacesCount;
	QLabel* lblFPS;
	QLabel* lblSelectedVertex;
	QLabel* lblSelectedFace;

	VideoModel* videoModel;
	MeshModel* currentFrameModel;
	GLuint textureRef;

	OutlineModel* outlineModel;
	ThumbnailRenderer* thumbnailRender;
	void setTexture(const QString file);

	QPixmap texture;
};

#endif
