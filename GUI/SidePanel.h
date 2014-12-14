
#include <QDockWidget>
#include "ui_SidePanel.h"
class MeshModel;
class VideoModel;

class SidePanel : public QDockWidget, public Ui_sidePanel
{
	Q_OBJECT
public:
	SidePanel(QWidget* parent);

public slots:
	void onAnimationStarted();
	void onAnimationStopped();
	void onFrameSwitched(MeshModel* model);
	void onMeshCreateButtonPressed();
	void onVideoModelLoaded(VideoModel* model);
signals:
	void meshCreationRequest(int requestedDensity);

private:
	bool currentisBDMORPH;
	VideoModel *videoModel;
};
