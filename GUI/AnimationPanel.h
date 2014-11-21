
#include <qdialog.h>
#include <QDockWidget>
#include "ui_AnimationPanel.h"
#include "ThumbnailRenderer.h"

class VideoModel;
class KVFModel;

class AnimationPanel : public QDockWidget, public Ui_AnimationPanel
{
	Q_OBJECT
public:
	AnimationPanel(QWidget* parent);
	void setThumbailRenderer(ThumbnailRenderer* r) { renderer = r; }

public slots:
	void onFrameSwitched(MeshModel* model);
	void onFrameEdited(KVFModel* model);
	void onVideoModelLoaded(VideoModel* model);
private:


private:
	VideoModel* currentVideoModel;
	ThumbnailRenderer* renderer;
};
