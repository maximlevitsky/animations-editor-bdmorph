
#include <qdialog.h>
#include <QDockWidget>
#include "ui_AnimationPanel.h"
#include "ThumbnailRenderer.h"

class VideoModel;
class KVFModel;
class VideoKeyFrame;

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
signals:
	/* we emit this when user clicks on a different frame */
	void frameSelectionChanged(MeshModel* model);

private slots:
	void onCurrentListItemChanged(int currentRow );
	void onCloneKeyFrame();
	void onDeleteKeyframe();
	void onKeyframeChangeTime();
	void onTextureChanged(GLuint texture);

	void onLstitemDoubleClicked ();

private:
	VideoModel* currentVideoModel;
	ThumbnailRenderer* renderer;

	VideoKeyFrame* getSelectedKeyframe();

	void updateListItem(int id);
	void insertPlus(int id);
	void updateItems(int startItem);

	QIcon plusIcon;
};
