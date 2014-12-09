
#include <qdialog.h>
#include <QDockWidget>
#include <QtOpenGL>
#include <QTimer>
#include <QLineEdit>
#include "ui_AnimationPanel.h"

class VideoModel;
class KVFModel;
class VideoKeyFrame;
class ThumbnailRenderer;
class MeshModel;

class AnimationPanel : public QDockWidget, public Ui_AnimationPanel
{
	Q_OBJECT
public:
	AnimationPanel(QWidget* parent);
	void setThumbailRenderer(ThumbnailRenderer* r) { renderer = r; }

public slots:
	/* Notifications from outside */
	void onFrameSwitched(MeshModel* model);
	void onFrameEdited(KVFModel* model);
	void onVideoModelLoaded(VideoModel* model);
	void onTextureChanged(GLuint texture);

	/* Signals from list view*/
	void onCurrentListItemChanged(int currentRow );
	void onCloneKeyFrame();
	void onDeleteKeyframe();
	void onKeyframeChangeTime();

	void onTimeTextFinished();
	void onTimeSliderMoved(int newValue);

	void onPlayPauseButtonPressed();
	void onBackwardButton();
	void onAnimationTimer();

signals:
	/* we emit this when user clicks on a different frame */
	void frameSelectionChanged(MeshModel* model);
	void animationStarted();
	void animationStopped();
	void FPSUpdated(double msec);

private:
	VideoModel* videoModel;
	ThumbnailRenderer* renderer;

	void updateItems(int startItem);
	void updateTimeSlider();

private:
	VideoKeyFrame* getSelectedKeyframe();
	void updateListItem(int id);
	void insertPlus(int id);

	QIcon plusIcon;
	QLineEdit *timeEdit;
	int timeEditItem;
	bool animationRunning;

	QTimer *animationTimer;
};
