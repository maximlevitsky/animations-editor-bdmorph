
#include <qdialog.h>
#include <QDockWidget>
#include <QtOpenGL>
#include <QTimer>
#include <QLineEdit>
#include <QThread>
#include "ui_AnimationPanel.h"

class VideoModel;
class KVFModel;
class VideoKeyFrame;
class ThumbnailRenderer;
class MeshModel;

/*****************************************************************************************/

class AnimationThread : public QThread
{
	Q_OBJECT
public:

	AnimationThread() : should_stop(false), videoModel(NULL),startTimeMsec(0)
	{}

	void start(VideoModel* model, int starttime)
	{
		videoModel = model;
		startTimeMsec = starttime;
		QThread::start();
	}

	void stop()
	{
		should_stop = true;
		wait();
		videoModel = NULL;
		startTimeMsec = 0;
	}

	void setRepeat(bool newrepeat) { repeat  = newrepeat;  }
signals:
	void animationStarted();
	void animationStopped();
	void frameSwitched(MeshModel* model);

private:
	bool should_stop;
	bool repeat;
	virtual void run();

	VideoModel* videoModel;
	int startTimeMsec;
};

/*****************************************************************************************/

class AnimationPanel : public QDockWidget, public Ui_AnimationPanel
{
	Q_OBJECT
public:
	AnimationPanel(QWidget* parent);
	void setThumbailRenderer(ThumbnailRenderer* r) { renderer = r; }

	AnimationThread animationThread;

public slots:
	/* Notifications from outside */
	void onFrameSwitched(MeshModel* model);
	void onFrameEdited(MeshModel* model);
	void onVideoModelLoaded(VideoModel* model);
	void onTextureChanged(GLuint texture);

	/* Signals from list view*/
	void onItemClicked(QListWidgetItem *item);
	void onCloneKeyFramePressed();
	void onDeleteKeyframe();
	void onKeyframeChangeTime();

	void onTimeTextFinished();
	void onTimeSliderMoved(int newValue);

	void onPlayPauseButtonPressed();
	void onBackwardButton();

	void onAnimationStarted();
	void onAnimationStopped();

	void onRepeatButtonClicked(bool checked);

	void onClose();

signals:
	/* we emit this when user clicks on a different frame */
	void frameSwitched(MeshModel* model);
	void FPSUpdated(double msec);

private:
	VideoModel* videoModel;
	ThumbnailRenderer* renderer;
	MeshModel* currentRenderedModel;

private:
	void updateItems(int startItem);
	void updateTimeSlider();

	VideoKeyFrame* getSelectedKeyframe();
	int getSelectedKeyframeID();


	void updateListItem(int id);
	void insertPlus(int id);

	QIcon plusIcon;
	QLineEdit *timeEdit;
	int timeEditItem;
	bool animationRunning;


};
