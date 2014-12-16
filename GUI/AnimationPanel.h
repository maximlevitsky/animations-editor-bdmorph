
#include <qdialog.h>
#include <QDockWidget>
#include <QtOpenGL>
#include <QTimer>
#include <QLineEdit>
#include <QThread>
#include "ui_AnimationPanel.h"

#include "ProgramState.h"

class VideoModel;
class KVFModel;
class VideoKeyFrame;
class MeshModel;

/*****************************************************************************************/
class AnimationPanel : public QDockWidget, public Ui_AnimationPanel
{
	Q_OBJECT
public:
	AnimationPanel(QWidget* parent);
	virtual ~AnimationPanel() {  }
public slots:
	void programStateUpdated(int flags, void *param);
	void programStateCreated(ProgramState* state) { programstate = state; }

	/* Signals from list view*/
	void onItemClicked(QListWidgetItem *item);
	void onCloneKeyFramePressed();
	void onDeleteKeyframe();
	void onKeyframeChangeTime();
	void onTimeTextFinished();
	void onTimeSliderMoved(int newValue);
	void onPlayPauseButtonPressed();
	void onRepeatButtonClicked(bool checked);
	void onShowHide(bool show);
	void onBackwardButtonPressed();
	void onLoadKeyframe();
private:
	void updateItems(int startItem);
	void updateListItem(int id);
	void updateTimeSlider();

	void insertPlus(int id);
	void selectFrame(int index);
private:
	QIcon plusIcon;
	QLineEdit *timeEdit;
	ProgramState* programstate;
};
