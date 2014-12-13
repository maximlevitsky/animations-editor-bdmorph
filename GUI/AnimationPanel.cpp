
#include <algorithm>
#include <QDockWidget>
#include <QListWidgetItem>
#include <QPixmap>
#include <QImage>
#include <QPainter>
#include <QAction>
#include <QPixmap>
#include <QThread>
#include <stdio.h>

#include "AnimationPanel.h"
#include "VideoModel.h"
#include "MeshModel.h"
#include "ThumbnailRenderer.h"
#include "Utils.h"

#define INTEREVAL (1000/60)

QString printTime(int time)
{
	int fraction = (time % 1000);
	time /= 1000;
	int seconds = time % 60;
	time /= 60;
	int minutes = time;

	QString retval;
	retval.sprintf("%02d:%02d.%03d", minutes, seconds,fraction);
	return retval;
}


/******************************************************************************************************************************/
AnimationPanel::AnimationPanel(QWidget* parent) :
		QDockWidget(parent), videoModel(NULL), renderer(NULL), timeEditItem(-1),animationRunning(false),
		currentRenderedModel(NULL)
{
	setupUi(this);
	plusIcon = QIcon(":/AnimationPanel/add.png");

	QAction *cloneAction        = new QAction("Clone frame...", lstKeyFrames);
	QAction *deleteFramesAction = new QAction("Delete frame",   lstKeyFrames);
	QAction *changeTimeAction   = new QAction("Change time...", lstKeyFrames);

	lstKeyFrames->addAction(cloneAction);
	lstKeyFrames->addAction(deleteFramesAction);
	lstKeyFrames->addAction(changeTimeAction);
	lstKeyFrames->setContextMenuPolicy(Qt::ActionsContextMenu);

	/* Play/pause buttons */
	connect_(btnAnimationPlay, clicked(bool), 				this, onPlayPauseButtonPressed());
	connect_(btnBackward, clicked(bool), 					this, onBackwardButton());
	connect_(btnRepeat, clicked(bool),						this, onRepeatButtonClicked(bool));

	/* Slider*/
	connect_(sliderAnimationTime, sliderMoved (int), 		this, onTimeSliderMoved(int));

	/* Keyframes list */
	connect_(lstKeyFrames, itemClicked(QListWidgetItem*), 	this, onItemClicked(QListWidgetItem*));
	connect_(cloneAction, triggered(), 						this, onCloneKeyFramePressed());
	connect_(changeTimeAction, triggered(), 				this, onKeyframeChangeTime());
	connect_(deleteFramesAction, triggered(), 				this, onDeleteKeyframe());

	/* Time edit */
	timeEdit = new QLineEdit(lstKeyFrames);
	timeEdit->setInputMask("00:00.000");
	timeEdit->setAlignment(Qt::AlignCenter);
	timeEdit->hide();
	connect_(timeEdit, editingFinished (), 					this, onTimeTextFinished());
}

/******************************************************************************************************************************/
/* EXTERNAL EVENTS */
/******************************************************************************************************************************/
void AnimationPanel::onVideoModelLoaded(VideoModel* model)
{
	videoModel = model;
	lstKeyFrames->clear();
	if (videoModel) {
		updateItems(0);
	}
}

/******************************************************************************************************************************/
void AnimationPanel::onTextureChanged(GLuint newtex)
{
	/* need to update all list items*/
	updateItems(0);
}

/******************************************************************************************************************************/
void AnimationPanel::onFrameEdited(MeshModel* model)
{
	if (!videoModel) return;

	VideoKeyFrame* frame = dynamic_cast<VideoKeyFrame*>(model);
	if (!frame) return;

	int index = videoModel->getKeyFrameIndex(frame);
	if (index == -1) return;

	updateListItem(index);
	lstKeyFrames->repaint();
}

/******************************************************************************************************************************/
void AnimationPanel::onFrameSwitched(MeshModel* model)
{
	if (!videoModel)
		return;

	currentRenderedModel = model;

	VideoKeyFrame* keyFrame = dynamic_cast<VideoKeyFrame*>(model);
	if (!keyFrame) return;

	int index = videoModel->getKeyFrameIndex(keyFrame);
	if (index == -1) return;
	lstKeyFrames->setCurrentRow(index);
	updateTimeSlider();
}

/******************************************************************************************************************************/
void AnimationPanel::onAnimationStarted()
{
	lstKeyFrames->setDisabled(true);
	sliderAnimationTime->setDisabled(true);
	btnAnimationPlay->setIcon(QIcon(":/icons/pause.png"));
	animationRunning = true;
}

/******************************************************************************************************************************/
void AnimationPanel::onAnimationStopped()
{
	animationRunning = false;
	btnAnimationPlay->setIcon(QIcon(":/icons/play.png"));
	lstKeyFrames->setDisabled(false);
	sliderAnimationTime->setDisabled(false);
}

/******************************************************************************************************************************/
/* User input on our list  */
/******************************************************************************************************************************/

void AnimationPanel::onItemClicked(QListWidgetItem *item)
{
	if (!videoModel)
		return;

	int currentRow = lstKeyFrames->row(item);
	if (currentRow == videoModel->count()) {
		onCloneKeyFramePressed();
		return;
	}

	/* Single press on list item more or less */
	VideoKeyFrame* newKeyFrame = videoModel->getKeyframeByIndex(currentRow);
	if (newKeyFrame)
		emit frameSwitched(newKeyFrame);
}

/******************************************************************************************************************************/
void AnimationPanel::onCloneKeyFramePressed()
{
	if (animationRunning || !videoModel)
		return;

	/* Try selected item */
	int currentIndex = getSelectedKeyframeID();

	/* If nothing selected, try last keyframe */
	if (currentIndex == -1)
		currentIndex = videoModel->count()-1;

	/* No keyframes - bail out - shouldn't happen  */
	if (currentIndex < 0)
		return;

	VideoKeyFrame* keyframeToClone = videoModel->getKeyframeByIndex(currentIndex);
	if (!keyframeToClone) return;

	VideoKeyFrame* newFrame = videoModel->forkFrame(keyframeToClone);
	currentIndex++;


	/* Update all items after current keyframe */
	updateItems(currentIndex);
	updateTimeSlider();
	emit frameSwitched(newFrame);

	lstKeyFrames->scrollToItem(lstKeyFrames->item(videoModel->count()));
	lstKeyFrames->setCurrentRow(currentIndex);
}

/******************************************************************************************************************************/
void AnimationPanel::onDeleteKeyframe()
{
	if (animationRunning || !videoModel) return;

	/* Can't delete last keyframe */
	if (videoModel->count() == 1) return;

	/* Find what keyframe is selected - if none then its false call */
	int currentIndex = getSelectedKeyframeID();
	if (currentIndex == -1) return;

	VideoKeyFrame* victim = videoModel->getKeyframeByIndex(currentIndex);
	if (!victim) return;

	/* Tell everyone stop using our keyframe if selected for some reason*/
	if (currentRenderedModel == victim)
		emit frameSwitched(NULL);

	/* Delete the frame */
	videoModel->deleteFrame(victim);

	/* Update the items from new current frame and select it*/
	currentIndex = std::max(0,currentIndex-1);
	updateItems(currentIndex);
	lstKeyFrames->setCurrentRow(currentIndex);

	/* Tell everyone about new keyframe */
	VideoKeyFrame* currentFrame = videoModel->getKeyframeByIndex(currentIndex);
	emit frameSwitched(currentFrame);
}
/******************************************************************************************************************************/

void AnimationPanel::onKeyframeChangeTime()
{
	if (animationRunning || !videoModel) return;

	VideoKeyFrame* currentKeyFrame = getSelectedKeyframe();
	if (currentKeyFrame == NULL)
		return;
	int currentIndex = lstKeyFrames->currentRow();

	if (currentIndex == 0)
		return;

	/* TODO */
	QListWidgetItem* item = lstKeyFrames->item(currentIndex);
	QRect r = lstKeyFrames->visualItemRect(item);

	r.setLeft(r.left()+10);
	r.setRight(r.right()-10);

	timeEdit->move(r.left(),r.bottom() - timeEdit->height()-5);
	timeEdit->resize(r.width(),timeEdit->height());
	timeEdit->setText(printTime(videoModel->getKeyFrameTimeMsec(currentKeyFrame)));
	timeEdit->show();
	timeEdit->setFocus();
	timeEdit->setCursorPosition(0);
	timeEditItem = currentIndex;
}
/******************************************************************************************************************************/
void AnimationPanel::onTimeTextFinished()
{
	timeEdit->hide();

	if (animationRunning)
		return;

	VideoKeyFrame* prevFrame = videoModel->getKeyframeByIndex(timeEditItem-1);
	if (!prevFrame)
		return;

	QString time = timeEdit->text();

	QStringList tmp = time.split('.');
	QString fraction = tmp[1];

	tmp = tmp[0].split(':');
	QString minutes = tmp[0];
	QString seconds = tmp[1];

	int result = minutes.toInt() * 60;
	result += seconds.toInt();
	result *= 1000;
	result += fraction.toInt();

	int currentFrameTime = videoModel->getKeyFrameTimeMsec(prevFrame) + prevFrame->duration;
	int diff = result - currentFrameTime;
	int newDuration = prevFrame->duration + diff;

	prevFrame->duration = std::max(1, newDuration);
	updateItems(timeEditItem);
	timeEditItem = -1;
	updateTimeSlider();
}

/******************************************************************************************************************************/
/* Time Slider  */
/******************************************************************************************************************************/

void AnimationPanel::onPlayPauseButtonPressed()
{
	if (!videoModel)
		return;

	if (!animationThread.isRunning())
		animationThread.start(videoModel, sliderAnimationTime->value());
	else
		animationThread.stop();
}

void AnimationPanel::onRepeatButtonClicked(bool checked)
{
	animationThread.setRepeat(checked);
}

void AnimationPanel::onClose()
{
	/* TODO */
	animationThread.stop();
	animationThread.wait();
}

/******************************************************************************************************************************/
void AnimationPanel::onBackwardButton()
{
	sliderAnimationTime->setSliderPosition(0);
	onTimeSliderMoved(0);
}

/******************************************************************************************************************************/
void AnimationPanel::onTimeSliderMoved(int newValue)
{
	if (!videoModel)
		return;

	VideoKeyFrame* prevFrame = videoModel->getLastKeyframeBeforeTime(newValue);
	if (!prevFrame) return;

	int newIndex = videoModel->getKeyFrameIndex(prevFrame);
	int currentIndex = getSelectedKeyframeID();

	if (newIndex != currentIndex && newIndex != -1)
	{
		lstKeyFrames->setCurrentRow(newIndex);
		lstKeyFrames->scrollToItem(lstKeyFrames->item(newIndex));
	}

	VideoKeyFrame* nextFrame = videoModel->getKeyframeByIndex(newIndex+1);

	if (nextFrame != NULL)
	{
		int currTimeMsec = videoModel->getKeyFrameTimeMsec(prevFrame);
		int nextTimeMsec = videoModel->getKeyFrameTimeMsec(nextFrame);

		double t = newValue - currTimeMsec;
		t /= (nextTimeMsec-currTimeMsec);

		double time = videoModel->pFrame->interpolate_frame(prevFrame,nextFrame,t);
		emit frameSwitched(videoModel->pFrame);
		emit FPSUpdated(time);
	}
}

/******************************************************************************************************************************/
/* Other  */
/******************************************************************************************************************************/
void AnimationPanel::updateItems(int startItem)
{
	if (!videoModel)
			return;

	int frameCount = videoModel->count();
	for (int frame = startItem ; frame < frameCount ; frame++)
		updateListItem(frame);

	insertPlus(frameCount);
	int firstItemToRemove = frameCount + 1;

	while (lstKeyFrames->count() > firstItemToRemove)
	{
		QListWidgetItem* item = lstKeyFrames->takeItem(firstItemToRemove);
		delete item;
	}

	lstKeyFrames->repaint();
}

/******************************************************************************************************************************/
void AnimationPanel::updateListItem(int id)
{
	if (!videoModel)
		return;

	VideoKeyFrame* frame = videoModel->getKeyframeByIndex(id);
	if (!frame)
		return;

	QListWidgetItem* item = lstKeyFrames->item(id);

	if (!item) {
		item = new QListWidgetItem;
		lstKeyFrames->insertItem(id,item);
	}

	/* This code is gross... yuck and I wrote it*/
	QImage im = renderer->renderThumbnail(frame);
	QPixmap p = QPixmap::fromImage(im);

	QPainter painter(&p);
	painter.drawText(im.rect(), Qt::AlignBottom | Qt::AlignCenter, printTime(videoModel->getKeyFrameTimeMsec(frame)));

	item->setIcon(QIcon(p));
}

/******************************************************************************************************************************/
void AnimationPanel::insertPlus(int id)
{
	QListWidgetItem* item = lstKeyFrames->item(id);

	if (!item) {
		item = new QListWidgetItem;
		lstKeyFrames->insertItem(id,item);
	}

	item->setIcon(plusIcon);
}

/******************************************************************************************************************************/
VideoKeyFrame* AnimationPanel::getSelectedKeyframe()
{
	if (!videoModel)
			return NULL;

	int currentRow = lstKeyFrames->currentRow();
	VideoKeyFrame* newKeyFrame = videoModel->getKeyframeByIndex(currentRow);
	return newKeyFrame;
}

/******************************************************************************************************************************/
int AnimationPanel::getSelectedKeyframeID()
{
	int id = lstKeyFrames->currentRow();
	if (id >= videoModel->count())
		return -1;
	return id;
}

/******************************************************************************************************************************/
void AnimationPanel::updateTimeSlider()
{
	if (!videoModel)
		return;

	int totalDuration = videoModel->getTotalTime();
	int current_time = 0;

	VideoKeyFrame* selected = getSelectedKeyframe();

	if (selected)
		current_time = videoModel->getKeyFrameTimeMsec(selected);

	sliderAnimationTime->setMinimum(0);
	sliderAnimationTime->setMaximum(totalDuration);
	sliderAnimationTime->setSliderPosition(current_time);
	sliderAnimationTime->setTickPosition(QSlider::NoTicks);
	sliderAnimationTime->setSingleStep(1);
}
/******************************************************************************************************************************/

void AnimationThread::run()
{
	should_stop = false;
	assert(videoModel);

	while (!should_stop)
	{
		this->sleep(1);
	}

	assert(videoModel);
}
/******************************************************************************************************************************/
