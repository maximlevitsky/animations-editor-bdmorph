
#include <algorithm>
#include <QDockWidget>
#include <QListWidgetItem>
#include <QPixmap>
#include <QImage>
#include <QPainter>
#include <QAction>
#include <QPixmap>
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
AnimationPanel::AnimationPanel(QWidget* parent) : QDockWidget(parent), videoModel(NULL), renderer(NULL), timeEditItem(-1),animationRunning(false)
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

	/* Keyframes list */
	connect_(lstKeyFrames, currentRowChanged (int), this, onCurrentListItemChanged(int));
	connect_(lstKeyFrames, itemDoubleClicked(QListWidgetItem *), this, onCloneKeyFrame());
	connect_(cloneAction, triggered(), this, onCloneKeyFrame());
	connect_(changeTimeAction, triggered(), this, onKeyframeChangeTime());
	connect_(deleteFramesAction, triggered(), this, onDeleteKeyframe());

	/* Time edit */
	timeEdit = new QLineEdit(lstKeyFrames);
	timeEdit->setInputMask("00:00.000");
	timeEdit->setAlignment(Qt::AlignCenter);
	timeEdit->hide();
	connect_(timeEdit, editingFinished (), this, onTimeTextFinished());

	/* Slider*/
	connect_(sliderAnimationTime, sliderMoved (int), this, onTimeSliderMoved(int));

	/* Animation timer */
	animationTimer = new QTimer(this);
	connect_(animationTimer, timeout (), this, onAnimationTimer());

	/* Play/pause buttons */
	connect_(btnAnimationPlay, clicked(bool), this, onPlayPauseButtonPressed());
	connect_(btnBackward, clicked(bool), this, onBackwardButton());
}

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

	int id = videoModel->getKeyFrameIndex(frame);
	if (id == -1)
		return;
	updateListItem(id);
	lstKeyFrames->repaint();
}

/******************************************************************************************************************************/
void AnimationPanel::onFrameSwitched(MeshModel* model)
{
	if (!videoModel)
		return;

	VideoKeyFrame* keyFrame = dynamic_cast<VideoKeyFrame*>(model);
	if (!keyFrame) {
		return;
	}

	int index = videoModel->getKeyFrameIndex(keyFrame);
	lstKeyFrames->setCurrentRow(index);
	updateTimeSlider();
}

/******************************************************************************************************************************/
void AnimationPanel::onCurrentListItemChanged(int currentRow )
{
	VideoKeyFrame* newKeyFrame = getSelectedKeyframe();
	if (newKeyFrame)
		emit frameSelectionChanged(newKeyFrame);
}

/******************************************************************************************************************************/

void AnimationPanel::onCloneKeyFrame()
{
	if (animationRunning || !videoModel)
		return;

	VideoKeyFrame* currentKeyFrame = getSelectedKeyframe();
	if (currentKeyFrame == NULL) {
		if (videoModel->count())
			currentKeyFrame = videoModel->getKeyframeByIndex(videoModel->count()-1);
		else
			return;
	}

	int currentIndex = videoModel->getKeyFrameIndex(currentKeyFrame);
	videoModel->forkFrame(currentKeyFrame);
	updateItems(currentIndex+1);
	updateTimeSlider();

	emit frameSelectionChanged(getSelectedKeyframe());
}

/******************************************************************************************************************************/
void AnimationPanel::onDeleteKeyframe()
{
	if (animationRunning)
		return;

	VideoKeyFrame* currentKeyFrame = getSelectedKeyframe();
	if (currentKeyFrame == NULL)
		return;

	if (videoModel->count() == 1)
		return;

	emit frameSelectionChanged(NULL);

	int currentIndex = lstKeyFrames->currentRow();
	videoModel->deleteFrame(currentKeyFrame);

	updateItems(currentIndex);

	if (currentIndex == videoModel->count())
		currentIndex--;
	lstKeyFrames->setCurrentRow(currentIndex);

	currentKeyFrame = getSelectedKeyframe();
	emit frameSelectionChanged(currentKeyFrame);

	if (currentKeyFrame && currentIndex == videoModel->count() - 1)
		currentKeyFrame->duration = 200;
}
/******************************************************************************************************************************/

void AnimationPanel::onKeyframeChangeTime()
{
	if (animationRunning)
		return;

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

void AnimationPanel::onPlayPauseButtonPressed()
{
	if (!videoModel)
		return;

	if (animationRunning) {
		animationTimer->stop();
		btnAnimationPlay->setIcon(QIcon(":/icons/play.png"));
		lstKeyFrames->setDisabled(false);
		sliderAnimationTime->setDisabled(false);
		emit animationStopped();
	}else {
		emit animationStarted();
		lstKeyFrames->setDisabled(true);
		sliderAnimationTime->setDisabled(true);
		animationTimer->setInterval(INTEREVAL);
		animationTimer->setSingleShot(false);
		animationTimer->start();
		btnAnimationPlay->setIcon(QIcon(":/icons/pause.png"));
	}

	animationRunning = !animationRunning;
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

	VideoKeyFrame* nextFrame = videoModel->getKeyframeByIndex(newIndex+1);

	if (nextFrame != NULL)
	{
		int currTimeMsec = videoModel->getKeyFrameTimeMsec(prevFrame);
		int nextTimeMsec = videoModel->getKeyFrameTimeMsec(nextFrame);

		double t = newValue - currTimeMsec;
		t /= (nextTimeMsec-currTimeMsec);

		{
			double time = videoModel->pFrame->interpolate_frame(prevFrame,nextFrame,t);
			emit frameSelectionChanged(videoModel->pFrame);
			emit FPSUpdated(time);
		}
	}
}
/******************************************************************************************************************************/

void AnimationPanel::onAnimationTimer()
{
	int pos = sliderAnimationTime->sliderPosition()+INTEREVAL;

	if (pos > sliderAnimationTime->maximum()) {

		if (btnRepeat->isChecked())
			sliderAnimationTime->setSliderPosition(0);
		else {
			btnAnimationPlay->click();
		}
	}
	else
		sliderAnimationTime->setSliderPosition(pos);

	onTimeSliderMoved(pos);

}

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

