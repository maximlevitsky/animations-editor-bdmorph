
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

	QAction *cloneAction        = new QAction("Clone frame...", lstKeyFrames);
	QAction *deleteFramesAction = new QAction("Delete frame",   lstKeyFrames);
	QAction *changeTimeAction   = new QAction("Change time...", lstKeyFrames);

	lstKeyFrames->addAction(cloneAction);
	lstKeyFrames->addAction(deleteFramesAction);
	lstKeyFrames->addAction(changeTimeAction);
	lstKeyFrames->setContextMenuPolicy(Qt::ActionsContextMenu);

	connect_(lstKeyFrames, currentRowChanged (int), this, onCurrentListItemChanged(int));

	connect_(cloneAction, triggered(), this, onCloneKeyFrame());
	connect_(changeTimeAction, triggered(), this, onKeyframeChangeTime());
	connect_(deleteFramesAction, triggered(), this, onDeleteKeyframe());

	connect_(lstKeyFrames, itemDoubleClicked(QListWidgetItem *), this, onLstitemDoubleClicked ());

	plusIcon = QIcon(":/AnimationPanel/add.png");

	timeEdit = new QLineEdit(lstKeyFrames);
	timeEdit->setInputMask("00:00.000");
	timeEdit->setAlignment(Qt::AlignCenter);
	timeEdit->hide();

	connect_(timeEdit, editingFinished (), this, onTimeTextFinished());
	connect_(sliderAnimationTime, sliderMoved (int), this, onTimeSliderMoved(int));

	animationTimer = new QTimer(this);
	connect_(animationTimer, timeout (), this, onAnimationTimer());
	connect_(btnAnimationPlay, clicked(bool), this, onPlayPauseButtonPressed());
	connect_(btnBackward, clicked(bool), this, onBackwardButton());
}

/******************************************************************************************************************************/
void AnimationPanel::onVideoModelLoaded(VideoModel* model)
{
	videoModel = model;
	lstKeyFrames->clear();

	if (videoModel)
		updateItems(0);
}

/******************************************************************************************************************************/
void AnimationPanel::onTextureChanged(GLuint newtex)
{
	/* need to update all list items*/
	updateItems(0);
}


/******************************************************************************************************************************/
void AnimationPanel::onFrameEdited(KVFModel* model)
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
	if (animationRunning)
		return;

	VideoKeyFrame* currentKeyFrame = getSelectedKeyframe();
	if (currentKeyFrame == NULL)
		return;

	int currentIndex = lstKeyFrames->currentRow();
	videoModel->forkFrame(currentKeyFrame);
	updateItems(currentIndex+1);
	updateTimeSlider();
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
		currentKeyFrame->duration = 1000;

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

void AnimationPanel::onLstitemDoubleClicked ()
{
	if (animationRunning)
		return;

	VideoKeyFrame* currentKeyFrame = getSelectedKeyframe();

	if (currentKeyFrame == NULL)
	{
		if (videoModel->count())
			currentKeyFrame = videoModel->getKeyframeByIndex(videoModel->count()-1);
		else
			return;
	}

	VideoKeyFrame* newFrame = videoModel->forkFrame(currentKeyFrame);
	newFrame->duration = 1000;
	updateItems(videoModel->getKeyFrameIndex(newFrame));
	emit frameSelectionChanged(getSelectedKeyframe());
	updateTimeSlider();
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
	sliderAnimationTime->setTickInterval(totalDuration/20);
	sliderAnimationTime->setSingleStep(1);
}
/******************************************************************************************************************************/


void AnimationPanel::onTimeSliderMoved(int newValue)
{
	if (!videoModel)
		return;

	VideoKeyFrame* prevFrame = videoModel->getLastKeyframeBeforeTime(newValue);
	if (!prevFrame) return;

	int currentIndex = lstKeyFrames->currentRow();
	int newIndex = videoModel->getKeyFrameIndex(prevFrame);
	//if (currentIndex != newIndex) {
	//	lstKeyFrames->setCurrentRow(newIndex);
	//}

	VideoKeyFrame* nextFrame = videoModel->getKeyframeByIndex(newIndex+1);

	if (nextFrame == NULL)
	{
		//emit frameSelectionChanged(prevFrame);
	} else
	{
		int currTimeMsec = videoModel->getKeyFrameTimeMsec(prevFrame);
		int nextTimeMsec = videoModel->getKeyFrameTimeMsec(nextFrame);

		double t = newValue - currTimeMsec;
		t /= (nextTimeMsec-currTimeMsec);

		videoModel->pFrame.interpolate_frame(prevFrame,nextFrame,t);


		/* TODO: we won't emit this on this frame but we will apply BDMORPH and tell main view to show its output */
		/* Apply BDMORPH on prevFrame,nextFrame*/
		emit frameSelectionChanged(&videoModel->pFrame);
	}
}

/******************************************************************************************************************************/

#define INTEREVAL (1000/60)
void AnimationPanel::onPlayPauseButtonPressed()
{
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

void AnimationPanel::onBackwardButton()
{
	sliderAnimationTime->setSliderPosition(0);
	onTimeSliderMoved(0);
}
