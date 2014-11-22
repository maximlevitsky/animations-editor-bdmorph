
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
AnimationPanel::AnimationPanel(QWidget* parent) : QDockWidget(parent), currentVideoModel(NULL), renderer(NULL), timeEditItem(-1)
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
}

/******************************************************************************************************************************/
void AnimationPanel::onVideoModelLoaded(VideoModel* model)
{
	currentVideoModel = model;
	lstKeyFrames->clear();

	if (currentVideoModel)
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
	if (!currentVideoModel) return;

	VideoKeyFrame* frame = dynamic_cast<VideoKeyFrame*>(model);
	if (!frame) return;

	int id = currentVideoModel->getKeyFrameIndex(frame);
	if (id == -1)
		return;
	updateListItem(id);
	lstKeyFrames->repaint();
}

/******************************************************************************************************************************/
void AnimationPanel::onFrameSwitched(MeshModel* model)
{
	if (!currentVideoModel)
		return;

	VideoKeyFrame* keyFrame = dynamic_cast<VideoKeyFrame*>(model);

	if (!keyFrame) {
		lstKeyFrames->clearSelection();
		return;
	}

	int index = currentVideoModel->getKeyFrameIndex(keyFrame);
	lstKeyFrames->setCurrentRow(index);
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
	VideoKeyFrame* currentKeyFrame = getSelectedKeyframe();
	if (currentKeyFrame == NULL)
		return;

	int currentIndex = lstKeyFrames->currentRow();
	VideoKeyFrame* newFrame = currentVideoModel->forkFrame(currentKeyFrame);
	updateItems(currentIndex+1);
}

/******************************************************************************************************************************/
void AnimationPanel::onDeleteKeyframe()
{
	VideoKeyFrame* currentKeyFrame = getSelectedKeyframe();
	if (currentKeyFrame == NULL)
		return;

	if (currentVideoModel->count() == 1)
		return;

	emit frameSelectionChanged(NULL);

	int currentIndex = lstKeyFrames->currentRow();
	currentVideoModel->deleteFrame(currentKeyFrame);

	updateItems(currentIndex);

	if (currentIndex == currentVideoModel->count())
		currentIndex--;
	lstKeyFrames->setCurrentRow(currentIndex);
	emit frameSelectionChanged(getSelectedKeyframe());

}
/******************************************************************************************************************************/

void AnimationPanel::onKeyframeChangeTime()
{
	VideoKeyFrame* currentKeyFrame = getSelectedKeyframe();
	if (currentKeyFrame == NULL)
		return;
	int currentIndex = lstKeyFrames->currentRow();

	/* TODO */
	QListWidgetItem* item = lstKeyFrames->item(currentIndex);
	QRect r = lstKeyFrames->visualItemRect(item);

	r.setLeft(r.left()+10);
	r.setRight(r.right()-10);

	timeEdit->move(r.left(),r.bottom() - timeEdit->height()-5);
	timeEdit->resize(r.width(),timeEdit->height());
	timeEdit->setText(printTime(currentVideoModel->getKeyFrameTimeMsec(currentKeyFrame)));
	timeEdit->show();
	timeEdit->setFocus();
	timeEdit->setCursorPosition(0);
	timeEditItem = currentIndex;
}
/******************************************************************************************************************************/

void AnimationPanel::onLstitemDoubleClicked ()
{
	VideoKeyFrame* currentKeyFrame = getSelectedKeyframe();

	if (currentKeyFrame == NULL)
	{
		if (currentVideoModel->count())
			currentKeyFrame = currentVideoModel->getKeyframeByIndex(currentVideoModel->count()-1);
		else
			return;
	}

	VideoKeyFrame* newFrame = currentVideoModel->forkFrame(currentKeyFrame);
	updateItems(currentVideoModel->getKeyFrameIndex(newFrame));
	emit frameSelectionChanged(getSelectedKeyframe());
}

/******************************************************************************************************************************/

void AnimationPanel::updateItems(int startItem)
{
	if (!currentVideoModel)
			return;

	int frameCount = currentVideoModel->count();
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
	if (!currentVideoModel)
		return;

	VideoKeyFrame* frame = currentVideoModel->getKeyframeByIndex(id);
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
	painter.drawText(im.rect(), Qt::AlignBottom | Qt::AlignCenter, printTime(currentVideoModel->getKeyFrameTimeMsec(frame)));

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
	if (!currentVideoModel)
			return NULL;

	int currentRow = lstKeyFrames->currentRow();
	VideoKeyFrame* newKeyFrame = currentVideoModel->getKeyframeByIndex(currentRow);
	return newKeyFrame;
}

/******************************************************************************************************************************/
void AnimationPanel::onTimeTextFinished()
{
	timeEdit->hide();

	VideoKeyFrame* frame = currentVideoModel->getKeyframeByIndex(timeEditItem);
	if (!frame)
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

	int currentFrameTime = currentVideoModel->getKeyFrameTimeMsec(frame);
	int diff = result - currentFrameTime;
	int newDuration = frame->duration + diff;

	frame->duration = std::max(1, newDuration);
	updateItems(timeEditItem);
	timeEditItem = -1;
}
