
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
#include "OffScreenRenderer.h"
#include "utils.h"

#define INTEREVAL (1000/60)

/******************************************************************************************************************************/
AnimationPanel::AnimationPanel(QWidget* parent) :
		QDockWidget(parent), programstate(NULL)
{
	setupUi(this);
	plusIcon = QIcon(":/AnimationPanel/add.png");

	QAction *cloneAction        = new QAction("Clone frame...", lstKeyFrames);
	QAction *deleteFramesAction = new QAction("Delete frame",   lstKeyFrames);
	QAction *changeTimeAction   = new QAction("Change time...", lstKeyFrames);
	QAction *loadKeyframe       = new QAction("Load from file...", lstKeyFrames);

	lstKeyFrames->addAction(cloneAction);
	lstKeyFrames->addAction(deleteFramesAction);
	lstKeyFrames->addAction(changeTimeAction);
	lstKeyFrames->addAction(loadKeyframe);
	lstKeyFrames->setContextMenuPolicy(Qt::ActionsContextMenu);

	/* Play/pause buttons */
	connect_(btnAnimationPlay, clicked(bool), 				this, onPlayPauseButtonPressed());
	connect_(btnRepeat, clicked(bool),						this, onRepeatButtonClicked(bool));

	/* Slider*/
	connect_(sliderAnimationTime, sliderMoved (int), 		this, onTimeSliderMoved(int));

	/* Keyframes list */
	connect_(lstKeyFrames, itemPressed(QListWidgetItem*), 	this, onItemClicked(QListWidgetItem*));
	connect_(cloneAction, triggered(), 						this, onCloneKeyFramePressed());
	connect_(changeTimeAction, triggered(), 				this, onKeyframeChangeTime());
	connect_(deleteFramesAction, triggered(), 				this, onDeleteKeyframe());
	connect_(loadKeyframe, triggered(),						this, onLoadKeyframe());

	/* Time edit */
	timeEdit = new QLineEdit(lstKeyFrames);
	timeEdit->setInputMask("00:00.000");
	timeEdit->setAlignment(Qt::AlignCenter);
	timeEdit->hide();
	connect_(timeEdit, editingFinished (), 					this, onTimeTextFinished());
	connect_(btnBackward, clicked(),						this, onBackwardButtonPressed());

	sliderAnimationTime->setMinimum(0);
	sliderAnimationTime->setTickPosition(QSlider::NoTicks);

	thumbnailRenderer = new OffScreenRenderer(NULL, NULL,128,128);
}

/******************************************************************************************************************************/

AnimationPanel::~AnimationPanel()
{
	delete thumbnailRenderer;
}

/******************************************************************************************************************************/
/* EXTERNAL EVENTS */
/******************************************************************************************************************************/

void AnimationPanel::programStateUpdated(int flags, void *param)
{
	if (!programstate) return;

	if (flags & ProgramState::KEYFRAME_LIST_EDITED)
	{
		/* See if one of our keyframes got updated
		 * If so, update list items for it and all following keyframes */
		int updated_index = programstate->getCurrentKeyframeId();
		if (updated_index != -1)
		{
			updateItems(updated_index);
			updateTimeSlider();
		}
	}

	if (flags & ProgramState::KEYFRAME_EDITED )
	{
		int updated_index = programstate->getCurrentKeyframeId();
		updateListItem(updated_index);
	}


	if (flags & ProgramState::TEXTURE_CHANGED)
	{
		/* Texture changed - need to re-render everything */
		thumbnailRenderer->setTexture(programstate->texture);
		updateItems(0);
	}

	if ( flags & (ProgramState::ANIMATION_STEPPED))
	{
		sliderAnimationTime->setValue(programstate->currentAnimationTime);
		int index = programstate->getCurrentKeyframeId();
		selectFrame(index);
	}


	if (flags & ProgramState::CURRENT_MODEL_CHANGED)
	{
		sliderAnimationTime->setValue(programstate->currentAnimationTime);
	}

	if (flags & ProgramState::MODE_CHANGED)
	{
		if (programstate->getCurrentMode() != ProgramState::PROGRAM_MODE_ANIMATION) {
			updateItems(0);
			updateTimeSlider();
		}

		ProgramState::PROGRAM_MODE mode = programstate->getCurrentMode();
		lstKeyFrames->setDisabled(mode == ProgramState::PROGRAM_MODE_BUSY);
		sliderAnimationTime->setDisabled(mode == ProgramState::PROGRAM_MODE_BUSY);
		btnBackward->setDisabled(mode == ProgramState::PROGRAM_MODE_BUSY);

		if (mode == ProgramState::PROGRAM_MODE_BUSY)
			btnAnimationPlay->setIcon(QIcon(":/icons/pause.png"));
		else
			btnAnimationPlay->setIcon(QIcon(":/icons/play.png"));


	}

	if (flags & ProgramState::CURRENT_MODEL_CHANGED)
	{
		/* See if one of our frames is selected - if so select it */
		int newIndex = programstate->getCurrentKeyframeId();
		if (newIndex != -1)
			selectFrame(newIndex);
	}
}

/******************************************************************************************************************************/
/* User input on our list  */
/******************************************************************************************************************************/

void AnimationPanel::onItemClicked(QListWidgetItem *item)
{
	if (!programstate) return;
	int newID = lstKeyFrames->row(item);
	selectFrame(newID);

	programstate->switchToKeyframe(newID);

	/* If we clicked on + sign, add new as a clone of last one */
	if (newID == programstate->getKeyframeCount()) {
		programstate->cloneKeyframe(programstate->getKeyframeCount()-1);
		return;
	}

}

/******************************************************************************************************************************/
void AnimationPanel::onCloneKeyFramePressed()
{
	if (!programstate) return;

	int currentID = programstate->getCurrentKeyframeId();
	if (currentID == -1) return;
	programstate->cloneKeyframe(currentID);
}

/******************************************************************************************************************************/
void AnimationPanel::onDeleteKeyframe()
{
	if (!programstate) return;

	/* Find what keyframe is selected - if none then its false call */
	int currentID = programstate->getCurrentKeyframeId();
	if (currentID == -1) return;
	programstate->deleteKeyFrame(currentID);
}

/******************************************************************************************************************************/
void AnimationPanel::onKeyframeChangeTime()
{
	if (!programstate) return;

	int currentID = programstate->getCurrentKeyframeId();
	if (currentID == -1 || currentID == 0) return;

	QRect r = lstKeyFrames->visualItemRect(lstKeyFrames->item(currentID));

	int upstripe = (r.height() - 128)/2;

	r.setLeft(r.left()+10);
	r.setRight(r.right()-10);

	timeEdit->move(r.left(),r.bottom() - timeEdit->height()/2 - upstripe);
	timeEdit->resize(r.width(),timeEdit->height());
	timeEdit->setText(QString::fromStdString(printTime(programstate->getKeyframeTime(currentID))));
	timeEdit->show();
	timeEdit->setFocus();
	timeEdit->setCursorPosition(0);
}

/******************************************************************************************************************************/
void AnimationPanel::onTimeTextFinished()
{
	timeEdit->hide();
	if (!programstate) return;

	int newTime = getTime(timeEdit->text().toStdString());

	if (newTime < 0)
		return;

	int currentID = programstate->getCurrentKeyframeId();
	if (currentID == -1 || currentID == 0) return;

	int prevFrameTime = programstate->getKeyframeTime(currentID-1);
	programstate->setKeyframeTime(currentID-1,newTime-prevFrameTime);
	programstate->switchToKeyframe(currentID);
}

/******************************************************************************************************************************/
/* Time Slider  */
/******************************************************************************************************************************/

void AnimationPanel::onPlayPauseButtonPressed()
{
	if (!programstate) return;
	ProgramState::PROGRAM_MODE mode = programstate->getCurrentMode();

	if (mode == ProgramState::PROGRAM_MODE_BUSY)
		programstate->stopAnimations();
	else
		programstate->startAnimations(sliderAnimationTime->value());
}
/******************************************************************************************************************************/

void AnimationPanel::onRepeatButtonClicked(bool checked)
{
	if (!programstate) return;
	programstate->setAnimationRepeat(checked);
}

/******************************************************************************************************************************/
void AnimationPanel::onTimeSliderMoved(int newValue)
{
	if (!programstate) return;
	programstate->interpolateFrame(newValue);
}

/******************************************************************************************************************************/
/* Other  */
/******************************************************************************************************************************/
void AnimationPanel::updateItems(int startItem)
{
	if (!programstate) return;
	if (startItem == -1) return;

	int frameCount = programstate->getKeyframeCount();
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
	if (!programstate || !programstate->videoModel) return;

	VideoKeyFrame* frame = programstate->videoModel->getKeyframeByIndex(id);
	if (!frame) return;

	QListWidgetItem* item = lstKeyFrames->item(id);

	if (!item) {
		item = new QListWidgetItem;
		lstKeyFrames->insertItem(id,item);
	}

	/* This code is gross... yuck and I wrote it*/
	QImage im;
	thumbnailRenderer->renderToQImage(frame, im,30,1);
	QPixmap p = QPixmap::fromImage(im);

	QPainter painter(&p);
	painter.drawText(im.rect(), Qt::AlignBottom | Qt::AlignCenter,
			QString::fromStdString(printTime(programstate->videoModel->getKeyFrameTimeMsec(frame))));
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

void AnimationPanel::selectFrame(int index)
{
	lstKeyFrames->scrollToItem(lstKeyFrames->item(index+1));
	lstKeyFrames->setCurrentRow(index);
}

/******************************************************************************************************************************/
void AnimationPanel::updateTimeSlider()
{
	if (!programstate || !programstate->videoModel) return;

	int totalDuration = programstate->videoModel->getTotalTime();
	int current_time = programstate->getKeyframeTime(programstate->getCurrentKeyframeId());
	sliderAnimationTime->setMaximum(totalDuration);
	sliderAnimationTime->setValue(current_time);
}

/******************************************************************************************************************************/
void AnimationPanel::onShowHide(bool show)
{
	setVisible(show);
}
/******************************************************************************************************************************/

void AnimationPanel::onBackwardButtonPressed()
{
	if (!programstate) return;
	programstate->switchToKeyframe(0);
}

/******************************************************************************************************************************/

void AnimationPanel::onLoadKeyframe()
{
    QString filename = QFileDialog::getOpenFileName(this, tr("Choose model for the keyframe"),
    		QString(), QLatin1String("*.obj *.off"));
    if (filename == NULL)
		return;

    programstate->loadKeyframe(filename.toStdString());
}
