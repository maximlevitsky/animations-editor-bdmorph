
#include "AnimationPanel.h"
#include <QDockWidget>
#include <QListWidgetItem>
#include <QPixmap>
#include <QImage>
#include <QPainter>
#include <QAction>
#include <QPixmap>

#include "VideoModel.h"
#include "Utils.h"


/******************************************************************************************************************************/
AnimationPanel::AnimationPanel(QWidget* parent) : QDockWidget(parent), currentVideoModel(NULL), renderer(NULL)
{
	setupUi(this);
	QAction *cloneAction =new QAction("Clone frame...", lstKeyFrames);
	QAction *deleteFramesAction =new QAction("Delete frame", lstKeyFrames);
	QAction *changeTimeAction =new QAction("Change time...", lstKeyFrames);

	lstKeyFrames->addAction(cloneAction);
	lstKeyFrames->addAction(deleteFramesAction);
	lstKeyFrames->addAction(changeTimeAction);
	lstKeyFrames->setContextMenuPolicy(Qt::ActionsContextMenu);

	connect_(lstKeyFrames, currentRowChanged (int), this, onCurrentListItemChanged(int));

	connect_(cloneAction, triggered(), this, onCloneKeyFrame());
	connect_(changeTimeAction, triggered(), this, onKeyframeChangeTime());
	connect_(deleteFramesAction, triggered(), this, onDeleteKeyframe());

	/* TODO */
	plusIcon = QIcon("/home/maxim/2.png");
}

/******************************************************************************************************************************/
void AnimationPanel::onVideoModelLoaded(VideoModel* model)
{
	currentVideoModel = model;
	lstKeyFrames->clear();

	if (currentVideoModel)
	{
		updateListItem(0);
		insertPlus(1);
	}
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
}

/******************************************************************************************************************************/
void AnimationPanel::onFrameSwitched(MeshModel* model)
{
	/* we will emit this, the time slider will emit this, and on load we will switch to frame 0 because main window will emit this*/
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
	if (newKeyFrame == NULL)
		return;
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
VideoKeyFrame* AnimationPanel::getSelectedKeyframe()
{
	if (!currentVideoModel)
			return NULL;

	int currentRow = lstKeyFrames->currentRow();
	VideoKeyFrame* newKeyFrame = currentVideoModel->keyframe(currentRow);
	return newKeyFrame;
}

/******************************************************************************************************************************/
void AnimationPanel::updateListItem(int id)
{
	if (!currentVideoModel)
		return;

	VideoKeyFrame* frame = currentVideoModel->keyframe(id);
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
	painter.drawText(im.rect(), Qt::AlignBottom | Qt::AlignCenter, QString("00:00:00")); /*TODO*/

	item->setIcon(QIcon(p));
}

/******************************************************************************************************************************/

void AnimationPanel::updateItems(int startItem)
{
	if (!currentVideoModel)
			return;

	int frameCount = currentVideoModel->getKeyFrameCount();
	for (int frame = startItem ; frame < frameCount ; frame++)
		updateListItem(frame);

	insertPlus(frameCount);
	int firstItemToRemove = frameCount + 1;

	while (lstKeyFrames->count() > firstItemToRemove)
	{
		QListWidgetItem* item = lstKeyFrames->takeItem(firstItemToRemove);
		delete item;
	}
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
void AnimationPanel::onDeleteKeyframe()
{
	VideoKeyFrame* currentKeyFrame = getSelectedKeyframe();
	if (currentKeyFrame == NULL)
		return;

	if (currentVideoModel->getKeyFrameCount() == 1)
		return;

	int currentIndex = lstKeyFrames->currentRow();
	currentVideoModel->deleteFrame(currentKeyFrame);
	updateItems(currentIndex);

}
/******************************************************************************************************************************/

void AnimationPanel::onKeyframeChangeTime()
{
	VideoKeyFrame* currentKeyFrame = getSelectedKeyframe();
	if (currentKeyFrame == NULL)
		return;
	int currentIndex = lstKeyFrames->currentRow();

	/* TODO */

}

