
#include "AnimationPanel.h"
#include <QDockWidget>
#include <QListWidgetItem>
#include <QPixmap>
#include <QImage>
#include <QPainter>
#include <QAction>
#include <QPixmap>

#include "VideoModel.h"


/******************************************************************************************************************************/
AnimationPanel::AnimationPanel(QWidget* parent) : QDockWidget(parent), currentVideoModel(NULL), renderer(NULL)
{
	setupUi(this);
	QAction *deleteFramesAction =new QAction("Delete frame", lstKeyFrames);
	QAction *changeTimeAction =new QAction("Change time...", lstKeyFrames);
	lstKeyFrames->addAction(deleteFramesAction);
	lstKeyFrames->addAction(changeTimeAction);
	lstKeyFrames->setContextMenuPolicy(Qt::ActionsContextMenu);
}

/******************************************************************************************************************************/
void AnimationPanel::onFrameSwitched(MeshModel* model)
{
	/* we will emit this, the time slider will emit this, and on load we will switch to frame 0 because main window will emit this*/
	if (!currentVideoModel)
		return;
}

/******************************************************************************************************************************/
void AnimationPanel::onVideoModelLoaded(VideoModel* model)
{
	currentVideoModel = model;
	lstKeyFrames->clear();

	if (currentVideoModel) {
		/* TODO: make generic */
		VideoKeyFrame* firstFrame = currentVideoModel->keyframe(0);

		/* This code is gross... yuck and I wrote it*/
		QImage im = renderer->renderThumbnail(firstFrame);
		QPixmap p = QPixmap::fromImage(im);

		QPainter* painter = new QPainter(&p);
		painter->drawText(im.rect(), Qt::AlignBottom | Qt::AlignCenter, QString("00:00:00"));

		QListWidgetItem *listItem;
		listItem = new QListWidgetItem;
		listItem->setIcon(QIcon(p));
		listItem->setTextAlignment(Qt::AlignBottom|Qt::AlignVCenter);
		lstKeyFrames->addItem(listItem);
		delete painter;

		/* TODO: use resources */
		QPixmap* p2 = new QPixmap("/home/maxim/2.png");
		listItem = new QListWidgetItem;
		listItem->setIcon(QIcon(*p2));
		lstKeyFrames->insertItem(10,listItem);

	}
}
/******************************************************************************************************************************/
void AnimationPanel::onFrameEdited(KVFModel* model)
{
	if (!currentVideoModel) return;

	VideoKeyFrame* firstFrame = currentVideoModel->keyframe(0);

	/* This code is gross... yuck and I wrote it*/
	QImage im = renderer->renderThumbnail(firstFrame);
	QPixmap p = QPixmap::fromImage(im);

	QPainter painter(&p);
	painter.drawText(im.rect(), Qt::AlignBottom | Qt::AlignCenter, QString("00:00:00"));

	QListWidgetItem *listItem = lstKeyFrames->item(0);
	listItem->setIcon(QIcon(p));
	lstKeyFrames->repaint();
}
