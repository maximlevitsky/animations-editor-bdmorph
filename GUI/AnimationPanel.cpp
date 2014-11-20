
#include "AnimationPanel.h"
#include <QDockWidget>
#include <QListWidgetItem>
#include <QPixmap>
#include <QIcon>
#include <QPainter>
#include <QAction>

AnimationPanel::AnimationPanel(QWidget* parent) : QDockWidget(parent)
{
		setupUi(this);

		QPixmap* p = new QPixmap("/home/maxim/1.png");
		QPixmap p1(128,128);

		QPainter* painter = new QPainter(&p1);

		//     void drawPixmap(const QRectF &targetRect, const QPixmap &pixmap, const QRectF &sourceRect);

		painter->fillRect(p1.rect(), QColor(255,255,255));

		QPixmap pp = p->scaled(QSize(128,110), Qt::KeepAspectRatio, Qt::SmoothTransformation);


		painter->drawPixmap(QRectF(QPointF(0,0), QPointF(128,110)), pp, QRectF(pp.rect()));

		painter->drawText(p1.rect(),
						Qt::AlignBottom | Qt::AlignCenter,
						QString("00:00:00"));

		QListWidgetItem *listItem;
		for (int i = 0 ; i < 2 ; i++)
		{

			listItem = new QListWidgetItem;
			listItem->setIcon(QIcon(p1));
			//listItem->setText("00:00:05");
			listItem->setTextAlignment(Qt::AlignBottom|Qt::AlignVCenter);
			lstKeyFrames->addItem(listItem);
		}


		QPixmap* p2 = new QPixmap("/home/maxim/2.png");
		listItem = new QListWidgetItem;
		listItem->setIcon(QIcon(*p2));
		//listItem->setText("00:00:05");
		listItem->setTextAlignment(Qt::AlignBottom|Qt::AlignVCenter);
		lstKeyFrames->insertItem(10,listItem);

		QAction *deleteFramesAction =new QAction("Delete frame", lstKeyFrames);
		QAction *changeTimeAction =new QAction("Change time...", lstKeyFrames);

		lstKeyFrames->addAction(deleteFramesAction);
		lstKeyFrames->addAction(changeTimeAction);
		lstKeyFrames->setContextMenuPolicy(Qt::ActionsContextMenu);
		delete painter;
}

