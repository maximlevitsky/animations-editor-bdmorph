
#include <qdialog.h>
#include <QDockWidget>
#include "ui_AnimationPanel.h"

class AnimationPanel : public QDockWidget, public Ui_AnimationPanel
{
	Q_OBJECT
public:
	AnimationPanel(QWidget* parent);
};
