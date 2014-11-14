
#include <qdialog.h>
#include <QDockWidget>
#include "ui_SidePanel.h"

class SidePanel : public QDockWidget, public Ui_sidePanel
{
	Q_OBJECT
public:
	SidePanel(QWidget* parent);
};
