
#include <QDockWidget>
#include "SidePanel.h"

SidePanel::SidePanel(QWidget* parent) : QDockWidget(parent)
{
		setupUi(this);
}
/******************************************************************************************************************************/

void SidePanel::onAnimationStarted()
{
	this->setEnabled(false);
}
/******************************************************************************************************************************/

void SidePanel::onAnimationStopped()
{
	this->setEnabled(true);
}
