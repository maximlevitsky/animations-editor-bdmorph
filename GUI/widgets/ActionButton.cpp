
#include "ActionButton.h"


ActionCheckBox::ActionCheckBox(QWidget *parent) : QCheckBox(parent)
{
    actionOwner = NULL;
    setCheckable(true);
}

void ActionCheckBox::setAction(QAction *action)
{
    actionOwner = action;
    updateButtonStatusFromAction();

    connect( action, SIGNAL(triggered()),  this, SLOT(click()));
    connect( this, SIGNAL(clicked()),  this, SLOT(updateButtonStatusFromAction()));
}

void ActionCheckBox::updateButtonStatusFromAction()
{
	actionOwner->setChecked(this->isChecked());
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////

ActionButton::ActionButton(QWidget *parent) : QPushButton(parent)
{
    actionOwner = NULL;
    setCheckable(true);
}

void ActionButton::setAction(QAction *action)
{
    actionOwner = action;
    updateButtonStatusFromAction();

    connect( action, SIGNAL(triggered()),  this, SLOT(click()));
    connect( this, SIGNAL(clicked()),  this, SLOT(updateButtonStatusFromAction()));
}

void ActionButton::updateButtonStatusFromAction()
{
	actionOwner->setChecked(this->isChecked());
}
