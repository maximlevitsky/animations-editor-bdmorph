

#ifndef ACTIONBUTTON_H
#define ACTIONBUTTON_H

#include <QWidget>
#include <QCheckBox>
#include <QPushButton>
#include <QAction>

class ActionCheckBox : public QCheckBox
{
    Q_OBJECT
private:
    QAction* actionOwner;
public:
    ActionCheckBox(QWidget *parent = 0);
    void setAction( QAction* action );
public slots:
    void updateButtonStatusFromAction();
};


class ActionButton : public QPushButton
{
    Q_OBJECT
private:
    QAction* actionOwner;
public:
    ActionButton(QWidget *parent = 0);
    void setAction( QAction* action );
public slots:
    void updateButtonStatusFromAction();
};

#endif
