#include <QtGui>
#include <QGLWidget>

#include "MainScene.h"
#include "MainWindow.h"

int main(int argc, char **argv)
{
    QApplication app(argc, argv);
    MainWindow window;
    window.show();
    window.resize(1024, 768);
    return app.exec();
}
