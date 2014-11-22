#include <QtGui>

#include "MainScene.h"
#include "MainWindow.h"
#include "cholmod_matrix.h"

int main(int argc, char **argv)
{
	cholmod_initialize();

	QApplication app(argc, argv);

    MainWindow *window = new MainWindow();
    window->show();
    //window->resize(1024, 768);
    app.exec();

    delete window;
    cholmod_finalize();
}
