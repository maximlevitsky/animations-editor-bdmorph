#include <QtGui>

#include "MainWindow.h"
#include "cholmod_matrix.h"
#include <assert.h>

int main(int argc, char **argv)
{
	cholmod_initialize();

	QApplication app(argc, argv);

    MainWindow *window = new MainWindow();
    window->show();
    app.exec();

    delete window;
    cholmod_finalize();
}
