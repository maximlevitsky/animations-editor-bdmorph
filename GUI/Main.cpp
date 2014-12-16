#include <QtGui>

#include "MainWindow.h"
#include "cholmod_common.h"
#include "cholmod_matrix.h"
#include <assert.h>


#ifdef _WIN32
int CALLBACK WinMain(
  _In_  HINSTANCE hInstance,
  _In_  HINSTANCE hPrevInstance,
  _In_  LPSTR lpCmdLine,
  _In_  int nCmdShow
)
{
	int argc = 0;
	cholmod_initialize();
	QApplication app(argc, NULL);
    MainWindow *window = new MainWindow();
    window->show();
    app.exec();
    delete window;
    cholmod_finalize();
}
#else
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
#endif
