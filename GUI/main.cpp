#include <QtGui>
#include <QGLWidget>

#include "deformationscene.h"
#include "graphics_view.h"

int main(int argc, char **argv)
{
    QApplication app(argc, argv);

    QGLWidget *qgl = new QGLWidget(QGLFormat(QGL::SampleBuffers));
    qgl->setAttribute(Qt::WA_AcceptTouchEvents);

    DeformationScene *scene = new DeformationScene(qgl);

    GraphicsView view(scene);
    view.setViewport(qgl);
    view.setViewportUpdateMode(QGraphicsView::FullViewportUpdate);
    view.setScene(scene);
    view.show();
    view.resize(1024, 768);
    return app.exec();
}
