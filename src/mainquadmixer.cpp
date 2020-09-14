#include <iostream>

#include <QApplication>
#include "interface/quadmixerwindow.h"

#include <GL/glut.h>

int main(int argc, char *argv[])
{
    glutInit(&argc,argv);

    QApplication app(argc, argv);
    setlocale(LC_NUMERIC,"C");
    QuadMixerWindow qbw;
    qbw.showMaximized();
    return app.exec();
}
