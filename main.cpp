#include "mainwindow.h"
#include <QApplication>
#include "gamepadmonitor.h"

MainWindow* window;
//GamepadMonitor* monitor;

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    window = new MainWindow();
    window->show();
    GamepadMonitor monitor;
    monitor.associateWith(window);
    return app.exec();
}
