#include "mainwindow.hpp"

#include <QApplication>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "stretch_gui");
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    return a.exec();
}
