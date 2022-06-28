#include <QApplication>

#include "mainwindow.hpp"

Q_DECLARE_METATYPE(geometry_msgs::PointStamped::ConstPtr);
Q_DECLARE_METATYPE(geometry_msgs::PoseStamped::Ptr);

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "stretch_gui");
    QApplication a(argc, argv);
    qRegisterMetaType<geometry_msgs::PointStamped::ConstPtr>();
    qRegisterMetaType<geometry_msgs::PoseStamped::Ptr>();
    MainWindow w;
    w.show();
    return a.exec();
}
