#include <QApplication>
#include <QPixmap>

#include "Server.hpp"

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "stretch_gui_server");
    QApplication app(argc, argv);
    Server server;

    QRemoteObjectHost srcNode(QUrl(QStringLiteral("local:switch")));
    srcNode.enableRemoting(&server);
    qDebug() << "start";

    return app.exec();
}
