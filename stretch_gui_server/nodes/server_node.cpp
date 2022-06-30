#include <QApplication>
#include <csignal>

#include "Server.hpp"

QApplication *app;

void closeApplication(int signal) {
    if (app) {
        app->quit();
    }
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "stretch_gui_server");
    app = new QApplication(argc, argv);
    Server server;

    // QRemoteObjectHost srcNode(QUrl(QStringLiteral("local:switch")));
    QRemoteObjectHost srcNode(QUrl(QStringLiteral("tcp://192.168.86.157:9999")));
    srcNode.enableRemoting(&server);
    qDebug() << "start";

    signal(SIGINT, closeApplication);

    return app->exec();
}
