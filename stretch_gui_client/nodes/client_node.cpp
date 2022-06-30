#include <QApplication>

#include "Client.hpp"

int main(int argc, char *argv[]) {
    QApplication a(argc, argv);

    QSharedPointer<ServerReplica> ptr;  // shared pointer to hold source replica

    QRemoteObjectNode repNode;                                                 // create remote object node
    repNode.connectToNode(QUrl(QStringLiteral("tcp://192.168.86.157:9999")));  // connect with remote host node
    // repNode.connectToNode(QUrl(QStringLiteral("local:switch")));
    ptr.reset(repNode.acquire<ServerReplica>());  // acquire replica of source from host node

    Client c(ptr);
    c.show();
    return a.exec();
}
