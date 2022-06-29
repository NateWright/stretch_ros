#ifndef CLIENT_HPP
#define CLIENT_HPP

#include <QDebug>
#include <QGraphicsScene>
#include <QImage>
#include <QMainWindow>
#include <QPixmap>
#include <QTcpSocket>
#include <iostream>
#include <stretch_gui_library/sceneviewer.hpp>

#include "rep_Server_replica.h"

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}  // namespace Ui
QT_END_NAMESPACE

class Client : public QMainWindow {
    Q_OBJECT
   public:
    explicit Client(QSharedPointer<ServerReplica> ptr, QWidget *parent = nullptr);
    void initConnections();

   private:
    Ui::MainWindow *ui;
    QSharedPointer<ServerReplica> server_;

    QMetaObject::Connection DisplayFeedOne_,
        DisplayFeedTwo_;

   signals:

   private slots:
    void changeToPage1();
    void changeToPage2();
    void changeToPage3();
    void changeToPage4();
    void changeToPage5();
    void changeToPage6();
    void showButtonNavigateHome();
};

#endif  // CLIENT_HPP
