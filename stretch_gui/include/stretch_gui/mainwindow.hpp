#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <ros/ros.h>

#include <QGraphicsScene>
#include <QMainWindow>
#include <QThread>
#include <iostream>

#include "mapsubscriber.hpp"
#include "mapviewer.hpp"

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
    Q_OBJECT

   public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

   private:
    Ui::MainWindow *ui;
    ros::NodeHandle nh_;
    mapsubscriber *mapSub;
    QThread p_mapSub_node_thread_;

   private slots:
    void on_ButtonGrasp_clicked();
    void on_ButtonBack_clicked();
};
#endif  // MAINWINDOW_H
