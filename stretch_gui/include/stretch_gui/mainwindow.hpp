#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <ros/ros.h>

#include <QGraphicsScene>
#include <QMainWindow>
#include <QThread>
#include <iostream>

#include "mapsubscriber.hpp"
#include "sceneviewer.hpp"
#include "roscamera.hpp"
#include "graspnode.hpp"

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
    MapSubscriber *mapSub_;
    RosCamera *cameraSub_;
    GraspNode *graspNode_;

   private slots:
    void changeToPage2();
    void changeToPage1();
    void changeToPage3();

};
#endif  // MAINWINDOW_H