#include "mainwindow.hpp"

#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow) {
    ui->setupUi(this);

    nh_ = ros::NodeHandle();

    connect(ui->ButtonGrasp, SIGNAL(clicked()), this, SLOT(changeToPage2()));
    connect(ui->ButtonBack, SIGNAL(clicked()), this, SLOT(changeToPage1()));

    mapSub_ = new MapSubscriber(&nh_);
    connect(mapSub_, &MapSubscriber::mapUpdate, ui->DisplayMap, &SceneViewer::setPixmap);
    connect(ui->DisplayMap, &SceneViewer::sceneClick, mapSub_, &MapSubscriber::moveRobot);
    mapSub_->start();

    cameraSub_ = new RosCamera(&nh_);
    connect(cameraSub_, &RosCamera::imgUpdate, ui->DisplayCamera, &SceneViewer::setPixmap);
    connect(ui->DisplayCamera, &SceneViewer::sceneClick, cameraSub_, &RosCamera::sceneClicked);
    connect(ui->DisplayCamera, &SceneViewer::sceneClick, this, &MainWindow::changeToPage3);
    connect(ui->CameraMoveButtonUp, &QPushButton::clicked, cameraSub_, &RosCamera::moveUp);
    connect(ui->CameraMoveButtonDown, &QPushButton::clicked, cameraSub_, &RosCamera::moveDown);
    connect(ui->CameraMoveButtonLeft, &QPushButton::clicked, cameraSub_, &RosCamera::moveLeft);
    connect(ui->CameraMoveButtonRight, &QPushButton::clicked, cameraSub_, &RosCamera::moveRight);
    connect(ui->CameraMoveButtonHome, &QPushButton::clicked, cameraSub_, &RosCamera::moveHome);
    cameraSub_->start();

    graspNode_ = new GraspNode(&nh_);
    connect(graspNode_, &GraspNode::imgUpdate, ui->DisplayImage, &SceneViewer::setPixmap);
    connect(ui->ConfirmButtonNo, &QPushButton::clicked, this, &MainWindow::changeToPage2);
    connect(ui->ConfirmButtonNo, &QPushButton::clicked, graspNode_, &GraspNode::reset);
    graspNode_->start();


}

MainWindow::~MainWindow() {
    delete ui;
    delete mapSub_;
    delete cameraSub_;
    delete graspNode_;
}

void MainWindow::changeToPage2() {
    ui->PagesStackedWidget->setCurrentWidget(ui->page_2);
}

void MainWindow::changeToPage1() {
    ui->PagesStackedWidget->setCurrentWidget(ui->page_1);
    cameraSub_->moveHome();
}

void MainWindow::changeToPage3(){
  ui->PagesStackedWidget->setCurrentWidget(ui->page_3);
}

