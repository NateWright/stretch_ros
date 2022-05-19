#include "mainwindow.hpp"

#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow) {
    ui->setupUi(this);

    nh_ = ros::NodeHandle("stretch_gui");

    connect(ui->ButtonGrasp, SIGNAL(clicked()), this, SLOT(changeToPage2()));
    connect(ui->ButtonBack, SIGNAL(clicked()), this, SLOT(changeToPage1()));

    mapSub_ = new MapSubscriber(&nh_);
    connect(mapSub_, &MapSubscriber::mapUpdate, ui->DisplayMap, &SceneViewer::setPixmap);
    connect(ui->DisplayMap, &SceneViewer::mouseClick, mapSub_, &MapSubscriber::moveRobot);
    connect(ui->DisplayMap, &SceneViewer::mousePressInitiated, mapSub_, &MapSubscriber::mousePressInitiated);
    connect(ui->DisplayMap, &SceneViewer::mousePressCurrentLocation, mapSub_, &MapSubscriber::mousePressCurrentLocation);
    mapSub_->start();

    ui->DisplayCamera->setScaledContents(false);

    cameraSub_ = new RosCamera(&nh_);
    connect(cameraSub_, &RosCamera::imgUpdate, ui->DisplayCamera, &SceneViewer::setPixmap);
    connect(ui->DisplayCamera, &SceneViewer::mouseClick, cameraSub_, &RosCamera::sceneClicked);
    connect(cameraSub_, &RosCamera::clickSuccess, this, &MainWindow::changeToPage3);
    connect(ui->CameraMoveButtonUp, &QPushButton::clicked, cameraSub_, &RosCamera::moveUp);
    connect(ui->CameraMoveButtonDown, &QPushButton::clicked, cameraSub_, &RosCamera::moveDown);
    connect(ui->CameraMoveButtonLeft, &QPushButton::clicked, cameraSub_, &RosCamera::moveLeft);
    connect(ui->CameraMoveButtonRight, &QPushButton::clicked, cameraSub_, &RosCamera::moveRight);
    connect(ui->CameraMoveButtonHome, &QPushButton::clicked, cameraSub_, &RosCamera::moveHome);
    cameraSub_->start();

    ui->DisplayImage->setScaledContents(false);

    graspNode_ = new GraspNode(&nh_);
    connect(graspNode_, &GraspNode::imgUpdate, ui->DisplayImage, &SceneViewer::setPixmap);
    connect(ui->ConfirmButtonNo, &QPushButton::clicked, this, &MainWindow::changeToPage2);
    connect(ui->ConfirmButtonNo, &QPushButton::clicked, graspNode_, &GraspNode::reset);
    connect(cameraSub_, &RosCamera::imgUpdate, graspNode_, &GraspNode::setImage);
    connect(cameraSub_, &RosCamera::objectCenterPixel, graspNode_, &GraspNode::setPoint);
    graspNode_->start();


}

MainWindow::~MainWindow() {
    mapSub_->requestInterruption();
    cameraSub_->requestInterruption();
    graspNode_->requestInterruption();
    mapSub_->wait();
    cameraSub_->wait();
    graspNode_->wait();

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

