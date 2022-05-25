#include "mainwindow.hpp"

#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow) {
    ui->setupUi(this);
    ui->PagesStackedWidget->setCurrentWidget(ui->page_1);

    // Initialize all Nodes

    nh_ = ros::NodeHandle("stretch_gui");
    mapSub_ = new MapSubscriber(&nh_);
    moveBaseStatusNode_ = new MoveBaseStatus(&nh_);
    cameraSub_ = new RosCamera(&nh_);
    graspNode_ = new GraspNode(&nh_);

    // Page 1

    connect(ui->ButtonGrasp, &QPushButton::clicked, this, &MainWindow::changeToPage2);
    connect(ui->ButtonStop, &QPushButton::clicked, moveBaseStatusNode_, &MoveBaseStatus::stopRobot);

    connect(ui->DisplayMap, &SceneViewer::mouseClick, mapSub_, &MapSubscriber::moveRobot);
    connect(ui->DisplayMap, &SceneViewer::mousePressInitiated, mapSub_, &MapSubscriber::mousePressInitiated);
    connect(ui->DisplayMap, &SceneViewer::mousePressCurrentLocation, mapSub_, &MapSubscriber::mousePressCurrentLocation);

    connect(mapSub_, &MapSubscriber::mapUpdate, ui->DisplayMap, &SceneViewer::setPixmap);

    connect(moveBaseStatusNode_, &MoveBaseStatus::robotMoving, ui->PleaseWait, &QTextBrowser::setVisible);

    // Page 2

    ui->DisplayCamera->setScaledContents(false);

    connect(ui->ButtonBack, &QPushButton::clicked, this, &MainWindow::changeToPage1);

    connect(ui->CameraMoveButtonUp, &QPushButton::clicked, cameraSub_, &RosCamera::moveUp);
    connect(ui->CameraMoveButtonDown, &QPushButton::clicked, cameraSub_, &RosCamera::moveDown);
    connect(ui->CameraMoveButtonLeft, &QPushButton::clicked, cameraSub_, &RosCamera::moveLeft);
    connect(ui->CameraMoveButtonRight, &QPushButton::clicked, cameraSub_, &RosCamera::moveRight);
    connect(ui->CameraMoveButtonHome, &QPushButton::clicked, cameraSub_, &RosCamera::moveHome);

    connect(ui->DisplayCamera, &SceneViewer::mouseClick, cameraSub_, &RosCamera::sceneClicked);

    connect(cameraSub_, &RosCamera::imgUpdate, ui->DisplayCamera, &SceneViewer::setPixmap);
    connect(cameraSub_, &RosCamera::clickSuccess, this, &MainWindow::changeToPage3);
    connect(cameraSub_, &RosCamera::clickFailure, ui->CameraError, &QTextBrowser::setVisible);

    // Page 3

    ui->DisplayImage->setScaledContents(false);

    connect(ui->ConfirmButtonNo, &QPushButton::clicked, this, &MainWindow::changeToPage2);
    connect(ui->ConfirmButtonNo, &QPushButton::clicked, graspNode_, &GraspNode::reset);
    connect(ui->ConfirmButtonYes, &QPushButton::clicked, this, &MainWindow::changeToPage4);

    connect(cameraSub_, &RosCamera::imgUpdate, graspNode_, &GraspNode::setImage);
    connect(cameraSub_, &RosCamera::objectCenterPixel, graspNode_, &GraspNode::setPoint);

    connect(graspNode_, &GraspNode::displayWaitMessage, ui->ConfirmPleaseWait, &QTextBrowser::setVisible);
    connect(graspNode_, &GraspNode::imgUpdate, ui->DisplayImage, &SceneViewer::setPixmap);

    // Page 4

    ui->DisplayGrasp->setScaledContents(false);

    connect(ui->ButtonMoveStretch, &QPushButton::clicked, graspNode_, &GraspNode::navigate);
    connect(ui->ButtonLineUp, &QPushButton::clicked, graspNode_, &GraspNode::lineUp);

    connect(mapSub_, &MapSubscriber::mapUpdate, ui->DisplayMap_2, &SceneViewer::setPixmap);

    connect(graspNode_, &GraspNode::imgUpdate, ui->DisplayGrasp, &SceneViewer::setPixmap);
    connect(graspNode_, &GraspNode::navigateToPoint, mapSub_, &MapSubscriber::navigateToPoint);


    // Start Threads

    mapSub_->start();
    moveBaseStatusNode_->start();
    cameraSub_->start();
    graspNode_->start();
}

MainWindow::~MainWindow() {
    mapSub_->requestInterruption();
    moveBaseStatusNode_->requestInterruption();
    cameraSub_->requestInterruption();
    graspNode_->requestInterruption();
    mapSub_->wait();
    moveBaseStatusNode_->wait();
    cameraSub_->wait();
    graspNode_->wait();

    delete ui;
    delete mapSub_;
    delete moveBaseStatusNode_;
    delete cameraSub_;
    delete graspNode_;
}

void MainWindow::changeToPage1() {
  ui->PagesStackedWidget->setCurrentWidget(ui->page_1);
  cameraSub_->moveHome();
}

void MainWindow::changeToPage2() {
  ui->CameraError->setVisible(false);
  ui->PagesStackedWidget->setCurrentWidget(ui->page_2);
}

void MainWindow::changeToPage3(){
  ui->ConfirmPleaseWait->setVisible(true);
  ui->PagesStackedWidget->setCurrentWidget(ui->page_3);
}

void MainWindow::changeToPage4(){
  ui->PagesStackedWidget->setCurrentWidget(ui->page_4);
}

