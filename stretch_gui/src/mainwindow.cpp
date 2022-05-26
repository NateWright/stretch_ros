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

    // Find point in Camera
    connect(ui->DisplayCamera, &SceneViewer::mouseClick, cameraSub_, &RosCamera::sceneClicked);
    connect(ui->DisplayCamera, &SceneViewer::mouseClick, ui->PointPleaseWait, &QTextBrowser::show);
    connect(ui->DisplayCamera, &SceneViewer::mouseClick, ui->ErrorNanPoint, &QTextBrowser::hide);
    connect(ui->DisplayCamera, &SceneViewer::mouseClick, ui->ErrorOutOfRange, &QTextBrowser::hide);

    // Camera feed
    connect(cameraSub_, &RosCamera::imgUpdate, ui->DisplayCamera, &SceneViewer::setPixmap);
    // Error: Displays if NaN point was selected
    connect(cameraSub_, &RosCamera::clickFailure, ui->ErrorNanPoint, &QTextBrowser::show);
    connect(cameraSub_, &RosCamera::clickFailure, ui->PointPleaseWait, &QTextBrowser::hide);
    // Sends center point to grasp node
    connect(cameraSub_, &RosCamera::objectCenterPixel, graspNode_, &GraspNode::setPoint);
    connect(graspNode_, &GraspNode::checkPointInRange, mapSub_, &MapSubscriber::checkPointInRange);

    // Returns true if point is valid
    connect(mapSub_, &MapSubscriber::pointInRange, graspNode_, &GraspNode::checkPointReturn);
    connect(graspNode_, &GraspNode::validPoint, ui->PointPleaseWait, &QTextBrowser::hide);
    connect(graspNode_, &GraspNode::invalidPoint, ui->PointPleaseWait, &QTextBrowser::hide);
    // True
    connect(graspNode_, &GraspNode::validPoint, this, &MainWindow::changeToPage3);
    // False
    connect(graspNode_, &GraspNode::invalidPoint, ui->ErrorOutOfRange, &QTextBrowser::show);

    // Page 3

    ui->DisplayImage->setScaledContents(false);

    connect(ui->ConfirmButtonNo, &QPushButton::clicked, this, &MainWindow::changeToPage2);
    connect(ui->ConfirmButtonNo, &QPushButton::clicked, graspNode_, &GraspNode::reset);
    connect(ui->ConfirmButtonYes, &QPushButton::clicked, this, &MainWindow::changeToPage4);
//    connect(ui->ConfirmButtonYes, &QPushButton::clicked, graspNode_, &GraspNode::lineUp);
    connect(ui->ConfirmButtonYes, &QPushButton::clicked, cameraSub_, &RosCamera::lookAtArm);


    connect(cameraSub_, &RosCamera::imgUpdate, graspNode_, &GraspNode::setImage);

    connect(graspNode_, &GraspNode::imgUpdate, ui->DisplayImage, &SceneViewer::setPixmap);

    // Page 4

    ui->DisplayGrasp->setScaledContents(false);

    connect(ui->ButtonBack_2, &QPushButton::clicked, cameraSub_, &RosCamera::moveHome);
    connect(ui->ButtonBack_2, &QPushButton::clicked, this, &MainWindow::changeToPage3);

    connect(graspNode_, &GraspNode::imgUpdate, ui->DisplayGrasp, &SceneViewer::setPixmap);


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
  ui->ErrorNanPoint->setVisible(false);
  ui->ErrorOutOfRange->setVisible(false);
  ui->PointPleaseWait->setVisible(false);
  ui->PagesStackedWidget->setCurrentWidget(ui->page_2);
}

void MainWindow::changeToPage3(){
  ui->PagesStackedWidget->setCurrentWidget(ui->page_3);
}

void MainWindow::changeToPage4(){
  ui->PagesStackedWidget->setCurrentWidget(ui->page_4);
}
