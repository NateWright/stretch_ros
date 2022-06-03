#include "mainwindow.hpp"

#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow) {
    ui->setupUi(this);
    ui->PagesStackedWidget->setCurrentWidget(ui->page_1);

    qRegisterMetaType<geometry_msgs::PointStamped::ConstPtr>();
    qRegisterMetaType<geometry_msgs::PoseStamped::Ptr>();

    // Initialize all Nodes

    nh_ = ros::NodeHandle("stretch_gui");
    mapSub_ = new MapSubscriber(&nh_);
    moveBaseStatusNode_ = new MoveBaseStatus(&nh_);
    cameraSub_ = new RosCamera(&nh_);
    graspNode_ = new GraspNode(&nh_);
    moveItNode_ = new StretchMoveItInterface(&nh_);

    // Page 1

    ui->ButtonNavigateHome->setEnabled(false);

    connect(ui->ButtonGrasp, &QPushButton::clicked, this, &MainWindow::changeToPage2);
    connect(ui->ButtonStop, &QPushButton::clicked, moveBaseStatusNode_, &MoveBaseStatus::stopRobot);
    connect(ui->ButtonSetHome, &QPushButton::clicked, mapSub_, &MapSubscriber::setHome);
    connect(ui->ButtonSetHome, &QPushButton::clicked, this, &MainWindow::showButtonNavigateHome);
    connect(ui->ButtonNavigateHome, &QPushButton::clicked, mapSub_, &MapSubscriber::navigateHome);

    connect(ui->DisplayMap, &SceneViewer::mouseClick, mapSub_, &MapSubscriber::moveRobot);
    connect(ui->DisplayMap, &SceneViewer::mousePressInitiated, mapSub_, &MapSubscriber::mousePressInitiated);
    connect(ui->DisplayMap, &SceneViewer::mousePressCurrentLocation, mapSub_, &MapSubscriber::mousePressCurrentLocation);

    connect(mapSub_, &MapSubscriber::mapUpdate, ui->DisplayMap, &SceneViewer::setPixmap);

    connect(moveBaseStatusNode_, &MoveBaseStatus::robotMoving, ui->PleaseWait, &QTextBrowser::setVisible);

    // Page 2

    ui->DisplayCamera->setScaledContents(false);

    connect(ui->ButtonBack, &QPushButton::clicked, this, &MainWindow::changeToPage1);

    connect(ui->CameraMoveButtonUp, &QPushButton::clicked, moveItNode_, &StretchMoveItInterface::headUp);
    connect(ui->CameraMoveButtonDown, &QPushButton::clicked, moveItNode_, &StretchMoveItInterface::headDown);
    connect(ui->CameraMoveButtonLeft, &QPushButton::clicked, moveItNode_, &StretchMoveItInterface::headLeft);
    connect(ui->CameraMoveButtonRight, &QPushButton::clicked, moveItNode_, &StretchMoveItInterface::headRight);
    connect(ui->CameraMoveButtonHome, &QPushButton::clicked, moveItNode_, &StretchMoveItInterface::headHome);

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

    connect(cameraSub_, &RosCamera::checkPointInRange, mapSub_, &MapSubscriber::checkPointInRange);

    // True
    connect(mapSub_, &MapSubscriber::validPoint, this, &MainWindow::changeToPage3);
    // False
    connect(mapSub_, &MapSubscriber::invalidPoint, ui->ErrorOutOfRange, &QTextBrowser::show);
    connect(mapSub_, &MapSubscriber::invalidPoint, ui->PointPleaseWait, &QTextBrowser::hide);

    // Page 3

    ui->DisplayImage->setScaledContents(false);

    connect(ui->ConfirmButtonNo, &QPushButton::clicked, this, &MainWindow::changeToPage2);
    connect(ui->ConfirmButtonNo, &QPushButton::clicked, cameraSub_, &RosCamera::hideCenterPoint);
    connect(ui->ConfirmButtonYes, &QPushButton::clicked, this, &MainWindow::changeToPage4);
    connect(ui->ConfirmButtonYes, &QPushButton::clicked, cameraSub_, &RosCamera::hideCenterPoint);
    connect(ui->ConfirmButtonYes, &QPushButton::clicked, graspNode_, &GraspNode::lineUp);

    connect(cameraSub_, &RosCamera::imgUpdateWithPoint, ui->DisplayImage, &SceneViewer::setPixmap);

    // Page 4

    ui->DisplayGrasp->setScaledContents(false);

    connect(ui->ButtonBack_2, &QPushButton::clicked, graspNode_, &GraspNode::homeRobot);
    connect(ui->ButtonBack_2, &QPushButton::clicked, this, &MainWindow::changeToPage3);
    connect(ui->ButtonBack_2, &QPushButton::clicked, cameraSub_, &RosCamera::showCenterPoint);

    connect(cameraSub_, &RosCamera::imgUpdate, ui->DisplayGrasp, &SceneViewer::setPixmap);

    connect(graspNode_, &GraspNode::headSetRotation, moveItNode_, &StretchMoveItInterface::headSetRotation);
    connect(graspNode_, &GraspNode::headSetPan, moveItNode_, &StretchMoveItInterface::headSetPan);
    connect(graspNode_, &GraspNode::headSetTilt, moveItNode_, &StretchMoveItInterface::headSetTilt);
    connect(graspNode_, &GraspNode::armSetHeight, moveItNode_, &StretchMoveItInterface::armSetHeight);
    connect(graspNode_, &GraspNode::armSetReach, moveItNode_, &StretchMoveItInterface::armSetReach);
    connect(graspNode_, &GraspNode::gripperSetRotate, moveItNode_, &StretchMoveItInterface::gripperSetRotate);
    connect(graspNode_, &GraspNode::gripperSetGrip, moveItNode_, &StretchMoveItInterface::gripperSetGrip);
    connect(graspNode_, &GraspNode::navigate, mapSub_, &MapSubscriber::moveRobotLoc);


    // Start Threads

    mapSub_->start();
    moveBaseStatusNode_->start();
    cameraSub_->start();
    graspNode_->start();
    moveItNode_->start();
}

MainWindow::~MainWindow() {
    mapSub_->quit();
    moveBaseStatusNode_->quit();
    cameraSub_->quit();
    graspNode_->quit();
    moveItNode_->quit();
    mapSub_->wait();
    moveBaseStatusNode_->wait();
    cameraSub_->wait();
    graspNode_->wait();
    moveItNode_->wait();

    delete ui;
    delete mapSub_;
    delete moveBaseStatusNode_;
    delete cameraSub_;
    delete graspNode_;
    delete moveItNode_;
}

void MainWindow::changeToPage1() {
  ui->PagesStackedWidget->setCurrentWidget(ui->page_1);
  moveItNode_->headHome();
}

void MainWindow::changeToPage2() {
  ui->ErrorNanPoint->setVisible(false);
  ui->ErrorOutOfRange->setVisible(false);
  ui->PointPleaseWait->setVisible(false);
  ui->PagesStackedWidget->setCurrentWidget(ui->page_2);
}

void MainWindow::changeToPage3(){
  cameraSub_->showCenterPoint();
  ui->PagesStackedWidget->setCurrentWidget(ui->page_3);
}

void MainWindow::changeToPage4(){
  ui->PagesStackedWidget->setCurrentWidget(ui->page_4);
}

void MainWindow::showButtonNavigateHome(){
  ui->ButtonNavigateHome->setEnabled(true);
}
