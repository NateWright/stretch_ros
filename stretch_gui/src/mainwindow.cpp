#include "mainwindow.hpp"

#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow) {
    ui->setupUi(this);
    ui->PagesStackedWidget->setCurrentWidget(ui->page_1);

    qRegisterMetaType<geometry_msgs::PointStamped::ConstPtr>();

    // Initialize all Nodes

    nh_ = ros::NodeHandle("stretch_gui");
    mapSub_ = new MapSubscriber(&nh_);
    moveBaseStatusNode_ = new MoveBaseStatus(&nh_);
    cameraSub_ = new RosCamera(&nh_);
    graspNode_ = new GraspNode(&nh_);
    moveItNode_ = new StretchMoveItInterface(&nh_);

    // Page 1

    connect(ui->ButtonGrasp, &QPushButton::clicked, this, &MainWindow::changeToPage2, Qt::QueuedConnection);
    connect(ui->ButtonStop, &QPushButton::clicked, moveBaseStatusNode_, &MoveBaseStatus::stopRobot, Qt::QueuedConnection);

    connect(ui->DisplayMap, &SceneViewer::mouseClick, mapSub_, &MapSubscriber::moveRobot, Qt::QueuedConnection);
    connect(ui->DisplayMap, &SceneViewer::mousePressInitiated, mapSub_, &MapSubscriber::mousePressInitiated, Qt::QueuedConnection);
    connect(ui->DisplayMap, &SceneViewer::mousePressCurrentLocation, mapSub_, &MapSubscriber::mousePressCurrentLocation, Qt::QueuedConnection);

    connect(mapSub_, &MapSubscriber::mapUpdate, ui->DisplayMap, &SceneViewer::setPixmap, Qt::QueuedConnection);

    connect(moveBaseStatusNode_, &MoveBaseStatus::robotMoving, ui->PleaseWait, &QTextBrowser::setVisible, Qt::QueuedConnection);

    // Page 2

    ui->DisplayCamera->setScaledContents(false);

    connect(ui->ButtonBack, &QPushButton::clicked, this, &MainWindow::changeToPage1, Qt::QueuedConnection);

    connect(ui->CameraMoveButtonUp, &QPushButton::clicked, moveItNode_, &StretchMoveItInterface::headUp, Qt::QueuedConnection);
    connect(ui->CameraMoveButtonDown, &QPushButton::clicked, moveItNode_, &StretchMoveItInterface::headDown, Qt::QueuedConnection);
    connect(ui->CameraMoveButtonLeft, &QPushButton::clicked, moveItNode_, &StretchMoveItInterface::headLeft, Qt::QueuedConnection);
    connect(ui->CameraMoveButtonRight, &QPushButton::clicked, moveItNode_, &StretchMoveItInterface::headRight, Qt::QueuedConnection);
    connect(ui->CameraMoveButtonHome, &QPushButton::clicked, moveItNode_, &StretchMoveItInterface::headHome, Qt::QueuedConnection);

    // Find point in Camera
    connect(ui->DisplayCamera, &SceneViewer::mouseClick, cameraSub_, &RosCamera::sceneClicked, Qt::QueuedConnection);
    connect(ui->DisplayCamera, &SceneViewer::mouseClick, ui->PointPleaseWait, &QTextBrowser::show, Qt::QueuedConnection);
    connect(ui->DisplayCamera, &SceneViewer::mouseClick, ui->ErrorNanPoint, &QTextBrowser::hide, Qt::QueuedConnection);
    connect(ui->DisplayCamera, &SceneViewer::mouseClick, ui->ErrorOutOfRange, &QTextBrowser::hide, Qt::QueuedConnection);

    // Camera feed
    connect(cameraSub_, &RosCamera::imgUpdate, ui->DisplayCamera, &SceneViewer::setPixmap, Qt::QueuedConnection);
    // Error: Displays if NaN point was selected
    connect(cameraSub_, &RosCamera::clickFailure, ui->ErrorNanPoint, &QTextBrowser::show, Qt::QueuedConnection);
    connect(cameraSub_, &RosCamera::clickFailure, ui->PointPleaseWait, &QTextBrowser::hide, Qt::QueuedConnection);

    connect(cameraSub_, &RosCamera::checkPointInRange, mapSub_, &MapSubscriber::checkPointInRange, Qt::QueuedConnection);

    // True
    connect(mapSub_, &MapSubscriber::validPoint, this, &MainWindow::changeToPage3, Qt::QueuedConnection);
    // False
    connect(mapSub_, &MapSubscriber::invalidPoint, ui->ErrorOutOfRange, &QTextBrowser::show, Qt::QueuedConnection);
    connect(mapSub_, &MapSubscriber::invalidPoint, ui->PointPleaseWait, &QTextBrowser::hide, Qt::QueuedConnection);

    // Page 3

    ui->DisplayImage->setScaledContents(false);

    connect(ui->ConfirmButtonNo, &QPushButton::clicked, this, &MainWindow::changeToPage2, Qt::QueuedConnection);
    connect(ui->ConfirmButtonNo, &QPushButton::clicked, cameraSub_, &RosCamera::hideCenterPoint, Qt::QueuedConnection);
    connect(ui->ConfirmButtonYes, &QPushButton::clicked, this, &MainWindow::changeToPage4, Qt::QueuedConnection);
    connect(ui->ConfirmButtonYes, &QPushButton::clicked, cameraSub_, &RosCamera::hideCenterPoint, Qt::QueuedConnection);
    connect(ui->ConfirmButtonYes, &QPushButton::clicked, graspNode_, &GraspNode::doLineUp, Qt::QueuedConnection);

    connect(cameraSub_, &RosCamera::imgUpdateWithPoint, ui->DisplayImage, &SceneViewer::setPixmap, Qt::QueuedConnection);

    // Page 4

    ui->DisplayGrasp->setScaledContents(false);

    connect(ui->ButtonBack_2, &QPushButton::clicked, graspNode_, &GraspNode::doHomeRobot, Qt::QueuedConnection);
    connect(ui->ButtonBack_2, &QPushButton::clicked, this, &MainWindow::changeToPage3, Qt::QueuedConnection);
    connect(ui->ButtonBack_2, &QPushButton::clicked, cameraSub_, &RosCamera::showCenterPoint, Qt::QueuedConnection);

    connect(cameraSub_, &RosCamera::imgUpdate, ui->DisplayGrasp, &SceneViewer::setPixmap, Qt::QueuedConnection);

    connect(graspNode_, &GraspNode::headSetRotation, moveItNode_, &StretchMoveItInterface::headSetRotation, Qt::QueuedConnection);
    connect(graspNode_, &GraspNode::headSetPan, moveItNode_, &StretchMoveItInterface::headSetPan, Qt::QueuedConnection);
    connect(graspNode_, &GraspNode::headSetTilt, moveItNode_, &StretchMoveItInterface::headSetTilt, Qt::QueuedConnection);
    connect(graspNode_, &GraspNode::armSetHeight, moveItNode_, &StretchMoveItInterface::armSetHeight, Qt::QueuedConnection);
    connect(graspNode_, &GraspNode::gripperSetRotate, moveItNode_, &StretchMoveItInterface::gripperSetRotate, Qt::QueuedConnection);
    connect(graspNode_, &GraspNode::gripperSetGrip, moveItNode_, &StretchMoveItInterface::gripperSetGrip, Qt::QueuedConnection);


    // Start Threads

    mapSub_->start();
    moveBaseStatusNode_->start();
    cameraSub_->start();
    graspNode_->start();
    moveItNode_->start();
}

MainWindow::~MainWindow() {
    mapSub_->requestInterruption();
    moveBaseStatusNode_->requestInterruption();
    cameraSub_->requestInterruption();
    graspNode_->requestInterruption();
    moveItNode_->requestInterruption();
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
