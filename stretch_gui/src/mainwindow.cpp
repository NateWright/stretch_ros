#include "mainwindow.hpp"

#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow) {
    ui->setupUi(this);
    ui->PagesStackedWidget->setCurrentWidget(ui->page_1);

    qRegisterMetaType<geometry_msgs::PointStamped::ConstPtr>();
    qRegisterMetaType<geometry_msgs::PoseStamped::Ptr>();

    // Initialize all Nodes

    nh_.reset(new ros::NodeHandle("stretch_gui"));
    mapSub_ = new MapSubscriber(nh_);
    moveBaseStatusNode_ = new MoveBaseStatus(nh_);
    cameraSub_ = new RosCamera(nh_);
    graspNode_ = new GraspNode(nh_);
    moveItNode_ = new StretchMoveItInterface(nh_);

    connect(this, &MainWindow::enableMapping, mapSub_, &MapSubscriber::enableMapping);
    connect(this, &MainWindow::disableMapping, mapSub_, &MapSubscriber::disableMapping);

    // Page 1

    ui->ButtonNavigateHome->setEnabled(false);

    ui->ButtonBackToGrasp->setEnabled(false);
    ui->ButtonBackToGrasp->hide();
    connect(ui->ButtonGrasp, &QPushButton::clicked, this, &MainWindow::changeToPage2);
    connect(ui->ButtonGrasp, &QPushButton::clicked, mapSub_, &MapSubscriber::setHomeIfNone);
    connect(ui->ButtonStop, &QPushButton::clicked, moveBaseStatusNode_, &MoveBaseStatus::stopRobot);
    connect(ui->ButtonSetHome, &QPushButton::clicked, mapSub_, &MapSubscriber::setHome);
    connect(ui->ButtonNavigateHome, &QPushButton::clicked, mapSub_, &MapSubscriber::navigateHome);
    connect(ui->ButtonToggleNavType, &QPushButton::clicked, this, &MainWindow::changeToPage6);

    connect(ui->DisplayMap, &SceneViewer::mouseClick, mapSub_, &MapSubscriber::moveRobot);
    connect(ui->DisplayMap, &SceneViewer::mousePressInitiated, mapSub_, &MapSubscriber::mousePressInitiated);
    connect(ui->DisplayMap, &SceneViewer::mousePressCurrentLocation, mapSub_, &MapSubscriber::mousePressCurrentLocation);

    connect(mapSub_, &MapSubscriber::mapUpdate, ui->DisplayMap, &SceneViewer::setPixmap);
    connect(mapSub_, &MapSubscriber::homeSet, ui->ButtonNavigateHome, &QPushButton::setEnabled);

    connect(moveBaseStatusNode_, &MoveBaseStatus::robotMoving, ui->PleaseWait, &QTextBrowser::setVisible);

    // Page 2

    ui->DisplayCamera->setScaledContents(false);

    connect(this, &MainWindow::homeRobot, moveItNode_, &StretchMoveItInterface::homeRobot);
    connect(this, &MainWindow::cameraSetTilt, moveItNode_, &StretchMoveItInterface::headSetTilt);

    connect(ui->ButtonBack, &QPushButton::clicked, this, &MainWindow::changeToPage1);

    connect(ui->CameraMoveButtonUp, &QPushButton::clicked, moveItNode_, &StretchMoveItInterface::headUp);
    connect(ui->CameraMoveButtonDown, &QPushButton::clicked, moveItNode_, &StretchMoveItInterface::headDown);
    connect(ui->CameraMoveButtonLeft, &QPushButton::clicked, moveItNode_, &StretchMoveItInterface::headLeft);
    connect(ui->CameraMoveButtonRight, &QPushButton::clicked, moveItNode_, &StretchMoveItInterface::headRight);
    connect(ui->CameraMoveButtonHome, &QPushButton::clicked, moveItNode_, &StretchMoveItInterface::headHome);

    // Find point in Camera
    connect(ui->DisplayCamera, &SceneViewer::mouseClick, cameraSub_, &RosCamera::sceneClicked);
    connect(ui->DisplayCamera, &SceneViewer::mouseClick, ui->ErrorNanPoint, &QTextBrowser::hide);
    connect(ui->DisplayCamera, &SceneViewer::mouseClick, ui->ErrorOutOfRange, &QTextBrowser::hide);

    connect(cameraSub_, &RosCamera::clickInitiated, ui->PointPleaseWait, &QTextBrowser::show);

    // Camera feed
    connect(cameraSub_, &RosCamera::imgUpdate, ui->DisplayCamera, &SceneViewer::setCamera);
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
    connect(ui->ConfirmButtonYes, &QPushButton::clicked, this, &MainWindow::changeToPage4);
    connect(ui->ConfirmButtonYes, &QPushButton::clicked, graspNode_, &GraspNode::lineUp);

    connect(cameraSub_, &RosCamera::imgUpdateWithPoint, ui->DisplayImage, &SceneViewer::setCamera);

    // Page 4

    ui->DisplayGrasp->setScaledContents(false);

    connect(ui->ButtonBack_2, &QPushButton::clicked, graspNode_, &GraspNode::home);
    connect(ui->ButtonBack_2, &QPushButton::clicked, this, &MainWindow::changeToPage2);
    connect(ui->ButtonReturnObject, &QPushButton::clicked, graspNode_, &GraspNode::returnObject);
    connect(ui->ButtonRelease, &QPushButton::clicked, graspNode_, &GraspNode::releaseObject);
    connect(ui->ButtonReplaceObject, &QPushButton::clicked, graspNode_, &GraspNode::replaceObject);
    connect(ui->ButtonNavigate, &QPushButton::clicked, this, &MainWindow::changeToPage5);

    connect(cameraSub_, &RosCamera::imgUpdate, ui->DisplayGrasp, &SceneViewer::setCamera);

    connect(graspNode_, &GraspNode::headSetRotation, moveItNode_, &StretchMoveItInterface::headSetRotation, Qt::BlockingQueuedConnection);
    connect(graspNode_, &GraspNode::headSetPan, moveItNode_, &StretchMoveItInterface::headSetPan, Qt::BlockingQueuedConnection);
    connect(graspNode_, &GraspNode::headSetTilt, moveItNode_, &StretchMoveItInterface::headSetTilt, Qt::BlockingQueuedConnection);
    connect(graspNode_, &GraspNode::armSetHeight, moveItNode_, &StretchMoveItInterface::armSetHeight, Qt::BlockingQueuedConnection);
    connect(graspNode_, &GraspNode::armSetReach, moveItNode_, &StretchMoveItInterface::armSetReach, Qt::BlockingQueuedConnection);
    connect(graspNode_, &GraspNode::gripperSetRotate, moveItNode_, &StretchMoveItInterface::gripperSetRotate, Qt::BlockingQueuedConnection);
    connect(graspNode_, &GraspNode::gripperSetGrip, moveItNode_, &StretchMoveItInterface::gripperSetGrip, Qt::BlockingQueuedConnection);
    connect(graspNode_, &GraspNode::enableMapping, mapSub_, &MapSubscriber::enableMapping, Qt::BlockingQueuedConnection);
    connect(graspNode_, &GraspNode::disableMapping, mapSub_, &MapSubscriber::disableMapping, Qt::BlockingQueuedConnection);
    connect(graspNode_, &GraspNode::homeRobot, moveItNode_, &StretchMoveItInterface::homeRobot);
    connect(graspNode_, &GraspNode::navigate, mapSub_, &MapSubscriber::moveRobotLoc);
    connect(graspNode_, &GraspNode::navigateHome, mapSub_, &MapSubscriber::navigateHome);
    connect(graspNode_, &GraspNode::graspDone, ui->ButtonReturnObject, &QPushButton::setEnabled);
    connect(graspNode_, &GraspNode::turnLeft, mapSub_, &MapSubscriber::rotateLeft);

    // Page 5

    connect(ui->ButtonBackToGrasp, &QPushButton::clicked, this, &MainWindow::changeToPage4);

    // Page 6

    connect(ui->ButtonToggleNavType_2, &QPushButton::clicked, this, &MainWindow::changeToPage1);
    connect(ui->ButtonTurnLeft, &QPushButton::clicked, [=]() { mapSub_->rotateLeft(); });
    connect(ui->ButtonTurnRight, &QPushButton::clicked, [=]() { mapSub_->rotateRight(); });
    connect(ui->ButtonForward, &QPushButton::clicked, [=]() { mapSub_->drive(0.15); });
    //    connect(ui->ButtonBackwards, &QPushButton::clicked, [=](){mapSub_->drive(-0.2);});

    connect(cameraSub_, &RosCamera::imgUpdate, ui->DisplayCamera_2, &SceneViewer::setCamera);

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
    ui->ButtonGrasp->show();
    ui->ButtonGrasp->setEnabled(true);
    ui->ButtonBackToGrasp->hide();
    ui->ButtonBackToGrasp->setEnabled(false);
    emit homeRobot();
    emit enableMapping();
}

void MainWindow::changeToPage2() {
    emit disableMapping();
    emit cameraSetTilt(-30);
    ui->ErrorNanPoint->setVisible(false);
    ui->ErrorOutOfRange->setVisible(false);
    ui->PointPleaseWait->setVisible(false);
    ui->PagesStackedWidget->setCurrentWidget(ui->page_2);
}

void MainWindow::changeToPage3() {
    emit disableMapping();
    ui->PagesStackedWidget->setCurrentWidget(ui->page_3);
}

void MainWindow::changeToPage4() {
    emit disableMapping();
    ui->ButtonReturnObject->setDisabled(true);
    ui->PagesStackedWidget->setCurrentWidget(ui->page_4);
}

void MainWindow::changeToPage5() {
    emit enableMapping();
    ui->ButtonGrasp->hide();
    ui->ButtonGrasp->setEnabled(false);
    ui->ButtonBackToGrasp->show();
    ui->ButtonBackToGrasp->setEnabled(true);
    ui->PagesStackedWidget->setCurrentWidget(ui->page_1);
}

void MainWindow::changeToPage6() {
    emit enableMapping();
    ui->PagesStackedWidget->setCurrentWidget(ui->page_6);
}

void MainWindow::showButtonNavigateHome() {
    ui->ButtonNavigateHome->setEnabled(true);
}