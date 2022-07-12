#include "Server.hpp"

Server::Server(QObject* parent) : ServerSimpleSource(parent), headPanTilt_({0, -30}) {
    nh_.reset(new ros::NodeHandle("stretch_gui_server"));
    mapNode_ = new MapSubscriber(nh_);
    moveBaseStatusNode_ = new MoveBaseStatus(nh_);
    cameraNode_ = new RosCamera(nh_);
    graspNode_ = new GraspNode(nh_);
    moveItNode_ = new StretchMoveItInterface(nh_);

    initConnections();

    mapNode_->start();
    moveBaseStatusNode_->start();
    cameraNode_->start();
    graspNode_->start();
    moveItNode_->start();
}

void Server::initConnections() {
    // Page 1
    connect(this, &Server::enableMapping, mapNode_, &MapSubscriber::enableMapping);
    connect(this, &Server::disableMapping, mapNode_, &MapSubscriber::disableMapping);

    connect(this, &Server::ButtonGraspClicked, this, &Server::changeToPage2);
    connect(this, &Server::ButtonGraspClicked, mapNode_, &MapSubscriber::setHomeIfNone);
    connect(this, &Server::ButtonStopClicked, moveBaseStatusNode_, &MoveBaseStatus::stopRobot);
    connect(this, &Server::ButtonSetHomeClicked, mapNode_, &MapSubscriber::setHome);
    connect(this, &Server::ButtonNavigateHomeClicked, mapNode_, &MapSubscriber::navigateHome);
    connect(this, &Server::ButtonToggleNavTypeClicked, this, &Server::changeToPage6);

    connect(this, &Server::DisplayMapMouseClick, mapNode_, &MapSubscriber::moveRobot);
    connect(this, &Server::DisplayMapMousePressInitiated, mapNode_, &MapSubscriber::mousePressInitiated);
    connect(this, &Server::DisplayMapMousePressCurrentLocation, mapNode_, &MapSubscriber::mousePressCurrentLocation);

    connect(mapNode_, &MapSubscriber::mapUpdateQImage, this, &Server::newMap);
    connect(mapNode_, &MapSubscriber::homeSet, this, [this](bool b) { emit uiButtonNavigateHomeSetEnabled(b); });

    connect(moveBaseStatusNode_, &MoveBaseStatus::robotMoving, this, [this](bool b) { emit uiPleaseWaitSetVisible(b); });

    // Page 2

    connect(this, &Server::homeRobot, moveItNode_, &StretchMoveItInterface::homeRobot);
    connect(this, &Server::cameraSetRotation, moveItNode_, &StretchMoveItInterface::headSetRotation);

    connect(this, &Server::ButtonBackClicked, this, &Server::changeToPage1);  // Both

    connect(this, &Server::CameraMoveButtonUpClicked, moveItNode_, &StretchMoveItInterface::headUp);        // Client to server
    connect(this, &Server::CameraMoveButtonDownClicked, moveItNode_, &StretchMoveItInterface::headDown);    // Client to server
    connect(this, &Server::CameraMoveButtonLeftClicked, moveItNode_, &StretchMoveItInterface::headLeft);    // Client to server
    connect(this, &Server::CameraMoveButtonRightClicked, moveItNode_, &StretchMoveItInterface::headRight);  // Client to server
    connect(this, &Server::CameraMoveButtonHomeClicked, moveItNode_, &StretchMoveItInterface::headHome);    // Client to server

    // Find point in Camera
    connect(this, &Server::DisplayCameraMouseClicked, cameraNode_, &RosCamera::sceneClicked);

    connect(cameraNode_, &RosCamera::clickInitiated, this, &Server::uiPointPleaseWaitShow);  // Sever to client

    // Camera feed
    // connect(cameraNode_, &RosCamera::imgUpdateQImage, this, &Server::uiDisplayCameraSetCamera);
    // Server to client
    // Error: Displays if NaN point was selected
    connect(cameraNode_, &RosCamera::clickFailure, this, &Server::uiErrorNanPointShow);    // Server to client
    connect(cameraNode_, &RosCamera::clickFailure, this, &Server::uiPointPleaseWaitHide);  // Server to client

    connect(cameraNode_, &RosCamera::checkPointInRange, mapNode_, &MapSubscriber::checkPointInRange);  // Server only

    // True
    connect(mapNode_, &MapSubscriber::validPoint, this, &Server::changeToPage3);  // Both
    connect(mapNode_, &MapSubscriber::validPoint, this, &Server::uiChangeToPage3);
    // False
    connect(mapNode_, &MapSubscriber::invalidPoint, this, &Server::uiErrorOutOfRangeShow);  // Server to client
    connect(mapNode_, &MapSubscriber::invalidPoint, this, &Server::uiPointPleaseWaitHide);  // Server to client

    // Page 3

    connect(this, &Server::ConfirmButtonNoClicked, this, &Server::changeToPage2);     // Client to Both
    connect(this, &Server::ConfirmButtonYesClicked, this, &Server::changeToPage4);    // Client to Both
    connect(this, &Server::ConfirmButtonYesClicked, graspNode_, &GraspNode::lineUp);  // Client to server

    connect(cameraNode_, &RosCamera::imgUpdateWithPointQImage, this, &Server::uiDisplayImageSetCamera);  // Server to ui

    // Page 4

    connect(this, &Server::ButtonBack_2Clicked, graspNode_, &GraspNode::home);                  // Client to Server
    connect(this, &Server::ButtonBack_2Clicked, this, &Server::changeToPage2);                  // Client to Both
    connect(this, &Server::ButtonReturnObjectClicked, graspNode_, &GraspNode::returnObject);    // Client to server
    connect(this, &Server::ButtonReleaseClicked, graspNode_, &GraspNode::releaseObject);        // Client to server
    connect(this, &Server::ButtonReplaceObjectClicked, graspNode_, &GraspNode::replaceObject);  // Client to server
    connect(this, &Server::ButtonNavigateClicked, this, &Server::changeToPage5);                // Client to both

    //  connect(cameraNode_, &RosCamera::imgUpdate, ui->DisplayGrasp, &SceneViewer::setCamera);  // Server to client

    connect(graspNode_, &GraspNode::headSetRotation, moveItNode_, &StretchMoveItInterface::headSetRotation, Qt::BlockingQueuedConnection);    // Server
    connect(graspNode_, &GraspNode::headSetPan, moveItNode_, &StretchMoveItInterface::headSetPan, Qt::BlockingQueuedConnection);              // Server
    connect(graspNode_, &GraspNode::headSetTilt, moveItNode_, &StretchMoveItInterface::headSetTilt, Qt::BlockingQueuedConnection);            // Server
    connect(graspNode_, &GraspNode::armSetHeight, moveItNode_, &StretchMoveItInterface::armSetHeight, Qt::BlockingQueuedConnection);          // Server
    connect(graspNode_, &GraspNode::armSetReach, moveItNode_, &StretchMoveItInterface::armSetReach, Qt::BlockingQueuedConnection);            // Server
    connect(graspNode_, &GraspNode::gripperSetRotate, moveItNode_, &StretchMoveItInterface::gripperSetRotate, Qt::BlockingQueuedConnection);  // Server
    connect(graspNode_, &GraspNode::gripperSetGrip, moveItNode_, &StretchMoveItInterface::gripperSetGrip, Qt::BlockingQueuedConnection);
    connect(graspNode_, &GraspNode::enableMapping, mapNode_, &MapSubscriber::enableMapping, Qt::BlockingQueuedConnection);
    connect(graspNode_, &GraspNode::disableMapping, mapNode_, &MapSubscriber::disableMapping, Qt::BlockingQueuedConnection);
    connect(graspNode_, &GraspNode::homeRobot, moveItNode_, &StretchMoveItInterface::homeRobot);
    connect(graspNode_, &GraspNode::navigate, mapNode_, &MapSubscriber::moveRobotLoc);
    connect(graspNode_, &GraspNode::navigateHome, mapNode_, &MapSubscriber::navigateHome);
    connect(graspNode_, &GraspNode::graspDone, this, &Server::uiButtonReturnObjectSetEnabled);  // Server to client
    connect(graspNode_, &GraspNode::turnLeft, mapNode_, &MapSubscriber::rotateLeft);

    // Page 5

    connect(this, &Server::ButtonBackToGraspClicked, this, &Server::changeToPage4);

    // Page 6
}

Server::~Server() {
    mapNode_->quit();
    moveBaseStatusNode_->quit();
    cameraNode_->quit();
    graspNode_->quit();
    moveItNode_->quit();
    mapNode_->wait();
    moveBaseStatusNode_->wait();
    cameraNode_->wait();
    graspNode_->wait();
    moveItNode_->wait();

    delete mapNode_;
    delete moveBaseStatusNode_;
    delete cameraNode_;
    delete graspNode_;
    delete moveItNode_;
}

void Server::changeToPage1() {
    emit homeRobot();
    emit enableMapping();
    QObject::disconnect(cameraNodeImgUpdate_);
    mapImgUpdate_ = connect(mapNode_, &MapSubscriber::mapUpdateQImage, this, &Server::newMap);
    headPanTilt_ = {0, -30};
}

void Server::changeToPage2() {
    emit disableMapping();
    emit cameraSetRotation(headPanTilt_.first, headPanTilt_.second);
    QObject::disconnect(mapImgUpdate_);
    cameraNodeImgUpdate_ = connect(cameraNode_, &RosCamera::imgUpdateQImage, this, &Server::uiDisplayCameraSetCamera);
}

void Server::changeToPage3() {
    headPanTilt_ = moveItNode_->getHeadPanTilt();
    emit disableMapping();
}

void Server::changeToPage4() { emit disableMapping(); }

void Server::changeToPage5() { emit enableMapping(); }

void Server::changeToPage6() { emit enableMapping(); }
void Server::uiButtonGraspClicked() { emit ButtonGraspClicked(); }
void Server::uiButtonStopClicked() { emit ButtonStopClicked(); }
void Server::uiButtonSetHomeClicked() { emit ButtonSetHomeClicked(); }
void Server::uiButtonNavigateHomeClicked() { emit ButtonNavigateHomeClicked(); }
void Server::uiButtonToggleNavTypeClicked() { emit ButtonToggleNavTypeClicked(); }
void Server::uiDisplayMapMouseClick(QPoint press, QPoint release, QSize screen) { emit DisplayMapMouseClick(press, release, screen); }
void Server::uiDisplayMapMousePressInitiated(QPoint press, QSize screen) { emit DisplayMapMousePressInitiated(press, screen); }
void Server::uiDisplayMapMousePressCurrentLocation(QPoint loc, QSize screen) { emit DisplayMapMousePressCurrentLocation(loc, screen); }

void Server::uiButtonBackClicked() { emit ButtonBackClicked(); }

void Server::uiCameraMoveButtonUpClicked() { emit CameraMoveButtonUpClicked(); }

void Server::uiCameraMoveButtonDownClicked() { emit CameraMoveButtonDownClicked(); }

void Server::uiCameraMoveButtonLeftClicked() { emit CameraMoveButtonLeftClicked(); }

void Server::uiCameraMoveButtonRightClicked() { emit CameraMoveButtonRightClicked(); }

void Server::uiCameraMoveButtonHomeClicked() { emit CameraMoveButtonHomeClicked(); }

void Server::uiDisplayCameraMouseClicked(QPoint press, QPoint release, QSize screen) { emit DisplayCameraMouseClicked(press, release, screen); }

void Server::uiConfirmButtonNoClicked() { emit ConfirmButtonNoClicked(); }

void Server::uiConfirmButtonYesClicked() { emit ConfirmButtonYesClicked(); }

void Server::uiButtonBack_2Clicked() { emit ButtonBack_2Clicked(); }

void Server::uiButtonReturnObjectClicked() { emit ButtonReturnObjectClicked(); }

void Server::uiButtonReleaseClicked() { void ButtonReleaseClicked(); }

void Server::uiButtonReplaceObjectClicked() { emit ButtonReplaceObjectClicked(); }

void Server::uiButtonNavigateClicked() { emit ButtonNavigateClicked(); }

void Server::uiButtonBackToGraspClicked() { emit ButtonBackToGraspClicked(); }
