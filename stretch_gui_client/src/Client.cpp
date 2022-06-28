#include "Client.hpp"

#include "./ui_mainwindow.h"

Client::Client(QSharedPointer<ServerReplica> ptr, QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow), server_(ptr) {
    ui->setupUi(this);
    ui->PagesStackedWidget->setCurrentWidget(ui->page_1);

    // Page 1

    ui->ButtonNavigateHome->setEnabled(false);

    ui->ButtonBackToGrasp->setEnabled(false);
    ui->ButtonBackToGrasp->hide();

    ui->PleaseWait->setVisible(false);

    // Page 2

    ui->DisplayCamera->setScaledContents(false);

    // Page 3

    ui->DisplayImage->setScaledContents(false);

    // Page 4

    ui->DisplayGrasp->setScaledContents(false);

    initConnections();
}

void Client::initConnections(){

  // Page 1

  connect(ui->ButtonGrasp, &QPushButton::clicked, this, &Client::changeToPage2);
  connect(ui->ButtonGrasp, &QPushButton::clicked, server_.data(), &ServerReplica::uiButtonGraspClicked);
  connect(ui->ButtonStop, &QPushButton::clicked, server_.data(), &ServerReplica::uiButtonStopClicked);
  connect(ui->ButtonSetHome, &QPushButton::clicked, server_.data(), &ServerReplica::uiButtonSetHomeClicked);
  connect(ui->ButtonNavigateHome, &QPushButton::clicked, server_.data(), &ServerReplica::uiButtonNavigateHomeClicked);
  connect(ui->ButtonToggleNavType, &QPushButton::clicked, this, &Client::changeToPage6);
  connect(ui->ButtonToggleNavType, &QPushButton::clicked, server_.data(), &ServerReplica::uiButtonToggleNavTypeClicked);

  connect(ui->DisplayMap, &SceneViewer::mouseClick, server_.data(), &ServerReplica::uiDisplayMapMouseClick);
  connect(ui->DisplayMap, &SceneViewer::mousePressInitiated, server_.data(), &ServerReplica::uiDisplayMapMousePressInitiated);
  connect(ui->DisplayMap, &SceneViewer::mousePressCurrentLocation, server_.data(), &ServerReplica::uiDisplayMapMousePressCurrentLocation);

  connect(server_.data(), &ServerReplica::newMap, ui->DisplayMap, &SceneViewer::setMapQImage);
  connect(server_.data(), &ServerReplica::uiButtonNavigateHomeSetEnabled, ui->ButtonNavigateHome, &QPushButton::setEnabled);

  connect(server_.data(), &ServerReplica::uiPleaseWaitSetVisible, ui->PleaseWait, &QTextBrowser::setVisible);

  // Page 2

  connect(ui->ButtonBack, &QPushButton::clicked, server_.data(), &ServerReplica::uiButtonBackClicked);  // Both
  connect(ui->ButtonBack, &QPushButton::clicked, this, &Client::changeToPage1);

  connect(ui->CameraMoveButtonUp, &QPushButton::clicked, server_.data(), &ServerReplica::uiCameraMoveButtonUpClicked);        // Client to server
  connect(ui->CameraMoveButtonDown, &QPushButton::clicked, server_.data(), &ServerReplica::uiCameraMoveButtonDownClicked);    // Client to server
  connect(ui->CameraMoveButtonLeft, &QPushButton::clicked, server_.data(), &ServerReplica::uiCameraMoveButtonLeftClicked);    // Client to server
  connect(ui->CameraMoveButtonRight, &QPushButton::clicked, server_.data(), &ServerReplica::uiCameraMoveButtonRightClicked);  // Client to server
  connect(ui->CameraMoveButtonHome, &QPushButton::clicked, server_.data(), &ServerReplica::uiCameraMoveButtonHomeClicked);    // Client to server

  // Find point in Camera
  connect(ui->DisplayCamera, &SceneViewer::mouseClick, server_.data(), &ServerReplica::uiDisplayCameraMouseClicked);      // Client to server
  connect(ui->DisplayCamera, &SceneViewer::mouseClick, ui->ErrorNanPoint, &QTextBrowser::hide);    // client only
  connect(ui->DisplayCamera, &SceneViewer::mouseClick, ui->ErrorOutOfRange, &QTextBrowser::hide);  // client only

  connect(server_.data(), &ServerReplica::uiPointPleaseWaitShow, ui->PointPleaseWait, &QTextBrowser::show);  // Sever to client

  // Camera feed
    // Server to client
  // Error: Displays if NaN point was selected
  connect(server_.data(), &ServerReplica::uiErrorNanPointShow, ui->ErrorNanPoint, &QTextBrowser::show);    // Server to client
  connect(server_.data(), &ServerReplica::uiPointPleaseWaitHide, ui->PointPleaseWait, &QTextBrowser::hide);  // Server to client

  // True
  connect(server_.data(), &ServerReplica::uiChangeToPage3, this, &Client::changeToPage3);  // Both
  // False
  connect(server_.data(), &ServerReplica::uiErrorOutOfRangeShow, ui->ErrorOutOfRange, &QTextBrowser::show);  // Server to client
  connect(server_.data(), &ServerReplica::uiPointPleaseWaitHide, ui->PointPleaseWait, &QTextBrowser::hide);  // Server to client

  // Page 3

  connect(ui->ConfirmButtonNo, &QPushButton::clicked, this, &Client::changeToPage2);   // Client to Both
  connect(ui->ConfirmButtonNo, &QPushButton::clicked, server_.data(), &ServerReplica::uiConfirmButtonNoClicked);
  connect(ui->ConfirmButtonYes, &QPushButton::clicked, this, &Client::changeToPage4);  // Client to Both
  connect(ui->ConfirmButtonYes, &QPushButton::clicked, server_.data(), &ServerReplica::uiConfirmButtonYesClicked);  // Client to Both

  connect(server_.data(), &ServerReplica::uiDisplayImageSetCamera, ui->DisplayImage, &SceneViewer::setCameraQImage);  // Server to ui

  // Page 4

  connect(ui->ButtonBack_2, &QPushButton::clicked, server_.data(), &ServerReplica::uiButtonBack_2Clicked);                  // Client to Server
  connect(ui->ButtonBack_2, &QPushButton::clicked, this, &Client::changeToPage2);                  // Client to Both
  connect(ui->ButtonReturnObject, &QPushButton::clicked, server_.data(), &ServerReplica::uiButtonReturnObjectClicked);    // Client to server
  connect(ui->ButtonRelease, &QPushButton::clicked, server_.data(), &ServerReplica::uiButtonReleaseClicked);        // Client to server
  connect(ui->ButtonReplaceObject, &QPushButton::clicked, server_.data(), &ServerReplica::uiButtonReplaceObjectClicked);  // Client to server
  connect(ui->ButtonNavigate, &QPushButton::clicked, this, &Client::changeToPage5);
  connect(ui->ButtonNavigate, &QPushButton::clicked, server_.data(), &ServerReplica::uiButtonNavigateClicked);// Client to both

    // Server to client

  connect(server_.data(), &ServerReplica::uiButtonReturnObjectSetEnabled, ui->ButtonReturnObject, &QPushButton::setEnabled);

  // Page 5

  connect(ui->ButtonBackToGrasp, &QPushButton::clicked, server_.data(), &ServerReplica::uiButtonBackToGraspClicked);

  // Page 6
}

void Client::changeToPage1() {
    ui->PagesStackedWidget->setCurrentWidget(ui->page_1);
    ui->ButtonGrasp->show();
    ui->ButtonGrasp->setEnabled(true);
    ui->ButtonBackToGrasp->hide();
    ui->ButtonBackToGrasp->setEnabled(false);
    QObject::disconnect(DisplayFeedOne_);
    QObject::disconnect(DisplayFeedTwo_);
}

void Client::changeToPage2() {
    DisplayFeedOne_ = connect(server_.data(), &ServerReplica::uiDisplayCameraSetCamera, ui->DisplayCamera, &SceneViewer::setCameraQImage);
    QObject::disconnect(DisplayFeedTwo_);
    ui->ErrorNanPoint->setVisible(false);
    ui->ErrorOutOfRange->setVisible(false);
    ui->PointPleaseWait->setVisible(false);
    ui->PagesStackedWidget->setCurrentWidget(ui->page_2);
}

void Client::changeToPage3() {
    QObject::disconnect(DisplayFeedOne_);
    QObject::disconnect(DisplayFeedTwo_);
    ui->PagesStackedWidget->setCurrentWidget(ui->page_3);
}

void Client::changeToPage4() {
    DisplayFeedTwo_ = connect(server_.data(), &ServerReplica::uiDisplayCameraSetCamera, ui->DisplayGrasp, &SceneViewer::setCameraQImage);
    ui->ButtonReturnObject->setDisabled(true);
    ui->PagesStackedWidget->setCurrentWidget(ui->page_4);
}

void Client::changeToPage5() {
    ui->ButtonGrasp->hide();
    ui->ButtonGrasp->setEnabled(false);
    ui->ButtonBackToGrasp->show();
    ui->ButtonBackToGrasp->setEnabled(true);
    ui->PagesStackedWidget->setCurrentWidget(ui->page_1);
}

void Client::changeToPage6() {
    ui->PagesStackedWidget->setCurrentWidget(ui->page_6);
}

void Client::showButtonNavigateHome() {
    ui->ButtonNavigateHome->setEnabled(true);
}
