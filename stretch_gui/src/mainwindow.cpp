#include "mainwindow.hpp"

#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow) {
    ui->setupUi(this);

    nh_ = ros::NodeHandle();

    connect(ui->ButtonGrasp, SIGNAL(clicked()), this, SLOT(on_ButtonGrasp_clicked()));
    connect(ui->ButtonBack, SIGNAL(clicked()), this, SLOT(on_ButtonBack_clicked()));

    ui->DisplayMap->setScaledContents(true);

    mapSub_ = new mapsubscriber(&nh_);
    connect(mapSub_, &mapsubscriber::mapUpdate, ui->DisplayMap, &SceneViewer::setPixmap);
    connect(ui->DisplayMap, &SceneViewer::sceneClick, mapSub_, &mapsubscriber::moveRobot);
    mapSub_->start();

    cameraSub_ = new rosCamera(&nh_);
    connect(cameraSub_, &rosCamera::imgUpdate, ui->DisplayCamera, &SceneViewer::setPixmap);
    connect(ui->DisplayCamera, &SceneViewer::sceneClick, cameraSub_, &rosCamera::sceneClicked);
    connect(ui->CameraMoveButtonUp, &QPushButton::clicked, cameraSub_, &rosCamera::moveUp);
    connect(ui->CameraMoveButtonDown, &QPushButton::clicked, cameraSub_, &rosCamera::moveDown);
    connect(ui->CameraMoveButtonLeft, &QPushButton::clicked, cameraSub_, &rosCamera::moveLeft);
    connect(ui->CameraMoveButtonRight, &QPushButton::clicked, cameraSub_, &rosCamera::moveRight);
    connect(ui->CameraMoveButtonHome, &QPushButton::clicked, cameraSub_, &rosCamera::moveHome);
    cameraSub_->start();


}

MainWindow::~MainWindow() {
    delete ui;
}

void MainWindow::on_ButtonGrasp_clicked() {
    ui->PagesStackedWidget->setCurrentWidget(ui->page_2);
}

void MainWindow::on_ButtonBack_clicked() {
    ui->PagesStackedWidget->setCurrentWidget(ui->page_1);
    cameraSub_->moveHome();
}

