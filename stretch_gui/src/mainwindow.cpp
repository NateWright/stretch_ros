#include "mainwindow.hpp"

#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow) {
    ui->setupUi(this);

    nh_ = ros::NodeHandle();

    connect(ui->ButtonGrasp, SIGNAL(clicked()), this, SLOT(on_ButtonGrasp_clicked()));
    connect(ui->ButtonBack, SIGNAL(clicked()), this, SLOT(on_ButtonBack_clicked()));

    ui->DisplayMap->setScaledContents(true);

    mapSub = new mapsubscriber(&nh_);
    connect(mapSub, &mapsubscriber::mapUpdate, ui->DisplayMap, &MapViewer::setPixmap);

    mapSub->start();
}

MainWindow::~MainWindow() {
    delete ui;
}

void MainWindow::on_ButtonGrasp_clicked() {
    ui->PagesStackedWidget->setCurrentWidget(ui->page_2);
}

void MainWindow::on_ButtonBack_clicked() {
    ui->PagesStackedWidget->setCurrentWidget(ui->page_1);
}
