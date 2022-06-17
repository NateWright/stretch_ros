#include "sceneviewer.hpp"

void SceneViewer::mousePressEvent(QMouseEvent *event) {
    press = event->pos();
    setMouseTracking(true);
    emit mousePressInitiated(press, frameSize());
}

void SceneViewer::mouseReleaseEvent(QMouseEvent *event) {
    release = event->pos();
    setMouseTracking(false);
    emit mouseClick(press, release, frameSize());
}
void SceneViewer::mouseMoveEvent(QMouseEvent *event) {
    emit mousePressCurrentLocation(event->pos(), frameSize());
}
void SceneViewer::setMap(const QPixmap &map) {
    setPixmap(map.scaled(size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
}

void SceneViewer::setCamera(const QPixmap &pix) {
    resize(pix.width(), pix.height());
    setPixmap(pix);
}