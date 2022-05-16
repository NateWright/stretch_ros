#include "sceneviewer.hpp"

void SceneViewer::mousePressEvent(QMouseEvent *event){
    const int x = event->x();
    const int y = event->y();
    emit sceneClick(x, y, width(), height());
}

void SceneViewer::setMap(const QPixmap &map){
  setPixmap(map.scaled(size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
}
