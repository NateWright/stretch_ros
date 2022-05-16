#include "mapviewer.hpp"

void MapViewer::mousePressEvent(QMouseEvent *event){
    const int x = event->x();
    const int y = event->y();
    emit mapClick(x, y, width(), height());
}

void MapViewer::setMap(const QPixmap &map){
  setPixmap(map.scaled(width(),height(), Qt::KeepAspectRatio));
}
