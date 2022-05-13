#include "mapviewer.hpp"

void MapViewer::mousePressEvent(QMouseEvent *event){
    const int x = event->x();
    const int y = event->y();
    emit mapClick(x,y);
}
