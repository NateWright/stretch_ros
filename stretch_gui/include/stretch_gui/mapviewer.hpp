#ifndef MAPVIEWER_HPP
#define MAPVIEWER_HPP

#include <QLabel>
#include <QMouseEvent>
#include <QObject>
#include <QWidget>
#include <QDebug>

class MapViewer : public QLabel {
    Q_OBJECT
   public:
    MapViewer(QWidget* parent = 0) : QLabel(parent) {}
    ~MapViewer() {}
   signals:
    void mapClick(int x, int y, int width, int height);
   public slots:
    void setMap(const QPixmap&);

   protected:
    virtual void mousePressEvent(QMouseEvent* event);
};

#endif  // MAPVIEWER_HPP
