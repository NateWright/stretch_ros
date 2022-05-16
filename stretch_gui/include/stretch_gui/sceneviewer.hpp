#ifndef SCENEVIEWER_H
#define SCENEVIEWER_H

#include <QLabel>
#include <QMouseEvent>
#include <QObject>
#include <QWidget>
#include <QDebug>

class SceneViewer : public QLabel {
    Q_OBJECT
   public:
    SceneViewer(QWidget* parent = 0) : QLabel(parent) {}
    ~SceneViewer() {}
   signals:
    void sceneClick(int x, int y, int width, int height);
   public slots:
    void setMap(const QPixmap&);

   protected:
    virtual void mousePressEvent(QMouseEvent* event);
};

#endif  // SCENEVIEWER_H
