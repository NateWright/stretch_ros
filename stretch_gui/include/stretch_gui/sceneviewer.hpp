#ifndef SCENEVIEWER_H
#define SCENEVIEWER_H

#include <QLabel>
#include <QMouseEvent>
#include <QObject>
#include <QWidget>
#include <QDebug>
#include <QPoint>
#include <QSize>

class SceneViewer : public QLabel {
    Q_OBJECT
    public:
        SceneViewer(QWidget* parent = 0) : QLabel(parent) {}
        ~SceneViewer() {}
   private:
        QPoint press;
        QPoint release;
   signals:
    void mouseClick(QPoint press, QPoint release, QSize screen);
    void mousePressInitiated(QPoint press, QSize screen);
    void mousePressCurrentLocation(QPoint loc, QSize screen);
   public slots:
    void setMap(const QPixmap&);

   protected:
    virtual void mousePressEvent(QMouseEvent* event);
    virtual void mouseReleaseEvent(QMouseEvent* event);
    virtual void mouseMoveEvent(QMouseEvent *event);
};

#endif  // SCENEVIEWER_H
