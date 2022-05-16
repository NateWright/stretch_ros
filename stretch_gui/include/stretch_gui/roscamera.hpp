#ifndef ROSCAMERA_HPP
#define ROSCAMERA_HPP

#include <QThread>
#include <QObject>
#include <QWidget>

class rosCamera : public QThread
{
  Q_OBJECT
public:
  explicit rosCamera(QObject *parent = nullptr);
};

#endif // ROSCAMERA_HPP
