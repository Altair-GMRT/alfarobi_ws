#ifndef BASEMONITOR_H
#define BASEMONITOR_H

#include <QMainWindow>

namespace Ui {
class BaseMonitor;
}

class BaseMonitor : public QMainWindow
{
  Q_OBJECT

public:
  explicit BaseMonitor(QWidget *parent = 0);
  ~BaseMonitor();

private:
  Ui::BaseMonitor *ui;
};

#endif // BASEMONITOR_H
