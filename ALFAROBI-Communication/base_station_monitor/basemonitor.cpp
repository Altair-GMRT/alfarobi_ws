#include "basemonitor.h"
#include "ui_basemonitor.h"

BaseMonitor::BaseMonitor(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::BaseMonitor)
{
  ui->setupUi(this);
}

BaseMonitor::~BaseMonitor()
{
  delete ui;
}
