#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QGraphicsView>
#include <QGraphicsItem>
#include <QPainter>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/qserialportinfo.h>
#include "robotitem.h"
#include "arduinocontroller.h"
#include <QtCharts/QChart>
#include <QtCharts/QChartView>
#include <QtCharts/QChart>
#include <chart.h>
#include <QIntValidator>
#include <QLineEdit>

#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <string>
#include <fstream>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(QWidget *parent = 0);
  ~MainWindow();
  void loadButton(const std::string path);

private:
  Ui::MainWindow *ui;

  QGraphicsScene *scene;
  RobotItem *robot;
  ArduController *controller;
  QtCharts::QChartView *chart_plotter;
  QIntValidator *intValidator;

  std::map<int, std::string> button_mapping;

  bool isCompas;


  QVector<QString> serial_port_list;

  void updateRobot(double angle);

  Chart* chart;
  Chart* chart_gyro;

  bool use_ros_;


public slots:
  void updateRPY(double roll, double pitch, double yaw);
  void updateGyro(double roll, double pitch, double yaw);
  void updateButton(int x);
  void on_refresh_button_clicked();
  void on_connect_button_clicked();
  void displayOffset(double x);

private slots:
  void checkCustomBaudRatePolicy(int idx);


signals:


protected:
//    void paintEvent(QPainter *e, const QStyleOptionGraphicsItem *option);
};

#endif // MAINWINDOW_H
