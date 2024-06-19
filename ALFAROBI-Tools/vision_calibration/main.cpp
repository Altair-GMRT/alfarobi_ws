#include <main_window.h>
#include <QApplication>
#include <QColorDialog>
#include <ros/ros.h>

int main(int argc, char *argv[])
{
  QApplication a(argc, argv);
  ros::init(argc,argv,"vision_calibration");

  MainWindow w;
  w.show();

  return a.exec();
}
