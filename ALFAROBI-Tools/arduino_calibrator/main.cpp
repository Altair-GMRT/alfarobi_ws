#include "mainwindow.h"
#include <ros/ros.h>
#include <QApplication>

int main(int argc, char **argv)
{
    ros::init(argc,argv, "arduino_calibrator");

    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();
}
