#pragma once

#include <QtCore>
#include <QtGui>
#include <QtWidgets>

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

// #include "vision_utils/Particles.h"
// #include "vision_utils/Features.h"

#include "odom_monitor/dataprocess.h"

#include <sstream>

    static constexpr int FIELD_LENGTH = 800;
    static constexpr int FIELD_WIDTH = 500;
    static constexpr int GOAL_DEPTH = 60;
    static constexpr int GOAL_WIDTH = 300;
    static constexpr int GOAL_AREA_LENGTH = 100;
    static constexpr int GOAL_AREA_WIDTH = 500;
    static constexpr int PENALTY_MARK_DISTANCE = 210;
    static constexpr int CENTER_CIRCLE_DIAMETER = 150;
    static constexpr int BORDER_STRIP_WIDTH = 70;

namespace Math{
    static constexpr float PI = 3.14159265359f;
    static constexpr float TWO_PI = 2.0f * PI;
    static constexpr float PI_TWO = PI / 2.0f;
    static constexpr float THREE_PI_TWO = 3.0f * PI_TWO;
    static constexpr float DEG2RAD = PI/180.0f;
    static constexpr float RAD2DEG = 1.0f/DEG2RAD;
}

class DataViewer:public QGraphicsView{
    Q_OBJECT
private:
    QGraphicsScene *data_scene_;

    DataProcess *data_process_;

    QImage field_background_;
    ros::NodeHandle nh_;
    ros::Subscriber robot_state_subs_;

    void odomCallback(const geometry_msgs::PoseStampedConstPtr &_msg1);

    geometry_msgs::Pose2D robot_state_;


    void drawRobot();

public:
    DataViewer(QObject *parent=0);
    ~DataViewer();

    void processBegin();
    void updateData();
    bool show_grid_;
    bool show_particles_;
    
signals:
    void updateViewer();

};

