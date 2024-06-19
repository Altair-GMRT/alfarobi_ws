#include "odom_monitor/dataviewer.h"

DataViewer::DataViewer(QObject *parent):
    nh_(ros::this_node::getName()){

    (void)parent;

    this->setFixedSize(BORDER_STRIP_WIDTH*2 + FIELD_LENGTH,BORDER_STRIP_WIDTH*2 + FIELD_WIDTH);
    std::stringstream field_bg_path;

    field_bg_path << ros::package::getPath("odom_monitor") << "/data/lapangan_odom_baru.png";

    field_background_.load(field_bg_path.str().c_str());

    data_scene_ = new QGraphicsScene;
    data_scene_->addPixmap(std::move(QPixmap::fromImage(field_background_)));
    this->setScene(data_scene_);

    robot_state_subs_ = nh_.subscribe("/alfarobi/odometry",1, &DataViewer::odomCallback, this);

    data_process_ = new DataProcess;

}

DataViewer::~DataViewer(){

}

void DataViewer::odomCallback(const geometry_msgs::PoseStampedConstPtr &msg){
    robot_state_.x = (msg->pose.position.x * 100) + 500; //460, makin naik nilainya, makin ke kanan
    robot_state_.y = (-1*msg->pose.position.y * 100) + 380; //430, makin naik nilainya, makin ke di bawah odommontironuya
    robot_state_.theta = -1*msg->pose.orientation.z;

    //std::cout << "robot_state_.theta : " << robot_state_.theta << "\n";
    
    emit updateViewer();
}

void DataViewer::updateData(){
    data_scene_->clear();
    data_scene_->addPixmap(std::move(QPixmap::fromImage(field_background_)));

    drawRobot();

    this->scene()->update();
}

void DataViewer::drawRobot(){
    if(robot_state_.x == 999.0 && robot_state_.y == 999.0){
        robot_state_.x = BORDER_STRIP_WIDTH>>1;
        robot_state_.y = BORDER_STRIP_WIDTH>>1;
        robot_state_.theta = .0;
    }

    double rad = robot_state_.theta;
    constexpr int w = 20,l = 54;
    constexpr double alpha = atan(3.0*w/l);
    constexpr float constant = (float)sqrt((l*l)/9.0f + w*w);

    QPolygonF robot_poly;
    robot_poly << QPoint(robot_state_.x-constant*cos(alpha+rad),robot_state_.y-constant*sin(alpha+rad))
            << QPoint(robot_state_.x-constant*cos(alpha-rad),robot_state_.y+constant*sin(alpha-rad))
            << QPoint(robot_state_.x+(int)(2.0*l*cos(rad)/3.0),robot_state_.y+(int)(2.0*l*sin(rad)/3.0));
    QPen robot_pen(QColor(221,153,255), 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    data_scene_->addPolygon(robot_poly,robot_pen);
}

void DataViewer::processBegin(){
    data_process_->begin();
}
