#include <odometry/odometry.h>
#include <math.h>

using namespace Eigen;
#define PI 3.14159265
#define dt 0.016
#define DIFF 0.2

namespace robotis_op
{

    Odometry::Odometry()
        : control_cycle_msec_(8),
          dsp_flag(false),
          DEBUG_PRINT(false),
          counter_id(0),
          length_temp(0),
          resetOffset(0.0)
    {
        module_name_ = "odometry";

        // joint table
        joint_table_["r_hip_yaw"] = 0;
        joint_table_["r_hip_roll"] = 1;
        joint_table_["r_hip_pitch"] = 2;
        joint_table_["r_knee"] = 3;
        joint_table_["r_ank_pitch"] = 4;
        joint_table_["r_ank_roll"] = 5;

        joint_table_["l_hip_yaw"] = 6;
        joint_table_["l_hip_roll"] = 7;
        joint_table_["l_hip_pitch"] = 8;
        joint_table_["l_knee"] = 9;
        joint_table_["l_ank_pitch"] = 10;
        joint_table_["l_ank_roll"] = 11;

        joint_table_["head_pan"] = 12;
        joint_table_["head_tilt"] = 13;

        // // goal joint table
        goal_joint_table_["head_pan"] = 0;
        goal_joint_table_["head_tilt"] = 1;

        goal_joint_table_["l_ank_pitch"] = 2;
        goal_joint_table_["l_ank_roll"] = 3;
        goal_joint_table_["l_hip_pitch"] = 4;
        goal_joint_table_["l_hip_roll"] = 5;
        goal_joint_table_["l_hip_yaw"] = 6;
        goal_joint_table_["l_knee"] = 7;
        goal_joint_table_["l_sho_pitch"] = 8;
        goal_joint_table_["l_sho_roll"] = 9;

        goal_joint_table_["r_ank_pitch"] = 10;
        goal_joint_table_["r_ank_roll"] = 11;
        goal_joint_table_["r_hip_pitch"] = 12;
        goal_joint_table_["r_hip_roll"] = 13;
        goal_joint_table_["r_hip_yaw"] = 14;
        goal_joint_table_["r_knee"] = 15;
        goal_joint_table_["r_sho_pitch"] = 16;
        goal_joint_table_["r_sho_roll"] = 17;

        // ------------------------------------------
        // joint_table_["r_hip_yaw"] = 0;
        // joint_table_["r_hip_roll"] = 1;
        // joint_table_["r_hip_pitch"] = 2;
        // joint_table_["r_knee"] = 3;
        // joint_table_["r_ank_pitch"] = 4;
        // joint_table_["r_ank_roll"] = 5;

        // joint_table_["l_hip_yaw"] = 6;
        // joint_table_["l_hip_roll"] = 7;
        // joint_table_["l_hip_pitch"] = 8;
        // joint_table_["l_knee"] = 9;
        // joint_table_["l_ank_pitch"] = 10;
        // joint_table_["l_ank_roll"] = 11;

        // joint_table_["head_pan"] = 12;
        // joint_table_["head_tilt"] = 13;


        // goal_joint_table_["head_pan"] = 14;
        // goal_joint_table_["head_tilt"] = 17;

        // goal_joint_table_["l_ank_pitch"] = 10;
        // goal_joint_table_["l_ank_roll"] = 11;
        // goal_joint_table_["l_hip_pitch"] = 8;
        // goal_joint_table_["l_hip_roll"] = 7;
        // goal_joint_table_["l_hip_yaw"] = 6;
        // goal_joint_table_["l_knee"] = 9;
        // goal_joint_table_["l_sho_pitch"] = 15;
        // goal_joint_table_["l_sho_roll"] = 16;

        // goal_joint_table_["r_ank_pitch"] = 4;
        // goal_joint_table_["r_ank_roll"] = 5;
        // goal_joint_table_["r_hip_pitch"] = 2;
        // goal_joint_table_["r_hip_roll"] = 1;
        // goal_joint_table_["r_hip_yaw"] = 0;
        // goal_joint_table_["r_knee"] = 3;
        // goal_joint_table_["r_sho_pitch"] = 12;
        // goal_joint_table_["r_sho_roll"] = 13;

        //offset
        joint[0] = "r_hip_yaw";
        joint[1] = "r_hip_roll";
        joint[2] = "r_hip_pitch";
        joint[3] = "r_knee";
        joint[4] = "r_ank_pitch";
        joint[5] = "r_ank_roll";
        joint[6] = "l_hip_yaw";
        joint[7] = "l_hip_roll";
        joint[8] = "l_hip_pitch";
        joint[9] = "l_knee";
        joint[10] = "l_ank_pitch";
        joint[11] = "l_ank_roll";

        R_HIP = Affine3d(Translation3d(Vector3d(0, -k.LEG_SIDE_OFFSET, 0)));
        R_KNEE = Affine3d(Translation3d(Vector3d(0, 0, -k.THIGH_LENGTH)));
        R_ANKLE = Affine3d(Translation3d(Vector3d(0, 0, -k.CALF_LENGTH)));
        R_FOOT = Affine3d(Translation3d(Vector3d(0, 0, -k.ANKLE_LENGTH)));
        m_R_FOOT = Affine3d(Translation3d(Vector3d(0, 0, 0)));
        L_HIP = Affine3d(Translation3d(Vector3d(0, k.LEG_SIDE_OFFSET, 0)));
        L_KNEE = Affine3d(Translation3d(Vector3d(0, 0, -k.THIGH_LENGTH)));
        L_ANKLE = Affine3d(Translation3d(Vector3d(0, 0, -k.CALF_LENGTH)));
        L_FOOT = Affine3d(Translation3d(Vector3d(0, 0, -k.ANKLE_LENGTH)));
        m_L_FOOT = Affine3d(Translation3d(Vector3d(0, 0, 0)));

        current_position_ = Eigen::MatrixXd::Zero(1, joint_table_.size());
        joint_axis_direction_ = Eigen::MatrixXi::Zero(1, joint_table_.size());
        odometry_pub_counter = 0;
    }

    Odometry::~Odometry()
    {
        queue_thread_.join();
    }

    void Odometry::queueThread()
    {
        ros::NodeHandle ros_node;
        ros::CallbackQueue callback_queue;

        ros_node.setCallbackQueue(&callback_queue);

        /* publish topics */
        vis_pub = ros_node.advertise<visualization_msgs::Marker>("/alfarobi/visualization_marker", 0);
        odometry_pub_ = ros_node.advertise<geometry_msgs::PoseStamped>("/alfarobi/odometry", 10);
        robot_height_pub_ = ros_node.advertise<geometry_msgs::Point>("/alfarobi/robot_height", 10);
        angle_pub_ = ros_node.advertise<geometry_msgs::Point>("/alfarobi/angle", 10);
        angular_velocity_pub_ = ros_node.advertise<geometry_msgs::Point>("/alfarobi/angular_velocity", 10);

        ros::Subscriber goal_joint_state_sub = ros_node.subscribe("/robotis/goal_joint_states", 10, &Odometry::goalPosCommandCallback, this);
        ros::Subscriber odometry_command_sub = ros_node.subscribe("/alfarobi/odometry_command", 10, &Odometry::odometryCommandCallback, this);
        ros::Subscriber imu_sub = ros_node.subscribe("/arduino_controller/imu", 10, &Odometry::ImuCallback, this);
        ros::Subscriber button_sub = ros_node.subscribe("/arduino_controller/button", 1, &Odometry::buttonHandlerCallback, this);

        ros::WallDuration duration(control_cycle_msec_ / 1000.0);
        while (ros_node.ok())
        {
            callback_queue.callAvailable(duration);
            // process(, );
        }
    }

    void Odometry::goalPosCommandCallback(const sensor_msgs::JointState::ConstPtr &msg)
    {

        for (std::map<std::string, int>::iterator state_iter = joint_table_.begin(); state_iter != joint_table_.end(); state_iter++)
        {
            std::string _joint_name = state_iter->first;
            int joint_index = joint_table_[_joint_name];
            int goal_joint_index = goal_joint_table_[_joint_name];

            current_position_.coeffRef(0, joint_index) = msg->position[goal_joint_index];

            // ROS_INFO("Position [%s] => [%f]", _joint_name, msg->position[goal_joint_index]);
            // std::cout << "Position " << _joint_name << " => " << msg->position[goal_joint_index] << "\n";
        }

    } 

    void Odometry::odometryCommandCallback(const std_msgs::String::ConstPtr &msg)
    {
        if (msg->data == "start")
            start();
        else if (msg->data == "stop")
            stop();
    }

    void Odometry::ImuCallback(const sensor_msgs::Imu::ConstPtr &msg)
    {
        Vector3d imu;
        Quaterniond imu_orientation;

        imu(0) = msg->angular_velocity.x;
        imu(1) = msg->angular_velocity.y;
        imu(2) = msg->angular_velocity.z;

        imu_orientation.x() = msg->orientation.x;
        imu_orientation.y() = msg->orientation.y;
        imu_orientation.z() = msg->orientation.z;
        imu_orientation.w() = msg->orientation.w;

        m_imuGyr = imu;
        m_imuOri = robotis_framework::convertQuaternionToRPY(imu_orientation);

        angle_.x = m_imuOri.x();
        angle_.y = m_imuOri.y();
        angle_.z = m_imuOri.z();

        angular_velocity_.x = m_imuGyr.x();
        angular_velocity_.y = m_imuGyr.y();
        angular_velocity_.z = m_imuGyr.z();

        odometry_out_.pose.orientation.z = m_imuOri.z() - resetOffset;

        angle_pub_.publish(angle_);
        angular_velocity_pub_.publish(angular_velocity_);
    }

    void Odometry::buttonHandlerCallback(const std_msgs::String::ConstPtr &msg)
    {
        std::string buttonStatus = msg->data;

        if (buttonStatus == "B2")
        {
            resetOffset = angle_.z;
            nonpoly_odom.x = 0;
            nonpoly_odom.y = 0;
            odometry_out_.pose.position.x = 0;
            odometry_out_.pose.position.y = 0;
        }
        else if (buttonStatus == "B4")
        {
            // nonpoly_odom.x = 0;
            // nonpoly_odom.y = 0;
            // odometry_out_.pose.position.x = 0;
            // odometry_out_.pose.position.y = 0;
            
        }
    }

    void Odometry::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
    {
        queue_thread_ = boost::thread(boost::bind(&Odometry::queueThread, this));
        control_cycle_msec_ = control_cycle_msec;
        ros::Time::init();

        ROS_DEBUG("Motion: Odometry");

        offset_imu_x = 0;
        offset_imu_y = 0;
        offset_imu_z = 0;

        ros::NodeHandle ros_node;
    }

    void Odometry::start()
    {
        m_active = true;
    }

    void Odometry::stop()
    {
        m_active = false;
    }

    void Odometry::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
                           std::map<std::string, robotis_framework::Sensor *> sensors)
    {
        // present angle
        current_joint_pos_ = KDL::JntArray(14);
        for (std::map<std::string, int>::iterator state_iter = joint_table_.begin();
             state_iter != joint_table_.end(); state_iter++)
        {
            std::string _joint_name = state_iter->first;
            int joint_index = joint_table_[_joint_name];

            robotis_framework::Dynamixel *dxl = NULL;
            //Read data (Lintang, 2018)
            std::map<std::string, robotis_framework::Dynamixel *>::iterator dxl_it = dxls.find(_joint_name);
            if (dxl_it != dxls.end())
                dxl = dxl_it->second; //akses container per element
            else
                continue;
            
            current_joint_pos_(joint_index) = dxl->dxl_state_->present_position_;
            // std::cout << "DXL Raw Position " << _joint_name << " => "<< current_joint_pos_(joint_index) << "[" << joint_index << "]" << "\n";
        }
        // ROS_INFO("--");
        //update pose
        current_joint_pos = KDL::JntArray(14);

        //                     R_HIP_YAW,   R_HIP_ROLL, R_HIP_PITCH, R_KNEE,      R_ANKLE_PITCH, R_ANKLE_ROLL,
        //                     L_HIP_YAW,   L_HIP_ROLL, L_HIP_PITCH, L_KNEE,      L_ANKLE_PITCH, L_ANKLE_ROLL,
        //                     R_ARM_PITCH, R_ARM_ROLL, R_ARM_ELBOW, L_ARM_PITCH, L_ARM_ROLL,    L_ARM_ELBOW
        joint_axis_direction_ << -1, -1, -1, -1, 1, 1,
                                 -1, -1, 1, 1, -1, 1,
                                  1, -1;

        // std::cout<<"Raw IMU x (deg): "<<m_imuOri.x()*180/M_PI<<"\tRaw IMU y (deg): "<<m_imuOri.y()*180/M_PI<<"\tRaw IMU z (deg): "<<m_imuOri.z()*180/M_PI<<std::endl;
        // std::cout<<"Offset IMU x (deg): "<<offset_imu_x*180/M_PI<<"\tOffset IMU y (deg): "<<offset_imu_y*180/M_PI<<"\tOffset IMU z (deg): "<<offset_imu_z*180/M_PI<<std::endl;
        // std::cout<<"Norm IMU x (deg): "<<angle_.x*180/M_PI<<"\tNorm IMU y (deg): "<<angle_.y*180/M_PI<<"\tNorm IMU z (deg): "<<angle_.z*180/M_PI<<std::endl;
        // std::cout<<""<<std::endl;
        for (int idx = 0; idx < 12; idx++)
        {
            current_joint_pos(idx) = joint_axis_direction_(idx) * (current_joint_pos_(idx));
            // ROS_INFO("Position => [%f][%d]", current_joint_pos(idx) * 180/3.14,idx);
            // ROS_INFO("Position => [%f][%d]", current_joint_pos(idx), idx);
            

        }

        forwardKinematic();

        calculation();

        if (odometry_.x != -400 || odometry_.y != -400)
        {
            ODOM_VECTOR = ODOM.translation();
            
            nonpoly_odom.x -= odometry_.x;
            nonpoly_odom.y += odometry_.y;
            nonpoly_odom.z = odometry_.z;
            
            odometry_out_.pose.position.x = nonpoly_odom.x;//0.0041*nonpoly_odom.x*nonpoly_odom.x + 0.8161*nonpoly_odom.x + 0.0344;;
            odometry_out_.pose.position.y = nonpoly_odom.y;//0.0221*nonpoly_odom.y*nonpoly_odom.y + 0.6516*nonpoly_odom.y - 0.0815;
            odometry_out_.pose.position.z = nonpoly_odom.z;

            // odometry_out_.pose.position.x = -odometry_.x;
            // odometry_out_.pose.position.y = odometry_.y;
            // odometry_out_.pose.position.z = odometry_.z;

            odometry_pub_counter++;

            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time();
            marker.ns = "my_namespace";
            marker.id = counter_id;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = odometry_out_.pose.position.x;
            marker.pose.position.y = odometry_out_.pose.position.y;
            marker.pose.position.z = odometry_out_.pose.position.z;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.05;
            marker.scale.y = 0.05;
            marker.scale.z = 0.05;
            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;

            vis_pub.publish(marker);
            counter_id++;
            std::cout << "Odom X: " << odometry_.x << " Odom Y: " << odometry_.y << " odom Z: " <<odometry_.z << '\n';
        }

        odometry_pub_.publish(odometry_out_);
        robot_height_pub_.publish(odometry_);
        
    }

    void Odometry::forwardKinematic()
    {
        R_HIP.linear() = Eigen::Matrix3d::Identity();
        R_KNEE.linear() = Eigen::Matrix3d::Identity();
        R_ANKLE.linear() = Eigen::Matrix3d::Identity();
        L_HIP.linear() = Eigen::Matrix3d::Identity();
        L_KNEE.linear() = Eigen::Matrix3d::Identity();
        L_ANKLE.linear() = Eigen::Matrix3d::Identity();

        R_HIP.rotate(AngleAxisd(current_joint_pos(0), Vector3d(0, 0, 1) * joint_axis_direction_(0)) *
                     AngleAxisd(current_joint_pos(1), Vector3d(1, 0, 0) * joint_axis_direction_(1)) *
                     AngleAxisd(current_joint_pos(2), Vector3d(0, 1, 0) * joint_axis_direction_(2)));
        R_KNEE.rotate(AngleAxisd(current_joint_pos(3), Vector3d(0, 1, 0) * joint_axis_direction_(3)));
        R_ANKLE.rotate(AngleAxisd(current_joint_pos(4), Vector3d(0, 1, 0) * joint_axis_direction_(4)) *
                       AngleAxisd(current_joint_pos(5), Vector3d(1, 0, 0) * joint_axis_direction_(5)));
        L_HIP.rotate(AngleAxisd(current_joint_pos(6), Vector3d(0, 0, 1) * joint_axis_direction_(6)) *
                     AngleAxisd(current_joint_pos(7), Vector3d(1, 0, 0) * joint_axis_direction_(7)) *
                     AngleAxisd(current_joint_pos(8), Vector3d(0, 1, 0) * joint_axis_direction_(8)));
        L_KNEE.rotate(AngleAxisd(current_joint_pos(9), Vector3d(0, 1, 0) * joint_axis_direction_(9)));
        L_ANKLE.rotate(AngleAxisd(current_joint_pos(10), Vector3d(0, 1, 0) * joint_axis_direction_(10)) *
                       AngleAxisd(current_joint_pos(11), Vector3d(1, 0, 0) * joint_axis_direction_(11)));

        m_R_FOOT = R_HIP * R_KNEE * R_ANKLE * R_FOOT;
        m_L_FOOT = L_HIP * L_KNEE * L_ANKLE * L_FOOT;

        R_FOOT_WORLD = m_R_FOOT.translation();
        L_FOOT_WORLD = m_L_FOOT.translation();

        L_FOOT_WORLD(0) *= -1;

        R_ROT_WORLD = robotis_framework::convertRotationToRPY(m_R_FOOT.linear());
        L_ROT_WORLD = robotis_framework::convertRotationToRPY(m_L_FOOT.linear());
    }

    int Odometry::checkSupport()
    {
        double Right_Diff = fabs(R_FOOT_WORLD(2)) - fabs(L_FOOT_WORLD(2));
        double Left_Diff = fabs(L_FOOT_WORLD(2)) - fabs(R_FOOT_WORLD(2));

        if (fabs(R_FOOT_WORLD(2)) > fabs(L_FOOT_WORLD(2)) && Right_Diff > DIFF) {
            // std::cout << "01 " << "R_FOOT_WORLD X : " << fabs(R_FOOT_WORLD(0)) << " " << "L_FOOT_WORLD: " << fabs(L_FOOT_WORLD(0)) << "\n";
            // std::cout << "01 " << "R_FOOT_WORLD Y : " << fabs(R_FOOT_WORLD(1)) << " " << "L_FOOT_WORLD: " << fabs(L_FOOT_WORLD(1)) << "\n";
            // std::cout << "01 " << "R_FOOT_WORLD Z : " << fabs(R_FOOT_WORLD(2)) << " " << "L_FOOT_WORLD: " << fabs(L_FOOT_WORLD(2)) << "\n\n";
            return Right_Support;
        }
        else if (fabs(L_FOOT_WORLD(2)) > fabs(R_FOOT_WORLD(2)) && Left_Diff > DIFF) {
            // std::cout << "02 " << "R_FOOT_WORLD X : " << fabs(R_FOOT_WORLD(0)) << " " << "L_FOOT_WORLD: " << fabs(L_FOOT_WORLD(0)) << "\n";
            // std::cout << "02 " << "R_FOOT_WORLD Y : " << fabs(R_FOOT_WORLD(1)) << " " << "L_FOOT_WORLD: " << fabs(L_FOOT_WORLD(1)) << "\n";
            // std::cout << "02 " << "R_FOOT_WORLD Z : " << fabs(R_FOOT_WORLD(2)) << " " << "L_FOOT_WORLD: " << fabs(L_FOOT_WORLD(2)) << "\n\n";
            return Left_Support;
        }
        else
        {
            if (fabs(fabs(L_FOOT_WORLD(0)) - fabs(R_FOOT_WORLD(0))) < 0.01)
                length_temp = 0;

            return Double_Support;
        }
    }

    void Odometry::calculation()
    {
        // L_FOOT_WORLD(0) = L_FOOT_WORLD(0) - (-0.21398 + 0.000237851);
        // R_FOOT_WORLD(0) = R_FOOT_WORLD(0) - (0.04728 - 2.2998e-05);
        // L_FOOT_WORLD(1) = L_FOOT_WORLD(1) - (-0.05447 - 2.2998e-05);
        // R_FOOT_WORLD(1) = R_FOOT_WORLD(1) - (0.1700 - 9.68288e-05);
        // -- change --
        // L_FOOT_WORLD(0) = L_FOOT_WORLD(0) - 0.015;
        // R_FOOT_WORLD(0) = R_FOOT_WORLD(0);
        // L_FOOT_WORLD(1) = L_FOOT_WORLD(1) - (-0.1125871 + 0.09701 + 0.002);
        // R_FOOT_WORLD(1) = R_FOOT_WORLD(1) - (0.1080561 - 0.09624 + 0.0088);

        // 6 JUNI
        L_FOOT_WORLD(0) = L_FOOT_WORLD(0);
        R_FOOT_WORLD(0) = R_FOOT_WORLD(0);
        L_FOOT_WORLD(1) = L_FOOT_WORLD(1);
        R_FOOT_WORLD(1) = R_FOOT_WORLD(1);
        R_FOOT_WORLD(2) = R_FOOT_WORLD(2) * 3.10775550569 ;
        L_FOOT_WORLD(2) = L_FOOT_WORLD(2);


        //td::cout << "L_FOOT_WORLD(0) :" << L_FOOT_WORLD(0) << std::endl;
        //std::cout << "R_FOOT_WORLD(0) :" << R_FOOT_WORLD(0) << std::endl;
        //std::cout << "L_FOOT_WORLD(1) :" << L_FOOT_WORLD(1) << std::endl;
        //std::cout << "R_FOOT_WORLD(1) :" << R_FOOT_WORLD(1) << std::endl;
        
        current_support = checkSupport();
        // std::cout << "current_support: " << current_support << "\n";

        double angle_yaw = angle_.z - resetOffset;
        // ROS_INFO("YAWW [%f] ", angle_yaw * 180 / 3.1415926);
        if (current_support == Double_Support && prev_support == Right_Support)
        {
            if (dsp_flag)
            {
                std::cout << "prev: right support" << "\n";
                // tuning disini
                // change
                // odometry_.x = (L_FOOT_WORLD(0) - R_FOOT_WORLD(0)) * 9/6; 
                // odometry_.y = (R_FOOT_WORLD(1) + L_FOOT_WORLD(1)) *6/9; //1/9
                // odometry_.z = R_FOOT_WORLD(2);

                // odometry_.x = -(L_FOOT_WORLD(0) - R_FOOT_WORLD(0)) * 9/6; 
                // 6 JUNI
                odometry_.x = (L_FOOT_WORLD(0) - R_FOOT_WORLD(0)); 
                odometry_.y = (R_FOOT_WORLD(1) + L_FOOT_WORLD(1));
                odometry_.z = R_FOOT_WORLD(2);

                // ROS_INFO("<======== [%f] [%f] [%f] [%f] [%f]", angle_yaw * 180 / 3.1415926, odometry_.x, odometry_.y, odometry_out_.x, odometry_out_.y, sin(angle_yaw), cos(angle_yaw));

                Vector2f odom_vector;
                odom_vector << odometry_.x, odometry_.y;

                Rotation2Df rotMat(-angle_yaw);

                Vector2f rotatedVect = rotMat.toRotationMatrix() * odom_vector;

                // // odometry_.x = (odometry_.x * cos(angle_.z)) - (odometry_.y * sin(angle_.z));
                // // odometry_.y = (odometry_.x * sin(angle_.z)) + (odometry_.y * cos(angle_.z));

                odometry_.x = rotatedVect.x();
                odometry_.y = rotatedVect.y();

                // ROS_INFO("========> [%f] [%f] [%f] [%f] [%f]", angle_yaw * 180 / 3.1415926, odometry_.x, odometry_.y, odometry_out_.x, odometry_out_.y, sin(angle_yaw), cos(angle_yaw));

                // length_temp = L_FOOT_WORLD(0);
                ROS_INFO("<<<<==================================>>>>> ");

            }
            else
            {
                odometry_.x = -400;
                odometry_.y = -400;
                odometry_.z = -400;
            }

            dsp_flag = false;
        }
        else if (current_support == Double_Support && prev_support == Left_Support)
        {
            if (dsp_flag)
            {
                std::cout << "prev: left support" << "\n";
                // tuning disini
                // odometry_.x = -(R_FOOT_WORLD(0) - L_FOOT_WORLD(0)) * 75/71;
                // -- change --
                // odometry_.x = (L_FOOT_WORLD(0) - R_FOOT_WORLD(0)) * 75/71; 
                // odometry_.y = (R_FOOT_WORLD(1) + L_FOOT_WORLD(1)) *24/22;
                // odometry_.z = L_FOOT_WORLD(2);

                odometry_.x = (L_FOOT_WORLD(0) - R_FOOT_WORLD(0)); 
                odometry_.y = (R_FOOT_WORLD(1) + L_FOOT_WORLD(1));
                odometry_.z = L_FOOT_WORLD(2);

                

                // ROS_INFO("<======== [%f] [%f] [%f] [%f] [%f]", angle_yaw * 180 / 3.1415926, odometry_.x, odometry_.y, odometry_out_.x, odometry_out_.y, sin(angle_yaw), cos(angle_yaw));

                Vector2f odom_vector;
                odom_vector << odometry_.x, odometry_.y;

                Rotation2Df rotMat(-angle_yaw);

                Vector2f rotatedVect = rotMat.toRotationMatrix() * odom_vector;

                // // odometry_.x = (odometry_.x * cos(angle_.z)) - (odometry_.y * sin(angle_.z));
                // // odometry_.y = (odometry_.x * sin(angle_.z)) + (odometry_.y * cos(angle_.z));

                odometry_.x = rotatedVect.x();
                odometry_.y = rotatedVect.y();

                // ROS_INFO("========> [%f] [%f] [%f] [%f] [%f]", angle_yaw * 180 / 3.1415926, odometry_.x, odometry_.y, odometry_out_.x, odometry_out_.y, sin(angle_yaw), cos(angle_yaw));

                // length_temp = R_FOOT_WORLD(0);
            }
            else
            {
                odometry_.x = -400;
                odometry_.y = -400;
                odometry_.z = -400;
            }
            dsp_flag = false;
        }
        else
        {
            odometry_.x = -400;
            odometry_.y = -400;
            odometry_.z = -400;
            dsp_flag = true;
        }

        if (fabs(R_FOOT_WORLD(2)) > fabs(L_FOOT_WORLD(2)))
        {
            hip_to_base_pitch = fabs(L_FOOT_WORLD(2)) * cos(angle_.y);
            hip_to_base_roll = fabs(L_FOOT_WORLD(2)) * sin(angle_.x);
            hip_to_base = sqrt(fabs(pow(hip_to_base_pitch, 2) - pow(hip_to_base_roll, 2)));
            camera_to_base = fabs(L_FOOT_WORLD(2)) + k.BODY_LENGTH;
            odometry_.z = hip_to_base / fabs(L_FOOT_WORLD(2)) * camera_to_base;
            // std::cout<<"Odometry Z  : "<<odometry_.z<<"\n";
            // std::cout<<"Camera to Base  : "<<camera_to_base<<"\n";
            // std::cout<<"L FOOT WORLD  : "<<fabs(L_FOOT_WORLD(2))<<"\n \n";
        }
        else
        {
            hip_to_base_pitch = fabs(R_FOOT_WORLD(2)) * cos(angle_.y);
            hip_to_base_roll = fabs(R_FOOT_WORLD(2)) * sin(angle_.x);
            hip_to_base = sqrt(fabs(pow(hip_to_base_pitch, 2) - pow(hip_to_base_roll, 2)));
            camera_to_base = fabs(R_FOOT_WORLD(2)) + k.BODY_LENGTH;
            odometry_.z = hip_to_base / fabs(R_FOOT_WORLD(2)) * camera_to_base;

            // std::cout<<"Odometry Z  : "<<odometry_.z<<"\n";
            // std::cout<<"Camera to Base  : "<<camera_to_base<<"\n";
            // std::cout<<"L FOOT WORLD  : "<<fabs(L_FOOT_WORLD(2))<<"\n \n";
        }

        prev_support = current_support;
    }

} // namespace robotis_op
