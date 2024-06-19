#include "alfarobi/strategycontroller.h"

using namespace std;
using namespace Eigen;

namespace robotis_op
{

Strategy::Strategy()
  :
    useComm(false),
    r_ballFound(false),
    f_ballFound(false),
    friendAvail(false),
    toDo(WorkAlone),
    ballisAligned(false)
{

  ros::NodeHandle nh(ros::this_node::getName());

  nh.param<bool>("GlobalConfig/Global/useComm", useComm, false);

  ros::Subscriber loc_data_;
  ros::Subscriber team_comm_ = nh.subscribe("team_communication/", 10, &Strategy::TeamDataCallback, this);

}

Strategy::~Strategy()
{

}

void Strategy::TeamDataCallback(const team_communication::team_data::ConstPtr &msg)
{
  comm_data_ = *msg;

  friendAvail = comm_data_.robots_data.size() > 0 ? 1 : 0;
}

void Strategy::LocalizationDataCallback()
{

}

char Strategy::process()
{
  if(useComm)
  {
    if(friendAvail) // we have friend
    {
        if(f_ballFound && r_ballFound) //if ball seen
        {
            ballisAligned = true;
            calculateGlobalMean();
            CalculateDistance();
            return whatToDo();
        }

        else if(!ballisAligned && f_ballFound && !r_ballFound) // align to ball and check if ball seen
            return AlignToBall;
        else if(!f_ballFound && r_ballFound)
        {
            ballisAligned = true;
            calculateSelf();
            return whatToDo();
        }
        else if(!f_ballFound && !r_ballFound)
            return SearchBall;

    }
    else // no friend
    {
      calculateSelf();
      return whatToDo();
    }
  }
  else // no comm
  {
    calculateSelf();
    return whatToDo();
  }
}

void Strategy::CalculateDistance()
{
  std::vector<double> distance;
  for(int j=0; j < num_robot; j++)
  {
    distance[j] = sqrt(abs((global_mean.x() - robots_pos_[j].x()) + (global_mean.y() - robots_pos_[j].y())));
  }

  int minElementIndex = std::min_element(distance.begin(),distance.end()) - distance.begin();
  if(minElementIndex == num_robot)
    toDo = GoToBall;
  else
    toDo = WaitHere;
}

void Strategy::calculateGlobalMean()
{
  double lx = 0;
  double ly = 0;
  num_robot = comm_data_.robots_data.size();
  robots_pos_.resize(num_robot);
  for(int i = 0; i < num_robot; i++)
  {
    lx += comm_data_.robots_data[i].ball_local.x;
    ly += comm_data_.robots_data[i].ball_local.y;
    geometry_msgs::Pose2D robot_pos = comm_data_.robots_data[i].robot_pos;
    robots_pos_.push_back(Eigen::Vector3d(robot_pos.x, robot_pos.y, robot_pos.theta));
  }

  // add our own ball calc  make mean
  if(r_ballFound)
  {
    lx+=robot_ball.x();
    ly+=robot_ball.y();
    num_robot++;
    robots_pos_.resize(num_robot);
    robots_pos_.push_back(robot_pos);
  }

  global_mean.x() = lx / num_robot;
  global_mean.y() = ly / num_robot;

}

void Strategy::calculateSelf()
{

  global_mean = robot_ball;
  toDo = GoToBall;
}

char Strategy::whatToDo()
{
  return toDo;
}

}
