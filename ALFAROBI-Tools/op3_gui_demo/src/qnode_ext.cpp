#include <op3_gui_demo/qnode.hpp>

namespace robotis_op {
// Quintic Walk

void QNodeOP3::applyQuinticWalkParam(const quintic_walk_msgs::WalkingParam &walking_param)
{
    quintic_walk_param_ = walking_param;
    set_quintic_walk_param_pub.publish(quintic_walk_param_);
}

void QNodeOP3::setQuinticWalkCommand(const std::string &command)
{
    std_msgs::String _commnd_msg;
    _commnd_msg.data = command;
    set_quintic_walk_command_pub.publish(_commnd_msg);
}

void QNodeOP3::refreshQuinticWalkParam()
{
    quintic_walk_msgs::GetWalkingParam walking_param_msg;

    if (get_quintic_walk_param_client_.call(walking_param_msg))
    {
      quintic_walk_param_ = walking_param_msg.response.parameters;

      // update ui
      Q_EMIT updateQuinticWalkParameters(quintic_walk_param_);
    }
}

}