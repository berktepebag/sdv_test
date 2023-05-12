#include "mpc_local_planner_ros.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(mpc_local_planner::MpcLocalPlannerROS, nav_core::BaseLocalPlanner)
// PLUGINLIB_EXPORT_CLASS(mpc_local_planner::MpcLocalPlannerROS, mbf_costmap_core::CostmapController)


namespace mpc_local_planner{

MpcLocalPlannerROS::MpcLocalPlannerROS() : costmap_ros_(NULL), tf_(NULL), initialized_(false) {}

MpcLocalPlannerROS::MpcLocalPlannerROS(std::string name, tf2_ros::Buffer* tf,
                           costmap_2d::Costmap2DROS* costmap_ros)
    : costmap_ros_(NULL), tf_(NULL), initialized_(false)
{
    initialize(name, tf, costmap_ros);
}

MpcLocalPlannerROS::~MpcLocalPlannerROS() {}

// Take note that tf::TransformListener* has been changed to tf2_ros::Buffer* in ROS Noetic
void MpcLocalPlannerROS::initialize(std::string name, tf2_ros::Buffer* tf,
                              costmap_2d::Costmap2DROS* costmap_ros)
{
    if(!initialized_)
    {
        tf_ = tf;
        costmap_ros_ = costmap_ros;
        initialized_ = true;
    }
}

bool MpcLocalPlannerROS::setPlan(
    const std::vector<geometry_msgs::PoseStamped>& orig_global_plan
)
{
    if(!initialized_)
    {
        ROS_ERROR("This planner has not been initialized");
        return false;
    }
    return true;
}

bool MpcLocalPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
    if(!initialized_)
    {
        ROS_ERROR("This planner has not been initialized");
        return false;
    }
    return true;
}

bool MpcLocalPlannerROS::isGoalReached()
{
    if(!initialized_)
    {
        ROS_ERROR("This planner has not been initialized");
        return false;
    }
    return false;
}
}
