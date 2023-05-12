#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "eigen3/Eigen/Dense"
#include <vector>
#include "MPC.h"

#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include "geometry_msgs/TransformStamped.h"
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_datatypes.h>


// /move_base/NavfnROS/plan

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

bool DEBUG = true;

// Evaluate a polynomial.
double polyeval(const VectorXd &coeffs, double x);
// Fit a polynomial.
VectorXd polyfit(const VectorXd &xvals, const VectorXd &yvals, int order);

VectorXd polyfit(const VectorXd &xvals, const VectorXd &yvals, int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);

  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); ++i) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); ++j) {
    for (int i = 0; i < order; ++i) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);

  return result;
}

double polyeval(const VectorXd &coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); ++i) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}


void GlobalPlannerCallback(const nav_msgs::Path::ConstPtr& msg, VectorXd& coeffs, bool coeffsIsSet, int polyDeg, geometry_msgs::Pose& globalPathGoalPointPose)
{
  globalPathGoalPointPose.position.x = msg-> poses[0].pose.position.x;
  globalPathGoalPointPose.position.y = msg-> poses[0].pose.position.y;
  
  globalPathGoalPointPose.orientation.x = msg-> poses[0].pose.orientation.x;
  globalPathGoalPointPose.orientation.y = msg-> poses[0].pose.orientation.y;
  globalPathGoalPointPose.orientation.z = msg-> poses[0].pose.orientation.z;
  globalPathGoalPointPose.orientation.w = msg-> poses[0].pose.orientation.w;
  
  // Number of samples to be used to calculate coeffs of the path function
  int samplesize = 10;

  if(msg -> poses.size() >= samplesize){

    VectorXd xvals(samplesize);
    VectorXd yvals(samplesize);

    int stepsize = msg->poses.size() / samplesize;
    
    if(DEBUG & false)
    {
    ROS_INFO("x:[%f]",msg->poses[0].pose.position.x);
    ROS_INFO("x:[%ld]",msg->poses.size());
    ROS_INFO("stepsize:[%d]",stepsize);
    }

    for(int i = 0; i < samplesize; i++)
    {
      xvals[i] = msg -> poses[i * stepsize].pose.position.x;
      yvals[i] = msg -> poses[i * stepsize].pose.position.y;
      // ROS_INFO("stepsize*i:[%d]",stepsize * i);
    }

    coeffs = polyfit(xvals, yvals, polyDeg);
    // ROS_INFO("x3:[%f], x2:[%f], x1:[%f]",coeffs[0],coeffs[1],coeffs[2]);
    ROS_INFO("FIRST VARIABLES-> a: %.2f b: %.2f c: %.2f d: %.2f",coeffs[0],coeffs[1],coeffs[2],coeffs[3]);

  }
  else{
    ROS_INFO("Insufficient Path Pose recevied. # of poses received / required [%ld]/[%d]", msg -> poses.size(), samplesize);
  }

}

void RobotPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg,     geometry_msgs::PoseWithCovarianceStamped& robotP)
{
  robotP.pose.pose.position.x = msg -> pose.pose.position.x;  
  robotP.pose.pose.position.y = msg -> pose.pose.position.y;
  robotP.pose.pose.orientation.z = msg -> pose.pose.orientation.z;
}

void RobotVelCallback(const nav_msgs::Odometry::ConstPtr& msg, vector<float>& robot_vel)
{
  // Linear Vel
  robot_vel[0] = msg->twist.twist.linear.x; 
  // Angular
  robot_vel[1] = msg->twist.twist.angular.z; 
}

void GoalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, geometry_msgs::Point& goalPosition)
{
  ROS_INFO("x: %f y: %f", msg->pose.position.x,msg->pose.position.y);

}

void pathMarker(visualization_msgs::MarkerArray &markerArray, std::string frame_, const std::vector<double> pos_x, const std::vector<double> pos_y, tf::Quaternion q_, double r_, double g_, double b_, double a_)
{
    for (int id = 0; id < pos_x.size(); id++)
    {
        markerArray.markers[id].header.frame_id = frame_;
        markerArray.markers[id].header.stamp = ros::Time();
        markerArray.markers[id].type = visualization_msgs::Marker::CUBE;
        markerArray.markers[id].action = visualization_msgs::Marker::ADD;
        markerArray.markers[id].id = id;
        markerArray.markers[id].pose.position.x = pos_x[id];
        markerArray.markers[id].pose.position.y = pos_y[id];
        markerArray.markers[id].pose.orientation.x = q_[0];
        markerArray.markers[id].pose.orientation.y = q_[1];
        markerArray.markers[id].pose.orientation.z = q_[2];
        markerArray.markers[id].pose.orientation.w = q_[3];
        markerArray.markers[id].scale.x = 0.1;
        markerArray.markers[id].scale.y = 0.1;
        markerArray.markers[id].scale.z = 0.1;
        markerArray.markers[id].color.a = a_; // Don't forget to set the alpha!
        markerArray.markers[id].color.r = r_ - (0.05 * id);
        markerArray.markers[id].color.g = g_ - (0.05 * id);
        markerArray.markers[id].color.b = b_ - (0.05 * id);
    }
}

// int getClosestPathPoint(const nav_msgs::Path globalTrajectory, geometry_msgs::PoseWithCovarianceStamped robotP)
{
    nav_msgs::Path closestPath = globalTrajectory;
    double shortest_dist = 1.0e9;
    double smallest_theta = 1.0e9;
    int closest_point = 0;

    // Find closest path point to the car and return a path starting from that point
    for (int i = 0; i < globalTrajectory.poses.size(); i++)
    {
        double dist_x = 0;
        double dist_y = 0;
        double new_dist = 0.0;
        double new_theta = 0.0;

        // dist_x = globalTrajectory.poses[i].pose.position.x - poseCar.position.x;
        // dist_y = globalTrajectory.poses[i].pose.position.y - poseCar.position.y;

        tf::Pose traj_pose;
        tf::poseMsgToTF(globalTrajectory.poses[i].pose, traj_pose);
        tf::Pose poseCarTf;
        tf::poseMsgToTF(robotP.pose, poseCarTf);

        tf::Pose diffTF = traj_pose.inverseTimes(poseCarTf);

        geometry_msgs::Transform diffMsg;
        tf::transformTFToMsg(diffTF, diffMsg);
        // ROS_INFO("translation: x %.2f y %.2f, rotation: z %.2f",diffMsg.translation.x,diffMsg.translation.y, tf::getYaw(diffMsg.rotation) * 180 / 3.1415);

        double dist_xy = sqrt(diffMsg.translation.x * diffMsg.translation.x + diffMsg.translation.y * diffMsg.translation.y);
        double theta_angle = fabs(tf::getYaw(diffMsg.rotation));

        if (dist_xy < shortest_dist)
        {
            if (fabs(theta_angle) < M_PI * 3.0 / 4.0)
            {
                shortest_dist = dist_xy;
                smallest_theta = theta_angle;
                closest_point = i;
            }
        }
    }
    // If closest point found get x,y positions and z rotation
    if (closest_point >= 0)
    {
        double pos_x = globalTrajectory.poses[closest_point].pose.position.x;
        double pos_y = globalTrajectory.poses[closest_point].pose.position.y;
        double rot_z = tf::getYaw(globalTrajectory.poses[closest_point].pose.orientation);
        // ROS_INFO("shortest dist %.2f @ %d x: %.2f y: %.2f z: %.2f",shortest_dist,closest_point, pos_x, pos_y,rot_z);
        // Show the closest point on RVIZ as a marker
        setClosestPointMarker(pos_x, pos_y);
    }

    return closest_point;
}

vector<double> predictControl(
  VectorXd coeffs, 
  geometry_msgs::PoseWithCovarianceStamped robotP, 
  float robot_linear_vel, 
  visualization_msgs::MarkerArray& predictedPathMarker, 
  visualization_msgs::MarkerArray& closestPathMarker, 
  geometry_msgs::Pose& globalPathGoalPointPose)
{
  float robot_pos_x = robotP.pose.pose.position.x;
  float robot_pos_y = robotP.pose.pose.position.y;
  float robot_psi = robotP.pose.pose.orientation.z;

  // ROS_INFO("COEFFS VARIABLES-> a: %.2f b: %.2f c: %.2f d: %.2f",coeffs[0],coeffs[1],coeffs[2],coeffs[3]);

  float cte = robot_pos_y - polyeval(coeffs,robot_pos_x);
  // ROS_INFO("cte:[%f]",cte);

  float path_psi = atan(coeffs[1] + 2 * robot_pos_x * coeffs[2] + 3 * coeffs[3] * pow(robot_pos_x, 2));  
  float epsi = robot_psi - path_psi;

  Eigen::VectorXd state(6);
  state << 0,0,0,0,0,0;
  state << robot_pos_x, robot_pos_y, robot_psi, robot_linear_vel, cte, epsi;

  // std::cout << "State: \n" << state << std::endl;

  MPC mpc;
  vector<double> command = mpc.Solve(state, coeffs);
  // ROS_INFO("A:[%f] Steer:[%f]",command[0], command[1]);

  std::vector<double> mpc_x_vals;
  std::vector<double> mpc_y_vals;
  
  for (int i = 2; i < command.size(); i++)
  {
    if (i % 2 == 0)
    {
        mpc_x_vals.push_back(command[i]);
        // ROS_INFO("mpc_x_vals: %.5f", command[i]);
    }
    else
    {
        mpc_y_vals.push_back(command[i]);
        // ROS_INFO("mpc_y_vals: %.5f", command[i]);
    }
  }

  tf::Quaternion q_car(
        robot_pos_x,
        robot_pos_y,
        robot_psi,
        robotP.pose.pose.orientation.w);
    // q_car_global = q_car;

    tf::Matrix3x3 m_car(q_car);
    double car_roll, car_pitch, car_yaw;
    m_car.getRPY(car_roll, car_pitch, car_yaw);

  if (mpc_x_vals.size() > 0)
  {
      predictedPathMarker.markers.resize(mpc_x_vals.size());
      pathMarker(predictedPathMarker, "base_link", mpc_x_vals, mpc_y_vals, q_car, 0.0, 1.0, 0.0, 1.0);
      
  }

  // Predicted Global Path from Coeffs
  double poly_inc = 0.025;
  int num_points = 20;
  closestPathMarker.markers.resize(num_points);
  std::vector<double> x_vals;
  std::vector<double> y_vals;

  float closestPoints_x = globalPathGoalPointPose.position.x;
  float closestPoints_y = globalPathGoalPointPose.position.y;

  // closestPathPoint Orientation
  tf::Quaternion q_closestPathPoint(
      globalPathGoalPointPose.orientation.x,
      globalPathGoalPointPose.orientation.y,
      globalPathGoalPointPose.orientation.z,
      globalPathGoalPointPose.orientation.w);

  for (int i = 1; i < num_points; i++)
        {
            float x_val, y_val;
            x_val = poly_inc * i;
            y_val = polyeval(coeffs, x_val);
            ROS_INFO("GlobalPathFromCoeffs[%d] -> x_val: %.4f y_val: %.4f", i, x_val, y_val);
            // ROS_INFO("x_vals[%d]: %.8f y_vals[%d]: %.2f", i, x_vals[i], i, y_vals[i]);
            x_vals.push_back(x_val);
            y_vals.push_back(y_val);
        }
        // pathMarker(closestPathMarker, poly_inc*i,polyeval(localCoeffs,poly_inc*i), i);
        pathMarker(closestPathMarker, "map", x_vals, y_vals, q_closestPathPoint, 1.0, 0.0, 0.0, 1.0);


  return command;
}

int main(int argc, char **argv)
{
  geometry_msgs::PoseWithCovarianceStamped robotP;
  // vector<float> robotP(3);
  vector<float> robot_vel(2);

  float psi_path;
  float psi_robot;

  // Polynomial & Coeffients
  bool coeffsIsSet = false;
  int polyDeg = 3; // Degree of polynomial. i.e. x^3 = 3
  polyDeg++; // Since n degree polynomials will have n+1 coeefs.
  VectorXd coeffs(polyDeg);


  ros::init(argc, argv, "global_path_mpc_node");
  ros::NodeHandle nh;

  ros::Publisher cmd_pub;
  cmd_pub = nh.advertise<geometry_msgs::Twist>("/tricycle_controller/cmd_vel", 1);
  geometry_msgs::Twist ctrl_msg;
  geometry_msgs::Point goalPosition;  

  ros::Publisher predictedPathMarker_pub;
  predictedPathMarker_pub = nh.advertise<visualization_msgs::MarkerArray>("/predictedPath", 10);
  visualization_msgs::MarkerArray predictedPathMarker;

  ros::Publisher closestPathMarker_pub;
  closestPathMarker_pub = nh.advertise<visualization_msgs::MarkerArray>("/closestPath", 10);
  visualization_msgs::MarkerArray closestPathMarker;
  geometry_msgs::Pose globalPathGoalPointPose;
  nav_msgs::Path 

  // Receive global path from TEB global planner. 
  // TODO: Change this with a better method. 
  ros::Subscriber planSub= nh.subscribe<nav_msgs::Path> ("/move_base/NavfnROS/plan", 100, boost::bind(GlobalPlannerCallback, _1, boost::ref(coeffs), boost::ref(coeffsIsSet), polyDeg, boost::ref(globalPathGoalPointPose) ));
  
  // Receive Robot Pose from amcl
  ros::Subscriber robotPoseSub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped> ("/amcl_pose", 100, boost::bind(RobotPoseCallback, _1, boost::ref(robotP)));

  // Receive Robot_vel from /tricycle_controller/odom
  ros::Subscriber robotVelSub = nh.subscribe<nav_msgs::Odometry> ("/tricycle_controller/odom", 100, boost::bind(RobotVelCallback, _1, boost::ref(robot_vel)));

  ros::Subscriber goalPoseSub = nh.subscribe<geometry_msgs::PoseStamped> ("/move_base_simple/goal", 100, boost::bind(GoalPoseCallback, _1, boost::ref(goalPosition)));

  ros::Rate r(20); // 10 hz
  while (true)
  {
    if(DEBUG){
    float cte = robotP.pose.pose.position.y - polyeval(coeffs,robotP.pose.pose.position.x);
    float cte_sqr = pow(cte,2);

    float e_psi = psi_path - psi_robot;

    // ROS_INFO("Robot x:[%f],y:[%f]",robotP[0],robotP[1]);
    // ROS_INFO("Path  x:[%f],y:[%f]",robotP[0],polyeval(coeffs, robotP[0]));
    // ROS_INFO("Robot lin_x:[%f], rot_z:[%f]",robot_vel[0],robot_vel[1]);
    // ROS_INFO("cte:[%f]",cte);
    // ROS_INFO("cte_sqr:[%f]",cte_sqr);
    // ROS_INFO("e_psi:[%f]",e_psi);
    // ROS_INFO("e_psi_sqr:[%f]",e_psi);
    }

    // getClosestPathPoint(const nav_msgs::Path globalTrajectory, geometry_msgs::PoseWithCovarianceStamped robotP);

    auto command = predictControl(coeffs, robotP, robot_vel[0], predictedPathMarker, closestPathMarker, globalPathGoalPointPose);

    ctrl_msg.linear.x = command[1];
    ctrl_msg.angular.z = -command[0];
    cmd_pub.publish(ctrl_msg);
    ROS_INFO("A:[%f] Steer:[%f]",command[1], command[0]);

    predictedPathMarker_pub.publish(predictedPathMarker);
    closestPathMarker_pub.publish(closestPathMarker);


    ros::spinOnce();
    r.sleep();
  }

  return 0;
}