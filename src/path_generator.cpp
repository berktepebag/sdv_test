#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PointStamped.h"
#include "eigen3/Eigen/Dense"
#include "vector"

using Eigen::VectorXd;
using namespace std;

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
// void clickedPointCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
void clickedPointCallback(const geometry_msgs::PointStamped::ConstPtr& msg, vector< vector<float> >& clickedPoints_, int& local_clickedpoint_counter)
{
    ROS_INFO("x: [%f] y: [%f]", msg->point.x, msg->point.y);

    clickedPoints_.push_back({msg->point.x, msg->point.y});
    local_clickedpoint_counter++;
    // cout << "cp_inside_callback: " << clickedPoints_.size() << endl;
}

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


int main(int argc, char **argv)
{
    int order  = 3;
    int global_clickedpoint_counter = 0;
    int local_clickedpoint_counter = 0;

    vector< vector<float> > clickedPoints;

    ros::init(argc, argv, "clicked_point_listener");

    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<geometry_msgs::PointStamped> ("/clicked_point", 10, boost::bind(clickedPointCallback, _1, boost::ref(clickedPoints) , boost::ref(local_clickedpoint_counter) ) );

    ros::Rate r(10); // 10 hz
    while (true)
    {
    // cout << "cp_inside_MAIN: " << clickedPoints.size();
    if(clickedPoints.size()-1 >= order & (global_clickedpoint_counter != local_clickedpoint_counter) )
    {
        // cout << "New point arrived" << endl;
        global_clickedpoint_counter++;
        VectorXd xvals(clickedPoints.size());
        VectorXd yvals(clickedPoints.size());

        cout << "----------" << endl;
        for (int i = 0; i < clickedPoints.size(); i++)
            {
                cout << "x: " << clickedPoints[i][0] << " y: " << clickedPoints[i][1] << endl;
                xvals[i] = clickedPoints[i][0];
                yvals[i] = clickedPoints[i][1];
            }
        
        VectorXd coeffs = polyfit(xvals, yvals, order);

        for (double x = 0; x <= 20; ++x) {
         
            std::cout << "x: " << x << "y: " << polyeval(coeffs, x) << std::endl;
        }

    }
    ros::spinOnce();
    r.sleep();
    }

    return 0;
}