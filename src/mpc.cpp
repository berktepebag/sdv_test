#include <iostream>

#include "path_utilities.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mpc");

    // Instantiate Classes
    MPC mpc;
    Trajectory trj(mpc);

    ros::Rate rate(5);

    double cte, epsi;

    trj.subscribeToTopics();

    while (ros::ok())
    {
        trj.calculateLocalPlanningCost();

        trj.closestPointMarkerPublish();
        trj.pathMarkerPublish();

        trj.predictControl();
        trj.cmdPublish();

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}