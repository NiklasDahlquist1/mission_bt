


#include "bt_handler.hpp"
//#include "behaviors_mission.h"

//#include "ros/ros.h"
//#include "tf/tf.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "bt_handler");

    ros::NodeHandle nh;

/*
    tf::Quaternion q;
    q.setRPY(10,20,30);

    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    std::cout << "rpy: " << roll << ", " << pitch << ", " << yaw << std::endl;
*/
    Bt_handler client = Bt_handler();
    
    ros::Rate r = ros::Rate(1);
    r.sleep();
    ros::spinOnce(); // delay to allow callbacks to update state, etc.

    client.loop(50.0);
    

    return 0;
}







