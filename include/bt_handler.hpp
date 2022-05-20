








#ifndef BT_HANDLER_HPP
#define BT_HANDLER_HPP




#include "ros/ros.h"
#include "behaviortree_cpp_v3/bt_factory.h"



#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"



//#include "behaviors_mission.h"



class Bt_handler
{
    public:
    Bt_handler();
    void loop(double loopRate);




    struct state_struct
    {
        geometry_msgs::Pose currentPose;
        geometry_msgs::Pose moveTo1;
        geometry_msgs::Point moveTo2;


        ros::Publisher muxPathState_pub;
        ros::Publisher muxPathWaypoint_pub;

        ros::Publisher muxControllerState_pub;
        ros::Publisher muxControllerPose_pub;


        ros::Publisher launchShafter_pub;
        

        bool hasMove1 = false;
        bool hasMove2 = false;
        bool takeoffComplete = false;

        bool startCommand = false;
    };


    private:

    ros::NodeHandle nodeHandle;

    ros::Subscriber movePose_sub;
    ros::Subscriber movePoint_sub;
    ros::Subscriber startCommand_sub;
    ros::Subscriber takeoffComplete_sub;
    ros::Subscriber odom_sub;

    void odomCB(const nav_msgs::Odometry& msg);
    void movePoseCB(const geometry_msgs::PoseStamped& msg);
    void movePointCB(const geometry_msgs::PointStamped& msg);
    void startCommandCB(const std_msgs::Bool& msg);
    void takeoffCompleteCB(const std_msgs::String& msg);

    ros::Publisher muxReference_pub;
    ros::Publisher muxState_pub;


    state_struct state;
    BT::BehaviorTreeFactory factory;
    BT::Tree tree;
    std::string treeXML =  R"(
            <root main_tree_to_execute = "MainTree" >
                <BehaviorTree ID="MainTree">
                    <Sequence name="root">
                        <Action ID="MoveToPose"/>
                        <Action ID="AllignRotation"/>
                        <Action ID="LaunchShafter"/>
                        <Action ID="MoveToPoint"/>
                    </Sequence>
                </BehaviorTree>
            </root>
            )";


    void initNodes(const BT::Tree& tree);
    void initFactory(BT::BehaviorTreeFactory& factory);


    //void resetMission();

};














#endif
