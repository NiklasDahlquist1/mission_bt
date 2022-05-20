

#include "bt_handler.hpp"


#include "behaviors_mission.h"




Bt_handler::Bt_handler()
{
    // ros
    odom_sub = nodeHandle.subscribe("odom", 1000, &Bt_handler::odomCB, this);
    movePose_sub = nodeHandle.subscribe("move_base_simple/goal", 1000, &Bt_handler::movePoseCB, this);
    movePoint_sub = nodeHandle.subscribe("clicked_point", 1000, &Bt_handler::movePointCB, this);
    startCommand_sub = nodeHandle.subscribe("start_command", 1000, &Bt_handler::startCommandCB, this);
    takeoffComplete_sub = nodeHandle.subscribe("take_off/complete", 1000, &Bt_handler::takeoffCompleteCB, this);



    state.muxPathState_pub = nodeHandle.advertise<std_msgs::String>("state/path", 1000);
    state.muxPathWaypoint_pub = nodeHandle.advertise<geometry_msgs::PoseStamped>("mux_path_wp", 1000);

    state.muxControllerState_pub = nodeHandle.advertise<std_msgs::String>("state", 1000);
    state.muxControllerPose_pub = nodeHandle.advertise<geometry_msgs::PoseStamped>("mux_controller_wp", 1000);
    state.launchShafter_pub = nodeHandle.advertise<std_msgs::String>("take_off/launching", 1000);




    //state.muxReference_pub = nodeHandle.advertise<geometry_msgs::PoseStamped>("muxReference", 1000);
    //state.muxState_pub = nodeHandle.advertise<std_msgs::String>("state", 1000);
    
    initFactory(factory);


    tree = factory.createTreeFromText(treeXML);
    initNodes(tree);
}
void Bt_handler::loop(double loopRate)
{
    ros::Rate rate = ros::Rate(loopRate);
    bool treeCompleted = false;
    bool printStart = true;

    std::cout << "Starting, wating for input (provide odom, pose 1, point 2 and start command)" << std::endl;


    BT::NodeStatus status;
    while (ros::ok())
    {
        ros::spinOnce();



        if(state.hasMove1 && state.hasMove2 && state.startCommand && treeCompleted == false)
        {
            if(printStart)
            {
                std::cout << "Starting BT:" << std::endl;
                BT::printTreeRecursively(tree.rootNode());
                printStart = false;
            }
            status = tree.tickRoot();
            //std::cout << "status: " << status << std::endl;


            if(status == BT::NodeStatus::SUCCESS)
            {
                std::cout << "BT complete" << std::endl;
                treeCompleted = true;
                // TODO: add reset function. reset all (4) bools, and allow for restart
            }
        }
        

        rate.sleep();
    }

}




void Bt_handler::initFactory(BT::BehaviorTreeFactory& factory)
{   
    factory.registerNodeType<behaviors_mission::MoveToPose>("MoveToPose");
    factory.registerNodeType<behaviors_mission::AllignRotation>("AllignRotation");
    factory.registerNodeType<behaviors_mission::LaunchShafter>("LaunchShafter");
    factory.registerNodeType<behaviors_mission::MoveToPoint>("MoveToPoint");

    //std::cout << "init nodes" << std::endl;

    return;
}

void Bt_handler::initNodes(const BT::Tree& tree)
{
    // Iterate through all the nodes and call init() if it is an Action_B
    for( auto& node: tree.nodes )
    {
        // Not a typo: it is "=", not "=="
        if(auto moveToPose = dynamic_cast<behaviors_mission::MoveToPose*>( node.get()))
        {
            moveToPose->init(&state);
        }
        if(auto allignRotation = dynamic_cast<behaviors_mission::AllignRotation*>( node.get()))
        {
            allignRotation->init(&state);
        }
        if(auto launchShafter = dynamic_cast<behaviors_mission::LaunchShafter*>( node.get()))
        {
            launchShafter->init(&state);
        }
        if(auto moveToPoint = dynamic_cast<behaviors_mission::MoveToPoint*>( node.get()))
        {
            moveToPoint->init(&state);
        }
    }
}





void Bt_handler::movePoseCB(const geometry_msgs::PoseStamped& msg)
{
    state.moveTo1 = msg.pose;
    state.hasMove1 = true;
    std::cout << "Got new goal1 pose (x,y,z): " << msg.pose.position.x << ", " << msg.pose.position.y << ", " << msg.pose.position.z << std::endl;

}
void Bt_handler::movePointCB(const geometry_msgs::PointStamped& msg)
{
    state.moveTo2 = msg.point;
    state.hasMove2 = true;
    std::cout << "Got new goal2 point (x,y,z): " << msg.point.x << ", " << msg.point.y << ", " << msg.point.z << std::endl;

}
void Bt_handler::startCommandCB(const std_msgs::Bool& msg)
{
    state.startCommand = msg.data;
    std::cout << "Got new start command" << std::endl;
}
void Bt_handler::takeoffCompleteCB(const std_msgs::String& msg)
{
    state.takeoffComplete = true;
    std::cout << "Got new shafter takeoff complete" << std::endl;

}
void Bt_handler::odomCB(const nav_msgs::Odometry& msg)
{
    state.currentPose = msg.pose.pose;
    std::cout << "Got new odom pose (x,y,z): " << state.currentPose.position.x << ", " << state.currentPose.position.y << ", " << state.currentPose.position.z << std::endl;
}
















