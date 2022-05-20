



#ifndef BEHAVIORS_MISSION_H
#define BEHAVIORS_MISSION_H

#include "bt_handler.hpp"

#include "tf/tf.h"


#define POINT_1_TOLERANCE 0.5
#define ANGLE_TOLERANCE 0.25
#define POINT_2_TOLERANCE 0.5






namespace behaviors_mission
{
    



class MoveToPose : public BT::StatefulActionNode
    {
        private: 
            Bt_handler::state_struct* statePtr;

            bool atPoint2D(geometry_msgs::Point point)
            {
                double error2 = pow(statePtr->currentPose.position.x - point.x, 2) + 
                                pow(statePtr->currentPose.position.y - point.y, 2);

                double tol = POINT_1_TOLERANCE;
                return error2 < pow(tol, 2);
            }

        public:

            void init(Bt_handler::state_struct* statePtr)
            {
                if(statePtr == nullptr)
                {
                    //error
                }
                this->statePtr = statePtr;
            }

            MoveToPose(const std::string& name, const BT::NodeConfiguration& config) : StatefulActionNode(name, config)
            {
            }

            static BT::PortsList providedPorts()
            {
                return{ };
            }

            BT::NodeStatus logic()
            {
                // TODO publish goal point to somewhere
                geometry_msgs::PoseStamped goalStamped;
                goalStamped.pose = statePtr->moveTo1;
                goalStamped.header.stamp = ros::Time::now();

                // publish wp to mux_path
                statePtr->muxPathWaypoint_pub.publish(goalStamped);




                // check if close enough
                if(atPoint2D(statePtr->moveTo1.position))
                {
                    std::cout << "Moved to first point completed" << std::endl;
                    return BT::NodeStatus::SUCCESS;
                }
                else
                {
                    return BT::NodeStatus::RUNNING;
                }
            }
            BT::NodeStatus onStart()
            {
                // update states
                std_msgs::String muxPath_msg;
                std_msgs::String muxController_msg;
                muxPath_msg.data = "SINGLE";
                muxController_msg.data = "PATH";

                statePtr->muxPathState_pub.publish(muxPath_msg);
                statePtr->muxControllerState_pub.publish(muxController_msg);

                return logic();
            }

            BT::NodeStatus onRunning()
            {
                return logic();
            }

            void onHalted()
            {
                //
            }
    };







class AllignRotation : public BT::StatefulActionNode
    {
        private: 
            Bt_handler::state_struct* statePtr;

            bool atRotationYaw(geometry_msgs::Quaternion q)
            {
                tf::Quaternion q_goal;
                q_goal.setX(q.x);
                q_goal.setY(q.y);
                q_goal.setZ(q.z);
                q_goal.setW(q.w);

                tf::Matrix3x3 m(q_goal);
                double goalRoll, goalPitch, goalYaw;
                m.getRPY(goalRoll, goalPitch, goalYaw);


                tf::Quaternion q_current;
                q_current.setX(statePtr->currentPose.orientation.x);
                q_current.setY(statePtr->currentPose.orientation.y);
                q_current.setZ(statePtr->currentPose.orientation.z);
                q_current.setW(statePtr->currentPose.orientation.w);

                m.setRotation(q_current);
                double currentRoll, currentPitch, currentYaw;
                m.getRPY(currentRoll, currentPitch, currentYaw);



                double error = std::abs(currentYaw - goalYaw);
                if (error > 6) error -= 2 * M_PI;
                
                //std::cout << "angle absolute error: " << error << std::endl;


                double tol = ANGLE_TOLERANCE;
                return error < tol;
            }

        public:

            void init(Bt_handler::state_struct* statePtr)
            {
                if(statePtr == nullptr)
                {
                    //error
                }
                this->statePtr = statePtr;
            }

            AllignRotation(const std::string& name, const BT::NodeConfiguration& config) : StatefulActionNode(name, config)
            {
            }

            static BT::PortsList providedPorts()
            {
                return{ };
            }

            BT::NodeStatus logic()
            {
                // TODO: publish goal pose to somewhere, fix quaternion (z = yaw, w = 1)?
                geometry_msgs::PoseStamped goalStamped;
                goalStamped.pose = statePtr->moveTo1;
                goalStamped.header.stamp = ros::Time::now();
                statePtr->muxControllerPose_pub.publish(goalStamped);


                // check if close enough
                if(atRotationYaw(statePtr->moveTo1.orientation))
                {
                    std::cout << "Allign rotation complete" << std::endl;
                    return BT::NodeStatus::SUCCESS;
                }
                else
                {
                    return BT::NodeStatus::RUNNING;
                }
            }
            BT::NodeStatus onStart()
            {
                // update states
                //std_msgs::String muxPath_msg;
                std_msgs::String muxController_msg;
                //muxPath_msg.data = "SINGLE";
                muxController_msg.data = "ROTATE";

                //statePtr->muxPathState_pub.publish(muxPath_msg);
                statePtr->muxControllerState_pub.publish(muxController_msg);


                return logic();
            }

            BT::NodeStatus onRunning()
            {
                return logic();
            }

            void onHalted()
            {
                //
            }
    };











class LaunchShafter : public BT::StatefulActionNode
    {
        private: 
            Bt_handler::state_struct* statePtr;

        public:

            void init(Bt_handler::state_struct* statePtr)
            {
                if(statePtr == nullptr)
                {
                    //error
                }
                this->statePtr = statePtr;
            }

            LaunchShafter(const std::string& name, const BT::NodeConfiguration& config) : StatefulActionNode(name, config)
            {
            }

            static BT::PortsList providedPorts()
            {
                return{ };
            }

            BT::NodeStatus logic()
            {
                // TODO: publish shafter takeoff to somewhere (if we want it to be done only once, move to onStart())
                //statePtr->takeoffComplete = false // is this needed?
                std_msgs::String str;
                str.data = "";
                statePtr->launchShafter_pub.publish(str);



                if(statePtr->takeoffComplete)
                {
                    std::cout << "shafter takeoff complete" << std::endl;
                    return BT::NodeStatus::SUCCESS;
                }
                else
                {
                    return BT::NodeStatus::RUNNING;
                }
            }
            BT::NodeStatus onStart()
            {
                // update states
                //std_msgs::String muxPath_msg;
                std_msgs::String muxController_msg;
                //muxPath_msg.data = "SINGLE";
                muxController_msg.data = "HOLD";

                //statePtr->muxPathState_pub.publish(muxPath_msg);
                statePtr->muxControllerState_pub.publish(muxController_msg);


                return logic();
            }

            BT::NodeStatus onRunning()
            {
                return logic();
            }

            void onHalted()
            {
                //
            }
    };









class MoveToPoint : public BT::StatefulActionNode
    {
        private: 
            Bt_handler::state_struct* statePtr;

            bool atPoint2D(geometry_msgs::Point point)
            {
                double error2 = pow(statePtr->currentPose.position.x - point.x, 2) + 
                                pow(statePtr->currentPose.position.y - point.y, 2);

                double tol = POINT_1_TOLERANCE;
                return error2 < pow(tol, 2);
            }

        public:

            void init(Bt_handler::state_struct* statePtr)
            {
                if(statePtr == nullptr)
                {
                    //error
                }
                this->statePtr = statePtr;
            }

            MoveToPoint(const std::string& name, const BT::NodeConfiguration& config) : StatefulActionNode(name, config)
            {
            }

            static BT::PortsList providedPorts()
            {
                return{ };
            }

            BT::NodeStatus logic()
            {
                // publish goal point to somewhere
                geometry_msgs::PoseStamped goalStamped;
                goalStamped.pose = statePtr->moveTo1;
                goalStamped.header.stamp = ros::Time::now();
                statePtr->muxPathWaypoint_pub.publish(goalStamped);
                


                // check if close enough
                if(atPoint2D(statePtr->moveTo2))
                {
                    std::cout << "Moved to second point completed" << std::endl;
                    return BT::NodeStatus::SUCCESS;
                }
                else
                {
                    return BT::NodeStatus::RUNNING;
                }
            }
            BT::NodeStatus onStart()
            {
                // update states
                std_msgs::String muxPath_msg;
                std_msgs::String muxController_msg;
                muxPath_msg.data = "SINGLE";
                muxController_msg.data = "PATH";

                statePtr->muxPathState_pub.publish(muxPath_msg);
                statePtr->muxControllerState_pub.publish(muxController_msg);


                return logic();
            }

            BT::NodeStatus onRunning()
            {
                return logic();
            }

            void onHalted()
            {
                //
            }
    };






} // namespace behaviors_mision









#endif




