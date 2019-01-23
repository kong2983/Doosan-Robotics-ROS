#include <ros/ros.h>
#include <signal.h>

#include "dsr_robot.h"
#include <dsr_msgs/GripperMove.h>
using namespace DSR_Robot;
using namespace std;



//----- set tartget robot----------------------------------------------------
string ROBOT_ID     = "dsr01";
string ROBOT_MODEL  = "m1013";
void SET_ROBOT(string id, string model) {ROBOT_ID = id; ROBOT_MODEL= model;}   
//---------------------------------------------------------------------------

int gripper_move(float width)             
{
      
    //ros::ServiceClient srvMoveJoint = nh.serviceClient<dsr_msgs::MoveJoint>("/dsr01m1013/motion/move_joint");
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient srvGripperMove = node->serviceClient<dsr_msgs::GripperMove>("/"+ROBOT_ID + ROBOT_MODEL+"/gripper/gripper_move");

    dsr_msgs::GripperMove srv;

    srv.request.maxLength = width;

    if(srvGripperMove.call(srv))
    {         
        ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
        return (srv.response.success);
    }
    else
    {    
        ROS_ERROR("Failed to call service dr_control_service : gripper_move\n");
        ros::shutdown();  
        return -1;
    }

    return 0; 
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SigHandler(int sig)
{
    // Do some custom action.
    // For example, publish a stop message to some other nodes.
  
    // All the default sigint handler does is call shutdown()
    ROS_INFO("shutdown time!");
    ROS_INFO("shutdown time!");
    ROS_INFO("shutdown time!");
    //ros::ServiceClient srvMoveStop = nh.serviceClient<dsr_msgs::MoveStop>("/dsr01m1013/motion/move_stop");

    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::Publisher pubRobotStop = node->advertise<dsr_msgs::RobotStop>("/"+ROBOT_ID +ROBOT_MODEL+"/stop",100);
    
    dsr_msgs::RobotStop msg;
    
    msg.stop_mode  = STOP_TYPE_QUICK;
    pubRobotStop.publish(msg);
    
    ros::shutdown();
}

static void thread_subscriber()
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ///ros::Subscriber subRobotState = node->subscribe("/dsr01m1013/state", 100, msgRobotState_cb);
    ///ros::spin();
    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dsr_service_test_cpp", ros::init_options::NoSigintHandler);  
    ros::NodeHandle nh("~");

    //----- set target robot --------------- 
    const string my_robot_id    = "dsr01";
    const string my_robot_model = "m1013_with_gripper";
    SET_ROBOT(my_robot_id, my_robot_model);
    CDsrRobot robot(nh, my_robot_id, my_robot_model);

    ros::Publisher  pubRobotStop = nh.advertise<dsr_msgs::RobotStop>("/"+ROBOT_ID +ROBOT_MODEL+"/stop",10);
    ///ros::Subscriber subRobotState = nh.subscribe("/dsr01m1013/state", 100, msgRobotState_cb);
    // run subscriber thread (for monitoring)
    boost::thread thread_sub(thread_subscriber);

    float p0[6]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    float p1[6]={0.0, 0.0, 90.0, 0.0, 90.0 , 0.0};                             //joint
    float p2[6]={180.0, 0.0, 90, 0.0, 90.0, 0.0};   //joint
    
    float x1[6]={0, 0, -200, 0, 0, 0}; //task
    float x2[6]={0, 0, 200, 0, 0, 0}; //task
    float velx[2]={50, 50};     // 태스크 속도를 50(mm/sec), 50(deg/sec)로 설정
    float accx[2]={100, 100};   // 태스크 가속도를 100(mm/sec2), 100(deg/sec2)로 설정

    while(ros::ok())
    {
        robot.movej(p0, 60, 30);
        robot.movej(p1, 60, 30);
        gripper_move(0.0);
        robot.movel(x1, velx, accx, 2, MOVE_MODE_RELATIVE);
        gripper_move(0.8);
        robot.movel(x2, velx, accx, 2, MOVE_MODE_RELATIVE);

        robot.movej(p2, 60, 30, 3);
        robot.movel(x1, velx, accx, 2, MOVE_MODE_RELATIVE);
        gripper_move(0.0);
        robot.movel(x2, velx, accx, 2, MOVE_MODE_RELATIVE);
    }

    ROS_INFO("dsr_service_test finished !!!!!!!!!!!!!!!!!!!!!");
    ROS_INFO("dsr_service_test finished !!!!!!!!!!!!!!!!!!!!!");
    ROS_INFO("dsr_service_test finished !!!!!!!!!!!!!!!!!!!!!");

    ///if(&robot) delete (&robot); 
    thread_sub.join();
    
    ROS_INFO("dsr_service_test finished !!!!!!!!!!!!!!!!!!!!!");
    return 0;
}
