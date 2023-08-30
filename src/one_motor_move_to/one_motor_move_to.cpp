#include "ros/ros.h"
#include <std_msgs/Float64.h>

#include "pubSysCls.h"

using namespace sFnd;

#define ACC_LIM_RPM_PER_SEC	20
#define VEL_LIM_RPM			200
#define TOTAL_CNTS_PER_REV  20000 

float desired_angle = 0;
float step_to_go_to =0;



void cmdAngleCallback(const std_msgs::Float64 &angle) {
    desired_angle = angle.data;
    step_to_go_to = desired_angle * TOTAL_CNTS_PER_REV/2;
};


int main(int argc, char* argv[]){
    ros::init(argc, argv, "one_motor_move_to");
    ros::NodeHandle nh("~");
    ros::Subscriber angle_sub = nh.subscribe("cmd_angle", 1, &cmdAngleCallback);

    // all the motor stuff was copied from sFoundation's Example-Motion.cpp
    // it builds and spits an error when run without the motor plugged in
    std::vector<std::string> comHubPorts;
    
    SysManager myMgr; 
    
    // ports
    
    SysManager::FindComHubPorts(comHubPorts); 
    ROS_INFO("Found %ld SC Hubs\n", comHubPorts.size());
    
    myMgr.ComHubPort(0, comHubPorts[0].c_str()); 	//define the first SC Hub port (port 0) to be associated 
													// with COM portnum (as seen in device manager)
    
    myMgr.PortsOpen(1);

    IPort &myPort = myMgr.Ports(0);
    ROS_INFO(" Port[%d]: state=%d, nodes=%d\n",
				myPort.NetNumber(), myPort.OpenState(), myPort.NodeCount());

    //node on port
    INode &motorNode = myPort.Nodes(0);

    motorNode.EnableReq(false);
    
    myMgr.Delay(200);

	motorNode.Status.AlertsClear();				//Clear Alerts on node 
	motorNode.Motion.NodeStopClear();	        //Clear Nodestops on Node  				
	motorNode.EnableReq(true);					//Enable node

    ROS_INFO("motorNode enabled\n");

    motorNode.Motion.MoveWentDone();					//Clear the rising edge Move done register

    // motion limits
	motorNode.AccUnit(INode::RPM_PER_SEC);				//Set the units for Acceleration to RPM/SEC
	motorNode.VelUnit(INode::RPM);						//Set the units for Velocity to RPM
	motorNode.Motion.AccLimit = ACC_LIM_RPM_PER_SEC;	//Set Acceleration Limit (RPM/Sec)
	motorNode.Motion.VelLimit = VEL_LIM_RPM;			//Set Velocity Limit (RPM)
																															
	motorNode.Motion.Adv.TriggerGroup(1);               // Set the trigger group indicator

    ROS_INFO("Now looping");
    while (ros::ok()) {
        motorNode.Motion.Adv.MovePosnStart(step_to_go_to,true);
        motorNode.Motion.Adv.TriggerMovesInMyGroup();
        ros::spinOnce();
        }
};