#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include "pubSysCls.h"

using namespace sFnd;

#define ACC_LIM_RPM_PER_SEC	50
#define VEL_LIM_RPM			50
#define TOTAL_CNTS_PER_REV  6400
#define TIME_TILL_TIMEOUT	10000	//The timeout used for homing(ms)

float desired_angle = 0;
int step_to_go_to =0;

void cmdAngleCallback(const std_msgs::Float64 &angle) {               
    desired_angle = angle.data;
    ROS_INFO("Desired angle %f", desired_angle);
    step_to_go_to = desired_angle * TOTAL_CNTS_PER_REV/2;
    ROS_INFO("step_to_go_to %i", step_to_go_to);
};


int main(int argc, char* argv[]){
    ros::init(argc, argv, "one_motor_move_to");
    ros::NodeHandle nodeHandle;
    ros::Subscriber angle_sub = nodeHandle.subscribe("cmd_angle", 10, &cmdAngleCallback);

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
    double timeout = myMgr.TimeStampMsec() + TIME_TILL_TIMEOUT;

    while (!motorNode.Motion.IsReady()) {
		if (myMgr.TimeStampMsec() > timeout) {
			ROS_INFO("Error: Timed out waiting for motorNode to enable\n");
			return -2;
		}
	}

    //homing node
    ROS_INFO("Now homing node");
    if (!motorNode.Motion.Homing.HomingValid()){
        ROS_INFO("Homing is not valid");
        return -3;
    };
    motorNode.Motion.Homing.Initiate();
    while (!motorNode.Motion.Homing.WasHomed()){}
    myMgr.Delay(1000);
    /*
    myMgr.Delay(15000); // wait for homing
    if (!motorNode.Motion.Homing.WasHomed()) {
        ROS_INFO("homing did not finish within 15s");
        return -4;
    };
    */

    motorNode.Motion.MoveWentDone();					//Clear the rising edge Move done register

    // motion limits
	motorNode.AccUnit(INode::RPM_PER_SEC);				//Set the units for Acceleration to RPM/SEC
	motorNode.VelUnit(INode::RPM);						//Set the units for Velocity to RPM
	motorNode.Motion.AccLimit = ACC_LIM_RPM_PER_SEC;	//Set Acceleration Limit (RPM/Sec)
	motorNode.Motion.VelLimit = VEL_LIM_RPM;			//Set Velocity Limit (RPM)
																															
	motorNode.Motion.Adv.TriggerGroup(1);               // Set the trigger group indicator

    motorNode.Motion.Adv.MovePosnStart(0,true);
    motorNode.Motion.Adv.TriggerMovesInMyGroup();

    ROS_INFO("Now looping");
    while (ros::ok()) {
        //ROS_INFO("start loop");
        ros::spinOnce();
        motorNode.Motion.MoveWentDone();

        motorNode.AccUnit(INode::RPM_PER_SEC);				//Set the units for Acceleration to RPM/SEC
	    motorNode.VelUnit(INode::RPM);						//Set the units for Velocity to RPM
	    motorNode.Motion.AccLimit = ACC_LIM_RPM_PER_SEC;	//Set Acceleration Limit (RPM/Sec)
	    motorNode.Motion.VelLimit = VEL_LIM_RPM;

        motorNode.Motion.Adv.MovePosnStart(step_to_go_to,true);
        motorNode.Motion.Adv.TriggerMovesInMyGroup();
        double timeout = myMgr.TimeStampMsec() + motorNode.Motion.MovePosnDurationMsec(step_to_go_to, true) + 100;			//define a timeout in case the node is unable to enable

		while (!motorNode.Motion.MoveIsDone()) {
			if (myMgr.TimeStampMsec() > timeout) {
				ROS_INFO("Error: Timed out waiting for move to complete");
				
				return -2;
			}
		}
        
        }
};