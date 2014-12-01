#include "Aria.h"
#include <ros/ros.h>
#include "ArTerabotArm.h"

#include <sensor_msgs/JointState.h>

void move_arm(const sensor_msgs::JointState::ConstPtr& j);
void checker(int selectedJoint);


static float park [5]= {149, -94, 155, -149, 0}; // collapsed on back of robot
static float forwardReady [5]= {0, -40, -90, -40, 0};
static float base [5]= {0, -40, -90, -40, 0};
static float secondpose [5]= {-70, -40, -90, -40, 0};
static float thirdpose [5]= {-70, -50, -90, -40, 0};
static float forthpose [5]= {90, -40, -90, -40, 0};
static float fifthpose [5]= {90, -50, -90, -40, 0};

static float straightForward [5]= {0, -90, 0, 0, 0};
static float start [5]= {0, 0, 0, 0, 0};
static float pickUpFloor [5]= {0, -95, -76.5, -3.5, -0};
static float right [5]= {45, -54.52, -69.04, -6.93, 0};
static float left[5] = {-45, -54.52, -69.04, -6.93, 0};
static float defaultJointSpeed = 15;

//static float begin [5]= {-5, -94, 155, -149, 0}; 
ArRobot robot;
ArTerabotArm arm(&robot);
unsigned short *s = arm.getJointStatus();
int main(int argc, char **argv)
{
  Aria::init();
  ros::init(argc, argv, "teleoparm");
 
  ArLog::init(ArLog::StdErr, ArLog::Normal);
  ArArgumentParser parser(&argc, argv);
  parser.loadDefaultArguments();
  //ArRobot robot;
 
  ArRobotConnector robotConnector(&parser, &robot);
  
  if(!robotConnector.connectRobot())
  {
    ArLog::log(ArLog::Terse, "terabotArmDemo: Could not connect to the robot.");
    if(parser.checkHelpAndWarnUnparsed())
    {
        Aria::logOptions();
        Aria::exit(1);
    }
  }
  if (!Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed())
  {
    Aria::logOptions();
    Aria::exit(1);
  }
  //ArTerabotArm arm(&robot);
  ArLog::log(ArLog::Normal, "terabotArmDemo: Connected to mobile robot.");

   
  if(!arm.open())
  {
      ArLog::log(ArLog::Terse, "terabotArmDemo: Error opening serial connection to arm");
      Aria::exit(1);
  }

 
  robot.runAsync(true);

  arm.powerOn();
  arm.reset();
  arm.enable();
	
  arm.setAllJointSpeeds(defaultJointSpeed);

  ArLog::log(ArLog::Terse, "\nWARNING\n\nAbout to move the manipulator arm to the front of the robot in 5 seconds. Press Control-C to cancel and exit the program.\n...");
  ArUtil::sleep(5000);
  ArLog::log(ArLog::Terse, "Now moving the arm...");
 // arm.moveArm(forwardReady);
 //arm.moveArm(park);


  ArUtil::sleep(500); // need to have read some data from the arm for key handler to work
  robot.lock();
  
  ArModeUnguardedTeleop unguardedTeleopMode(&robot, "unguarded teleop", 'u', 'U');
  ArModeTeleop teleopMode(&robot, "teleop", 't', 'T');
  ArModeLaser laserMode(&robot, "laser", 'l', 'L');
  ArModeCommand commandMode(&robot, "direct robot commands", 'd', 'D');
 

 
  ros::NodeHandle n; 
  sensor_msgs::JointState joints;
  ros::Rate loop_rate(10);
  ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("/joint_states", 50);//50 1000
  ros::Subscriber arm_sub_ = n.subscribe<sensor_msgs::JointState>("/joint_states", 10, move_arm);//it was 100

float positions[5];


// float park1 [5]= {50, -95,-50, -149, 0};
 float park1 [5]= {149, -94, 155, -149, 0};//this is th real park
float initial [5]= {-30.0, 0.0, 0.0, 0.0, 0.0};

// std::cout<<"Gripper Status before:"<<arm.getGripperStatus()<<"\n";
// std::cout<<"Gripper Status before:"<<arm.getGripperStatus()<<"\n";
// std::cout<<"Gripper Status before:"<<arm.getGripperStatus()<<"\n";
// std::cout<<"Gripper Status before:"<<arm.getGripperStatus()<<"\n";
// arm.grip(-100);
// std::cout<<"Gripper Status after:"<<arm.getGripperStatus()<<"\n";
// std::cout<<"Gripper Status after:"<<arm.getGripperStatus()<<"\n";
// std::cout<<"Gripper Status after:"<<arm.getGripperStatus()<<"\n";
// std::cout<<"Gripper Status after:"<<arm.getGripperStatus()<<"\n";

  //*********testing**************************
 // arm.moveArm(start);
  //arm.closeGripper();
   // arm.moveArm(park1); 
  arm.moveArm(forwardReady);// used when the manual control GUI is used

  ArUtil::sleep(20000);// time for the non blocking predefined movements to take place
  //******************************************
  while(ros::ok())
  {
	joints.name.push_back("j1");
        joints.name.push_back("j2");
	joints.name.push_back("j3");
        joints.name.push_back("j4");
	joints.name.push_back("j5");
         
	// why is there no joint 
	//joints.position.push_back(11);//<<
	 joints.position.clear();
         joints.position.push_back(positions[0]);
         joints.position.push_back(positions[1]);
	 joints.position.push_back(positions[2]);
         joints.position.push_back(positions[3]);
         joints.position.push_back(positions[4]);
        joint_pub.publish(joints);
	//arm.getJointStatus(&j1,&j2,&j3,&j4,&j5);
        arm.getArmPos(positions);
	// std::cout<<"j1:"<<j1<<"j2:"<<j2<<"j3:"<<j3<<"j4:"<<j4<<"j5:"<<j5<<"\n"; fflush(stdout);
	std::cout<<"position1:"<<positions[0]<<"position2:"<<positions[1]<<"position3:"<<positions[2]<<"position4:"<<positions[3]<<"position5:"<<positions[4]<<"\n"; fflush(stdout);
        //ros::spin(); 
        ros::spinOnce();
         loop_rate.sleep();
        // std::cout<<"loopend\n";
   }
  
  robot.enableMotors(); 
  robot.unlock();
  robot.waitForRunExit();


  ArLog::log(ArLog::Normal, "terabotArmDemo: Either mobile robot or arm disconnected. Exiting.");
  Aria::exit(0);
}

void move_arm(const sensor_msgs::JointState::ConstPtr& j)
{		
           std::cout<<"MOVE ARM to position:"<<j->position[0]<<"\n"; fflush(stdout);

               if(strcmp(j->name[0].c_str(),"j1")==0)
	       {
			forwardReady[0]=  j->position[0];
			arm.moveArm(forwardReady);
			
	       }	
               if(strcmp(j->name[0].c_str(),"j2")==0)
	       {
			forwardReady[1]= j->position[0];
			arm.moveArm(forwardReady);
                      
	       }
		if(strcmp(j->name[0].c_str(),"j3")==0)
	       {
			forwardReady[2]= j->position[0];
			arm.moveArm(forwardReady);
                    
	       }
		if(strcmp(j->name[0].c_str(),"j4")==0)
	       {
			forwardReady[3]= j->position[0];
			arm.moveArm(forwardReady);
                     
	       }
		if(strcmp(j->name[0].c_str(),"j5")==0)
	       {
			forwardReady[4]= j->position[0];
			arm.moveArm(forwardReady);
                      
	       }		
		if(strcmp(j->name[0].c_str(),"g")==0)
	       {
			if(j->position[0] == 0)
			{
			arm.openGripper();
			}

			else
			arm.closeGripper();	       
		}	
		if(strcmp(j->name[0].c_str(),"grasp")==0)
	       {
        		std::cout<<"grasp\n";
  			arm.moveArm(base);
			ArUtil::sleep(3000);
			arm.openGripper();
                        ArUtil::sleep(4000);
			arm.moveArm(secondpose);
			ArUtil::sleep(7000);
			arm.moveArm(thirdpose);
                        ArUtil::sleep(7000);
			arm.closeGripper();
			ArUtil::sleep(9000);
                        arm.moveArm(forthpose);
			ArUtil::sleep(11000);
                        arm.moveArm(fifthpose);
			ArUtil::sleep(3000);
			arm.openGripper();	       
	       }		

               
}


