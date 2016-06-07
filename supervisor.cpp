#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <wheelchair_common_supervisor/supervisorState.h>
#include <wheelchair_common_shared_autonomy/SharedAutonomy.h>
#include <boost/thread.hpp>
#include <std_msgs/Int32.h>
#include <signal.h>
#include <ros/xmlrpc_manager.h>
#include <vector>

using namespace std;

// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;


class readData{
public:
    readData(string filename);
    vector<vector<double> > allDataPointsVec;
    vector<double> time;
    vector<double> fb_joy;
    vector<double> lr_joy;
    vector<double> linear_vel;
    vector<double> angular_vel;

};

readData::readData(string filename)
{
    std::ifstream fin(filename.c_str(), std::ios::in);
    if(!fin.is_open())
    {
        cout<<"cannot open file"<<endl;
    }

    istringstream istr;

    double oneDimension;
    vector<double> dataPointVec;
    string str;

     while(getline(fin,str))
     {
        istr.str(str);
        while(istr>>oneDimension)
        {
            dataPointVec.push_back(oneDimension);
        }
        allDataPointsVec.push_back(dataPointVec);
        dataPointVec.clear();
        istr.clear();
        str.clear();
     }
     fin.close();

    int numOfDimensions=allDataPointsVec[0].size();
    int numOfElements=allDataPointsVec.size();

    for(int i=0; i<numOfElements; i++)
    {
        for(int j=0; j<numOfDimensions; j++)
        {
            cout<<"The joystick and VO values "<<i<<" "<<j<<"t"<<setprecision(20)<<"value is "<<allDataPointsVec[i][j]<<endl;
            time.push_back(allDataPointsVec[i][0]);
            fb_joy.push_back(allDataPointsVec[i][1]);
            lr_joy.push_back(allDataPointsVec[i][2]);
            linear_vel.push_back(allDataPointsVec[i][3]);
            angular_vel.push_back(allDataPointsVec[i][4]);
        }

    }

}




class supervisor
{
public:
	supervisor();
	~supervisor();
	void
	publish_loop(double);
	void
	setRobotSpeed(double, double);
	ros::Publisher vel_pub;

private:

	void
	joyCallback(const sensor_msgs::Joy::ConstPtr&);

	void
	guiCallback(const std_msgs::Int32::ConstPtr&);

	void
	cmdCallback(const geometry_msgs::Twist::ConstPtr&);

	void
	cmdImitationCallback(const geometry_msgs::Twist::ConstPtr&);

	void
	chairJoyCallback(const sensor_msgs::Joy::ConstPtr&);

	void
	cmdRotCallback(const geometry_msgs::Twist::ConstPtr&);

	bool
	stateCallback(wheelchair_common_supervisor::supervisorState::Request&,
			wheelchair_common_supervisor::supervisorState::Response&);

	void
	sigINTHandler(int signum);

	ros::NodeHandle nh_;

	double l_scale_, a_scale_;
	double lin_vel, ang_vel;
	double max_linear_velocity, min_linear_velocity, max_angular_velocity, min_angular_velocity;
	bool tracking_on, semi_manual_on, manual_on, auto_on, passDoor_on, followingPath_on,
	park_on, inverserl_on, chair_joy_mode_on, imitation_mode_on, sharedAut_on;
	int linear_axis, angular_axis, manual_mode_button, semi_manual_mode_button,
	auto_mode_button, tracking_mode_button, pass_door_mode_button,
	following_path_mode_button, park_mode_button, inverserl_mode_button,
	chair_joy_mode_button, imitation_mode_button, sharedAut_mode_button;
	double passDoor_duration, park_duration;

	//ros::Publisher vel_pub;
	ros::Publisher vel_joy_pub;
	ros::ServiceServer supervisorState_srv;
	ros::Subscriber joy_sub;
	ros::Subscriber gui_sub;

	ros::Subscriber cmd_sub;
	ros::Subscriber chair_joy_sub;
	ros::Subscriber imitation_sub;
	ros::Subscriber cmdRot_sub;

	boost::thread* thread_;
	boost::mutex mutex_;

	geometry_msgs::Twist cmd_auto;
	geometry_msgs::Twist cmd_vel_imitation;
	geometry_msgs::Twist cmdRot_auto;
	geometry_msgs::Twist cmd_vel_chair_joy;
	ros::Time lastMessageReceivedTime;
	ros::Duration timeoutDuration;

};

supervisor::supervisor() :
																		tracking_on(0), semi_manual_on(0), manual_on(1), auto_on(0), passDoor_on(0), followingPath_on(
																				0), sharedAut_on(0), lin_vel(0.0), ang_vel(0.0)
{
	std::string joy_sub_, cmd_vel_pub_, cmd_vel_joy_pub_, supervisorState_srv_, chair_joy_sub_,
	cmd_vel_imitation_sub;

	double publish_period, timeoutPeriod;

	if (!nh_.getParam("/supervisor/publish_period", publish_period))
		publish_period = 0.1;
	if (!nh_.getParam("/supervisor/joy_sub_", joy_sub_))
		joy_sub_ = "joy";
	if (!nh_.getParam("/supervisor/cmd_vel_pub_", cmd_vel_pub_))
		cmd_vel_pub_ = "cmd_vel";
	if (!nh_.getParam("/supervisor/cmd_vel_joy_pub_", cmd_vel_joy_pub_))
		cmd_vel_pub_ = "cmd_vel_joy";
	if (!nh_.getParam("/supervisor/chair_joy_sub_", chair_joy_sub_))
		chair_joy_sub_ = "chair_joy";

	if (!nh_.getParam("/supervisor/cmd_vel_imitation_sub_", cmd_vel_imitation_sub))
		cmd_vel_imitation_sub = "cmd_vel_imitation";

	if (!nh_.getParam("/supervisor/supervisorState_srv", supervisorState_srv_))
		supervisorState_srv_ = "supervisor_state";

	if (!nh_.getParam("/supervisor/linear_axis", linear_axis))
		linear_axis = 0;
	if (!nh_.getParam("/supervisor/angular_axis", angular_axis))
		angular_axis = 1;
	if (!nh_.getParam("/supervisor/tracking_mode_button", tracking_mode_button))
		tracking_mode_button = 4;
	if (!nh_.getParam("/supervisor/pass_door_mode_button", pass_door_mode_button))
		pass_door_mode_button = 5;
	if (!nh_.getParam("/supervisor/park_mode_button", park_mode_button))
		park_mode_button = 7;
	if (!nh_.getParam("/supervisor/following_path_mode_button", following_path_mode_button))
		following_path_mode_button = 6;
	if (!nh_.getParam("/supervisor/manual_mode_button", manual_mode_button))
		manual_mode_button = 1;
	if (!nh_.getParam("/supervisor/semi_manual_mode_button", semi_manual_mode_button))
		semi_manual_mode_button = 2;
	if (!nh_.getParam("/supervisor/auto_mode_button", auto_mode_button))
		auto_mode_button = 3;
	if (!nh_.getParam("/supervisor/chair_joy_mode_button", chair_joy_mode_button))
		chair_joy_mode_button = 8;
	//	if (!nh_.getParam ("/supervisor/inverserl_mode_button", inverserl_mode_button))
	//		inverserl_mode_button = 0;
	if (!nh_.getParam("/supervisor/imitation_mode_button", imitation_mode_button))
		imitation_mode_button = 0;
	if (!nh_.getParam("/supervisor/sharedAut_mode_button", sharedAut_mode_button))
		sharedAut_mode_button = 9;
	if (!nh_.getParam("/supervisor/scale_linear", l_scale_))
		l_scale_ = 1;
	if (!nh_.getParam("/supervisor/scale_angular", a_scale_))
		a_scale_ = -1;
	if (!nh_.getParam("/supervisor/max_linear_velocity", max_linear_velocity))
		max_linear_velocity = 1;
	if (!nh_.getParam("/supervisor/min_linear_velocity", min_linear_velocity))
		min_linear_velocity = 1;
	if (!nh_.getParam("/supervisor/max_angular_velocity", max_angular_velocity))
		max_angular_velocity = 1;
	if (!nh_.getParam("/supervisor/min_angular_velocity", min_angular_velocity))
		min_angular_velocity = 1;

	if (!nh_.getParam("/supervisor/passDoor_duration", passDoor_duration))
		passDoor_duration = 1.0;
	if (!nh_.getParam("/supervisor/park_duration", park_duration))
		park_duration = 1.0;

    ///crates the publisher, and tells it to publish to the cmd_vel_pub_ topic, with a queue size of 100
	vel_pub = nh_.advertise<geometry_msgs::Twist>(cmd_vel_pub_, 0);

	vel_joy_pub = nh_.advertise<geometry_msgs::Twist>(cmd_vel_joy_pub_, 0);

	supervisorState_srv = nh_.advertiseService(supervisorState_srv_, &supervisor::stateCallback,
			this);

	joy_sub = nh_.subscribe<sensor_msgs::Joy>(joy_sub_, 1, &supervisor::joyCallback, this);
	gui_sub = nh_.subscribe<std_msgs::Int32>("gui_mode", 1, &supervisor::guiCallback, this);

	cmd_sub = nh_.subscribe<geometry_msgs::Twist>("/cmd_vel_auto", 1, &supervisor::cmdCallback,
			this);

	cmdRot_sub = nh_.subscribe<geometry_msgs::Twist>("/cmd_vel_rot", 1, &supervisor::cmdRotCallback,
			this);

	chair_joy_sub = nh_.subscribe<sensor_msgs::Joy>(chair_joy_sub_, 1,
			&supervisor::chairJoyCallback, this);

	imitation_sub = nh_.subscribe<geometry_msgs::Twist>(cmd_vel_imitation_sub, 1,
			&supervisor::cmdImitationCallback, this);

	//thread_ = new boost::thread(boost::bind(&supervisor::publish_loop, this, publish_period));

	//timeoutDuration = ros::Duration(publish_period);
}

supervisor::~supervisor()
{
	if (thread_)
	{
		thread_->join();
		delete thread_;
	}
}

bool supervisor::stateCallback(wheelchair_common_supervisor::supervisorState::Request &req,
		wheelchair_common_supervisor::supervisorState::Response &res)
{
	res.manual_mode = manual_on;
	res.semiManual_mode = semi_manual_on;
	res.auto_mode = auto_on;
	res.tracking_mode = tracking_on;
	res.passDoor_mode = passDoor_on;
	res.followingPath_mode = followingPath_on;
	res.park_mode = park_on;
	res.inverserl_mode = inverserl_on;
	res.chair_joy_mode = chair_joy_mode_on;
	res.sharedAut_mode = sharedAut_on;
	res.imitation_mode = imitation_mode_on;

	return true;
}

void supervisor::guiCallback(const std_msgs::Int32::ConstPtr& msg)
{
	boost::mutex::scoped_lock(mutex_);
	if (msg->data == 0)
	{
		manual_on = 0;
		semi_manual_on = 0;
		auto_on = 0;
		tracking_on = 0;
		passDoor_on = 0;
		park_on = 0;
		followingPath_on = 0;
		inverserl_on = 0;
		chair_joy_mode_on = 1;
		sharedAut_on = 0;
		imitation_mode_on = 0;
	}
	if (msg->data == 1)
	{
		manual_on = 0;
		semi_manual_on = 0;
		auto_on = 1;
		tracking_on = 0;
		passDoor_on = 0;
		park_on = 0;
		followingPath_on = 0;
		chair_joy_mode_on = 0;
		sharedAut_on = 0;
		imitation_mode_on = 0;
	}
}

void supervisor::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	boost::mutex::scoped_lock(mutex_);
	lin_vel = l_scale_ * joy->axes[linear_axis];
	ang_vel = a_scale_ * joy->axes[angular_axis];

	if (joy->buttons[manual_mode_button])
	{
		manual_on = 1;
		semi_manual_on = 0;
		auto_on = 0;
		tracking_on = 0;
		passDoor_on = 0;
		park_on = 0;
		followingPath_on = 0;
		inverserl_on = 0;
		chair_joy_mode_on = 0;
		sharedAut_on = 0;
		imitation_mode_on = 0;
	}
	if (joy->buttons[semi_manual_mode_button])
	{
		manual_on = 0;
		semi_manual_on = 1;
		auto_on = 0;
		tracking_on = 0;
		passDoor_on = 0;
		park_on = 0;
		followingPath_on = 0;
		inverserl_on = 0;
		chair_joy_mode_on = 0;
		sharedAut_on = 0;
		imitation_mode_on = 0;
	}
	if (joy->buttons[auto_mode_button])
	{
		manual_on = 0;
		semi_manual_on = 0;
		auto_on = 1;
		tracking_on = 0;
		passDoor_on = 0;
		park_on = 0;
		followingPath_on = 0;
		chair_joy_mode_on = 0;
		sharedAut_on = 0;
		imitation_mode_on = 0;
	}
	if (joy->buttons[tracking_mode_button])
	{
		manual_on = 0;
		semi_manual_on = 0;
		auto_on = 0;
		tracking_on = 1;
		passDoor_on = 0;
		park_on = 0;
		followingPath_on = 1;
		inverserl_on = 0;
		chair_joy_mode_on = 0;
		sharedAut_on = 0;
		imitation_mode_on = 0;
	}
	if (joy->buttons[pass_door_mode_button])
	{
		tracking_on = 0;
		passDoor_on = 1;
		park_on = 0;
		followingPath_on = 0;
		sharedAut_on = 0;
		imitation_mode_on = 0;
	}
	if (joy->buttons[following_path_mode_button])
	{
		tracking_on = 0;
		passDoor_on = 0;
		park_on = 0;
		followingPath_on = 1;
		sharedAut_on = 0;
		imitation_mode_on = 0;
	}
	if (joy->buttons[park_mode_button])
	{
		tracking_on = 0;
		passDoor_on = 0;
		park_on = 1;
		followingPath_on = 0;
		sharedAut_on = 0;
		imitation_mode_on = 0;
	}
	if (joy->buttons[imitation_mode_button])
	{
		manual_on = 0;
		semi_manual_on = 0;
		auto_on = 0;
		inverserl_on = 0;
		tracking_on = 0;
		passDoor_on = 0;
		park_on = 0;
		followingPath_on = 0;
		chair_joy_mode_on = 0;
		sharedAut_on = 0;
		imitation_mode_on = 1;
	}
	if (joy->buttons[chair_joy_mode_button])
	{
		manual_on = 0;
		semi_manual_on = 0;
		auto_on = 0;
		inverserl_on = 0;
		tracking_on = 0;
		passDoor_on = 0;
		park_on = 0;
		followingPath_on = 0;
		chair_joy_mode_on = 1;
		sharedAut_on = 0;
		imitation_mode_on = 0;
	}
	if (joy->buttons[sharedAut_mode_button])
	{
		manual_on = 0;
		semi_manual_on = 0;
		auto_on = 0;
		inverserl_on = 0;
		tracking_on = 0;
		passDoor_on = 0;
		park_on = 0;
		followingPath_on = 0;
		chair_joy_mode_on = 0;
		sharedAut_on = 1;
		imitation_mode_on = 0;
	}

	if (manual_on)
	{
		lastMessageReceivedTime = ros::Time::now();
	}
}

void supervisor::cmdCallback(const geometry_msgs::Twist::ConstPtr& velAuto)
{
	boost::mutex::scoped_lock(mutex_);
	cmd_auto.linear.x = velAuto->linear.x;
	cmd_auto.angular.z = velAuto->angular.z;
	if (auto_on || sharedAut_on)
	{
		lastMessageReceivedTime = ros::Time::now();
	}
}

void supervisor::cmdImitationCallback(const geometry_msgs::Twist::ConstPtr& velImitation)
{
	boost::mutex::scoped_lock(mutex_);
	cmd_vel_imitation.linear.x = velImitation->linear.x;
	cmd_vel_imitation.angular.z = velImitation->angular.z;
	if (imitation_mode_on)
	{
		lastMessageReceivedTime = ros::Time::now();
	}
}

void supervisor::cmdRotCallback(const geometry_msgs::Twist::ConstPtr& velAuto)
{
	boost::mutex::scoped_lock(mutex_);
	cmdRot_auto.linear.x = velAuto->linear.x;
	cmdRot_auto.angular.z = velAuto->angular.z;
	if (auto_on && cmdRot_auto.angular.z != 0)
	{
		lastMessageReceivedTime = ros::Time::now();
	}
}

void supervisor::chairJoyCallback(const sensor_msgs::Joy::ConstPtr& velJoy)
{
	boost::mutex::scoped_lock(mutex_);
	cmd_vel_chair_joy.linear.x = l_scale_ * velJoy->axes[1];
	cmd_vel_chair_joy.angular.z = a_scale_ * velJoy->axes[0];
	if (chair_joy_mode_on)
	{
		lastMessageReceivedTime = ros::Time::now();
	}
}

void supervisor::publish_loop(double publish_period)
{
    int count_passDoor_duration = 0, count_park_duration = 0;
    ros::Rate r(1/publish_period);

    ROS_INFO("start publishing ...");
    r.sleep();

    while(ros::ok())
    {
        ROS_DEBUG("while loop: park = %d, track = %d, door =%d", park_on, tracking_on,
                passDoor_on);

        // permit searching door for a lapse in time
        if(passDoor_on)
        {
            count_passDoor_duration++;
            if(passDoor_duration < count_passDoor_duration * publish_period)
            {
                //follow the path is exist
                passDoor_on =0;
                followingPath_on =1;
                count_passDoor_duration =0;
            }
        }

		// permit praking for a lapse in time
		if (park_on)
		{
			count_park_duration++;
			if (park_duration < count_park_duration * publish_period)
			{
				// follow the path is exist
				park_on = 0;
				followingPath_on = 1;
				count_park_duration = 0;
			}
		}

		if (manual_on)
		{
			setRobotSpeed(lin_vel, ang_vel);
			POLICY_UPDATE = true;
		}
		else if (auto_on)
		{
			if (cmdRot_auto.angular.z != 0)
			{
				setRobotSpeed(cmdRot_auto.linear.x, cmdRot_auto.angular.z);
				POLICY_UPDATE = true;
			}
			else
			{
				setRobotSpeed(cmd_auto.linear.x, cmd_auto.angular.z);
				POLICY_UPDATE = true;
			}
		}

		else if (chair_joy_mode_on)
		{
			setRobotSpeed(cmd_vel_chair_joy.linear.x, cmd_vel_chair_joy.angular.z);
			POLICY_UPDATE = true;
		}

		else if (sharedAut_on)
		{
			//Beginning of the shared autonomy
			tf::TransformListener tfListener;
			if (POLICY_UPDATE)
			{ //if we enter this mode for the first time
				ROS_INFO("Starting Shared_Autonomy");
				SharedAutonomy::Policy->Initialize(SharedAutonomy::object_targets,SharedAutonomy::targets_dist, tfListener);
				SharedAutonomy::action_control.linear.x = SharedAutonomy::human_control.linear.x;
				SharedAutonomy::action_control.angular.z = SharedAutonomy::human_control.angular.z;
				POLICY_UPDATE = false;
			}

			// The user puts/changes a goal by himself
			if (GOAL_UPDATE)
			{
				setRobotSpeed(0,0);
				SharedAutonomy::Policy->ClearPolicy();
				SharedAutonomy::Policy->Initialize(SharedAutonomy::object_targets,SharedAutonomy::targets_dist, tfListener);
				SharedAutonomy::action_control.linear.x = SharedAutonomy::human_control.linear.x;
				SharedAutonomy::action_control.angular.z = SharedAutonomy::human_control.angular.z;
				ROS_INFO("Shared Autonomy: UPDATING GOAL");
				GOAL_UPDATE = false;
			}
			else
			{
				geometry_msgs::PoseStamped curr_pose =
						shared_autonomy::NavigationInstance::path_fun->getCurrentRobotPose(
								tfListener);

				//Prediction phase of the shared autonomy
				SharedAutonomy::Policy->UpdatePrediction(curr_pose,
						SharedAutonomy::action_control, SharedAutonomy::human_control);
				//Assistance phase of the shared autonomy
				shared_autonomy::NavigationInstance::ActionVector action;
				action = SharedAutonomy::Policy->AssistedAction(SharedAutonomy::object_targets);
				SharedAutonomy::action_control.linear.x = action.linear.x;
				SharedAutonomy::action_control.angular.z = action.angular.z;

				// Publish the velocity
				setRobotSpeed(SharedAutonomy::action_control.linear.x, SharedAutonomy::action_control.angular.z);
			}
		}

		else if (imitation_mode_on)
		{
			setRobotSpeed(cmd_vel_imitation.linear.x, cmd_vel_imitation.angular.z);
		}

		r.sleep();

	}

}

void supervisor::setRobotSpeed(double lin_vel, double ang_vel)
{
	boost::mutex::scoped_lock(mutex_);
	geometry_msgs::Twist vel;
	double timeFromLastMessage = ros::Time::now().toSec() - lastMessageReceivedTime.toSec();

	/*if (timeFromLastMessage > timeoutDuration.toSec())
	 {
	 ROS_WARN("Last received speed message was %.2f seconds ago. The oldest allowed time is %.2f seconds. Stopping. LastMessageReceivedTime: %.2f", timeFromLastMessage, timeoutDuration.toSec(), lastMessageReceivedTime.toSec());
	 //		vel.linear.x = 0;
	 //		vel.angular.z = 0;
	 return;
	 }*/
	if (lin_vel > max_linear_velocity)
		lin_vel = max_linear_velocity;
	else if (lin_vel < min_linear_velocity)
		lin_vel = min_linear_velocity;
	if (ang_vel > max_angular_velocity)
		ang_vel = max_angular_velocity;
	else if (ang_vel < min_angular_velocity)
		ang_vel = min_angular_velocity;

	vel.linear.x = lin_vel;
	vel.angular.z = ang_vel;
	vel_pub.publish(vel);
}

//Our interrupt handler will request shutdown and will ber able to gracefully quit by setting the robot speed to 0,0
void sigINTHandler(int signum)
{
	std::cout << "Interrupt signal (" << signum << ") received.\n";

	// cleanup and close up stuff here
	// terminate program
	g_request_shutdown = 1; // Set flag
	ros::shutdown();

}

// Replacement "shutdown" XMLRPC callback
void shutdownCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
	int num_params = 0;

	if (params.getType() == XmlRpc::XmlRpcValue::TypeArray)
		num_params = params.size();
	if (num_params > 1)
	{
		std::string reason = params[1];
		ROS_WARN("Shutdown request received. Reason: [%s]", reason.c_str());
		g_request_shutdown = 1; // Set flag
	}

	result = ros::xmlrpc::responseInt(1, "", 0);
}

int main(int argc, char** argv)
{

	ros::init(argc, argv, "supervisor", ros::init_options::NoSigintHandler);
	signal(SIGINT, sigINTHandler);
	signal(SIGTERM, sigINTHandler);
	ros::NodeHandle nh;
	supervisor sup;

	//Path_functions and Shared_autonomy
	tf::TransformListener tfListener;
	shared_autonomy::NavigationInstance::path_fun.reset(new PathFunctions(nh, tfListener));
	// initializeServiceMoveBase random seed
	srand(time(NULL));
	// create a rviz marker to act as the fake goal
	SharedAutonomy::initializeMarker();
	shared_autonomy::NavigationInstance::path_fun->initializeServiceMoveBase();
	SharedAutonomy sh_aut(nh);
	SharedAutonomy::Policy.reset(new shared_autonomy::HOP<shared_autonomy::NavigationInstance>(nh));

	double publish_period = 0.0625;
	ros::Rate r(1 / publish_period);

	//	while (!g_request_shutdown)

	string data_loc="/home/lci/workspace/wheelchairTest/velocity1.txt";
    readData read_data(data_loc);


    geometry_msgs::Twist msg;





   // while (ros::ok())
      double distance=0;
       while(ros::ok())
       {

            for(int i=0; i<read_data.linear_vel.size(); i++)
            {
                msg.linear.x = read_data.linear_vel[i];
                distance= distance+ msg.linear.x *publish_period;
                cout<<"travelled distance is "<<distance<<endl;
                //msg.linear.x=2;
                msg.angular.z=0;
                sup.vel_pub.publish(msg);
                ros::spinOnce();
                r.sleep();
            }
            break;
        }

       std::cout<<" The total distance travelled "<<distance<<endl;

       while(ros::ok())
       {
            msg.linear.x=0;
            sup.vel_pub.publish(msg);
            ros::spinOnce();
            r.sleep();
       }

        //msg.linear.x=2;

        // Do non-callback stuff
        //ros::spinOnce();
        //r.sleep();
   //}
	// Do pre-shutdown tasks

	sup.setRobotSpeed(0, 0);
	//  ros::shutdown();
}
