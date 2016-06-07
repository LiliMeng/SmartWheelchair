#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <wheelchair_common_supervisor/supervisorState.h>
#include <boost/thread.hpp>
#include <std_msgs/Int32.h>
#include <signal.h>
#include <ros/xmlrpc_manager.h>

// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;

class supervisor
{
  public:
    supervisor ();
    ~supervisor ();
    void
    publish_loop (double);
    void
    setRobotSpeed (double, double);

  private:

    void
    joyCallback (const sensor_msgs::Joy::ConstPtr&);

    void
    guiCallback (const std_msgs::Int32::ConstPtr&);

    void
    cmdCallback (const geometry_msgs::Twist::ConstPtr&);

    void
    chairJoyCallback (const sensor_msgs::Joy::ConstPtr&);

    void
    cmdRotCallback (const geometry_msgs::Twist::ConstPtr&);

    bool
    stateCallback (wheelchair_common_supervisor::supervisorState::Request&,
		   wheelchair_common_supervisor::supervisorState::Response&);

    void
    sigINTHandler (int signum);

    ros::NodeHandle nh_;

    double l_scale_, a_scale_;
    double lin_vel, ang_vel;
    double max_linear_velocity, min_linear_velocity, max_angular_velocity, min_angular_velocity;
    bool tracking_on, semi_manual_on, manual_on, auto_on, passDoor_on, followingPath_on, park_on, inverserl_on,
	chair_joy_mode_on;
    int linear_axis, angular_axis, manual_mode_button, semi_manual_mode_button, auto_mode_button, tracking_mode_button,
	pass_door_mode_button, following_path_mode_button, park_mode_button, inverserl_mode_button,
	chair_joy_mode_button;
    double passDoor_duration, park_duration;

    ros::Publisher vel_pub;
    ros::Publisher vel_joy_pub;
    ros::ServiceServer supervisorState_srv;
    ros::Subscriber joy_sub;
    ros::Subscriber gui_sub;

    ros::Subscriber cmd_sub;
    ros::Subscriber chair_joy_sub;
    ros::Subscriber cmdRot_sub;

    boost::thread* thread_;
    boost::mutex mutex_;

    geometry_msgs::Twist cmd_auto;
    geometry_msgs::Twist cmdRot_auto;
    geometry_msgs::Twist cmd_vel_chair_joy;
};

supervisor::supervisor () :
    tracking_on (0), semi_manual_on (0), manual_on (1), auto_on (0), passDoor_on (0), followingPath_on (0), lin_vel (
	0.0), ang_vel (0.0)
{
  std::string joy_sub_, cmd_vel_pub_, cmd_vel_joy_pub_, supervisorState_srv_, chair_joy_sub_;
  double publish_period;

  if (!nh_.getParam ("/supervisor/publish_period", publish_period))
    publish_period = 0.1;
  if (!nh_.getParam ("/supervisor/joy_sub_", joy_sub_))
    joy_sub_ = "joy";
  if (!nh_.getParam ("/supervisor/cmd_vel_pub_", cmd_vel_pub_))
    cmd_vel_pub_ = "cmd_vel";
  if (!nh_.getParam ("/supervisor/cmd_vel_joy_pub_", cmd_vel_joy_pub_))
    cmd_vel_pub_ = "cmd_vel_joy";
  if (!nh_.getParam ("/supervisor/chair_joy_sub_", chair_joy_sub_))
    chair_joy_sub_ = "chair_joy";
  if (!nh_.getParam ("/supervisor/supervisorState_srv", supervisorState_srv_))
    supervisorState_srv_ = "supervisor_state";

  if (!nh_.getParam ("/supervisor/linear_axis", linear_axis))
    linear_axis = 0;
  if (!nh_.getParam ("/supervisor/angular_axis", angular_axis))
    angular_axis = 1;
  if (!nh_.getParam ("/supervisor/tracking_mode_button", tracking_mode_button))
    tracking_mode_button = 4;
  if (!nh_.getParam ("/supervisor/pass_door_mode_button", pass_door_mode_button))
    pass_door_mode_button = 5;
  if (!nh_.getParam ("/supervisor/park_mode_button", park_mode_button))
    park_mode_button = 7;
  if (!nh_.getParam ("/supervisor/following_path_mode_button", following_path_mode_button))
    following_path_mode_button = 6;
  if (!nh_.getParam ("/supervisor/manual_mode_button", manual_mode_button))
    manual_mode_button = 1;
  if (!nh_.getParam ("/supervisor/semi_manual_mode_button", semi_manual_mode_button))
    semi_manual_mode_button = 2;
  if (!nh_.getParam ("/supervisor/auto_mode_button", auto_mode_button))
    auto_mode_button = 3;
  if (!nh_.getParam ("/supervisor/chair_joy_mode_button", chair_joy_mode_button))
    chair_joy_mode_button = 8;
  if (!nh_.getParam ("/supervisor/inverserl_mode_button", inverserl_mode_button))
    inverserl_mode_button = 0;

  if (!nh_.getParam ("/supervisor/scale_linear", l_scale_))
    l_scale_ = 1;
  if (!nh_.getParam ("/supervisor/scale_angular", a_scale_))
    a_scale_ = -1;
  if (!nh_.getParam ("/supervisor/max_linear_velocity", max_linear_velocity))
    max_linear_velocity = 1;
  if (!nh_.getParam ("/supervisor/min_linear_velocity", min_linear_velocity))
    min_linear_velocity = 1;
  if (!nh_.getParam ("/supervisor/max_angular_velocity", max_angular_velocity))
    max_angular_velocity = 1;
  if (!nh_.getParam ("/supervisor/min_angular_velocity", min_angular_velocity))
    min_angular_velocity = 1;

  if (!nh_.getParam ("/supervisor/passDoor_duration", passDoor_duration))
    passDoor_duration = 1.0;
  if (!nh_.getParam ("/supervisor/park_duration", park_duration))
    park_duration = 1.0;

  ///creates the velocity publisher, and tells it to publish to the /cmd_vel topic, with a queue size of 100
  vel_pub = nh_.advertise<geometry_msgs::Twist> (cmd_vel_pub_, 100);


  vel_joy_pub = nh_.advertise<geometry_msgs::Twist> (cmd_vel_joy_pub_, 0);

  supervisorState_srv = nh_.advertiseService (supervisorState_srv_, &supervisor::stateCallback, this);

  joy_sub = nh_.subscribe<sensor_msgs::Joy> (joy_sub_, 1, &supervisor::joyCallback, this);
  gui_sub = nh_.subscribe<std_msgs::Int32> ("gui_mode", 1, &supervisor::guiCallback, this);

  cmd_sub = nh_.subscribe<geometry_msgs::Twist> ("/cmd_vel_auto", 1, &supervisor::cmdCallback, this);

  cmdRot_sub = nh_.subscribe<geometry_msgs::Twist> ("/cmd_vel_rot", 1, &supervisor::cmdRotCallback, this);

  chair_joy_sub = nh_.subscribe<sensor_msgs::Joy> (chair_joy_sub_, 1, &supervisor::chairJoyCallback, this);

  thread_ = new boost::thread (boost::bind (&supervisor::publish_loop, this, publish_period));

}

supervisor::~supervisor ()
{
  if (thread_)
  {
    thread_->join ();
    delete thread_;
  }
}

bool
supervisor::stateCallback (wheelchair_common_supervisor::supervisorState::Request &req,
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

  return true;
}

void
supervisor::guiCallback (const std_msgs::Int32::ConstPtr& msg)
{
  boost::mutex::scoped_lock (mutex_);
  if (msg->data == 0)
  {
    manual_on = 1;
    auto_on = 0;
  }
  if (msg->data == 1)
  {
    manual_on = 0;
    auto_on = 1;
  }
}

void
supervisor::joyCallback (const sensor_msgs::Joy::ConstPtr& joy)
{

  boost::mutex::scoped_lock (mutex_);
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
  }
  if (joy->buttons[pass_door_mode_button])
  {
    tracking_on = 0;
    passDoor_on = 1;
    park_on = 0;
    followingPath_on = 0;
  }
  if (joy->buttons[following_path_mode_button])
  {
    tracking_on = 0;
    passDoor_on = 0;
    park_on = 0;
    followingPath_on = 1;
  }
  if (joy->buttons[park_mode_button])
  {
    tracking_on = 0;
    passDoor_on = 0;
    park_on = 1;
    followingPath_on = 0;
  }
  if (joy->buttons[inverserl_mode_button])
  {
    manual_on = 0;
    semi_manual_on = 0;
    auto_on = 0;
    inverserl_on = 1;
    tracking_on = 0;
    passDoor_on = 0;
    park_on = 0;
    followingPath_on = 0;
    chair_joy_mode_on = 0;
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
  }
}

void
supervisor::cmdCallback (const geometry_msgs::Twist::ConstPtr& velAuto)
{
  cmd_auto.linear.x = velAuto->linear.x;
  cmd_auto.angular.z = velAuto->angular.z;
}

void
supervisor::cmdRotCallback (const geometry_msgs::Twist::ConstPtr& velAuto)
{
  cmdRot_auto.linear.x = velAuto->linear.x;
  cmdRot_auto.angular.z = velAuto->angular.z;
}

void
supervisor::chairJoyCallback (const sensor_msgs::Joy::ConstPtr& velJoy)
{
  cmd_vel_chair_joy.linear.x = l_scale_ * velJoy->axes[1];
  cmd_vel_chair_joy.angular.z = a_scale_ * velJoy->axes[0];
}

void
supervisor::publish_loop (double publish_period)
{
  int count_passDoor_duration = 0, count_park_duration = 0;
  ros::Rate r (1 / publish_period);

  ROS_INFO("Start publishing ...");
  r.sleep ();
  while (ros::ok ())
  {
    ROS_DEBUG("while loop: park = %d, track = %d, door = %d", park_on, tracking_on, passDoor_on);
    // permit searching door for a lapse in time
    if (passDoor_on)
    {
      count_passDoor_duration++;
      if (passDoor_duration < count_passDoor_duration * publish_period)
      {
	// follow the path is exist
	passDoor_on = 0;
	followingPath_on = 1;
	count_passDoor_duration = 0;
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

    // publish velocity in the right topic
    /*		geometry_msgs::Twist vel;

     if(lin_vel > max_linear_velocity)
     lin_vel = max_linear_velocity;
     if(lin_vel < min_linear_velocity)
     lin_vel = min_linear_velocity;
     if(ang_vel > max_angular_velocity)
     ang_vel = max_angular_velocity;
     if(ang_vel < min_angular_velocity)
     ang_vel = min_angular_velocity;

     vel.linear.x = lin_vel;
     vel.angular.z  = ang_vel;
     */
    /*		if(semi_manual_on)
     {
     vel_joy_pub.publish(vel);
     }
     */
    if (manual_on)
    {
      setRobotSpeed (lin_vel, ang_vel);
//			vel_pub.publish(vel);
    }

    else if (auto_on)
    {
      if (cmdRot_auto.angular.z != 0)
      {
	//		vel_pub.publish(cmdRot_auto);
	setRobotSpeed (cmdRot_auto.linear.x, cmdRot_auto.angular.z);
      }
      else
      {
	setRobotSpeed (cmd_auto.linear.x, cmd_auto.angular.z);
      }
    }

    else if (chair_joy_mode_on)
    {
//			vel_pub.publish(cmd_vel_chair_joy);
      setRobotSpeed (cmd_vel_chair_joy.linear.x, cmd_vel_chair_joy.angular.z);
    }

    r.sleep ();
  }

}

void
supervisor::setRobotSpeed (double lin_vel, double ang_vel)
{
  geometry_msgs::Twist vel;

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

  vel_pub.publish (vel);
}

//Our interrupt handler will request shutdown and will ber able to gracefully quit by setting the robot speed to 0,0
void
sigINTHandler (int signum)
{
  std::cout << "Interrupt signal (" << signum << ") received.\n";

  // cleanup and close up stuff here
  // terminate program
  g_request_shutdown = 1; // Set flag
  ros::shutdown ();
}

// Replacement "shutdown" XMLRPC callback
void
shutdownCallback (XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  int num_params = 0;
  if (params.getType () == XmlRpc::XmlRpcValue::TypeArray)
    num_params = params.size ();
  if (num_params > 1)
  {
    std::string reason = params[1];
    ROS_WARN("Shutdown request received. Reason: [%s]", reason.c_str ());
    g_request_shutdown = 1; // Set flag
  }

  result = ros::xmlrpc::responseInt (1, "", 0);
}

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "supervisor", ros::init_options::NoSigintHandler);
  signal (SIGINT, sigINTHandler);
  signal (SIGTERM, sigINTHandler);
  supervisor sup;
  double publish_period = 0.1;
  ros::Rate r (1 / publish_period);
//	while (!g_request_shutdown)


  while (ros::ok ())
  {
    ///declare the message to be sent
    geometry_msgs::Twist msg;
    msg.linear.x=2;
    msg.angular.z=1.8;

    sup.vel_pub.publish(msg);



    // Do non-callback stuff
    ros::spinOnce ();
    r.sleep ();
  }

  // Do pre-shutdown tasks

  sup.setRobotSpeed (0, 0);
//  ros::shutdown();
}
