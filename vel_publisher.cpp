#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

using namespace std;

std::string exec(char* cmd) {
    FILE* pipe = popen(cmd, "r");
    if (!pipe) return "ERROR";
    char buffer[128];
    std::string result = "";
    while(!feof(pipe)) {
        if(fgets(buffer, 128, pipe) != NULL)
            result += buffer;
    }
    pclose(pipe);
    return result;
}




int main(int argc, char *argv[])
{
     ros::init(argc, argv, "vel_publisher");
     ros::NodeHandle nh;
     string cmd_vel_pub_="cmd_vel";
     ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_pub_, 1000);

    string output=exec("/home/lci/catkin_ws/src/nn_pwc/src/executePredict.sh");

    cout<<output<<endl;

    istringstream iss(output);


	ros::Rate r(5);


    geometry_msgs::Twist msg;


      while(ros::ok())
      {
            int i=0;

            while(i<7710&&iss)
            {

                //string tmp = output.substr(i, i+8);
                string tmp;
                iss>>tmp;
                double tmp1 = atof(tmp.c_str());
                msg.linear.x=tmp1;
                msg.angular.z=0;
                vel_pub.publish(msg);
                cout<<"current linear veloicty is: "<<msg.linear.x<<endl;
                i=i+8;
                ros::spinOnce();
                r.sleep();
            }


            ROS_INFO("Done");
     }


    return 0;

}
