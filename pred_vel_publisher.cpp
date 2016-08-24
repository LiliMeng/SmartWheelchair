#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

using namespace std;


int main(int argc, char *argv[])
{
     ros::init(argc, argv, "pred_vel_publisher");
     ros::NodeHandle nh;
     string cmd_vel_pub_="cmd_vel";
     ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_pub_, 1000);

     const char* cmd="/home/lci/catkin_ws/src/nn_pwc/src/executePredict.sh";

     ros::Rate r(20);
     geometry_msgs::Twist msg;

     FILE* pipe = popen(cmd, "r");
     if (!pipe)
     {
        cout<<"cannot execute the pipe command"<<endl;

     }
     char mystring[128];




     while(!feof(pipe)&&ros::ok())
     {

        if(fgets(mystring, 128, pipe) != NULL)
        {
            //result += mystring;
            string final_output= mystring;

           ROS_INFO(mystring);
           //cout<<result<<endl;
           // istringstream iss(result);
        //   cout<<buffer.size()<<endl;
           /*
            while(iss)
            {
               string tmp;
               iss>>tmp;
               //cout<<tmp<<endl;

              double tmp1 = atof(tmp.c_str());
               msg.linear.x=tmp1;
               msg.angular.z=0;
               vel_pub.publish(msg);
               cout<<"current linear veloicty is: "<<msg.linear.x<<endl;

            }*/
           }
            ros::spinOnce();
            r.sleep();
    }

   // cout<<output.size()<<endl;

   // istringstream iss(output);



      /*
      while(ros::ok())
      {

         cout<<i<<endl;
         ros::spinOnce();
         r.sleep();


            while(iss)
            {

                //string tmp = output.substr(i, i+8);
                string tmp;
                iss>>tmp;
                cout<<tmp<<endl;

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
     }*/

    return 0;

}

