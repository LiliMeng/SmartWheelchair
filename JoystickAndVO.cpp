#include <stdio.h>
#include <assert.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <limits.h>
#include <vector>
#include <string>
#include <iomanip>
#include <math.h>

using namespace std;

typedef struct VOData {

    int secs;
    int nsecs;
    double global_timestamp;
    double linear_vel;
    double angular_vel;

} VOData;


typedef struct JoystickData {

    int secs;
    int nsecs;
    double global_timestamp;
    double left_right;
    double forward_backward;

} JoystickData;


double linear_interpolation(double x0, double y0, double x1, double y1, double x)
{
    double a=(y1-y0)/(x1-x0);
    double b=-a*x0+y0;
    double y=a*x+b;

    return y;

}


static void help()
{
    printf("program         VO_input_file        VO_output_file       Joy_input_file       VO_output_file       Joy_interp_file       VO_interp_file          Joy_VO_interp_file\n");
    printf("JoystickAndVO   VO_input_filename    VO_output_filename   Joy_input_filename   Joy_output_filename  Joy_interp_filename   VO_interp_filename      Joy_VO_interp_filename\n");
    printf("parameter fits to extracted Fovis Visual Odometry and wheelchair joystick Data\n");
}


int main(int argc, const char * argv[])
{
    if (argc != 8) {
        printf("argc is %d, should be 8\n", argc);
        help();
        return -1;
    }

    const char * vo_input_file= argv[1];
    const char * vo_output_file = argv[2];
    const char * joy_input_file= argv[3];
    const char * joy_output_file = argv[4];
    const char * joy_interp_file = argv[5];
    const char * VO_interp_file = argv[6];
    const char * joy_VO_file = argv[7];

   FILE *fp_VO;
   fp_VO = fopen(vo_input_file, "r");
   assert(fp_VO);
   ofstream fout_VO(vo_output_file);
   ofstream fout_VO_interp(VO_interp_file);
   vector<VOData> VOVec;

   FILE *fp_Joy;
   fp_Joy = fopen(joy_input_file, "r");
   assert(fp_Joy);
   ofstream fout_Joy(joy_output_file);
   ofstream fout_Joy_interp(joy_interp_file);
   vector<JoystickData> JoyVec;

   std::string::size_type sz;     // alias of size_t

   double frequency = 16.0;

   ofstream fout_JoyVO(joy_VO_file);

   while(!feof(fp_VO))
   {
       double global_timestamp;

       VOData single_VOData;
       char header[1024] = {NULL};
       char seq_s[1024] = {NULL};
       int seq = 0;
       char stamp_s[1024] = {NULL};
       char secs_s[1024] = {NULL};
       int secs = 0;
       char nsecs_s[1024] = {NULL};
       int nsecs = 0;
       char frame_id[1024] = {NULL};
       char odom[1024] = {NULL};
       char child_frame_id[1024] = {NULL};

       char base_link[1024]={NULL};
       char pose1[1024]={NULL};
       char pose2[1024] = {NULL};
       char position[1024]={NULL};
       char x[1024] = {NULL};
       char xcoord[1024] = {NULL};

       char y[1024] = {NULL};
       char ycoord[1024] = {NULL};

       char z[1024] = {NULL};
       char zcoord[1024] = {NULL};

       char orientation[1024] = {NULL};

       char x1[1024] = {NULL};
       char xcoord1[1024] = {NULL};

       char y1[1024] = {NULL};
       char ycoord1[1024] = {NULL};

       char z1[1024] = {NULL};
       char zcoord1[1024] = {NULL};

       char w[1024] = {NULL};
       char wcoord[1024] = {NULL};

        fscanf(fp_VO, "%s", header);
        fscanf(fp_VO, "%s %d", seq_s, &seq);
        fscanf(fp_VO, "%s", stamp_s);
        fscanf(fp_VO, "%s %d", secs_s, &secs);
        fscanf(fp_VO, "%s %d", nsecs_s, &nsecs);
        fscanf(fp_VO, "%s %s", frame_id, odom);

        fscanf(fp_VO, "%s %s", child_frame_id, base_link);


        fscanf(fp_VO, "%s %s %s", pose1, pose2, position);

         fscanf(fp_VO, "%s %s", x, xcoord);

         fscanf(fp_VO, "%s %s", y, ycoord);

         fscanf(fp_VO, "%s %s", z, zcoord);



         fscanf(fp_VO, "%s %s %s", orientation, x1, xcoord1);

         fscanf(fp_VO, "%s %s", y1, ycoord1);

         fscanf(fp_VO, "%s %s", z1, zcoord1);

         fscanf(fp_VO, "%s %s",w,wcoord);


         char cov_string[1024] = {NULL};

         for(int m = 0; m<37; m++)
         {
           fscanf(fp_VO, "%s", cov_string);

         }


         ///Extract the linear velocity

         char twist1[1024] = {NULL};
         char twist2[1024] = {NULL};
         char linear[1024] = {NULL};
         fscanf(fp_VO, "%s %s %s", twist1,twist2,linear);


         char twistx[1024] = {NULL};
         char twistXcoord[1024] = {NULL};
         char twisty[1024] = {NULL};
         char twistYcoord[1024] = {NULL};
         char twistz[1024] = {NULL};
         char twistZcoord[1024] = {NULL};
         fscanf(fp_VO, "%s %s", twistx, twistXcoord);

         fscanf(fp_VO, "%s %s", twisty, twistYcoord);

         fscanf(fp_VO, "%s %s", twistz, twistZcoord);

         char angular[1024] = {NULL};
         fscanf(fp_VO, "%s", angular);


        ///Extract the angular velocity
         char twistAngularX[1024] = {NULL};
         char twistAngularXcoord[1024] = {NULL};
         char twistAngularY[1024] = {NULL};
         char twistAngularYcoord[1024] = {NULL};
         char twistAngularZ[1024] = {NULL};
         char twistAngularZcoord[1024] = {NULL};
         fscanf(fp_VO, "%s %s", twistAngularX, twistAngularXcoord);
         fscanf(fp_VO, "%s %s", twistAngularY, twistAngularYcoord);
         fscanf(fp_VO, "%s %s", twistAngularZ, twistAngularZcoord);


         char cov_string1[1024] = {NULL};

         for(int n = 0; n<37; n++)
         {
           fscanf(fp_VO, "%s", cov_string1);
           //printf("%s", cov_string1);
         }

         char part[1024] ={NULL};
         fscanf(fp_VO, "%s", part);
        // cout<<part<<endl;
         if(seq!=0)
         {
            single_VOData.secs=secs;
            single_VOData.nsecs=nsecs;
            single_VOData.linear_vel=stod(twistXcoord, &sz);
            single_VOData.angular_vel=stod(twistAngularZcoord, &sz);

            global_timestamp = secs+nsecs*0.000000001;
            single_VOData.global_timestamp=global_timestamp;
           // cout<< std::setprecision(15) <<global_timestamp <<endl;
         //   fout_VO<<single_VOData.secs<<"   "<<single_VOData.nsecs<<" "<<single_VOData.linear_vel<<" "<<single_VOData.angular_vel<<endl;
            //cout<<single_VOData.secs<<"   "<<single_VOData.nsecs<<" "<<single_VOData.linear_vel<<" "<<single_VOData.angular_vel<<endl;
            VOVec.push_back(single_VOData);
         }

   }

   double duration_VO = VOVec[VOVec.size()-1].global_timestamp-VOVec[0].global_timestamp;

   for(int i=0; i<VOVec.size(); i++)
   {
       fout_VO<<VOVec[i].global_timestamp-VOVec[0].global_timestamp<<" "<<VOVec[i].linear_vel<<" "<<VOVec[i].angular_vel<<endl;
   }

   cout<<duration_VO<<endl;
   cout<<"rounded duration "<<floor(duration_VO)<<endl;

   vector<double> new_time_interval_VOvec;

   double new_time_interval_VO=0.0;


   for(int i=0; i<duration_VO; i++)
   {
        for(int j=0; j<16; j++)
        {
            new_time_interval_VO=i+1.0/frequency*j;
            new_time_interval_VOvec.push_back(new_time_interval_VO);
           // cout<<new_time_interval<<endl;

        }
   }

   cout<<"new_time_interval_VOvec.size() "<<new_time_interval_VOvec.size()<<endl;

   double interpolated_linear_vel=0;
   double interpolated_angular_vel=0;

   vector<VOData> interpolated_VOVec;

   /*

   for(int i=0; i<new_time_interval_VOvec.size(); i++)
   {
       VOData single_interp_VO;
       interpolated_linear_vel = linear_interpolation(0, VOVec[0].linear_vel, VOVec[i].global_timestamp-VOVec[0].global_timestamp, VOVec[i].linear_vel, new_time_interval_VOvec[i]);
       interpolated_angular_vel = linear_interpolation(0, VOVec[0].angular_vel, VOVec[i].global_timestamp-VOVec[0].global_timestamp, VOVec[i].angular_vel, new_time_interval_VOvec[i]);
       single_interp_VO.global_timestamp = new_time_interval_VOvec[i];

       if(i==0)
       {
            single_interp_VO.linear_vel = VOVec[0].linear_vel;
            single_interp_VO.angular_vel = VOVec[0].angular_vel;

       }
       else
       {
            single_interp_VO.linear_vel = interpolated_linear_vel;
            single_interp_VO.angular_vel = interpolated_angular_vel;
       }

       fout_VO_interp<<single_interp_VO.global_timestamp<<" "<<single_interp_VO.linear_vel<<" "<<single_interp_VO.angular_vel<<endl;
       cout<<single_interp_VO.global_timestamp<<" "<<single_interp_VO.linear_vel<<" "<<single_interp_VO.angular_vel<<endl;

   }*/

   while(!feof(fp_Joy))
   {
       JoystickData single_JoyData;
       char header[1024] = {NULL};
       char seq_s[1024] = {NULL};
       int seq=0;
       char stamp_s[1024] = {NULL};
       char secs_s[1024] = {NULL};
       int secs=0;
       char nsecs_s[1024] = {NULL};
       int nsecs=0;
       char frame_id[1024] = {NULL};
       char chair_joy[1024] = {NULL};
       char axes[1024] = {NULL};

       char coordinate1[1024]={NULL};
       char coordinate2[1024]={NULL};
       char buttons[1024] = {NULL};
       char braces[1024]={NULL};
       char part[1024] = {NULL};

        fscanf(fp_Joy, "%s", header);
        fscanf(fp_Joy, "%s %d", seq_s, &seq);
        fscanf(fp_Joy, "%s", stamp_s);
        fscanf(fp_Joy, "%s %d", secs_s, &secs);
        fscanf(fp_Joy, "%s %d" , nsecs_s, &nsecs);
        fscanf(fp_Joy, "%s %s", frame_id, chair_joy);
        fscanf(fp_Joy, "%s", axes);
        fscanf(fp_Joy, "%s %s", coordinate1, coordinate2);
        fscanf(fp_Joy, "%s %s", buttons, braces);
        fscanf(fp_Joy, "%s", part);

        string coord1=string(coordinate1);
        string coord2=string(coordinate2);

        for(int m=0; m<coord1.size(); m++)
        {
            if(coord1[m] == '['||coord1[m]==',')
            {
                coord1.erase(m,1);
            }
        }

        for(int n=0; n<coord2.size(); n++)
        {
            if(coord2[n] == ']')
            {
                coord2.erase(n,1);
            }
        }

        if(seq!=0)
        {

            single_JoyData.secs=secs;
            single_JoyData.nsecs=nsecs;
            single_JoyData.left_right=stod(coord1, &sz);
            single_JoyData.forward_backward=stod(coord2, &sz);
            single_JoyData.global_timestamp=secs+nsecs*0.000000001;
            JoyVec.push_back(single_JoyData);
        }
   }

   cout<<"JoyVec.size() "<<JoyVec.size()<<endl;

   double duration_Joy = JoyVec[JoyVec.size()-1].global_timestamp-JoyVec[0].global_timestamp;

   for(int i=0; i<JoyVec.size(); i++)
   {
       fout_Joy<<JoyVec[i].global_timestamp-JoyVec[0].global_timestamp<<" "<<JoyVec[i].forward_backward<<" "<<JoyVec[i].left_right<<endl;
   }

   cout<<duration_Joy<<endl;
   cout<<"rounded duration "<<floor(duration_Joy)<<endl;

   vector<double> new_time_interval_JoyVec;

   double new_time_interval_Joy=0.0;


   for(int i=0; i<duration_Joy; i++)
   {
        for(int j=0; j<16; j++)
        {
            new_time_interval_Joy=i+1.0/frequency*j;
            new_time_interval_JoyVec.push_back(new_time_interval_Joy);
           // cout<<new_time_interval<<endl;

        }
   }

   double interpolated_forward_backward=0;
   double interpolated_left_right=0;

   vector<JoystickData> interpolated_JoyVec;

   assert(new_time_interval_JoyVec.size()==new_time_interval_VOvec.size());
   for(int i=0; i<new_time_interval_JoyVec.size(); i++)
   {
       JoystickData single_interp_Joy;
       interpolated_forward_backward = linear_interpolation(0, JoyVec[0].forward_backward, JoyVec[i].global_timestamp-JoyVec[0].global_timestamp, JoyVec[i].forward_backward, new_time_interval_JoyVec[i]);
       interpolated_left_right = linear_interpolation(0, JoyVec[0].left_right, JoyVec[i].global_timestamp-JoyVec[0].global_timestamp, JoyVec[i].left_right, new_time_interval_JoyVec[i]);
       single_interp_Joy.global_timestamp = new_time_interval_JoyVec[i];
       VOData single_interp_VO;
       interpolated_linear_vel = linear_interpolation(0, VOVec[0].linear_vel, VOVec[i].global_timestamp-VOVec[0].global_timestamp, VOVec[i].linear_vel, new_time_interval_VOvec[i]);
       interpolated_angular_vel = linear_interpolation(0, VOVec[0].angular_vel, VOVec[i].global_timestamp-VOVec[0].global_timestamp, VOVec[i].angular_vel, new_time_interval_VOvec[i]);
       single_interp_VO.global_timestamp = new_time_interval_VOvec[i];


       if(i==0)
       {
            single_interp_Joy.forward_backward= JoyVec[0].forward_backward;
            single_interp_Joy.left_right = JoyVec[0].left_right;
            single_interp_VO.linear_vel = VOVec[0].linear_vel;
            single_interp_VO.angular_vel = VOVec[0].angular_vel;


       }
       else
       {
            single_interp_Joy.forward_backward = interpolated_forward_backward;
            single_interp_Joy.left_right = interpolated_left_right;
            single_interp_VO.linear_vel = interpolated_linear_vel;
            single_interp_VO.angular_vel = interpolated_angular_vel;
       }


       fout_VO_interp<<single_interp_VO.global_timestamp<<" "<<single_interp_VO.linear_vel<<" "<<single_interp_VO.angular_vel<<endl;
       cout<<single_interp_VO.global_timestamp<<" "<<single_interp_VO.linear_vel<<" "<<single_interp_VO.angular_vel<<endl;

       fout_Joy_interp<<single_interp_Joy.global_timestamp<<" "<<single_interp_Joy.forward_backward<<" "<<single_interp_Joy.left_right<<endl;
       cout<<single_interp_Joy.global_timestamp<<" "<<single_interp_Joy.forward_backward<<" "<<single_interp_Joy.left_right<<endl;

       fout_JoyVO<<single_interp_Joy.global_timestamp<<" "<<single_interp_Joy.forward_backward<<" "<<single_interp_Joy.left_right<<single_interp_VO.linear_vel<<" "<<single_interp_VO.angular_vel<<endl;


   }

   cout<<"new_time_interval_JoyVec.size() "<<new_time_interval_JoyVec.size()<<endl;

   cout<<"new_time_interval_VOvec.size() "<<new_time_interval_VOvec.size()<<endl;




   fclose(fp_VO);
   fclose(fp_Joy);


   return 0;
}

