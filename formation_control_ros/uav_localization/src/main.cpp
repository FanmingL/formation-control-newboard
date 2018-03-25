#include "ros/ros.h"
#include "std_msgs/String.h"
#include "uav_msgs/uav_position.h"
#include "std_msgs/Header.h"
#include <ros/ros.h>
#include <stdio.h>
#include <errno.h>
#include "apriltag.h"
#include "tag36h11.h"
#include "tag36h10.h"
#include "tag36artoolkit.h"
#include "tag25h9.h"
#include "tag25h7.h"
#include "common/getopt.h"
#include "common/homography.h"
#include <cmath>
#include <string>
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "withrobot_camera.hpp"
#include <iostream>
#include <queue>
#include <thread>
 
#ifndef PI
const double PI = 3.14159265358979323846;
#endif
#define d0 5.60

const double TWOPI = 2.0*PI;
inline double standardRad(double t);
void wRo_to_euler(const matd_t *pose, double& yaw, double& pitch, double& roll);
bool FLAG=false;
double x_center[300],y_center[300];
//void imageCallback(const sensor_msgs::ImageConstPtr& msg){
 //   cv_bridge::toCvShare(msg, "mono8")->image.copyTo(gray);
//    FLAG=true;
//}
double abs(double x){
	return (x>0)?x:-x;
}
    cv::Mat srcImg(cv::Size(320, 240), CV_8UC2);
    cv::Mat gray(cv::Size(320, 240), CV_8UC1);
    cv::Mat con = (cv::Mat_<double>(4, 1) << -0.4632, 0.2662, 0.0, 0.0);
    cv::Mat iner = (cv::Mat_<double>(3, 3) << 330.7671, 0, 158.8534, 0, 330.6081, 109.9210, 0, 0, 1.0000);
    //     cv::Mat con = (cv::Mat_<double>(4,1)<<-0.4267,0.2130,0.0,0.0);
     //   cv::Mat iner = (cv::Mat_<double>(3,3)<<660.3323,0,304.6620,0,655.4458,243.3709,0,0,1);
	cv::Mat map1, map2;
 const char* devPath = "/dev/video0";
    Withrobot::Camera camera(devPath);
Withrobot::camera_format camFormat;

//ros::Rate rate2(120);
int RefreshFlag=0;
    double yaw,pitch,roll,an_X,an_Y,an_Z;
    //const double internum[4]={660.3323,655.4458,304.6620,243.3709};
    const double internum[4]={331.1390, 327.2666,161.2342,128.4313};
    matd_t *pose_end,*pose_id_old;
    int id,id_old=-1000;
    bool id_old_flag=false;
    std::queue<double>x_reco,y_reco;
   // std::queue<ros::Time>time_reco;
    double x_old[300],y_old[300];
    getopt_t *getopt_temp = getopt_create();
    apriltag_family_t *tf = NULL;
    uav_msgs::uav_position msg;
    ros::Publisher  pub;
    apriltag_detector_t *td = apriltag_detector_create();
    ros::Time old;
    double x_world=-1000,y_world=-1000,x_world_old=-1000,y_world_old=-1000;
    double pitch_old,roll_old;
void  GetPic(void)
{
 	static ros::Rate rate2(60);
	while(ros::ok()){
	int size = camera.get_frame(srcImg.data, camFormat.image_size, 1);
        if (size == -1) {
            printf("error number: %d\n", errno);
            perror("Cannot get image from camera");
            camera.stop();
            camera.start();
            continue;
        }
        
        cv::remap(srcImg,srcImg,map1,map2,cv::INTER_LINEAR);
	cv::cvtColor(srcImg,gray,CV_YUV2GRAY_YUYV);
	RefreshFlag=1;
	rate2.sleep();
	}

}

void SendPosition(void)
{
	static ros::Rate rate1(500);
	while(ros::ok()){
		if(RefreshFlag==0)continue;
		else RefreshFlag=0;
//		cv::imshow("1",gray);////////////////////////////////
//		cv::waitKey(1);
            image_u8_t im = { .width = gray.cols,
                              .height = gray.rows,
                              .stride = gray.cols,
                              .buf = gray.data
                            };
            zarray_t *detections = apriltag_detector_detect(td, &im);
            for (int i=0;i<zarray_size(detections);i++)
            {
                apriltag_detection_t *det;
                zarray_get(detections, i, &det);
                matd_t *pose=homography_to_pose(det->H,internum[0],internum[1],internum[2],internum[3]);
                pose = matd_inverse(pose);
                if (i==0){
                    pose_end=pose;
                    id=det->id;
                }
                // if (det->id==id_old){
                //     pose_id_old=pose;
                //     id_old_flag=true;
                // }
                if ((pose->data[3]) * (pose->data[3])+(pose->data[7]) * (pose->data[7]) < (pose_end->data[3])*(pose_end->data[3])+(pose_end->data[7])*(pose_end->data[7]))
                {
                    pose_end=pose;
                    id=det->id;
                }
                
                if (i==zarray_size(detections)-1){


                        wRo_to_euler(pose_end,yaw,pitch,roll);
                        x_world=d0*(pose_end->data[3])+*(x_center+id);y_world=d0*(pose_end->data[7])+*(y_center+id);msg.z=(pose_end->data[11])*d0;
                        msg.x=x_world;msg.y=y_world;

                        msg.pitch=pitch;msg.yaw=(yaw<0)?(yaw + PI):(yaw-PI);msg.roll=roll;
                     
                        msg.header.stamp=ros::Time::now();
                        x_reco.push(x_world);y_reco.push(y_world);
                       // time_reco.push(now);
                        msg.id=id;

                      //  msg.v_roll=(roll-roll_old)/((now-old).toSec());
                      //  msg.v_pitch=(pitch-pitch_old)/((now-old).toSec());
                      /*  if (x_reco.size()>10){
                       double vx_temp=(x_reco.back()-x_reco.front())/((time_reco.back()-time_reco.front()).toSec());
                        double vy_temp=(y_reco.back()-y_reco.front())/((time_reco.back()-time_reco.front()).toSec());
                         msg.vx=-vy_temp*sin(yaw)-vx_temp*cos(yaw);
                         msg.vy=vy_temp*cos(yaw)-vx_temp*sin(yaw);
                         x_reco.pop();y_reco.pop();time_reco.pop();
                        }*/
                        {
                            msg.vx=0.0;msg.vy=0.0;
                        }
                        
                       // x_world_old=x_world;y_world_old=y_world;
                       // old=now;
                       // id_old=id;
                        pitch_old=pitch;
                        roll_old=roll;
                        pub.publish(msg);
                  //  }

                }
                
            }
            zarray_destroy(detections);
        
        ros::spinOnce();
	rate1.sleep();
}
    }
int main (int argc, char* argv[])
{

    ros::init(argc, argv, "uav_localization");
    ros::NodeHandle n;
    pub = n.advertise<uav_msgs::uav_position>("position", 1);


//******************************************************************************************************************************//
   
    camera.set_format(320, 240, Withrobot::fourcc_to_pixformat('Y','U','Y','V'), 1, 30);

    camera.get_current_format(camFormat);
    std::string camName = camera.get_dev_name();
    std::string camSerialNumber = camera.get_serial_number();
    printf("dev: %s, serial number: %s\n", camName.c_str(), camSerialNumber.c_str());
    printf("----------------- Current format informations -----------------\n");
    camFormat.print();
    printf("---------------------------------------------------------------\n");
    if (!camera.start()) {
        perror("Failed to start.");
        exit(0);
    }
    std::string windowName = camName + " " + camSerialNumber;


    cv::initUndistortRectifyMap(iner,con,cv::Mat(),iner,cv::Size(camFormat.width,camFormat.height),CV_16SC2,map1,map2);

//******************************************************************************************************************************//


    tf = tag36h11_create();
    tf->black_border=1;

    apriltag_detector_add_family(td, tf);
    td->quad_decimate = 1.0;
    td->quad_sigma = 0.0;
    td->nthreads = 8;
    td->debug = false;
    td->refine_edges = false;
    td->refine_decode = false;
    td->refine_pose = false;
  //  image_transport::ImageTransport it(n);
  //  image_transport::Subscriber sub = it.subscribe("april_pic/image", 1, imageCallback);

    for (int i=0;i<300;i++){x_old[i]=-1000;y_old[i]=-1000;}
    for (int i=0;i<49;i++){
                            x_center[i]=(i%7)*d0*3;y_center[i]=i/7*d0*3;
                            x_center[i+49]=(i%7)*d0*3;y_center[i+49]=(i/7)*d0*3+119.9;
                            x_center[i+98]=(i%7)*d0*3+119.9;y_center[i+98]=i/7*d0*3;
                            x_center[i+147]=(i%7)*d0*3+119.9;y_center[i+147]=i/7*d0*3+119.9;
							x_center[i+196]=(i%7)*d0*3+120.0*2;y_center[i+196]=i/7*d0*3;
							x_center[i+245]=(i%7)*d0*3+120.0*2;y_center[i+245]=i/7*d0*3+119.9;

    }


    std::thread get_pic_task(GetPic);
    std::thread send_pos_task(SendPosition);
    get_pic_task.join();
    send_pos_task.join();

   // cv::namedWindow("1");
    
	//cv::destroyAllWindows();
    apriltag_detector_destroy(td);
    tag36h11_destroy(tf);
    //getopt_destroy(getopt_temp);
    printf("Done.\n");
    return 0;
}





inline double standardRad(double t) {
    if (t >= 0.) {
        t = fmod(t+PI, TWOPI) - PI;
    } else {
        t = fmod(t-PI, -TWOPI) + PI;
    }
    return t;
}

void wRo_to_euler(const matd_t *pose, double& yaw, double& pitch, double& roll) {
    const double *data = pose->data;
    yaw = standardRad(atan2(data[4], data[0]));
    double c = cos(yaw);
    double s = sin(yaw);
    pitch = standardRad(atan2(-data[8], data[0]*c + data[4]*s));
    roll  = standardRad(atan2(data[2]*s - data[6]*c, -data[1]*s + data[5]*c));
}


