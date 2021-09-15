#include <ros/ros.h>
#include "opencv2/opencv.hpp"
#include "std_msgs/Int32.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/Twist.h>
#include <dynamic_reconfigure/server.h>
#include <dynamic_tutorials/TutorialsConfig.h>
#include <bits/stdc++.h>
int flag=0;
using namespace std;
using namespace cv;

RNG rng(12345);
int k=190;
float kp=0,ki=0,kd=0;
float linear_x=0,linear_y=0,linear_z=0,ang_z=0;
float prev_error=0,control_signal,delta_error=0,curr_error=0,cum_error=0;
float prev_error_r=0,control_signal_r,delta_error_r=0,curr_error_r=0,cum_error_r=0;
ros::Publisher pub;

void chatterCallback(const sensor_msgs::ImageConstPtr& msg){
 // ROS_INFO("error is [%d] \n",msg->data);
  cv_bridge::CvImagePtr cv_ptr;

    try{
    cv_ptr=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
  }
  catch(cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }


 //cout<<"succesffulyy converted front image"<<endl;
  cv::namedWindow("front_camera",CV_WINDOW_NORMAL);

  Mat dst;
    threshold( cv_ptr->image, dst, 40, 255 , 1);
    Mat canny_output;
    Canny( dst, canny_output, 0,3,3 );
    vector<vector<Point> > contours;
    findContours( canny_output, contours, RETR_TREE, CHAIN_APPROX_SIMPLE );
    //cout<<contours.size()<<endl;
    vector<vector<Point> > contours_poly( contours.size() );
    vector<Rect> boundRect( contours.size() );
    vector<Point2f>centers( contours.size() );
    vector<float>radius( contours.size() );
    priority_queue<pair<int,int> > pq;
    for( size_t i = 0; i < contours.size(); i++ )
    {
        approxPolyDP( contours[i], contours_poly[i], 3, true );
        boundRect[i] = boundingRect( contours_poly[i] );
        minEnclosingCircle( contours_poly[i], centers[i], radius[i] );
        pq.push(make_pair(radius[i],i));
        //cout<<radius[i]<<endl;
    }
    pq.pop();
    cout<<"pixel half-height is: "<<pq.top().first<<endl;
    Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
    /*for( size_t i = 0; i< contours.size(); i++ )
    {
        Scalar color = Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
        drawContours( drawing, contours_poly, (int)i, color );
        rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2 );
        circle( drawing, centers[i], (int)radius[i], color, 2 );
    }*/
    //int center_x=,center_y
    circle(drawing,centers[pq.top().second],(int)pq.top().first,Scalar(0,255,0),2);
    circle(drawing,Point(int((drawing.cols)/2),int((drawing.rows)/2)),8,Scalar(0,0,255),-1);
    circle(drawing,Point(int(centers[pq.top().second].x),int(centers[pq.top().second].y)),8,Scalar(0,255,0),-1);
    curr_error=centers[pq.top().second].x-drawing.cols/2;
    curr_error_r=(k/linear_x)-pq.top().first;
    imshow( "Contours", drawing );
    cv::imshow("front_camera", cv_ptr->image);
  cv::waitKey(1);

  geometry_msgs::Twist vel;
  //curr_error=msg->data;
  cum_error=curr_error+prev_error;
  cum_error_r=curr_error_r+prev_error_r;
  delta_error=curr_error-prev_error;
  delta_error_r=curr_error_r-prev_error_r;
  control_signal=(kp*curr_error+ki*cum_error+kd*delta_error);
  control_signal_r=(kp*curr_error_r+ki*cum_error_r+kd*delta_error_r);
  ROS_INFO("control input is [%f]",control_signal/100);
  ROS_INFO("control input_r is [%f]",control_signal_r/10);
  //ROS_INFO("kp ki kd are [%f] [%f] [%f]",kp,ki,kd);
  prev_error=curr_error;
  prev_error_r=curr_error_r;
  vel.linear.x=(control_signal_r/15);//0.075;
  //vel.linear.x=linear_x;
  vel.linear.y=linear_y;
  vel.linear.z=linear_z;
 // vel.linear.z=control_signal_r/120;
  vel.angular.z=-(control_signal/110);
  //vel.angular.z=ang_z;
  pub.publish(vel);

  ros::spinOnce();

}
void callback(dynamic_tutorials::TutorialsConfig &config, uint32_t level) {
  // ROS_INFO("Reconfigure Request: %f %f %f %f \n", 
  //           config.rad, config.speed, config.height,config.ang);
  // kp=config.kp;
  // ki=config.ki;
  // kd=config.kd;
  // linear_x=config.rad;
  // linear_y=config.speed;
  // linear_z=config.height;
  // ang_z=config.ang;
  // //ROS_INFO("kp kd ki are {%f] [%f] [%f]",kp,kd,ki);
  // ros::spinOnce();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "dynamic_tutorials");
   ros::NodeHandle n;
  dynamic_reconfigure::Server<dynamic_tutorials::TutorialsConfig> server;
  dynamic_reconfigure::Server<dynamic_tutorials::TutorialsConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);
  pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  ros::Subscriber sub = n.subscribe("/front_cam/camera/image", 10, chatterCallback);

  ROS_INFO("Spinning node");
  ros::spin();
  return 0;
}
