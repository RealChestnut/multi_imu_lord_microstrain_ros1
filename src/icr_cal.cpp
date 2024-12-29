#include <vector>
#include <ros/ros.h>
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <std_msgs/String.h>
#include <cmath>
#include <cstdio>
#include <chrono>

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Bool.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>

#include "tf/transform_datatypes.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include "nav_msgs/Odometry.h"

// Subscriber //

ros::Subscriber imu_data_; // imu data callback



ros::Publisher ang_vel_imu1; // rearranged imu1 angular velocity data
ros::Publisher ang_imu1;     // rearranged imu1 angular data
ros::Publisher imu_data_array_pub;
ros::Publisher imu_induced_lin_vel_pub;

ros::Publisher delta_time;

// Clock data //
std::chrono::high_resolution_clock::time_point end=std::chrono::high_resolution_clock::now();
std::chrono::high_resolution_clock::time_point start=std::chrono::high_resolution_clock::now();
std::chrono::duration<double> delta_t;
std_msgs::Float32 dt;

//IMU DATA CALLBACK//
geometry_msgs::Quaternion imu_quaternion;
geometry_msgs::Vector3 imu1_rpy;
geometry_msgs::Vector3 imu1_ang_vel;
geometry_msgs::Vector3 imu1_lin_acl;

geometry_msgs::Vector3 imu2_rpy;
geometry_msgs::Vector3 imu2_ang_vel;
geometry_msgs::Vector3 imu2_lin_acl;

geometry_msgs::Vector3 imu3_rpy;
geometry_msgs::Vector3 imu3_ang_vel;
geometry_msgs::Vector3 imu3_lin_acl;

geometry_msgs::Vector3 prev_ang_vel;

std_msgs::Float32MultiArray imu_data_array;
std_msgs::Float32MultiArray imu_lin_vel_acl2lpf;

////////////////////Accelerometer_LPF///////////////////////
double accel_cutoff_freq = 1.0;
std::vector<geometry_msgs::Vector3> filtered_data_list;
// Accelerometer Low-Pass Filter with 9x9 Matrix
void Accelerometer_LPF_9x9(const std::vector<geometry_msgs::Vector3>& imu_data_list,
                           std::vector<geometry_msgs::Vector3>& filtered_data_list,
                           double accel_cutoff_freq,
                           double delta_t) {
    // Initialize 9x9 state matrices
    Eigen::MatrixXd X_dot = Eigen::MatrixXd::Zero(9, 1); // Derivative matrix
    Eigen::MatrixXd X = Eigen::MatrixXd::Zero(9, 1);     // State matrix
    Eigen::MatrixXd U = Eigen::MatrixXd::Zero(9, 1);     // Input (IMU data) matrix

    // Fill the input matrix (U) with IMU data
    for (size_t i = 0; i < imu_data_list.size() && i < 3; ++i) {
        U(i * 3, 0) = imu_data_list[i].x;
        U(i * 3 + 1, 0) = imu_data_list[i].y;
        U(i * 3 + 2, 0) = imu_data_list[i].z;
    }

    // Low-pass filter: X_dot = -cutoff_freq * X + U
    X_dot = -accel_cutoff_freq * X + U;

    // Update state: X += X_dot * delta_t
    X += X_dot * delta_t;

    // Filtered acceleration: lin_acl = cutoff_freq * X
    Eigen::MatrixXd lin_acl = accel_cutoff_freq * X;

    // Extract filtered data back into geometry_msgs::Vector3 format
    filtered_data_list.clear();
    for (size_t i = 0; i < imu_data_list.size() && i < 3; ++i) {
        geometry_msgs::Vector3 filtered;
        filtered.x = lin_acl(i * 3, 0);
        filtered.y = lin_acl(i * 3 + 1, 0);
        filtered.z = lin_acl(i * 3 + 2, 0);
        filtered_data_list.push_back(filtered);
    }
}


////////////////////////////////////////////////////////////

void Clock();

void imu_Callback(const sensor_msgs::Imu& msg);

void publisherSet();

void PublishData();

int main(int argc, char **argv)
{

  ros::init(argc, argv, "icr_cal");
  ros::NodeHandle nh;

  imu_data_ 	 = nh.subscribe("/imu/data",1,imu_Callback,ros::TransportHints().tcpNoDelay()); // angle data from IMU
  
  imu_data_array_pub = nh.advertise<std_msgs::Float32MultiArray>("imu123_data_array",1);
  ang_vel_imu1   = nh.advertise<geometry_msgs::Vector3>("ang_vel",1); // imu1 angular velocity 
  ang_imu1       = nh.advertise<geometry_msgs::Vector3>("ang",1); // imu1 angle
  imu_induced_lin_vel_pub = nh.advertise<std_msgs::Float32MultiArray>("imu_induced_lin_vel",1);
  delta_time     = nh.advertise<std_msgs::Float32>("delta_t",1);

  imu_data_array.data.resize(27);
  imu_lin_vel_acl2lpf.data.resize(9);
	


ros::Timer timerPublish = nh.createTimer(ros::Duration(1.0/200.0),std::bind(publisherSet));
    ros::spin();
    return 0;
}

 void publisherSet(){
    Clock();


    std::vector<geometry_msgs::Vector3> imu_data_list(3);

    imu_data_list[0].x = imu1_lin_acl.x; imu_data_list[0].y = imu1_lin_acl.y; imu_data_list[0].z = imu1_lin_acl.z;
    imu_data_list[1].x = imu2_lin_acl.x; imu_data_list[1].y = imu2_lin_acl.y; imu_data_list[1].z = imu2_lin_acl.z;
    imu_data_list[2].x = imu3_lin_acl.x; imu_data_list[2].y = imu3_lin_acl.y; imu_data_list[2].z = imu3_lin_acl.z;
	

    Accelerometer_LPF_9x9(imu_data_list, filtered_data_list, accel_cutoff_freq, delta_t.count());

    imu_lin_vel_acl2lpf.data[0]= filtered_data_list[0].x;
    imu_lin_vel_acl2lpf.data[1]= filtered_data_list[0].y;
    imu_lin_vel_acl2lpf.data[2]= filtered_data_list[0].z;

    imu_lin_vel_acl2lpf.data[3]= filtered_data_list[1].x;
    imu_lin_vel_acl2lpf.data[4]= filtered_data_list[1].y;
    imu_lin_vel_acl2lpf.data[5]= filtered_data_list[1].z;

    imu_lin_vel_acl2lpf.data[6]= filtered_data_list[2].x;
    imu_lin_vel_acl2lpf.data[7]= filtered_data_list[2].y;
    imu_lin_vel_acl2lpf.data[8]= filtered_data_list[2].z;

    PublishData();


 }

///////////////////////////////CALLBACK FUNCTION DATA//////////////////////////////////////
void Clock(){
   	end=std::chrono::high_resolution_clock::now();
   	delta_t=end-start;
   	start=std::chrono::high_resolution_clock::now();
   	dt.data=delta_t.count();
   	//ROS_INFO_STREAM("Clock Time : " << dt.data);

}


//IMU DATA CALLBACK//

void imu_Callback(const sensor_msgs::Imu& msg){
	

	std::string frame_id = msg.header.frame_id;	

	
	// TP attitude - Quaternion representation
	
    	imu_quaternion=msg.orientation;

    	tf::Quaternion quat;
    	tf::quaternionMsgToTF(imu_quaternion,quat);

    	// TP attitude - Euler representation
	
	if(frame_id == "sensor1"){
    	tf::Matrix3x3(quat).getRPY(imu1_rpy.x,imu1_rpy.y,imu1_rpy.z); //t265_rpy.z
	imu1_ang_vel=msg.angular_velocity;
        imu1_lin_acl=msg.linear_acceleration;
	imu_data_array.data[0] = imu1_rpy.x;
	imu_data_array.data[1] = imu1_rpy.y;
	imu_data_array.data[2] = imu1_rpy.z;

	imu_data_array.data[3] = imu1_ang_vel.x;
        imu_data_array.data[4] = imu1_ang_vel.y;
        imu_data_array.data[5] = imu1_ang_vel.z;

	imu_data_array.data[6] = imu1_lin_acl.x;
        imu_data_array.data[7] = imu1_lin_acl.y;
        imu_data_array.data[8] = imu1_lin_acl.z;
	
	}else if(frame_id == "sensor2"){
        tf::Matrix3x3(quat).getRPY(imu2_rpy.x,imu2_rpy.y,imu2_rpy.z); //t265_rpy.z
        imu2_ang_vel=msg.angular_velocity;
        imu2_lin_acl=msg.linear_acceleration;

	imu_data_array.data[9]  = imu2_rpy.x;
        imu_data_array.data[10] = imu2_rpy.y;
        imu_data_array.data[11] = imu2_rpy.z;

        imu_data_array.data[12] = imu2_ang_vel.x;
        imu_data_array.data[13] = imu2_ang_vel.y;
        imu_data_array.data[14] = imu2_ang_vel.z;

        imu_data_array.data[15] = imu2_lin_acl.x;
        imu_data_array.data[16] = imu2_lin_acl.y;
        imu_data_array.data[17] = imu2_lin_acl.z;


        }else if(frame_id == "sensor3"){
        tf::Matrix3x3(quat).getRPY(imu3_rpy.x,imu3_rpy.y,imu3_rpy.z); //t265_rpy.z
        imu3_ang_vel=msg.angular_velocity;
        imu3_lin_acl=msg.linear_acceleration;

	imu_data_array.data[18] = imu3_rpy.x;
        imu_data_array.data[19] = imu3_rpy.y;
        imu_data_array.data[20] = imu3_rpy.z;

        imu_data_array.data[21] = imu3_ang_vel.x;
        imu_data_array.data[22] = imu3_ang_vel.y;
        imu_data_array.data[23] = imu3_ang_vel.z;

        imu_data_array.data[24] = imu3_lin_acl.x;
        imu_data_array.data[25] = imu3_lin_acl.y;
        imu_data_array.data[26] = imu3_lin_acl.z;


        }else{
        ROS_WARN("Unknown frame_id: %s", frame_id.c_str());
	}
	



}

void PublishData(){
	

		
	imu_data_array_pub.publish(imu_data_array);
	ang_vel_imu1.publish(imu1_ang_vel);	
	ang_imu1.publish(imu1_rpy);
        imu_induced_lin_vel_pub.publish(imu_lin_vel_acl2lpf);
	delta_time.publish(dt);

}
