#ifndef ROSHANDLER_H
#define ROSHANDLER_H

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Vector3.h>

extern volatile int action, index_bola;
extern volatile float odom_x, odom_y, odom_z, u_odom_x, u_odom_y, u_odom_z;

void update_position();
void starting(const std_msgs::Int32 & dat);
void odometry(const geometry_msgs::Vector3 & dat);
void cam_dat(const std_msgs::Int32 & dat);
void pw_v(const geometry_msgs::Vector3 & dat);
void publish_encoder();
void publish_bola();

void init_ros();
void ros_routine();

#endif
