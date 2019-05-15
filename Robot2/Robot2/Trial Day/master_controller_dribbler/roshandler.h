#ifndef ROSHANDLER_H
#define ROSHANDLER_H

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Vector3.h>

extern volatile float odom_x, odom_y, odom_z;
extern volatile float u_odom_x, u_odom_y, u_odom_z;
extern volatile int tendang, index_lokalisasi, index_complete, index_action, index_dribbler;

void update_position();
void odometry(const geometry_msgs::Vector3 & dat);
void perintah_tendang(const std_msgs::Int32 & dat);
void lokalisasi(const std_msgs::Int32 & dat);
void get_dribbler_direction(const std_msgs::Int32 & dat);
void pw_v(const geometry_msgs::Vector3 & dat);
void publish_encoder();
void publish_acc();
void ball_detection(unsigned char data);
void ros_routine();

void init_ros();

#endif
