#include "roshandler.h"
#include "i2cdriver.h"

volatile int action = 0, index_bola=0;
volatile float odom_x, odom_y, odom_z;
volatile byte landmark=0;
volatile float u_odom_x, u_odom_y, u_odom_z;
uint64_t number=0, lastdata=0;
bool stat = false;

geometry_msgs::Vector3 enc_val;
geometry_msgs::Vector3 update_odom;
std_msgs::Int32 bola;
ros::Subscriber<geometry_msgs::Vector3> od("robot3/odometry", &odometry);
ros::Subscriber<geometry_msgs::Vector3> pv("robot3/pwm_val", &pw_v);
ros::Subscriber<std_msgs::Int32> srt("robot3/action", &starting);
ros::Subscriber<std_msgs::Int32> c("robot3/camera_error", &cam_dat);
ros::Publisher odom("robot3/encoder", &enc_val);
ros::Publisher oc("robot3/encoder_correction", &update_odom);
ros::Publisher bp("robot3/ball_position", &bola);

void update_position(){
  update_odom.x = u_odom_x;
  update_odom.y = u_odom_y;
  update_odom.z = u_odom_z;
  oc.publish(&update_odom);
}

void starting(const std_msgs::Int32 & dat){
  action = dat.data;
}

void odometry(const geometry_msgs::Vector3 & dat){
  odom_x = dat.x;
  odom_y = dat.y;
  odom_z = dat.z;  
}

void cam_dat(const std_msgs::Int32 & dat){
  cam = dat.data;
}

void pw_v(const geometry_msgs::Vector3 & dat){
  pwm[0] = dat.x; //pwm kiri
  pwm[1] = dat.y; //pwm tengah
  pwm[2] = dat.z; //pwm kanan
}

void publish_encoder(){
  enc_val.x=enc[0];//kiri
  enc_val.y=enc[1];//tengah
  enc_val.z=enc[2];//kanan
  odom.publish(&enc_val);
  //nh.spinOnce();
}

void publish_bola(){
  bola.data = index_bola;
  bp.publish(&bola);
}

void init_ros(){
  nh.initNode();
  nh.subscribe(od);//odometry
  nh.subscribe(pv);//pwm
  nh.subscribe(srt);
  nh.subscribe(c);//pwm
  nh.advertise(odom);//publish encoder
  nh.advertise(oc);
  nh.advertise(bp);
}

void ros_routine(){
  nh.spinOnce();
}
