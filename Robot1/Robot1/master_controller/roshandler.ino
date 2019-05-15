#include "roshandler.h"
#include "i2cdriver.h"

volatile float odom_x, odom_y, odom_z;
volatile float u_odom_x, u_odom_y, u_odom_z;
volatile int tendang = 0, index_lokalisasi = 0, index_complete = 0, index_action=0;
uint64_t number=0, lastdata=0;
bool stat = false;
byte delkick=10;

geometry_msgs::Vector3 enc_val;
geometry_msgs::Vector3 update_odom;
std_msgs::Int32 status_bola;
std_msgs::Int32 com;
ros::Subscriber<geometry_msgs::Vector3> od("robot1/odometry", &odometry);
ros::Subscriber<geometry_msgs::Vector3> pv("robot1/pwm_val", &pw_v);
ros::Subscriber<std_msgs::Int32> kb("robot1/kick_ball", &perintah_tendang);
ros::Subscriber<std_msgs::Int32> lk("robot1/localization", &lokalisasi);
ros::Subscriber<std_msgs::Int32> ac("robot1/action", &get_action);
ros::Publisher odom("robot1/encoder", &enc_val);
ros::Publisher oc("robot1/encoder_correction", &update_odom);
ros::Publisher bp("robot1/ball_position", &status_bola);
ros::Publisher th("robot1/command_clear", &com);

void update_position(){
  update_odom.x = u_odom_x;
  update_odom.y = u_odom_y;
  update_odom.z = u_odom_z;
  oc.publish(&update_odom);
}

void perintah_tendang(const std_msgs::Int32 & dat){
  tendang = dat.data;  
}

void lokalisasi(const std_msgs::Int32 & dat){
  index_lokalisasi = dat.data;
  index_lock = 0;
}

void get_action(const std_msgs::Int32 & dat){
  index_action = dat.data;  
}

void odometry(const geometry_msgs::Vector3 & dat){
  odom_x = dat.x;
  odom_y = dat.y;
  odom_z = dat.z;  
}

void pw_v(const geometry_msgs::Vector3 & dat){
  pwm[0] = dat.x; //pwm kiri
  pwm[1] = dat.y; //pwm tengah
  pwm[2] = dat.z; //pwm kanan
}

void publish_acc(){
  com.data = index_complete;
  th.publish(&com);
}

void publish_encoder(){
  enc_val.x=enc[0];//kiri
  enc_val.y=enc[1];//tengah
  enc_val.z=enc[2];//kanan
  odom.publish(&enc_val);
}

void ball_detection(unsigned char data){
  status_bola.data = data;
  bp.publish(&status_bola);
}

void init_ros(){
  nh.initNode();
  nh.subscribe(od);//subscribe data odometry
  nh.subscribe(pv);//subscribe nilai pwm
  nh.subscribe(kb);//subscribe perintah tendang
  nh.subscribe(lk);//subscribe lokalisasi
  nh.subscribe(ac);//subscribe action
  nh.advertise(odom);//publish encoder
  nh.advertise(oc);//publish update posisi
  nh.advertise(bp);//publish posisi bola
  nh.advertise(th);//publish command acc
}

void ros_routine(){
  nh.spinOnce();
}
