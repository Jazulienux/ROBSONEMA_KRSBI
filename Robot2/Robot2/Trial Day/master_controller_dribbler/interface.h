#ifndef INTERFACE_H
#define INTERFACE_H

#define lmp_on        digitalWrite(lcd_lmp,HIGH)
#define lmp_off       digitalWrite(lcd_lmp,LOW)
#define tb_bawah      digitalRead(pintombol[0])==HIGH //52
#define tb_kiri       digitalRead(pintombol[1])==HIGH //50
#define tb_atas       digitalRead(pintombol[2])==HIGH //48
#define tb_kanan      digitalRead(pintombol[3])==HIGH //46
#define tb_back       digitalRead(pintombol[4])==HIGH //44
#define tb_enter      digitalRead(pintombol[5])==HIGH //42
#define pr_bola       digitalRead(pinproximity[0])==HIGH //23

#define lcd_lmp       40
#define trigger_pin   9

extern volatile byte mode, delay_rate[4], pintombol[6], dribble_spd[5], lmp, pos, pinproximity[4];
extern volatile int spd_dep[2], spd_bel, spoint[2], ylim[2], xyz_camera[3], throtle[5], spdmax, spdmin;
extern volatile int additional[4];

void init_interface();
void menu_pos();
void clear_screen();
void standby_screen();
void odom_screen();
void odom_screen1();
void scansensor_screen();
void manual_screen();
void menu_screen();
void set_screen(unsigned char mode);
void running_screen();
void kalibrasi_screen();
void check_screen();
//function menu
void menu_pid();
void menu_sensor();
void menu_kicker();
void menu_camera();
void xyz_scan();
void menu_speed();
void menu_motor();
void kick_ball(unsigned char del, unsigned char target);
void cari_garis_screen();

#endif
