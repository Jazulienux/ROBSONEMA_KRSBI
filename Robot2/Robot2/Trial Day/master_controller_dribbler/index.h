#ifndef INDEX_H
#define INDEX_H

extern volatile bool strt;
extern volatile int adds, index_lock;

void start_positioning();
void run_program2();
void update_z();
void update_x();
void run_program1();

#endif
