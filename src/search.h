/*
 * search.h
 *
 *  Created on: 2018/07/14
 *      Author: kouhei
 */

#ifndef SEARCH_H_
#define SEARCH_H_

#define NORTH (0)
#define EAST (1)
#define SOUTH (2)
#define WEST (3)

#define T_MODE (1)
#define S_MODE (0)

#define LEFT90 (90)
#define RIGHT90 (-90)

#define SEARCH_SPEED (350)//[mm/s]580
#define SEARCH_TURN_SPEED (350) //[mm/s]350
#define SEARCH_ACC (2)
#define PRE_LENGTH (19)//25 20 [mm]18
#define SLA_AT_LENGTH_L (230)//step number1163
#define SLA_ST_LENGTH_L (118)//step number1163
#define PRE_L (17)
#define AFTER_L (22)

#define SLA_AT_LENGTH_R (229)//step number1144
#define SLA_ST_LENGTH_R (113)//step number1163
#define PRE_R (19)
#define AFTER_R (23)


#define STRAIGHTRUN (0)
#define LEFTTURN (1)
#define RIGHTTURN (2)
#define PIVOTTURN (3)


typedef unsigned char uchar;

extern uchar h_map[16][16];

void search2018(uchar gx,uchar gy);
void hight_map(uchar gx, uchar gy, uchar h,uchar mode);
void update_map(uchar x, uchar y, uchar dir);
short decide_action(uchar mx, uchar my, uchar gx, uchar gy, uchar dir, uchar mode);
void move(uchar action);
void smove(uchar action, short speed);
void update_state(uchar act, uchar *x, uchar *y, uchar *dir);
void make_mapdata(void);
void make_shortest_run_command(uchar gx, uchar gy, uchar* command, uchar diag_flag );
void front_wall_adj(void);

#endif /* SEARCH_H_ */
