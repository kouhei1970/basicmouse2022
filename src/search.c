#include "search.h"
#include "Run.h"
#include "Map.h"
#include "Mouse_Data.h"
#include "Wall_Sensor.h"
#include "iodefine.h"
#include "Clock.h"
#include "UI.h"
#include "Wait.h"
#include "SCI.h"
#include "xprintf.h"
#include "ADC.h"
#include "Interrupt.h"
#include "Motor.h"
#include "Wall_Sensor.h"
#include "Buzzer.h"
#include "Battery.h"
#include "My_Func.h"
#include "E2Flash_RX220.h"
#include "Map.h"
#include "Control.h"
#include "Run.h"
#include "Log.h"

unsigned char h_map[16][16];


void search2018(unsigned char gx,unsigned char gy){

    uchar first_f = 1;      //スタートする前か？初期値 １
    uchar goal_f  = 0;      //ゴールしたかしてないか？初期値 0
    uchar search_end_f = 0; //探索終了か？初期値 0
    uchar mx,my;
    uchar goalx;
    uchar goaly ;
    unsigned char dir = NORTH;
    short speed = SEARCH_SPEED;
    short acc = SEARCH_ACC;

    while(!search_end_f){
        if (first_f == 1){//スタート処理

            goalx=gx;
            goaly=gy;
            //地図の初期化
            //reset_map();
            //座標等の初期化
            mx=0;
            my=1;
            wait_sw_on();
            wait_ms(1000);



            //自動的にコースの真ん中にセットしセンサーキャリブレーション
            turn(RIGHT90);                  //右向き右90度
            straight(70, -100, 0, 2, 0);	//後退ケツタッチ
            straight(34, 350, 0, 2, 0);     //真ん中に前進
            turn(LEFT90);                   //前向きなおり左90度
            straight(70, -100, 0, 2, 0);    //後退ケツタッチ
            wait_ms(500);
            update_center_ref();            //センサーキャリブレーション



            set_wall_gain(2);//5            //制御ゲイン設定

            //マウス出発
            hotel_buzzer();

            straight(H_SECTION + 34, speed, speed, 2, 1);  //124mm前進 スタート
            first_f = 0;//フラグリセット（スタート終わり）
        }

        else if (goalx==mx && goaly==my){//目的地（ゴールまたはスタート）に入った時の処理
            update_map(mx, my, dir);                                    //（１）区画の境目で壁の情報を更新
            LED_on(1);LED_on(2);LED_on(3);LED_on(4);
            straight(H_SECTION, speed, 0, acc, 1);//半歩前進
            goal_f = 1;

            if(goalx==0&&goaly==0){//スタート区画処理
                save_map();             //map seved
                church_save_buzzer();   //save sound

                goalx=gx;
                goaly=gy;

                //向きを変えて再出発
                update_state(PIVOTTURN, &mx, &my, &dir);
                turn(LEFT90);

				//自動的にコースの真ん中にセットしセンサーキャリブレーション
				//turn(RIGHT90);                  //右向き右90度
				straight(70, -100, 0, 2, 0);	//後退ケツタッチ
				straight(34, 350, 0, 2, 0);     //真ん中に前進
				turn(LEFT90);                   //前向きなおり左90度
				straight(70, -100, 0, 2, 0);	//後退ケツタッチ
				wait_ms(500);
				update_center_ref();            //センサーキャリブレーション
                // Restart !
				straight(H_SECTION + 34, speed, speed, 2, 1);  //124mm前進 スタート
			}

			else {//ゴール区画処理
				level_up_buzzer();//ゴールファンファーレ
				goalx=0;
				goaly=0;
				save_map();
				church_save_buzzer();

				//向きを変えて再出発
				update_state(PIVOTTURN, &mx, &my, &dir);
				turn(180);
				straight(H_SECTION, speed, speed, 2, 1);

			}//スタートを目指す
			//search_end_f = 1;
		}

		else if (first_f==0 && mx==0 && my==0){//スタートに戻った時の処理
			//LED_on(1);LED_on(2);LED_on(3);LED_on(4);
			//straight(H_SECTION, speed, 0, acc, 1);//半歩前進
			//level_up_buzzer();//ゴールファンファーレ
			//goal_f = 1;
			//gx=0;gy=0;//スタートを目指す
			//search_end_f = 1;
		}

		else {//普通の探索
			update_map(mx, my, dir);                                            //（１）区画の境目で壁の情報を更新
			uchar action = decide_action(mx, my, goalx, goaly, dir, S_MODE);    //（２）壁の情報に基づき移動方向を決定
			move(action);                                                       //（３）決定した行動を実行する
			update_state(action, &mx ,&my, &dir);                               //（４）座標、方向の状態変数を更新する
		}
	}
}

void make_shortest_run_command(uchar sx , uchar sy, uchar gx, uchar gy, uchar* command, uchar diag_flag ){
    uchar x,y,dir;
    x = sx;
    y = sy;
    dir = 0;
    while (x==gx && y==gy){
        *command = decide_action(x, y, gx, gy, dir, T_MODE);
        update_state(*command, &x, &y, &dir );
    }
}

void shortest_run(uchar sx, uchar sy, uchar gx, uchar gy, uchar* command){
    
    
}


#if 0
void find_unsearched_block(uchar ox, uchar oy){
	uchar x,y;
	uchar cnt;
	cnt=1
			for (y=oy-cnt;y<oy+cnt;y++){
				for(x=ox-cnt;x<ox+cnt;x++){

				}
			}

}
#endif


/* ======================================================================= */
/* 等高線作成モジュール                                                    */
/* ======================================================================= */
void hight_map(uchar gx, uchar gy, uchar h, uchar mode)
{
	// (gy,gx)ゴールにしたい座標
	// h使っていない
	// mode　未探索区画の歩数を数えるか数えないかT_MODE：未探索区画数えない（最短走行時の計算に使用）

	uchar	pt0,pt1,ct;
	uchar	x,y;
	uchar	wdata;

	//等高線マップを最候歩数255にリセット
	for (y=0;y<16;y++){
		for (x=0;x<16;x++){
			h_map[y][x] = 255;
		}
	}
	h_map[gy][gx] = 0;//ゴールの場所の歩数を0歩に

	pt0 = 0;

	do{
		ct = 0;
		pt1 = pt0+1;

		for (y=0;y<16;y++){
			for (x=0;x<16;x++){
				if (h_map[y][x]==pt0){
					wdata=map[y][x];
					if (mode==T_MODE){  //未探索区画には進まないモード
						if (((wdata & 0x11) == 0x10)&&(y!=15)){
							if (h_map[y+1][x] == 255){h_map[y+1][x] = pt1;ct++;}}
						if (((wdata & 0x22) == 0x20)&&(x!=15)){
							if (h_map[y][x+1] == 255){h_map[y][x+1] = pt1;ct++;}}
						if (((wdata & 0x44) == 0x40)&&(y!=0)){
							if (h_map[y-1][x] == 255){h_map[y-1][x] = pt1;ct++;}}
						if (((wdata & 0x88) == 0x80)&&(x!=0)){
							if (h_map[y][x-1] == 255){h_map[y][x-1] = pt1;ct++;}}
					}else{              //未探索区画に進むモード


						if (((wdata & 0x01) == 0x00)&&(y!=15)){
							if (h_map[y+1][x] == 255){h_map[y+1][x] = pt1;ct++;}}
						if (((wdata & 0x02) == 0x00)&&(x!=15)){
							if (h_map[y][x+1] == 255){h_map[y][x+1] = pt1;ct++;}}
						if (((wdata & 0x04) == 0x00)&&(y!=0)){
							if (h_map[y-1][x] == 255){h_map[y-1][x] = pt1;ct++;}}
						if (((wdata & 0x08) == 0x00)&&(x!=0)){
							if (h_map[y][x-1] == 255){h_map[y][x-1] = pt1;ct++;}}
					}
				}
			}
		}
		pt0 = pt0+1;

	}while(ct!=0);
}


void update_map(uchar x, uchar y, uchar dir){
	// 壁のあるなしでLEDを点灯する
	// LED 左から１２３４
#if 1
	if (wall_ls)LED_on(1);
	else LED_off(1);
	if (wall_lf)LED_on(2);
	else LED_off(2);
	if (wall_rf)LED_on(3);
	else LED_off(3);
	if (wall_rs)LED_on(4);
	else LED_off(4);
#endif

//向いている方向に応じて、地図の壁情報を更新する。
	if ( dir == NORTH){//北向きだったら
		set_map(x, y, NORTH, wall_lf | wall_rf );//前壁はOR取り
		set_map(x, y, WEST , wall_ls );
		set_map(x, y, EAST , wall_rs );
	}

	else if ( dir == EAST){//東向きだったら
		set_map(x, y, EAST ,wall_lf|wall_rf );
		set_map(x, y, NORTH, wall_ls );
		set_map(x, y, SOUTH, wall_rs);

	}

	else if ( dir == SOUTH){//南向きだったら
		set_map(x, y, SOUTH, wall_lf|wall_rf );
		set_map(x, y, EAST , wall_ls );
		set_map(x, y, WEST , wall_rs);
	}

	else if ( dir == WEST){//西向きだったら
		set_map(x, y, WEST , wall_lf|wall_rf );
		set_map(x, y, SOUTH, wall_ls );
		set_map(x, y, NORTH, wall_rs);

	}
	return;
}

short decide_action(uchar mx, uchar my, uchar gx, uchar gy, uchar dir, uchar mode){

	uchar front=255, left=255, right=255;

	hight_map(gx, gy, 0, mode);//等高線マップ作る
	front=255;left=255;right=255;

	if ( dir == NORTH){//北向きだったら
		if (my<15 && get_map(mx,my,NORTH)==0) front = h_map[my+1][mx];
		if (mx>0  && get_map(mx,my,WEST)==0) left  = h_map[my][mx-1];
		if (mx<15 && get_map(mx,my,EAST)==0) right = h_map[my][mx+1];

		if (front==255&&left==255&&right==255) return PIVOTTURN;
		if (front<=left && front<=right)       return STRAIGHTRUN;
		if (left<=front && left<=right)        return LEFTTURN;
		if (right<=front && right<=left)       return RIGHTTURN;
	}

	else if ( dir == EAST){//東向きだったら
		if (mx<15 && get_map(mx,my,EAST)==0) front = h_map[my][mx+1];
		if (my<15 && get_map(mx,my,NORTH)==0) left  = h_map[my+1][mx];
		if (my>0  && get_map(mx,my,SOUTH)==0) right = h_map[my-1][mx];

		if (front==255&&left==255&&right==255) return PIVOTTURN;
		if (front<=left && front<=right)       return STRAIGHTRUN;
		if (left<=front && left<=right)        return LEFTTURN;
		if (right<=front && right<=left)       return RIGHTTURN;
	}

	else if ( dir == SOUTH){//南向きだったら
		if (my>0  && get_map(mx,my,SOUTH)==0) front = h_map[my-1][mx];
		if (mx<15 && get_map(mx,my,EAST)==0) left  = h_map[my][mx+1];
		if (mx>0  && get_map(mx,my,WEST)==0) right = h_map[my][mx-1];

		if (front==255&&left==255&&right==255) return PIVOTTURN;
		if (front<=left && front<=right)       return STRAIGHTRUN;
		if (left<=front && left<=right)        return LEFTTURN;
		if (right<=front && right<=left)       return RIGHTTURN;

	}

	else if ( dir == WEST){//西向きだったら
		if (mx>0  && get_map(mx,my,WEST)==0) front = h_map[my][mx-1];
		if (my>0  && get_map(mx,my,SOUTH)==0) left  = h_map[my-1][mx];
		if (my<15 && get_map(mx,my,NORTH)==0) right = h_map[my+1][mx];

		if (front==255&&left==255&&right==255) return PIVOTTURN;
		if (front<=left && front<=right)       return STRAIGHTRUN;
		if (left<=front && left<=right)        return LEFTTURN;
		if (right<=front && right<=left)       return RIGHTTURN;

	}

	return -1;
}


void move(uchar action){

    short control_en  = 1;
    short control_den = 0;

	//物理パラメータ設定
	short acc=SEARCH_ACC;
	short speed = SEARCH_SPEED;
	short tspeed= SEARCH_TURN_SPEED;
	short pre=PRE_LENGTH;//40;// defo 47
	//short slalen=SLA_LENGTH_L;//85;// defo 73
    short range;
    range=0;
	if (action == STRAIGHTRUN) {
		//直進180mm
		straight(SECTION, speed, speed, acc, control_en);
	}

	else if (action == LEFTTURN) {
		//Left Turn
	    //range=get_range();
	    //if(range>pre)range=0;
		straight(PRE_L, speed, tspeed, acc,  control_en);
		slalom(SLA_AT_LENGTH_L, SLA_ST_LENGTH_L, acc, 1, control_den);
		straight(AFTER_L, speed, speed, acc,  control_en);
	}

	else if (action == RIGHTTURN) {
		//Right Turn
        //range=get_range();
        //if(range>pre)range=0;
		straight(PRE_R, speed, tspeed, acc,  control_en);
		slalom(SLA_AT_LENGTH_R, SLA_ST_LENGTH_R, acc, -1, control_den);
		straight(AFTER_R, speed, speed, acc,  control_en);
	}

	else if (action == PIVOTTURN) {
		//Pivot Turn
		straight(H_SECTION, speed, 0, acc,  control_en);
        //
        if(wall_lf&&wall_rf)front_wall_adj();//2019/7/9 add
        //
		turn(180);
		straight(H_SECTION, speed, speed, acc,  control_en);
	}
}

void smove(uchar action, short speed){
//Function for Shortest Run
    short control_en  = 1;
    short control_den = 0;

	//物理パラメータ設定
	short acc=3;
	//short speed = SEARCH_SPEED;
	short tspeed= SEARCH_TURN_SPEED;
	short pre=PRE_LENGTH;//40;// defo 47
	//short slalen=SLA_LENGTH_L;//85;// defo 73


	if (action == STRAIGHTRUN) {
		//直進180mm
		straight(SECTION, speed, speed, acc, control_en);
	}

	else if (action == LEFTTURN) {
		//Left Turn
		straight(pre, speed, tspeed, acc, control_en);
		slalom(SLA_AT_LENGTH_L, SLA_ST_LENGTH_L, acc, 1, control_den);
		straight(pre, speed, speed, acc, control_en);
	}

	else if (action == RIGHTTURN) {
		//Right Turn
		straight(pre, speed, tspeed, acc, control_en);
		slalom(SLA_AT_LENGTH_R, SLA_ST_LENGTH_R, acc, -1, control_den);
		straight(pre, speed, speed, acc, control_en);
	}

	else if (action == PIVOTTURN) {
		//Pivot Turn
		straight(H_SECTION, speed, 0, acc, control_en);
		turn(180);
		straight(H_SECTION, speed, speed, acc, control_en);
	}
}



void update_state(uchar act, uchar *x, uchar *y, uchar *dir){

	if (act==STRAIGHTRUN){
		if (*dir==NORTH){
			(*y)++;
		}
		else if (*dir == EAST){
			(*x)++;
		}
		else if (*dir == SOUTH ){
			(*y)--;
		}
		else if (*dir == WEST){
			(*x)--;
		}
	}

	else if (act==LEFTTURN){
		if (*dir==NORTH){
			(*x)--;
		}
		else if (*dir == EAST){
			(*y)++;
		}
		else if (*dir == SOUTH ){
			(*x)++;
		}
		else if (*dir == WEST){
			(*y)--;
		}
		(*dir)--;
		*dir=(*dir) & 0x03;
	}

	else if (act==RIGHTTURN){
		if (*dir==NORTH){
			(*x)++;
		}
		else if (*dir == EAST){
			(*y)--;
		}
		else if (*dir == SOUTH ){
			(*x)--;
		}
		else if (*dir == WEST){
			(*y)++;
		}

		(*dir)++;
		*dir=(*dir) & 0x03;
	}

	else if (act==PIVOTTURN){
		if (*dir==NORTH){
			(*y)--;
		}
		else if (*dir == EAST){
			(*x)--;
		}
		else if (*dir == SOUTH ){
			(*y)++;
		}
		else if (*dir == WEST){
			(*x)++;
		}
		(*dir)++;
		(*dir)++;
		*dir=(*dir) & 0x03;
	}

	return ;
}

//2019/7/9 Add by kouhei
void front_wall_adj(void)
{
    short lf=3338;
    short rf=2298;


    straight(50, -200, 0, 2, 1);
    while(100*(sensor_lf+sensor_rf)<95*(lf+rf)){
        straight(1, 60, 60, 2, 1);
    }
    straight(1, 60, 0, 2, 0);
}


void make_mapdata(void)
{
	uchar i,j;
	uchar n_wall,e_wall,s_wall,w_wall;
	//2017 Clasic mouse expart final maze
	uchar smap[33][66]={
			"+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+",//    0
			"|   |                                                           |",//15  1
			"+   +   +   +---+---+---+---+---+---+---+---+---+---+   +   +---+",//    2
			"|       |   |               |               |       |   |       |",//14  3
			"+   +---+   +   +---+---+   +   +---+---+   +   +   +---+---+   +",//    4
			"|           |   |       |       |       |       |           |   |",//13  5
			"+   +---+   +   +   +   +---+---+   +   +---+---+---+---+   +   +",//    6
			"|           |       |               |               |       |   |",//12  7
			"+   +---+---+---+---+---+---+---+---+---+---+---+   +   +---+   +",//    8
			"|   |       |       |           |       |   |       |       |   |",//11  9
			"+   +   +   +   +   +   +   +   +   +   +   +   +---+---+   +   +",//   10
			"|   |   |       |       |   |       |           |           |   |",//10 11
			"+   +   +---+---+---+---+---+---+---+---+---+---+---+   +---+   +",//   12
			"|   |       |           |               |       |           |   |",// 9 13
			"+   +---+   +   +   +---+   +   +---+   +   +   +---+   +---+   +",//   14
			"|   |       |   |       |   |       |       |               |   |",// 8 15
			"+   +   +---+   +---+   +   +   +   +---+---+---+---+   +---+   +",//   16
			"|   |   |       |       |   |       |   |                   |   |",// 7 17
			"+   +   +   +---+   +---+   +---+---+   +   +   +---+---+---+   +",//   18
			"|   |       |           |                   |               |   |",// 6 19
			"+   +---+---+   +---+---+---+---+---+---+---+---+---+---+   +   +",//   20
			"|   |           |       |   |       |   |           |       |   |",// 5 21
			"+   +   +---+---+   +   +   +   +   +   +   +   +---+   +---+   +",//   22
			"|   |       |       |           |           |   |   |       |   |",// 4 23
			"+   +---+   +   +---+---+   +---+---+   +---+   +   +---+   +   +",//   24
			"|   |       |       |   |   |       |   |       |           |   |",// 3 25
			"+   +   +---+---+   +   +---+   +   +---+   +---+   +---+---+   +",//   26
			"|   |   |           |           |           |   |   |           |",// 2 27
			"+   +   +   +---+---+   +---+---+---+---+---+   +   +   +   +   +",//   28
			"|       |   |   |   |                               |   |   |   |",// 1 29
			"+   +---+   +   +   +---+---+---+---+---+---+---+---+---+---+   +",//   30
			"|   |                                                           |",// 0 31
			"+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+"//    32
			// 0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15
	};

	for (i=0;i<16;i++){//南北方向
		for (j=0;j<16; j++) {//東西方向
			n_wall = 0x01*(smap[30-2*i][2+4*j]=='-');
			e_wall = 0x02*(smap[31-2*i][4+4*j]=='|');
			s_wall = 0x04*(smap[32-2*i][2+4*j]=='-');
			w_wall = 0x08*(smap[31-2*i][4*j]=='|');
			map[i][j] = s_wall | w_wall | e_wall | n_wall;
		}
	}
}



#if 0
//------------------------------------------------------------------------
//最短
//------------------------------------------------------------------------
void most_short(char y,char x,int spd)
{
	uchar course;
	LED_num(spd/50);
	pause(500);
	control_mode=1;
	if(over_flag!=1){
		hight_map_initalize();
		hight_map(y,x,0,T_MODE);
	}
	else
		over_flag=0;

	while( 1 ){
		// １つのループは区間中心から次の区間中心まで
		// 最初に半区画直進
		rdir = 0; ldir = 0;           // 回転方向を直進
		STEP = 0;                     // 距離カウンタリセット
		speed = spd;                  // 速度設定
		control_mode = 1;             // 姿勢制御ON

		if(sura_flag!=1){while( STEP < GO_STEP / 2 );}  // 半区間進む
		else {STEP = GO_STEP/2;}

		sura_flag=0;
		place_update();               // 現在座標更新
		LED_num(h_map[my][mx]);

		if(my==0&&mx==0&&goal_flag==1)
		{
			while( STEP < GO_STEP - ( speed_now - 1 ) * 2 );
			speed = 1;
			while( STEP < GO_STEP )if(SW_EXEC==SW_ON)return;
			com_turn(2);
			com_stop();
			countdown();
			goal_flag=0;
			hight_map_initalize();
			hight_map(x,y,0,T_MODE);
			control_mode=1;
			return;
		}
		else if(my==y&&mx==x)
		{
			while( STEP < GO_STEP - ( speed_now - 1 ) * 2 );
			speed = 1;
			while( STEP < GO_STEP )if(SW_EXEC==SW_ON)return;
			com_turn(2);
			com_stop();
			hight_map_initalize();
			hight_map(0,0,0,T_MODE);
			countdown();
			goal_flag=1;
			control_mode=1;
		}
		else{
			course=map_load();
			if(course==2)
			{
				while( STEP < GO_STEP - ( speed_now - 1 ) * 2 );
				speed = 1;                    // 残り半区間を減速しながら直進
				while( STEP < GO_STEP )if(SW_EXEC==SW_ON)return;      // さらに半区間進む
				com_turn( 2 );                // 右90度旋回
			}
			else if(course<2)
				com_sura(course);
			else
				while(STEP<GO_STEP)if(SW_EXEC==SW_ON)return;
		}

	}

}

//------------------------------------------------------------------------
//矢印をたどる方向
//------------------------------------------------------------------------
unsigned char vector(void)
{
	uchar vec_tmp;

	vec_tmp=(back_map[my][mx]&0x0f);

	if(vec_tmp>head)
		switch(vec_tmp/head)
		{
		case 1:
			return 3;
			break;
		case 2:
			return 0;
			break;
		case 4:
			return 2;
			break;
		case 8:
			return 1;
			break;
		}
	else
		switch(head/vec_tmp)
		{
		case 1:
			return 3;
			break;
		case 2:
			return 1;
			break;
		case 4:
			return 2;
			break;
		case 8:
			return 0;
			break;
		}
	return 3;
}
#endif
