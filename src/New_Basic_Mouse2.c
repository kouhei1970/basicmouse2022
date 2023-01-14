/****************************************************************************/
/*																			*/
/*			NewBasicMouse ver 0.1			2018/07/14						*/
/*																			*/
/****************************************************************************/
/* 2022/7/13 Open Campus Mod*/

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
#include "search.h"

void mode0(void);
void mode1(void);
void mode2(void);
void mode3(void);
void mode4(void);
void mode5(void);
void mode6(void);
void mode7(void);
void mode8(void);
void mode9(void);
void mode10(void);
void mode11(void);
void mode12(void);
void mode13(void);
void mode14(void);
void mode15(void);
void select_mode(void);
void init(void);
void pullup_unused_port(void);

void main(void)
{
	init();
	//check_battery();	//動作確認が終わったらコメントアウトを外してください
	reset_log();

	pipo_buzzer();

	while(1){
		select_mode();
	}
}

//タクトスイッチを押したらLED点灯
void mode0(void)
{
	//地図の初期化
	reset_map();

	search2018(GOALX, GOALY);
	wait_ms(1000);
	LED_stream();
}

//重ね探索
void mode1(void)
{
	//地図の初期化
	reset_map();
	load_map();
	search2018(GOALX, GOALY);
	wait_ms(1000);
	LED_stream();

}

//Front wall Adjust
void mode2(void)
{

	//load_map();//もともとあったコード
	//church_save_buzzer();//もともとあったコード
    short lf=3338;
    short rf=2298;

    straight(70, -100, 0, 2, 0);
    while(100*(sensor_lf+sensor_rf)<95*(lf+rf)){
        straight(1, 100, 100, 2, 0);

    }
    straight(1, 100, 0, 2, 0);


}

//壁センサーをPC画面に表示
void mode3(void)
{
	output_wall_sensor();
}

//タクトスイッチを押すとモーターが回る
void mode4(void)
{
	while(1){
		if(get_sw_state(UP) && get_sw_state(DOWN)){
			change_motor_speed(200, 200);
		}
		else if(get_sw_state(UP)){
			change_motor_speed(0, 200);
		}
		else if(get_sw_state(DOWN)){
			change_motor_speed(200, 0);
		}
		else{
			change_motor_speed(0, 0);
		}
		wait_ms(50);
		xprintf("left_step:%6d, right_step:%6d\n", motor_step_l, motor_step_r);
	}
}

//指定区画前進
void mode5(void)
{
	short n = get_num();
	set_wall_gain(5);		//壁制御ゲインをセット

	turn_on_motor();
	wait_sw_on();
	wait_ms(1000);
	update_center_ref();
	LED_on(4);
	straight(SECTION*n, 500, 0, 1, 0);
	LED_off(4);
}

//その場旋回
void mode6(void)
{
	turn_on_motor();
	wait_sw_on();
	wait_ms(1000);
	LED_on(4);
	for(short i = 0; i < 1; i++){
		cturn(360*5);
	}
	LED_off(4);
}

//左手法(極地旋回)
void mode7(void)
{
	wait_sw_on();
	wait_ms(1000);

	turn(-90);
	straight(70, -100, 0, 2, 0);	//ケツタッチ
	straight(34, 500, 0, 2, 0);
	turn(90);
	straight(70, -100, 0, 2, 0);	//ケツタッチ
	wait_ms(500);
	update_center_ref();
	straight(34, 500, 0, 2, 0);

	set_wall_gain(5);
	short acc=4;
	straight(H_SECTION , LEFT_SPEED, LEFT_SPEED, acc, 1);

	while(1){
		if(wall_ls == 0){							//左壁なし
			straight(H_SECTION, LEFT_SPEED, 0, acc, 1);
			turn(90);
			straight(H_SECTION, LEFT_SPEED, LEFT_SPEED, acc, 1);
		}
		else if((wall_lf == 0) && (wall_rf == 0)){	//前壁なし
			straight(SECTION, LEFT_SPEED, LEFT_SPEED, acc, 1);
		}
		else if(wall_rs == 0){						//右壁なし
			straight(H_SECTION, LEFT_SPEED, 0, acc, 1);
			turn(-90);
			straight(H_SECTION, LEFT_SPEED, LEFT_SPEED, acc, 1);
		}
		else{										//袋小路
			straight(H_SECTION, LEFT_SPEED, 0, acc, 1);
			turn(180);
			straight(H_SECTION, LEFT_SPEED, LEFT_SPEED, acc, 1);
		}
	}
}

//スラローム調整
void mode8(void)
{
	short acc=2;
	short speed=350;
	short tspeed=350;
	short pre_r=19;// after 40
    short after_r = 23;

    short pre_l = 17;
    short after_l = 22;

	unsigned short atr = 229;
	unsigned short str = 113;

    unsigned short atl = 230;
    unsigned short stl = 118;

	unsigned short slalen_l=1150;// after 91
    unsigned short slalen_r=1129;
    turn_on_motor();
    wait_sw_on();
    wait_ms(1000);


     //自動的にコースの真ん中にセットしセンサーキャリブレーション
     turn(RIGHT90);                  //右向き右90度
     straight(70, -100, 0, 2, 0);    //後退ケツタッチ
     straight(34, 350, 0, 2, 0);     //真ん中に前進
     turn(LEFT90);                   //前向きなおり左90度
     straight(70, -100, 0, 2, 0);    //後退ケツタッチ
     wait_ms(500);
     update_center_ref();            //センサーキャリブレーション



     //マウス出発
     hotel_buzzer();

     straight(90 + 34, speed, speed, 2, 0);  //124mm前進 スタート

     //CW
     straight(pre_l, speed, tspeed, acc, 0);
     slalom(atl, stl, acc, 1, 0);
     straight(after_l, speed, speed, acc, 0);

    short n=8;
	for(short i=0;i<n;i++){
#if 1
	    //CW
		straight(pre_l, speed, tspeed, acc, 0);
		slalom(atl, stl, acc, 1, 0);
		straight(after_l, speed, speed, acc, 0);
        straight(180, speed, speed, acc, 0);
#endif
#if 0
        // right around
        straight(pre_ns, speed, tspeed, acc, 0);
        slalom(slalen_r, acc, -1, 0);
        straight(pre_ew, speed, speed, acc, 0);
        straight(180, speed, speed, acc, 0);

        straight(pre_ew, speed, tspeed, acc, 0);
        slalom(slalen_r, acc, -1, 0);
        straight(pre_ns, speed, speed, acc, 0);
        straight(180, speed, speed, acc, 0);
#endif
#if 0
        // left around
        straight(pre_l_ns, speed, tspeed, acc, 0);
        slalom(slalen_l, acc, 1, 0);
        straight(pre_l_ew, speed, speed, acc, 0);
        straight(180, speed, speed, acc, 0);

        straight(pre_l_ew, speed, tspeed, acc, 0);
        slalom(slalen_l, acc, 1, 0);
        straight(pre_l_ns, speed, speed, acc, 0);
        straight(180, speed, speed, acc, 0);
#endif


	}
    straight(90, speed, 0, acc, 0);

}

#if 0
//左手法（スラローム）
void mode8(void)
{
    wait_sw_on();
    wait_ms(1000);

    turn(-90);
    straight(70, -100, 0, 2, 0);    //ケツタッチ
    straight(34, 500, 0, 2, 0);
    turn(90);

    straight(70, -100, 0, 2, 0);    //ケツタッチ

    wait_ms(500);
    update_center_ref();
    set_wall_gain(5);

    straight(34, 500, 0, 2, 0);


    short acc=4;
    short speed=350;
    short tspeed=350;
    short pre=40;// defo 47
    short slalen=85;// defo 73
    straight(H_SECTION , speed, speed, acc, 1);

    while(1){
        if(wall_ls == 0){                           //左壁なし
            straight(pre, speed, tspeed, acc, 1);
            slalom(slalen, acc, 1, 0);
            straight(pre, speed, speed, acc, 1);
        }
        else if((wall_lf == 0) && (wall_rf == 0)){  //前壁なし
            straight(SECTION, speed, speed, acc, 1);
        }
        else if(wall_rs == 0){                      //右壁なし
            straight(pre, speed, tspeed, acc, 1);
            slalom(slalen, acc, -1, 0);
            straight(pre, speed, speed, acc, 1);
        }
        else{                                       //袋小路
            straight(H_SECTION, speed, 0, acc, 1);
            turn(180);
            straight(H_SECTION, speed, speed, acc, 1);
        }
    }

}
#endif



//mapデータの保存
void mode9(void)
{
	reset_map();

	//各区画の東側の壁だけセットする
	for(short x = 0; x < MAZE_SIZE; x++){
		for(short y = 0; y < MAZE_SIZE; y++){
			set_map(x, y, 1, 1);
		}
	}

	output_map();	//map配列をPCに出力
	save_map();		//map配列をデータフラッシュに保存
	reset_map();	//map配列をリセット
    save_map();     //map配列をデータフラッシュに保存
	load_map();		//データフラッシュからmap配列にデータをロード
	output_map();	//map配列をPCに出力
}

//ログ取り
void mode10(void)
{
    //
	set_log_var(sensor_ls, short, 0);
	set_log_var(sensor_rs, short, 1);
	set_log_var(now_speed, short, 2);

	turn_on_motor();
	wait_sw_on();
	wait_ms(1000);
	update_center_ref();
	set_wall_gain(5);

	LED_on(4);
	//start_ring_log();
	start_log();
	straight(SECTION*2, LEFT_SPEED, 0, 4, 1);
	LED_off(4);

	output_log();
}



//バッテリ電圧をPC画面に表示
void mode11(void)
{
	while(1){
		xprintf("battery:%5d[mV]\n", battery_vol);
		wait_ms(100);
	}
}

//
void mode12(void)
{
    load_map();
	output_map();
}

//ブザーを鳴らす
void mode13(void)
{
	while(1){

		wait_ms(500);
		church_save_buzzer();
		wait_ms(500);
		hotel_buzzer();
        wait_ms(500);
        level_up_buzzer();
        wait_ms(500);
        coin_buzzer();
        wait_ms(500);
        pipo_buzzer();

		/*
		sound_buzzer(scale[BUZ_LA0], 500);
		sound_buzzer(scale[BUZ_SI0], 500);
		sound_buzzer(scale[BUZ_DO1], 500);
		sound_buzzer(scale[BUZ_RE1], 500);
		sound_buzzer(scale[BUZ_MI1], 500);
		sound_buzzer(scale[BUZ_FA1], 500);
		sound_buzzer(scale[BUZ_SO1], 500);
		sound_buzzer(scale[BUZ_LA1], 500);
		sound_buzzer(scale[BUZ_SI1], 500);
		sound_buzzer(scale[BUZ_DO2], 500);
		sound_buzzer(scale[BUZ_RE2], 500);
		sound_buzzer(scale[BUZ_MI2], 500);
		sound_buzzer(scale[BUZ_FA2], 500);
		sound_buzzer(scale[BUZ_SO2], 500);
		sound_buzzer(scale[BUZ_LA2], 500);
		sound_buzzer(scale[BUZ_SI2], 500);
		sound_buzzer(scale[BUZ_DO3], 500);
		*/
	}
}

//Sensor value output
void mode14(void)
{
    while(!get_sw_state(EXEC))
        output_wall_sensor();

}

//スイッチを押したらLEDが点灯する
void mode15(void)
{
	while(1){
		if(get_sw_state(DOWN)){
			LED_on(1);
		}
		else{
			LED_off(1);
		}
		if(get_sw_state(UP)){
			LED_on(2);
		}
		else{
			LED_off(2);
		}
		if(get_sw_state(EXEC)){
			LED_on(3);
		}
		else{
			LED_off(3);
		}
	}

}

//モードを選択する
void select_mode(void)
{
	switch(get_num()){
	case  0:mode0();break;
	case  1:mode1();break;
	case  2:mode2();break;
	case  3:mode3();break;
	case  4:mode4();break;
	case  5:mode5();break;
	case  6:mode6();break;
	case  7:mode7();break;
	case  8:mode8();break;
	case  9:mode9();break;
	case 10:mode10();break;
	case 11:mode11();break;
	case 12:mode12();break;
	case 13:mode13();break;
	case 14:mode14();break;
	case 15:mode15();break;
	}
}

//初期設定
void init(void)
{
	init_clock();
	pullup_unused_port();
	init_ui();
	init_wait();
	init_SCI();
	xdev_out(print_char_sci);	//xprintf関数に1文字出力する関数を設定する
	init_ADC();
	init_buzzer();
	init_wall_sensor();
	init_motor();
	init_Data_Flash();
	init_interrupt_1ms();
	init_interrupt_50us();
}

//使ってないポートをプルアップに設定する
void pullup_unused_port(void)
{
	PORT4.PCR.BIT.B4 = 1;
	PORTE.PCR.BIT.B0 = 1;
	PORTE.PCR.BIT.B1 = 1;
	PORTE.PCR.BIT.B2 = 1;
	PORTE.PCR.BIT.B3 = 1;
	PORTE.PCR.BIT.B4 = 1;
	PORTE.PCR.BIT.B5 = 1;
	PORTA.PCR.BIT.B0 = 1;
	PORTA.PCR.BIT.B1 = 1;
	PORTA.PCR.BIT.B3 = 1;
	PORTA.PCR.BIT.B4 = 1;
	PORTA.PCR.BIT.B6 = 1;
	PORT0.PCR.BIT.B3 = 1;
	PORT0.PCR.BIT.B5 = 1;
	PORTB.PCR.BIT.B7 = 1;
	PORTC.PCR.BIT.B3 = 1;
	PORTC.PCR.BIT.B4 = 1;
	PORTC.PCR.BIT.B5 = 1;
	PORTH.PCR.BIT.B1 = 1;
	PORT1.PCR.BIT.B6 = 1;
	PORT2.PCR.BIT.B7 = 1;
}
