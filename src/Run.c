#include "Run.h"
#include "Motor.h"
#include "Control.h"
#include "Wall_Sensor.h"
#include "Mouse_Data.h"
#include "Wait.h"
#include "My_Func.h"
#include "Mouse_Data.h"

//直進関数
//length		走行距離[mm]
//top_speed		最高速度[mm/s]
//end_speed		終端速度[mm/s]
//acc			加速度[m/s^2]
//wall_control	0:壁制御OFF 1:壁制御ON
void straight(short length, short top_speed, short end_speed, short acc, char wall_control)
{
	//0以下の場合はreturnする
	if(length <= 0 || end_speed < 0 || acc <= 0){return;}

	//モータードライバが無効なら有効にする
	if(get_motor_en_status() == MOTOR_OFF){
		turn_on_motor();
		wait_ms(500);
	}

	reset_motor_step();

	wall_sensor_en = 1;
	wall_control_en = wall_control;

	//壁センサの値を更新させるために少し待つ
	if(now_speed == 0){wait_ms(1);}

	speed_control_en = 1;
	target_speed = top_speed;
	acceleration = acc;
	run_mode = STRAIGHT;

	//short start_speed = now_speed;
	long target_step = len_to_step(length);

	//short accel_length = ((top_speed + start_speed) * (top_speed - start_speed)) / (2 * 1000 * acc);
	short break_length = ((top_speed + end_speed) * (top_speed - end_speed)) / (2 * 1000 * acc);

#if 0
	//加速と減速に必要な距離が走行距離より上回っていたら減速開始地点を変更する
	if(length < (accel_length + break_length)){
		//top_speed^2 - start_speed^2 = 2.0 * 1000.0 * acc * x1(加速距離)
		//end_speed^2 - top_speed^2 = 2.0 * 1000.0 * -acc * x2(減速距離)
		//(x1 + x2 = length)
		//この3つの式を変形すると下式になる
		long speed2 = ((2 * 1000 * acc * length) + (start_speed * start_speed) + (end_speed * end_speed)) / 2;
		break_length = (speed2 - (end_speed * end_speed)) / (2 * 1000 * acc);
	}
#endif

	long break_step = target_step - len_to_step(break_length);

	//減速開始まで待つ
	while(motor_step_sum < break_step);

	target_speed = end_speed;
	if(end_speed == 0){
		target_speed = 1;
	}

	//目標距離に到達するまで待つ
	while(motor_step_sum < target_step);

	if(end_speed == 0){
		target_speed = 0;
		now_speed = 0;
		wall_control_en = 0;
		//wait_ms(250);
		wait_ms(100);

	}

	speed_control_en = 0;
	wall_sensor_en = 0;
	wall_control_en = 0;
}

//直進関数
//length		走行距離[mm]
//top_speed		最高速度[mm/s]
//end_speed		終端速度[mm/s]
//acc			加速度[m/s^2]
//wall_control	0:壁制御OFF 1:壁制御ON
void straight_diag(short length, short top_speed, short end_speed, short acc, char wall_control)
{
	//0以下の場合はreturnする
	if(length <= 0 || end_speed < 0 || acc <= 0){return;}

	//モータードライバが無効なら有効にする
	if(get_motor_en_status() == MOTOR_OFF){
		turn_on_motor();
		wait_ms(500);
	}

	reset_motor_step();

	wall_sensor_en = 1;
	wall_control_en = wall_control;

	//壁センサの値を更新させるために少し待つ
	if(now_speed == 0){wait_ms(1);}

	speed_control_en = 1;
	target_speed = top_speed;
	acceleration = acc;
	run_mode = STRAIGHT_DIAG;

	short start_speed = now_speed;
	long target_step = len_to_step(length);

	short accel_length = ((top_speed + start_speed) * (top_speed - start_speed)) / (2 * 1000 * acc);
	short brake_length = ((top_speed + end_speed) * (top_speed - end_speed)) / (2 * 1000 * acc);

	//加速と減速に必要な距離が走行距離より上回っていたら減速開始地点を変更する
	if(length < (accel_length + brake_length)){
		//top_speed^2 - start_speed^2 = 2.0 * 1000.0 * acc * x1(加速距離)
		//end_speed^2 - top_speed^2 = 2.0 * 1000.0 * -acc * x2(減速距離)
		//(x1 + x2 = length)
		//この3つの式を変形すると下式になる
		long speed2 = ((2 * 1000 * acc * length) + (start_speed * start_speed) + (end_speed * end_speed)) / 2;
		brake_length = (speed2 - (end_speed * end_speed)) / (2 * 1000 * acc);
	}

	long brake_step = target_step - len_to_step(brake_length);

	//減速開始まで待つ
	while(motor_step_sum < brake_step);

	target_speed = end_speed;
	if(end_speed == 0){
		target_speed = 1;
	}

	//目標距離に到達するまで待つ
	while(motor_step_sum < target_step);

	if(end_speed == 0){
		target_speed = 0;
		now_speed = 0;
		wall_control_en = 0;
		//wait_ms(250);
		wait_ms(100);

	}

	speed_control_en = 0;
	wall_sensor_en = 0;
	wall_control_en = 0;
}

//スラローム関数
//at_length		加速ターン走行距離[ステップ数]
//st_length     定速ターン走行距離[ステップ数]
//dir			旋回方向　1:CCW -1:CW 0:角速度を保つ
//acc			加速度[m/s^2]
//wall_control	0:壁制御OFF 1:壁制御ON
void slalom(unsigned short at_length, unsigned short st_length, short acc, char dir, char wall_control)
{
	//0以下の場合はreturnする
	if(at_length <= 0 || st_length <= 0 || acc <= 0){return;}

	//モータードライバが無効なら有効にする
	if(get_motor_en_status() == MOTOR_OFF){
		turn_on_motor();
		wait_ms(500);
	}

	reset_motor_step();

	wall_sensor_en = 1;
	wall_control_en = wall_control;

	//壁センサの値を更新させるために少し待つ
	if(now_speed == 0){wait_ms(1);}

	speed_control_en = 1;
	target_speed = now_speed;
	acceleration = acc;
	turn_dir = dir;
	run_mode = SLALOM;

	//Start !

	//加減速ターン１開始
	//定速開始まで待つ
	while(motor_step_sum < 2*at_length);

    //定速ターン開始
    turn_dir = 0;//定速ターン
    target_speed = now_speed;
    while(motor_step_sum < 2*(at_length + st_length));


	//加減速ターン２開始
	turn_dir = -dir;//反対の減速
	target_speed = now_speed;


	//目標距離に到達するまで待つ
	while(motor_step_sum < 2*(2*at_length + st_length));

	speed_control_en = 0;
	wall_sensor_en = 0;
	wall_control_en = 0;
}




//旧スラローム関数
//length        走行距離[ステップ数]
//dir           旋回方向　1:CCW -1:CW
//acc           加速度[m/s^2]
//wall_control  0:壁制御OFF 1:壁制御ON
void slalom2(unsigned short length, short acc, char dir, char wall_control)
{
    //0以下の場合はreturnする
    if(length <= 0 || acc <= 0){return;}

    //モータードライバが無効なら有効にする
    if(get_motor_en_status() == MOTOR_OFF){
        turn_on_motor();
        wait_ms(500);
    }

    reset_motor_step();

    wall_sensor_en = 1;
    wall_control_en = wall_control;

    //壁センサの値を更新させるために少し待つ
    if(now_speed == 0){wait_ms(1);}

    speed_control_en = 1;
    target_speed = now_speed;
    acceleration = acc;
    turn_dir = dir;
    run_mode = SLALOM;

#if 0
    short start_speed = now_speed;
    long target_step = ((unsigned long)(MOTOR_STEP_NUM * 2 ) * (length) / (unsigned long)(PI*TIRE_DIAMETER)  );

    //long accel_length = length/2;
    //long brake_length = length/2;

    long break_step = target_step/2;// - ((unsigned long)(MOTOR_STEP_NUM * 2 ) * (brake_length) / (unsigned long)(PI*TIRE_DIAMETER)  );;
#endif
    //減速開始まで待つ
    while(motor_step_sum < length/2);

    turn_dir = -dir;//反対の減速
    target_speed = now_speed;


    //目標距離に到達するまで待つ
    while(motor_step_sum < length);

//  if(end_speed == 0){
//      target_speed = 0;
//      now_speed = 0;
//      wall_control_en = 0;
//      //wait_ms(250);
//      wait_ms(100);
//  }

    speed_control_en = 0;
    wall_sensor_en = 0;
    wall_control_en = 0;
}



//超信地旋回
//angle		旋回角度[dig] 正:反時計回り 負:時計回り
void turn(short angle)
{
	//移動中なら止まるまで待つ
	target_speed = 0;
	acceleration = 1;
	while(now_speed != 0);

	//目標角度が0ならreturn
	if(angle > 0)		{turn_dir = 1;}
	else if(angle < 0)	{turn_dir = -1;}
	else				{return;}

	//モータードライバが無効なら有効にする
	if(get_motor_en_status() == MOTOR_OFF){
		turn_on_motor();
		wait_ms(500);
	}

	angle = my_abs(angle);
	angle %= 360;

	//目標ステップ数計算
	long target_step = angle_to_step(angle);

	reset_motor_step();
	speed_control_en = 1;
	target_speed = TURN_SPEED;
	acceleration = TURN_ACC;
	run_mode = TURN;
	wall_sensor_en = 0;
	wall_control_en = 0;

	//減速開始まで待つ
	while(motor_step_sum < (target_step / 2));

	target_speed = 1;

	//目標距離に到達するまで待つ
	while(motor_step_sum < target_step);

	target_speed = 0;
	now_speed = 0;

	//wait_ms(250);
	wait_ms(100);
	speed_control_en = 0;
}

//連続超信地旋回
//angle		旋回角度[dig] 正:反時計回り 負:時計回り
void cturn(short angle)
{
	//移動中なら止まるまで待つ
	target_speed = 0;
	acceleration = 1;
	while(now_speed != 0);

	//目標角度が0ならreturn
	if(angle > 0)		{turn_dir =  1;}
	else if(angle < 0)	{turn_dir = -1;}
	else				{return;}

	//モータードライバが無効なら有効にする
	if(get_motor_en_status() == MOTOR_OFF){
		turn_on_motor();
		wait_ms(500);
	}

	angle = my_abs(angle);
	//angle %= 360;

	//motor control config
    target_speed = 100;//MIN_SPEED;
    acceleration = 1;
    run_mode = TURN;
    wall_sensor_en = 0;
    wall_control_en = 0;

	//目標ステップ数計算
	long target_step = angle_to_step(angle);
    long break_step = target_step - 48;

     //Start motor control !
	reset_motor_step();
	speed_control_en = 1;


	//減速開始まで待つ
	while(motor_step_sum < (break_step));

	target_speed = 1;

	//目標距離に到達するまで待つ
	while(motor_step_sum < target_step);

	target_speed = 0;
	now_speed = 0;

	wait_ms(250);
	speed_control_en = 0;
}
