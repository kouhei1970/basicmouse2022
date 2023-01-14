#ifndef MOUSE_DATA_H_
#define MOUSE_DATA_H_

//lipoセル数
#define BAT_CELL		(3)

//タイヤ直径[mm] 1000倍の値 進み過ぎる時は増やす
#define TIRE_DIAMETER	((26.5 + 0.05) * 1000.0)
//トレッド幅[mm] 1000倍の値 回転しすぎる時は減らす
#define TREAD			((72.0 + 0.08) * 1000.0)
//ステッピングモータ1回転あたりのステップ数
#define MOTOR_STEP_NUM	(400L)

//最低速度[mm/s] 小さくしすぎないこと
#define MIN_SPEED		(50)
//最高速度[mm/s]
#define MAX_SPEED		(2000)
//ターン時の最高速度[mm/s]
#define TURN_SPEED		MAX_SPEED
//ターン時の加速度[m/ss]
#define TURN_ACC		(2)

//迷路1区画の距離[mm]
#define SECTION			(180)
//迷路半区画の距離[mm]
#define H_SECTION		(SECTION/2)


#endif /* MOUSE_DATA_H_ */
