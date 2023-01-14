#ifndef MOUSE_DATA_H_
#define MOUSE_DATA_H_

//lipoセル数
#define BAT_CELL		(3)

//タイヤ直径[mm] 1000倍の値 進み過ぎる時は増やす
#define TIRE_DIAMETER	(26732) //((26.85) * 1000.0)26.687 26.85 26.524 26.732
//トレッド幅[mm] 1000倍の値 回転しすぎる時は減らす
#define TREAD			(73015)//(73.015) * 1000.0
#define MOTOR_STEP_NUM	(400L)

//最低速度[mm/s] 小さくしすぎないこと
#define MIN_SPEED		(50)
//最高速度[mm/s]
#define MAX_SPEED		(2000)
//ターン時の最高速度[mm/s]
#define TURN_SPEED		MAX_SPEED
//ターン時の加速度[m/ss]
#define TURN_ACC		(2)
//左手デモ走行速度
#define LEFT_SPEED		(840)


//迷路1区画の距離[mm]
#define SECTION			(180)
//迷路半区画の距離[mm]
#define H_SECTION		(SECTION/2)

#define GOALX (8)
#define GOALY (8)


#endif /* MOUSE_DATA_H_ */
