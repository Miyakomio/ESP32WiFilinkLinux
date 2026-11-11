#ifndef PID_H
#define PID_H

typedef struct {
	float Target;//目标值
	float Actual;//实际值
	float Actual1;//上次实际值
	float Out;//输出值
	
	float Kp;//比例系数
	float Ki;//积分系数
	float Kd;//微分系数
	
	float Error0;//当前误差
	float Error1;//上次误差
	float	Diff;
	float	Inter;
	
	float POut;//比例输出
	float IOut;//积分输出
	float DOut;//微分输出
	
	float OutMax;//输出最大值
	float OutMin;//输出最小值
	
	float OutOffset;//输出偏置
	
	float dt;
	float IntegralMax;
	float IntegralMin;
	bool Initialized;

	/* 微分低通滤波（抑制高频噪声） */
	float DFilterCoef;   /* 滤波系数 0~1，越小滤波越强 */
	float DFiltered;     /* 滤波后的微分值 */
	float LastActual;    /* 上一帧实际值（用于微分滤波） */
} PID_t;

#endif

