#include "decode_frame.hpp"



//uart reicer flag
#define b_uart_head  80  //收到55 头 标志位
#define b_rx_over    0x40  //收到完整的帧标志
// USART Receiver buffer
#define RX_BUFFER_SIZE 100 //接收缓冲区字节数

void Decode_frame(unsigned char data);
volatile unsigned char rx_buffer[RX_BUFFER_SIZE]; //接收数据缓冲区
volatile unsigned char rx_wr_index; //缓冲写指针
volatile unsigned char RC_Flag_1;  //接收1位标志字节
//解算后的角度值

float 	roll,  //偏航角
				pitch,//俯仰
				yaw, //滚转
				tempr;//温度

float 	wx,wy,wz;//角速度

void DecodeFrame::Decode_frame(unsigned char data){
	static unsigned char ucRxBuffer[250];
	static unsigned char ucRxCnt = 0;
	ucRxBuffer[ucRxCnt++]=data;
	if (ucRxBuffer[0] != 0x55)
	{
		ucRxCnt=0;
		return;
	}
	if (ucRxCnt<11)
		return;
	else
	{
		if(ucRxBuffer[1]==0x53)
		{
			memcpy(&stcAngle,&ucRxBuffer[2],8);
			// 此段代码将角度范围从[-180,180]转换成[0,360]
			// if(stcAngle.Angle[2]&0x8000)
			// 	stcAngle.Angle[2] = 0-(stcAngle.Angle[2]&0x7fff);
			// else
			// 	stcAngle.Angle[2] = (stcAngle.Angle[2]&0x7fff);
			yaw = (float)stcAngle.Angle[2]/32768*180;
			imu_data.x = yaw;
        	imu_data.y = pitch;
        	imu_data.z = roll;
			ROS_INFO("%.3f", yaw);
		}
		ucRxCnt=0;
	}
}