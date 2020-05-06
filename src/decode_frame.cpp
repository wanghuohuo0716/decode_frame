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












// //在接收完一帧IMU姿态报告后，调用这个子程序来取出姿态数据
// void UART2_Get_IMU(void)
// {
// 	int16_t temp;

// 	roll=0; //滚转

// 	pitch=0;//俯仰

// 	temp = 0;
// 	temp = rx_buffer[5];
// 	temp <<= 8;
// 	temp |= rx_buffer[4];
// 	// if(temp&0x8000){ // 此段代码将角度范围从[-180,180]转换成[0,360]
// 	// temp = 0-(temp&0x7fff);
// 	// }else temp = (temp&0x7fff);
// 	yaw=(float)temp / 32768.0f*180.0f;//偏航角

// 	ROS_INFO("yaw=%f,pitch=%f,roll=%f", yaw, pitch, roll);


// 	temp = 0;
// 	temp = rx_buffer[7];
// 	temp <<= 8;
// 	temp |= rx_buffer[6];
// 	// if(temp&0x8000){
// 	// temp = 0-(temp&0x7fff);
// 	// }else temp = (temp&0x7fff);
// 	tempr=(float)temp / 100.0f;//温度
// }

// //在接收一帧ReportMotion 后调用这个子程序来取出ADC数据
// void UART2_Get_Motion(void)
// {
// 	int16_t temp;

// 	temp = 0;
// 	temp = rx_buffer[2];
// 	temp <<= 8;
// 	temp |= rx_buffer[3];
// 	if(temp&0x8000){
// 	temp = 0-(temp&0x7fff);
// 	}else temp = (temp&0x7fff);
// 	wx=temp /  32768.0f*2000.0f;//速度计 X轴的ADC值

// 	temp = 0;
// 	temp = rx_buffer[4];
// 	temp <<= 8;
// 	temp |= rx_buffer[5];
// 	if(temp&0x8000){
// 	temp = 0-(temp&0x7fff);
// 	}else temp = (temp&0x7fff);
// 	wy=temp /  32768.0f*2000.0f;//速度计 Y轴的ADC值

// 	temp = 0;
// 	temp = rx_buffer[6];
// 	temp <<= 8;
// 	temp |= rx_buffer[7];
// 	if(temp&0x8000){
// 	temp = 0-(temp&0x7fff);
// 	}else temp = (temp&0x7fff);
// 	wz=temp /  32768.0f*2000.0f;//速度计 Z轴的ADC值

// 	temp = 0;
// 	temp = rx_buffer[8];
// 	temp <<= 8;
// 	temp |= rx_buffer[9];
// 	if(temp&0x8000){
// 	temp = 0-(temp&0x7fff);
// 	}else temp = (temp&0x7fff);
// 	tempr=(float)temp / 100.0f;//温度


// }


// //--校验当前接收到的一帧数据是否 与帧校验字节一致
// unsigned char Sum_check(void)
// {
//   unsigned char i;
//   unsigned int checksum=0x55+0x53;
//   for(i=0;i<8;i++)
//    checksum+=rx_buffer[i];

//   if((checksum%256)==rx_buffer[8])
// 		return(0x01); //Checksum successful
//   else
//    return(0x00); //Checksum error
// }



// //--这个子程序需要在主程序中 定时调用,以检查 串口是否接收完一帧数据
// void UART2_CommandRoute(void)
// {
//  if(RC_Flag_1&b_rx_over){  //已经接收完一帧?
// 		RC_Flag_1&=~b_rx_over; //清标志先
// 		if(Sum_check()){
// 		//校验通过
// 		 UART2_Get_IMU();	//取数据
// 		}//校验是否通过?
// 	}
// }

// //收到一个字节调用一次。把收到的字节做为输入。
// void Decode_frame(unsigned char data){
//   if (data==0x55)
// 	{
// 			RC_Flag_1|=b_uart_head; //如果接收到55 置位帧头标专位
// 			rx_buffer[rx_wr_index++]=data; //保存这个字节
// 		}
//   else if (data==0x53){
// 	  if(RC_Flag_1&b_uart_head) //如果上一个字节是55 那么认定 这个是帧起始字节
// 	    {
// 				rx_wr_index=0;  //重置 缓冲区指针
// 		    RC_Flag_1&=~b_rx_over; //这个帧才刚刚开始收
//       }
// 		else //上一个字节不是55
// 		  rx_buffer[rx_wr_index++]=data;
//     RC_Flag_1&=~b_uart_head; //清帧头标志
//        }
//   else
// 	{
// 			rx_buffer[rx_wr_index++]=data;
// 			RC_Flag_1&=~b_uart_head;
// 			if(rx_wr_index==9) //收够了字节数
// 			{
// 				RC_Flag_1 |= b_rx_over; //置位 接收完整的一帧数据
// 				UART2_CommandRoute(); //立即提取数据
// 			}
// 		}

//   if(rx_wr_index==RX_BUFFER_SIZE) //防止缓冲区溢出
//   rx_wr_index--;
// }
