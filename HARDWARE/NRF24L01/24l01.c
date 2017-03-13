#include "delay.h"
#include "spi.h"	
#include "24L01.h"					  
//////////////////////////////////////////////////////////////////////////////////
extern u8 tmp_buf[32];
extern u8 txbuf[32];
u8 tmp_buf2[32]={0X22};
u8 TX_ADDRESS[5]  = {0x12,0x34,0x56,0x78,0xA1};//路由节点地址，更改2-5通道时改第一个地址即可
u8 RX_ADDRESS[5]  = {0x12,0x34,0x56,0x78,0xA1};//路由节点地址，更改2-5通道时改第一个地址即可
//初始化24L01的IO口
void NRF24L01_Init(void)
{
  	RCC->APB2ENR|=1<<3;     //使能PORTB时钟 	    
 	RCC->APB2ENR|=1<<2;		//使能PORTA时钟

	GPIOB->CRL&=0XFFFFFFF0; 
	GPIOB->CRL|=0X00000008;	//PB0 IRQ 输入 	
	GPIOB->ODR|=1;    		//PB0 上拉	 
	GPIOB->CRH&=0XFFFF0FFF; 
	GPIOB->CRH|=0X00003000;	//PB11 CE 输出   	    				   
	GPIOA->CRL&=0XFFF0FFFF; 
	GPIOA->CRL|=0X00030000;	//PA4 CSN 推挽输出 	       
	
	SPI1_Init();    		//初始化SPI	  
	//针对NRF的特点修改SPI的设置
 	SPI1->CR1&=~(1<<6); 	//SPI设备失能
	SPI1->CR1&=~(1<<1); 	//空闲模式下SCK为0 CPOL=0
	SPI1->CR1&=~(1<<0); 	//数据采样从第1个时间边沿开始,CPHA=0  
	SPI1->CR1|=1<<6; 		//SPI设备使能

	SPI1_SetSpeed(SPI_SPEED_4); //spi速度为9Mhz（24L01的最大SPI时钟为10Mhz）    // 不用每次都设置SPI速度
	NRF24L01_CE=0; 			//使能24L01
	NRF24L01_CSN=1;			//SPI片选取消	 		 	 
}
 	 	 
//SPI写寄存器
//reg:指定寄存器地址
//value:写入的值
u8 NRF24L01_Write_Reg(u8 reg,u8 value)
{
	u8 status;	
   	NRF24L01_CSN=0;                 //使能SPI传输
  	status =SPI1_ReadWriteByte(reg);//发送寄存器号 
  	SPI1_ReadWriteByte(value);      //写入寄存器的值
  	NRF24L01_CSN=1;                 //禁止SPI传输	   
  	return(status);       			//返回状态值
}
//读取SPI寄存器值
//reg:要读的寄存器
u8 NRF24L01_Read_Reg(u8 reg)
{
	u8 reg_val;	    
 	NRF24L01_CSN = 0;          //使能SPI传输		
  	SPI1_ReadWriteByte(reg);   //发送寄存器号
  	reg_val=SPI1_ReadWriteByte(0XFF);//读取寄存器内容
  	NRF24L01_CSN = 1;          //禁止SPI传输		    
  	return(reg_val);           //返回状态值
}	
//在指定位置读出指定长度的数据
//reg:寄存器(位置)
//*pBuf:数据指针
//len:数据长度
//返回值,此次读到的状态寄存器值 
u8 NRF24L01_Read_Buf(u8 reg,u8 *pBuf,u8 len)
{
	u8 status,u8_ctr;	       
  	NRF24L01_CSN = 0;           //使能SPI传输
  	status=SPI1_ReadWriteByte(reg);//发送寄存器值(位置),并读取状态值   	   
 	for(u8_ctr=0;u8_ctr<len;u8_ctr++)pBuf[u8_ctr]=SPI1_ReadWriteByte(0XFF);//读出数据
  	NRF24L01_CSN=1;       //关闭SPI传输
  	return status;        //返回读到的状态值
}
//在指定位置写指定长度的数据
//reg:寄存器(位置)
//*pBuf:数据指针
//len:数据长度
//返回值,此次读到的状态寄存器值
u8 NRF24L01_Write_Buf(u8 reg, u8 *pBuf, u8 len)
{
	u8 status,u8_ctr;	    
 	NRF24L01_CSN = 0;          //使能SPI传输
  	status = SPI1_ReadWriteByte(reg);//发送寄存器值(位置),并读取状态值
  	for(u8_ctr=0; u8_ctr<len; u8_ctr++)SPI1_ReadWriteByte(*pBuf++); //写入数据	 
  	NRF24L01_CSN = 1;       //关闭SPI传输
  	return status;          //返回读到的状态值
}				   
 
//启动NRF24L01发送一次数据
//txbuf:待发送数据首地址
//返回值:0，接收完成；其他，错误代码
u8 VAL=0;
u8 sss=0;
u8 NRF24L01_RxPacket(u8 *rxbuf,u8*txbuf)
{
	u8 sta;	 							   
	sta=NRF24L01_Read_Reg(STATUS);  //读取状态寄存器的值   		
	NRF24L01_Write_Reg(WRITE_REG+STATUS,sta); //清除TX_DS或MAX_RT中断标志
	if(sta&RX_OK)//发送完成
	{	 	 
		VAL=NRF24L01_Read_Reg(0X60);
	//	NRF24L01_Write_Buf(W_ACK_PYLOD,txbuf,VAL);			// 发送ACK 数据包
		NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,VAL);			// 读取已接收数据
		NRF24L01_Write_Reg(FLUSH_RX,0xff);					//清除RX FIFO寄存器 
		return 	128;
	}   
	return 1;//没收到任何数据
}				    
//该函数初始化NRF24L01到RX模式
//设置RX地址,写RX数据宽度,选择RF频道,波特率和LNA HCURR
//当CE变高后,即进入RX模式,并可以接收数据了
u8 FF1=0,DYN=0;; 		   
void NRF24L01_RX_Mode(void)
{
	NRF24L01_CE=0;	  
  	NRF24L01_Write_Buf(WRITE_REG+RX_ADDR_P0,(u8*)RX_ADDRESS,RX_ADR_WIDTH);//写RX节点地址
  	NRF24L01_Write_Buf(WRITE_REG+TX_ADDR,(u8*)TX_ADDRESS,TX_ADR_WIDTH);//写TX节点地址 
			  
  	NRF24L01_Write_Reg(WRITE_REG+EN_AA,0x01);    //使能通道0的自动应答    
	NRF24L01_Write_Reg(WRITE_REG+SETUP_RETR,0X1A);//设置自动重发间隔时间:500us + 86us;最大自动重发次数:10次
  	NRF24L01_Write_Reg(WRITE_REG+RF_CH,40);	     //设置RF通信频率		
	
    NRF24L01_Write_Reg(NRF_WRITE_REG|NRF_FEATURE , NRD_EN_DPL | NRF_EN_ACK_PAYLOAD);
    if(NRF24L01_Read_Reg(NRF_FEATURE)== 0x00 && NRF24L01_Read_Reg(NRF_DYNPD) == 0x00)
	{
        NRF24L01_Write_Reg(NRF_ACTIVATE, 0x73);
    }
    NRF24L01_Write_Reg(NRF_WRITE_REG|NRF_FEATURE , NRD_EN_DPL | NRF_EN_ACK_PAYLOAD);
    NRF24L01_Write_Reg(NRF_WRITE_REG|NRF_DYNPD, NRF_DPL_P0 | NRF_DPL_P1| NRF_DPL_P2| NRF_DPL_P3| NRF_DPL_P4| NRF_DPL_P5);
    	    	  
  	NRF24L01_Write_Reg(WRITE_REG+RF_SETUP,0x0F);//设置TX发射参数,0db增益,1Mbps,低噪声增益开启   
  	NRF24L01_Write_Reg(WRITE_REG+CONFIG, 0x0f);//配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式 
  	NRF24L01_CE = 1; //CE为高,进入接收模式 
}						 

//检测24L01是否存在
//返回值:0，成功;1，失败	
u8 NRF24L01_Check(void)
{
	u8 buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
	u8 i;
	NRF24L01_Write_Buf(WRITE_REG+TX_ADDR,buf,5);//写入5个字节的地址.	
	NRF24L01_Read_Buf(TX_ADDR,buf,5); //读出写入的地址  
	for(i=0;i<5;i++)if(buf[i]!=0XA5)break;	 							   
	if(i!=5)return 1;//检测24L01错误	
	return 0;		 //检测到24L01
}	 







 






