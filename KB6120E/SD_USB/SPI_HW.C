/* CH376芯片 硬件标准SPI串行连接的硬件抽象层 V1.0 */
/* 提供I/O接口子程序 */
/****************************CH376底层接口***********************************/
#include	"hal.h"
#include "CH376Init.h"
#include "BSP.H"
#include "BIOS.H"
#ifndef	__bool_true_false_are_defined
#include <stdbool.h>
typedef	bool	BOOL;
#endif

extern void	delay_us ( uint32_t us );
uint8_t  CH376_Flag=0;

/*******************************************************************************
* 函  数  名      : CH376_Write
* 描      述      : SPI3向CH376写一个字节
*
* 输      入      : 要向CH376写的字节
* 返      回      : 无
*******************************************************************************/
void CH376_Write( uint8_t CMD_DAT )
{
	bus_SPIxShift( CMD_DAT );
}
/*******************************************************************************
* 函  数  名      : WriteCH376Cmd
* 描      述      : 向CH376写命令
*
* 输      入      : 要向376写的命令 （单字节）
* 返      回      : 无
*******************************************************************************/
void	WriteCH376Cmd( uint8_t mCmd )
{
	bus_SPIxNSS(1);    /* 防止之前未通过xEndCH376Cmd禁止SPI片选 */
	delay_us(5);
	bus_SPIxNSS(0);      /* SPI片选有效 */
	CH376_Write( mCmd );  /* 发出命令码 */
	delay_us( 2 );   /* 延时1.5uS确保读写周期大于1.5uS,或者用上面一行的状态查询代替 */
}
/*******************************************************************************
* 函  数  名      : WriteCH376Data
* 描      述      : 向CH376写数据
*
* 输      入      : 要向3769写的数据
* 返      回      : 无
*******************************************************************************/
void WriteCH376Data(uint8_t dat)
{
	CH376_Write(dat);
}
/*******************************************************************************
* 函  数  名      : ReadCH376Data
* 描      述      : 从CH376读一个字节
*
* 输      入      : 0XFF
* 返      回      : CH376返回的数据
*******************************************************************************/
uint8_t ReadCH376Data(void)
{
	uint8_t d;
	d = bus_SPIxShift(0XFF);
	return( d );
}
/*******************************************************************************
* 函  数  名      : CH376QueryInterrupt
* 描      述      : CH376的中断标志读取函数
*
* 输      入      : 无
* 返      回      : CH376的中断标志状态
*******************************************************************************/
uint8_t CH376QueryInterrupt(void)
{
	if( CH376_Flag == 1 )
	{
		CH376_Flag=0;
		return 1;
	}
	else
	{
		return 0;
	}
}

/*******************************************************************************
* 函  数  名      : EXTI_PC13_Config
* 描      述      : 外部中断线配置
*
* 输      入      : 无
* 返      回      : 无
*******************************************************************************/
void	INT_IRQ_Enable( void )
{
	SET_BIT( RCC->APB2ENR, RCC_APB2ENR_AFIOEN );
	AFIO->EXTICR[3] =  AFIO_EXTICR4_EXTI13_PC;
	CLEAR_BIT( EXTI->EMR,  0x2000u );	// no event
	CLEAR_BIT( EXTI->IMR,  0xDC00u );	// 禁止中断
	SET_BIT(   EXTI->RTSR, 0x0000u );	// rising edge trigger
	SET_BIT(   EXTI->FTSR, 0x2000u );	// falling edge trigger
	WRITE_REG( EXTI->PR,   0x2000u );	// 写1复位中断标志位
	SET_BIT(   EXTI->IMR,  0x2000u );	//
}
/**/
void EXTI15_10_IRQHandler(void)
{
	CH376_Flag=1;
	EXTI->PR=1<<13;
}
void	mDelay0_5uS( void )  /* 至少延时0.5uS,根据单片机主频调整 */
{
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
}



void	CH376_PORT_INIT( void )  /* 由于使用SPI读写时序,所以进行初始化 */
{
	INT_IRQ_Enable();

	bus_SPIxPortInit();
	GPIOA->BSRR = ( 1 << 15 );
	MODIFY_REG( GPIOA->CRH, 0xF0000000u, 0x30000000u );	//	for CS, output
	/* 如果是硬件SPI接口,那么可使用mode3(CPOL=1&CPHA=1)或mode0(CPOL=0&CPHA=0),CH376在时钟上升沿采样输入,下降沿输出,数据位是高位在前 */
	bus_SPIxNSS(1); /* 禁止SPI片选 */
}

UINT8	SPI376Exchange( UINT8 d )  /* 硬件SPI输出且输入8个位数据 */
{
	return bus_SPIxShift( d );
}

void	xEndCH376Cmd( )
{
	bus_SPIxNSS(1);
}  /* SPI片选无效,结束CH376命令,仅用于SPI接口方式 */

void	xWriteCH376Cmd( UINT8 mCmd )  /* 向CH376写命令 */
{

#ifdef	CH376_SPI_BZ
	UINT8	i;
#endif
	bus_SPIxNSS(1);  /* 防止之前未通过xEndCH376Cmd禁止SPI片选 */
	/* 对于双向I/O引脚模拟SPI接口,那么必须确保已经设置SPI_SCS,SPI_SCK,SPI_SDI为输出方向,SPI_SDO为输入方向 */
	bus_SPIxNSS(0);  /* SPI片选有效 */
	SPI376Exchange( mCmd );  /* 发出命令码 */
#ifdef	CH376_SPI_BZ

	for ( i = 30; i != 0 && CH376_SPI_BZ; -- i );  /* SPI忙状态查询,等待CH376不忙,或者下面一行的延时1.5uS代替 */

#else
	mDelay0_5uS( );
	mDelay0_5uS( );
	mDelay0_5uS( );/* 延时1.5uS确保读写周期大于1.5uS,或者用上面一行的状态查询代替 */
#endif
}

#ifdef	FOR_LOW_SPEED_MCU  /* 不需要延时 */
#define	xWriteCH376Data( d )	{ WriteCH376Data( d ); }  /* 向CH376写数据 */
#define	xReadCH376Data( )		(  ReadCH376Data( void ); )  /* 从CH376读数据 */
#else
void	xWriteCH376Data( UINT8 mData )  /* 向CH376写数据 */
{
	WriteCH376Data( mData );
	mDelay0_5uS( );  /* 确保读写周期大于0.6uS */
}
UINT8	xReadCH376Data( void )  /* 从CH376读数据 */
{
	mDelay0_5uS( );  /* 确保读写周期大于0.6uS */
	return( ReadCH376Data( ) );
}
#endif
/* 查询CH376中断(INT#低电平) */
UINT8	Query376Interrupt( void )
{
	return ( CH376QueryInterrupt() );
}
UINT8	res;
UINT8	mInitCH376Host( UINT8 USBSelect )  /* 初始化CH376 */
{

	CH376_PORT_INIT( );  /* 接口硬件初始化 */
	xWriteCH376Cmd( CMD11_CHECK_EXIST );  /* 测试单片机与CH376之间的通讯接口 */
	xWriteCH376Data( 0xaa );
	res = xReadCH376Data( );
	xEndCH376Cmd( );

	if ( res != 0x55 ) return( ERR_USB_UNKNOWN );  /* 通讯接口不正常,可能原因有:接口连接异常,其它设备影响(片选不唯一),串口波特率,一直在复位,晶振不工作 */

	xWriteCH376Cmd( CMD11_SET_USB_MODE );  /* 设备USB工作模式 */
	xWriteCH376Data( USBSelect );  ///06  U盘 03 SD卡
	mDelayuS( 20 );
	res = xReadCH376Data( );
	xEndCH376Cmd( );
#ifndef	CH376_INT_WIRE
#ifdef	CH376_SPI_SDO
	xWriteCH376Cmd( CMD20_SET_SDO_INT );  /* 设置SPI的SDO引脚的中断方式 */
	xWriteCH376Data( 0x16 );
	xWriteCH376Data( 0x90 );  /* SDO引脚在SCS片选无效时兼做中断请求输出 */
	xEndCH376Cmd( );
#endif
#endif

	if ( res == CMD_RET_SUCCESS ) return( USB_INT_SUCCESS );
	else return( ERR_USB_UNKNOWN );  /* 设置模式错误 */
}
