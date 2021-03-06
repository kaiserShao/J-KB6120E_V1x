/**************** (C) COPYRIGHT 2013 青岛金仕达电子科技有限公司 ****************
* 文 件 名: MBM_RTU.C
* 创 建 者: 董峰
* 描  述  : MODBUS 主机 RTU 模式
* 最后修改: 2013年12月28日
*********************************** 修订记录 ***********************************
* 版  本:
* 修订人:
*******************************************************************************/
#include "MBM_BSP.H"
#include "MBM.H"
#include "MBM_Port.H"

/* ----------------------- Start implementation -----------------------------*/
/* The function returns the CRC as a unsigned short type */
/* message to calculate CRC upon */
/* quantity of bytes in message */
uint16_t	usMBCRC16 ( const uint8_t * puchMsg, uint16_t usDataLen )
{
	static uint8_t const auchCRCHi[] = /* Table of CRC values for high–order byte */
	{
		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
		0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
		0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
		0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
		0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
		0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
		0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
		0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
		0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
		0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
		0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
		0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
		0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
		0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
		0x40
	} ;

	static uint8_t const auchCRCLo[] = /* Table of CRC values for low–order byte */
	{
		0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
		0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
		0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
		0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
		0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
		0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
		0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
		0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
		0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
		0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
		0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
		0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
		0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
		0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
		0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
		0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
		0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
		0x40
	};
	uint8_t uchCRCHi = 0xFFu;	/* high byte of CRC initialized */
	uint8_t uchCRCLo = 0xFFu;	/* low byte of CRC initialized */
	uint8_t	uIndex; 			/* will index into CRC lookup table */

	while ( usDataLen-- ) 		/* pass through message buffer */
	{
		uIndex = uchCRCLo ^ *puchMsg++ ;	/* calculate the CRC */
		uchCRCLo = uchCRCHi ^ auchCRCHi[uIndex] ;
		uchCRCHi = auchCRCLo[uIndex] ;
	}

	return (uchCRCHi << 8 | uchCRCLo) ;
}


/********************************** 功能说明 ***********************************
*  RTU 发送
*******************************************************************************/
static	volatile	uint8_t 	*pucSndBufferCur;
static	volatile	uint16_t	usSndBufferCount;

//	允许RTU发送前的准备工作
void	vMBM_RTU_Send_Init( void )
{
	pucSndBufferCur = ucSerialPDU;
	vMBM_SerialPut_Cmd( TRUE );			//	允许发送
}

//	RTU 事件处理：发送一字节
void 	vMBM_RTU_Send_ISR( void )
{
	/* check if we are finished. */
	if ( 0u != usSndBufferCount )
	{
		vMBM_SerialPutByte( *pucSndBufferCur++ );
		usSndBufferCount--;
	}
	else
	{
		vMBM_SerialPut_Cmd( FALSE );
		vMBM_EventPut_Post();
	}
}

//	装配RTU命令
eMBErrorCode	eMBM_RTU_Assemble( uint8_t ucSlaveAddress, uint16_t usLen )
{
	eMBErrorCode    eStatus = MB_ENOERR;
	uint16_t		usCRC16;

	//	设定接收地址
	ucSerialPDU[MB_SER_PDU_ADDR_OFF] = ucSlaveAddress;
	++usLen;
	//	生成检验 CRC
	usCRC16 = usMBCRC16( ucSerialPDU, usLen );
	ucSerialPDU[usLen++] = LOBYTE( usCRC16 );
	ucSerialPDU[usLen++] = HIBYTE( usCRC16 );

	usSndBufferCount = usLen;

	return eStatus;
}


/********************************** 功能说明 ***********************************
*  RTU 接收
*******************************************************************************/
static	volatile	BOOL		eRcvErrorOccured;
static	volatile	uint16_t	usRcvBufferCount;

//	允许RTU接收前的准备工作
void	vMBM_RTU_Receive_Init( void )
{
	eRcvErrorOccured = FALSE;
	usRcvBufferCount = 0u;
	vMBM_SerialGet_Cmd( TRUE );			//	允许接收
}

//	RTU 事件处理：接收一字节
void	vMBM_RTU_Receive_ISR( void )
{
	uint8_t ucByte;

	/* Always read the character. */
	if ( ! xMBM_SerialGetByte( &ucByte ))
	{
		eRcvErrorOccured = TRUE;
	}

	if ( ! eRcvErrorOccured )
	{
		if ( usRcvBufferCount < MB_SER_PDU_SIZE_MAX )
		{
			ucSerialPDU[usRcvBufferCount++] = ucByte;
		}
		else
		{
			eRcvErrorOccured = TRUE;
		}
	}

	/* Enable t3.5 timers. */
	vMBM_Timers_Cmd( TRUE );
}

//	解析RTU应答
eMBErrorCode	eMBM_RTU_Analyze( uint8_t * pucRcvAddress, uint16_t * pusLen )
{
	eMBErrorCode    eStatus = MB_ENOERR;

	if ( eRcvErrorOccured )
	{
		eStatus = MB_EIO;	//	接收过程发现错误
	}
	else if ( usRcvBufferCount < MB_SER_PDU_SIZE_MIN )
	{
		eStatus = MB_EIO;	//	接收长度异常
	}
	else if ( 0u != usMBCRC16( ucSerialPDU, usRcvBufferCount ))
	{
		eStatus = MB_EIO;	//	CRC 检验失败
	}
	else
	{
		//	成功
		uint16_t	usLen = usRcvBufferCount - MB_SER_PDU_SIZE_CRC;
		uint8_t		ucRcvAddress = ucSerialPDU[MB_SER_PDU_ADDR_OFF];
		* pucRcvAddress = ucRcvAddress;
		* pusLen = usLen - 1u;
	}

	return eStatus;
}


/********************************** 功能说明 ***********************************
*  RTU事件处理：T35
*******************************************************************************/
void	vMBM_RTU_T35_ISR( void )
{
	vMBM_SerialGet_Cmd( FALSE );	//	禁止串口继续接收
	vMBM_EventGet_Post();			//	发送接收完成信号
	vMBM_Timers_Cmd( FALSE );		//	关闭定时器
}

/********  (C) COPYRIGHT 2013 青岛金仕达电子科技有限公司  **** End Of File ****/
