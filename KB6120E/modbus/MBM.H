/**************** (C) COPYRIGHT 2013 �ൺ���˴���ӿƼ����޹�˾ ****************
* �� �� ��: MBM.H
* �� �� ��: ����
* ��  ��  : MODBUS ���ڵ�˵���ļ�
* ����޸�: 2013��12��28��
*********************************** �޶���¼ ***********************************
* ��  ��: 
* �޶���: 
*******************************************************************************/
#ifdef	__cplusplus
extern "C" {
#endif

typedef enum
{
	MB_RTU,					 		/*!< RTU transmission mode. */
	MB_ASCII,				   		/*!< ASCII transmission mode. */
	MB_TCP					  		/*!< TCP mode. */
} eMBMode;

typedef enum
{
	MB_PAR_NONE,					/*!< No parity. */
	MB_PAR_ODD,				 		/*!< Odd parity. */
	MB_PAR_EVEN				 		/*!< Even parity. */
} eMBParity;

typedef	enum
{
	MB_ENOERR,						//  No error.  
	MB_EPORTERR,					//  Porting layer error.  
	MB_EINVAL,						//	Illegal argument.  
	MB_EILLSTATE,					//  Protocol stack in illegal state.  
	MB_ETIMEDOUT,					//  Timeout error occurred.
	MB_EIO,							//  I/O error.��һ��ָͨѶУ�����

	MB_EX_Response_Unknow = 0x60u,	//	δ֪����Ӧ��Ϣ����
	MB_EX_Response_IllegalSlave,	//	�������ӻ���ַ��Ӧ
	MB_EX_Response_IllegalFunction,	//	��������������Ӧ
	MB_EX_Response_IllegalLength,	//	Ӧ��֡�ĳ�����������һ��
	MB_EX_Response_IllegalByteCnt,  //	��ָ��ص�������������һ��
	MB_EX_Response_IllegalAddress,	//	дָ��صĵ�ַ������һ��
	MB_EX_Response_IllegalValue,	//	дָ��ص�����ֵ������һ��
	MB_EX_Response_IllegalCount,	//	дָ��ص�������������һ��

	MB_EX_Unknow = 0x80u,    		//	δ֪���쳣
	MB_EX_ILLEGAL_FUNCTION,         //  Illegal function exception.              01���쳣���Ƿ�����
	MB_EX_ILLEGAL_DATA_ADDRESS,     //  Illegal data address.                    02���쳣���Ƿ����ݵ�ַ
	MB_EX_ILLEGAL_DATA_VALUE,       //  Illegal data value.                      03���쳣���Ƿ�����ֵ
	MB_EX_SLAVE_DEVICE_FAILURE,     //  Slave device failure.                    04���쳣����վ�豸����
	MB_EX_ACKNOWLEDGE,              //  Slave acknowledge.                       05���쳣��ȷ��
	MB_EX_SLAVE_BUSY,               //  Slave device busy.                       06���쳣�������豸æ
	MB_EX_MEMORY_PARITY_ERROR,      //  Memory parity error.                     08���쳣���洢��ż�Բ��
	MB_EX_GATEWAY_PATH_UNAVAILABLE, //  Gateway path unavailable.                0A���쳣������������·��
	MB_EX_GATEWAY_TARGET_FAILED,   	//  Gateway target device failed to respond. 0B���쳣������Ŀ���豸��Ӧʧ��
} eMBErrorCode ;

/*	��ʼ��MBUS	*/
eMBErrorCode	eMBMInit( eMBMode eMode, uint32_t ulBaudrate, eMBParity eParity );

/* FC01 Read Coils */
eMBErrorCode	eMBMReadCoils( uint8_t Slave, uint16_t Start, uint16_t Count, uint8_t Coils[] );

/* FC02 Read Discretes */
eMBErrorCode	eMBMReadDiscretes( uint8_t Slave, uint16_t Start, uint16_t Count, uint8_t Discretes[] );

/* FC03 Read holding registers */
eMBErrorCode	eMBMReadHoldingRegisters( uint8_t Slave, uint16_t Start, uint16_t Count, uint16_t HoldingRegisters[] );

/* FC04 Read Input Registers */
eMBErrorCode	eMBMReadInputRegisters( uint8_t Slave, uint16_t Start, uint16_t Count, uint16_t InputRegisters[] );

/* FC05	Write Signle Coil */
eMBErrorCode	eMBMWriteSingleCoil( uint8_t Slave, uint16_t Start, BOOL CoilVal );

/* FC06 Write Signle Register */
eMBErrorCode	eMBMWriteSingleRegister( uint8_t Slave, uint16_t Start, uint16_t RegisterVal );

/* FC15	Write Multiple Coil */
eMBErrorCode	eMBMWriteMultipleCoils( uint8_t Slave, uint16_t Start, uint16_t Count, const uint8_t CoilsVal[] );

/* FC16 Write Multiple Registers */
eMBErrorCode	eMBMWriteMultipleRegisters( uint8_t Slave, uint16_t Start, uint16_t Count, const uint16_t RegistersVal[] );

/*
	FC01/FC02	Read DO/DI
	FC03/FC04	Read AO/AI
*/
eMBErrorCode	eMBMRead ( uint8_t SlaveAddress, uint16_t Register, uint16_t Count,	void * pBuf );
/*
	FC15/FC16	Write DO/AO
*/
eMBErrorCode	eMBMWrite( uint8_t SlaveAddress, uint16_t Register, uint16_t Count, const void * pBuf );

#ifdef	__cplusplus
}
#endif
/********  (C) COPYRIGHT 2013 �ൺ���˴���ӿƼ����޹�˾  **** End Of File ****/