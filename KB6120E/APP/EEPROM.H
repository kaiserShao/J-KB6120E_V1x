/**************** (C) COPYRIGHT 2014 �ൺ���˴���ӿƼ����޹�˾ ****************
* �� �� ��: EEPROM.H
* �� �� ��: ����
* ��  ��  : KB-612E ������ݷ��ʣ��ڴ��ļ����������ķ��ʽӿڡ�
* ����޸�: 2014��4��21��
*********************************** �޶���¼ ***********************************
* ��  ��: V2.0
* �޶���: ����
* ˵  ��: ���ط����ļ��Ľӿڣ������ļ��ṹʹ��ͬһ���������ʡ�
*********************************** ���˵�� ***********************************
�ļ����0..Max����Max+1���ļ�������ǰ�ļ����ڽ��м�¼��ʵ�ʿɲ�ѯ���ļ���Max����
������ʼʱ��Ϊ�㣬��ʾ�ǿ��ļ���ע�����ļ�Ҳ�������ļ���
*******************************************************************************/
#include <time.h>

#define	FileNum_Max			10000u
#define	PowerLogIndex_Max	10000u

////////////////////////////////////////////////////////////////////////////////
//	�ꡢ�䡢���� ��ȡ
////////////////////////////////////////////////////////////////////////////////
extern	void	ConfigureSave( void );
extern	void	ConfigureLoad( void );
extern	void	ConfigureLoadDefault( void );
extern	void	CalibrateSave( void );	
extern	void	CalibrateLoad( void );
extern	void	SampleSetSave( void );
extern	void	SampleSetLoad( void );
extern	void	KB6102_CalibrateSave( void );	//	enum enumBoardSelect Board );
extern	void	KB6102_CalibrateLoad( void );	//	enum enumBoardSelect Board );


//	//	2014��2��22��	ʹ�ú���������
//	void	File_Load( uint16_t FileNum, struct uFile_TSP * pFile );
//	void	File_Load( uint16_t FileNum, struct uFile_AIR * pFile );
//	void	File_Load( uint16_t FileNum, struct uFile_R24 * pFile );
//	void	File_Load( uint16_t FileNum, struct uFile_SHI * pFile );
//	void	File_Save( uint16_t FileNum, struct uFile_TSP const * pFile );
//	void	File_Save( uint16_t FileNum, struct uFile_AIR const * pFile );
//	void	File_Save( uint16_t FileNum, struct uFile_R24 const * pFile );
//	void	File_Save( uint16_t FileNum, struct uFile_SHI const * pFile );

////////////////////////////////////////////////////////////////////////////////
//	���ۼ�ʱ���¼
////////////////////////////////////////////////////////////////////////////////
extern	uint32_t	PumpSumTimeLoad( uint8_t Select );
extern	void		PumpSumTimeSave( uint8_t Select, uint32_t SumTime );

////////////////////////////////////////////////////////////////////////////////
//	�����ػ���¼���ļ��еĸ�ʽ
////////////////////////////////////////////////////////////////////////////////
struct	uPowerLog
{
	uint32_t boot;	//	�������ϵ磩ʱ��
	uint32_t shut;	//	�ػ������磩ʱ��
};

extern	void	PowerLogSave_Boot( void );
extern	void	PowerLogSave_Shut( void );
extern	void	PowerLogLoad( uint16_t index, struct uPowerLog * pLog );
extern	void	PowerLog_Clean( void );
////////////////////////////////////////////////////////////////////////////////
//	�ļ���ȡ
////////////////////////////////////////////////////////////////////////////////
//	struct	uFile
//	{
//		virtual	void	Save( uint16_t FileNum ) = 0;
//		virtual	void	Load( uint16_t FileNum ) = 0;
//	};

struct	uFile_TSP //: public uFile
{
	//	�ļ���Ϣ �� ��������
	uint32_t	sample_begin;		//	��¼������ʼʱ��
	uint16_t	set_time;			//	���ò���ʱ��
	uint16_t	set_flow;			//	���ò�������
	uint8_t		set_loops; 			//	���ò�������
	uint8_t		run_loops;			//	��¼��������
	//	���������е�ͳ�Ʋ���
	uint16_t	sum_min;
	FP32		sum_Ba;				//	����ѹ��
	FP32		sum_tr;				//	��ǰ�¶�
	FP32		sum_pr;				//	��ǰ�¶�
	FP32		max_pr;				//	��ǰѹ�����ֵ
	//	�������ۼ�ֵ
	FP32		vd, vnd;			//	�ۼƲ������
	
//	public:
//		void	Save( uint16_t FileNum );
//		void	Load( uint16_t FileNum );
// 	uint32_t	DataValidMask;
};

struct	uFile_SHI
{
	//	�ļ���Ϣ �� ��������
	uint32_t	sample_begin;		//	��¼������ʼʱ��
	uint16_t	set_time[2];		//	���ò���ʱ��
	uint16_t	set_flow[2];		//	���ò�������
	uint8_t		set_loops; 			//	���ò�������
	uint8_t		run_loops;			//	��¼��������
	//	���������е�ͳ�Ʋ���
	uint16_t	sum_min[2];			//	ʵ�ʲ���ʱ�䣬ÿ����һ�ƣ����24Сʱ
	FP32		sum_Ba;				//	����ѹ��
	FP32		sum_tr[2];			//	��ǰ�¶�
	FP32		sum_pr[2];			//	��ǰѹ��
	FP32		max_pr[2];			//	��ǰѹ�����ֵ
	//	�������ۼ�ֵ
	FP32		vnd[2];				//	�ۼƵĲ��������ֻ��¼������
	FP32		vd[2];				//	2015��7��23�� �ۼƱ�����

//	public:
//		void	Save( uint16_t FileNum );
//		void	Load( uint16_t FileNum );
// 	uint32_t	DataValidMask;
};

struct	uFile_R24	//	�վ���ʱ��ʹ�õļ�¼�ṹ��ȫ��ͬ�����ֱ��壬���ڽ������ͼ��
{
	//	�ļ���Ϣ �� ��������
	uint32_t	sample_begin;		//	��¼������ʼʱ��
	uint16_t	set_time[2];		//	���ò���ʱ��
	uint16_t	set_flow[2];		//	���ò�������
	uint8_t		set_loops; 			//	���ò�������
	uint8_t		run_loops;			//	��¼��������
	uint8_t  _r[2];
	//	���������е�ͳ�Ʋ���
	uint16_t	sum_min[2];			//	ʵ�ʲ���ʱ�䣬ÿ����һ�ƣ����24Сʱ
	FP32		sum_Ba;				//	����ѹ��
	FP32		sum_tr[2];			//	��ǰ�¶�
	FP32		sum_pr[2];			//	��ǰѹ��
	FP32		max_pr[2];			//	��ǰѹ�����ֵ
	//	�������ۼ�ֵ
	FP32		vnd[2];				//	�ۼƵĲ��������ֻ��¼������
	FP32		vd[2];				//	2015��7��23�� �ۼƱ�����

//	public:
//		void	Save( uint16_t FileNum );
//		void	Load( uint16_t FileNum );
// 	uint32_t	DataValidMask;
};

struct	uFile_AIR
{
	
	uint32_t	sample_begin;		//	��¼������ʼʱ��
	uint16_t	set_time;			//	���ò���ʱ��
	uint16_t	set_flow[2];			//	���ò�������
	uint8_t		set_loops; 			//	���ò�������
	uint8_t		run_loops;			//	��¼��������
	//	���������е�ͳ�Ʋ���	
	uint16_t	sum_min;			//	ʵ�ʲ���ʱ�䣬ÿ����һ�ƣ����24Сʱ
// 	FP32		vnd[2];				//	�ۼƵĲ��������ֻ��¼������
//	FP32		sum_Ba;				//	����ѹ��

//	public:
//		void	Save( uint16_t FileNum );
//		void	Load( uint16_t FileNum );
// 	uint32_t	DataValidMask;
};


extern	void	File_Save_TSP( uint16_t FileNum, struct uFile_TSP const * pFile );
extern	void	File_Load_TSP( uint16_t FileNum, struct uFile_TSP       * pFile );
extern	void	File_Save_R24( uint16_t FileNum, struct uFile_R24 const * pFile );
extern	void	File_Load_R24( uint16_t FileNum, struct uFile_R24       * pFile );
extern	void	File_Save_SHI( uint16_t FileNum, struct uFile_SHI const * pFile );
extern	void	File_Load_SHI( uint16_t FileNum, struct uFile_SHI       * pFile );
extern	void	File_Save_AIR( uint16_t FileNum, struct uFile_AIR const * pFile );
extern	void	File_Load_AIR( uint16_t FileNum, struct uFile_AIR       * pFile );

/********  (C) COPYRIGHT 2014 �ൺ���˴���ӿƼ����޹�˾  **** End Of File ****/