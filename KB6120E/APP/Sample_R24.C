void  _task_Sample_R24( void )
{
	const enum enumSamplerSelect	SamplerSelect  = Q_R24;
	const enum enumPumpSelect		PumpSelect_1 = PP_R24_A;
	const enum enumPumpSelect		PumpSelect_2 = PP_R24_B;

	BOOL	PumpState_1, PumpState_2;
	
	uint32_t	tt, now_minute = get_Now() / 60u;

	struct	uFile_R24	File;
	uint16_t	fname;	//	�����ļ����
	uint32_t	start;	//	������ʼʱ��
	uint8_t		iloop;	//	��ǰ��������

	PumpState_1 = FALSE;
	PumpState_2 = FALSE;
	Q_Pump[PumpSelect_1].xp_state = FALSE;
	Q_Pump[PumpSelect_2].xp_state = FALSE;

	start = SampleSet[SamplerSelect].start;
	iloop = SampleSet[SamplerSelect].iloop;

	if ( 0u != start )
	{	//	��ʼ����������
		if ( 0u == iloop )
		{	//	�����һ�β����Ŀ�ʼʱ��
			switch ( Configure.Mothed_Delay )
			{
			case enumByDelay:		//	��ʱ����
				start = start + SampleSet[SamplerSelect].delay1;
				break;
			default:
			case enumByAccurate:	//	��ʱ����
				{
					uint16_t	start_hour_minute = start % 1440u;
					
					if ( start_hour_minute > SampleSet[SamplerSelect].delay1 )
					{
						start += 1440u;	//	�Ƴٵ���������ʱ�����
					}
					start = start - start_hour_minute + SampleSet[SamplerSelect].delay1;
				}
				break;
			}
			iloop = 1u;
		}
		//	����ѭ��
		while (( ! SampleSwitch[SamplerSelect].Clean ) && ( iloop <= SampleSet[SamplerSelect].set_loops ))
		{
			//	���浱ǰ��������
			SampleSet[SamplerSelect].start = start;
			SampleSet[SamplerSelect].iloop = iloop;
			SampleSetSave();

			Q_Sampler[SamplerSelect].loops = iloop;									//	����״̬: �����������

			//	����ִ��ǰ������ʱ�ȴ�
			while (( ! SampleSwitch[SamplerSelect].Clean ) && ( now_minute < start ))
			{
				Q_Sampler[SamplerSelect].timer = (uint16_t)( start - now_minute );	//	����״̬: ʣ����ʱʱ��
				
				//	����ⲿ���ƶ���
				if ( SampleSwitch[SamplerSelect].Pause )
				{
					PumpState_1 = PumpState_2 = FALSE;
					Q_Sampler[SamplerSelect].state = state_PAUSE;					//	����״̬: �û�������ͣ
				}
				else
				{
					PumpState_1 = PumpState_2 = FALSE;
					Q_Sampler[SamplerSelect].state = state_SUSPEND;					//	����״̬: ��ʱ(����)
				}
				//	ִ�е����������ע������ظ����ص��
				if ( PumpState_1 != Q_Pump[PumpSelect_1].xp_state )
				{
					Pump_OutCmd( PumpSelect_1, PumpState_1 );
					Q_Pump[PumpSelect_1].xp_state = PumpState_1;
				}
				if ( PumpState_2 != Q_Pump[PumpSelect_2].xp_state )
				{
					Pump_OutCmd( PumpSelect_2, PumpState_2 );
					Q_Pump[PumpSelect_2].xp_state = PumpState_2;
				}
				delay( 1000u );
				
				now_minute = get_Now() / 60u;

				//	��ʾ��ѯ����
				Q_Pump[PumpSelect_1].sum_time = 0u;
				Q_Pump[PumpSelect_1].vnd      = 0.0f;
				Q_Pump[PumpSelect_2].sum_time = 0u;
				Q_Pump[PumpSelect_2].vnd      = 0.0f;
			}

			//	����ִ��ǰ��ȷ��Ҫ������ļ�(������Ҫ��ʼ����Ҳ������Ҫ�ָ�)
			fname = SampleSet[SamplerSelect].FileNum;		//	ȡ��󱣴��ļ���
			if ( ++fname > FileNum_Max ){  fname = 1u; }	//	����Ҫʹ���ļ���
			File_Load_R24 ( fname, &File );
			if ( 0u == File.sample_begin )
			{	//	���ͳ�����ݣ���ҪĿ�����ڿ�ʼ����ǰ��ʾ��ʱ����ʾ��
				File.sum_min [Q_PP1] = 0u;
				File.max_pr  [Q_PP1] = 0.0f;
				File.sum_tr  [Q_PP1] = 0.0f;
				File.sum_pr  [Q_PP1] = 0.0f;
				File.vnd     [Q_PP1] = 0.0f;
				File.sum_Ba           = 0.0f;
				File.sum_min [Q_PP2] = 0u;
				File.max_pr  [Q_PP2] = 0.0f;
				File.sum_tr  [Q_PP2] = 0.0f;
				File.sum_pr  [Q_PP2] = 0.0f;
				File.vnd     [Q_PP2] = 0.0f;
			}

			//	1.������ִ�в���
			while ( ! SampleSwitch[SamplerSelect].Clean ) // && �жϲ���ʱ�䵽
			{
				//	����״̬: ʣ�����ʱ��
				switch ( Configure.Mothed_Sample )
				{
				default:
				case enumBySet:	//	�۳����磬������ʱ������
					if ( now_minute < ( start + SampleSet[SamplerSelect].sample_1 ))
					{
						Q_Sampler[SamplerSelect].timer = (uint16_t)(( start + SampleSet[SamplerSelect]. sample_1 ) - now_minute );	//	ʣ������ʱ�䣨����ʱ��
					}
					else
					{
						Q_Sampler[SamplerSelect].timer = 0u;
					}
					break;
				case enumBySum:	//	���۵��磬���ۼ�ʱ������(˫·,A�ɴ���B�ɣ���A·���м���)
					if ( File.sum_min[Q_PP1] < SampleSet[SamplerSelect].sample_1 ) 
					{
						Q_Sampler[SamplerSelect].timer = SampleSet[SamplerSelect]. sample_1 - File.sum_min[Q_PP1];		//	ʣ������ʱ�䣨����ʱ��
					}
					else
					{
						Q_Sampler[SamplerSelect].timer = 0u;
					}
					break;
				}

				if ( Q_Sampler[SamplerSelect].timer <= 0 ){  break; }	//	�жϲ���ʱ�䵽

				//	����ⲿ���ƶ���
				if ( SampleSwitch[SamplerSelect].Fatal )
				{
					PumpState_1 = PumpState_2 = FALSE;
					Q_Sampler[SamplerSelect].state = state_ERROR;					//	����״̬: ����������ͣ
				}
				else if ( SampleSwitch[SamplerSelect].Pause )
				{
					PumpState_1 = PumpState_2 = FALSE;
					Q_Sampler[SamplerSelect].state = state_PAUSE;					//	����״̬: �û�������ͣ
				}
				else
				{
					PumpState_1 = TRUE;
					//	PumpState_2 = ?;
					switch ( Configure.Mothed_Sample )
					{
					default:
					case enumBySet:	//	�۳����磬������ʱ������
						PumpState_2 = ( now_minute < ( start + SampleSet[SamplerSelect].sample_2 ));
						break;
					case enumBySum:	//	���۵��磬���ۼ�ʱ������(˫·,A�ɴ���B�ɣ���A·���м���)
						PumpState_2 =  ( File.sum_min[Q_PP2] < SampleSet[SamplerSelect].sample_2 );
						break;
					}
					Q_Sampler[SamplerSelect].state = state_SAMPLE;				//	����״̬: ��������
				}
				//	ִ�е����������ע������ظ����ص��
				if ( PumpState_1 != Q_Pump[PumpSelect_1].xp_state )
				{
					Pump_OutCmd( PumpSelect_1, PumpState_1 );
					Q_Pump[PumpSelect_1].xp_state = PumpState_1;
				}
				if ( PumpState_2 != Q_Pump[PumpSelect_2].xp_state )
				{
					Pump_OutCmd( PumpSelect_2, PumpState_2 );
					Q_Pump[PumpSelect_2].xp_state = PumpState_2;
				}
				delay( 1000u );

				tt = get_Now() / 60u;
				if ( now_minute != tt )	// 1���ӵ���
				{
					if ( state_SAMPLE == Q_Sampler[SamplerSelect].state )
					{	//	ͳ��ǰ��ʼ��
						if ( 0u == File.sample_begin )
						{	//	���ļ�����ʼ��
							File.set_loops = SampleSet[SamplerSelect].set_loops;
							File.run_loops = iloop;
							File.set_time[Q_PP1] = SampleSet[SamplerSelect].sample_1;
							File.set_time[Q_PP2] = SampleSet[SamplerSelect].sample_2;
							File.set_flow[Q_PP1] = Configure.SetFlow[PumpSelect_1];
							File.set_flow[Q_PP2] = Configure.SetFlow[PumpSelect_2];
							File.sample_begin = now_minute * 60u + 1u;	//	��ʵ�ʿ�ʼ����ǰ��¼��ʼʱ��
						}
						//	ͳ����������
						if ( PumpState_1 )
						{
							FP32	Ba = get_Ba();
							FP32	Tr = get_Tr( PumpSelect_1 );
							FP32	Pr = get_Pr( PumpSelect_1 );

							if ( fabs( File.max_pr[Q_PP1] ) < fabs( Pr ))
							{
								File.max_pr[Q_PP1] = Pr;
							}
							File.sum_tr  [Q_PP1] += Tr;
							File.sum_pr  [Q_PP1] += Pr;
							File.vnd     [Q_PP1] += get_fstd( PumpSelect_1 );

							File.sum_Ba  += Ba;
							File.sum_min [Q_PP1] += 1u;
						}

						if ( PumpState_2 )
						{
							FP32	Tr = get_Tr( PumpSelect_2 );
							FP32	Pr = get_Pr( PumpSelect_2 );

							if ( fabs( File.max_pr[Q_PP2] ) < fabs( Pr ))
							{
								File.max_pr[Q_PP2] = Pr;
							}
							File.sum_tr  [Q_PP2] += Tr;
							File.sum_pr  [Q_PP2] += Pr;
							File.vnd     [Q_PP2] += get_fstd( PumpSelect_2 );
							File.sum_min [Q_PP2] += 1u;
						}

						File_Save_R24 ( fname, &File );
					}
					now_minute = tt;
				}

				//	��ʾ��ѯ����
				Q_Pump[PumpSelect_1].sum_time = File.sum_min [Q_PP1];
				Q_Pump[PumpSelect_1].vnd      = File.vnd     [Q_PP1];
 
				Q_Pump[PumpSelect_2].sum_time = File.sum_min [Q_PP2];
				Q_Pump[PumpSelect_2].vnd      = File.vnd     [Q_PP2];
 
			}

			//	���һ�β��������ļ����л���
			if (( 0u != File.sum_min[Q_PP1] ) || ( 0u != File.sum_min[Q_PP2] ))
			{	//	����ͳ�����ݷǿ�ʱִ��
				SampleSet[SamplerSelect].FileNum = fname;
				SampleSetSave();
				if ( ++fname > FileNum_Max ){  fname = 1u; }
				//	��¼���ۼ�����
				PumpSumTimeSave( PumpSelect_1, PumpSumTimeLoad( PumpSelect_1 ) + File.sum_min[Q_PP1] );
				PumpSumTimeSave( PumpSelect_2, PumpSumTimeLoad( PumpSelect_2 ) + File.sum_min[Q_PP2] );
			}
			//	���ͳ�����ݷǿգ���յ������ļ������򣬼���ʹ�õ�ǰ�ļ�
			File.sample_begin = 0u;
			File_Save_R24 ( fname, &File );			

			//	�����´β����� ��������ʼʱ��
			switch ( Configure.Mothed_Sample )
			{
			default:
			case enumBySet:	//	���ݲ���ʱ���������У��۳����磬������ʱ������
				start = ( start + SampleSet[SamplerSelect].sample_1 );
				break;	
			case enumBySum:	//	�����ۼ�ʱ���������У����۵��磬�ӵ�ǰʱ����ʱ
				start = now_minute;
				break;
			}
			start += SampleSet[SamplerSelect].suspend_1;
			iloop ++;
		}
		//	�����Ѿ���ɣ�ǿ��ִ��һ�ιرö��������Լ�¼�ı�״̬��
			Pump_OutCmd( PumpSelect_1, FALSE );	Q_Pump[PumpSelect_1].xp_state = FALSE;
			Pump_OutCmd( PumpSelect_2, FALSE );	Q_Pump[PumpSelect_2].xp_state = FALSE;
				
	
		//	ɾ����������
		SampleSet[SamplerSelect].start = 0u;
		SampleSet[SamplerSelect].iloop = 0u;
		SampleSetSave();
	}
	//	����ȫ�����
	Q_Sampler[SamplerSelect].state	= state_FINISH;
	Q_Sampler[SamplerSelect].loops	= 0u;
	Q_Sampler[SamplerSelect].timer	= 0u;

	Q_Pump[PumpSelect_1].sum_time = 0u;
	Q_Pump[PumpSelect_1].vnd	  = 0.0f;
 
	Q_Pump[PumpSelect_2].sum_time = 0u;
	Q_Pump[PumpSelect_2].vnd	  = 0.0f;
 

	//  osThreadTerminate( osThreadGetId());
}
/**/