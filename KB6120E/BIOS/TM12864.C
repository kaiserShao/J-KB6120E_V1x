/**************** (C) COPYRIGHT 2012 �ൺ���˴���ӿƼ����޹�˾ ****************
* �� �� ��: TM12864.C
* �� �� ��: Dean
* ��  ��  : ����128*64Һ����д����
*         : 
* ����޸�: 2012��4��3��
*********************************** �޶���¼ ***********************************
* ��  ��: 
* �޶���: 
*******************************************************************************/
#include "BSP.H"
#include "BIOS.H"

static	uint8_t	invert_byte( uint8_t U8B )
{
	U8B = (( U8B & 0x0Fu ) << 4 ) | (( U8B & 0xF0u ) >> 4 );
	U8B = (( U8B & 0x33u ) << 2 ) | (( U8B & 0xCCu ) >> 2 );
	U8B = (( U8B & 0x55u ) << 1 ) | (( U8B & 0xAAu ) >> 1 );

	return	U8B;
}

//	BIOS.C -  TM12864 access
extern	void	  TM12864_PortInit( void );
extern	void	  TM12864_SelectL( void );
extern	void	  TM12864_SelectR( void );
extern	uint8_t	TM12864_ReadState( void );
extern	uint8_t	TM12864_ReadData( void );
extern	void	  TM12864_WriteCommand( uint8_t OutCommand );
extern	void	  TM12864_WriteData( uint8_t	OutData );

extern	void	  TM12864_GrayInit( void );

///////////////////////////////////////////////////////////////////
#define	 DISPON 		  0x3FU	// ��ʾ on
#define	 DISPOFF		  0x3EU	// ��ʾ off
#define	 DISPFIRST		0xC0U	// ��ʾ��ʼ�ж���
#define	 SET_COL		  0x40U	// X��λ�趨ָ��У�
#define	 SET_PAG		  0xB8U	// Y��λ�趨ָ�ҳ��

///////////////////////////////////////////////////////////////////
// д����Һ���ӳ� Һ������Ϊ(4�� X 8��)���֣�ȫ��ʹ��ģ��ӿڷ�ʽ
///////////////////////////////////////////////////////////////////
#define  text_width		8U		// ��Ļ������8��Ϊ��λ���м���
#define  max_txt_row	8U		// ��(ҳ)
#define  max_txt_col	16U		// ��

///////////////////////////////////////////////////////////////////
// �����趨���������ݣ���λLCM�ϵ���һ��������Ԫλ��
///////////////////////////////////////////////////////////////////
static	void	moveto( uint8_t yy, uint8_t xx )
{
#if _Flip
	( xx < 64 ) ? TM12864_SelectL() : TM12864_SelectR();
	TM12864_WriteCommand(((yy) %  8U ) + SET_PAG );
	TM12864_WriteCommand(((xx) % 64U ) + SET_COL );
#else
	( xx < 64 ) ? TM12864_SelectR() : TM12864_SelectL();
	TM12864_WriteCommand(((~yy) %  8U ) + SET_PAG );
	TM12864_WriteCommand(((~xx) % 64U ) + SET_COL );
#endif
}

///////////////////////////////////////////////////////////////////
//	����
///////////////////////////////////////////////////////////////////
void	TM12864_cls( void )
{	// ����
	uint8_t	row, col;

	TM12864_SelectL(); TM12864_WriteCommand( DISPOFF );
	TM12864_SelectR(); TM12864_WriteCommand( DISPOFF );	// �ر���ʾ��

	TM12864_SelectL(); TM12864_WriteCommand(DISPFIRST);
	TM12864_SelectR(); TM12864_WriteCommand(DISPFIRST);	// ������ʾ��ʼ��Ϊ��

	// �����ʾ������ RAM
	for( row = 0U; row < 8u; ++row )
	{
		// Left  part
		moveto( row, 0U  );
		for( col = 64U; col != 0U; --col )
		{	TM12864_WriteData(0x00U);	}
		// Right part
		moveto( row, 64U );
		for( col = 64U; col != 0U; --col )
		{	TM12864_WriteData(0x00U);	}
	}

	TM12864_SelectL(); TM12864_WriteCommand( DISPON );
	TM12864_SelectR(); TM12864_WriteCommand( DISPON );	// ����ʾ��
}

///////////////////////////////////////////////////////////////////
//	����
///////////////////////////////////////////////////////////////////
void	TM12864_mask( uint16_t yx, uint8_t xlen )
{	//	����
	uint8_t 	i, row, col, col_end;
	uint8_t 	InDat;

	assert( 0U != xlen );

	row = ( yx ) / 256u;
	col = ( yx ) % 256u;
	assert( row < max_txt_row );
	assert( col < max_txt_col );

	col_end = col + xlen;
	if ( col_end > max_txt_col )
	{
		col_end = max_txt_col;
	}

	do {   
		for( i = 0u; i < 8u; ++i )
		{
			moveto( row,      (uint8_t)( col * text_width ) + i );	InDat = TM12864_ReadData();
			moveto( row,      (uint8_t)( col * text_width ) + i );	TM12864_WriteData((uint8_t) ~InDat );
		}
		for( i = 0u; i < 8u; ++i )
		{
			moveto( row + 1U, (uint8_t)( col * text_width ) + i );	InDat = TM12864_ReadData();
			moveto( row + 1U, (uint8_t)( col * text_width ) + i );	TM12864_WriteData((uint8_t) ~InDat );
		}
	} while ( ++col < col_end );
}

///////////////////////////////////////////////////////////////////
//	����ַ���
///////////////////////////////////////////////////////////////////
void	TM12864_puts( uint16_t yx, const CHAR * sz )
{	//	����ַ���
	CGROM		pDot;
	CHAR		sDat;
	size_t		slen;
	uint8_t		i, row, col, col_end;

	assert( sz );
	slen = strlen( sz );
	if ( slen > max_txt_col )
	{
		slen = max_txt_col;
	}

	row = ( yx ) / 256u;
	col = ( yx ) % 256u;
	assert( row < max_txt_row );
	assert( col < max_txt_col );

	col_end = col + (uint8_t)slen;
	if ( col_end > max_txt_col )
	{
		col_end = max_txt_col;
	}

	do {
		sDat = *sz++;

		if ( 0U == ((uint8_t)sDat & 0x80U ))
		{	// DBC �����
			pDot = DotSeekDBC( sDat );
			for ( i = 0; i < 8u; ++i ){	//	for( i = 8U; i != 0U; --i )	{
			moveto( row, (uint8_t)( col * text_width )+i);
			TM12864_WriteData(invert_byte(*pDot++));	}
			for ( i = 0; i < 8u; ++i ){	//	for( i = 8U; i != 0U; --i ){
			moveto( row + 1U, (uint8_t)( col * text_width )+i);
			TM12864_WriteData(invert_byte(*pDot++));	}
			col += 1U;
		}
		else
		{	// SBC ȫ����
			pDot = DotSeekSBC( sDat, *sz++ );
			for ( i = 0; i < 8u; ++i ){	//	for( i = 8U; i != 0U; --i ){
				moveto( row, (uint8_t)( col * text_width )+i);
			TM12864_WriteData(invert_byte(*pDot++));	}
			for ( i = 0; i < 8u; ++i ){	//	for( i = 8U; i != 0U; --i ){
				moveto( row, (uint8_t)(( col + 1U ) * text_width )+i);
			TM12864_WriteData(invert_byte(*pDot++));	}
			for ( i = 0; i < 8u; ++i ){	//	for( i = 8U; i != 0U; --i ){
				moveto( row + 1U, (uint8_t)( col  * text_width )+i);
			TM12864_WriteData(invert_byte(*pDot++));	}
			for ( i = 0; i < 8u; ++i ){	//	for( i = 8U; i != 0U; --i ){
				moveto( row + 1U, (uint8_t)(( col + 1U ) * text_width )+i);
			TM12864_WriteData(invert_byte(*pDot++));	}
			col += 2U;
		}
	}	while ( col < col_end  );
}

/********************************** ����˵�� ***********************************
 *	��ʾģ���ʼ��
*******************************************************************************/
void	TM12864_Init( void )
{  //  ��ʾģ���ʼ��
	TM12864_PortInit();

	TM12864_cls();                                 		// �����ʾ������ RAM

	TM12864_GrayInit();
}

/********************************** ����˵�� ***********************************
 *	������ʾģ��
*******************************************************************************/
BOOL	TM12864_Test( void )
{
	uint8_t state_off_L, state_on_L;
	uint8_t state_off_R, state_on_R;
	
	TM12864_PortInit();

	TM12864_cls();                                 		// �����ʾ������ RAM

	TM12864_SelectL(); TM12864_WriteCommand( DISPOFF );
	TM12864_SelectR(); TM12864_WriteCommand( DISPOFF );	// �ر���ʾ��
	TM12864_SelectL(); state_off_L = 0x20 & TM12864_ReadState();
	TM12864_SelectR(); state_off_R = 0x20 & TM12864_ReadState();
	TM12864_SelectL(); TM12864_WriteCommand( DISPON );
	TM12864_SelectR(); TM12864_WriteCommand( DISPON );	// ����ʾ��
	TM12864_SelectL(); state_on_L  = 0x20 & TM12864_ReadState();
	TM12864_SelectR(); state_on_R  = 0x20 & TM12864_ReadState();

	if (( state_off_L == 0x20u ) && ( state_on_L == 0x00 ) 
	 && ( state_off_R == 0x20u ) && ( state_on_R == 0x00 ))
	{
		return	TRUE;
	}
	else
	{
		return	FALSE;
	}
}

/********  (C) COPYRIGHT 2012 �ൺ���˴���ӿƼ����޹�˾  **** End Of File ****/