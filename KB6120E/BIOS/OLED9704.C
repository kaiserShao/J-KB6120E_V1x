/**************** (C) COPYRIGHT 2012 �ൺ���˴���ӿƼ����޹�˾ ****************
* �� �� ��: OLED9704.C
* �� �� ��: Dean
* ��  ��  : �n��OLED9704ģ����ʳ���
*         : 
* ����޸�: 2012��4��5��
*********************************** �޶���¼ ***********************************
* ��  ��: 
* �޶���: 
*******************************************************************************/
#include "BSP.H"
#include "BIOS.H"


//	BIOS.C - OLED9704 access
extern	void	  OLED9704_PortInit( void );
extern	uint8_t	OLED9704_ReadState( void  );
extern	uint8_t	OLED9704_ReadData( void  );
extern	void	  OLED9704_WriteReg( uint8_t OutCommand );
extern	void	  OLED9704_WriteData( uint8_t OutData );
extern	void	  OLED9704_DisplayEnable( void );
extern	void	  OLED9704_DisplayDisable( void );

///////////////////////////////////////////////////////////////////
// д����Һ���ӳ� Һ������Ϊ(4�� X 8��)���֣�ȫ��ʹ��ģ��ӿڷ�ʽ
///////////////////////////////////////////////////////////////////
#define  text_width		8U		// ��Ļ������8��Ϊ��λ���м���
#define  max_txt_col	16U		// ��
#define  max_txt_row	8U		// ��(ҳ)

///////////////////////////////////////////////////////////////////
// �����趨���������ݣ���λ��һ��������Ԫλ��
///////////////////////////////////////////////////////////////////
static  void  move_to( uint8_t yy, uint16_t xx )
{
	xx += 0x02U;					
	OLED9704_WriteReg( 0xB0U + ( yy % 0x08U ));			// Set page
	OLED9704_WriteReg( 0x10U + ((uint8_t)xx / 0x10U ));	// Higher col address
	OLED9704_WriteReg( 0x00U + ((uint8_t)xx % 0x10U ));	// Lower col address
}

///////////////////////////////////////////////////////////////////
//	����
///////////////////////////////////////////////////////////////////
void  OLED9704_cls( void  )
{	// ����
	uint8_t	row, col;

	for( row = 0U; row < 8u; ++row )
	{
		move_to( row, 0U  );
		for( col = 128U; col != 0U; --col )
		{
			OLED9704_WriteData( 0xffu );
		}
	}

	OLED9704_WriteReg(0xAFU);   		// Display on
}

///////////////////////////////////////////////////////////////////
//	����
///////////////////////////////////////////////////////////////////
void	OLED9704_mask( uint16_t yx, uint8_t xlen )
{	//	����
	uint8_t		i, row, col, col_end;
	uint8_t		InData;

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
			move_to( row,      (uint8_t)( col * text_width ) + i );	InData = OLED9704_ReadData();
			move_to( row,      (uint8_t)( col * text_width ) + i );	OLED9704_WriteData((uint8_t)(~InData));
			move_to( row + 1U, (uint8_t)( col * text_width ) + i );	InData = OLED9704_ReadData();
			move_to( row + 1U, (uint8_t)( col * text_width ) + i );	OLED9704_WriteData((uint8_t)(~InData));
		}
	} while ( ++col < col_end );
}

///////////////////////////////////////////////////////////////////
//	����ַ���
///////////////////////////////////////////////////////////////////
// 	static	__inline	uint8_t	invert_byte( uint8_t U8B ){
// 		U8B = (uint8_t)((uint8_t)( U8B << 4 ) & 0x0F0U ) | (uint8_t)((uint8_t)( U8B >> 4 ) & 0x00FU );
// 		U8B = (uint8_t)((uint8_t)( U8B << 2 ) & 0x0CCU ) | (uint8_t)((uint8_t)( U8B >> 2 ) & 0x033U );
// 		U8B = (uint8_t)((uint8_t)( U8B << 1 ) & 0x0AAU ) | (uint8_t)((uint8_t)( U8B >> 1 ) & 0x055U );

// 		return	U8B;
// 	}

void	OLED9704_puts( uint16_t yx, const CHAR * sz )
{	//	����ַ���
	CGROM		pDot;
	CHAR		sDat;
	size_t		slen;
	uint8_t		i, row, col, col_end;

	assert( sz != NULL );
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
			move_to( row, (uint8_t)( col * text_width ) );
			for( i = 8U; i != 0U; --i )	{	OLED9704_WriteData(*pDot++);	}
			move_to( row + 1U, (uint8_t)( col * text_width ) );
			for( i = 8U; i != 0U; --i )	{	OLED9704_WriteData(*pDot++);	}
			col += 1U;
		}
		else
		{	// SBC ȫ����
			pDot = DotSeekSBC( sDat, *sz++ );
			move_to( row, (uint8_t)( col * text_width ) );
			for( i = 16U; i != 0U; --i ) {	OLED9704_WriteData(*pDot++);	}
			move_to( row + 1U, (uint8_t)( col * text_width ) );
			for( i = 16U; i != 0U; --i ) {	OLED9704_WriteData(*pDot++);	}
			col += 2U;
		}
	}	while ( col < col_end  );
}


///////////////////////////////////////////////////////////////////
// ������ʾ����
///////////////////////////////////////////////////////////////////
void	OLED9704_SetLight( uint8_t SetLight )
{
	uint8_t OutValue = (uint16_t)SetLight * 255u / 100u;
  OLED9704_WriteReg(0x81u);
	OLED9704_WriteReg( OutValue );    // Contrast setting: contrast
}

///////////////////////////////////////////////////////////////////
// Һ����ʼ��
///////////////////////////////////////////////////////////////////
void	OLED9704_Init( void )
{
	//  ��ʾģ���ʼ��
	OLED9704_PortInit();
	OLED9704_WriteReg(0xAEU);   		// Display off
	OLED9704_WriteReg(0xADU);  OLED9704_WriteReg(0x8AU);    // Internal DC-DC off: Second byte
	OLED9704_WriteReg(0xA8U);  OLED9704_WriteReg(0x3FU);    // MUX Ratio: 64 duty
	OLED9704_WriteReg(0xD3U);  OLED9704_WriteReg(0x00U);    // Display offset: Second byte
	OLED9704_WriteReg(0x40U);   		// Start line
//	OLED9704_WriteReg(0xA0U);   		// Set Segment remap (0xA0:normal, 0xA1:remapped)//�������Ҫ�ĵĵط�a0.����	a1.����
	OLED9704_WriteReg(0xA1U);
	OLED9704_WriteReg(0xC8U);   		// Set COM remap (0xC0:normal, 0xC8: enable )
	OLED9704_WriteReg(0xA6U);   		// Set normal/inverse display (0xA6:Normal display)
	OLED9704_WriteReg(0xA4U);   		// Set entire display on/off (0xA4:Normal display)
//    OLED9704_WriteReg(0x81U);  OLED9704_WriteReg(0x5CU);    // Contrast setting: Second byte
	OLED9704_WriteReg(0xD5U);  OLED9704_WriteReg(0x60U);    // Frame rate: 85 Hz
	OLED9704_WriteReg(0xD8U);  OLED9704_WriteReg(0x00U);    // Mode setting: Mono mode
	OLED9704_WriteReg(0xD9U);  OLED9704_WriteReg(0x84U);    // Set Pre-char_tge period: Second byte
	OLED9704_cls();

	OLED9704_DisplayEnable();
}

/********  (C) COPYRIGHT 2012 �ൺ���˴���ӿƼ����޹�˾  **** End Of File ****/