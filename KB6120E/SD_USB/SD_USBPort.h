#ifndef __SDCARD_PORT_H
#define __SDCARD_PORT_H

#include <stdio.h>
#include <string.h>
#include "Pin.H"
#include "BSP.H"
#include "time.h"
#include "file_sys.h"
#include "CH376INC.h"

//��Ŀ¼����
void Byte_CREAT_CON_DIR ( const char * BUF_Name );
void Byte_CREAT_WRITE_DIR ( const char * BUF_Name, uint8_t * Content, uint32_t offset );
void Byte_OPEN_READ_DIR ( const char * BUF_Name , uint32_t Addr, uint8_t * bufread, uint32_t offset );
void Byte_OPEN_WRITE_DIR ( const char * BUF_Name , uint8_t * Content , uint32_t Addr, uint32_t offset );
//���Ŀ¼����
void Byte_CREAT_WRITE_PATH ( const char * BUF_Name , uint8_t * Content ,uint32_t offset );
uint8_t Byte_OPEN_READ_PATH ( const char * BUF_Name , uint32_t Addr, uint8_t* bufread, uint32_t offset );
uint8_t Byte_OPEN_WRITE_PATH ( const char * BUF_Name ,   uint8_t * Content , uint32_t Addr, uint32_t offset );
//SD����ʼ��
void SD_Init(void);
void	SD_File_Creat(void);
//SD�ӿ�
BOOL ByteLoad( const char * B_Name , uint32_t Addr, uint8_t * bufread, uint32_t offset  );
BOOL ByteSave( const char * BUF_Name ,  uint8_t * Content , uint32_t Addr,uint32_t offset );
BOOL ByteFill( const char * BUF_Name , uint32_t Addr , uint16_t size);
//		�ļ����� 			д��ʼ��ַ					Ҫд������
uint32_t ByteGetSize(const char * BUF_Name);

//��Ŀ¼����
void Sec_CREAT_CON_DIR ( const char * BUF_Name );
void Sec_CREAT_WRITE_DIR ( const char * BUF_Name, uint8_t * Content, uint8_t offset );
void Sec_OPEN_READ_DIR ( const char * BUF_Name , uint32_t Addr, uint8_t * bufread, uint8_t offset );
void Sec_OPEN_WRITE_DIR ( const char * BUF_Name , uint8_t * Content , uint32_t Addr, uint8_t offset );
//���Ŀ¼����
void Sec_CREAT_WRITE_PATH ( const char * BUF_Name , uint8_t * Content ,uint8_t offset );
uint8_t Sec_OPEN_READ_PATH ( const char * BUF_Name , uint32_t Addr, uint8_t* bufread, uint8_t offset );
uint8_t Sec_OPEN_WRITE_PATH ( const char * BUF_Name ,   uint8_t * Content , uint32_t Addr, uint8_t offset );
//Sec����ʼ��
uint8_t USB_Init(void);
void		USB_File_Creat(void);
uint8_t USB_CHAK(void);
//void USBInit_Test(void);
//USB�ӿ�
BOOL SecLoad( const char * B_Name , uint32_t Addr, uint8_t * bufread, uint8_t offset  );
BOOL SecSave( const char * BUF_Name ,  uint8_t * Content , uint32_t Addr,uint8_t offset );
BOOL SecFill( const char * BUF_Name , uint32_t Addr , uint8_t size);
//		�ļ����� 			д��ʼ��ַ					Ҫд������
uint32_t SecGetSize(const char * BUF_Name);
#endif
/**/
