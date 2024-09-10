#include "stm32f10x.h"
void SPI_FLASH_Init(void);
void SPI_FLASH_SectorErase(u32 SectorAddr);
void SPI_FLASH_ChipErase(void);
void SPI_FLASH_PageWrite(u8* pBuffer, u32 WriteAddr, u16 NumByteToWrite);
void SPI_FLASH_BufferWrite(u8* pBuffer, u32 WriteAddr, u16 NumByteToWrite);
void SPI_FLASH_BufferRead(u8* pBuffer, u32 ReadAddr, u16 NumByteToRead);
u32 SPI_FLASH_ReadID(void);
u32 SPI_FLASH_ReadDeviceID(void);
void SPI_FLASH_StartReadSequence(u32 ReadAddr);
u8 SPI_FLASH_SendByte(u8 byte);
u16 SPI_FLASH_SendHalfWord(u16 HalfWord);
void SPI_FLASH_WriteEnable(void);
void SPI_FLASH_WaitForWriteEnd(void);
void SPI_Flash_PowerDown(void);
void SPI_Flash_WAKEUP(void);
u8 Buffercmp(u8* pBuffer1, u8* pBuffer2, u16 BufferLength);
void SPI_FLASH_Test(void);
uint8_t SPI_Flash_Init(void);
u8 SPIx_ReadWriteByte(u8 byte);
void W25X_Read_Sector(uint32_t nSector, u8* pBuffer);
void W25X_Write_Sector(uint32_t nSector, u8* pBuffer);
void SPI_Flash_Erase_Sector(u32 Dst_Addr);
