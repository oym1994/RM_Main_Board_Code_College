/*
*********************************************************************************************************
*                                     DJI BOARD SUPPORT PACKAGE
*
*                                   (c) Copyright 2015; Dji, Inc.
*
*               All rights reserved.  Protected by international copyright laws.
*               Knowledge of the source code may NOT be used to develop a similar product.
*               Please help us continue to provide the Embedded community with the finest
*               software available.  Your honesty is greatly appreciated.
*********************************************************************************************************
*/

#include "mytype.h"
#include "bsp_flash.h"
#include "string.h"
#include "error_task.h"
#include "cmsis_os.h"


static FLASH_EraseInitTypeDef EraseInitStruct;
u32			SectorError;
/**
  * @brief  Gets the sector of a given address
  * @param  None
  * @retval The sector of a given address
  */
static uint32_t GetSector(uint32_t Address)
{
  uint32_t sector = 0;

  if((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0))
  {
    sector = FLASH_SECTOR_0;
  }
  else if((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1))
  {
    sector = FLASH_SECTOR_1;
  }
  else if((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2))
  {
    sector = FLASH_SECTOR_2;
  }
  else if((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3))
  {
    sector = FLASH_SECTOR_3;
  }
  else if((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4))
  {
    sector = FLASH_SECTOR_4;
  }
  else if((Address < ADDR_FLASH_SECTOR_6) && (Address >= ADDR_FLASH_SECTOR_5))
  {
    sector = FLASH_SECTOR_5;
  }
  else if((Address < ADDR_FLASH_SECTOR_7) && (Address >= ADDR_FLASH_SECTOR_6))
  {
    sector = FLASH_SECTOR_6;
  }
  else if((Address < ADDR_FLASH_SECTOR_8) && (Address >= ADDR_FLASH_SECTOR_7))
  {
    sector = FLASH_SECTOR_7;
  }
  else if((Address < ADDR_FLASH_SECTOR_9) && (Address >= ADDR_FLASH_SECTOR_8))
  {
    sector = FLASH_SECTOR_8;
  }
  else if((Address < ADDR_FLASH_SECTOR_10) && (Address >= ADDR_FLASH_SECTOR_9))
  {
    sector = FLASH_SECTOR_9;
  }
  else if((Address < ADDR_FLASH_SECTOR_11) && (Address >= ADDR_FLASH_SECTOR_10))
  {
    sector = FLASH_SECTOR_10;
  }
  else if((Address < ADDR_FLASH_SECTOR_12) && (Address >= ADDR_FLASH_SECTOR_11))
  {
    sector = FLASH_SECTOR_11;
  }
  else if((Address < ADDR_FLASH_SECTOR_13) && (Address >= ADDR_FLASH_SECTOR_12))
  {
    sector = FLASH_SECTOR_12;
  }
  else if((Address < ADDR_FLASH_SECTOR_14) && (Address >= ADDR_FLASH_SECTOR_13))
  {
    sector = FLASH_SECTOR_13;
  }
  else if((Address < ADDR_FLASH_SECTOR_15) && (Address >= ADDR_FLASH_SECTOR_14))
  {
    sector = FLASH_SECTOR_14;
  }
  else if((Address < ADDR_FLASH_SECTOR_16) && (Address >= ADDR_FLASH_SECTOR_15))
  {
    sector = FLASH_SECTOR_15;
  }
  else if((Address < ADDR_FLASH_SECTOR_17) && (Address >= ADDR_FLASH_SECTOR_16))
  {
    sector = FLASH_SECTOR_16;
  }
  else if((Address < ADDR_FLASH_SECTOR_18) && (Address >= ADDR_FLASH_SECTOR_17))
  {
    sector = FLASH_SECTOR_17;
  }
  else if((Address < ADDR_FLASH_SECTOR_19) && (Address >= ADDR_FLASH_SECTOR_18))
  {
    sector = FLASH_SECTOR_18;
  }
  else if((Address < ADDR_FLASH_SECTOR_20) && (Address >= ADDR_FLASH_SECTOR_19))
  {
    sector = FLASH_SECTOR_19;
  }
  else if((Address < ADDR_FLASH_SECTOR_21) && (Address >= ADDR_FLASH_SECTOR_20))
  {
    sector = FLASH_SECTOR_20;
  }
  else if((Address < ADDR_FLASH_SECTOR_22) && (Address >= ADDR_FLASH_SECTOR_21))
  {
    sector = FLASH_SECTOR_21;
  }
  else if((Address < ADDR_FLASH_SECTOR_23) && (Address >= ADDR_FLASH_SECTOR_22))
  {
    sector = FLASH_SECTOR_22;
  }
  else /* (Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_23) */
  {
    sector = FLASH_SECTOR_23;
  }  
  return sector;
}


/**
  * @brief  write bytes to flash  
  *         start addr  = PARAM_SAVED_START_ADDRESS
  * @param  none
  * @retval flash write status
  */
u8 BSP_FLASH_Write(u8* pbuff, u32 len)
{
	/*erase flash before program */
	uint32_t WriteSector = GetSector(PARAM_SAVED_START_ADDRESS);
	u32				start_addr = PARAM_SAVED_START_ADDRESS;
	HAL_FLASH_Unlock();
	
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	EraseInitStruct.Sector = WriteSector;
	EraseInitStruct.NbSectors = 1;
	if (HAL_OK != HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) )
		while(1);
	
	/*start program flash*//*write data in here*/
	while (len -- ){
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, start_addr, *pbuff);
		start_addr++;
		pbuff++;
	}
	/*write data end*/
	HAL_FLASH_Lock(); 
	return 0;
}

/*when you read just use memcpy()....*/
u8 BSP_FLASH_Read(u8* pbuff, u32 len)
{
	/*erase flash before program */
//	uint32_t WriteSector = GetSector(PARAM_SAVED_START_ADDRESS);
//	u32				start_addr = PARAM_SAVED_START_ADDRESS;
//	HAL_FLASH_Unlock();
//	
//	EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
//  EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
//  EraseInitStruct.Sector = WriteSector;
//  EraseInitStruct.NbSectors = 1;
//	if (HAL_OK != HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) )
//		while(1);
//	
//	/*start program flash*//*write data in here*/
//	while (len -- ){
//		HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, start_addr, *pbuff);
//		start_addr++;
//		pbuff++;
//	}
//	/*write data end*/
//	HAL_FLASH_Lock(); 
	return 0;
}
