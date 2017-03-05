/**
  ******************************************************************************
  * @file   fatfs.c
  * @brief  Code for fatfs applications
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

#include "fatfs.h"

uint8_t retUSER;    /* Return value for USER */
char USER_Path[4];  /* USER logical drive path */

/* USER CODE BEGIN Variables */

FATFS FatFs; 		/* File system object for User logical drive */
FIL   Fil; 		/* File object */
uint32_t wbytes; 	/* File write counts */
uint8_t wtext[] = "text to write logical disk by elgarbe"; /* File write buffer */
extern uint8_t rxBuff[512];
extern void Error_Handler(void);

/* USER CODE END Variables */    

void MX_FATFS_Init(void) 
{
		  /*## FatFS: Link the USER driver ###########################*/
		  retUSER = FATFS_LinkDriver(&USER_Driver, USER_Path);

  /* USER CODE BEGIN Init */
	uint16_t lastLog=0;
	if(retUSER == 0)
	{
		if(f_mount(&FatFs, (TCHAR const*)USER_Path, 0) == FR_OK)
		{
			// Busco el último LOG en la SD y creo el archivo para loguear
			lastLog = fs_get_last_log("/");
			fs_CreateLOG(lastLog + 1);
		}else{
			Error_Handler();
		}
	}else{
		Error_Handler();
	}
  /* USER CODE END Init */
}

/**
  * @brief  Gets Time from RTC 
  * @param  None
  * @retval Time in DWORD
  */
DWORD get_fattime(void)
{
  /* USER CODE BEGIN get_fattime */
  return 0;
  /* USER CODE END get_fattime */  
}

/* USER CODE BEGIN Application */
FRESULT fs_CreateLOG(uint16_t numLOG)
{
	FRESULT rc;				/* Result code */
	uint8_t NomLOG[11]="LOGxxxx.TXT";
	NomLOG[3] = numLOG/1000 + 48;
	numLOG -= (numLOG/1000)*1000;
	NomLOG[4] = numLOG/100 + 48;
	numLOG -= (numLOG/100)*100;
	NomLOG[5] = numLOG/10 + 48;
	numLOG -= (numLOG/10)*10;
	NomLOG[6] = numLOG/1 + 48;
	rc = f_open(&Fil, (const char *)NomLOG, FA_WRITE | FA_CREATE_ALWAYS);
	if(rc == FR_OK)
	{
		f_sync(&Fil);
	}
	return rc;
}

FRESULT fs_WriteFile(uint8_t f_TipoEscritura)
{
	FRESULT rc;				// Result code
	UINT bw;				// bytes escritos
	uint16_t count;

	// Verifico si debo escrivir SD_WR_BUFF_SIZE
	if(f_TipoEscritura == 1)
	{
		// Intento escrivir los datos que hay en sd_write_buf. bw guarda la cantidad de bytes escritos
		rc = f_write(&Fil, rxBuff, SD_WR_BUFF_SIZE, &bw);
	}else // o si son los ultimos bytes que había en el buffer
	{
		// Intento escrivir los datos que hay en sd_write_buf. bw guarda la cantidad de bytes escritos
		rc = f_write(&Fil, &rxBuff[512], SD_WR_BUFF_SIZE, &bw);
	}

	// Verifico que se hallan escrito tantos bytes como WR_BUFF_SIZE.
	// No sé si es muy útil esta verificacion. Quizás sirva para detectar la última escritura
	// que puede no ser de WR_BUFF_SIZE bytes y cerrar el archivo. Pero si es justo de WR_BUFF_SIZE bytes?
	if(bw == SD_WR_BUFF_SIZE)
	{
		// Me aseguro de que se almacene en la SD todos los datos.
		rc = f_sync(&Fil);
		return rc;
	}else
	{
		// escribí menos bytes que WR_BUFF_SIZE. Deve ser que son los ultimos bytes que había en el RB
//		rc = f_close(&Fil);
		return 0;
	}
}

/* Fusco el mayor de los LOGxxxx.TXT */
uint16_t fs_get_last_log (char* path)
{
    FRESULT res;
    FILINFO fno;
    DIR dir;

    uint16_t Fileint;		// El número del archivo en formato numerico
    uint16_t Filemax=0;

    res = f_opendir(&dir, path);						/* Open the directory */
    if (res == FR_OK)
    {
        for (;;)
        {
            res = f_readdir(&dir, &fno);                   /* Read a directory item */
            if (res != FR_OK || fno.fname[0] == 0) break;  /* Break on error or end of dir */
            if (fno.fname[0] == '.') continue;             /* Ignore dot entry */
            if (fno.fattrib & AM_ARC) 						/* It is a file. */
            {
            	// Verifico que sea un LOGxxxx.TXT
                if(fno.fname[0]=='L' && fno.fname[1]=='O' && fno.fname[2]=='G' &&
                	fno.fname[7]=='.' && fno.fname[8]=='T' && fno.fname[9]=='X' && fno.fname[10]=='T')
                {
					Fileint = (fno.fname[6]-48)     + (fno.fname[5]-48)*10 +
							  (fno.fname[4]-48)*100 + (fno.fname[3]-48)*1000;
					if(Fileint>Filemax)
						Filemax=Fileint;
                }
            }
        }
    }
    return Filemax;
}
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
