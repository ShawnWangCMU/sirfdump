/**
 * @addtogroup platform_src_sirf_pal_posix
 * @{
 */


/*
 *                   SiRF Technology, Inc. GPS Software
 *
 *    Copyright (c) 2005-2009 by SiRF Technology, Inc.  All rights reserved.
 *
 *    This Software is protected by United States copyright laws and
 *    international treaties.  You may not reverse engineer, decompile
 *    or disassemble this Software.
 *
 *    WARNING:
 *    This Software contains SiRF Technology Inc.�s confidential and
 *    proprietary information. UNAUTHORIZED COPYING, USE, DISTRIBUTION,
 *    PUBLICATION, TRANSFER, SALE, RENTAL OR DISCLOSURE IS PROHIBITED
 *    AND MAY RESULT IN SERIOUS LEGAL CONSEQUENCES.  Do not copy this
 *    Software without SiRF Technology, Inc.�s  express written
 *    permission.   Use of any portion of the contents of this Software
 *    is subject to and restricted by your signed written agreement with
 *    SiRF Technology, Inc.
 *
 */

/**
 * @file   sirf_pal_hw_reset.c
 *
 * @brief  Tracker reset control module.
 */

#include <stdio.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <errno.h>
#if defined(OS_LINUX) && defined(PHYTEC_LPC3180)
#include <linux/gpio.h>
#endif

#include "sirf_types.h"
#include "sirf_pal.h"

#ifdef OS_ANDROID
#define  LOG_TAG  "gps_sirf"
#include <cutils/log.h>
#endif

#if defined(OS_ANDROID) && defined(NEO_FREERUNNER)
#define RESET_GPIO "/sys/bus/platform/devices/neo1973-pm-gps.0/eint3_power_on"
#endif

#if defined(OS_LINUX) && defined(PHYTEC_LPC3250)
#define GPIO_RESET 1

 

struct gpio_control
{
   unsigned int pin;
   unsigned int arg;
};
#endif

/* ----------------------------------------------------------------------------
 *    Functions
 * ------------------------------------------------------------------------- */
/* A global handler for the reset port to control EXT_SRESET_N pin */
tSIRF_HANDLE ResetPortHandle = NULL;

/**
 * @brief Open and Init the reset port
 *
 * @param[in] level           Initial line value
 * @return                    SIRF_FAILURE or SIRF_SUCCESS.
 */
tSIRF_RESULT SIRF_PAL_HW_OpenRESET( tSIRF_UINT32 level )
{
   tSIRF_RESULT result = SIRF_FAILURE;

   ResetPortHandle = NULL;

   switch(g_userPalConfig.ext_sreset_n_line_usage)
   {
   case UI_CTRL_MODE_HW_RESET_UART_DTR:
      if( (SIRF_SUCCESS == SIRF_PAL_COM_UART_Create(&ResetPortHandle)) &&
          (SIRF_SUCCESS == SIRF_PAL_COM_UART_Open(ResetPortHandle, &g_userPalConfig.on_off_port[0])) )
      {
         if( SIRF_SUCCESS == SIRF_PAL_HW_WriteRESET( level ))
         {
            result = SIRF_SUCCESS;
         }
      }
      break;

   case UI_CTRL_MODE_HW_RESET_NONE:
   case UI_CTRL_MODE_HW_RESET_GPIO:
      ResetPortHandle = NULL;
      SIRF_PAL_HW_WriteRESET(level);
      result = SIRF_SUCCESS;
      break;

   default:
      break;
   }
   
   return result;
}

/**
* @brief Close the reset port.
* @return                       Success code.
*/
tSIRF_RESULT SIRF_PAL_HW_CloseRESET( void )
{
   tSIRF_RESULT result = SIRF_FAILURE;

   switch( g_userPalConfig.ext_sreset_n_line_usage)
   {
      case UI_CTRL_MODE_HW_RESET_UART_DTR:
         if( (SIRF_SUCCESS == SIRF_PAL_COM_UART_Close(ResetPortHandle)) &&
             (SIRF_SUCCESS == SIRF_PAL_COM_UART_Delete(&ResetPortHandle)) )
         {
            result = SIRF_SUCCESS;
         }
         break;

      case UI_CTRL_MODE_HW_RESET_GPIO:
      case UI_CTRL_MODE_HW_RESET_NONE:
         result = SIRF_SUCCESS;
         break;

      default:
         result = SIRF_FAILURE;
         break;
   }

   return result;
}

/**
* @brief Reset the tracker.
* @param[in] level              Non-zero to put into reset.
* @return                       Success code.
*/
tSIRF_RESULT SIRF_PAL_HW_WriteRESET( tSIRF_UINT32 level )
{
   tSIRF_RESULT retVal = SIRF_FAILURE;

   /* Call appropriate function based on how the ON_OFF line is configured */
   switch ( g_userPalConfig.ext_sreset_n_line_usage )
   {
   case UI_CTRL_MODE_HW_RESET_UART_DTR:
      if ( SIRF_PAL_HW_RESET_HIGH == level )
      {
         retVal = SIRF_PAL_COM_UART_ClrDTR( ResetPortHandle );
      }
      else
      {
         retVal = SIRF_PAL_COM_UART_SetDTR( ResetPortHandle );
      }
      break;

   case UI_CTRL_MODE_HW_RESET_GPIO:
      {
		#ifdef NS115
		int fd_gpio;
                char reset_buff[2] = {'1', '0'};
                //char reset_on = 1;
                //char reset_off = 0;
                fd_gpio = open("sys/class/gpio/gpio60/value", O_RDWR);
                LOGE("returning fd from GPIO 0x%08x\n", fd_gpio);
                LOGE("GPS \n");
                LOGE("GPS \n");
                LOGE("GPS \n");
                //LOGE("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA \n");
                if(fd_gpio >= 0)
                {   
    
                        /*write(fd_gpio,&reset_buff[0],1);
                        SIRF_PAL_OS_THREAD_Sleep(10);
                        LOGE("GPS RESET IS HIGH\n");*/
                        if(level == SIRF_PAL_HW_ON_HIGH)
                        {   
    
                                //SIRF_PAL_OS_THREAD_Sleep(10);
                                //cmd |= 0x40;
                                write(fd_gpio,&reset_buff[0],1);
                                SIRF_PAL_OS_THREAD_Sleep(10);
                                //SIRF_PAL_OS_THREAD_Sleep(10000000);
                                //write(fd_gpio,&reset_on,1);
                                LOGE("GPS RESET IS HIGH\n");
                        }   
                        else
                        {   
                                //cmd &= ~0x40;
                                write(fd_gpio,&reset_buff[1],1);
                                //write(fd_gpio,&reset_off,1);
                                LOGE("GPS RESET IS LOW\n");
                        }   
                        close(fd_gpio);
                }   
                else
                {   
                        LOGE("%s: could not open gpio for GPS reset", __FUNCTION__);
                }	

		#else
                int fd_gpio;
                int ret;
                unsigned char cmd = 0;
                struct RWREGS_PARAM parm;
                parm.start_reg = 0xFC23;
                parm.byte_cnt = 1;
                parm.buf = &cmd;

                fd_gpio = open("/dev/io373x", O_RDWR);
                LOGE("returning fd from GPIO 0x%08x\n", fd_gpio);
                //LOGE("BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB \n");
                if(fd_gpio >= 0)
                {

                        cmd = 0;
                        parm.start_reg = 0xFC23;
                        ret = ioctl(fd_gpio, ENEEC_IOC_READ_REGS, &parm);
                        if(ret < 0){
                                LOGE("Read EC 0xFC23 out state error: %d\n", ret);
                        }
                        //LOGE("FC25: 0x%02x\n", (0x08 & cmd));
                        if(level == SIRF_PAL_HW_ON_HIGH)
                        {

                                SIRF_PAL_OS_THREAD_Sleep(10);
                                cmd |= 0x40;
                                LOGE("GPS RESET ON\n");
                        }
                        else
                        {
                                cmd &= ~0x40;
                                LOGE("GPS RESET OFF\n");
                        }

                        ret = ioctl(fd_gpio, ENEEC_IOC_WRITE_REGS, &parm);
                        if(ret < 0){
                                LOGE("Write EC 0xFC23 error: %d\n", ret);
                        }
                        else
                        {
                                retVal = SIRF_SUCCESS;
                        }
                        close(fd_gpio);
                        LOGE("closed GPIO for reset\n");
                }
                else
                {
                        LOGE("%s: could not open EC for GPS reset", __FUNCTION__);
                }
        #endif



         break;
      }

   case UI_CTRL_MODE_HW_RESET_NONE:
      retVal = SIRF_SUCCESS;
      break;

   default:
      retVal = SIRF_FAILURE;
      break;
   }

   return retVal;
} /* SIRF_PAL_HW_WriteRESET() */

/**
 * @}
 * End of file.
 */


