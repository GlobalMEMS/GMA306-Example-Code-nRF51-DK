/*
 *
 ****************************************************************************
 * Copyright (C) 2016 GlobalMEMS, Inc. <www.globalmems.com>
 * All rights reserved.
 *
 * File : gma306.c
 *
 * Usage: GMA306 sensor driver file
 *
 ****************************************************************************
 * 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 **************************************************************************/
 
/*! @file gma306.c
 *  @brief  GMA306 Sensor Driver File 
 *  @author Joseph FC Tseng
 */
 
#include <stddef.h>
#include "gma306.h"
 
static bus_support_t* pBus_support = 0;
 
/*!
 * @brief Read multiple data from the starting regsiter address
 *
 * @param u8Addr Starting register address
 * @param pu8Data The data array of values read
 * @param u8Len Number of bytes to read
 * 
 * @return Result from the burst read function
 * @retval >= 0 Success
 * @retval -127 Error null bus
 * @retval -1   Communication error
 *
 */
s8 gma306_burst_read(u8 u8Addr, u8* pu8Data, u8 u8Len){
	
  s8 comRslt = -1;
  if(pBus_support == NULL){
    return -127;
  }
  else{
    comRslt = pBus_support->bus_read(pBus_support->p_app_twi, pBus_support->u8DevAddr, u8Addr, pu8Data, u8Len);
    if(comRslt == NRF_SUCCESS) //success, return # of bytes read
      comRslt = u8Len;
    else //return the nRF51 error code
      comRslt = -comRslt;
  }
	
  return comRslt;
}
 

/*!
 * @brief Write multiple data to the starting regsiter address
 *
 * @param u8Addr Starting register address
 * @param pu8Data The data array of values to write
 * @param u8Len Number of bytes to write
 * 
 * @return Result from the burst write function
 * @retval >= 0 Success
 * @retval -127 Error null bus
 * @retval -1   Communication error
 *
 */
s8 gma306_burst_write(u8 u8Addr, u8* pu8Data, u8 u8Len){
	
  s8 comRslt = -1;
  if(pBus_support == NULL){
    return -127;
  }
  else{
    comRslt = pBus_support->bus_write(pBus_support->p_app_twi, pBus_support->u8DevAddr, u8Addr, pu8Data, u8Len);
    if(comRslt == NRF_SUCCESS) //success, return # of bytes write
      comRslt = u8Len;
    else //return the nRF51 error code
      comRslt = -comRslt;
  }
	
  return comRslt;;	
}

/*!
 * @brief GMA306 initialize communication bus
 *
 * @param pbus Pointer to the I2C/SPI read/write bus support struct
 * 
 * @return Result from bus communication function
 * @retval 0 Success
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
s8 gma306_bus_init(bus_support_t* pbus){
	
  s8 comRslt = -1;
  u8 u8Data;
	
  //assign the I2C/SPI bus
  if(pbus == NULL)
    return -127;
  else
    pBus_support = pbus;
	
  //Read chip ID
  comRslt = gma306_burst_read(GMA1302_REG_PID, &u8Data, 1);
	
  return comRslt;
}
 
/*!
 * @brief GMA306 soft reset
 *
 * @param None
 * 
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
s8 gma306_soft_reset(void){
	
  s8 comRslt = -1;
  u8 u8Data = 0;
	
  u8Data = GMA306_SET_BITSLICE(u8Data, GMA306_RST, 1);

  //Set the RST bit
  comRslt = gma306_burst_write(GMA306_RST__REG, &u8Data, 1);
	
  return comRslt;
}

/*!
 * @brief GMA306 set operation mode.
 *
 * @param opMode: operation mode
 * @param odr: output data rate
 * 
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
s8 gma306_set_operation_mode(GMA306_OP_MODE_T opMode, GMA306_ODR_T odr){

  s8 comRslt = -1, s8Tmp;
  u8 u8Data[4];
  u8 dataLen = 0;
  GMA306_ODR_T odrToSet = GMA306_ODR_NA; //default
	
  switch(opMode){
  case GMA306_OP_MODE_STANDBY:
    //GMA306 set to standby mode. ADC will stop. Registers values will be kept.
    u8Data[0] = 0x02;
    u8Data[1] = 0x00;
    dataLen = 2;
    break;
  case GMA306_OP_MODE_SUSPEND:
    //Suspend GMA306. Register values will be lost. Minimum power consumption.
    u8Data[0] = 0;
    u8Data[0] = GMA306_SET_BITSLICE(u8Data[0], GMA306_PD_LDO, 1);
    u8Data[0] = GMA306_SET_BITSLICE(u8Data[0], GMA306_PD_BG, 1);
    comRslt = gma306_burst_write(GMA306_PD__REG, u8Data, 1);
    goto EXIT;
  case GMA306_OP_MODE_CM:
    //Stop DSP, then enter continuous mode
    //02h = 0x02, 0x00, 0x04, 0x00
    u8Data[0] = 0x02;
    u8Data[1] = 0x00;
    u8Data[2] = 0x04;
    u8Data[3] = 0x00;
    dataLen = 4;
    break;
  case GMA306_OP_MODE_NCM:
    //Stop DSP, then enter non-continuous mode
    //02h = 0x02, 0x00, 0x08, 0x00
    u8Data[0] = 0x02;
    u8Data[1] = 0x00;
    u8Data[2] = 0x08;
    u8Data[3] = 0x00;
    dataLen = 4;
    odrToSet = odr; //only NCM will take the ODR parameter
    break;
  default:
    goto EXIT;
  }
	
  //Write ACTR register to change mode
  comRslt = gma306_burst_write(GMA306_ACTR__REG, u8Data, dataLen);
	
  if(odrToSet != GMA306_ODR_NA){ 		//set NCM ODR
    //Get the original configuration
    s8Tmp = gma306_burst_read(GMA306_NCM_ODR__REG, u8Data, 1);
    if(s8Tmp < 0){ //communication error
      comRslt = s8Tmp;
      goto EXIT;
    }
    else
      comRslt += s8Tmp;
		
    u8Data[0] = GMA306_SET_BITSLICE(u8Data[0], GMA306_NCM_ODR, odrToSet);
		
    //Write the new filter setting 
    s8Tmp = gma306_burst_write(GMA306_NCM_ODR__REG, u8Data, 1);
    if(s8Tmp < 0){ //communication error
      comRslt = s8Tmp;
      goto EXIT;
    }
    else
      comRslt += s8Tmp;			
  }
	
 EXIT:
  return comRslt;
}


/*!
 * @brief GMA306 set motion threshold
 *
 * @param threshold: motion threshold, 1 threshold code = 0.25g, max 0x1F (31)
 * 
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
s8 gma306_set_motion_threshold(u8 threshold){
	
  s8 comRslt = -1;
  u8 th = threshold;
	
  if(th > MAX_MOTION_THRESHOLD) 
    th = MAX_MOTION_THRESHOLD;
	
  comRslt = gma306_burst_write(GMA1302_REG_MTHR, &th, 1);
	
  return comRslt;
}

/*!
 * @brief GMA306 set oversampling ratio
 *
 * @param ratio: oversampling ratio to set
 * 
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
s8 gma306_set_osm(GMA306_OSM_T ratio){
	
  s8 comRslt = -1, s8Tmp;
  u8 u8Data[4];
	
  //Get the original OSM register value 
  s8Tmp = gma306_burst_read(GMA306_OSM__REG, u8Data, 1);
  if(s8Tmp < 0){ //communication error
    comRslt = s8Tmp;
    goto EXIT;
  }
  else
    comRslt += s8Tmp;
	
  u8Data[0] = GMA306_SET_BITSLICE(u8Data[0], GMA306_OSM, ratio);

  //Write the OSM 
  s8Tmp = gma306_burst_write(GMA306_OSM__REG, u8Data, 1);
  if(s8Tmp < 0){ //communication error
    comRslt = s8Tmp;
    goto EXIT;
  }
  else
    comRslt += s8Tmp;	

  //Stop DSP, then enter continuous mode
  //02h = 0x02, 0x00, 0x04, 0x00
  u8Data[0] = 0x02;
  u8Data[1] = 0x00;
  u8Data[2] = 0x04;
  u8Data[3] = 0x00;
  s8Tmp = gma306_burst_write(GMA306_ACTR__REG, u8Data, 4);
  if(s8Tmp < 0){ //communication error
    comRslt = s8Tmp;
    goto EXIT;
  }
  else
    comRslt += s8Tmp;	

 EXIT:
  return comRslt;
	
}

/*!
 * @brief Set GMA306 filter
 *
 * @param filter: GMA306_LP for low pass filter
 *                GMA306_HP for high pass filter
 * @param turnOn: 1 to turn on
 *                0 to turn off
 *                When turn on low-pass filter, high-pass filter will be turn off automatically.
 *                Likewise when high-pass is turned on, low-pass filter will be turned off automatically.
 * 
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
s8 gma306_set_filter(GMA306_FILTER_T filter, u8 turnOn){
	
  s8 comRslt = 0, s8Tmp;
  u8 u8Data, u8LpBitSet;
	
  //Get the original filter setting 
  s8Tmp = gma306_burst_read(GMA1302_REG_CONTR1, &u8Data, 1);
  if(s8Tmp < 0){ //communication error
    comRslt = s8Tmp;
    goto EXIT;
  }
  else
    comRslt += s8Tmp;	
	
  switch(turnOn){
  case 0: //turn off
    if(filter == GMA306_LP){
      //turn off CM LP
      u8Data = GMA306_SET_BITSLICE(u8Data, GMA306_LP_CM, 0);
      //turn off NCM LP
      u8Data = GMA306_SET_BITSLICE(u8Data, GMA306_LP_NCM, 0);
    }
    else{
      //turn off CM HP
      u8Data = GMA306_SET_BITSLICE(u8Data, GMA306_HP_CM, 0);
      //turn off NCM HP
      u8Data = GMA306_SET_BITSLICE(u8Data, GMA306_HP_NCM, 0);
    }
    break;
  default: //turn on
    if(filter == GMA306_LP)
      u8LpBitSet = 1; //Turn on low pass, auto turn-off high pass
    else
      u8LpBitSet = 0; //Turn on high pass, auto turn-off low pass
			
    u8Data = GMA306_SET_BITSLICE(u8Data, GMA306_LP_CM, u8LpBitSet);
    u8Data = GMA306_SET_BITSLICE(u8Data, GMA306_LP_NCM, u8LpBitSet);
    u8Data = GMA306_SET_BITSLICE(u8Data, GMA306_HP_CM, ~u8LpBitSet);
    u8Data = GMA306_SET_BITSLICE(u8Data, GMA306_HP_NCM, ~u8LpBitSet);
    break;
  }
	
  //Write the new filter setting 
  s8Tmp = gma306_burst_write(GMA1302_REG_CONTR1, &u8Data, 1);
  if(s8Tmp < 0){ //communication error
    comRslt = s8Tmp;
    goto EXIT;
  }
  else
    comRslt += s8Tmp;		
	
 EXIT:
  return comRslt;
}

/*!
 * @brief Set GMA306 interrupt
 *
 * @param interrupt: GMA306_INT_DATA for data ready interrupt
 *                   GMA306_INT_MOTION for motion interrupt
 * @param turnOn: 1 to turn on
 *                0 to turn off
 * 
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
s8 gma306_set_interrupt(GMA306_INTERRUPT_T interrupt, u8 turnOn){
	
  s8 comRslt = 0, s8Tmp;
  u8 u8Data, u8DrdyBitSet;
	
  //Get the original interrupt setting 
  s8Tmp = gma306_burst_read(GMA1302_REG_INTCR, &u8Data, 1);
  if(s8Tmp < 0){ //communication error
    comRslt = s8Tmp;
    goto EXIT;
  }
  else
    comRslt += s8Tmp;
	
  switch(turnOn){
  case 0: //turn off
    u8DrdyBitSet = 0;
    break;
  default: //turn on
    u8DrdyBitSet = 1;
    break;
  }	
	
  switch(interrupt){
  case GMA306_INT_DATA:
    //Set CM data ready INT bit
    u8Data = GMA306_SET_BITSLICE(u8Data, GMA306_DRDY_CM, u8DrdyBitSet);
    //Set NCM data ready INT bit
    u8Data = GMA306_SET_BITSLICE(u8Data, GMA306_DRDY_NCM, u8DrdyBitSet);
    break;
  default:
    //Set CM motion INT bit
    u8Data = GMA306_SET_BITSLICE(u8Data, GMA306_MOTION_CM, u8DrdyBitSet);
    //Set NCM motion INT bit
    u8Data = GMA306_SET_BITSLICE(u8Data, GMA306_MOTION_NCM, u8DrdyBitSet);			
    break;
  }

  //Write the new interrupt setting 
  s8Tmp = gma306_burst_write(GMA1302_REG_INTCR, &u8Data, 1);
  if(s8Tmp < 0){ //communication error
    comRslt = s8Tmp;
    goto EXIT;
  }
  else
    comRslt += s8Tmp;	
	
 EXIT:
  return comRslt;
}

/*!
 * @brief Configure GMA306 interrupt pin configuration
 *
 * @param cfg: cfg[0]: 1'b0=>push-pull, 1'b1=>open-drain
 *             cfg[1]: 1'b0=>active low, 1'b1=>active high        
 * 
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
s8 gma306_config_INT_pin(u8 cfg){
	
  s8 comRslt = 0, s8Tmp;
  u8 u8Data;
	
  //Get the original interrupt setting 
  s8Tmp = gma306_burst_read(GMA1302_REG_INTCR, &u8Data, 1);
  if(s8Tmp < 0){ //communication error
    comRslt = s8Tmp;
    goto EXIT;
  }
  else
    comRslt += s8Tmp;
	
  //cfg[0]: INT pin type
  u8Data = GMA306_SET_BITSLICE(u8Data, GMA306_INT_PIN_TYPE_CM, (cfg&0x01));
  u8Data = GMA306_SET_BITSLICE(u8Data, GMA306_INT_PIN_TYPE_NCM, (cfg&0x01));
  //cfg[1]: INT polarity
  u8Data = GMA306_SET_BITSLICE(u8Data, GMA306_INT_PIN_POLARITY, (cfg&0x02)>>1);

  //Write the new interrupt setting 
  s8Tmp = gma306_burst_write(GMA1302_REG_INTCR, &u8Data, 1);
  if(s8Tmp < 0){ //communication error
    comRslt = s8Tmp;
    goto EXIT;
  }
  else
    comRslt += s8Tmp;		
	
 EXIT:
  return comRslt;	
}

/*!
 * @brief GMA306 initialization
 *        1. Turn on offset temperature compensation
 *        2. Set to continuous mode
 *        3. Turn on low pass filter
 *        4. Set data ready INT, ative high, push-pull
 *
 * @param None
 * 
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
s8 gma306_initialization(void){
	
  s8 comRslt = 0, s8Tmp;
  u8 u8Data;
	
  //Turn on offset temperture compensation
  //Set 18h=0x40
  u8Data = 0x40;
  s8Tmp = gma306_burst_write(GMA1302_REG_CONTR3, &u8Data, 1);
  if(s8Tmp < 0){ //communication error
    comRslt = s8Tmp;
    goto EXIT;
  }
  else
    comRslt += s8Tmp;

  //Set to continuous mode
  s8Tmp = gma306_set_operation_mode(GMA306_OP_MODE_CM, GMA306_ODR_NA);

  if(s8Tmp < 0){ //communication error
    comRslt = s8Tmp;
    goto EXIT;
  }
  else
    comRslt += s8Tmp;	

  //Turn-on low pass filter
  s8Tmp = gma306_set_filter(GMA306_LP, 1);
  if(s8Tmp < 0){ //communication error
    comRslt = s8Tmp;
    goto EXIT;
  }
  else
    comRslt += s8Tmp;

  //Set data ready 
  s8Tmp = gma306_set_interrupt(GMA306_INT_DATA, 1);
  if(s8Tmp < 0){ //communication error
    comRslt = s8Tmp;
    goto EXIT;
  }
  else
    comRslt += s8Tmp;

  //INT push-pull, Active high
  gma306_config_INT_pin(GMA306_INT_PIN_PP | GMA306_INT_PIN_AH);
  if(s8Tmp < 0){ //communication error
    comRslt = s8Tmp;
    goto EXIT;
  }
  else
    comRslt += s8Tmp;
	
 EXIT:
  return comRslt;
	
}

s8 _gma306_read_data(raw_data_xyzt_t* pxyzt, u8 dLen){
	
  s8 comRslt = -1;
  u8 u8Data[11];
  s16 s16Tmp, i;

  do{
	
    if(dLen == 3) //xyz
      comRslt = gma306_burst_read(GMA306_STADR__REG, u8Data, 9);
    else  //xyzt
      comRslt = gma306_burst_read(GMA306_STADR__REG, u8Data, 11);
		
    if(comRslt < 0) goto EXIT;
		
  } while(0 && (GMA306_GET_BITSLICE(u8Data[2], GMA306_DRDY) == 0));//No Check DRDY bit
	
	
  for(i = 0; i < dLen; ++i){
    s16Tmp = (u8Data[2*i + 4] << 8) | (u8Data[2*i + 3]);
    pxyzt->v[i] = s16Tmp;
  }

  //z-axis sensitivty adjustment
  pxyzt->u.z >>= 2;
  pxyzt->u.z <<= 3;
	
 EXIT:
  return comRslt;
	
}

/*!
 * @brief GMA306 read data XYZ
 *
 * @param pxyzt Data buffer to store the values
 * 
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
s8 gma306_read_data_xyz(raw_data_xyzt_t* pxyzt){
	
  return _gma306_read_data(pxyzt, 3);
	
}

/*!
 * @brief GMA306 read data XYZT
 *
 * @param pxyzt Data buffer to store the values
 * 
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
s8 gma306_read_data_xyzt(raw_data_xyzt_t* pxyzt){
	
  return _gma306_read_data(pxyzt, 4);
	
}
