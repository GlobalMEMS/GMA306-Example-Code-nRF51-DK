/*! @mainpage
 *
 ****************************************************************************
 * Copyright (C) 2016 GlobalMEMS, Inc. <www.globalmems.com>
 * All rights reserved.
 *
 * File : sensor_driver_test.c
 *
 * Usage: GMA306 Sensor Driver Test
 *
 ****************************************************************************
 * @section License
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
 
/*! @file sensor_driver_test.c
 *  @brief  GMA306 Sensor Driver Test Main Program 
 *  @author Joseph FC Tseng
 */
 
#include <stdio.h>
#include <stdlib.h>
#include "nrf_drv_clock.h"
#include "nrf_delay.h"
#include "nrf_soc.h"
#include "app_error.h"
#include "app_uart.h"
#include "boards.h"
#include "nrf.h"
#include "bsp.h"
#include "gma306.h"
#include "app_twi.h"
#include "gSensor_autoNil.h"

#define UART_TX_BUF_SIZE            256                  // UART TX buffer size
#define UART_RX_BUF_SIZE            1                    // UART RX buffer size
#define MAX_PENDING_TRANSACTIONS    5                    // TWI (I2C)
#define DELAY_MS(ms)	            nrf_delay_ms(ms)

static app_twi_t m_app_twi = APP_TWI_INSTANCE(0);
static uint8_t ui8StartAutoNilFlag = 0;

static void event_handler_uart(app_uart_evt_t * p_event){

  uint8_t cr;

  switch (p_event->evt_type){

  case APP_UART_DATA_READY: //echo

    while(app_uart_get(&cr) == NRF_SUCCESS){
      printf("%c", cr);
      if(cr == 'y' || cr == 'Y'){
	ui8StartAutoNilFlag = 1;
      }
    }

    break;
  case APP_UART_TX_EMPTY:
    //do nothin
    break;
  case APP_UART_COMMUNICATION_ERROR:
    APP_ERROR_HANDLER(p_event->data.error_communication);
    break;
  case APP_UART_FIFO_ERROR:
    APP_ERROR_HANDLER(p_event->data.error_code);
    break;

  default:
    break;
  }
}

void init_lfclk(void){

  uint32_t err_code;

  // Start internal LFCLK XTAL oscillator - it is needed by BSP to handle
  // buttons with the use of APP_TIMER

  err_code = nrf_drv_clock_init(NULL);
  APP_ERROR_CHECK(err_code);
  nrf_drv_clock_lfclk_request();

}

void init_uart(void)
{
  uint32_t err_code;

  app_uart_comm_params_t const comm_params =
    {
      RX_PIN_NUMBER,
      TX_PIN_NUMBER,
      RTS_PIN_NUMBER,
      CTS_PIN_NUMBER,
      APP_UART_FLOW_CONTROL_DISABLED,
      false,
      UART_BAUDRATE_BAUDRATE_Baud115200
    };

  APP_UART_FIFO_INIT(&comm_params,
		     UART_RX_BUF_SIZE,
		     UART_TX_BUF_SIZE,
		     event_handler_uart,
		     APP_IRQ_PRIORITY_LOW,
		     err_code);

  APP_ERROR_CHECK(err_code);
}

/**
 * Initialize two wire interface (I2C)
 */
void init_twi(nrf_twi_frequency_t clk){

  uint32_t err_code;

  nrf_drv_twi_config_t const config = {
    .scl                = ARDUINO_SCL_PIN,
    .sda                = ARDUINO_SDA_PIN,
    .frequency          = clk,
    .interrupt_priority = APP_IRQ_PRIORITY_LOW
  };

  APP_TWI_INIT(&m_app_twi, &config, MAX_PENDING_TRANSACTIONS, err_code);
  APP_ERROR_CHECK(err_code);

}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{

  u8 c;
  bus_support_t gma306_bus;
  raw_data_xyzt_t rawData;
  raw_data_xyzt_t offsetData;

  //Config and initialize LFCLK
  init_lfclk();

  //Config. and initialize UART
  init_uart();

  //Config. and initialize TWI (I2C)
  init_twi(NRF_TWI_FREQ_400K);
	
  /* GMA306 I2C bus setup */
  bus_init_I2C(&gma306_bus, &m_app_twi, GMA306_7BIT_I2C_ADDR);
  gma306_bus_init(&gma306_bus);

  /* GMA306 soft reset */
  gma306_soft_reset();

  /* GMA306 initialization */
  gma306_initialization();
	
  /* GMA306 Offset AutoNil */
  printf("Place and hold g-sensor in level for offset AutoNil.\r"); 
  printf("Press y when ready.\n");

  do{
    sd_app_evt_wait();
  }while(ui8StartAutoNilFlag == 0);

  //Conduct g-sensor AutoNil, g is along the Z-axis
  gSensorAutoNil(gma306_read_data_xyz, AUTONIL_AUTO + AUTONIL_Z, GMA306_RAW_DATA_SENSITIVITY, &offsetData);

  printf("Offset_XYZ=%d,%d,%d\n", offsetData.u.x, offsetData.u.y, offsetData.u.z);
	
  for(;;)
    {

      /* Read XYZT data */
      gma306_read_data_xyzt(&rawData);

      printf("Raw_XYZT=%d,%d,%d,%d\n", rawData.u.x, rawData.u.y, rawData.u.z, rawData.u.t);
			
      printf("Calib_XYZ=%d,%d,%d\n", rawData.u.x - offsetData.u.x, rawData.u.y - offsetData.u.y, rawData.u.z - offsetData.u.z);

      /* Delay 1 sec */
      DELAY_MS(1000);
		
    }
}
