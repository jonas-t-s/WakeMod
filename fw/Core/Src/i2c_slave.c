#include "main.h"
#include "ook.h"
#include "stm32u0xx_hal.h"
#include "string.h"
#include "ulog.h"
#include "utils.h"

uint8_t transmit_data[I2C_TRANSMIT_SIZE] = {
    0}; // transmit: data that should be transmitted to the master as response
        // to his read request
uint8_t read_data[I2C_READ_SIZE] = {
    0}; // read: data that should be received from the master as response to his
        // write request
uint8_t reg = 255;
bool next_is_data = false;

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c) {
  HAL_ResumeTick(); // resume tick in case IRQ was fired by a shutdown impulse
  HAL_I2C_EnableListen_IT(hi2c);
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection,
                          uint16_t AddrMatchCode) {
  HAL_ResumeTick(); // Resume tick, as it was possibly stopped due to sleep;
  ULOG_INF("Got I2C address callback");
  if (TransferDirection ==
      I2C_DIRECTION_TRANSMIT) // if the master wants to transmit the data
  {
    reg = 255;            // reset the register which is manipulated
    next_is_data = false; // next one is register
    HAL_I2C_Slave_Seq_Receive_IT(hi2c, &reg, 1, I2C_FIRST_FRAME);
  }

  else {
    HAL_I2C_Slave_Seq_Transmit_IT(hi2c, transmit_data, I2C_TRANSMIT_SIZE,
                                  I2C_FIRST_AND_LAST_FRAME);
  }
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c) {
  HAL_ResumeTick(); // resume tick in case IRQ was fired by a shutdown impulse
  ULOG_INF("Successfuly sent data");
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {
  HAL_ResumeTick(); // resume tick in case IRQ was fired by a shutdown impulse
  // WHO AM I
  if (!next_is_data && reg == 0x00) {
    ULOG_DBG("Got 'WHO_AM_I' request");
    memset(transmit_data, 0, I2C_TRANSMIT_SIZE);
    transmit_data[0] = 0x42;
    transmit_data[1] = 0x42;
  }

  // SETUP WuR (will be followed by data)
  else if (reg == 0x01) {
    if (!next_is_data) {
      ULOG_DBG("Got 'SETUP_WUR' request");
      next_is_data = true;
      memset(read_data, 0, I2C_READ_SIZE);
      HAL_I2C_Slave_Seq_Receive_IT(hi2c, read_data, I2C_READ_SIZE,
                                   I2C_LAST_FRAME);
    } else {
      ULOG_DBG("Got 'DATA' for 'SETUP_WUR'");
      ULOG_DBG("%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d", read_data[0],
               read_data[1], read_data[2], read_data[3], read_data[4],
               read_data[5], read_data[6], read_data[7], read_data[8],
               read_data[9], read_data[10]);
      struct i2c_read_message msg = {.reg = reg, .data = {0}};
      memcpy(msg.data, read_data, I2C_READ_SIZE);
      lwrb_write(&i2c_read_ring_buffer, (void *)&msg,
                 sizeof(struct i2c_read_message));
    }
  }

  // SEND WuC (will be followed by data)
  else if (reg == 0x02) {
    if (!next_is_data) {
      ULOG_DBG("Got 'SEND_WUC' request");
      next_is_data = true;
      memset(read_data, 0, I2C_READ_SIZE);
      HAL_I2C_Slave_Seq_Receive_IT(hi2c, read_data, I2C_READ_SIZE,
                                   I2C_LAST_FRAME);
    } else {
      ULOG_DBG("Got 'DATA' for 'SEND_WUC'");
      ULOG_DBG("%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d", read_data[0],
               read_data[1], read_data[2], read_data[3], read_data[4],
               read_data[5], read_data[6], read_data[7], read_data[8],
               read_data[9], read_data[10]);
      struct i2c_read_message msg = {.reg = reg, .data = {0}};
      memcpy(msg.data, read_data, I2C_READ_SIZE);
      lwrb_write(&i2c_read_ring_buffer, (void *)&msg,
                 sizeof(struct i2c_read_message));
    }
  }

  // IRQ REASON
  else if (!next_is_data && reg == 0x03) {
    if (!next_is_data) {
      ULOG_DBG("Got 'IRQ_REASON' request");
      memset(transmit_data, 0, I2C_TRANSMIT_SIZE);
      transmit_data[0] = last_irq_data.irq_source;
      transmit_data[1] = last_irq_data.got_data;
      memcpy(&transmit_data[2], (void *)last_irq_data.data, WUC_DATA_LENGTH);
    }
  }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
  HAL_ResumeTick(); // resume tick in case IRQ was fired by a shutdown impulse
  HAL_I2C_EnableListen_IT(hi2c);
}
