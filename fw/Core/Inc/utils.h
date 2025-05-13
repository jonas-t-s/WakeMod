#ifndef UTILS_H_
#define UTILS_H_

#include "fh101rf_reg.h"
#include "main.h"

#include "fh101rf.h"
#include "logging.h"
#include "ook.h"

#include "lwrb/lwrb.h"

// Enums and Definitions
enum ant_ctrl {
  FH101RF_DEV,
  OOK_DEV,
};

enum wusrc {
  WUSRC_RST,
  WUSRC_WUR_IRQ,
  WUSRC_SYS_SDN,
};

enum pinstate {
  PINSTATE_NONE,
  PINSTATE_WUR_IRQ,
  PINSTATE_SYS_SDN,
};

#define WUC_ADDR_LENGTH 2
#define WUC_DATA_LENGTH 6

#define I2C_READ_SIZE (WUC_DATA_LENGTH + 5)
#define I2C_TRANSMIT_SIZE (WUC_DATA_LENGTH + 2)

struct __attribute__((packed)) i2c_read_message {
  uint8_t reg;
  uint8_t data[I2C_READ_SIZE];
  uint8_t _unused;
};

struct __attribute__((packed)) wur_irq_data {
  uint8_t irq_source;
  uint8_t got_data;
  uint8_t data[WUC_DATA_LENGTH];
};

// External Variables
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern I2C_HandleTypeDef hi2c3;

extern struct fh101rf_h fh101rf_dev;
extern struct ook_h ook_dev;

extern volatile struct wur_irq_data last_irq_data;

extern lwrb_t i2c_read_ring_buffer;

// Driver Helper Functions
void driver_log(char *msg, bool is_err, bool has_int_arg, uint32_t arg);
void set_ant_control(enum ant_ctrl dev);

fh101rf_err_t init_struct_fh101rf(struct fh101rf_h *dev);
fh101rf_err_t fh101rf_spi_read_reg(uint8_t reg_adr, uint8_t *res);
fh101rf_err_t fh101rf_spi_write_reg(uint8_t reg_adr, uint8_t val);
void fh101rf_rst_set(bool level);

ook_err_t ook_spi_write(uint8_t *data, uint16_t len);
ook_err_t ook_spi_speed(enum phy_speed speed);
ook_err_t ook_spi_enable(bool enable);
void ook_log(char *msg, bool is_err, bool has_int_arg, uint32_t arg);

// Logic Functions
void goto_shutdown(bool sys_irq);
void goto_sleep(void);
void read_backup_reg(void);
void write_backup_reg(void);
enum wusrc check_wu_src(void);
enum pinstate check_pinstate(void);
bool handle_wakeup(enum wusrc wakeup_source);

/**
 * @brief Converts a phy_speed enum value to a fh101rf_sample_rate enum value.
 *
 * @param speed The phy_speed value to convert.
 * @return The corresponding fh101rf_sample_rate value, or if not existing then
 * 1024bps
 */
static inline enum fh101rf_sample_rate
ook_speed_to_fh101rf_sample_rate(enum phy_speed speed) {
  switch (speed) {
  case PHYSPEED_256:
    return FH101RF_SAMPLE_RATE_SR_0_256;
  case PHYSPEED_512:
    return FH101RF_SAMPLE_RATE_SR_0_512;
  case PHYSPEED_1024:
    return FH101RF_SAMPLE_RATE_SR_1_024;
  case PHYSPEED_2048:
    return FH101RF_SAMPLE_RATE_SR_2_048;
  case PHYSPEED_4096:
    return FH101RF_SAMPLE_RATE_SR_4_096;
  case PHYSPEED_8192:
    return FH101RF_SAMPLE_RATE_SR_8_192;
  case PHYSPEED_16384:
    return FH101RF_SAMPLE_RATE_SR_16_384;
  case PHYSPEED_32768:
    return FH101RF_SAMPLE_RATE_SR_32_768;
  default:
    return FH101RF_SAMPLE_RATE_SR_1_024;
  }
}
#endif // UTILS_H_
