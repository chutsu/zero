#ifndef ZERO_DRIVER_SBGC_H
#define ZERO_DRIVER_SBGC_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>

#include "zero/log.h"
#include "zero/core.h"

/* GENERAL */
#define SBGC_CMD_MAX_BYTES 255
#define SBGC_CMD_PAYLOAD_BYTES 5
#define DEG_PER_BIT 0.02197265625    // deg per bit
#define DEG_SEC_PER_BIT 0.1220740379 // deg/sec per bit
#define ACC_UNIT (1.0 / 512.0)       // G
#define GYRO_UNIT 0.06103701895      // deg per sec

/* CMD ID */
#define CMD_READ_PARAMS 82
#define CMD_WRITE_PARAMS 87
#define CMD_REALTIME_DATA 68
#define CMD_BOARD_INFO 86
#define CMD_CALIB_ACC 65
#define CMD_CALIB_GYRO 103
#define CMD_CALIB_EXT_GAIN 71
#define CMD_USE_DEFAULTS 70
#define CMD_CALIB_POLES 80
#define CMD_RESET 114
#define CMD_HELPER_DATA 72
#define CMD_CALIB_OFFSET 79
#define CMD_CALIB_BAT 66
#define CMD_MOTORS_ON 77
#define CMD_MOTORS_OFF 109
#define CMD_CONTROL 67
#define CMD_TRIGGER_PIN 84
#define CMD_EXECUTE_MENU 69
#define CMD_GET_ANGLES 73
#define CMD_GET_ANGLES_EXT 61
#define CMD_CONFIRM 67
// board v3.x only
#define CMD_BOARD_INFO_3 20
#define CMD_READ_PARAMS_3 21
#define CMD_WRITE_PARAMS_3 22
#define CMD_REALTIME_DATA_3 23
#define CMD_REALTIME_DATA_4 25
#define CMD_SELECT_IMU_3 24
#define CMD_READ_PROFILE_NAMES 28
#define CMD_WRITE_PROFILE_NAMES 29
#define CMD_QUEUE_PARAMS_INFO_3 30
#define CMD_SET_ADJ_VARS_VAL 31
#define CMD_SAVE_PARAMS_3 32
#define CMD_READ_PARAMS_EXT 33
#define CMD_WRITE_PARAMS_EXT 34
#define CMD_AUTO_PID 35
#define CMD_SERVO_OUT 36
#define CMD_I2C_WRITE_REG_BUF 39
#define CMD_I2C_READ_REG_BUF 40
#define CMD_WRITE_EXTERNAL_DATA 41
#define CMD_READ_EXTERNAL_DATA 42
#define CMD_READ_ADJ_VARS_CFG 43
#define CMD_WRITE_ADJ_VARS_CFG 44
#define CMD_API_VIRT_CH_CONTROL 45
#define CMD_ADJ_VARS_STATE 46
#define CMD_EEPROM_WRITE 47
#define CMD_EEPROM_READ 48
#define CMD_CALIB_INFO 49
#define CMD_BOOT_MODE_3 51

/* CMD FRAME SIZE */
#define MIN_FRAME_SIZE 5 // 4 bytes for header + 1 body checksum
#define CMD_BOARD_INFO_FRAME_SIZE 5 + 18
#define CMD_REALTIME_DATA_3_FRAME_SIZE 5 + 63

/* CMD CONTROL */
#define MODE_NO_CONTROL 0
#define MODE_SPEED 1
#define MODE_ANGLE 2
#define MODE_SPEED_ANGLE 3
#define MODE_RC 4
#define MODE_ANGLE_REL_FRAME 5

/* SYSTEM ERRORS */
// ERR_NO_SENSOR (1<<0)
// ERR_CALIB_ACC (1<<1)
// ERR_SET_POWER (1<<2)
// ERR_CALIB_POLES (1<<3)
// ERR_PROTECTION (1<<4)
// ERR_SERIAL (1<<5)
// Beside that, extended error contains bits:
// ERR_LOW_BAT1 (1<<6)
// ERR_LOW_BAT2 (1<<7)
// ERR_GUI_VERSION (1<<8)
// ERR_MISS_STEPS (1<<9)
// ERR_SYSTEM (1<<10)
// ERR_EMERGENCY_STOP (1<<11)

/* MACROS */
#define S16BIT(DATA, HI_BYTE, LOW_BYTE)                                        \
  (int16_t)((DATA[HI_BYTE] << 8) | (DATA[LOW_BYTE] & 0xff))

#define U16BIT(DATA, HI_BYTE, LOW_BYTE)                                        \
  (uint16_t)((DATA[HI_BYTE] << 8) | (DATA[LOW_BYTE] & 0xff))

/**
 * SBGC UART protocol frame
 */
typedef struct sbgc_frame_t {
  uint8_t cmd_id;
  uint8_t data_size;
  uint8_t header_checksum;
  uint8_t *data;
  uint8_t data_checksum;
} sbgc_frame_t;

/**
 * SBGC realtime data
 */
typedef struct sbgc_realtime_data_t {
  vec3_t accel;
  vec3_t gyro;

  vec3_t camera_angles;
  vec3_t frame_angles;
  vec3_t rc_angles;
  vec3_t encoder_angles;

  int cycle_time;
  int i2c_error_count;
  int system_error;
  int battery_level;
} sbgc_realtime_data_t;

/**
 * SBGC
 */
typedef struct sbgc_t {
  int connected;

  sbgc_realtime_data_t data;
  char *port;
  int serial;

  uint8_t board_version;
  uint16_t firmware_version;
  uint8_t debug_mode;
  uint16_t board_features;
  uint8_t connection_flags;
} sbgc_t;

void sbgc_frame_print(const sbgc_frame_t *frame);
void sbgc_frame_set_checksum(sbgc_frame_t *frame);
void sbgc_frame_set_header(sbgc_frame_t *frame,
                           uint8_t cmd_id,
                           uint8_t data_size);
void sbgc_frame_set_body(sbgc_frame_t *frame, uint8_t *data);
void sbgc_frame_build(sbgc_frame_t *frame,
                      int cmd_id,
                      uint8_t *data,
                      int data_size);
int sbgc_frame_parse_header(sbgc_frame_t *frame, uint8_t *data);
int sbgc_frame_parse_body(sbgc_frame_t *frame, uint8_t *data);
int sbgc_frame_parse(sbgc_frame_t *frame, uint8_t *data);

void sbgc_realtime_data_print(const sbgc_realtime_data_t *data);

int sbgc_connect(sbgc_t *sbgc);
int sbgc_disconnect(sbgc_t *sbgc);
// int sbgc_send(const sbgc_t *sbgc, const sbgc_frame_t &cmd);
// int sbgc_read(const sbgc_t *sbgc,
//               const uint8_t read_length,
//               sbgc_frame_t *frame);
// int sbgc_on(const sbgc_t *sbgc);
// int sbgc_off(const sbgc_t *sbgc);
// int sbgc_reset(sbgc_t *sbgc);
// int sbgc_get_board_info(sbgc_t *sbgc);
// int sbgc_get_realtime_data(sbgc_t *sbgc);
// int sbgc_get_realtime_data4(sbgc_t *sbgc);
// int sbgc_get_angles_ext(sbgc_t *sbgc);
// int sbgc_set_angle(const sbgc_t *sbgc,
//                    const double roll,
//                    const double pitch,
//                    const double yaw);
// int sbgc_set_speed_angle(const sbgc_t *sbgc,
//                          const double roll,
//                          const double pitch,
//                          const double yaw,
//                          const double roll_speed,
//                          const double pitch_speed,
//                          const double yaw_speed);
#endif /* ZERO_DRIVER_SBGC_H */
