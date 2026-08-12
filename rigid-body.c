#include "wokwi-api.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define REG_WHO_AM_I   0x00
#define REG_GYRO_BASE  0x10
#define REG_Q_MEAS     0x20
#define REG_Q_TRUE     0x30
#define REG_W_TRUE     0x40
#define REG_TORQUE     0x50
#define REG_CONTROL    0x60
#define REG_GYRO_SIGMA 0x61

typedef struct {
  // Rotational Dynamics State (Hamilton order: w, x, y, z)
  float q[4];
  float w[3];
  float tau[3];
  float J[3];
  
  // Wire Interface State
  uint8_t current_reg;
  uint8_t wire_buffer[64];
  uint8_t buffer_index;
  
  // Simulation Scaffolding
  uint32_t i2c_dev;
} chip_state_t;

static void on_timer_tick(void *user_data) {
  chip_state_t *chip = (chip_state_t *)user_data;
  
  // M0: Scaffolding only. Plant dynamics (RK4) attach here in M1.
  (void)chip;
}

static bool on_i2c_connect(void *user_data, uint8_t address, bool is_write) {
  chip_state_t *chip = (chip_state_t *)user_data;
  chip->buffer_index = 0;
  return true; // Acknowledge connection
}

static uint8_t on_i2c_read(void *user_data) {
  chip_state_t *chip = (chip_state_t *)user_data;
  uint8_t data = 0;

  // Handle reads out of the multi-byte layout
  if (chip->current_reg == REG_WHO_AM_I) {
    data = 0x51;
  } else if (chip->current_reg >= REG_GYRO_BASE && chip->current_reg < 0x60) {
    // Map internal floats directly to byte stream for multi-byte reads
    uint8_t *state_bytes = (uint8_t *)&chip->q[0]; // State block offset alignment
    int offset = chip->current_reg - REG_GYRO_BASE;
    // Multi-byte safety mapping logic implemented in M1
    data = 0x00; 
  }

  // Auto-increment pointer to support single-transaction multi-byte bursts
  chip->current_reg++;
  return data;
}

static bool on_i2c_write(void *user_data, uint8_t data) {
  chip_state_t *chip = (chip_state_t *)user_data;
  
  if (chip->buffer_index == 0) {
    // First byte of a write sequence establishes the target register pointer
    chip->current_reg = data;
    chip->buffer_index++;
    return true;
  }

  // Collect subsequent bytes for command writes (e.g., Torques)
  if (chip->current_reg >= REG_TORQUE && chip->current_reg <= REG_GYRO_SIGMA) {
    // Buffer input data for multi-byte parsing at disconnect
    if (chip->buffer_index - 1 < sizeof(chip->wire_buffer)) {
      chip->wire_buffer[chip->buffer_index - 1] = data;
      chip->buffer_index++;
    }
  }
  return true;
}

static void on_i2c_disconnect(void *user_data) {
  chip_state_t *chip = (chip_state_t *)user_data;
  
  // If we processed incoming data for the torque registers, load it
  if (chip->current_reg >= REG_TORQUE && chip->buffer_index > 1) {
    int payload_len = chip->buffer_index - 1;
    if (chip->current_reg == REG_TORQUE && payload_len == 12) {
      memcpy(chip->tau, chip->wire_buffer, 12);
    }
  }
  chip->buffer_index = 0;
}

void chip_init(void) {
  chip_state_t *chip = malloc(sizeof(chip_state_t));
  memset(chip, 0, sizeof(chip_state_t));

  // Initialize State to standard defaults
  chip->q[0] = 1.0f; // Identity Quaternion
  chip->J[0] = 1.0f; // Jx
  chip->J[1] = 2.0f; // Jy
  chip->J[2] = 3.0f; // Jz

  // Configure I2C Peripherals
  const i2c_config_t i2c_config = {
    .address = 0x42,
    .scl = pin_init("SCL", INPUT),
    .sda = pin_init("SDA", INPUT),
    .connect = on_i2c_connect,
    .read = on_i2c_read,
    .write = on_i2c_write,
    .disconnect = on_i2c_disconnect,
    .user_data = chip,
  };
  chip->i2c_dev = i2c_init(&i2c_config);

  // Spool up the high-frequency physical simulation thread (1 kHz)
  const timer_config_t timer_config = {
    .callback = on_timer_tick,
    .period = 1000, // 1000 microseconds
    .user_data = chip,
  };
  timer_start(timer_init(&timer_config));
}