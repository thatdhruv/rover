/* Test hooks for the MPU-6050 register emulator. */
#ifndef ROVER_MPU6050_EMUL_H_
#define ROVER_MPU6050_EMUL_H_

#include <stdint.h>
#include <zephyr/drivers/emul.h>

/* Load raw sensor registers: accel xyz, temp, gyro xyz. */
void rover_mpu6050_emul_set_raw(const struct emul *target, const int16_t raw[7]);

/* Read back a register (to check what the driver configured). */
uint8_t rover_mpu6050_emul_get_reg(const struct emul *target, uint8_t reg);

/* Make the next register access fail, to test error paths. */
void rover_mpu6050_emul_set_fail(const struct emul *target, bool fail);

#endif /* ROVER_MPU6050_EMUL_H_ */
