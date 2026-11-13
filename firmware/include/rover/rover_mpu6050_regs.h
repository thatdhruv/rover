/* Register map for MPU-6050 compatible IMUs (shared by driver and emulator). */
#ifndef ROVER_MPU6050_REGS_H_
#define ROVER_MPU6050_REGS_H_

#define MPU6050_REG_SMPLRT_DIV   0x19
#define MPU6050_REG_CONFIG       0x1A
#define MPU6050_REG_GYRO_CONFIG  0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_WHO_AM_I     0x75

#define MPU6050_PWR_MGMT_1_RESET 0x80
#define MPU6050_PWR_MGMT_1_SLEEP 0x40
#define MPU6050_CLKSEL_PLL_XGYRO 0x01

/* ACCEL_XOUT_H .. GYRO_ZOUT_L: accel xyz, temp, gyro xyz (big endian) */
#define MPU6050_SAMPLE_BYTES     14
#define MPU6050_NUM_RAW          7

#endif /* ROVER_MPU6050_REGS_H_ */
