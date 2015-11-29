/*
 * mpu6050.h
 *
 * Created: 11/28/2015 12:53:10 PM
 *  Author: Vo Huu Tai
 */ 


#ifndef MPU6050_H_
#define MPU6050_H_
void init_wire();
int MPU6050_read(int start, uint8_t *buffer, int size);
int MPU6050_write(int start, const uint8_t *pData, int size);
int MPU6050_write_reg(int reg, uint8_t data);

#endif /* MPU6050_H_ */