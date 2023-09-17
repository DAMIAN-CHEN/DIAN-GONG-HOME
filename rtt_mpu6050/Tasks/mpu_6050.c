//
// Created by DAMIAN-CHEN on 2023/9/1.
//

#include "mpu_6050.h"

#include <math.h>
rt_thread_t mpu_6050_thread;
MPU6050_t mpu_6050_data;
struct rt_i2c_bus_device *i2c_bus;

void  mpu_6050_init(void){
i2c_bus=(struct rt_i2c_bus_device *) rt_device_find("i2c1");
    MPU6050_Init(i2c_bus);
}
void mpu_6050_thread_entry(void *argument){
    while (1) {
        /*rt_pin_write(GET_PIN(C, 13), PIN_HIGH);
        rt_thread_mdelay(200);
        rt_pin_write( GET_PIN(C, 13), PIN_LOW);
        rt_thread_mdelay(200);*/
        MPU6050_Read_Accel(i2c_bus, &mpu_6050_data);
        rt_thread_mdelay(20);

    }
}
void MPU_6050_READ(){
    MPU6050_Read_Accel(i2c_bus, &mpu_6050_data);
}
int c8t6_task_init(void){
    mpu_6050_thread= rt_thread_create("i2c1",mpu_6050_thread_entry,i2c_bus,1024,25,100);
    if (mpu_6050_thread != RT_NULL)
      rt_thread_startup(mpu_6050_thread);
    return RT_EOK;
}
INIT_APP_EXPORT(c8t6_task_init)

#define RAD_TO_DEG 57.3

#define WHO_AM_I_REG 0x75
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV_REG 0x19
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_CONFIG_REG 0x1B
#define GYRO_XOUT_H_REG 0x43

// Setup MPU6050
#define MPU6050_ADDR 0x68
const double Accel_Z_corrector = 14418.0;

uint32_t timer;

Kalman_t KalmanX = {
        .Q_angle = 0.001f,
        .Q_bias = 0.003f,
        .R_measure = 0.03f};

Kalman_t KalmanY = {
        .Q_angle = 0.001f,
        .Q_bias = 0.003f,
        .R_measure = 0.03f,
};

uint8_t MPU6050_Init(struct rt_i2c_bus_device *I2Cx)
{
    uint8_t check;
    uint8_t Data; // data buffer

    // check device ID WHO_AM_I
    i2c_read_reg(I2Cx, MPU6050_ADDR,WHO_AM_I_REG,&check);
    //HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, i2c_timeout);

    if (check == 0x68) // 0x68 will be returned by the sensor if everything goes well
    {    Data =0x80;
        i2c_write_reg(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG,Data);
        // power management register 0X6B we should write all 0's to wake the sensor up
        rt_thread_mdelay(100);
        Data =0x00;
        i2c_write_reg(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG,Data);
        //HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, i2c_timeout);
        // Set DATA RATE of 1KHz by writing SMPLRT_DIV register
        // Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
        // Gyroscope Output Rate = 8kHz when the DLPF is disabled, and 1kHz when the DLPF is enabled.
        Data = 0x07;
        i2c_write_reg(I2Cx, MPU6050_ADDR, SMPLRT_DIV_REG, Data);
        //HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, i2c_timeout);

        // Set accelerometer configuration in ACCEL_CONFIG Register
        // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> +- 2g
        Data = 0x00;
        i2c_write_reg(I2Cx, MPU6050_ADDR, ACCEL_CONFIG_REG, Data);
       // HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, i2c_timeout);

        // Set Gyroscopic configuration in GYRO_CONFIG Register
        // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> +- 250 degree/s
        Data = 0x00;
        i2c_write_reg(I2Cx, MPU6050_ADDR,  GYRO_CONFIG_REG, Data);
        //HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, i2c_timeout);
        return 0;
    }
    return 1;
}

void MPU6050_Read_Accel(struct rt_i2c_bus_device *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[6];
    for (int i = 0; i < 6; ++i) {
        // Read 6 BYTES of data starting from ACCEL_XOUT_H register
        i2c_read_reg(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG+i, &Rec_Data[i]);
        //HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, i2c_timeout);
    }
    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    /*** convert the RAW values into acceleration in 'g'
         we have to divide according to the Full scale value set in FS_SEL
         I have configured FS_SEL = 0. So I am dividing by 16384.0
         for more details check ACCEL_CONFIG Register              ****/

    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
    DataStruct->Az = DataStruct->Accel_Z_RAW / Accel_Z_corrector;
//    DataStruct->Az = DataStruct->Accel_Z_RAW / 16384.0;
}

void MPU6050_Read_Gyro(struct rt_i2c_bus_device *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[6];
    for (int i = 0; i < 6; ++i) {
        // Read 6 BYTES of data starting from GYRO_XOUT_H register
        i2c_read_reg(I2Cx, MPU6050_ADDR, GYRO_XOUT_H_REG+i, &Rec_Data[i]);
        //HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, i2c_timeout);
    }
    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    /*** convert the RAW values into dps (�/s)
         we have to divide according to the Full scale value set in FS_SEL
         I have configured FS_SEL = 0. So I am dividing by 131.0
         for more details check GYRO_CONFIG Register              ****/

    DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0;
}

void MPU6050_Read_Temp(struct rt_i2c_bus_device *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[2];
    int16_t temp;
    for (int i = 0; i < 2; ++i) {
        // Read 2 BYTES of data starting from TEMP_OUT_H_REG register
        i2c_read_reg(I2Cx, MPU6050_ADDR, TEMP_OUT_H_REG + i, &Rec_Data[i]);
        // HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, TEMP_OUT_H_REG, 1, Rec_Data, 2, i2c_timeout);
    }
    temp = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);
}

void MPU6050_Read_All(struct rt_i2c_bus_device *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[14];
    int16_t temp;

    // Read 14 BYTES of data starting from ACCEL_XOUT_H register
    // Accel and gyro's x, y and z data is seriate, so just read 14 bytes for the first register
    for (int i = 0; i < 14; ++i) {
        i2c_read_reg(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG+i, &Rec_Data[i]);
        // HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 14, i2c_timeout);
    }
    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
    temp = (int16_t)(Rec_Data[6] << 8 | Rec_Data[7]);
    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);

    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0; // unit: g
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
    DataStruct->Az = DataStruct->Accel_Z_RAW / Accel_Z_corrector;
    DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);
    DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0;

    // Kalman angle solve
    double dt = (double)(rt_tick_get() - timer) / 1000;
    timer = rt_tick_get();
    double roll;
    double roll_sqrt = sqrt(
            DataStruct->Accel_X_RAW * DataStruct->Accel_X_RAW + DataStruct->Accel_Z_RAW * DataStruct->Accel_Z_RAW);
    if (roll_sqrt != 0.0)
    {
        roll = atan(DataStruct->Accel_Y_RAW / roll_sqrt) * RAD_TO_DEG;
    }
    else
    {
        roll = 0.0;
    }
    double pitch = atan2(-DataStruct->Accel_X_RAW, DataStruct->Accel_Z_RAW) * RAD_TO_DEG;
    if ((pitch < -90 && DataStruct->KalmanAngleY > 90) || (pitch > 90 && DataStruct->KalmanAngleY < -90))
    {
        KalmanY.angle = pitch;
        DataStruct->KalmanAngleY = pitch;
    }
    else
    {
        DataStruct->KalmanAngleY = Kalman_getAngle(&KalmanY, pitch, DataStruct->Gy, dt);
    }
    if (fabs(DataStruct->KalmanAngleY) > 90)
        DataStruct->Gx = -DataStruct->Gx;
    DataStruct->KalmanAngleX = Kalman_getAngle(&KalmanX, roll, DataStruct->Gx, dt);
}

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt)
{
    double rate = newRate - Kalman->bias;
    Kalman->angle += dt * rate;

    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    double S = Kalman->P[0][0] + Kalman->R_measure;
    double K[2];
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;

    double y = newAngle - Kalman->angle;
    Kalman->angle += K[0] * y;
    Kalman->bias += K[1] * y;

    double P00_temp = Kalman->P[0][0];
    double P01_temp = Kalman->P[0][1];

    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;

    return Kalman->angle;
};

rt_err_t i2c_read_regs(struct rt_i2c_bus_device *bus, uint16_t slave_addr, uint8_t reg, uint8_t* buffer, uint16_t count)
{
    rt_size_t ret;
    struct rt_i2c_msg msgs[2];

    msgs[0].addr = slave_addr;
    msgs[0].flags = RT_I2C_WR | bus->flags;
    msgs[0].buf = &reg;
    msgs[0].len = 1;

    msgs[1].addr = slave_addr;
    msgs[1].flags = RT_I2C_RD | bus->flags;
    msgs[1].buf = buffer;
    msgs[1].len = count;

    ret = rt_i2c_transfer(bus, msgs, 2);

    return ret == 2 ? RT_EOK : RT_ERROR;
}

rt_err_t i2c_write_regs(struct rt_i2c_bus_device *bus, uint16_t slave_addr, uint8_t reg, uint8_t* vals, uint16_t count) {
    rt_size_t ret;
    struct rt_i2c_msg msgs[2];

    msgs[0].addr = slave_addr;
    msgs[0].flags = RT_I2C_WR | bus->flags;
    msgs[0].buf = &reg;
    msgs[0].len = 1;

    msgs[1].addr = slave_addr;
    msgs[1].flags = RT_I2C_WR | bus->flags;
    msgs[1].buf = vals;
    msgs[1].len = count;

    ret = rt_i2c_transfer(bus, msgs, 2);

    return ret == 2 ? RT_EOK : RT_ERROR;

}
    rt_err_t i2c_read_reg(struct rt_i2c_bus_device *bus, uint16_t slave_addr, uint8_t reg, uint8_t* buffer)
    {
        rt_size_t ret;
        struct rt_i2c_msg msgs[2];

        msgs[0].addr = slave_addr;
        msgs[0].flags = RT_I2C_WR | bus->flags;
        msgs[0].buf = &reg;
        msgs[0].len = 1;

        msgs[1].addr = slave_addr;
        msgs[1].flags = RT_I2C_RD | bus->flags;
        msgs[1].buf = buffer;
        msgs[1].len = 1;

        ret = rt_i2c_transfer(bus, msgs, 2);

        return ret == 2 ? RT_EOK : RT_ERROR;
    }

    rt_err_t i2c_write_reg(struct rt_i2c_bus_device *bus, uint16_t slave_addr, uint8_t reg, uint8_t val)
    {
        rt_size_t ret;
        rt_uint8_t buffer[2];
        struct rt_i2c_msg msgs;

        buffer[0] = reg;
        buffer[1] = val;

        msgs.addr = slave_addr;
        msgs.flags = RT_I2C_WR | bus->flags;
        msgs.buf = buffer;
        msgs.len = 2;

        ret = rt_i2c_transfer(bus, &msgs, 1);

        return ret == 1 ? RT_EOK : RT_ERROR;
    }

