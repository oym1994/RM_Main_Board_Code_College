/**
  ********************************(C) COPYRIGHT 2017 DJI************************
  * @file       mpu.c
	* @brief      mpu6050/mpu6500/mpu9250 module driver.
  *             Configuration MPU6500 or MPU 9250 and Read the Accelerator
	*             and Gyrometer data using SPI interface
	* @update	  
	*	@history
	* Version     Date          Author           Modification
  * V1.0.0      Nov-19-2015   langgo.chen      
  * @verbatim
	*   
	********************************(C) COPYRIGHT 2017 DJI************************
	*/


#include "mpu.h"
#include "calibrate.h"
#include "ist8310_reg.h" //magenet meter
#include "pid.h"
#include "stm32f4xx_hal.h"
#include <math.h>

#define USE_FREE_RTOS

#ifdef USE_FREE_RTOS
#include "cmsis_os.h"
#define MPU_DELAY(x) osDelay(x)
#else
#define MPU_DELAY(x) HAL_Delay(x)
#endif

#define MPU6500
#define MPU6500_USE_SPI

//6500 and 6050 is the same addr
#define MPU_ADDR 0X68

#if defined(MPU6500) || defined(MPU9250)
#if !defined(MPU6500_USE_IIC) && !defined(MPU6500_USE_SPI)
#error "define a interface MPU6500_USE_IIC/MPU6500_USE_SPI for MPU6500"
#endif
#endif


#if defined(MPU6500) || defined(MPU9250)
#include "mpu6500_reg.h"
#if defined(MPU6500_USE_SPI)
#include "spi.h"
#define MPU_HSPI hspi5
#define MPU_NSS_LOW HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET)
#define MPU_NSS_HIGH HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET)
static u8 tx, rx;
static u8 tx_buff[14] = { 0xff };
#elif defined(MPU6500_USE_IIC)
#include "myiic.h"
#endif
#elif defined(MPU6050)
#include "mpu6050_reg.h"
#include "myiic.h"
#endif

u8             mpu_buff[14];
MPU_OriginData mpu_data;
imu_t          imu={0};

u8 mpu_write_reg(u8 const reg, u8 const data)
{
#if defined(MPU6500) || defined(MPU9250)
#if defined(MPU6500_USE_SPI)
    MPU_NSS_LOW;
    tx = reg & 0x7F;
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
    tx = data;
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
    MPU_NSS_HIGH;
    return 0;
#elif defined(MPU6500_USE_IIC)
    u8 res = IIC_Write_Reg(MPU_ADDR, reg, data);
    return res; //0 if success!
#endif
#elif defined(MPU6050)
    u8 res = IIC_Write_Reg(MPU_ADDR, reg, data);
    return res; //0 if success!
#endif
}

u8 mpu_read_reg(u8 const reg)
{
#if defined(MPU6500) || defined(MPU9250)
#if defined(MPU6500_USE_SPI)
    MPU_NSS_LOW;
    tx = reg | 0x80;

    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
    MPU_NSS_HIGH;
    return rx;
#elif defined(MPU6500_USE_IIC)
    u8 res = IIC_Read_Reg(MPU_ADDR, reg);
    return res;
#endif

#elif defined(MPU6050)
    u8 res = IIC_Read_Reg(MPU_ADDR, reg);
    return res;
#endif
}

u8 mpu_read_regs(u8 const regAddr, u8* pData, u8 len)
{
#if defined(MPU6500) || defined(MPU9250)
#if defined(MPU6500_USE_SPI)
    MPU_NSS_LOW;
    tx         = regAddr | 0x80;
    tx_buff[0] = tx;
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
    HAL_SPI_TransmitReceive(&MPU_HSPI, tx_buff, pData, len, 55);
    MPU_NSS_HIGH;
    return 0;
#elif defined(MPU6500_USE_IIC)
    u8 res = IIC_Read_Bytes(MPU_ADDR, regAddr, pData, len);
    return res;
#else
#error "define a interface spi or iic for 6500!!!"
#endif

#elif defined(MPU6050)

    u8 res = IIC_Read_Bytes(MPU_ADDR, regAddr, pData, len);
    return res;
#endif
}

/**
	* @fn       ist_reg_write_by_mpu
	* @brief    Write IST8310 register through MPU6500's I2C Master
	* @param    Register address¡êoreg_address, Register content: reg_data
	* @retval   void
	* @note     ist8310_init need to be called before.
	*/
static void ist_reg_write_by_mpu(u8 addr, u8 data)
{
    //turn off slave 1 at first
    mpu_write_reg(MPU6500_I2C_SLV1_CTRL, 0x00);
    MPU_DELAY(2);
    mpu_write_reg(MPU6500_I2C_SLV1_REG, addr);
    MPU_DELAY(2);
    mpu_write_reg(MPU6500_I2C_SLV1_DO, data);
    MPU_DELAY(2);
    //turn on slave 1 with one byte transmitting
    mpu_write_reg(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
    //wait longer to ensure the data is transmitted from slave 1
    MPU_DELAY(10);
}

/**
	* @fn       ist_reg_read_by_mpu
	* @brief    Write IST8310 register through MPU6500's I2C Master
	* @param    Register address¡êoreg_address
	* @retval   void
	* @note     ist8310_init need to be called before.
	*/
static u8 ist_reg_read_by_mpu(u8 addr)
{
    u8 retval;
    mpu_write_reg(MPU6500_I2C_SLV4_REG, addr);
    MPU_DELAY(10);
    mpu_write_reg(MPU6500_I2C_SLV4_CTRL, 0x80);
    MPU_DELAY(10);
    retval = mpu_read_reg(MPU6500_I2C_SLV4_DI);
    //turn off slave4 after read
    mpu_write_reg(MPU6500_I2C_SLV4_CTRL, 0x00);
    MPU_DELAY(10);
    return retval;
}

/**
	* @fn       mpu_mst_i2c_auto_read_config
	* @brief    Initialize the MPU6500 I2C Slave 0 for I2C reading.
	* @param    Slave device address, Address[6:0]
	* @retval   void
	* @note     
	*/
static void mpu_mst_i2c_auto_read_config(uint8_t device_address, uint8_t reg_base_addr, uint8_t data_num)
{
    //configure the device address of the IST8310
    //use slave1,auto transmit single measure mode.
    mpu_write_reg(MPU6500_I2C_SLV1_ADDR, device_address);
    MPU_DELAY(2);
    mpu_write_reg(MPU6500_I2C_SLV1_REG, IST8310_R_CONFA);
    MPU_DELAY(2);
    mpu_write_reg(MPU6500_I2C_SLV1_DO, IST8310_ODR_MODE);
    MPU_DELAY(2);

    //use slave0,auto read data
    mpu_write_reg(MPU6500_I2C_SLV0_ADDR, 0x80 | device_address);
    MPU_DELAY(2);
    mpu_write_reg(MPU6500_I2C_SLV0_REG, reg_base_addr);
    MPU_DELAY(2);

    //every eight mpu6500 internal samples one i2c master read
    mpu_write_reg(MPU6500_I2C_SLV4_CTRL, 0x03);
    MPU_DELAY(2);
    //enable slave 0 and 1 access delay
    mpu_write_reg(MPU6500_I2C_MST_DELAY_CTRL, 0x01 | 0x02);
    MPU_DELAY(2);
    //enable slave 1 auto transmit
    mpu_write_reg(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
    MPU_DELAY(6); //Wait 6ms (minimum waiting time for 16 times internal average setup)
    //enable slave 0 with data_num bytes reading
    mpu_write_reg(MPU6500_I2C_SLV0_CTRL, 0x80 | data_num);
    MPU_DELAY(2);
}

u8 ist8310_init()
{
    mpu_write_reg(MPU6500_USER_CTRL, 0x30); //enable iic master mode
    MPU_DELAY(10);
    mpu_write_reg(MPU6500_I2C_MST_CTRL, 0x0d); //enable iic 400khz
    MPU_DELAY(10);

    //turn on slave 1 for ist write and slave 4 to ist read
    mpu_write_reg(MPU6500_I2C_SLV1_ADDR, IST8310_ADDRESS); //enable iic 400khz
    MPU_DELAY(10);
    mpu_write_reg(MPU6500_I2C_SLV4_ADDR, 0x80 | IST8310_ADDRESS); //enable iic 400khz
    MPU_DELAY(10);

    //IST8310_R_CONFB 0x01	= device rst
    ist_reg_write_by_mpu(IST8310_R_CONFB, 0x01); //soft rst
    MPU_DELAY(10);
    if (IST8310_DEVICE_ID_A != ist_reg_read_by_mpu(IST8310_WHO_AM_I))
        return 1; //wrong

    ist_reg_write_by_mpu(IST8310_R_CONFB, 0x01); //rst
    MPU_DELAY(10);

    ist_reg_write_by_mpu(IST8310_R_CONFA, 0x00); //config as ready mode to access reg
    if (ist_reg_read_by_mpu(IST8310_R_CONFA) != 0x00)
        return 2;
    MPU_DELAY(10);

    ist_reg_write_by_mpu(IST8310_R_CONFB, 0x00); //normal state, no int
    if (ist_reg_read_by_mpu(IST8310_R_CONFB) != 0x00)
        return 3;
    MPU_DELAY(10);
    //config  low noise mode, x,y,z axis 16 time 1 avg,
    ist_reg_write_by_mpu(IST8310_AVGCNTL, 0x24); //100100
    if (ist_reg_read_by_mpu(IST8310_AVGCNTL) != 0x24)
        return 4;
    MPU_DELAY(10);

    //Set/Reset pulse duration setup,normal mode
    ist_reg_write_by_mpu(IST8310_PDCNTL, 0xc0);
    if (ist_reg_read_by_mpu(IST8310_PDCNTL) != 0xc0)
        return 5;
    MPU_DELAY(10);

    //turn off slave1 & slave 4
    mpu_write_reg(MPU6500_I2C_SLV1_CTRL, 0x00);
    MPU_DELAY(10);
    mpu_write_reg(MPU6500_I2C_SLV4_CTRL, 0x00);
    MPU_DELAY(10);

    //configure and turn on slave 0
    mpu_mst_i2c_auto_read_config(IST8310_ADDRESS, IST8310_R_XL, 0x06);
    MPU_DELAY(100);
    return 0;
}

u8    ist_buff[6];
float test_yaw1, test_yaw2;
void ist8310_get_data(u8* buff)
{
    mpu_read_regs(MPU6500_EXT_SENS_DATA_00, buff, 6); //¶ÁÈ¡µ½´ÅÁ¦¼ÆÊý¾Ý
}

void mpu_get_data()
{

#if defined(MPU6500) || defined(MPU9250)
    mpu_read_regs(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);
#elif defined(MPU6050)
    mpu_read_regs(MPU_ACCEL_XOUTH_REG, mpu_buff, 14);
#endif

    mpu_data.ax   = mpu_buff[0] << 8 | mpu_buff[1];
    mpu_data.ay   = mpu_buff[2] << 8 | mpu_buff[3];
    mpu_data.az   = mpu_buff[4] << 8 | mpu_buff[5];
    mpu_data.temp = mpu_buff[6] << 8 | mpu_buff[7];
    //	if(gAppParam.ImuCaliList[CALI_GYRO].isAlreadyCalied == 0x55){
	  mpu_data.gx = ((mpu_buff[8] << 8 | mpu_buff[9])   - mpu_data.gx_offset);
    mpu_data.gy = ((mpu_buff[10] << 8 | mpu_buff[11]) - mpu_data.gy_offset);
    mpu_data.gz = ((mpu_buff[12] << 8 | mpu_buff[13]) - mpu_data.gz_offset);
    //	}
    ist8310_get_data(ist_buff);
    memcpy(&mpu_data.mx, ist_buff, 6);

    memcpy(&imu.ax, &mpu_data.ax, 6 * sizeof(s16));
    imu.temp = 21 + mpu_data.temp / 333.87f;
		imu.wx   = mpu_data.gx / 16.384f / 57.3f; //2000dps -> rad/s
    imu.wy   = mpu_data.gy / 16.384f / 57.3f; //2000dps -> rad/s
    imu.wz   = mpu_data.gz / 16.384f / 57.3f; //2000dps -> rad/s
    /*
		imu.wx   = (mpu_data.gx-mpu_data.gx_offset) / 16.384f / 57.3f; //2000dps -> rad/s
    imu.wy   = (mpu_data.gy-mpu_data.gy_offset) / 16.384f / 57.3f; //2000dps -> rad/s
    imu.wz   = (mpu_data.gz-mpu_data.gz_offset) / 16.384f / 57.3f; //2000dps -> rad/s
		*/
		/*
		imu.vx   += (mpu_data.ax -	mpu_data.ax_offset) / 4096.*100.*0.005; //2000dps -> rad/s
    imu.vy   += (mpu_data.ay - mpu_data.ay_offset) / 4096.*100.*0.005; //2000dps -> rad/s
    imu.vz   += (mpu_data.az - mpu_data.az_offset) /4096.*100.*0.005; //2000dps -> rad/s
		*/
    test_yaw1 = 57.3f * atan2f(mpu_data.my, mpu_data.mx);

    imuCaliHook(CALI_GYRO, &mpu_data.gx);
    imuCaliHook(CALI_ACC, &mpu_data.ax);
    imuCaliHook(CALI_MAG, &mpu_data.mx);
}

/**
	* @brief    set imu 6050/6500 gyroscope measure range
	* @param    fsr: range
	* @note     fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps   
	*/
u8 MPU_Set_Gyro_Fsr(u8 fsr)
{
#if defined(MPU6050)
    return mpu_write_reg(MPU_GYRO_CFG_REG, fsr << 3);
#elif defined(MPU6500) || defined(MPU9250)
    return mpu_write_reg(MPU6500_GYRO_CONFIG, fsr << 3);

#endif
}
/**
	* @brief    set imu 6050/6500 accelerate measure range
	* @param    fsr: range
	* @note     fsr:0,±2g;1,±4g;2,±8g;3,±16g
	*/
u8 MPU_Set_Accel_Fsr(u8 fsr)
{
#if defined(MPU6050)
    return mpu_write_reg(MPU_ACCEL_CFG_REG, fsr << 3); //?????????????
#elif defined(MPU6500) || defined(MPU9250)
    return mpu_write_reg(MPU6500_ACCEL_CONFIG, fsr << 3); //?????????????
#endif
}
/**
	* @brief    set MPU6050 digital low pass filter
	* @param    lpf: frequency, unit is Hz
	*/
u8 MPU_Set_Gyro_LPF(u16 lpf)
{
#if defined(MPU6050)
    u8 data = 0;
    if (lpf >= 188)
        data = 1;
    else if (lpf >= 98)
        data = 2;
    else if (lpf >= 42)
        data = 3;
    else if (lpf >= 20)
        data = 4;
    else if (lpf >= 10)
        data = 5;
    else
        data = 6;
    return mpu_write_reg(MPU_CFG_REG, data); //?????????
#elif defined(MPU6500) || defined(MPU9250)
    u8 data = 0;
    if (lpf > 250)
        data = 7; //fcut  = 3600hz
    else if (lpf >= 184)
        data = 0; //fcut = 250hz
    else if (lpf >= 92)
        data = 1;
    else if (lpf >= 41)
        data = 2;
    else if (lpf >= 20)
        data = 3;
    else if (lpf >= 10)
        data = 4;
    else if (lpf >= 5)
        data = 5;
    else
        data = 6;

    return mpu_write_reg(MPU6500_CONFIG, data); //?????????
#endif
}


#if defined(MPU6050)
short MPU_Get_Temperature(void)
{
    u8    buf[2];
    short raw;
    float temp;
    IIC_Read_Bytes(MPU_ADDR, MPU_TEMP_OUTH_REG, buf, 2);
    raw  = ((u16)buf[0] << 8) | buf[1];
    temp = 36.53 + ((double)raw) / 340;
    return temp * 100;
}
#endif

u8 id;
/**
  * @brief     initialize imu mpu6500 and magnet meter ist3810
  * @attention before gimbal control loop in gimbal_task() function
  */
u8 mpu_device_init(void)
{
#if defined(MPU6500) || defined(MPU9250)
#if defined(MPU6500_USE_IIC)
    IIC_Init();
#endif

    MPU_DELAY(100);

    id                          = mpu_read_reg(MPU6500_WHO_AM_I);
    u8 i                        = 0;
    u8 MPU6500_Init_Data[10][2] = {
        { MPU6500_PWR_MGMT_1, 0x80 }, // Reset Device
        { MPU6500_PWR_MGMT_1, 0x03 }, // Clock Source - Gyro-Z
        { MPU6500_PWR_MGMT_2, 0x00 }, // Enable Acc & Gyro
        { MPU6500_CONFIG, 0x04 }, // LPF 41Hz//gyro bandwidth 41Hz
        { MPU6500_GYRO_CONFIG, 0x18 }, // +-2000dps
        { MPU6500_ACCEL_CONFIG, 0x10 }, // +-8G
        { MPU6500_ACCEL_CONFIG_2, 0x02 }, // enable LowPassFilter  Set Acc LPF
        { MPU6500_USER_CTRL, 0x20 }, // Enable AUX
    };
    for (i = 0; i < 10; i++)
    {
        mpu_write_reg(MPU6500_Init_Data[i][0], MPU6500_Init_Data[i][1]);
        MPU_DELAY(1);
    }

    MPU_Set_Gyro_Fsr(3); //0=250,1=500,2=1000,3=Ѳ000dps
    MPU_Set_Accel_Fsr(2); //0=Ѳg,1=4g, 2=8g, 3=16g

    ist8310_init();
		mpu_offset_cal();
    return 0;
#elif defined(MPU6050)
    IIC_Init(); 
    id = IIC_Read_Reg(MPU_ADDR, MPU_DEVICE_ID_REG);
    IIC_Write_Reg(MPU_ADDR, MPU_PWR_MGMT1_REG, 0X80); //??MPU6050
    MPU_DELAY(100);
    IIC_Write_Reg(MPU_ADDR, MPU_PWR_MGMT1_REG, 0X00); //??MPU6050
    MPU_Set_Gyro_Fsr(3); //??????,0=250,1=500,2=1000,3=Ѳ000dps
    MPU_Set_Accel_Fsr(2); //??????,0=Ѳg,1=4g, 2=8g, 3=16g
    IIC_Write_Reg(MPU_ADDR, MPU_INT_EN_REG, 0X00); //??????
    IIC_Write_Reg(MPU_ADDR, MPU_USER_CTRL_REG, 0X00); //I2C?????
    IIC_Write_Reg(MPU_ADDR, MPU_FIFO_EN_REG, 0X00); //??FIFO
    IIC_Write_Reg(MPU_ADDR, MPU_INTBP_CFG_REG, 0X80); //INT???????
    if (id == MPU6050_ID) //??ID??
    {
        IIC_Write_Reg(MPU_ADDR, MPU_PWR_MGMT1_REG, 0X01); //??CLKSEL,PLL X????
        IIC_Write_Reg(MPU_ADDR, MPU_PWR_MGMT2_REG, 0X00); //??????????
    }
    else
        return 1;
		//mpu_offset_cal();
    return 0;
#endif

}

void mpu_offset_cal(void){
	int i;
	for (i=0; i<300;i++){
	#if defined(MPU6500) || defined(MPU9250)
			mpu_read_regs(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);
	#elif defined(MPU6050)
			mpu_read_regs(MPU_ACCEL_XOUTH_REG, mpu_buff, 14);
	#endif

			mpu_data.ax_offset   += mpu_buff[0] << 8 | mpu_buff[1];
			mpu_data.ay_offset   += mpu_buff[2] << 8 | mpu_buff[3];
			mpu_data.az_offset   += mpu_buff[4] << 8 | mpu_buff[5];
			//	if(gAppParam.ImuCaliList[CALI_GYRO].isAlreadyCalied == 0x55){
			mpu_data.gx_offset += mpu_buff[8] << 8 | mpu_buff[9];
			mpu_data.gy_offset += mpu_buff[10] << 8 | mpu_buff[11];
			mpu_data.gz_offset += mpu_buff[12] << 8 | mpu_buff[13];
			//	}
			osDelay(5);
		}
	mpu_data.ax_offset=mpu_data.ax_offset / 300; 
	mpu_data.ay_offset=mpu_data.ay_offset / 300; 
	mpu_data.az_offset=mpu_data.az_offset / 300; 
	mpu_data.gx_offset=mpu_data.gx_offset	/ 300;
	mpu_data.gy_offset=mpu_data.gx_offset	/ 300;
	mpu_data.gz_offset=mpu_data.gz_offset	/ 300;	
}

