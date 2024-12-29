/**
  ******************************************************************************
  * @file    scha63x.c
  * @author  muRata
  * @version
  * @date
  * @brief   SCHA63X处理函数
  ******************************************************************************
**/

/*----------------------------------------------------------------------------

    Driver for SCHA63X sensor.

 *----------------------------------------------------------------------------*/


#include "scha63x.h"


/*----------------------------------------------------------------------------
  Define product type and the sensitivities
 *----------------------------------------------------------------------------*/

#define SENSITIVITY_ACC 4905  //设置加速度计灵敏度为4905:输出信号变化与输入角度速度之间的比值;

#define SCHA63X_X03 // Use SCHA634-D03 or SCHA63T-K03

#ifdef SCHA63X_X01		//如果定义这个宏，则执行以下代码
#define SENSITIVITY_GYRO_X 160		//设置陀螺仪X轴灵敏度为160，其他同理
#define SENSITIVITY_GYRO_Y 160
#define SENSITIVITY_GYRO_Z 160
#elif defined SCHA634_D02		//如果定义这个宏，则执行以下代码
#define SENSITIVITY_GYRO_X 160
#define SENSITIVITY_GYRO_Y 160
#define SENSITIVITY_GYRO_Z 80
#elif defined SCHA63X_X03		//如果定义这个宏，则执行以下代码
#define SENSITIVITY_GYRO_X 80
#define SENSITIVITY_GYRO_Y 80
#define SENSITIVITY_GYRO_Z 80
#endif

// Macros for parsing values from sensor MISO words（从传感器MISO字中解析值）
#define SPI_DATA_INT8_UPPER(a)      ((int8_t)(((a) >> 16) & 0xff))	//解析出8位的上部数据
#define SPI_DATA_INT8_LOWER(a)      ((int8_t)(((a) >> 8) & 0xff))	//解析出8位的下部数据
#define SPI_DATA_INT16(a)           ((int16_t)(((a) >> 8) & 0xffff))	//解析出16位的有符号整数
#define SPI_DATA_UINT16(a)          ((uint16_t)(((a) >> 8) & 0xffff))	//解析出16位的无符号整数
#define SPI_DATA_CHECK_RS_ERROR(a)  ((((a) >> 24) & 0x03) != 1 ? true : false) // true = RS error
#define GET_TEMPERATURE(a)          (25 + ((a) / 30.0))		//从传感器数据中获取温度值


// Pre-calculated（预先计算）SPI frames（spi协议中的数据帧）for various operations（为了支持多种不同的操作）
//
#define SPI_FRAME_READ_GYRO_X           0x040000F7	//读取陀螺仪x轴数据的spi帧
#define SPI_FRAME_READ_GYRO_Y           0x0C0000FB	//读取陀螺仪y轴数据的spi帧
#define SPI_FRAME_READ_GYRO_Z           0x040000F7	//读取陀螺仪z轴数据的spi帧
#define SPI_FRAME_READ_ACC_X            0x100000E9	//读取加速度计x轴数据的spi帧
#define SPI_FRAME_READ_ACC_Y            0x140000EF	//读取加速度计y轴数据的spi帧
#define SPI_FRAME_READ_ACC_Z            0x180000E5	//读取加速度计z轴数据的spi帧
#define SPI_FRAME_READ_TEMP             0x1C0000E3	//读取温度数据的spi帧

#define SPI_FRAME_READ_SUMMARY_STATUS   0x380000D5	//摘要状态读帧
#define SPI_FRAME_READ_RATE_STATUS_1    0x40000091	//速率状态1
#define SPI_FRAME_READ_RATE_STATUS_2    0x44000097
#define SPI_FRAME_READ_COMMON_STATUS_1  0x50000089	//公共状态1
#define SPI_FRAME_READ_COMMON_STATUS_2  0x5400008F
#define SPI_FRAME_READ_ACC_STATUS_1     0x4800009D	//加速度计状态1

#define SPI_FRAME_WRITE_RESET           0xE000017C	//重置传感器
#define SPI_FRAME_WRITE_REG_BANK_0      0xFC000073 // Select register bank 0（寄存银行）
#define SPI_FRAME_READ_TRC_0            0x740000BF // Traceability 0 register 1Dh（跟踪传感器）
#define SPI_FRAME_READ_TRC_1            0x780000B5 // Traceability 1 register 1Eh
#define SPI_FRAME_READ_TRC_2            0x700000B9 // Traceability 2 Register 1Ch

#define SPI_FRAME_WRITE_OP_MODE_NORMAL   0xE4000067 // Set normal operating mode (register 19h)
#define SPI_FRAME_WRITE_FILTER_46HZ_RATE 0xD812129E // Set 46 Hz filter for rate (register 16h)（设置滤波器速率）
#define SPI_FRAME_WRITE_FILTER_46HZ_ACC  0xE8022248 // Set 46 Hz filter for acceleration (register 1Ah)（为加速度计设置滤波器）
#define SPI_FRAME_WRITE_EOI_BIT          0xE000025B

// Frames needed for test mode activation. Mode register address: 19h
#define SPI_FRAME_READ_MODE             0x640000A7	//读取当前状态
#define SPI_FRAME_WRITE_MODE_ASM_010    0xE40010AA  // Unlock_ASM[2:0] = 010（解锁asm模式）
#define SPI_FRAME_WRITE_MODE_ASM_001    0xE400088F  // Unlock_ASM[2:0] = 001
#define SPI_FRAME_WRITE_MODE_ASM_100    0xE40020E0  // Unlock_ASM[2:0] = 100

/**

  Static function prototypes

**/
static bool scha63x_check_init_due(void);	//检查初始化是否到期
static bool scha63x_check_init_uno(void);
static bool scha63x_check_rs_error(uint32_t *data, int size); //检查读取操作中是否存在错误
static bool scha63x_check_rs_error_3(uint32_t miso_1, uint32_t miso_2, uint32_t miso_3);	//检查三个miso数据中是否存在错误的静态函数
static bool scha63x_check_rs_error_5(uint32_t miso_1, uint32_t miso_2, uint32_t miso_3,
                                     uint32_t miso_4, uint32_t miso_5);		//检查五个miso数据中是否存在错误的静态函数


/**

  Internal data structures

**/

bool DueErrorAvailable = false;
bool UnoErrorAvailable = false;


// Sampling_callback stores summed data here.
// Use ring buffer for reducing probability of main() missing some data.


static scha63x_cacv scha63x_cac_values; // Cross-axis compensation values（交叉轴补偿值）


/**

    Init SCHA63X sensor（初始化传感器）

    Parameters:         serial_num - string where to read component serial number. Minimum size 14 bytes.

    Return value:       SCHA63X_OK = success, SCHA63X_ERR_* = failure, SCHA63X_WRN_* = warning. Please see header file for definitions.

**/
int scha63x_init(char *serial_num)
{
    bool status_DUE = false;
    bool status_UNO = false;
    int attempt;
    int const num_attempts = 2;

    // HW Reset (reset via SPI)
    SPI_ASIC_UNO(SPI_FRAME_WRITE_RESET);	//通过spi发送重置命令
    SPI_ASIC_DUE(SPI_FRAME_WRITE_REG_BANK_0);   // Make sure we are in bank 0, otherwise SPI reset is not available.
    SPI_ASIC_DUE(SPI_FRAME_WRITE_RESET);	//再次发送指令

    //IMU初始化有一些较长的延时，防止被狗咬
    IWDG_ReloadNew(IWDG_PRESCALER_32, 2000);
    // Wait 25 ms for the non-volatile memory (NVM)（非易失性存储器）Read
    HAL_Delay(25);

    // Read UNO asic serial number	//读取UNO ASIC序列号
    SPI_ASIC_UNO(SPI_FRAME_READ_TRC_2);  //发送SPI帧来读取UNO ASIC的跟踪寄存器
    uint16_t trc_2 = SPI_DATA_UINT16(SPI_ASIC_UNO(SPI_FRAME_READ_TRC_0));
    uint16_t trc_0 = SPI_DATA_UINT16(SPI_ASIC_UNO(SPI_FRAME_READ_TRC_1));
    uint16_t trc_1 = SPI_DATA_UINT16(SPI_ASIC_UNO(SPI_FRAME_READ_TRC_1));

    // Build serial number string
		//将读取到的序列号的各个部分（trc_0、trc_1、trc_2）组合成一个字符串，并使用sprintf函数格式化为一个14位的序列号
    uint16_t id_1 = (trc_2 >> 8) & 0x0f;
    uint16_t id_0 = trc_0 & 0xffff;
    uint16_t id_2 = trc_1 & 0xffff;
    snprintf(serial_num, 14, "%05d%01x%04X", id_2, id_1, id_0);

    // Activate DUE asic test mode to be able to read cross-axis
    // compensation values from DUE NVM.
    SPI_ASIC_DUE(SPI_FRAME_WRITE_MODE_ASM_010);
    SPI_ASIC_DUE(SPI_FRAME_READ_MODE);
    SPI_ASIC_DUE(SPI_FRAME_WRITE_MODE_ASM_001);
    SPI_ASIC_DUE(SPI_FRAME_READ_MODE);
    SPI_ASIC_DUE(SPI_FRAME_WRITE_MODE_ASM_100);
    SPI_ASIC_DUE(SPI_FRAME_READ_MODE);
    uint32_t resp = SPI_ASIC_DUE(SPI_FRAME_READ_MODE);

    // Check if device is correctly set to test mode
    if ((SPI_DATA_UINT16(resp) & 0x7) == 7) {

        // Read cross-axis compensation values
        SPI_ASIC_DUE(0xFC00051A);
        SPI_ASIC_DUE(0x2C0000CB);
        uint32_t cxx_cxy = SPI_ASIC_DUE(0x4C00009B);
        uint32_t cxz_cyx = SPI_ASIC_DUE(0x50000089);
        uint32_t cyy_cyz = SPI_ASIC_DUE(0x5400008F);
        uint32_t czx_czy = SPI_ASIC_DUE(0x58000085);
        uint32_t czz_bxx = SPI_ASIC_DUE(0x5C000083);
        uint32_t bxy_bxz = SPI_ASIC_DUE(0x600000A1);
        uint32_t byx_byy = SPI_ASIC_DUE(0x6C0000AB);
        uint32_t byz_bzx = SPI_ASIC_DUE(0x700000B9);
        uint32_t bzy_bzz = SPI_ASIC_DUE(0x700000B9);

        scha63x_cac_values.cxx = SPI_DATA_INT8_LOWER(cxx_cxy) / 4096.0 + 1;
        scha63x_cac_values.cxy = SPI_DATA_INT8_UPPER(cxx_cxy) / 4096.0;
        scha63x_cac_values.cxz = SPI_DATA_INT8_LOWER(cxz_cyx) / 4096.0;
        scha63x_cac_values.cyx = SPI_DATA_INT8_UPPER(cxz_cyx) / 4096.0;
        scha63x_cac_values.cyy = SPI_DATA_INT8_LOWER(cyy_cyz) / 4096.0 + 1;
        scha63x_cac_values.cyz = SPI_DATA_INT8_UPPER(cyy_cyz) / 4096.0;
        scha63x_cac_values.czx = SPI_DATA_INT8_LOWER(czx_czy) / 4096.0;
        scha63x_cac_values.czy = SPI_DATA_INT8_UPPER(czx_czy) / 4096.0;
        scha63x_cac_values.czz = SPI_DATA_INT8_LOWER(czz_bxx) / 4096.0 + 1;
        scha63x_cac_values.bxx = SPI_DATA_INT8_UPPER(czz_bxx) / 4096.0 + 1;
        scha63x_cac_values.bxy = SPI_DATA_INT8_LOWER(bxy_bxz) / 4096.0;
        scha63x_cac_values.bxz = SPI_DATA_INT8_UPPER(bxy_bxz) / 4096.0;
        scha63x_cac_values.byx = SPI_DATA_INT8_LOWER(byx_byy) / 4096.0;
        scha63x_cac_values.byy = SPI_DATA_INT8_UPPER(byx_byy) / 4096.0 + 1;
        scha63x_cac_values.byz = SPI_DATA_INT8_LOWER(byz_bzx) / 4096.0;
        scha63x_cac_values.bzx = SPI_DATA_INT8_UPPER(byz_bzx) / 4096.0;
        scha63x_cac_values.bzy = SPI_DATA_INT8_LOWER(bzy_bzz) / 4096.0;
        scha63x_cac_values.bzz = SPI_DATA_INT8_UPPER(bzy_bzz) / 4096.0 + 1;
    }
    else {
        // Return error if test mode can not be activated
        return SCHA63X_ERR_TEST_MODE_ACTIVATION;
    }

    //IMU初始化有一些较长的延时，防止被狗咬
    IWDG_ReloadNew(IWDG_PRESCALER_32, 2000);
    // HW Reset to get DUE asic out from test mode (reset via SPI).
    SPI_ASIC_DUE(SPI_FRAME_WRITE_REG_BANK_0);   // Return to bank 0 to make SPI reset command available.
    SPI_ASIC_DUE(SPI_FRAME_WRITE_RESET);        // Reset DUE after reading cross-axis registers


    // Start UNO & DUE
    HAL_Delay(25);                                // Wait 25ms for the non-volatile memory (NVM) Read

    // DUE asic initial startup
    SPI_ASIC_UNO(SPI_FRAME_WRITE_OP_MODE_NORMAL);       // Set UNO operation mode on
    SPI_ASIC_DUE(SPI_FRAME_WRITE_OP_MODE_NORMAL);       // Set DUE operation mode on twice
    SPI_ASIC_DUE(SPI_FRAME_WRITE_OP_MODE_NORMAL);
    HAL_Delay(75);                                        // Wait minimum 70ms (includes UNO 50ms 'SPI accessible' wait)

    //IMU初始化有一些较长的延时，防止被狗咬
    IWDG_ReloadNew(IWDG_PRESCALER_32, 2000);
    SPI_ASIC_UNO(SPI_FRAME_WRITE_FILTER_46HZ_RATE);     // Select UNO 46Hz filter for RATE
    SPI_ASIC_UNO(SPI_FRAME_WRITE_FILTER_46HZ_ACC);      // Select UNO 46Hz filter for ACC

    // Restart DUE
    SPI_ASIC_DUE(SPI_FRAME_WRITE_RESET);                // Reset DUE again
    HAL_Delay(25);                                        // Wait 25 ms for the NVM read

    SPI_ASIC_DUE(SPI_FRAME_WRITE_OP_MODE_NORMAL);       // Set DUE operation mode
    SPI_ASIC_DUE(SPI_FRAME_WRITE_OP_MODE_NORMAL);       // DUE operation mode must be set twice

    HAL_Delay(1);                                         // Wait 1 ms for SPI to be accesible
    SPI_ASIC_DUE(SPI_FRAME_WRITE_FILTER_46HZ_RATE);     // Select DUE 46Hz filter for RATE

    for (attempt = 0; attempt < num_attempts; attempt++) {

        //IMU初始化有一些较长的延时，防止被狗咬
        IWDG_ReloadNew(IWDG_PRESCALER_32, 2000);
        // Wait 405 ms (Gyro and ACC start up)
        HAL_Delay(405);

        //IMU初始化有一些较长的延时，防止被狗咬
        IWDG_ReloadNew(IWDG_PRESCALER_32, 2000);

        // Set EOI=1 (End of initialization command)
        SPI_ASIC_UNO(SPI_FRAME_WRITE_EOI_BIT);          // Set EOI bit for UNO
        SPI_ASIC_DUE(SPI_FRAME_WRITE_EOI_BIT);          // Set EOI bit for DUE

        // Check initialization status from the summary status registers
        status_UNO = scha63x_check_init_uno();
        status_DUE = scha63x_check_init_due();

        //IMU初始化有一些较长的延时，防止被狗咬
        IWDG_ReloadNew(IWDG_PRESCALER_32, 2000);
        // Check if restart is needed
        if ((status_UNO == false || status_DUE == false) && attempt < (num_attempts - 1)) {
            SPI_ASIC_UNO(SPI_FRAME_WRITE_RESET);
            SPI_ASIC_DUE(SPI_FRAME_WRITE_RESET);

            HAL_Delay(25);                                   // Wait 25 ms for the NVM read
            SPI_ASIC_UNO(SPI_FRAME_WRITE_OP_MODE_NORMAL);  // Set UNO operation mode on
            SPI_ASIC_DUE(SPI_FRAME_WRITE_OP_MODE_NORMAL);  // Set DUE operation mode on twice
            SPI_ASIC_DUE(SPI_FRAME_WRITE_OP_MODE_NORMAL);
            HAL_Delay(50);                                   // Wait 50ms before communicating with UNO
            SPI_ASIC_UNO(SPI_FRAME_WRITE_FILTER_46HZ_RATE);// Select UNO 46Hz filter for RATE
            SPI_ASIC_UNO(SPI_FRAME_WRITE_FILTER_46HZ_ACC); // Select UNO 46Hz filter for ACC
            SPI_ASIC_DUE(SPI_FRAME_WRITE_FILTER_46HZ_RATE);// Select DUE 46Hz filter for RATE
            HAL_Delay(45);                                   // Adjust restart duration to 500 ms
        }
        else {
            break;
        }
    }

    if (status_DUE == false || status_UNO == false) {
        return SCHA63X_ERR_RS_STATUS_NOK;
    }

    return SCHA63X_OK;

}


/**

    Check initialization success of SCHA63X sensor DUE asic

    Return value:       true = success, false = error

**/
static bool scha63x_check_init_due(void)
{
    uint32_t resp;
    bool due_ok;

    // Read summary status two times (first time may show incorrectly FAIL after start-up)
    SPI_ASIC_DUE(SPI_FRAME_READ_SUMMARY_STATUS);
    SPI_ASIC_DUE(SPI_FRAME_READ_SUMMARY_STATUS);
    HAL_Delay(3);
    resp = SPI_ASIC_DUE(SPI_FRAME_READ_SUMMARY_STATUS);
    due_ok = SPI_DATA_CHECK_RS_ERROR(resp) == true ? false: true;

    return due_ok;
}

/**

    Check initialization success of SCHA63X sensor UNO asic

    Return value:       true = success, false = error

**/
static bool scha63x_check_init_uno(void)
{
    uint32_t resp;
    bool uno_ok;

    // Read summary status two times (first time may show incorrectly FAIL after start-up)
    SPI_ASIC_UNO(SPI_FRAME_READ_SUMMARY_STATUS);
    SPI_ASIC_UNO(SPI_FRAME_READ_SUMMARY_STATUS);
    HAL_Delay(3);
    resp = SPI_ASIC_UNO(SPI_FRAME_READ_SUMMARY_STATUS);
    uno_ok = SPI_DATA_CHECK_RS_ERROR(resp) == true ? false: true;

    return uno_ok;
}


/**

    Read acceleration, rate and temperature data from sensor. Called by sampling_callback()

    Parameters:         *data               - pointer to "raw" data from sensor

    Return value:       None

**/
void scha63x_read_data(scha63x_raw_data *data)//先读，再解析，最后赋值
{

    SPI_ASIC_DUE(SPI_FRAME_READ_GYRO_Y);                         // gyro y
    uint32_t gyro_y_lsb   = SPI_ASIC_DUE(SPI_FRAME_READ_GYRO_Z); // gyro z
    uint32_t gyro_z_lsb   =  SPI_ASIC_DUE(SPI_FRAME_READ_TEMP);  // temperature
    uint32_t temp_due_lsb = SPI_ASIC_DUE(SPI_FRAME_READ_TEMP);

    SPI_ASIC_UNO(SPI_FRAME_READ_GYRO_X);                         // gyro x
    uint32_t gyro_x_lsb   = SPI_ASIC_UNO(SPI_FRAME_READ_ACC_X);  // acc x
    uint32_t acc_x_lsb    = SPI_ASIC_UNO(SPI_FRAME_READ_ACC_Y);  // acc y
    uint32_t acc_y_lsb    = SPI_ASIC_UNO(SPI_FRAME_READ_ACC_Z);  // acc z
    uint32_t acc_z_lsb    = SPI_ASIC_UNO(SPI_FRAME_READ_TEMP);   // temperature
    uint32_t temp_uno_lsb = SPI_ASIC_UNO(SPI_FRAME_READ_TEMP);

    // Get possible errors
    data->rs_error_due = scha63x_check_rs_error_3(gyro_y_lsb, gyro_z_lsb, temp_due_lsb);
    data->rs_error_uno = scha63x_check_rs_error_5(gyro_x_lsb, acc_x_lsb, acc_y_lsb, acc_z_lsb, temp_uno_lsb);

    // Parse MISO data to structure
    data->acc_x_lsb = SPI_DATA_INT16(acc_x_lsb);
    data->acc_y_lsb = SPI_DATA_INT16(acc_y_lsb);
    data->acc_z_lsb = SPI_DATA_INT16(acc_z_lsb);
    data->gyro_x_lsb = SPI_DATA_INT16(gyro_x_lsb);
    data->gyro_y_lsb = SPI_DATA_INT16(gyro_y_lsb);
    data->gyro_z_lsb = SPI_DATA_INT16(gyro_z_lsb);
    data->temp_due_lsb = SPI_DATA_INT16(temp_due_lsb);
    data->temp_uno_lsb = SPI_DATA_INT16(temp_uno_lsb);

}


/**

    Check if MISO frames have RS error bits set

    Parameters:         *data       - pointer to 32-bit MISO frames from sensor
                        size        - number of frames to check

    Return value:       true  = RS error bits set
                        false = no RS error

**/
static bool scha63x_check_rs_error(uint32_t *data, int size)
{
    for (int i = 0; i < size; i++)
    {
        uint32_t value = data[i];
        if (SPI_DATA_CHECK_RS_ERROR(value))
        {
            return true;
        }
    }

    return false;
}


/**

    Check if any of three MISO frames have RS error bits set

    Parameters:         miso_1 .. miso_3       - MISO words 1 .. 3

    Return value:       true  = RS error bits set
                        false = no RS error

**/
static bool scha63x_check_rs_error_3(uint32_t miso_1, uint32_t miso_2, uint32_t miso_3)
{
    uint32_t miso_words[] = { miso_1, miso_2, miso_3 };
    return scha63x_check_rs_error(miso_words, (sizeof(miso_words) / sizeof(uint32_t)));
}


/**

    Check if any of five MISO frames have RS error bits set

    Parameters:         miso_1 .. miso_5       - MISO words 1 .. 5

    Return value:       true  = RS error bits set
                        false = no RS error

**/
static bool scha63x_check_rs_error_5(uint32_t miso_1, uint32_t miso_2, uint32_t miso_3,
                                     uint32_t miso_4, uint32_t miso_5)
{
    uint32_t miso_words[] = { miso_1, miso_2, miso_3, miso_4, miso_5 };
    return scha63x_check_rs_error(miso_words, (sizeof(miso_words) / sizeof(uint32_t)));
}


/**

    Convert summed raw binary data from sensor to real values. Also calculate averages values.

    Parameters:         *data_in    - pointer to summed "raw" data from sensor
                        *data_out   - pointer to converted values

    Return value:       None

*/
void scha63x_convert_data(scha63x_raw_data_summed *data_in, scha63x_real_data *data_out)
{
    data_out->acc_x = data_in->acc_x_lsb;
    data_out->acc_y = data_in->acc_y_lsb;
    data_out->acc_z = data_in->acc_z_lsb;
    data_out->gyro_x = data_in->gyro_x_lsb;
    data_out->gyro_y = data_in->gyro_y_lsb;
    data_out->gyro_z = data_in->gyro_z_lsb;

    // Convert from LSB to sensitivity and calculate averages here for faster execution
    data_out->acc_x = data_out->acc_x / (SENSITIVITY_ACC * SAMPLE_COUNT);
    data_out->acc_y = data_out->acc_y / (SENSITIVITY_ACC * SAMPLE_COUNT);
    data_out->acc_z = data_out->acc_z / (SENSITIVITY_ACC * SAMPLE_COUNT);
    data_out->gyro_x = data_out->gyro_x / (SENSITIVITY_GYRO_X * SAMPLE_COUNT);
    data_out->gyro_y = data_out->gyro_y / (SENSITIVITY_GYRO_Y * SAMPLE_COUNT);
    data_out->gyro_z = data_out->gyro_z / (SENSITIVITY_GYRO_Z * SAMPLE_COUNT);

    // Convert temperatures and calculate averages
    data_out->temp_due = GET_TEMPERATURE(data_in->temp_due_lsb / SAMPLE_COUNT);
    data_out->temp_uno = GET_TEMPERATURE(data_in->temp_uno_lsb / SAMPLE_COUNT);
}


/**

    Calculate cross-axis compensation

    Parameters:         *data   - pointer to sensor data

    Return value:       None

**/
void scha63x_cross_axis_compensation(scha63x_real_data *data)
{
    float acc_x_comp, acc_y_comp, acc_z_comp;
    float gyro_x_comp, gyro_y_comp, gyro_z_comp;

		//矩阵运算
    acc_x_comp = (scha63x_cac_values.bxx * data->acc_x) + (scha63x_cac_values.bxy * data->acc_y) + (scha63x_cac_values.bxz * data->acc_z);
    acc_y_comp = (scha63x_cac_values.byx * data->acc_x) + (scha63x_cac_values.byy * data->acc_y) + (scha63x_cac_values.byz * data->acc_z);
    acc_z_comp = (scha63x_cac_values.bzx * data->acc_x) + (scha63x_cac_values.bzy * data->acc_y) + (scha63x_cac_values.bzz * data->acc_z);
    gyro_x_comp = (scha63x_cac_values.cxx * data->gyro_x) + (scha63x_cac_values.cxy * data->gyro_y) + (scha63x_cac_values.cxz * data->gyro_z);
    gyro_y_comp = (scha63x_cac_values.cyx * data->gyro_x) + (scha63x_cac_values.cyy * data->gyro_y) + (scha63x_cac_values.cyz * data->gyro_z);
    gyro_z_comp = (scha63x_cac_values.czx * data->gyro_x) + (scha63x_cac_values.czy * data->gyro_y) + (scha63x_cac_values.czz * data->gyro_z);

    data->acc_x = acc_x_comp;
    data->acc_y = acc_y_comp;
    data->acc_z = acc_z_comp;
    data->gyro_x = gyro_x_comp;
    data->gyro_y = gyro_y_comp;
    data->gyro_z = gyro_z_comp;
}


/**

    Read sensor status from UNO ASIC

    Parameters:         *status    - pointer to sensor status structure where the data is read to

    Return value:       None

**/
void scha63x_read_sensor_status_uno(scha63x_sensor_status *status)
{

    SPI_ASIC_UNO(SPI_FRAME_READ_SUMMARY_STATUS);
    uint32_t sum_stat   = SPI_ASIC_UNO(SPI_FRAME_READ_RATE_STATUS_1);
    uint32_t rat_stat   = SPI_ASIC_UNO(SPI_FRAME_READ_ACC_STATUS_1);
    uint32_t acc_stat   = SPI_ASIC_UNO(SPI_FRAME_READ_COMMON_STATUS_1);
    uint32_t com_stat_1 = SPI_ASIC_UNO(SPI_FRAME_READ_COMMON_STATUS_2);
    uint32_t com_stat_2 = SPI_ASIC_UNO(SPI_FRAME_READ_COMMON_STATUS_2);

    status->summary_status = SPI_DATA_UINT16(sum_stat);
    status->rate_status1 = SPI_DATA_UINT16(rat_stat);
    status->acc_status1 = SPI_DATA_UINT16(acc_stat);
    status->common_status1 = SPI_DATA_UINT16(com_stat_1);
    status->common_status2 = SPI_DATA_UINT16(com_stat_2);

}


/**

    Read sensor status from DUE ASIC

    Parameters:         *status    - pointer to sensor status structure where the data is read to

    Return value:       None

**/
void scha63x_read_sensor_status_due(scha63x_sensor_status *status)
{

    SPI_ASIC_DUE(SPI_FRAME_READ_SUMMARY_STATUS);
    uint32_t sum_stat   = SPI_ASIC_DUE(SPI_FRAME_READ_RATE_STATUS_1);
    uint32_t rat_stat_1 = SPI_ASIC_DUE(SPI_FRAME_READ_RATE_STATUS_2);
    uint32_t rat_stat_2 = SPI_ASIC_DUE(SPI_FRAME_READ_COMMON_STATUS_1);
    uint32_t com_stat_1 = SPI_ASIC_DUE(SPI_FRAME_READ_COMMON_STATUS_2);
    uint32_t com_stat_2 = SPI_ASIC_DUE(SPI_FRAME_READ_COMMON_STATUS_2);

    status->summary_status = SPI_DATA_UINT16(sum_stat);
    status->rate_status1 = SPI_DATA_UINT16(rat_stat_1);
    status->rate_status2 = SPI_DATA_UINT16(rat_stat_2);
    status->common_status1 = SPI_DATA_UINT16(com_stat_1);
    status->common_status2 = SPI_DATA_UINT16(com_stat_2);

}
/*----------------------------------------------------------------------------
  SPI communication
 *----------------------------------------------------------------------------*/
static uint32_t SPI_ASIC(uint32_t dout, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    uint32_t resp;	//存储接收到的数据
    uint32_t data_out;	//准备发送的数据

    // SCHA63X data frame length is 32 bits and the data must be sent out MSB first.
    // Function SPI_Transfer() does not support sending 32 bit data so the frame is divided to two 16 bit
    // transfers (SPI is configured to 16 bit mode in SPI_Initialize() earlier) and software CS is used
    // to hold the CS low during the whole transfer sequence.

    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET); // Set CS active激活片选信号

    // Swap 16-bit word order of data to send. By default the SPI_Transfer() sends the lower 16 bit word first
    // so the word order is swapped here to comply with MSB first requirement.
	  //由于 SCHA63X 数据帧长度为32位，且数据需要以最高有效位（MSB）优先的方式发送，但 SPI_Transfer() 函数默认以最低有效位（LSB）优先发送，因此需要交换16位字的顺序
    data_out =  (dout >> 16) & 0x0000ffff;	//将 dout 的高16位移动到 data_out 的低16位
    data_out |= (dout << 16) & 0xffff0000;	//将 dout 的低16位移动到 data_out 的高16位

    HAL_SPI_TransmitReceive(IMU_SPI,(unsigned char *)&data_out,(unsigned char *)&resp,2,10);

    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);   // Set CS inactive激活片选信号

    // Collect 32-bit response frame from the received 16-bit words.
    resp = ((resp << 16) & 0xffff0000) | ((resp >> 16) & 0x0000ffff);

    return resp;
}


/*----------------------------------------------------------------------------
  SPI communication to ASIC1 (DUE) using CS1
 *----------------------------------------------------------------------------*/
uint32_t SPI_ASIC_DUE(uint32_t dout)
{
    return SPI_ASIC(dout, DUE_NSS_GPIO_Port, DUE_NSS_Pin);
}


/*----------------------------------------------------------------------------
  SPI communication to ASIC2 (UNO) using CS2
 *----------------------------------------------------------------------------*/
uint32_t SPI_ASIC_UNO(uint32_t dout)
{
    return SPI_ASIC(dout, UNO_NSS_GPIO_Port, UNO_NSS_Pin);
}

