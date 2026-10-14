// I2Cdev library collection - MPU6050 I2C device class
// Based on InvenSense MPU-6050 register map document rev. 2.0, 5/19/2011 (RM-MPU-6000A-00)
// 8/24/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib

/* ============================================
 I2Cdev device library code is placed under the MIT license
 Copyright (c) 2011 Jeff Rowberg
 Adapted to Crazyflie FW by Bitcraze

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
 ===============================================
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "i2cdev.h"
#include "mpu6050.h"
#include "eprintf.h"
#include "stm32_legacy.h"
#define DEBUG_MODULE "MPU6050"
#include "debug_cf.h"

static uint8_t devAddr;
static I2C_Dev *I2Cx;
static uint8_t buffer[14];
static bool isInit;

/** Default constructor, uses default I2C address.
 * @see MPU6050_DEFAULT_ADDRESS
 */
void mpu6050Init(I2C_Dev *i2cPort)
{
    if (isInit) {
        return;
    }

    I2Cx = i2cPort;
    devAddr = MPU6050_ADDRESS_AD0_LOW;
    isInit = true;
}

bool mpu6050Test(void)
{
    bool testStatus;

    if (!isInit) {
        return false;
    }

    testStatus = mpu6050TestConnection();

    return testStatus;
}

/** Verify the I2C connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, false otherwise
 */
bool mpu6050TestConnection()
{
    vTaskDelay(M2T(100));
    uint8_t id = mpu6050GetDeviceID();
    // 0b110100 (0x34) = MPU6050 (0x68 >> 1)
    // 0b111000 (0x38) = MPU6500 (0x70 >> 1)
    return (id == 0b110100 || id == 0b111000);
}

/** Do a MPU6050/MPU6500 self test.
 * @return True if self test passed, false otherwise
 */
bool mpu6050SelfTest()
{
    if (!mpu6050TestConnection()) {
        return false;
    }
    DEBUG_PRINTD("mpu6050/6500 Self test [OK].\n");
    return true;
}

/** Evaluate the values from a MPU6050 self test.
 * @param low The low limit of the self test
 * @param high The high limit of the self test
 * @param value The value to compare with.
 * @param string A pointer to a string describing the value.
 * @return True if self test within low - high limit, false otherwise
 */
bool mpu6050EvaluateSelfTest(float low, float high, float value, char *string)
{
    if (value < low || value > high) {
        DEBUG_PRINTD("Self test %s [FAIL]. low: %0.2f, high: %0.2f, measured: %0.2f\n",
                     string, (double)low, (double)high, (double)value);
        return false;
    }

    return true;
}

// AUX_VDDIO register (InvenSense demo code calls this RA_*G_OFFS_TC)

uint8_t mpu6050GetAuxVDDIOLevel()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_YG_OFFS_TC, MPU6050_TC_PWR_MODE_BIT, buffer);
    return buffer[0];
}

void mpu6050SetAuxVDDIOLevel(uint8_t level)
{
    i2cdevWriteBit(I2Cx, devAddr, MPU6050_RA_YG_OFFS_TC, MPU6050_TC_PWR_MODE_BIT, level);
}

// SMPLRT_DIV register

uint8_t mpu6050GetRate()
{
    i2cdevReadByte(I2Cx, devAddr, MPU6050_RA_SMPLRT_DIV, buffer);
    return buffer[0];
}

void mpu6050SetRate(uint8_t rate)
{
    i2cdevWriteByte(I2Cx, devAddr, MPU6050_RA_SMPLRT_DIV, rate);
}

// CONFIG register

uint8_t mpu6050GetExternalFrameSync()
{
    i2cdevReadBits(I2Cx, devAddr, MPU6050_RA_CONFIG, MPU6050_CFG_EXT_SYNC_SET_BIT,
                   MPU6050_CFG_EXT_SYNC_SET_LENGTH, buffer);
    return buffer[0];
}

void mpu6050SetExternalFrameSync(uint8_t sync)
{
    i2cdevWriteBits(I2Cx, devAddr, MPU6050_RA_CONFIG, MPU6050_CFG_EXT_SYNC_SET_BIT,
                    MPU6050_CFG_EXT_SYNC_SET_LENGTH, sync);
}

uint8_t mpu6050GetDLPFMode()
{
    i2cdevReadBits(I2Cx, devAddr, MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT,
                   MPU6050_CFG_DLPF_CFG_LENGTH, buffer);
    return buffer[0];
}

void mpu6050SetDLPFMode(uint8_t mode)
{
    i2cdevWriteBits(I2Cx, devAddr, MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT,
                    MPU6050_CFG_DLPF_CFG_LENGTH, mode);
}

// GYRO_CONFIG register

uint8_t mpu6050GetFullScaleGyroRangeId()
{
    i2cdevReadBits(I2Cx, devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT,
                   MPU6050_GCONFIG_FS_SEL_LENGTH, buffer);
    return buffer[0];
}

float mpu6050GetFullScaleGyroDPL()
{
    int32_t rangeId;
    float range;

    rangeId = mpu6050GetFullScaleGyroRangeId();

    switch (rangeId) {
        case MPU6050_GYRO_FS_250:
            range = MPU6050_DEG_PER_LSB_250;
            break;

        case MPU6050_GYRO_FS_500:
            range = MPU6050_DEG_PER_LSB_500;
            break;

        case MPU6050_GYRO_FS_1000:
            range = MPU6050_DEG_PER_LSB_1000;
            break;

        case MPU6050_GYRO_FS_2000:
            range = MPU6050_DEG_PER_LSB_2000;
            break;

        default:
            range = MPU6050_DEG_PER_LSB_1000;
            break;
    }

    return range;
}

void mpu6050SetFullScaleGyroRange(uint8_t range)
{
    i2cdevWriteBits(I2Cx, devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT,
                    MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

void mpu6050SetGyroXSelfTest(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_XG_ST_BIT, enabled);
}

void mpu6050SetGyroYSelfTest(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_YG_ST_BIT, enabled);
}

void mpu6050SetGyroZSelfTest(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_ZG_ST_BIT, enabled);
}

// ACCEL_CONFIG register

bool mpu6050GetAccelXSelfTest()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_XA_ST_BIT, buffer);
    return buffer[0];
}

void mpu6050SetAccelXSelfTest(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_XA_ST_BIT, enabled);
}

bool mpu6050GetAccelYSelfTest()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_YA_ST_BIT, buffer);
    return buffer[0];
}

void mpu6050SetAccelYSelfTest(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_YA_ST_BIT, enabled);
}

bool mpu6050GetAccelZSelfTest()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_ZA_ST_BIT, buffer);
    return buffer[0];
}

void mpu6050SetAccelZSelfTest(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_ZA_ST_BIT, enabled);
}

uint8_t mpu6050GetFullScaleAccelRangeId()
{
    i2cdevReadBits(I2Cx, devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT,
                   MPU6050_ACONFIG_AFS_SEL_LENGTH, buffer);
    return buffer[0];
}

float mpu6050GetFullScaleAccelGPL()
{
    int32_t rangeId;
    float range;

    rangeId = mpu6050GetFullScaleAccelRangeId();

    switch (rangeId) {
        case MPU6050_ACCEL_FS_2:
            range = MPU6050_G_PER_LSB_2;
            break;

        case MPU6050_ACCEL_FS_4:
            range = MPU6050_G_PER_LSB_4;
            break;

        case MPU6050_ACCEL_FS_8:
            range = MPU6050_G_PER_LSB_8;
            break;

        case MPU6050_ACCEL_FS_16:
            range = MPU6050_G_PER_LSB_16;
            break;

        default:
            range = MPU6050_DEG_PER_LSB_1000;
            break;
    }

    return range;
}

void mpu6050SetFullScaleAccelRange(uint8_t range)
{
    i2cdevWriteBits(I2Cx, devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT,
                    MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}

uint8_t mpu6050GetDHPFMode()
{
    i2cdevReadBits(I2Cx, devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_ACCEL_HPF_BIT,
                   MPU6050_ACONFIG_ACCEL_HPF_LENGTH, buffer);
    return buffer[0];
}

void mpu6050SetDHPFMode(uint8_t bandwidth)
{
    i2cdevWriteBits(I2Cx, devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_ACCEL_HPF_BIT,
                    MPU6050_ACONFIG_ACCEL_HPF_LENGTH, bandwidth);
}

// FF_THR register

uint8_t mpu6050GetFreefallDetectionThreshold()
{
    i2cdevReadByte(I2Cx, devAddr, MPU6050_RA_FF_THR, buffer);
    return buffer[0];
}

void mpu6050SetFreefallDetectionThreshold(uint8_t threshold)
{
    i2cdevWriteByte(I2Cx, devAddr, MPU6050_RA_FF_THR, threshold);
}

// FF_DUR register

uint8_t mpu6050GetFreefallDetectionDuration()
{
    i2cdevReadByte(I2Cx, devAddr, MPU6050_RA_FF_DUR, buffer);
    return buffer[0];
}

void mpu6050SetFreefallDetectionDuration(uint8_t duration)
{
    i2cdevWriteByte(I2Cx, devAddr, MPU6050_RA_FF_DUR, duration);
}

// MOT_THR register

uint8_t mpu6050GetMotionDetectionThreshold()
{
    i2cdevReadByte(I2Cx, devAddr, MPU6050_RA_MOT_THR, buffer);
    return buffer[0];
}

void mpu6050SetMotionDetectionThreshold(uint8_t threshold)
{
    i2cdevWriteByte(I2Cx, devAddr, MPU6050_RA_MOT_THR, threshold);
}

// MOT_DUR register

uint8_t mpu6050GetMotionDetectionDuration()
{
    i2cdevReadByte(I2Cx, devAddr, MPU6050_RA_MOT_DUR, buffer);
    return buffer[0];
}

void mpu6050SetMotionDetectionDuration(uint8_t duration)
{
    i2cdevWriteByte(I2Cx, devAddr, MPU6050_RA_MOT_DUR, duration);
}

// ZRMOT_THR register

uint8_t mpu6050GetZeroMotionDetectionThreshold()
{
    i2cdevReadByte(I2Cx, devAddr, MPU6050_RA_ZRMOT_THR, buffer);
    return buffer[0];
}

void mpu6050SetZeroMotionDetectionThreshold(uint8_t threshold)
{
    i2cdevWriteByte(I2Cx, devAddr, MPU6050_RA_ZRMOT_THR, threshold);
}

// ZRMOT_DUR register

uint8_t mpu6050GetZeroMotionDetectionDuration()
{
    i2cdevReadByte(I2Cx, devAddr, MPU6050_RA_ZRMOT_DUR, buffer);
    return buffer[0];
}

void mpu6050SetZeroMotionDetectionDuration(uint8_t duration)
{
    i2cdevWriteByte(I2Cx, devAddr, MPU6050_RA_ZRMOT_DUR, duration);
}

// FIFO_EN register

bool mpu6050GetTempFIFOEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_FIFO_EN, MPU6050_TEMP_FIFO_EN_BIT, buffer);
    return buffer[0];
}

void mpu6050SetTempFIFOEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, MPU6050_RA_FIFO_EN, MPU6050_TEMP_FIFO_EN_BIT, enabled);
}

bool mpu6050GetXGyroFIFOEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_FIFO_EN, MPU6050_XG_FIFO_EN_BIT, buffer);
    return buffer[0];
}

void mpu6050SetXGyroFIFOEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, MPU6050_RA_FIFO_EN, MPU6050_XG_FIFO_EN_BIT, enabled);
}

bool mpu6050GetYGyroFIFOEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_FIFO_EN, MPU6050_YG_FIFO_EN_BIT, buffer);
    return buffer[0];
}

void mpu6050SetYGyroFIFOEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, MPU6050_RA_FIFO_EN, MPU6050_YG_FIFO_EN_BIT, enabled);
}

bool mpu6050GetZGyroFIFOEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_FIFO_EN, MPU6050_ZG_FIFO_EN_BIT, buffer);
    return buffer[0];
}

void mpu6050SetZGyroFIFOEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, MPU6050_RA_FIFO_EN, MPU6050_ZG_FIFO_EN_BIT, enabled);
}

bool mpu6050GetAccelFIFOEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_FIFO_EN, MPU6050_ACCEL_FIFO_EN_BIT, buffer);
    return buffer[0];
}

void mpu6050SetAccelFIFOEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, MPU6050_RA_FIFO_EN, MPU6050_ACCEL_FIFO_EN_BIT, enabled);
}

bool mpu6050GetSlave2FIFOEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_FIFO_EN, MPU6050_SLV2_FIFO_EN_BIT, buffer);
    return buffer[0];
}

void mpu6050SetSlave2FIFOEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, MPU6050_RA_FIFO_EN, MPU6050_SLV2_FIFO_EN_BIT, enabled);
}

bool mpu6050GetSlave1FIFOEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_FIFO_EN, MPU6050_SLV1_FIFO_EN_BIT, buffer);
    return buffer[0];
}

void mpu6050SetSlave1FIFOEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, MPU6050_RA_FIFO_EN, MPU6050_SLV1_FIFO_EN_BIT, enabled);
}

bool mpu6050GetSlave0FIFOEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_FIFO_EN, MPU6050_SLV0_FIFO_EN_BIT, buffer);
    return buffer[0];
}

void mpu6050SetSlave0FIFOEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, MPU6050_RA_FIFO_EN, MPU6050_SLV0_FIFO_EN_BIT, enabled);
}

// I2C_MST_CTRL register

bool mpu6050GetMultiMasterEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_I2C_MST_CTRL, MPU6050_MULT_MST_EN_BIT, buffer);
    return buffer[0];
}

void mpu6050SetMultiMasterEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, MPU6050_RA_I2C_MST_CTRL, MPU6050_MULT_MST_EN_BIT, enabled);
}

bool mpu6050GetWaitForExternalSensorEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_I2C_MST_CTRL, MPU6050_WAIT_FOR_ES_BIT, buffer);
    return buffer[0];
}

void mpu6050SetWaitForExternalSensorEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, MPU6050_RA_I2C_MST_CTRL, MPU6050_WAIT_FOR_ES_BIT, enabled);
}

bool mpu6050GetSlave3FIFOEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_I2C_MST_CTRL, MPU6050_SLV_3_FIFO_EN_BIT, buffer);
    return buffer[0];
}

void mpu6050SetSlave3FIFOEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, MPU6050_RA_I2C_MST_CTRL, MPU6050_SLV_3_FIFO_EN_BIT, enabled);
}

bool mpu6050GetSlaveReadWriteTransitionEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_I2C_MST_CTRL, MPU6050_I2C_MST_P_NSR_BIT, buffer);
    return buffer[0];
}

void mpu6050SetSlaveReadWriteTransitionEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, MPU6050_RA_I2C_MST_CTRL, MPU6050_I2C_MST_P_NSR_BIT, enabled);
}

uint8_t mpu6050GetMasterClockSpeed()
{
    i2cdevReadBits(I2Cx, devAddr, MPU6050_RA_I2C_MST_CTRL, MPU6050_I2C_MST_CLK_BIT,
                   MPU6050_I2C_MST_CLK_LENGTH, buffer);
    return buffer[0];
}

void mpu6050SetMasterClockSpeed(uint8_t speed)
{
    i2cdevWriteBits(I2Cx, devAddr, MPU6050_RA_I2C_MST_CTRL, MPU6050_I2C_MST_CLK_BIT,
                    MPU6050_I2C_MST_CLK_LENGTH, speed);
}

// I2C_SLV* registers (Slave 0-3)

uint8_t mpu6050GetSlaveAddress(uint8_t num)
{
    if (num > 3) {
        return 0;
    }

    i2cdevReadByte(I2Cx, devAddr, MPU6050_RA_I2C_SLV0_ADDR + num * 3, buffer);
    return buffer[0];
}

void mpu6050SetSlaveAddress(uint8_t num, uint8_t address)
{
    if (num > 3) {
        return;
    }

    i2cdevWriteByte(I2Cx, devAddr, MPU6050_RA_I2C_SLV0_ADDR + num * 3, address);
}

uint8_t mpu6050GetSlaveRegister(uint8_t num)
{
    if (num > 3) {
        return 0;
    }

    i2cdevReadByte(I2Cx, devAddr, MPU6050_RA_I2C_SLV0_REG + num * 3, buffer);
    return buffer[0];
}

void mpu6050SetSlaveRegister(uint8_t num, uint8_t reg)
{
    if (num > 3) {
        return;
    }

    i2cdevWriteByte(I2Cx, devAddr, MPU6050_RA_I2C_SLV0_REG + num * 3, reg);
}

bool mpu6050GetSlaveEnabled(uint8_t num)
{
    if (num > 3) {
        return 0;
    }

    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_I2C_SLV0_CTRL + num * 3, MPU6050_I2C_SLV_EN_BIT, buffer);
    return buffer[0];
}

void mpu6050SetSlaveEnabled(uint8_t num, bool enabled)
{
    if (num > 3) {
        return;
    }

    i2cdevWriteBit(I2Cx, devAddr, MPU6050_RA_I2C_SLV0_CTRL + num * 3, MPU6050_I2C_SLV_EN_BIT,
                   enabled);
}

bool mpu6050GetSlaveWordByteSwap(uint8_t num)
{
    if (num > 3) {
        return 0;
    }

    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_I2C_SLV0_CTRL + num * 3, MPU6050_I2C_SLV_BYTE_SW_BIT,
                  buffer);
    return buffer[0];
}

void mpu6050SetSlaveWordByteSwap(uint8_t num, bool enabled)
{
    if (num > 3) {
        return;
    }

    i2cdevWriteBit(I2Cx, devAddr, MPU6050_RA_I2C_SLV0_CTRL + num * 3, MPU6050_I2C_SLV_BYTE_SW_BIT,
                   enabled);
}

bool mpu6050GetSlaveWriteMode(uint8_t num)
{
    if (num > 3) {
        return 0;
    }

    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_I2C_SLV0_CTRL + num * 3, MPU6050_I2C_SLV_REG_DIS_BIT,
                  buffer);
    return buffer[0];
}

void mpu6050SetSlaveWriteMode(uint8_t num, bool mode)
{
    if (num > 3) {
        return;
    }

    i2cdevWriteBit(I2Cx, devAddr, MPU6050_RA_I2C_SLV0_CTRL + num * 3, MPU6050_I2C_SLV_REG_DIS_BIT,
                   mode);
}

bool mpu6050GetSlaveWordGroupOffset(uint8_t num)
{
    if (num > 3) {
        return 0;
    }

    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_I2C_SLV0_CTRL + num * 3, MPU6050_I2C_SLV_GRP_BIT, buffer);
    return buffer[0];
}

void mpu6050SetSlaveWordGroupOffset(uint8_t num, bool enabled)
{
    if (num > 3) {
        return;
    }

    i2cdevWriteBit(I2Cx, devAddr, MPU6050_RA_I2C_SLV0_CTRL + num * 3, MPU6050_I2C_SLV_GRP_BIT,
                   enabled);
}

uint8_t mpu6050GetSlaveDataLength(uint8_t num)
{
    if (num > 3) {
        return 0;
    }

    i2cdevReadBits(I2Cx, devAddr, MPU6050_RA_I2C_SLV0_CTRL + num * 3, MPU6050_I2C_SLV_LEN_BIT,
                   MPU6050_I2C_SLV_LEN_LENGTH, buffer);
    return buffer[0];
}

void mpu6050SetSlaveDataLength(uint8_t num, uint8_t length)
{
    if (num > 3) {
        return;
    }

    i2cdevWriteBits(I2Cx, devAddr, MPU6050_RA_I2C_SLV0_CTRL + num * 3, MPU6050_I2C_SLV_LEN_BIT,
                    MPU6050_I2C_SLV_LEN_LENGTH, length);
}

// I2C_SLV* registers (Slave 4)

uint8_t mpu6050GetSlave4Address()
{
    i2cdevReadByte(I2Cx, devAddr, MPU6050_RA_I2C_SLV4_ADDR, buffer);
    return buffer[0];
}

void mpu6050SetSlave4Address(uint8_t address)
{
    i2cdevWriteByte(I2Cx, devAddr, MPU6050_RA_I2C_SLV4_ADDR, address);
}

uint8_t mpu6050GetSlave4Register()
{
    i2cdevReadByte(I2Cx, devAddr, MPU6050_RA_I2C_SLV4_REG, buffer);
    return buffer[0];
}

void mpu6050SetSlave4Register(uint8_t reg)
{
    i2cdevWriteByte(I2Cx, devAddr, MPU6050_RA_I2C_SLV4_REG, reg);
}

void mpu6050SetSlave4OutputByte(uint8_t data)
{
    i2cdevWriteByte(I2Cx, devAddr, MPU6050_RA_I2C_SLV4_DO, data);
}

bool mpu6050GetSlave4Enabled()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_I2C_SLV4_CTRL, MPU6050_I2C_SLV4_EN_BIT, buffer);
    return buffer[0];
}

void mpu6050SetSlave4Enabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, MPU6050_RA_I2C_SLV4_CTRL, MPU6050_I2C_SLV4_EN_BIT, enabled);
}

bool mpu6050GetSlave4InterruptEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_I2C_SLV4_CTRL, MPU6050_I2C_SLV4_INT_EN_BIT, buffer);
    return buffer[0];
}

void mpu6050SetSlave4InterruptEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, MPU6050_RA_I2C_SLV4_CTRL, MPU6050_I2C_SLV4_INT_EN_BIT, enabled);
}

bool mpu6050GetSlave4WriteMode()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_I2C_SLV4_CTRL, MPU6050_I2C_SLV4_REG_DIS_BIT, buffer);
    return buffer[0];
}

void mpu6050SetSlave4WriteMode(bool mode)
{
    i2cdevWriteBit(I2Cx, devAddr, MPU6050_RA_I2C_SLV4_CTRL, MPU6050_I2C_SLV4_REG_DIS_BIT, mode);
}

uint8_t mpu6050GetSlave4MasterDelay()
{
    i2cdevReadBits(I2Cx, devAddr, MPU6050_RA_I2C_SLV4_CTRL, MPU6050_I2C_SLV4_MST_DLY_BIT,
                   MPU6050_I2C_SLV4_MST_DLY_LENGTH, buffer);
    return buffer[0];
}

void mpu6050SetSlave4MasterDelay(uint8_t delay)
{
    i2cdevWriteBits(I2Cx, devAddr, MPU6050_RA_I2C_SLV4_CTRL, MPU6050_I2C_SLV4_MST_DLY_BIT,
                    MPU6050_I2C_SLV4_MST_DLY_LENGTH, delay);
}

uint8_t mpu6050GetSlate4InputByte()
{
    i2cdevReadByte(I2Cx, devAddr, MPU6050_RA_I2C_SLV4_DI, buffer);
    return buffer[0];
}

// I2C_MST_STATUS register

bool mpu6050GetPassthroughStatus()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_I2C_MST_STATUS, MPU6050_MST_PASS_THROUGH_BIT, buffer);
    return buffer[0];
}

bool mpu6050GetSlave4IsDone()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_I2C_MST_STATUS, MPU6050_MST_I2C_SLV4_DONE_BIT, buffer);
    return buffer[0];
}

bool mpu6050GetLostArbitration()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_I2C_MST_STATUS, MPU6050_MST_I2C_LOST_ARB_BIT, buffer);
    return buffer[0];
}

bool mpu6050GetSlave4Nack()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_I2C_MST_STATUS, MPU6050_MST_I2C_SLV4_NACK_BIT, buffer);
    return buffer[0];
}

bool mpu6050GetSlave3Nack()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_I2C_MST_STATUS, MPU6050_MST_I2C_SLV3_NACK_BIT, buffer);
    return buffer[0];
}

bool mpu6050GetSlave2Nack()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_I2C_MST_STATUS, MPU6050_MST_I2C_SLV2_NACK_BIT, buffer);
    return buffer[0];
}

bool mpu6050GetSlave1Nack()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_I2C_MST_STATUS, MPU6050_MST_I2C_SLV1_NACK_BIT, buffer);
    return buffer[0];
}

bool mpu6050GetSlave0Nack()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_I2C_MST_STATUS, MPU6050_MST_I2C_SLV0_NACK_BIT, buffer);
    return buffer[0];
}

// INT_PIN_CFG register

bool mpu6050GetInterruptMode()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_LEVEL_BIT, buffer);
    return buffer[0];
}

void mpu6050SetInterruptMode(bool mode)
{
    i2cdevWriteBit(I2Cx, devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_LEVEL_BIT, mode);
}

bool mpu6050GetInterruptDrive()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_OPEN_BIT, buffer);
    return buffer[0];
}

void mpu6050SetInterruptDrive(bool drive)
{
    i2cdevWriteBit(I2Cx, devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_OPEN_BIT, drive);
}

bool mpu6050GetInterruptLatch()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_LATCH_INT_EN_BIT, buffer);
    return buffer[0];
}

void mpu6050SetInterruptLatch(bool latch)
{
    i2cdevWriteBit(I2Cx, devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_LATCH_INT_EN_BIT, latch);
}

bool mpu6050GetInterruptLatchClear()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_RD_CLEAR_BIT, buffer);
    return buffer[0];
}

void mpu6050SetInterruptLatchClear(bool clear)
{
    i2cdevWriteBit(I2Cx, devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_RD_CLEAR_BIT, clear);
}

bool mpu6050GetFSyncInterruptLevel()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_FSYNC_INT_LEVEL_BIT, buffer);
    return buffer[0];
}

void mpu6050SetFSyncInterruptLevel(bool level)
{
    i2cdevWriteBit(I2Cx, devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_FSYNC_INT_LEVEL_BIT, level);
}

bool mpu6050GetFSyncInterruptEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_FSYNC_INT_EN_BIT, buffer);
    return buffer[0];
}

void mpu6050SetFSyncInterruptEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_FSYNC_INT_EN_BIT, enabled);
}

bool mpu6050GetI2CBypassEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, buffer);
    return buffer[0];
}

void mpu6050SetI2CBypassEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}

bool mpu6050GetClockOutputEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_CLKOUT_EN_BIT, buffer);
    return buffer[0];
}

void mpu6050SetClockOutputEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_CLKOUT_EN_BIT, enabled);
}

// INT_ENABLE register

uint8_t mpu6050GetIntEnabled()
{
    i2cdevReadByte(I2Cx, devAddr, MPU6050_RA_INT_ENABLE, buffer);
    return buffer[0];
}

void mpu6050SetIntEnabled(uint8_t enabled)
{
    i2cdevWriteByte(I2Cx, devAddr, MPU6050_RA_INT_ENABLE, enabled);
}

bool mpu6050GetIntFreefallEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_FF_BIT, buffer);
    return buffer[0];
}

void mpu6050SetIntFreefallEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_FF_BIT, enabled);
}

bool mpu6050GetIntMotionEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_MOT_BIT, buffer);
    return buffer[0];
}

void mpu6050SetIntMotionEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_MOT_BIT, enabled);
}

bool mpu6050GetIntZeroMotionEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_ZMOT_BIT, buffer);
    return buffer[0];
}

void mpu6050SetIntZeroMotionEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_ZMOT_BIT, enabled);
}

bool mpu6050GetIntFIFOBufferOverflowEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_FIFO_OFLOW_BIT, buffer);
    return buffer[0];
}

void mpu6050SetIntFIFOBufferOverflowEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_FIFO_OFLOW_BIT, enabled);
}

bool mpu6050GetIntI2CMasterEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_I2C_MST_INT_BIT, buffer);
    return buffer[0];
}

void mpu6050SetIntI2CMasterEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_I2C_MST_INT_BIT, enabled);
}

bool mpu6050GetIntDataReadyEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_DATA_RDY_BIT, buffer);
    return buffer[0];
}

void mpu6050SetIntDataReadyEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_DATA_RDY_BIT, enabled);
}

// INT_STATUS register

uint8_t mpu6050GetIntStatus()
{
    i2cdevReadByte(I2Cx, devAddr, MPU6050_RA_INT_STATUS, buffer);
    return buffer[0];
}

bool mpu6050GetIntFreefallStatus()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_INT_STATUS, MPU6050_INTERRUPT_FF_BIT, buffer);
    return buffer[0];
}

bool mpu6050GetIntMotionStatus()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_INT_STATUS, MPU6050_INTERRUPT_MOT_BIT, buffer);
    return buffer[0];
}

bool mpu6050GetIntZeroMotionStatus()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_INT_STATUS, MPU6050_INTERRUPT_ZMOT_BIT, buffer);
    return buffer[0];
}

bool mpu6050GetIntFIFOBufferOverflowStatus()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_INT_STATUS, MPU6050_INTERRUPT_FIFO_OFLOW_BIT, buffer);
    return buffer[0];
}

bool mpu6050GetIntI2CMasterStatus()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_INT_STATUS, MPU6050_INTERRUPT_I2C_MST_INT_BIT, buffer);
    return buffer[0];
}

bool mpu6050GetIntDataReadyStatus()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_INT_STATUS, MPU6050_INTERRUPT_DATA_RDY_BIT, buffer);
    return buffer[0];
}

// ACCEL_*OUT_* registers

void mpu6050GetMotion9(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz,
                       int16_t *mx, int16_t *my, int16_t *mz)
{
    mpu6050GetMotion6(ax, ay, az, gx, gy, gz);
}

void mpu6050GetMotion6(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz)
{
    i2cdevReadReg8(I2Cx, devAddr, MPU6050_RA_ACCEL_XOUT_H, 14, buffer);
    *ax = (((int16_t) buffer[0]) << 8) | buffer[1];
    *ay = (((int16_t) buffer[2]) << 8) | buffer[3];
    *az = (((int16_t) buffer[4]) << 8) | buffer[5];
    *gx = (((int16_t) buffer[8]) << 8) | buffer[9];
    *gy = (((int16_t) buffer[10]) << 8) | buffer[11];
    *gz = (((int16_t) buffer[12]) << 8) | buffer[13];
}

void mpu6050GetAcceleration(int16_t *x, int16_t *y, int16_t *z)
{
    i2cdevReadReg8(I2Cx, devAddr, MPU6050_RA_ACCEL_XOUT_H, 6, buffer);
    *x = (((int16_t) buffer[0]) << 8) | buffer[1];
    *y = (((int16_t) buffer[2]) << 8) | buffer[3];
    *z = (((int16_t) buffer[4]) << 8) | buffer[5];
}

int16_t mpu6050GetAccelerationX()
{
    i2cdevReadReg8(I2Cx, devAddr, MPU6050_RA_ACCEL_XOUT_H, 2, buffer);
    return (((int16_t) buffer[0]) << 8) | buffer[1];
}

int16_t mpu6050GetAccelerationY()
{
    i2cdevReadReg8(I2Cx, devAddr, MPU6050_RA_ACCEL_YOUT_H, 2, buffer);
    return (((int16_t) buffer[0]) << 8) | buffer[1];
}

int16_t mpu6050GetAccelerationZ()
{
    i2cdevReadReg8(I2Cx, devAddr, MPU6050_RA_ACCEL_ZOUT_H, 2, buffer);
    return (((int16_t) buffer[0]) << 8) | buffer[1];
}

// TEMP_OUT_* registers

int16_t mpu6050GetTemperature()
{
    i2cdevReadReg8(I2Cx, devAddr, MPU6050_RA_TEMP_OUT_H, 2, buffer);
    return (((int16_t) buffer[0]) << 8) | buffer[1];
}

// GYRO_*OUT_* registers

void mpu6050GetRotation(int16_t *x, int16_t *y, int16_t *z)
{
    i2cdevReadReg8(I2Cx, devAddr, MPU6050_RA_GYRO_XOUT_H, 6, buffer);
    *x = (((int16_t) buffer[0]) << 8) | buffer[1];
    *y = (((int16_t) buffer[2]) << 8) | buffer[3];
    *z = (((int16_t) buffer[4]) << 8) | buffer[5];
}

int16_t mpu6050GetRotationX()
{
    i2cdevReadReg8(I2Cx, devAddr, MPU6050_RA_GYRO_XOUT_H, 2, buffer);
    return (((int16_t) buffer[0]) << 8) | buffer[1];
}

int16_t mpu6050GetRotationY()
{
    i2cdevReadReg8(I2Cx, devAddr, MPU6050_RA_GYRO_YOUT_H, 2, buffer);
    return (((int16_t) buffer[0]) << 8) | buffer[1];
}

int16_t mpu6050GetRotationZ()
{
    i2cdevReadReg8(I2Cx, devAddr, MPU6050_RA_GYRO_ZOUT_H, 2, buffer);
    return (((int16_t) buffer[0]) << 8) | buffer[1];
}

// EXT_SENS_DATA_* registers

uint8_t mpu6050GetExternalSensorByte(int position)
{
    i2cdevReadByte(I2Cx, devAddr, MPU6050_RA_EXT_SENS_DATA_00 + position, buffer);
    return buffer[0];
}

uint16_t mpu6050GetExternalSensorWord(int position)
{
    i2cdevReadReg8(I2Cx, devAddr, MPU6050_RA_EXT_SENS_DATA_00 + position, 2, buffer);
    return (((uint16_t) buffer[0]) << 8) | buffer[1];
}

uint32_t mpu6050GetExternalSensorDWord(int position)
{
    i2cdevReadReg8(I2Cx, devAddr, MPU6050_RA_EXT_SENS_DATA_00 + position, 4, buffer);
    return (((uint32_t) buffer[0]) << 24) | (((uint32_t) buffer[1]) << 16)
           | (((uint16_t) buffer[2]) << 8) | buffer[3];
}

// MOT_DETECT_STATUS register

bool mpu6050GetXNegMotionDetected()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_MOT_DETECT_STATUS, MPU6050_MOTION_MOT_XNEG_BIT, buffer);
    return buffer[0];
}

bool mpu6050GetXPosMotionDetected()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_MOT_DETECT_STATUS, MPU6050_MOTION_MOT_XPOS_BIT, buffer);
    return buffer[0];
}

bool mpu6050GetYNegMotionDetected()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_MOT_DETECT_STATUS, MPU6050_MOTION_MOT_YNEG_BIT, buffer);
    return buffer[0];
}

bool mpu6050GetYPosMotionDetected()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_MOT_DETECT_STATUS, MPU6050_MOTION_MOT_YPOS_BIT, buffer);
    return buffer[0];
}

bool mpu6050GetZNegMotionDetected()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_MOT_DETECT_STATUS, MPU6050_MOTION_MOT_ZNEG_BIT, buffer);
    return buffer[0];
}

bool mpu6050GetZPosMotionDetected()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_MOT_DETECT_STATUS, MPU6050_MOTION_MOT_ZPOS_BIT, buffer);
    return buffer[0];
}

bool mpu6050GetZeroMotionDetected()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_MOT_DETECT_STATUS, MPU6050_MOTION_MOT_ZRMOT_BIT, buffer);
    return buffer[0];
}

// I2C_SLV*_DO register

void mpu6050SetSlaveOutputByte(uint8_t num, uint8_t data)
{
    if (num > 3) {
        return;
    }

    i2cdevWriteByte(I2Cx, devAddr, MPU6050_RA_I2C_SLV0_DO + num, data);
}

// I2C_MST_DELAY_CTRL register

bool mpu6050GetExternalShadowDelayEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_I2C_MST_DELAY_CTRL, MPU6050_DELAYCTRL_DELAY_ES_SHADOW_BIT,
                  buffer);
    return buffer[0];
}

void mpu6050SetExternalShadowDelayEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, MPU6050_RA_I2C_MST_DELAY_CTRL,
                   MPU6050_DELAYCTRL_DELAY_ES_SHADOW_BIT, enabled);
}

bool mpu6050GetSlaveDelayEnabled(uint8_t num)
{
    if (num > 4) {
        return 0;
    }

    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_I2C_MST_DELAY_CTRL, num, buffer);
    return buffer[0];
}

void mpu6050SetSlaveDelayEnabled(uint8_t num, bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, MPU6050_RA_I2C_MST_DELAY_CTRL, num, enabled);
}

// SIGNAL_PATH_RESET register

void mpu6050ResetGyroscopePath()
{
    i2cdevWriteBit(I2Cx, devAddr, MPU6050_RA_SIGNAL_PATH_RESET, MPU6050_PATHRESET_GYRO_RESET_BIT, 1);
}

void mpu6050ResetAccelerometerPath()
{
    i2cdevWriteBit(I2Cx, devAddr, MPU6050_RA_SIGNAL_PATH_RESET, MPU6050_PATHRESET_ACCEL_RESET_BIT, 1);
}

void mpu6050ResetTemperaturePath()
{
    i2cdevWriteBit(I2Cx, devAddr, MPU6050_RA_SIGNAL_PATH_RESET, MPU6050_PATHRESET_TEMP_RESET_BIT, 1);
}

// MOT_DETECT_CTRL register

uint8_t mpu6050GetAccelerometerPowerOnDelay()
{
    i2cdevReadBits(I2Cx, devAddr, MPU6050_RA_MOT_DETECT_CTRL, MPU6050_DETECT_ACCEL_ON_DELAY_BIT,
                   MPU6050_DETECT_ACCEL_ON_DELAY_LENGTH, buffer);
    return buffer[0];
}

void mpu6050SetAccelerometerPowerOnDelay(uint8_t delay)
{
    i2cdevWriteBits(I2Cx, devAddr, MPU6050_RA_MOT_DETECT_CTRL, MPU6050_DETECT_ACCEL_ON_DELAY_BIT,
                    MPU6050_DETECT_ACCEL_ON_DELAY_LENGTH, delay);
}

uint8_t mpu6050GetFreefallDetectionCounterDecrement()
{
    i2cdevReadBits(I2Cx, devAddr, MPU6050_RA_MOT_DETECT_CTRL, MPU6050_DETECT_FF_COUNT_BIT,
                   MPU6050_DETECT_FF_COUNT_LENGTH, buffer);
    return buffer[0];
}

void mpu6050SetFreefallDetectionCounterDecrement(uint8_t decrement)
{
    i2cdevWriteBits(I2Cx, devAddr, MPU6050_RA_MOT_DETECT_CTRL, MPU6050_DETECT_FF_COUNT_BIT,
                    MPU6050_DETECT_FF_COUNT_LENGTH, decrement);
}

uint8_t mpu6050GetMotionDetectionCounterDecrement()
{
    i2cdevReadBits(I2Cx, devAddr, MPU6050_RA_MOT_DETECT_CTRL, MPU6050_DETECT_MOT_COUNT_BIT,
                   MPU6050_DETECT_MOT_COUNT_LENGTH, buffer);
    return buffer[0];
}

void mpu6050SetMotionDetectionCounterDecrement(uint8_t decrement)
{
    i2cdevWriteBits(I2Cx, devAddr, MPU6050_RA_MOT_DETECT_CTRL, MPU6050_DETECT_MOT_COUNT_BIT,
                    MPU6050_DETECT_MOT_COUNT_LENGTH, decrement);
}

// USER_CTRL register

bool mpu6050GetFIFOEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_EN_BIT, buffer);
    return buffer[0];
}

void mpu6050SetFIFOEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_EN_BIT, enabled);
}

bool mpu6050GetI2CMasterModeEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, buffer);
    return buffer[0];
}

void mpu6050SetI2CMasterModeEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}

void mpu6050SwitchSPIEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_IF_DIS_BIT, enabled);
}

void mpu6050ResetFIFO()
{
    i2cdevWriteBit(I2Cx, devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_RESET_BIT, 1);
}

void mpu6050ResetI2CMaster()
{
    i2cdevWriteBit(I2Cx, devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_RESET_BIT, 1);
}

void mpu6050ResetSensors()
{
    i2cdevWriteBit(I2Cx, devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_SIG_COND_RESET_BIT, 1);
}

// PWR_MGMT_1 register

void mpu6050Reset()
{
    i2cdevWriteBit(I2Cx, devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET_BIT, 1);
}

bool mpu6050GetSleepEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, buffer);
    return buffer[0];
}

void mpu6050SetSleepEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}

bool mpu6050GetWakeCycleEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CYCLE_BIT, buffer);
    return buffer[0];
}

void mpu6050SetWakeCycleEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CYCLE_BIT, enabled);
}

bool mpu6050GetTempSensorEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_TEMP_DIS_BIT, buffer);
    return buffer[0] == 0;
}

void mpu6050SetTempSensorEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_TEMP_DIS_BIT, !enabled);
}

uint8_t mpu6050GetClockSource()
{
    i2cdevReadBits(I2Cx, devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT,
                   MPU6050_PWR1_CLKSEL_LENGTH, buffer);
    return buffer[0];
}

void mpu6050SetClockSource(uint8_t source)
{
    i2cdevWriteBits(I2Cx, devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT,
                    MPU6050_PWR1_CLKSEL_LENGTH, source);
}

// PWR_MGMT_2 register

uint8_t mpu6050GetWakeFrequency()
{
    i2cdevReadBits(I2Cx, devAddr, MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_LP_WAKE_CTRL_BIT,
                   MPU6050_PWR2_LP_WAKE_CTRL_LENGTH, buffer);
    return buffer[0];
}

void mpu6050SetWakeFrequency(uint8_t frequency)
{
    i2cdevWriteBits(I2Cx, devAddr, MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_LP_WAKE_CTRL_BIT,
                    MPU6050_PWR2_LP_WAKE_CTRL_LENGTH, frequency);
}

bool mpu6050GetStandbyXAccelEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_XA_BIT, buffer);
    return buffer[0];
}

void mpu6050SetStandbyXAccelEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_XA_BIT, enabled);
}

bool mpu6050GetStandbyYAccelEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_YA_BIT, buffer);
    return buffer[0];
}

void mpu6050SetStandbyYAccelEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_YA_BIT, enabled);
}

bool mpu6050GetStandbyZAccelEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_ZA_BIT, buffer);
    return buffer[0];
}

void mpu6050SetStandbyZAccelEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_ZA_BIT, enabled);
}

bool mpu6050GetStandbyXGyroEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_XG_BIT, buffer);
    return buffer[0];
}

void mpu6050SetStandbyXGyroEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_XG_BIT, enabled);
}

bool mpu6050GetStandbyYGyroEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_YG_BIT, buffer);
    return buffer[0];
}

void mpu6050SetStandbyYGyroEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_YG_BIT, enabled);
}

bool mpu6050GetStandbyZGyroEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_ZG_BIT, buffer);
    return buffer[0];
}

void mpu6050SetStandbyZGyroEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_ZG_BIT, enabled);
}

// FIFO_COUNT* registers

uint16_t mpu6050GetFIFOCount()
{
    i2cdevReadReg8(I2Cx, devAddr, MPU6050_RA_FIFO_COUNTH, 2, buffer);
    return (((uint16_t) buffer[0]) << 8) | buffer[1];
}

// FIFO_R_W register

uint8_t mpu6050GetFIFOByte()
{
    i2cdevReadByte(I2Cx, devAddr, MPU6050_RA_FIFO_R_W, buffer);
    return buffer[0];
}

void mpu6050GetFIFOBytes(uint8_t *data, uint8_t length)
{
    i2cdevReadReg8(I2Cx, devAddr, MPU6050_RA_FIFO_R_W, length, data);
}

void mpu6050SetFIFOByte(uint8_t data)
{
    i2cdevWriteByte(I2Cx, devAddr, MPU6050_RA_FIFO_R_W, data);
}

// WHO_AM_I register

uint8_t mpu6050GetDeviceID()
{
    i2cdevReadBits(I2Cx, devAddr, MPU6050_RA_WHO_AM_I, MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_LENGTH,
                   buffer);
    return buffer[0];
}

void mpu6050SetDeviceID(uint8_t id)
{
    i2cdevWriteBits(I2Cx, devAddr, MPU6050_RA_WHO_AM_I, MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_LENGTH,
                    id);
}

// ======== UNDOCUMENTED/DMP REGISTERS/METHODS ========

uint8_t mpu6050GetOTPBankValid()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OTP_BNK_VLD_BIT, buffer);
    return buffer[0];
}

void mpu6050SetOTPBankValid(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OTP_BNK_VLD_BIT, enabled);
}

int8_t mpu6050GetXGyroOffset()
{
    i2cdevReadBits(I2Cx, devAddr, MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OFFSET_BIT,
                   MPU6050_TC_OFFSET_LENGTH, buffer);
    return buffer[0];
}

void mpu6050SetXGyroOffset(int8_t offset)
{
    i2cdevWriteBits(I2Cx, devAddr, MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OFFSET_BIT,
                    MPU6050_TC_OFFSET_LENGTH, offset);
}

int8_t mpu6050GetYGyroOffset()
{
    i2cdevReadBits(I2Cx, devAddr, MPU6050_RA_YG_OFFS_TC, MPU6050_TC_OFFSET_BIT,
                   MPU6050_TC_OFFSET_LENGTH, buffer);
    return buffer[0];
}

void mpu6050SetYGyroOffset(int8_t offset)
{
    i2cdevWriteBits(I2Cx, devAddr, MPU6050_RA_YG_OFFS_TC, MPU6050_TC_OFFSET_BIT,
                    MPU6050_TC_OFFSET_LENGTH, offset);
}

int8_t mpu6050GetZGyroOffset()
{
    i2cdevReadBits(I2Cx, devAddr, MPU6050_RA_ZG_OFFS_TC, MPU6050_TC_OFFSET_BIT,
                   MPU6050_TC_OFFSET_LENGTH, buffer);
    return buffer[0];
}

void mpu6050SetZGyroOffset(int8_t offset)
{
    i2cdevWriteBits(I2Cx, devAddr, MPU6050_RA_ZG_OFFS_TC, MPU6050_TC_OFFSET_BIT,
                    MPU6050_TC_OFFSET_LENGTH, offset);
}

int8_t mpu6050GetXFineGain()
{
    i2cdevReadByte(I2Cx, devAddr, MPU6050_RA_X_FINE_GAIN, buffer);
    return buffer[0];
}

void mpu6050SetXFineGain(int8_t gain)
{
    i2cdevWriteByte(I2Cx, devAddr, MPU6050_RA_X_FINE_GAIN, gain);
}

int8_t mpu6050GetYFineGain()
{
    i2cdevReadByte(I2Cx, devAddr, MPU6050_RA_Y_FINE_GAIN, buffer);
    return buffer[0];
}

void mpu6050SetYFineGain(int8_t gain)
{
    i2cdevWriteByte(I2Cx, devAddr, MPU6050_RA_Y_FINE_GAIN, gain);
}

int8_t mpu6050GetZFineGain()
{
    i2cdevReadByte(I2Cx, devAddr, MPU6050_RA_Z_FINE_GAIN, buffer);
    return buffer[0];
}

void mpu6050SetZFineGain(int8_t gain)
{
    i2cdevWriteByte(I2Cx, devAddr, MPU6050_RA_Z_FINE_GAIN, gain);
}

int16_t mpu6050GetXAccelOffset()
{
    i2cdevReadReg8(I2Cx, devAddr, MPU6050_RA_XA_OFFS_H, 2, buffer);
    return (((int16_t) buffer[0]) << 8) | buffer[1];
}

void mpu6050SetXAccelOffset(int16_t offset)
{
    i2cdevWriteReg8(I2Cx, devAddr, MPU6050_RA_XA_OFFS_H, 2, (uint8_t *)&offset);
}

int16_t mpu6050GetYAccelOffset()
{
    i2cdevReadReg8(I2Cx, devAddr, MPU6050_RA_YA_OFFS_H, 2, buffer);
    return (((int16_t) buffer[0]) << 8) | buffer[1];
}

void mpu6050SetYAccelOffset(int16_t offset)
{
    i2cdevWriteReg8(I2Cx, devAddr, MPU6050_RA_YA_OFFS_H, 2, (uint8_t *)&offset);
}

int16_t mpu6050GetZAccelOffset()
{
    i2cdevReadReg8(I2Cx, devAddr, MPU6050_RA_ZA_OFFS_H, 2, buffer);
    return (((int16_t) buffer[0]) << 8) | buffer[1];
}

void mpu6050SetZAccelOffset(int16_t offset)
{
    i2cdevWriteReg8(I2Cx, devAddr, MPU6050_RA_ZA_OFFS_H, 2, (uint8_t *)&offset);
}

int16_t mpu6050GetXGyroOffsetUser()
{
    i2cdevReadReg8(I2Cx, devAddr, MPU6050_RA_XG_OFFS_USRH, 2, buffer);
    return (((int16_t) buffer[0]) << 8) | buffer[1];
}

void mpu6050SetXGyroOffsetUser(int16_t offset)
{
    i2cdevWriteReg8(I2Cx, devAddr, MPU6050_RA_XG_OFFS_USRH, 2, (uint8_t *)&offset);
}

int16_t mpu6050GetYGyroOffsetUser()
{
    i2cdevReadReg8(I2Cx, devAddr, MPU6050_RA_YG_OFFS_USRH, 2, buffer);
    return (((int16_t) buffer[0]) << 8) | buffer[1];
}

void mpu6050SetYGyroOffsetUser(int16_t offset)
{
    i2cdevWriteReg8(I2Cx, devAddr, MPU6050_RA_YG_OFFS_USRH, 2, (uint8_t *)&offset);
}

int16_t mpu6050GetZGyroOffsetUser()
{
    i2cdevReadReg8(I2Cx, devAddr, MPU6050_RA_ZG_OFFS_USRH, 2, buffer);
    return (((int16_t) buffer[0]) << 8) | buffer[1];
}

void mpu6050SetZGyroOffsetUser(int16_t offset)
{
    i2cdevWriteReg8(I2Cx, devAddr, MPU6050_RA_ZG_OFFS_USRH, 2, (uint8_t *)&offset);
}

// INT_ENABLE register (DMP functions)

bool mpu6050GetIntPLLReadyEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_PLL_RDY_INT_BIT, buffer);
    return buffer[0];
}

void mpu6050SetIntPLLReadyEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_PLL_RDY_INT_BIT, enabled);
}

bool mpu6050GetIntDMPEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_DMP_INT_BIT, buffer);
    return buffer[0];
}

void mpu6050SetIntDMPEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_DMP_INT_BIT, enabled);
}

// DMP_INT_STATUS

bool mpu6050GetDMPInt5Status()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_DMP_INT_STATUS, MPU6050_DMPINT_5_BIT, buffer);
    return buffer[0];
}

bool mpu6050GetDMPInt4Status()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_DMP_INT_STATUS, MPU6050_DMPINT_4_BIT, buffer);
    return buffer[0];
}

bool mpu6050GetDMPInt3Status()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_DMP_INT_STATUS, MPU6050_DMPINT_3_BIT, buffer);
    return buffer[0];
}

bool mpu6050GetDMPInt2Status()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_DMP_INT_STATUS, MPU6050_DMPINT_2_BIT, buffer);
    return buffer[0];
}

bool mpu6050GetDMPInt1Status()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_DMP_INT_STATUS, MPU6050_DMPINT_1_BIT, buffer);
    return buffer[0];
}

bool mpu6050GetDMPInt0Status()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_DMP_INT_STATUS, MPU6050_DMPINT_0_BIT, buffer);
    return buffer[0];
}

// INT_STATUS register (DMP functions)

bool mpu6050GetIntPLLReadyStatus()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_INT_STATUS, MPU6050_INTERRUPT_PLL_RDY_INT_BIT, buffer);
    return buffer[0];
}

bool mpu6050GetIntDMPStatus()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_INT_STATUS, MPU6050_INTERRUPT_DMP_INT_BIT, buffer);
    return buffer[0];
}

// USER_CTRL register (DMP functions)

bool mpu6050GetDMPEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_DMP_EN_BIT, buffer);
    return buffer[0];
}

void mpu6050SetDMPEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_DMP_EN_BIT, enabled);
}

void mpu6050ResetDMP()
{
    i2cdevWriteBit(I2Cx, devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_DMP_RESET_BIT, 1);
}

// BANK_SEL register

void mpu6050SetMemoryBank(uint8_t bank, bool prefetchEnabled, bool userBank)
{
    bank &= 0x1F;

    if (userBank) {
        bank |= 0x20;
    }

    if (prefetchEnabled) {
        bank |= 0x40;
    }

    i2cdevWriteByte(I2Cx, devAddr, MPU6050_RA_BANK_SEL, bank);
}

// MEM_START_ADDR register

void mpu6050SetMemoryStartAddress(uint8_t address)
{
    i2cdevWriteByte(I2Cx, devAddr, MPU6050_RA_MEM_START_ADDR, address);
}

// MEM_R_W register

uint8_t mpu6050ReadMemoryByte()
{
    i2cdevReadByte(I2Cx, devAddr, MPU6050_RA_MEM_R_W, buffer);
    return buffer[0];
}

void mpu6050WriteMemoryByte(uint8_t data)
{
    i2cdevWriteByte(I2Cx, devAddr, MPU6050_RA_MEM_R_W, data);
}

void mpu6050ReadMemoryBlock(uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address)
{
    mpu6050SetMemoryBank(bank, true, true);
    mpu6050SetMemoryStartAddress(address);
    uint8_t chunkSize;
    uint16_t i;

    for (i = 0; i < dataSize;) {
        chunkSize = MPU6050_DMP_MEMORY_CHUNK_SIZE;

        if (i + chunkSize > dataSize) {
            chunkSize = dataSize - i;
        }

        if (chunkSize > 256 - address) {
            chunkSize = 256 - address;
        }

        i2cdevReadReg8(I2Cx, devAddr, MPU6050_RA_MEM_R_W, chunkSize, data + i);

        i += chunkSize;
        address += chunkSize;

        if (i < dataSize) {
            if (address == 0) {
                bank++;
            }

            mpu6050SetMemoryBank(bank, true, true);
            mpu6050SetMemoryStartAddress(address);
        }
    }
}

bool mpu6050WriteMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address,
                             bool verify)
{
    static uint8_t verifyBuffer[MPU6050_DMP_MEMORY_CHUNK_SIZE];
    uint8_t chunkSize;
    uint8_t *progBuffer;
    uint16_t i;

    mpu6050SetMemoryBank(bank, true, true);
    mpu6050SetMemoryStartAddress(address);

    for (i = 0; i < dataSize;) {
        chunkSize = MPU6050_DMP_MEMORY_CHUNK_SIZE;

        if (i + chunkSize > dataSize) {
            chunkSize = dataSize - i;
        }

        if (chunkSize > 256 - address) {
            chunkSize = 256 - address;
        }

        progBuffer = (uint8_t *) data + i;

        i2cdevWriteReg8(I2Cx, devAddr, MPU6050_RA_MEM_R_W, chunkSize, progBuffer);

        if (verify) {
            uint32_t j;
            mpu6050SetMemoryBank(bank, true, true);
            mpu6050SetMemoryStartAddress(address);
            i2cdevReadReg8(I2Cx, devAddr, MPU6050_RA_MEM_R_W, chunkSize, verifyBuffer);

            for (j = 0; j < chunkSize; j++) {
                if (progBuffer[j] != verifyBuffer[j]) {
                    return false;
                }
            }
        }

        i += chunkSize;
        address += chunkSize;

        if (i < dataSize) {
            if (address == 0) {
                bank++;
            }

            mpu6050SetMemoryBank(bank, true, true);
            mpu6050SetMemoryStartAddress(address);
        }
    }

    return true;
}

bool mpu6050WriteProgMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank,
                                 uint8_t address, bool verify)
{
    return mpu6050WriteMemoryBlock(data, dataSize, bank, address, verify);
}

#define MPU6050_DMP_CONFIG_BLOCK_SIZE 6
bool mpu6050WriteDMPConfigurationSet(const uint8_t *data, uint16_t dataSize)
{
    uint8_t *progBuffer, success, special;
    uint16_t i;

    uint8_t bank = 0;
    uint8_t offset = 0;
    uint8_t length = 0;

    for (i = 0; i < dataSize;) {
        bank = data[i++];
        offset = data[i++];
        length = data[i++];
    }

    if (length > 0) {
        progBuffer = (uint8_t *) data + i;
        success = mpu6050WriteMemoryBlock(progBuffer, length, bank, offset, true);
        i += length;
    } else {
        special = data[i++];

        if (special == 0x01) {
            mpu6050SetIntZeroMotionEnabled(true);
            mpu6050SetIntFIFOBufferOverflowEnabled(true);
            mpu6050SetIntDMPEnabled(true);
            success = true;
        } else {
            success = false;
        }
    }

    if (!success) {
        return false;
    }

    return true;
}

bool mpu6050WriteProgDMPConfigurationSet(const uint8_t *data, uint16_t dataSize)
{
    return mpu6050WriteDMPConfigurationSet(data, dataSize);
}

// DMP_CFG_1 register

uint8_t mpu6050GetDMPConfig1()
{
    i2cdevReadByte(I2Cx, devAddr, MPU6050_RA_DMP_CFG_1, buffer);
    return buffer[0];
}

void mpu6050SetDMPConfig1(uint8_t config)
{
    i2cdevWriteByte(I2Cx, devAddr, MPU6050_RA_DMP_CFG_1, config);
}

// DMP_CFG_2 register

uint8_t mpu6050GetDMPConfig2()
{
    i2cdevReadByte(I2Cx, devAddr, MPU6050_RA_DMP_CFG_2, buffer);
    return buffer[0];
}

void mpu6050SetDMPConfig2(uint8_t config)
{
    i2cdevWriteByte(I2Cx, devAddr, MPU6050_RA_DMP_CFG_2, config);
}
