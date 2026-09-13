/**
 * @file SyntrakImuVersion.h
 * @author Leo
 * @brief Header file for the Syntrak Sensor
 */

#ifndef SYNTRAK_IMU_H
#define SYNTRAK_IMU_H

//------------------------------------------------------------------------------
// Includes

#ifdef __cplusplus
extern "C" {
#endif

#define STR_IMPL(x) #x
#define STR(x) STR_IMPL(x)

//Global Share
//Device Type
#define     Syntrak_TYPE_IMU                0xA0
#define     Syntrak_TYPE_MAINCONTROLLER     0xC0
//SYNTRAK_IMU_ID
#define IMU_ID_Waist               1
#define IMU_ID_Hip_Left            2
#define IMU_ID_Hip_Right           3
#define IMU_ID_Thigh_Left          4
#define IMU_ID_Thigh_Right         5
#define IMU_ID_Calf_Left           6
#define IMU_ID_Calf_Right          7
#define IMU_ID_Shoulder_Left       8
#define IMU_ID_Shoulder_Right      9
#define IMU_ID_UpperArm_Left       10
#define IMU_ID_UpperArm_Right      11
#define IMU_ID_ForeArm_Left        12
#define IMU_ID_ForeArm_Right       13




#ifdef __cplusplus
}
#endif

#endif

//------------------------------------------------------------------------------
// End of file
