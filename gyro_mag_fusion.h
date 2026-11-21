/*
 * gyro_mag_fusion.h
 *
 *  Created on: Oct 9, 2026
 *      Author: USER
 */

#ifndef INC_GYRO_MAG_FUSION_H_
#define INC_GYRO_MAG_FUSION_H_

#include "main.h"

#define GYRO_SIGN  (-1.0f)   // flip to +1 if your mag/gyro directions already agree

typedef struct {
    float O, b;        // state : heading [deg], gyro bias [deg/s]
    float P[2][2];      //covariance matrix
} HdgKF;

void HdgKF_Init(HdgKF *k, float th0);
void HdgKF_Predict(HdgKF *k, float gz_dps, float dt, float q_th, float q_b);
void HdgKF_Update(HdgKF *k, float mag_deg, float r);

#endif /* INC_GYRO_MAG_FUSION_H_ */
