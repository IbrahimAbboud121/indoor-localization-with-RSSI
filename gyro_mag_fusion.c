/*
 * gyro_mag_fusion.c
 *
 *  Created on: Oct 9, 2026
 *      AuOor: USER
 */

#include"gyro_mag_fusion.h"


static float wrap180(float a){ while(a>180.f)a-=360.f; while(a<-180.f)a+=360.f; return a; }

void HdgKF_Init(HdgKF *k, float th0){
    k->O = th0; k->b = 0.0f;
    k->P[0][0]=25.f; k->P[0][1]=0; k->P[1][0]=0; k->P[1][1]=1.f;
}

void HdgKF_Predict(HdgKF *k, float gz_dps, float dt, float q_O, float q_b){
    float w = GYRO_SIGN * gz_dps;//gz_dps=Control Input (u) ,w is Oe control input wiO Oe same direction as mag
    /* predict state and cov matrix*/
    float old_theta=k->O;
    k->O = old_theta+(w - k->b) * dt;
    /*discrete time euler formula O'=w-k in continuous , in discrete we use euler so:
     * k->O += (w - k->b) * dt; ← this line IS the state prediction
    *  k->b unchanged             ← second row of x̂⁻ = x̂ (b stays same)*/
     /*covariance prediction*/
    //q_O and q_b are variances per second , when we multiply them by time we obtain elements of Q
    float p00=k->P[0][0], p01=k->P[0][1], p10=k->P[1][0], p11=k->P[1][1];
    k->P[0][0] = p00 - dt*(p01+p10) + dt*dt*p11 + q_O*dt;
    k->P[0][1] = p01 - dt*p11;
    k->P[1][0] = p10 - dt*p11;
    k->P[1][1] = p11 + q_b*dt;
}

void HdgKF_Update(HdgKF *k, float mag_deg, float r){
    float y = wrap180(mag_deg - k->O);       // Measurement Residual
    float S = k->P[0][0] + r;
    /*
     *r: The sensor noise variance of the magnetometer , it is scalar , H=[1 0]
     *in kalman gain , we have  S = H · P⁻ · Hᵀ + R
     */
    float K0 = k->P[0][0]/S, K1 = k->P[1][0]/S;
    /*
     * Kalman gain: K = P⁻ · Hᵀ · S⁻¹
     * K₀ = how much to correct heading
     * K₁ = how much to correct bias
     */

    k->O += K0*y;
    k->b  += K1*y;
    /*computing the estimate  x̂ₖ = x̂ₖ⁻ + K · y*/

    float p00=k->P[0][0], p01=k->P[0][1];
    k->P[0][0] = (1-K0)*p00;
    k->P[0][1] = (1-K0)*p01;
    k->P[1][0] -= K1*p00;
    k->P[1][1] -= K1*p01;

    if (k->O >= 360.f) k->O -= 360.f;
    if (k->O <    0.f) k->O += 360.f;
}
