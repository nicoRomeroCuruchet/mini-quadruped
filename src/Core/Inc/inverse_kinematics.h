/*
 * inverse_kinematics.h
 *
 *  Created on: Mar 31, 2022
 *      Author: nicoromero
 */

#ifndef INC_INVERSE_KINEMATICS_H_
#define INC_INVERSE_KINEMATICS_H_



/**
  * @brief
  * @retval None
  */
void inverse_kinematics(float* x,  float L_1, float L_2, float L_3, char side, float* res);
void roto_translation(float psi, float phi, float theta, float* T, float* vec, float* res);
void subtract(float* v_1, float* v_2, float* res);

#endif /* INC_INVERSE_KINEMATICS_H_ */
