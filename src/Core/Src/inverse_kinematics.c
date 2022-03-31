/*
 * inverse_kinematics.c
 *
 *  Created on: Mar 31, 2022
 *      Author: nicoromero
 */

#include <math.h>
#include <stdlib.h>

void inverse_kinematics(float* x,
		                float L_1,
						float L_2,
						float L_3,
						char side,
						float* res)
{

	float s = 1.0;

	if(side == 'l') s = -1.0;

	float x_1 = x[0];
	float x_2 = x[1];
	float x_3 = x[2];

	float D = (x_1*x_1 + x_2*x_2 + x_3*x_3 - L_1*L_1 - L_2*L_2 - L_3*L_3) / (2*L_2*L_3);

	float delta = sqrt(x_2*x_2 + x_3*x_3 - L_1*L_1);

	if(1 - D*D >=0)
	{
		res[0] = atan2(x_2, -x_3)  +  atan2(delta, -s*L_1);
		res[2] = atan2(sqrt(1 - D*D), D);
		res[1] = atan2(x_1, delta) - atan2(L_3*sin(res[2]), L_2 + L_3*cos(res[2]));
	}

}

void roto_translation(float psi,
					  float phi,
					  float theta,
					  float* T,
					  float* vec,
					  float* res)
{
	res[0] = vec[2]*sin(phi) + vec[0]*cos(phi)*cos(psi) - vec[1]*cos(phi)*sin(psi) + T[0];
	res[1] = vec[0]*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + vec[1]*(cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta)) - vec[2]*cos(phi)*sin(theta) + T[1];
	res[2] = vec[0]*(sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)) + vec[1]*(cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)) + vec[2]*cos(phi)*cos(theta) + T[2];
}

void subtract(float* v_1, float* v_2, float* res)
{
	res[0] = v_1[0] - v_2[0];
	res[1] = v_1[1] - v_2[1];
	res[2] = v_1[2] - v_2[2];
}
