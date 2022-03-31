/*
 * servo_configurations.h
 *
 *  Created on: Mar 30, 2022
 *      Author: nicoromerocuruchet
 */

#ifndef INC_SERVO_CONFIGURATION_H_
#define INC_SERVO_CONFIGURATION_H_

/* Left Front Leg */
#define LF_a_servo_i  584.09839082
#define LF_b_servo_i  1465.0

#define LF_a_servo_ii  -627.07020704
#define LF_b_servo_ii  970

#define LF_a_servo_iii  949.2240125
#define LF_b_servo_iii  -25.43047706
// (i):
#define MAX_LF_servo_i 2000
#define MIN_LF_servo_i 1000
#define LF_servo_i   TIM1->CCR2
// (ii)
#define MAX_LF_servo_ii 2450
#define MIN_LF_servo_ii 550
#define LF_servo_ii  TIM1->CCR1
// (iii)
#define MAX_LF_servo_iii 2100
#define MIN_LF_servo_iii 900
#define LF_servo_iii TIM1->CCR4

/* Left Rear Leg */
#define LR_a_servo_i  -601.60542705
#define LR_b_servo_i  1425.0

#define LR_a_servo_ii  -604.78852455
#define LR_b_servo_ii  1050.0

#define LR_a_servo_iii 957.22527156
#define LR_b_servo_iii -114.77667663

// (i):
#define MAX_LR_servo_i 2000
#define MIN_LR_servo_i 1000
#define LR_servo_i   TIM3->CCR1
// (ii)
#define MAX_LR_servo_ii 2450
#define MIN_LR_servo_ii 600
#define LR_servo_ii  TIM3->CCR3
// (iii)
#define MAX_LR_servo_iii 2100
#define MIN_LR_servo_iii 1100
#define LR_servo_iii TIM3->CCR4

/* Right Front Leg */
#define RF_a_servo_i 598.42232956
#define RF_b_servo_i 1500.0

#define RF_a_servo_ii 601.60542705
#define RF_b_servo_ii 1985.0

#define RF_a_servo_iii  -990.64488319
#define RF_b_servo_iii  3088.20381872

// (i)
#define MAX_RF_servo_i 2000
#define MIN_RF_servo_i 1000
#define RF_servo_i  TIM2->CCR4
// (ii)
#define MAX_RF_servo_ii 2370
#define MIN_RF_servo_ii 600
#define RF_servo_ii TIM1->CCR3
//(iii)
#define MAX_RF_servo_iii 2100
#define MIN_RF_servo_iii 900
#define RF_servo_iii TIM2->CCR3

/* Right Rear Leg */
#define RR_a_servo_i -601.60542705
#define RR_b_servo_i 1473.33333333

#define RR_a_servo_ii  636.61949953
#define RR_b_servo_ii  2020.0

#define RR_a_servo_iii  -1053.44201921 //
#define RR_b_servo_iii  3297.69769341

// (i)
#define MAX_RR_servo_i 2000
#define MIN_RR_servo_i 1000
#define RR_servo_i   TIM2->CCR1
// (ii)
#define MAX_RR_servo_ii 2400
#define MIN_RR_servo_ii 600
#define RR_servo_ii  TIM3->CCR2
//(iii)
#define MAX_RR_servo_iii 2100
#define MIN_RR_servo_iii 900
#define RR_servo_iii TIM2->CCR2


#endif /* INC_SERVO_CONFIGURATION_H_ */
