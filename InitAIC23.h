/*
 * InitAIC23.h
 *
 *  Created on: Dec 23, 2020
 *      Author: colin
 */

#ifndef INITAIC23_H_
#define INITAIC23_H_

#include <F28x_Project.h>

typedef enum audioMode_t {
    I2S_32b = 0,
    I2S_16b = 1,
    DSP_32b = 2,
    DSP_16b = 3
} audioMode_t;

void InitMcBSPb(audioMode_t mode);

void InitSPIA(void);

void InitAIC23(audioMode_t mode);

void SpiaTransmit(Uint16 data);




#endif /* INITAIC23_H_ */
