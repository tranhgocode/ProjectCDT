/*
 * my_app.h
 *
 *  Created on: May 16, 2026
 *      Author: Lap4all
 */

#ifndef MYLIB_INC_MY_APP_H_
#define MYLIB_INC_MY_APP_H_

#include "TMC2209.h"

extern TMC2209_HandleTypeDef motor1;
extern TMC2209_HandleTypeDef motor2;
extern TMC2209_HandleTypeDef motor3;

void Motors_Config(void);
void Motors_Init(void);
void MyApp_TestStep1(void);
void MyApp_SensorUsb_Init(void);
void MyApp_SensorUsb_Task(void);

#endif /* MYLIB_INC_MY_APP_H_ */
