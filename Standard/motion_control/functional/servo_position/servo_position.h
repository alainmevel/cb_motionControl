/*2023-10-04T11:03:54-01:00*/

/********************************************************************
 * servo_position.h
 * 
 * generated by: plcgen 3.1.391
 ********************************************************************/

#ifndef _MOTION_CONTROL__SERVO_POSITION_H
#define _MOTION_CONTROL__SERVO_POSITION_H


/********************** Variable declarations ***********************/

extern CB_Index servo_position__num;
extern CB_Mem_Bool *servo_position_MoveAbsEnable;
extern CB_Mem_Bool *servo_position_MoveAbsStart;
extern CB_Mem_Float *servo_position_MovePosition;
extern CB_Mem_Float *servo_position_MoveSpeed;
extern CB_Mem_Float *servo_position_MoveAcceleration;
extern CB_Mem_Float *servo_position_MoveDeceleration;
extern CB_Mem_Float *servo_position_MAX_SPEED;
extern CB_Mem_Float *servo_position_MAX_ACCEL;
extern CB_Mem_Float *servo_position_MAX_DECEL;
extern CB_Mem_Float *servo_position_MODULO_LENGTH;
extern CB_Mem_Int *servo_position_status;
extern CB_Mem_Int *servo_position_errorFlag;
extern CB_Mem_Bool *servo_position_MoveAbsDone;
extern CB_Mem_Float *servo_position_ActualPositionAx;
extern CB_Mem_Float *servo_position_ActualVelocity;
extern CB_Mem_Bool *servo_position_firstStep;
extern CB_Mem_Bool *servo_position_stepDone;
extern CB_Mem_Float *servo_position_timeToStop;
extern CB_Mem_Float *servo_position_distanceToStop;
extern CB_Mem_Float *servo_position_distance;
extern CB_Mem_Float *servo_position_currentTime;
extern CB_Mem_Float *servo_position_currentSpeed1;
extern CB_Mem_Float *servo_position_targetPosition;
extern CB_Mem_Float *servo_position_ActualPositionAx1;
extern CB_Mem_Float *servo_position_acceleration;
extern CB_Mem_Float *servo_position_deceleration;
extern CB_Mem_Float *servo_position_orderSpeed;
extern CB_Mem_Float *servo_position_moveDirection;
extern CB_Mem_Float *servo_position_moveDirection1;
extern CB_Mem_Float *servo_position_endChangeSpeed;
extern CB_Mem_Float *servo_position_ActualVelocity1;

/********************** Function declarations ***********************/

extern int _motion_control__servo_position_init(void);
extern int motion_control__servo_position(void);

#endif /*_MOTION_CONTROL__SERVO_POSITION_H*/
