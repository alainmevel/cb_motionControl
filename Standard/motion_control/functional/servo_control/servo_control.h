/*2023-10-04T11:03:55-01:00*/

/********************************************************************
 * servo_control.h
 * 
 * generated by: plcgen 3.1.391
 ********************************************************************/

#ifndef _MOTION_CONTROL__SERVO_CONTROL_H
#define _MOTION_CONTROL__SERVO_CONTROL_H


/********************** Variable declarations ***********************/

extern CB_Index servo_control__num;
extern CB_Mem_Bool *servo_control_HomeCmd;
extern CB_Mem_Bool *servo_control_MoveVelCmd;
extern CB_Mem_Bool *servo_control_MoveAbsCmd;
extern CB_Mem_Bool *servo_control_MoveVel1ManualCmd;
extern CB_Mem_Bool *servo_control_MoveVel2ManualCmd;
extern CB_Mem_Bool *servo_control_MoveAbs1ManualCmd;
extern CB_Mem_Bool *servo_control_MoveAbs2ManualCmd;
extern CB_Mem_Bool *servo_control_StopCmd;
extern CB_Mem_Float *servo_control_MovePosition;
extern CB_Mem_Float *servo_control_MoveSpeed;
extern CB_Mem_Float *servo_control_MoveAcceleration;
extern CB_Mem_Float *servo_control_MoveDeceleration;
extern CB_Mem_Float *servo_control_offsetAxToMa;
extern CB_Mem_Float *servo_control_physicalPosInit;
extern CB_Mem_Float *servo_control_MAX_SPEED;
extern CB_Mem_Int *servo_control_status;
extern CB_Mem_Int *servo_control_errorFlag;
extern CB_Mem_Float *servo_control_IOActualPositionAx;
extern CB_Mem_Bool *servo_control_IOVelDone;
extern CB_Mem_Bool *servo_control_IOAbsDone;
extern CB_Mem_Float *servo_control_IOActualVelocity;
extern CB_Mem_Int *servo_control_OperationMode;
extern CB_Mem_Bool *servo_control_StatusReady;
extern CB_Mem_Bool *servo_control_StatusMove;
extern CB_Mem_Bool *servo_control_StatusStand;
extern CB_Mem_Float *servo_control_ActualPositionMa;
extern CB_Mem_Float *servo_control_ActualVelocity;
extern CB_Mem_Float *servo_control_MaximumVelocity;
extern CB_Mem_Bool *servo_control_VelEnabled;
extern CB_Mem_Bool *servo_control_VelCmd;
extern CB_Mem_Bool *servo_control_PosEnabled;
extern CB_Mem_Bool *servo_control_PosCmd;
extern CB_Mem_Float *servo_control_Position;
extern CB_Mem_Float *servo_control_Speed;
extern CB_Mem_Int *servo_control_ManualMode;
extern CB_Mem_Bool *servo_control_initDone;
extern CB_Mem_Float *servo_control_ActualPositionMa1;
extern CB_Mem_Float *servo_control_offsetAxToMa1;
extern CB_Mem_Float *servo_control_HOME_POSITION;
extern CB_Mem_Float *servo_control_MANUAL_POSITION1;
extern CB_Mem_Float *servo_control_MANUAL_POSITION2;
extern CB_Mem_Float *servo_control_MANUAL_SPEED1;
extern CB_Mem_Float *servo_control_MANUAL_SPEED2;
extern CB_Mem_Float *servo_control_MANUAL_SPEED;

/********************** Function declarations ***********************/

extern int _motion_control__servo_control_init(void);
extern int motion_control__servo_control(void);

#endif /*_MOTION_CONTROL__SERVO_CONTROL_H*/
