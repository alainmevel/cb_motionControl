/*2023-10-04T11:03:53-01:00*/

/********************************************************************
 * robot6axis.h
 * 
 * generated by: plcgen 3.1.391
 ********************************************************************/

#ifndef _MOTION_LIB__ROBOT6AXIS_H
#define _MOTION_LIB__ROBOT6AXIS_H


/********************** Variable declarations ***********************/

extern CB_Index robot6axis__num;
extern CB_Mem_Float *robot6axis_armOrigin;
extern CB_Mem_Float *robot6axis_structRotVectorFOOT;
extern CB_Mem_Float *robot6axis_structRotDegFOOT;
extern CB_Mem_Float *robot6axis_ownRotDegFOOT;
extern CB_Mem_Float *robot6axis_structRotVectorFOOT2;
extern CB_Mem_Float *robot6axis_structRotDegFOOT2;
extern CB_Mem_Float *robot6axis_structRotVectorArm2;
extern CB_Mem_Float *robot6axis_structRotDegArm2;
extern CB_Mem_Float *robot6axis_driveRotVector1;
extern CB_Mem_Float *robot6axis_driveRotVector2;
extern CB_Mem_Float *robot6axis_driveRotVector3;
extern CB_Mem_Float *robot6axis_driveRotVector4;
extern CB_Mem_Float *robot6axis_driveRotVector5;
extern CB_Mem_Float *robot6axis_driveRotVector6;
extern CB_Mem_Float *robot6axis_structRotVector1RowA;
extern CB_Mem_Float *robot6axis_structRotDeg1RowA;
extern CB_Mem_Float *robot6axis_structRotVector1RowB;
extern CB_Mem_Float *robot6axis_structRotDeg1RowB;
extern CB_Mem_Float *robot6axis_structRotVectorMultiRowsA;
extern CB_Mem_Float *robot6axis_structRotDegMultiRowsA;
extern CB_Mem_Float *robot6axis_structRotVectorMultiRowsB;
extern CB_Mem_Float *robot6axis_structRotDegMultiRowsB;
extern CB_Mem_Bool *robot6axis_HomeCmd;
extern CB_Mem_Bool *robot6axis_MoveVelCmd;
extern CB_Mem_Bool *robot6axis_MoveAbsCmd;
extern CB_Mem_Bool *robot6axis_MoveVel1ManualCmd;
extern CB_Mem_Bool *robot6axis_MoveVel2ManualCmd;
extern CB_Mem_Bool *robot6axis_MoveAbs1ManualCmd;
extern CB_Mem_Bool *robot6axis_MoveAbs2ManualCmd;
extern CB_Mem_Bool *robot6axis_StopCmd;
extern CB_Mem_Float *robot6axis_MovePosition;
extern CB_Mem_Float *robot6axis_MoveSpeed;
extern CB_Mem_Float *robot6axis_MoveAcceleration;
extern CB_Mem_Float *robot6axis_MoveDeceleration;
extern CB_Mem_Float *robot6axis_offsetAxToMa;
extern CB_Mem_Bool *robot6axis_HomeCmd2;
extern CB_Mem_Bool *robot6axis_MoveVelCmd2;
extern CB_Mem_Bool *robot6axis_MoveAbsCmd2;
extern CB_Mem_Bool *robot6axis_MoveVel1ManualCmd2;
extern CB_Mem_Bool *robot6axis_MoveVel2ManualCmd2;
extern CB_Mem_Bool *robot6axis_MoveAbs1ManualCmd2;
extern CB_Mem_Bool *robot6axis_MoveAbs2ManualCmd2;
extern CB_Mem_Bool *robot6axis_StopCmd2;
extern CB_Mem_Float *robot6axis_MovePosition2;
extern CB_Mem_Float *robot6axis_MoveSpeed2;
extern CB_Mem_Float *robot6axis_MoveAcceleration2;
extern CB_Mem_Float *robot6axis_MoveDeceleration2;
extern CB_Mem_Float *robot6axis_offsetAxToMa2;
extern CB_Mem_Bool *robot6axis_HomeCmd3;
extern CB_Mem_Bool *robot6axis_MoveVelCmd3;
extern CB_Mem_Bool *robot6axis_MoveAbsCmd3;
extern CB_Mem_Bool *robot6axis_MoveVel1ManualCmd3;
extern CB_Mem_Bool *robot6axis_MoveVel2ManualCmd3;
extern CB_Mem_Bool *robot6axis_MoveAbs1ManualCmd3;
extern CB_Mem_Bool *robot6axis_MoveAbs2ManualCmd3;
extern CB_Mem_Bool *robot6axis_StopCmd3;
extern CB_Mem_Float *robot6axis_MovePosition3;
extern CB_Mem_Float *robot6axis_MoveSpeed3;
extern CB_Mem_Float *robot6axis_MoveAcceleration3;
extern CB_Mem_Float *robot6axis_MoveDeceleration3;
extern CB_Mem_Float *robot6axis_offsetAxToMa3;
extern CB_Mem_Bool *robot6axis_HomeCmd4;
extern CB_Mem_Bool *robot6axis_MoveVelCmd4;
extern CB_Mem_Bool *robot6axis_MoveAbsCmd4;
extern CB_Mem_Bool *robot6axis_MoveVel1ManualCmd4;
extern CB_Mem_Bool *robot6axis_MoveVel2ManualCmd4;
extern CB_Mem_Bool *robot6axis_MoveAbs1ManualCmd4;
extern CB_Mem_Bool *robot6axis_MoveAbs2ManualCmd4;
extern CB_Mem_Bool *robot6axis_StopCmd4;
extern CB_Mem_Float *robot6axis_MovePosition4;
extern CB_Mem_Float *robot6axis_MoveSpeed4;
extern CB_Mem_Float *robot6axis_MoveAcceleration4;
extern CB_Mem_Float *robot6axis_MoveDeceleration4;
extern CB_Mem_Float *robot6axis_offsetAxToMa4;
extern CB_Mem_Bool *robot6axis_HomeCmd5;
extern CB_Mem_Bool *robot6axis_MoveVelCmd5;
extern CB_Mem_Bool *robot6axis_MoveAbsCmd5;
extern CB_Mem_Bool *robot6axis_MoveVel1ManualCmd5;
extern CB_Mem_Bool *robot6axis_MoveVel2ManualCmd5;
extern CB_Mem_Bool *robot6axis_MoveAbs1ManualCmd5;
extern CB_Mem_Bool *robot6axis_MoveAbs2ManualCmd5;
extern CB_Mem_Bool *robot6axis_StopCmd5;
extern CB_Mem_Float *robot6axis_MovePosition5;
extern CB_Mem_Float *robot6axis_MoveSpeed5;
extern CB_Mem_Float *robot6axis_MoveAcceleration5;
extern CB_Mem_Float *robot6axis_MoveDeceleration5;
extern CB_Mem_Float *robot6axis_offsetAxToMa5;
extern CB_Mem_Bool *robot6axis_HomeCmd6;
extern CB_Mem_Bool *robot6axis_MoveVelCmd6;
extern CB_Mem_Bool *robot6axis_MoveAbsCmd6;
extern CB_Mem_Bool *robot6axis_MoveVel1ManualCmd6;
extern CB_Mem_Bool *robot6axis_MoveVel2ManualCmd6;
extern CB_Mem_Bool *robot6axis_MoveAbs1ManualCmd6;
extern CB_Mem_Bool *robot6axis_MoveAbs2ManualCmd6;
extern CB_Mem_Bool *robot6axis_StopCmd6;
extern CB_Mem_Float *robot6axis_MovePosition6;
extern CB_Mem_Float *robot6axis_MoveSpeed6;
extern CB_Mem_Float *robot6axis_MoveAcceleration6;
extern CB_Mem_Float *robot6axis_MoveDeceleration6;
extern CB_Mem_Float *robot6axis_offsetAxToMa6;
extern CB_Mem_Bool *robot6axis_PLC_StatusReady;
extern CB_Mem_Bool *robot6axis_PLC_StatusMoved;
extern CB_Mem_Bool *robot6axis_PLC_StatusStand;
extern CB_Mem_Float *robot6axis_PLC_ActualPositionAx;
extern CB_Mem_Float *robot6axis_PLC_ActualPositionMa;
extern CB_Mem_Float *robot6axis_PLC_ActualVelocity;
extern CB_Mem_Bool *robot6axis_PLC_StatusReady2;
extern CB_Mem_Bool *robot6axis_PLC_StatusMoved2;
extern CB_Mem_Bool *robot6axis_PLC_StatusStand2;
extern CB_Mem_Float *robot6axis_PLC_ActualPositionAx2;
extern CB_Mem_Float *robot6axis_PLC_ActualPositionMa2;
extern CB_Mem_Float *robot6axis_PLC_ActualVelocity2;
extern CB_Mem_Bool *robot6axis_PLC_StatusReady3;
extern CB_Mem_Bool *robot6axis_PLC_StatusMoved3;
extern CB_Mem_Bool *robot6axis_PLC_StatusStand3;
extern CB_Mem_Float *robot6axis_PLC_ActualPositionAx3;
extern CB_Mem_Float *robot6axis_PLC_ActualPositionMa3;
extern CB_Mem_Float *robot6axis_PLC_ActualVelocity3;
extern CB_Mem_Bool *robot6axis_PLC_StatusReady4;
extern CB_Mem_Bool *robot6axis_PLC_StatusMoved4;
extern CB_Mem_Bool *robot6axis_PLC_StatusStand4;
extern CB_Mem_Float *robot6axis_PLC_ActualPositionAx4;
extern CB_Mem_Float *robot6axis_PLC_ActualPositionMa4;
extern CB_Mem_Float *robot6axis_PLC_ActualVelocity4;
extern CB_Mem_Bool *robot6axis_PLC_StatusReady5;
extern CB_Mem_Bool *robot6axis_PLC_StatusMoved5;
extern CB_Mem_Bool *robot6axis_PLC_StatusStand5;
extern CB_Mem_Float *robot6axis_PLC_ActualPositionAx5;
extern CB_Mem_Float *robot6axis_PLC_ActualPositionMa5;
extern CB_Mem_Float *robot6axis_PLC_ActualVelocity5;
extern CB_Mem_Bool *robot6axis_PLC_StatusReady6;
extern CB_Mem_Bool *robot6axis_PLC_StatusMoved6;
extern CB_Mem_Bool *robot6axis_PLC_StatusStand6;
extern CB_Mem_Float *robot6axis_PLC_ActualPositionAx6;
extern CB_Mem_Float *robot6axis_PLC_ActualPositionMa6;
extern CB_Mem_Float *robot6axis_PLC_ActualVelocity6;
extern CB_Mem_Bool *robot6axis_gripSyringes;
extern CB_Mem_Float *robot6axis_ActualVelocity1;
extern CB_Mem_Float *robot6axis_ActualVelocity2;
extern CB_Mem_Float *robot6axis_ActualVelocity3;
extern CB_Mem_Float *robot6axis_ActualVelocity4;
extern CB_Mem_Float *robot6axis_ActualVelocity5;
extern CB_Mem_Float *robot6axis_ActualVelocity6;
extern CB_Mem_Float *robot6axis_armExtrem;
extern CB_Mem_Float *robot6axis_velX;
extern CB_Mem_Float *robot6axis_velY;
extern CB_Mem_Float *robot6axis_velZ;
extern CB_Mem_Bool *robot6axis_armInStop;

/********************** Function declarations ***********************/

extern int _motion_lib__robot6axis_init(void);
extern int motion_lib__robot6axis(void);

#endif /*_MOTION_LIB__ROBOT6AXIS_H*/
