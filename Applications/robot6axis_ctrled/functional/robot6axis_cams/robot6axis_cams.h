/*2023-10-04T11:04:00-01:00*/

/********************************************************************
 * robot6axis_cams.h
 * 
 * generated by: plcgen 3.1.391
 ********************************************************************/

#ifndef _ROBOT6AXIS_CTRLED__ROBOT6AXIS_CAMS_H
#define _ROBOT6AXIS_CTRLED__ROBOT6AXIS_CAMS_H


/********************** Variable declarations ***********************/

extern CB_Index robot6axis_cams__num;
extern CB_Mem_Int *robot6axis_cams_profileNr;
extern CB_Mem_Bool *robot6axis_cams_HomeCmd;
extern CB_Mem_Bool *robot6axis_cams_MoveVelCmd;
extern CB_Mem_Bool *robot6axis_cams_MoveAbsCmd;
extern CB_Mem_Bool *robot6axis_cams_MoveVel1ManualCmd;
extern CB_Mem_Bool *robot6axis_cams_MoveVel2ManualCmd;
extern CB_Mem_Bool *robot6axis_cams_MoveAbs1ManualCmd;
extern CB_Mem_Bool *robot6axis_cams_MoveAbs2ManualCmd;
extern CB_Mem_Bool *robot6axis_cams_StopCmd;
extern CB_Mem_Float *robot6axis_cams_MovePosition;
extern CB_Mem_Float *robot6axis_cams_MoveSpeed;
extern CB_Mem_Float *robot6axis_cams_MoveAcceleration;
extern CB_Mem_Float *robot6axis_cams_MoveDeceleration;
extern CB_Mem_Float *robot6axis_cams_offsetAxToMa;
extern CB_Mem_Char *robot6axis_cams_name1;
extern CB_Mem_Char *robot6axis_cams_name2;
extern CB_Mem_Char *robot6axis_cams_name3;
extern CB_Mem_Char *robot6axis_cams_name4;
extern CB_Mem_Char *robot6axis_cams_name5;
extern CB_Mem_Char *robot6axis_cams_name6;
extern CB_Mem_Char *robot6axis_cams_name1_rej;
extern CB_Mem_Char *robot6axis_cams_name2_rej;
extern CB_Mem_Char *robot6axis_cams_name3_rej;
extern CB_Mem_Char *robot6axis_cams_name4_rej;
extern CB_Mem_Char *robot6axis_cams_name5_rej;
extern CB_Mem_Char *robot6axis_cams_name6_rej;
extern CB_Mem_Float *robot6axis_cams_slavePosition1;
extern CB_Mem_Float *robot6axis_cams_slavePosition2;
extern CB_Mem_Float *robot6axis_cams_slavePosition3;
extern CB_Mem_Float *robot6axis_cams_slavePosition4;
extern CB_Mem_Float *robot6axis_cams_slavePosition5;
extern CB_Mem_Float *robot6axis_cams_slavePosition6;
extern CB_Mem_Float *robot6axis_cams_ActualPositionAx;
extern CB_Mem_Float *robot6axis_cams_masterPosition;

/********************** Function declarations ***********************/

extern int _robot6axis_ctrled__robot6axis_cams_init(void);
extern int robot6axis_ctrled__robot6axis_cams(void);

#endif /*_ROBOT6AXIS_CTRLED__ROBOT6AXIS_CAMS_H*/
