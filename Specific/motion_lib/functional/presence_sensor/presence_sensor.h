/*2023-10-04T11:03:51-01:00*/

/********************************************************************
 * presence_sensor.h
 * 
 * generated by: plcgen 3.1.391
 ********************************************************************/

#ifndef _MOTION_LIB__PRESENCE_SENSOR_H
#define _MOTION_LIB__PRESENCE_SENSOR_H


/********************** Variable declarations ***********************/

extern CB_Index presence_sensor__num;
extern CB_Mem_Float *presence_sensor_incPositionX;
extern CB_Mem_Float *presence_sensor_incPositionY;
extern CB_Mem_Float *presence_sensor_incPositionZ;
extern CB_Mem_Float *presence_sensor_coordCenterX;
extern CB_Mem_Float *presence_sensor_coordCenterY;
extern CB_Mem_Float *presence_sensor_coordCenterZ;
extern CB_Mem_Float *presence_sensor_incRotationX;
extern CB_Mem_Float *presence_sensor_incRotationY;
extern CB_Mem_Float *presence_sensor_incRotationZ;
extern CB_Mem_Bool *presence_sensor_sensorSupply;
extern CB_Mem_Bool *presence_sensor_presence;
extern CB_Mem_Int *presence_sensor_status_act_prod;
extern CB_Mem_Float *presence_sensor_m11;
extern CB_Mem_Float *presence_sensor_m12;
extern CB_Mem_Float *presence_sensor_m13;
extern CB_Mem_Float *presence_sensor_m14;
extern CB_Mem_Float *presence_sensor_m21;
extern CB_Mem_Float *presence_sensor_m22;
extern CB_Mem_Float *presence_sensor_m23;
extern CB_Mem_Float *presence_sensor_m24;
extern CB_Mem_Float *presence_sensor_m31;
extern CB_Mem_Float *presence_sensor_m32;
extern CB_Mem_Float *presence_sensor_m33;
extern CB_Mem_Float *presence_sensor_m34;
extern CB_Mem_Float *presence_sensor_m41;
extern CB_Mem_Float *presence_sensor_m42;
extern CB_Mem_Float *presence_sensor_m43;
extern CB_Mem_Float *presence_sensor_m44;
extern CB_Mem_Bool *presence_sensor_detection;
extern CB_Mem_Bool *presence_sensor_presenceLogic;
extern CB_Mem_Bool *presence_sensor_d_presence;
extern CB_Mem_Bool *presence_sensor_v_d_presence;

/********************** Function declarations ***********************/

extern int _motion_lib__presence_sensor_init(void);
extern int motion_lib__presence_sensor(void);

/************************** Access macros ***************************/

#define mx_acces_motion_lib__presence_sensor__status_act_prod Int[0].CB_current_value
#define mx_acces_motion_lib__presence_sensor__m11 Float[0].CB_current_value
#define mx_acces_motion_lib__presence_sensor__m12 Float[1].CB_current_value
#define mx_acces_motion_lib__presence_sensor__m13 Float[2].CB_current_value
#define mx_acces_motion_lib__presence_sensor__m14 Float[3].CB_current_value
#define mx_acces_motion_lib__presence_sensor__m21 Float[4].CB_current_value
#define mx_acces_motion_lib__presence_sensor__m22 Float[5].CB_current_value
#define mx_acces_motion_lib__presence_sensor__m23 Float[6].CB_current_value
#define mx_acces_motion_lib__presence_sensor__m24 Float[7].CB_current_value
#define mx_acces_motion_lib__presence_sensor__m31 Float[8].CB_current_value
#define mx_acces_motion_lib__presence_sensor__m32 Float[9].CB_current_value
#define mx_acces_motion_lib__presence_sensor__m33 Float[10].CB_current_value
#define mx_acces_motion_lib__presence_sensor__m34 Float[11].CB_current_value
#define mx_acces_motion_lib__presence_sensor__m41 Float[12].CB_current_value
#define mx_acces_motion_lib__presence_sensor__m42 Float[13].CB_current_value
#define mx_acces_motion_lib__presence_sensor__m43 Float[14].CB_current_value
#define mx_acces_motion_lib__presence_sensor__m44 Float[15].CB_current_value
#define mx_acces_motion_lib__presence_sensor__detection Bool[0].CB_current_value
#define mx_acces_motion_lib__presence_sensor__presenceLogic Bool[1].CB_current_value
#define mx_acces_motion_lib__presence_sensor__d_presence Bool[2].CB_current_value
#define mx_acces_motion_lib__presence_sensor__v_d_presence Bool[3].CB_current_value

#endif /*_MOTION_LIB__PRESENCE_SENSOR_H*/
