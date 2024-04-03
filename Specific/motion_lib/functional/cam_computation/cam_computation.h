/*2023-10-04T11:03:50-01:00*/

/********************************************************************
 * cam_computation.h
 * 
 * generated by: plcgen 3.1.391
 ********************************************************************/

#ifndef _MOTION_LIB__CAM_COMPUTATION_H
#define _MOTION_LIB__CAM_COMPUTATION_H


/********************** Variable declarations ***********************/

extern CB_Index cam_computation__num;
extern CB_Mem_Int *cam_computation_profileNr;
extern CB_Mem_Float *cam_computation_MasterPositionIn;
extern CB_Mem_Int *cam_computation_nbPoints0;
extern CB_Mem_Float *cam_computation_MasterPoints0;
extern CB_Mem_Float *cam_computation_SlavePoints0;
extern CB_Mem_Int *cam_computation_pointer0;
extern CB_Mem_Int *cam_computation_nbPoints1;
extern CB_Mem_Float *cam_computation_MasterPoints1;
extern CB_Mem_Float *cam_computation_SlavePoints1;
extern CB_Mem_Int *cam_computation_pointer1;
extern CB_Mem_Int *cam_computation_nbPoints2;
extern CB_Mem_Float *cam_computation_MasterPoints2;
extern CB_Mem_Float *cam_computation_SlavePoints2;
extern CB_Mem_Int *cam_computation_pointer2;
extern CB_Mem_Int *cam_computation_nbPoints3;
extern CB_Mem_Float *cam_computation_MasterPoints3;
extern CB_Mem_Float *cam_computation_SlavePoints3;
extern CB_Mem_Int *cam_computation_pointer3;
extern CB_Mem_Float *cam_computation_slavePosition;
extern CB_Mem_Float *cam_computation_masterPosition;
extern CB_Mem_Int *cam_computation_i;
extern CB_Mem_Float *cam_computation_modulo;
extern CB_Mem_Int *cam_computation_divider;
extern CB_Mem_Bool *cam_computation_faultProfileNr;
extern CB_Mem_Bool *cam_computation_faultMasterPositionOOB;
extern CB_Mem_Bool *cam_computation_faultLinComputation;
extern CB_Mem_Int *cam_computation_nbPoints;
extern CB_Mem_Float *cam_computation_MasterPoints;
extern CB_Mem_Float *cam_computation_SlavePoints;
extern CB_Mem_Bool *cam_computation_found;
extern CB_Mem_Int *cam_computation_index;
extern CB_Mem_Float *cam_computation_linDenom;
extern CB_Mem_Float *cam_computation_linNum;
extern CB_Mem_Float *cam_computation_linA;
extern CB_Mem_Float *cam_computation_linB;
extern CB_Mem_Float *cam_computation_FACTOR;

/********************** Function declarations ***********************/

extern int _motion_lib__cam_computation_init(void);
extern int motion_lib__cam_computation(void);

#endif /*_MOTION_LIB__CAM_COMPUTATION_H*/
