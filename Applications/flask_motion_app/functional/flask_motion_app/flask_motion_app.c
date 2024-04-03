/*2023-10-04T11:03:59-01:00*/

/********************************************************************
 * flask_motion_app.c
 * 
 * generated by: plcgen 3.1.391
 ********************************************************************/

#include "cb_comp.h"
#include "iec_1131.h"

#include "flask_motion_app.h"
#include "flask_motion_app/functional/plc/plc.h"
#include "robot6axis_ctrled/functional/robot6axis_ctrled/robot6axis_ctrled.h"
#include "flask_motion_app/functional/conveyors/conveyors.h"


/**************************** Variables *****************************/

CB_Index flask_motion_app__num = 0;
CB_Mem_Int *flask_motion_app_weight;
CB_Mem_Float *flask_motion_app_masterPosition;

/**************************** Variables *****************************/



/************************ Components offsets ************************/

static CB_Offset flask_motion_app__flask_motion_app_offset_plc;
static CB_Offset flask_motion_app__flask_motion_app_offset_robot6axis_ctrled1;
static CB_Offset flask_motion_app__flask_motion_app_offset_conveyors;
static CB_Offset flask_motion_app__flask_motion_app_offset__end = {0, 0, 0, 0};


/********************* Initialization function **********************/

int _flask_motion_app__flask_motion_app_init(void)
{
	CB_Object __start = self;
	*((short *)&((flask_motion_app_weight)->CB_current_value) + ALIGN_OFFSET_SHORT) = 150; /*weight*/
	self.Bool+=25;
	self.Int+=3;
	self.Float+=19;

	/*initialize child plc (flask_motion_app.plc):*/
	self_num++;
	flask_motion_app__flask_motion_app_offset_plc.Bool = self.Bool - __start.Bool;
	flask_motion_app__flask_motion_app_offset_plc.Int = self.Int - __start.Int;
	flask_motion_app__flask_motion_app_offset_plc.Float = self.Float - __start.Float;
	flask_motion_app__flask_motion_app_offset_plc.Char = self.Char - __start.Char;
	plc_presConvUp = __start.Bool+1; /*flask_motion_app__cn_31*/
	plc_presConvDw = __start.Bool+2; /*flask_motion_app__cn_30*/
	plc_masterPosition = flask_motion_app_masterPosition;
	plc_idRead = __start.Int+1; /*flask_motion_app__cn_29*/
	plc_mesuredWeight = __start.Int+2; /*flask_motion_app__cn_28*/
	plc_bpStart = __start.Bool+3; /*flask_motion_app__cn_27*/
	plc_bpAuto = __start.Bool+4; /*flask_motion_app__cn_26*/
	plc_bpManu = __start.Bool+5; /*flask_motion_app__cn_25*/
	plc_bpEmergencyStop = __start.Bool+6; /*flask_motion_app__cn_24*/
	plc_bpStop = __start.Bool+7; /*flask_motion_app__cn_23*/
	plc_HomeCmd = __start.Bool+8; /*flask_motion_app__cn_22*/
	plc_MoveVelCmd = __start.Bool+9; /*flask_motion_app__cn_21*/
	plc_MoveAbsCmd = __start.Bool+10; /*flask_motion_app__cn_20*/
	plc_MoveVel1ManualCmd = __start.Bool+11; /*flask_motion_app__cn_19*/
	plc_MoveVel2ManualCmd = __start.Bool+12; /*flask_motion_app__cn_18*/
	plc_MoveAbs1ManualCmd = __start.Bool+13; /*flask_motion_app__cn_17*/
	plc_MoveAbs2ManualCmd = __start.Bool+14; /*flask_motion_app__cn_16*/
	plc_StopCmd = __start.Bool+15; /*flask_motion_app__cn_15*/
	plc_MovePosition = __start.Float+14; /*flask_motion_app__cn_14*/
	plc_MoveSpeed = __start.Float+15; /*flask_motion_app__cn_13*/
	plc_MoveAcceleration = __start.Float+16; /*flask_motion_app__cn_12*/
	plc_MoveDeceleration = __start.Float+17; /*flask_motion_app__cn_11*/
	plc_offsetAxToMa = __start.Float+18; /*flask_motion_app__cn_10*/
	plc_gripSyringes = __start.Bool+16; /*flask_motion_app__cn_9*/
	plc_Auto = __start.Bool+17; /*flask_motion_app__cn_8*/
	plc_Manu = __start.Bool+18; /*flask_motion_app__cn_7*/
	plc_run = __start.Bool+19; /*flask_motion_app__cn_6*/
	plc_forwardUp = __start.Bool+20; /*flask_motion_app__cn_5*/
	plc_backwardUp = __start.Bool+21; /*flask_motion_app__cn_4*/
	plc_forwardDw = __start.Bool+22; /*flask_motion_app__cn_3*/
	plc_backwardDw = __start.Bool+23; /*flask_motion_app__cn_2*/
	plc_ejectBadProd = __start.Bool+24; /*flask_motion_app__cn_1*/
	_flask_motion_app__plc_init();

	/*initialize child robot6axis_ctrled1 (robot6axis_ctrled.robot6axis_ctrled):*/
	self_num++;
	flask_motion_app__flask_motion_app_offset_robot6axis_ctrled1.Bool = self.Bool - __start.Bool;
	flask_motion_app__flask_motion_app_offset_robot6axis_ctrled1.Int = self.Int - __start.Int;
	flask_motion_app__flask_motion_app_offset_robot6axis_ctrled1.Float = self.Float - __start.Float;
	flask_motion_app__flask_motion_app_offset_robot6axis_ctrled1.Char = self.Char - __start.Char;
	robot6axis_ctrled_armOrigin = __start.Float+9; /*flask_motion_app__cn_32*/
	robot6axis_ctrled_profileNr = __start.Int+0; /*flask_motion_app__cn_33*/
	robot6axis_ctrled_HomeCmd = __start.Bool+8; /*flask_motion_app__cn_22*/
	robot6axis_ctrled_MoveVelCmd = __start.Bool+9; /*flask_motion_app__cn_21*/
	robot6axis_ctrled_MoveAbsCmd = __start.Bool+10; /*flask_motion_app__cn_20*/
	robot6axis_ctrled_MoveVel1ManualCmd = __start.Bool+11; /*flask_motion_app__cn_19*/
	robot6axis_ctrled_MoveVel2ManualCmd = __start.Bool+12; /*flask_motion_app__cn_18*/
	robot6axis_ctrled_MoveAbs1ManualCmd = __start.Bool+13; /*flask_motion_app__cn_17*/
	robot6axis_ctrled_MoveAbs2ManualCmd = __start.Bool+14; /*flask_motion_app__cn_16*/
	robot6axis_ctrled_StopCmd = __start.Bool+15; /*flask_motion_app__cn_15*/
	robot6axis_ctrled_MovePosition = __start.Float+14; /*flask_motion_app__cn_14*/
	robot6axis_ctrled_MoveSpeed = __start.Float+15; /*flask_motion_app__cn_13*/
	robot6axis_ctrled_MoveAcceleration = __start.Float+16; /*flask_motion_app__cn_12*/
	robot6axis_ctrled_MoveDeceleration = __start.Float+17; /*flask_motion_app__cn_11*/
	robot6axis_ctrled_offsetAxToMa = __start.Float+18; /*flask_motion_app__cn_10*/
	robot6axis_ctrled_gripSyringes = __start.Bool+16; /*flask_motion_app__cn_9*/
	robot6axis_ctrled_Auto = __start.Bool+17; /*flask_motion_app__cn_8*/
	robot6axis_ctrled_Manu = __start.Bool+18; /*flask_motion_app__cn_7*/
	robot6axis_ctrled_run = __start.Bool+19; /*flask_motion_app__cn_6*/
	robot6axis_ctrled_armExtrem = __start.Float+4; /*flask_motion_app__cn_34*/
	robot6axis_ctrled_velX = __start.Float+3; /*flask_motion_app__cn_35*/
	robot6axis_ctrled_velY = __start.Float+2; /*flask_motion_app__cn_36*/
	robot6axis_ctrled_velZ = __start.Float+1; /*flask_motion_app__cn_37*/
	robot6axis_ctrled_armInStop = __start.Bool+0; /*flask_motion_app__cn_38*/
	robot6axis_ctrled_masterPosition = flask_motion_app_masterPosition;
	robot6axis_ctrled_ActualPositionAx = __start.Float+0; /*flask_motion_app__cn_39*/
	_robot6axis_ctrled__robot6axis_ctrled_init();

	/*initialize child conveyors (flask_motion_app.conveyors):*/
	self_num++;
	flask_motion_app__flask_motion_app_offset_conveyors.Bool = self.Bool - __start.Bool;
	flask_motion_app__flask_motion_app_offset_conveyors.Int = self.Int - __start.Int;
	flask_motion_app__flask_motion_app_offset_conveyors.Float = self.Float - __start.Float;
	flask_motion_app__flask_motion_app_offset_conveyors.Char = self.Char - __start.Char;
	conveyors_forwardUp = __start.Bool+20; /*flask_motion_app__cn_5*/
	conveyors_backwardUp = __start.Bool+21; /*flask_motion_app__cn_4*/
	conveyors_forwardDw = __start.Bool+22; /*flask_motion_app__cn_3*/
	conveyors_backwardDw = __start.Bool+23; /*flask_motion_app__cn_2*/
	conveyors_ejectBadProd = __start.Bool+24; /*flask_motion_app__cn_1*/
	conveyors_presConvUp = __start.Bool+1; /*flask_motion_app__cn_31*/
	conveyors_presConvDw = __start.Bool+2; /*flask_motion_app__cn_30*/
	conveyors_idRead = __start.Int+1; /*flask_motion_app__cn_29*/
	conveyors_mesuredWeight = __start.Int+2; /*flask_motion_app__cn_28*/
	conveyors_bpStart = __start.Bool+3; /*flask_motion_app__cn_27*/
	conveyors_bpAuto = __start.Bool+4; /*flask_motion_app__cn_26*/
	conveyors_bpManu = __start.Bool+5; /*flask_motion_app__cn_25*/
	conveyors_bpEmergencyStop = __start.Bool+6; /*flask_motion_app__cn_24*/
	conveyors_bpStop = __start.Bool+7; /*flask_motion_app__cn_23*/
	_flask_motion_app__conveyors_init();

	flask_motion_app__flask_motion_app_offset__end.Bool = self.Bool - __start.Bool;
	flask_motion_app__flask_motion_app_offset__end.Int = self.Int - __start.Int;
	flask_motion_app__flask_motion_app_offset__end.Float = self.Float - __start.Float;
	flask_motion_app__flask_motion_app_offset__end.Char = self.Char - __start.Char;

	return 0;
}


/************************ Behavior function *************************/

int flask_motion_app__flask_motion_app(void)
{
	CB_Object __start = self;
	int __ret;
	if (exec_mac_node(self_num)) {

		/*********************** Internals variables ************************/

																																																																														
		/********************* Pre-conditions execution *********************/

		/*no Pre-conditions*/

		/************************ Behavior execution ************************/

		{
			self.Bool = __start.Bool + flask_motion_app__flask_motion_app_offset_plc.Bool;
			self.Int = __start.Int + flask_motion_app__flask_motion_app_offset_plc.Int;
			self.Float = __start.Float + flask_motion_app__flask_motion_app_offset_plc.Float;
			self.Char = __start.Char + flask_motion_app__flask_motion_app_offset_plc.Char;

			/************************** plc execution ***************************/

			self_num++;
			if (plc__num != self_num) {
				plc__num = self_num;
				plc_presConvUp = __start.Bool+1; /*flask_motion_app__cn_31*/
				plc_presConvDw = __start.Bool+2; /*flask_motion_app__cn_30*/
				plc_masterPosition = flask_motion_app_masterPosition;
				plc_idRead = __start.Int+1; /*flask_motion_app__cn_29*/
				plc_mesuredWeight = __start.Int+2; /*flask_motion_app__cn_28*/
				plc_bpStart = __start.Bool+3; /*flask_motion_app__cn_27*/
				plc_bpAuto = __start.Bool+4; /*flask_motion_app__cn_26*/
				plc_bpManu = __start.Bool+5; /*flask_motion_app__cn_25*/
				plc_bpEmergencyStop = __start.Bool+6; /*flask_motion_app__cn_24*/
				plc_bpStop = __start.Bool+7; /*flask_motion_app__cn_23*/
				plc_HomeCmd = __start.Bool+8; /*flask_motion_app__cn_22*/
				plc_MoveVelCmd = __start.Bool+9; /*flask_motion_app__cn_21*/
				plc_MoveAbsCmd = __start.Bool+10; /*flask_motion_app__cn_20*/
				plc_MoveVel1ManualCmd = __start.Bool+11; /*flask_motion_app__cn_19*/
				plc_MoveVel2ManualCmd = __start.Bool+12; /*flask_motion_app__cn_18*/
				plc_MoveAbs1ManualCmd = __start.Bool+13; /*flask_motion_app__cn_17*/
				plc_MoveAbs2ManualCmd = __start.Bool+14; /*flask_motion_app__cn_16*/
				plc_StopCmd = __start.Bool+15; /*flask_motion_app__cn_15*/
				plc_MovePosition = __start.Float+14; /*flask_motion_app__cn_14*/
				plc_MoveSpeed = __start.Float+15; /*flask_motion_app__cn_13*/
				plc_MoveAcceleration = __start.Float+16; /*flask_motion_app__cn_12*/
				plc_MoveDeceleration = __start.Float+17; /*flask_motion_app__cn_11*/
				plc_offsetAxToMa = __start.Float+18; /*flask_motion_app__cn_10*/
				plc_gripSyringes = __start.Bool+16; /*flask_motion_app__cn_9*/
				plc_Auto = __start.Bool+17; /*flask_motion_app__cn_8*/
				plc_Manu = __start.Bool+18; /*flask_motion_app__cn_7*/
				plc_run = __start.Bool+19; /*flask_motion_app__cn_6*/
				plc_forwardUp = __start.Bool+20; /*flask_motion_app__cn_5*/
				plc_backwardUp = __start.Bool+21; /*flask_motion_app__cn_4*/
				plc_forwardDw = __start.Bool+22; /*flask_motion_app__cn_3*/
				plc_backwardDw = __start.Bool+23; /*flask_motion_app__cn_2*/
				plc_ejectBadProd = __start.Bool+24; /*flask_motion_app__cn_1*/
			}
			__ret = flask_motion_app__plc();
			if (__ret != 0) {
				return __ret; /*child execution failure*/
			}

			/******************* robot6axis_ctrled1 execution *******************/

			self_num++;
			if (robot6axis_ctrled__num != self_num) {
				robot6axis_ctrled__num = self_num;
				robot6axis_ctrled_armOrigin = __start.Float+9; /*flask_motion_app__cn_32*/
				robot6axis_ctrled_profileNr = __start.Int+0; /*flask_motion_app__cn_33*/
				robot6axis_ctrled_HomeCmd = __start.Bool+8; /*flask_motion_app__cn_22*/
				robot6axis_ctrled_MoveVelCmd = __start.Bool+9; /*flask_motion_app__cn_21*/
				robot6axis_ctrled_MoveAbsCmd = __start.Bool+10; /*flask_motion_app__cn_20*/
				robot6axis_ctrled_MoveVel1ManualCmd = __start.Bool+11; /*flask_motion_app__cn_19*/
				robot6axis_ctrled_MoveVel2ManualCmd = __start.Bool+12; /*flask_motion_app__cn_18*/
				robot6axis_ctrled_MoveAbs1ManualCmd = __start.Bool+13; /*flask_motion_app__cn_17*/
				robot6axis_ctrled_MoveAbs2ManualCmd = __start.Bool+14; /*flask_motion_app__cn_16*/
				robot6axis_ctrled_StopCmd = __start.Bool+15; /*flask_motion_app__cn_15*/
				robot6axis_ctrled_MovePosition = __start.Float+14; /*flask_motion_app__cn_14*/
				robot6axis_ctrled_MoveSpeed = __start.Float+15; /*flask_motion_app__cn_13*/
				robot6axis_ctrled_MoveAcceleration = __start.Float+16; /*flask_motion_app__cn_12*/
				robot6axis_ctrled_MoveDeceleration = __start.Float+17; /*flask_motion_app__cn_11*/
				robot6axis_ctrled_offsetAxToMa = __start.Float+18; /*flask_motion_app__cn_10*/
				robot6axis_ctrled_gripSyringes = __start.Bool+16; /*flask_motion_app__cn_9*/
				robot6axis_ctrled_Auto = __start.Bool+17; /*flask_motion_app__cn_8*/
				robot6axis_ctrled_Manu = __start.Bool+18; /*flask_motion_app__cn_7*/
				robot6axis_ctrled_run = __start.Bool+19; /*flask_motion_app__cn_6*/
				robot6axis_ctrled_armExtrem = __start.Float+4; /*flask_motion_app__cn_34*/
				robot6axis_ctrled_velX = __start.Float+3; /*flask_motion_app__cn_35*/
				robot6axis_ctrled_velY = __start.Float+2; /*flask_motion_app__cn_36*/
				robot6axis_ctrled_velZ = __start.Float+1; /*flask_motion_app__cn_37*/
				robot6axis_ctrled_armInStop = __start.Bool+0; /*flask_motion_app__cn_38*/
				robot6axis_ctrled_masterPosition = flask_motion_app_masterPosition;
				robot6axis_ctrled_ActualPositionAx = __start.Float+0; /*flask_motion_app__cn_39*/
			}
			__ret = robot6axis_ctrled__robot6axis_ctrled();
			if (__ret != 0) {
				return __ret; /*child execution failure*/
			}

			/*********************** conveyors execution ************************/

			self_num++;
			if (conveyors__num != self_num) {
				conveyors__num = self_num;
				conveyors_forwardUp = __start.Bool+20; /*flask_motion_app__cn_5*/
				conveyors_backwardUp = __start.Bool+21; /*flask_motion_app__cn_4*/
				conveyors_forwardDw = __start.Bool+22; /*flask_motion_app__cn_3*/
				conveyors_backwardDw = __start.Bool+23; /*flask_motion_app__cn_2*/
				conveyors_ejectBadProd = __start.Bool+24; /*flask_motion_app__cn_1*/
				conveyors_presConvUp = __start.Bool+1; /*flask_motion_app__cn_31*/
				conveyors_presConvDw = __start.Bool+2; /*flask_motion_app__cn_30*/
				conveyors_idRead = __start.Int+1; /*flask_motion_app__cn_29*/
				conveyors_mesuredWeight = __start.Int+2; /*flask_motion_app__cn_28*/
				conveyors_bpStart = __start.Bool+3; /*flask_motion_app__cn_27*/
				conveyors_bpAuto = __start.Bool+4; /*flask_motion_app__cn_26*/
				conveyors_bpManu = __start.Bool+5; /*flask_motion_app__cn_25*/
				conveyors_bpEmergencyStop = __start.Bool+6; /*flask_motion_app__cn_24*/
				conveyors_bpStop = __start.Bool+7; /*flask_motion_app__cn_23*/
			}
			__ret = flask_motion_app__conveyors();
			if (__ret != 0) {
				return __ret; /*child execution failure*/
			}

		}


		/******************** Post-conditions execution *********************/

		/*no Post-conditions*/
	}

	/*************************** self update ****************************/

	self.Bool = __start.Bool + flask_motion_app__flask_motion_app_offset__end.Bool;
	self.Int = __start.Int + flask_motion_app__flask_motion_app_offset__end.Int;
	self.Float = __start.Float + flask_motion_app__flask_motion_app_offset__end.Float;
	self.Char = __start.Char + flask_motion_app__flask_motion_app_offset__end.Char;

	return 0;
}

