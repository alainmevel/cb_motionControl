/*2023-10-04T11:03:58-01:00*/

/********************************************************************
 * conveyors.c
 * 
 * generated by: plcgen 3.1.391
/******
				*************************    */
 ********************************************************************/

#include "cb_comp.h"
#include "iec_1131.h"

#include "conveyors.h"
#include "motion_lib/functional/motor_2dir_1speed/motor_2dir_1speed.h"
#include "motion_control/functional/product_creator/product_creator.h"
#include "motion_control/functional/product_killer/product_killer.h"
#include "motion_lib/functional/single_acting_jack/single_acting_jack.h"
#include "motion_lib/functional/qrreader/qrreader.h"
#include "motion_lib/functional/weighingscale/weighingscale.h"
#include "motion_lib/functional/presence_sensor/presence_sensor.h"
#include "motion_lib/functional/conveyor/conveyor.h"
#include "motion_lib/functional/pusher/pusher.h"


/**************************** Variables *****************************/

CB_Index conveyors__num = 0;
CB_Mem_Bool *conveyors_forwardUp;
CB_Mem_Bool *conveyors_backwardUp;
CB_Mem_Bool *conveyors_forwardDw;
CB_Mem_Bool *conveyors_backwardDw;
CB_Mem_Bool *conveyors_ejectBadProd;
CB_Mem_Bool *conveyors_presConvUp;
CB_Mem_Bool *conveyors_presConvDw;
CB_Mem_Int *conveyors_idRead;
CB_Mem_Int *conveyors_mesuredWeight;
CB_Mem_Bool *conveyors_bpStart;
CB_Mem_Bool *conveyors_bpAuto;
CB_Mem_Bool *conveyors_bpManu;
CB_Mem_Bool *conveyors_bpEmergencyStop;
CB_Mem_Bool *conveyors_bpStop;

/**************************** Variables *****************************/



/************************ Components offsets ************************/

static CB_Offset flask_motion_app__conveyors_offset_motConvUp;
static CB_Offset flask_motion_app__conveyors_offset_mtConvDw;
static CB_Offset flask_motion_app__conveyors_offset_product_creator1;
static CB_Offset flask_motion_app__conveyors_offset_product_killer1;
static CB_Offset flask_motion_app__conveyors_offset_product_killer2;
static CB_Offset flask_motion_app__conveyors_offset_ejector;
static CB_Offset flask_motion_app__conveyors_offset_qrreader1;
static CB_Offset flask_motion_app__conveyors_offset_weighingscale1;
static CB_Offset flask_motion_app__conveyors_offset_prdEndConvUp;
static CB_Offset flask_motion_app__conveyors_offset_prdBegConvDw;
static CB_Offset flask_motion_app__conveyors_offset_conveyorUp;
static CB_Offset flask_motion_app__conveyors_offset_conveyorDown;
static CB_Offset flask_motion_app__conveyors_offset_pusher1;
static CB_Offset flask_motion_app__conveyors_offset__end = {0, 0, 0, 0};


/********************* Initialization function **********************/

int _flask_motion_app__conveyors_init(void)
{
	CB_Object __start = self;
	self.Bool+=19;
	self.Float+=45;

	/*initialize child motConvUp (motion_lib.motor_2dir_1speed):*/
	self_num++;
	flask_motion_app__conveyors_offset_motConvUp.Bool = self.Bool - __start.Bool;
	flask_motion_app__conveyors_offset_motConvUp.Int = self.Int - __start.Int;
	flask_motion_app__conveyors_offset_motConvUp.Float = self.Float - __start.Float;
	flask_motion_app__conveyors_offset_motConvUp.Char = self.Char - __start.Char;
	motor_2dir_1speed_voltage380 = __start.Bool+18; /*conveyors__cn_4*/
	motor_2dir_1speed_cmdSupply = __start.Bool+17; /*conveyors__cn_5*/
	motor_2dir_1speed_sensorSupply = __start.Bool+16; /*conveyors__cn_6*/
	motor_2dir_1speed_forward = conveyors_forwardUp;
	motor_2dir_1speed_backward = conveyors_backwardUp;
	motor_2dir_1speed_forwardFB = __start.Bool+15; /*conveyors__cn_7*/
	motor_2dir_1speed_backwardFB = __start.Bool+14; /*conveyors__cn_8*/
	motor_2dir_1speed_thermal = __start.Bool+13; /*conveyors__cn_9*/
	motor_2dir_1speed_speedVal = __start.Float+42; /*conveyors__cn_3*/
	motor_2dir_1speed_speedPercent = __start.Float+41; /*conveyors__cn_10*/
	_motion_lib__motor_2dir_1speed_init();

	/*initialize child mtConvDw (motion_lib.motor_2dir_1speed):*/
	self_num++;
	flask_motion_app__conveyors_offset_mtConvDw.Bool = self.Bool - __start.Bool;
	flask_motion_app__conveyors_offset_mtConvDw.Int = self.Int - __start.Int;
	flask_motion_app__conveyors_offset_mtConvDw.Float = self.Float - __start.Float;
	flask_motion_app__conveyors_offset_mtConvDw.Char = self.Char - __start.Char;
	motor_2dir_1speed_voltage380 = __start.Bool+12; /*conveyors__cn_11*/
	motor_2dir_1speed_cmdSupply = __start.Bool+11; /*conveyors__cn_12*/
	motor_2dir_1speed_sensorSupply = __start.Bool+10; /*conveyors__cn_13*/
	motor_2dir_1speed_forward = conveyors_forwardDw;
	motor_2dir_1speed_backward = conveyors_backwardDw;
	motor_2dir_1speed_forwardFB = __start.Bool+9; /*conveyors__cn_14*/
	motor_2dir_1speed_backwardFB = __start.Bool+8; /*conveyors__cn_15*/
	motor_2dir_1speed_thermal = __start.Bool+7; /*conveyors__cn_16*/
	motor_2dir_1speed_speedVal = __start.Float+43; /*conveyors__cn_2*/
	motor_2dir_1speed_speedPercent = __start.Float+40; /*conveyors__cn_17*/
	_motion_lib__motor_2dir_1speed_init();

	/*initialize child product_creator1 (motion_control.product_creator):*/
	self_num++;
	flask_motion_app__conveyors_offset_product_creator1.Bool = self.Bool - __start.Bool;
	flask_motion_app__conveyors_offset_product_creator1.Int = self.Int - __start.Int;
	flask_motion_app__conveyors_offset_product_creator1.Float = self.Float - __start.Float;
	flask_motion_app__conveyors_offset_product_creator1.Char = self.Char - __start.Char;
	_motion_control__product_creator_init();

	/*initialize child product_killer1 (motion_control.product_killer):*/
	self_num++;
	flask_motion_app__conveyors_offset_product_killer1.Bool = self.Bool - __start.Bool;
	flask_motion_app__conveyors_offset_product_killer1.Int = self.Int - __start.Int;
	flask_motion_app__conveyors_offset_product_killer1.Float = self.Float - __start.Float;
	flask_motion_app__conveyors_offset_product_killer1.Char = self.Char - __start.Char;
	_motion_control__product_killer_init();

	/*initialize child product_killer2 (motion_control.product_killer):*/
	self_num++;
	flask_motion_app__conveyors_offset_product_killer2.Bool = self.Bool - __start.Bool;
	flask_motion_app__conveyors_offset_product_killer2.Int = self.Int - __start.Int;
	flask_motion_app__conveyors_offset_product_killer2.Float = self.Float - __start.Float;
	flask_motion_app__conveyors_offset_product_killer2.Char = self.Char - __start.Char;
	_motion_control__product_killer_init();

	/*initialize child ejector (motion_lib.single_acting_jack):*/
	self_num++;
	flask_motion_app__conveyors_offset_ejector.Bool = self.Bool - __start.Bool;
	flask_motion_app__conveyors_offset_ejector.Int = self.Int - __start.Int;
	flask_motion_app__conveyors_offset_ejector.Float = self.Float - __start.Float;
	flask_motion_app__conveyors_offset_ejector.Char = self.Char - __start.Char;
	single_acting_jack_energy = __start.Bool+6; /*conveyors__cn_18*/
	single_acting_jack_cmdSupply = __start.Bool+5; /*conveyors__cn_19*/
	single_acting_jack_sensorSupply = __start.Bool+4; /*conveyors__cn_20*/
	single_acting_jack_valveWork = conveyors_ejectBadProd;
	single_acting_jack_outletSensor = __start.Bool+3; /*conveyors__cn_21*/
	single_acting_jack_inletSensor = __start.Bool+2; /*conveyors__cn_22*/
	single_acting_jack_rodPosition = __start.Float+39; /*conveyors__cn_23*/
	single_acting_jack_rodPercent = __start.Float+38; /*conveyors__cn_24*/
	single_acting_jack_incrementValue = __start.Float+44; /*conveyors__cn_1*/
	_motion_lib__single_acting_jack_init();

	/*initialize child qrreader1 (motion_lib.qrreader):*/
	self_num++;
	flask_motion_app__conveyors_offset_qrreader1.Bool = self.Bool - __start.Bool;
	flask_motion_app__conveyors_offset_qrreader1.Int = self.Int - __start.Int;
	flask_motion_app__conveyors_offset_qrreader1.Float = self.Float - __start.Float;
	flask_motion_app__conveyors_offset_qrreader1.Char = self.Char - __start.Char;
	qrreader_idRead = conveyors_idRead;
	_motion_lib__qrreader_init();

	/*initialize child weighingscale1 (motion_lib.weighingscale):*/
	self_num++;
	flask_motion_app__conveyors_offset_weighingscale1.Bool = self.Bool - __start.Bool;
	flask_motion_app__conveyors_offset_weighingscale1.Int = self.Int - __start.Int;
	flask_motion_app__conveyors_offset_weighingscale1.Float = self.Float - __start.Float;
	flask_motion_app__conveyors_offset_weighingscale1.Char = self.Char - __start.Char;
	weighingscale_Weight = conveyors_mesuredWeight;
	_motion_lib__weighingscale_init();

	/*initialize child prdEndConvUp (motion_lib.presence_sensor):*/
	self_num++;
	flask_motion_app__conveyors_offset_prdEndConvUp.Bool = self.Bool - __start.Bool;
	flask_motion_app__conveyors_offset_prdEndConvUp.Int = self.Int - __start.Int;
	flask_motion_app__conveyors_offset_prdEndConvUp.Float = self.Float - __start.Float;
	flask_motion_app__conveyors_offset_prdEndConvUp.Char = self.Char - __start.Char;
	presence_sensor_incPositionX = __start.Float+37; /*conveyors__cn_25*/
	presence_sensor_incPositionY = __start.Float+36; /*conveyors__cn_26*/
	presence_sensor_incPositionZ = __start.Float+35; /*conveyors__cn_27*/
	presence_sensor_coordCenterX = __start.Float+34; /*conveyors__cn_28*/
	presence_sensor_coordCenterY = __start.Float+33; /*conveyors__cn_29*/
	presence_sensor_coordCenterZ = __start.Float+32; /*conveyors__cn_30*/
	presence_sensor_incRotationX = __start.Float+31; /*conveyors__cn_31*/
	presence_sensor_incRotationY = __start.Float+30; /*conveyors__cn_32*/
	presence_sensor_incRotationZ = __start.Float+29; /*conveyors__cn_33*/
	presence_sensor_sensorSupply = __start.Bool+1; /*conveyors__cn_34*/
	presence_sensor_presence = conveyors_presConvUp;
	_motion_lib__presence_sensor_init();

	/*initialize child prdBegConvDw (motion_lib.presence_sensor):*/
	self_num++;
	flask_motion_app__conveyors_offset_prdBegConvDw.Bool = self.Bool - __start.Bool;
	flask_motion_app__conveyors_offset_prdBegConvDw.Int = self.Int - __start.Int;
	flask_motion_app__conveyors_offset_prdBegConvDw.Float = self.Float - __start.Float;
	flask_motion_app__conveyors_offset_prdBegConvDw.Char = self.Char - __start.Char;
	presence_sensor_incPositionX = __start.Float+28; /*conveyors__cn_35*/
	presence_sensor_incPositionY = __start.Float+27; /*conveyors__cn_36*/
	presence_sensor_incPositionZ = __start.Float+26; /*conveyors__cn_37*/
	presence_sensor_coordCenterX = __start.Float+25; /*conveyors__cn_38*/
	presence_sensor_coordCenterY = __start.Float+24; /*conveyors__cn_39*/
	presence_sensor_coordCenterZ = __start.Float+23; /*conveyors__cn_40*/
	presence_sensor_incRotationX = __start.Float+22; /*conveyors__cn_41*/
	presence_sensor_incRotationY = __start.Float+21; /*conveyors__cn_42*/
	presence_sensor_incRotationZ = __start.Float+20; /*conveyors__cn_43*/
	presence_sensor_sensorSupply = __start.Bool+0; /*conveyors__cn_44*/
	presence_sensor_presence = conveyors_presConvDw;
	_motion_lib__presence_sensor_init();

	/*initialize child conveyorUp (motion_lib.conveyor):*/
	self_num++;
	flask_motion_app__conveyors_offset_conveyorUp.Bool = self.Bool - __start.Bool;
	flask_motion_app__conveyors_offset_conveyorUp.Int = self.Int - __start.Int;
	flask_motion_app__conveyors_offset_conveyorUp.Float = self.Float - __start.Float;
	flask_motion_app__conveyors_offset_conveyorUp.Char = self.Char - __start.Char;
	conveyor_incPositionX = __start.Float+19; /*conveyors__cn_45*/
	conveyor_incPositionY = __start.Float+18; /*conveyors__cn_46*/
	conveyor_incPositionZ = __start.Float+17; /*conveyors__cn_47*/
	conveyor_coordCenterX = __start.Float+16; /*conveyors__cn_48*/
	conveyor_coordCenterY = __start.Float+15; /*conveyors__cn_49*/
	conveyor_coordCenterZ = __start.Float+14; /*conveyors__cn_50*/
	conveyor_incRotationX = __start.Float+13; /*conveyors__cn_51*/
	conveyor_incRotationY = __start.Float+12; /*conveyors__cn_52*/
	conveyor_incRotationZ = __start.Float+11; /*conveyors__cn_53*/
	conveyor_linearSpeed = __start.Float+42; /*conveyors__cn_3*/
	_motion_lib__conveyor_init();

	/*initialize child conveyorDown (motion_lib.conveyor):*/
	self_num++;
	flask_motion_app__conveyors_offset_conveyorDown.Bool = self.Bool - __start.Bool;
	flask_motion_app__conveyors_offset_conveyorDown.Int = self.Int - __start.Int;
	flask_motion_app__conveyors_offset_conveyorDown.Float = self.Float - __start.Float;
	flask_motion_app__conveyors_offset_conveyorDown.Char = self.Char - __start.Char;
	conveyor_incPositionX = __start.Float+10; /*conveyors__cn_54*/
	conveyor_incPositionY = __start.Float+9; /*conveyors__cn_55*/
	conveyor_incPositionZ = __start.Float+8; /*conveyors__cn_56*/
	conveyor_coordCenterX = __start.Float+7; /*conveyors__cn_57*/
	conveyor_coordCenterY = __start.Float+6; /*conveyors__cn_58*/
	conveyor_coordCenterZ = __start.Float+5; /*conveyors__cn_59*/
	conveyor_incRotationX = __start.Float+4; /*conveyors__cn_60*/
	conveyor_incRotationY = __start.Float+3; /*conveyors__cn_61*/
	conveyor_incRotationZ = __start.Float+2; /*conveyors__cn_62*/
	conveyor_linearSpeed = __start.Float+43; /*conveyors__cn_2*/
	_motion_lib__conveyor_init();

	/*initialize child pusher1 (motion_lib.pusher):*/
	self_num++;
	flask_motion_app__conveyors_offset_pusher1.Bool = self.Bool - __start.Bool;
	flask_motion_app__conveyors_offset_pusher1.Int = self.Int - __start.Int;
	flask_motion_app__conveyors_offset_pusher1.Float = self.Float - __start.Float;
	flask_motion_app__conveyors_offset_pusher1.Char = self.Char - __start.Char;
	pusher_moveX = __start.Float+1; /*conveyors__cn_63*/
	pusher_moveY = __start.Float+44; /*conveyors__cn_1*/
	pusher_moveZ = __start.Float+0; /*conveyors__cn_64*/
	_motion_lib__pusher_init();

	flask_motion_app__conveyors_offset__end.Bool = self.Bool - __start.Bool;
	flask_motion_app__conveyors_offset__end.Int = self.Int - __start.Int;
	flask_motion_app__conveyors_offset__end.Float = self.Float - __start.Float;
	flask_motion_app__conveyors_offset__end.Char = self.Char - __start.Char;

	return 0;
}


/************************ Behavior function *************************/

int flask_motion_app__conveyors(void)
{
	CB_Object __start = self;
	int __ret;
	if (exec_mac_node(self_num)) {

		/*********************** Internals variables ************************/

																																																																																																																																
		/********************* Pre-conditions execution *********************/

		/*no Pre-conditions*/

		/************************ Behavior execution ************************/

		{
			self.Bool = __start.Bool + flask_motion_app__conveyors_offset_motConvUp.Bool;
			self.Int = __start.Int + flask_motion_app__conveyors_offset_motConvUp.Int;
			self.Float = __start.Float + flask_motion_app__conveyors_offset_motConvUp.Float;
			self.Char = __start.Char + flask_motion_app__conveyors_offset_motConvUp.Char;

			/*********************** motConvUp execution ************************/

			self_num++;
			if (motor_2dir_1speed__num != self_num) {
				motor_2dir_1speed__num = self_num;
				motor_2dir_1speed_voltage380 = __start.Bool+18; /*conveyors__cn_4*/
				motor_2dir_1speed_cmdSupply = __start.Bool+17; /*conveyors__cn_5*/
				motor_2dir_1speed_sensorSupply = __start.Bool+16; /*conveyors__cn_6*/
				motor_2dir_1speed_forward = conveyors_forwardUp;
				motor_2dir_1speed_backward = conveyors_backwardUp;
				motor_2dir_1speed_forwardFB = __start.Bool+15; /*conveyors__cn_7*/
				motor_2dir_1speed_backwardFB = __start.Bool+14; /*conveyors__cn_8*/
				motor_2dir_1speed_thermal = __start.Bool+13; /*conveyors__cn_9*/
				motor_2dir_1speed_speedVal = __start.Float+42; /*conveyors__cn_3*/
				motor_2dir_1speed_speedPercent = __start.Float+41; /*conveyors__cn_10*/
			}
			__ret = motion_lib__motor_2dir_1speed();
			if (__ret != 0) {
				return __ret; /*child execution failure*/
			}

			/************************ mtConvDw execution ************************/

			self_num++;
			if (motor_2dir_1speed__num != self_num) {
				motor_2dir_1speed__num = self_num;
				motor_2dir_1speed_voltage380 = __start.Bool+12; /*conveyors__cn_11*/
				motor_2dir_1speed_cmdSupply = __start.Bool+11; /*conveyors__cn_12*/
				motor_2dir_1speed_sensorSupply = __start.Bool+10; /*conveyors__cn_13*/
				motor_2dir_1speed_forward = conveyors_forwardDw;
				motor_2dir_1speed_backward = conveyors_backwardDw;
				motor_2dir_1speed_forwardFB = __start.Bool+9; /*conveyors__cn_14*/
				motor_2dir_1speed_backwardFB = __start.Bool+8; /*conveyors__cn_15*/
				motor_2dir_1speed_thermal = __start.Bool+7; /*conveyors__cn_16*/
				motor_2dir_1speed_speedVal = __start.Float+43; /*conveyors__cn_2*/
				motor_2dir_1speed_speedPercent = __start.Float+40; /*conveyors__cn_17*/
			}
			__ret = motion_lib__motor_2dir_1speed();
			if (__ret != 0) {
				return __ret; /*child execution failure*/
			}

			/******************** product_creator1 execution ********************/

			self_num++;
			if (product_creator__num != self_num) {
				product_creator__num = self_num;
			}
			__ret = motion_control__product_creator();
			if (__ret != 0) {
				return __ret; /*child execution failure*/
			}

			/******************** product_killer1 execution *********************/

			self_num++;
			if (product_killer__num != self_num) {
				product_killer__num = self_num;
			}
			__ret = motion_control__product_killer();
			if (__ret != 0) {
				return __ret; /*child execution failure*/
			}

			/******************** product_killer2 execution *********************/

			self_num++;
			if (product_killer__num != self_num) {
				product_killer__num = self_num;
			}
			__ret = motion_control__product_killer();
			if (__ret != 0) {
				return __ret; /*child execution failure*/
			}

			/************************ ejector execution *************************/

			self_num++;
			if (single_acting_jack__num != self_num) {
				single_acting_jack__num = self_num;
				single_acting_jack_energy = __start.Bool+6; /*conveyors__cn_18*/
				single_acting_jack_cmdSupply = __start.Bool+5; /*conveyors__cn_19*/
				single_acting_jack_sensorSupply = __start.Bool+4; /*conveyors__cn_20*/
				single_acting_jack_valveWork = conveyors_ejectBadProd;
				single_acting_jack_outletSensor = __start.Bool+3; /*conveyors__cn_21*/
				single_acting_jack_inletSensor = __start.Bool+2; /*conveyors__cn_22*/
				single_acting_jack_rodPosition = __start.Float+39; /*conveyors__cn_23*/
				single_acting_jack_rodPercent = __start.Float+38; /*conveyors__cn_24*/
				single_acting_jack_incrementValue = __start.Float+44; /*conveyors__cn_1*/
			}
			__ret = motion_lib__single_acting_jack();
			if (__ret != 0) {
				return __ret; /*child execution failure*/
			}

			/*********************** qrreader1 execution ************************/

			self_num++;
			if (qrreader__num != self_num) {
				qrreader__num = self_num;
				qrreader_idRead = conveyors_idRead;
			}
			__ret = motion_lib__qrreader();
			if (__ret != 0) {
				return __ret; /*child execution failure*/
			}

			/********************* weighingscale1 execution *********************/

			self_num++;
			if (weighingscale__num != self_num) {
				weighingscale__num = self_num;
				weighingscale_Weight = conveyors_mesuredWeight;
			}
			__ret = motion_lib__weighingscale();
			if (__ret != 0) {
				return __ret; /*child execution failure*/
			}

			/********************** prdEndConvUp execution **********************/

			self_num++;
			if (presence_sensor__num != self_num) {
				presence_sensor__num = self_num;
				presence_sensor_incPositionX = __start.Float+37; /*conveyors__cn_25*/
				presence_sensor_incPositionY = __start.Float+36; /*conveyors__cn_26*/
				presence_sensor_incPositionZ = __start.Float+35; /*conveyors__cn_27*/
				presence_sensor_coordCenterX = __start.Float+34; /*conveyors__cn_28*/
				presence_sensor_coordCenterY = __start.Float+33; /*conveyors__cn_29*/
				presence_sensor_coordCenterZ = __start.Float+32; /*conveyors__cn_30*/
				presence_sensor_incRotationX = __start.Float+31; /*conveyors__cn_31*/
				presence_sensor_incRotationY = __start.Float+30; /*conveyors__cn_32*/
				presence_sensor_incRotationZ = __start.Float+29; /*conveyors__cn_33*/
				presence_sensor_sensorSupply = __start.Bool+1; /*conveyors__cn_34*/
				presence_sensor_presence = conveyors_presConvUp;
			}
			__ret = motion_lib__presence_sensor();
			if (__ret != 0) {
				return __ret; /*child execution failure*/
			}

			/********************** prdBegConvDw execution **********************/

			self_num++;
			if (presence_sensor__num != self_num) {
				presence_sensor__num = self_num;
				presence_sensor_incPositionX = __start.Float+28; /*conveyors__cn_35*/
				presence_sensor_incPositionY = __start.Float+27; /*conveyors__cn_36*/
				presence_sensor_incPositionZ = __start.Float+26; /*conveyors__cn_37*/
				presence_sensor_coordCenterX = __start.Float+25; /*conveyors__cn_38*/
				presence_sensor_coordCenterY = __start.Float+24; /*conveyors__cn_39*/
				presence_sensor_coordCenterZ = __start.Float+23; /*conveyors__cn_40*/
				presence_sensor_incRotationX = __start.Float+22; /*conveyors__cn_41*/
				presence_sensor_incRotationY = __start.Float+21; /*conveyors__cn_42*/
				presence_sensor_incRotationZ = __start.Float+20; /*conveyors__cn_43*/
				presence_sensor_sensorSupply = __start.Bool+0; /*conveyors__cn_44*/
				presence_sensor_presence = conveyors_presConvDw;
			}
			__ret = motion_lib__presence_sensor();
			if (__ret != 0) {
				return __ret; /*child execution failure*/
			}

			/*********************** conveyorUp execution ***********************/

			self_num++;
			if (conveyor__num != self_num) {
				conveyor__num = self_num;
				conveyor_incPositionX = __start.Float+19; /*conveyors__cn_45*/
				conveyor_incPositionY = __start.Float+18; /*conveyors__cn_46*/
				conveyor_incPositionZ = __start.Float+17; /*conveyors__cn_47*/
				conveyor_coordCenterX = __start.Float+16; /*conveyors__cn_48*/
				conveyor_coordCenterY = __start.Float+15; /*conveyors__cn_49*/
				conveyor_coordCenterZ = __start.Float+14; /*conveyors__cn_50*/
				conveyor_incRotationX = __start.Float+13; /*conveyors__cn_51*/
				conveyor_incRotationY = __start.Float+12; /*conveyors__cn_52*/
				conveyor_incRotationZ = __start.Float+11; /*conveyors__cn_53*/
				conveyor_linearSpeed = __start.Float+42; /*conveyors__cn_3*/
			}
			__ret = motion_lib__conveyor();
			if (__ret != 0) {
				return __ret; /*child execution failure*/
			}

			/********************** conveyorDown execution **********************/

			self_num++;
			if (conveyor__num != self_num) {
				conveyor__num = self_num;
				conveyor_incPositionX = __start.Float+10; /*conveyors__cn_54*/
				conveyor_incPositionY = __start.Float+9; /*conveyors__cn_55*/
				conveyor_incPositionZ = __start.Float+8; /*conveyors__cn_56*/
				conveyor_coordCenterX = __start.Float+7; /*conveyors__cn_57*/
				conveyor_coordCenterY = __start.Float+6; /*conveyors__cn_58*/
				conveyor_coordCenterZ = __start.Float+5; /*conveyors__cn_59*/
				conveyor_incRotationX = __start.Float+4; /*conveyors__cn_60*/
				conveyor_incRotationY = __start.Float+3; /*conveyors__cn_61*/
				conveyor_incRotationZ = __start.Float+2; /*conveyors__cn_62*/
				conveyor_linearSpeed = __start.Float+43; /*conveyors__cn_2*/
			}
			__ret = motion_lib__conveyor();
			if (__ret != 0) {
				return __ret; /*child execution failure*/
			}

			/************************ pusher1 execution *************************/

			self_num++;
			if (pusher__num != self_num) {
				pusher__num = self_num;
				pusher_moveX = __start.Float+1; /*conveyors__cn_63*/
				pusher_moveY = __start.Float+44; /*conveyors__cn_1*/
				pusher_moveZ = __start.Float+0; /*conveyors__cn_64*/
			}
			__ret = motion_lib__pusher();
			if (__ret != 0) {
				return __ret; /*child execution failure*/
			}

		}


		/******************** Post-conditions execution *********************/

		/*no Post-conditions*/
	}

	/*************************** self update ****************************/

	self.Bool = __start.Bool + flask_motion_app__conveyors_offset__end.Bool;
	self.Int = __start.Int + flask_motion_app__conveyors_offset__end.Int;
	self.Float = __start.Float + flask_motion_app__conveyors_offset__end.Float;
	self.Char = __start.Char + flask_motion_app__conveyors_offset__end.Char;

	return 0;
}

