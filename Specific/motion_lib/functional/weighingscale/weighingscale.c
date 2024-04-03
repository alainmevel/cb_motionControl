/*2023-10-04T11:03:51-01:00*/

/********************************************************************
 * weighingscale.c
 * 
 * generated by: plcgen 3.1.391
 ********************************************************************/

#include "cb_comp.h"
#include "iec_1131.h"
#include "cb_products_iec.h"

#include "weighingscale.h"


/**************************** Variables *****************************/

CB_Index weighingscale__num = 0;
CB_Mem_Int *weighingscale_Weight;
CB_Mem_Int *weighingscale_status_act_prod;
CB_Mem_Float *weighingscale_m11;
CB_Mem_Float *weighingscale_m12;
CB_Mem_Float *weighingscale_m13;
CB_Mem_Float *weighingscale_m14;
CB_Mem_Float *weighingscale_m21;
CB_Mem_Float *weighingscale_m22;
CB_Mem_Float *weighingscale_m23;
CB_Mem_Float *weighingscale_m24;
CB_Mem_Float *weighingscale_m31;
CB_Mem_Float *weighingscale_m32;
CB_Mem_Float *weighingscale_m33;
CB_Mem_Float *weighingscale_m34;
CB_Mem_Float *weighingscale_m41;
CB_Mem_Float *weighingscale_m42;
CB_Mem_Float *weighingscale_m43;
CB_Mem_Float *weighingscale_m44;
CB_Mem_Int *weighingscale_weight;

/**************************** Variables *****************************/

#define Weight (*((short *)&(weighingscale_Weight->CB_current_value) + ALIGN_OFFSET_SHORT))
#define status_act_prod (weighingscale_status_act_prod->CB_current_value)
#define m11 (weighingscale_m11->CB_current_value)
#define m12 (weighingscale_m12->CB_current_value)
#define m13 (weighingscale_m13->CB_current_value)
#define m14 (weighingscale_m14->CB_current_value)
#define m21 (weighingscale_m21->CB_current_value)
#define m22 (weighingscale_m22->CB_current_value)
#define m23 (weighingscale_m23->CB_current_value)
#define m24 (weighingscale_m24->CB_current_value)
#define m31 (weighingscale_m31->CB_current_value)
#define m32 (weighingscale_m32->CB_current_value)
#define m33 (weighingscale_m33->CB_current_value)
#define m34 (weighingscale_m34->CB_current_value)
#define m41 (weighingscale_m41->CB_current_value)
#define m42 (weighingscale_m42->CB_current_value)
#define m43 (weighingscale_m43->CB_current_value)
#define m44 (weighingscale_m44->CB_current_value)
#define weight (*((short *)&(weighingscale_weight->CB_current_value) + ALIGN_OFFSET_SHORT))


/************************ Components offsets ************************/


/********************* Initialization function **********************/

int _motion_lib__weighingscale_init(void)
{
	(self.Float+0)->CB_current_value = 1.0; /*m11*/
	(self.Float+5)->CB_current_value = 1.0; /*m22*/
	(self.Float+10)->CB_current_value = 1.0; /*m33*/
	(self.Float+15)->CB_current_value = 1.0; /*m44*/
	self.Int+=2;
	self.Float+=16;


	return 0;
}


/************************ Behavior function *************************/

int motion_lib__weighingscale(void)
{
	if (exec_term_node(self_num)) {

		/*********************** Internals variables ************************/

		weighingscale_status_act_prod = self.Int+0;
		weighingscale_m11 = self.Float+0;
		weighingscale_m12 = self.Float+1;
		weighingscale_m13 = self.Float+2;
		weighingscale_m14 = self.Float+3;
		weighingscale_m21 = self.Float+4;
		weighingscale_m22 = self.Float+5;
		weighingscale_m23 = self.Float+6;
		weighingscale_m24 = self.Float+7;
		weighingscale_m31 = self.Float+8;
		weighingscale_m32 = self.Float+9;
		weighingscale_m33 = self.Float+10;
		weighingscale_m34 = self.Float+11;
		weighingscale_m41 = self.Float+12;
		weighingscale_m42 = self.Float+13;
		weighingscale_m43 = self.Float+14;
		weighingscale_m44 = self.Float+15;
		weighingscale_weight = self.Int+1;

		/********************* Pre-conditions execution *********************/

		/*no Pre-conditions*/

		/************************ Actor Myself init *************************/

		setMyselfFor(self_num);

		/************************ Behavior execution ************************/

		{
			Weight = weight;
			weight = 0;
		}

		CB_post_int(weighingscale_Weight);

		/******************** Post-conditions execution *********************/

		/*no Post-conditions*/
	}

	/*************************** self update ****************************/

	self.Int += 2;
	self.Float += 16;

	return 0;
}

