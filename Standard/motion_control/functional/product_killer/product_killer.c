/*2023-10-04T11:03:56-01:00*/

/********************************************************************
 * product_killer.c
 * 
 * generated by: plcgen 3.1.391
 ********************************************************************/

#include "cb_comp.h"
#include "iec_1131.h"
#include "cb_products_iec.h"

#include "product_killer.h"



/**************************** Variables *****************************/

CB_Index product_killer__num = 0;
CB_Mem_Int *product_killer_status_act_prod;
CB_Mem_Float *product_killer_m11;
CB_Mem_Float *product_killer_m12;
CB_Mem_Float *product_killer_m13;
CB_Mem_Float *product_killer_m14;
CB_Mem_Float *product_killer_m21;
CB_Mem_Float *product_killer_m22;
CB_Mem_Float *product_killer_m23;
CB_Mem_Float *product_killer_m24;
CB_Mem_Float *product_killer_m31;
CB_Mem_Float *product_killer_m32;
CB_Mem_Float *product_killer_m33;
CB_Mem_Float *product_killer_m34;
CB_Mem_Float *product_killer_m41;
CB_Mem_Float *product_killer_m42;
CB_Mem_Float *product_killer_m43;
CB_Mem_Float *product_killer_m44;
CB_Mem_Bool *product_killer_isActive;

/**************************** Variables *****************************/

#define status_act_prod (product_killer_status_act_prod->CB_current_value)
#define m11 (product_killer_m11->CB_current_value)
#define m12 (product_killer_m12->CB_current_value)
#define m13 (product_killer_m13->CB_current_value)
#define m14 (product_killer_m14->CB_current_value)
#define m21 (product_killer_m21->CB_current_value)
#define m22 (product_killer_m22->CB_current_value)
#define m23 (product_killer_m23->CB_current_value)
#define m24 (product_killer_m24->CB_current_value)
#define m31 (product_killer_m31->CB_current_value)
#define m32 (product_killer_m32->CB_current_value)
#define m33 (product_killer_m33->CB_current_value)
#define m34 (product_killer_m34->CB_current_value)
#define m41 (product_killer_m41->CB_current_value)
#define m42 (product_killer_m42->CB_current_value)
#define m43 (product_killer_m43->CB_current_value)
#define m44 (product_killer_m44->CB_current_value)
#define isActive (product_killer_isActive->CB_current_value)


/************************ Components offsets ************************/


/********************* Initialization function **********************/

int _motion_control__product_killer_init(void)
{
	(self.Float+0)->CB_current_value = 1.0; /*m11*/
	(self.Float+5)->CB_current_value = 1.0; /*m22*/
	(self.Float+10)->CB_current_value = 1.0; /*m33*/
	(self.Float+15)->CB_current_value = 1.0; /*m44*/
	(self.Bool+0)->CB_current_value = 1; /*isActive*/
	self.Bool+=1;
	self.Int+=1;
	self.Float+=16;


	return 0;
}


/************************ Behavior function *************************/

int motion_control__product_killer(void)
{
	if (exec_term_node(self_num)) {

		/*********************** Internals variables ************************/

		product_killer_status_act_prod = self.Int+0;
		product_killer_m11 = self.Float+0;
		product_killer_m12 = self.Float+1;
		product_killer_m13 = self.Float+2;
		product_killer_m14 = self.Float+3;
		product_killer_m21 = self.Float+4;
		product_killer_m22 = self.Float+5;
		product_killer_m23 = self.Float+6;
		product_killer_m24 = self.Float+7;
		product_killer_m31 = self.Float+8;
		product_killer_m32 = self.Float+9;
		product_killer_m33 = self.Float+10;
		product_killer_m34 = self.Float+11;
		product_killer_m41 = self.Float+12;
		product_killer_m42 = self.Float+13;
		product_killer_m43 = self.Float+14;
		product_killer_m44 = self.Float+15;
		product_killer_isActive = self.Bool+0;

		/********************* Pre-conditions execution *********************/

		/*no Pre-conditions*/

		/************************ Actor Myself init *************************/

		setMyselfFor(self_num);

		/************************ Behavior execution ************************/

		{
			/* ----------------------------------------------------------------	*/
			/* Type : Model of Operative Behaviour					*/
			/* Category :  Actors and Products						*/
			/* Author : Dassault Systemes						*/
			/* Update date : June 2018							*/
			/* ----------------------------------------------------------------	*/
			/* This module simulates a product killer.				*/
			/* ----------------------------------------------------------------	*/
		}


		/******************** Post-conditions execution *********************/

		/*no Post-conditions*/
	}

	/*************************** self update ****************************/

	self.Bool += 1;
	self.Int += 1;
	self.Float += 16;

	return 0;
}

