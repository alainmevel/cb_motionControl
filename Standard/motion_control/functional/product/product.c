#define mx_self_is_prod
#include "cb_comp.h"
#include "iec_1131.h"
#ifndef GEN_EMBEDDED
#include "product.h"
#else
#include "motion_control_product.h"
#endif

extern int raise_debug_break(int cpos, int prePostCode, int typeOfBreak, int rank);
#if defined(mx_use_motion_lib__qrreader) | defined(mx_verif)
#ifndef GEN_EMBEDDED
#include "motion_lib/functional/qrreader/qrreader.h"
#else
#include "motion_lib_qrreader.h"
#endif
#endif
#if defined(mx_use_motion_lib__weighingscale) | defined(mx_verif)
#ifndef GEN_EMBEDDED
#include "motion_lib/functional/weighingscale/weighingscale.h"
#else
#include "motion_lib_weighingscale.h"
#endif
#endif
#if defined(mx_use_motion_lib__pusher) | defined(mx_verif)
#ifndef GEN_EMBEDDED
#include "motion_lib/functional/pusher/pusher.h"
#else
#include "motion_lib_pusher.h"
#endif
#endif
#if defined(mx_use_motion_control__robotic_arm) | defined(mx_verif)
#ifndef GEN_EMBEDDED
#include "motion_control/functional/robotic_arm/robotic_arm.h"
#else
#include "motion_control_robotic_arm.h"
#endif
#endif
#if defined(mx_use_motion_control__product_killer) | defined(mx_verif)
#ifndef GEN_EMBEDDED
#include "motion_control/functional/product_killer/product_killer.h"
#else
#include "motion_control_product_killer.h"
#endif
#endif
#if defined(mx_use_motion_lib__conveyor) | defined(mx_verif)
#ifndef GEN_EMBEDDED
#include "motion_lib/functional/conveyor/conveyor.h"
#else
#include "motion_lib_conveyor.h"
#endif
#endif
#if defined(mx_use_motion_lib__presence_sensor) | defined(mx_verif)
#ifndef GEN_EMBEDDED
#include "motion_lib/functional/presence_sensor/presence_sensor.h"
#else
#include "motion_lib_presence_sensor.h"
#endif
#endif
#if defined(mx_use_motion_control__product) | defined(mx_verif)
#ifndef GEN_EMBEDDED
#include "motion_control/functional/product/product.h"
#else
#include "motion_control_product.h"
#endif
#endif
#if defined(mx_use_motion_control__product_creator) | defined(mx_verif)
#ifndef GEN_EMBEDDED
#include "motion_control/functional/product_creator/product_creator.h"
#else
#include "motion_control_product_creator.h"
#endif
#endif


#define DIR_FLOW		1
#define DIR_INVERSE_FLOW	-1


/*****************************************************
                             Variables
******************************************************/
CB_Index product__num = 0;
CB_Mem_Int *product_status_act_prod;
CB_Mem_Float *product_m11;
CB_Mem_Float *product_m12;
CB_Mem_Float *product_m13;
CB_Mem_Float *product_m14;
CB_Mem_Float *product_m21;
CB_Mem_Float *product_m22;
CB_Mem_Float *product_m23;
CB_Mem_Float *product_m24;
CB_Mem_Float *product_m31;
CB_Mem_Float *product_m32;
CB_Mem_Float *product_m33;
CB_Mem_Float *product_m34;
CB_Mem_Float *product_m41;
CB_Mem_Float *product_m42;
CB_Mem_Float *product_m43;
CB_Mem_Float *product_m44;
CB_Mem_Float *product_movementConv;
CB_Mem_Bool *product_contact;
CB_Mem_Float *product_centreRotX;
CB_Mem_Float *product_centreRotZ;
CB_Mem_Float *product_centreRotY;
CB_Mem_Int *product_moveDirection;
CB_Mem_Int *product_idConv;
CB_Mem_Bool *product_moveAuthorization;
CB_Mem_Int *product_number;
CB_Mem_Int *product_idProdBelow;
CB_Mem_Int *product_prodId;
CB_Mem_Float *product_oldOtherPx;
CB_Mem_Float *product_oldOtherPy;
CB_Mem_Float *product_oldOtherPz;
CB_Mem_Float *product_oldOtherRx;
CB_Mem_Float *product_oldOtherRy;
CB_Mem_Float *product_oldOtherRz;
CB_Mem_Float *product_gravitySpeed;
CB_Mem_Int *product_oldIdConv;
CB_Mem_Int *product_display1;
CB_Mem_Int *product_display2;
CB_Mem_Int *product_display3;
CB_Mem_Int *product_display4;
CB_Mem_Int *product_display5;
CB_Mem_Int *product_display6;
CB_Mem_Int *product_display7;
CB_Mem_Int *product_display8;
CB_Mem_Int *product_display9;
CB_Mem_Int *product_display10;
CB_Mem_Float *product_oPx;
CB_Mem_Float *product_oPy;
CB_Mem_Float *product_oPz;
CB_Mem_Float *product_oAx;
CB_Mem_Float *product_oAy;
CB_Mem_Float *product_oAz;
CB_Mem_Bool *product_armActive;
CB_Mem_Bool *product_memArmActive;
CB_Mem_Float *product_refAngZ;
CB_Mem_Float *product_refAngY;
CB_Mem_Float *product_refAngX;
CB_Mem_Float *product_refZ;
CB_Mem_Float *product_refY;
CB_Mem_Float *product_refX;
CB_Mem_Float *product_refSizeOthX;
CB_Mem_Float *product_refAngOthZ;
CB_Mem_Float *product_refAngOthY;
CB_Mem_Float *product_refAngOthX;
CB_Mem_Float *product_refOthZ;
CB_Mem_Float *product_refOthY;
CB_Mem_Float *product_refOthX;
CB_Mem_Bool *product_empty1;
CB_Mem_Bool *product_empty2;
CB_Mem_Bool *product_empty3;
CB_Mem_Bool *product_empty4;
CB_Mem_Bool *product_empty5;
CB_Mem_Bool *product_empty6;
CB_Mem_Bool *product_empty7;
CB_Mem_Bool *product_empty8;
CB_Mem_Int *product_weight;
extern int compteur_char_alloc;
extern int compteur_booleen_alloc;
extern int compteur_entier_alloc;
extern int compteur_reel_alloc;

/*****************************************************
                             Variables
******************************************************/
#define status_act_prod (*(long *)(&(product_status_act_prod->CB_current_value)))
#define m11 (product_m11->CB_current_value)
#define m12 (product_m12->CB_current_value)
#define m13 (product_m13->CB_current_value)
#define m14 (product_m14->CB_current_value)
#define m21 (product_m21->CB_current_value)
#define m22 (product_m22->CB_current_value)
#define m23 (product_m23->CB_current_value)
#define m24 (product_m24->CB_current_value)
#define m31 (product_m31->CB_current_value)
#define m32 (product_m32->CB_current_value)
#define m33 (product_m33->CB_current_value)
#define m34 (product_m34->CB_current_value)
#define m41 (product_m41->CB_current_value)
#define m42 (product_m42->CB_current_value)
#define m43 (product_m43->CB_current_value)
#define m44 (product_m44->CB_current_value)
#define movementConv (product_movementConv->CB_current_value)
#define contact (product_contact->CB_current_value)
#define centreRotX (product_centreRotX->CB_current_value)
#define centreRotZ (product_centreRotZ->CB_current_value)
#define centreRotY (product_centreRotY->CB_current_value)
#define moveDirection (*(short *)(&(product_moveDirection->CB_current_value)))
#define idConv (*(long *)(&(product_idConv->CB_current_value)))
#define moveAuthorization (product_moveAuthorization->CB_current_value)
#define number (*(short *)(&(product_number->CB_current_value)))
#define idProdBelow (*(long *)(&(product_idProdBelow->CB_current_value)))
#define prodId (*(short *)(&(product_prodId->CB_current_value)))
#define oldOtherPx (product_oldOtherPx->CB_current_value)
#define oldOtherPy (product_oldOtherPy->CB_current_value)
#define oldOtherPz (product_oldOtherPz->CB_current_value)
#define oldOtherRx (product_oldOtherRx->CB_current_value)
#define oldOtherRy (product_oldOtherRy->CB_current_value)
#define oldOtherRz (product_oldOtherRz->CB_current_value)
#define gravitySpeed (product_gravitySpeed->CB_current_value)
#define oldIdConv (*(long *)(&(product_oldIdConv->CB_current_value)))
#define display1 (*(short *)(&(product_display1->CB_current_value)))
#define display2 (*(short *)(&(product_display2->CB_current_value)))
#define display3 (*(short *)(&(product_display3->CB_current_value)))
#define display4 (*(short *)(&(product_display4->CB_current_value)))
#define display5 (*(short *)(&(product_display5->CB_current_value)))
#define display6 (*(short *)(&(product_display6->CB_current_value)))
#define display7 (*(short *)(&(product_display7->CB_current_value)))
#define display8 (*(short *)(&(product_display8->CB_current_value)))
#define display9 (*(short *)(&(product_display9->CB_current_value)))
#define display10 (*(short *)(&(product_display10->CB_current_value)))
#define oPx (product_oPx->CB_current_value)
#define oPy (product_oPy->CB_current_value)
#define oPz (product_oPz->CB_current_value)
#define oAx (product_oAx->CB_current_value)
#define oAy (product_oAy->CB_current_value)
#define oAz (product_oAz->CB_current_value)
#define armActive (product_armActive->CB_current_value)
#define memArmActive (product_memArmActive->CB_current_value)
#define refAngZ (product_refAngZ->CB_current_value)
#define refAngY (product_refAngY->CB_current_value)
#define refAngX (product_refAngX->CB_current_value)
#define refZ (product_refZ->CB_current_value)
#define refY (product_refY->CB_current_value)
#define refX (product_refX->CB_current_value)
#define refSizeOthX (product_refSizeOthX->CB_current_value)
#define refAngOthZ (product_refAngOthZ->CB_current_value)
#define refAngOthY (product_refAngOthY->CB_current_value)
#define refAngOthX (product_refAngOthX->CB_current_value)
#define refOthZ (product_refOthZ->CB_current_value)
#define refOthY (product_refOthY->CB_current_value)
#define refOthX (product_refOthX->CB_current_value)
#define empty1 (product_empty1->CB_current_value)
#define empty2 (product_empty2->CB_current_value)
#define empty3 (product_empty3->CB_current_value)
#define empty4 (product_empty4->CB_current_value)
#define empty5 (product_empty5->CB_current_value)
#define empty6 (product_empty6->CB_current_value)
#define empty7 (product_empty7->CB_current_value)
#define empty8 (product_empty8->CB_current_value)
#define weight (*(short *)(&(product_weight->CB_current_value)))

/*****************************************************
                             Initialization function
******************************************************/
void motion_control__product_init()
{
	(self.Float + 16)->CB_current_value = 0.0; /*movementConv*/;
	(self.Bool + 0)->CB_current_value = 0; /*contact*/;
	(self.Float + 17)->CB_current_value = 0.0; /*centreRotX*/;
	(self.Float + 18)->CB_current_value = 0.0; /*centreRotZ*/;
	(self.Float + 19)->CB_current_value = 0.0; /*centreRotY*/;
	(self.Int + 1)->CB_current_value = 0; /*moveDirection*/;
	(self.Int + 2)->CB_current_value = 0; /*idConv*/;
	(self.Bool + 1)->CB_current_value = 0; /*moveAuthorization*/;
	(self.Int + 3)->CB_current_value = 0; /*number*/;
	(self.Int + 4)->CB_current_value = 0; /*idProdBelow*/;
	(self.Int + 5)->CB_current_value = 0; /*prodId*/;
	(self.Float + 20)->CB_current_value = 0.0; /*oldOtherPx*/;
	(self.Float + 21)->CB_current_value = 0.0; /*oldOtherPy*/;
	(self.Float + 22)->CB_current_value = 0.0; /*oldOtherPz*/;
	(self.Float + 23)->CB_current_value = 0.0; /*oldOtherRx*/;
	(self.Float + 24)->CB_current_value = 0.0; /*oldOtherRy*/;
	(self.Float + 25)->CB_current_value = 0.0; /*oldOtherRz*/;
	(self.Float + 26)->CB_current_value = 0.0; /*gravitySpeed*/;
	(self.Int + 6)->CB_current_value = 0; /*oldIdConv*/;
	(self.Int + 7)->CB_current_value = 0; /*display1*/;
	(self.Int + 8)->CB_current_value = 0; /*display2*/;
	(self.Int + 9)->CB_current_value = 0; /*display3*/;
	(self.Int + 10)->CB_current_value = 0; /*display4*/;
	(self.Int + 11)->CB_current_value = 0; /*display5*/;
	(self.Int + 12)->CB_current_value = 0; /*display6*/;
	(self.Int + 13)->CB_current_value = 0; /*display7*/;
	(self.Int + 14)->CB_current_value = 0; /*display8*/;
	(self.Int + 15)->CB_current_value = 0; /*display9*/;
	(self.Int + 16)->CB_current_value = 0; /*display10*/;
	(self.Float + 27)->CB_current_value = 0.0; /*oPx*/;
	(self.Float + 28)->CB_current_value = 0.0; /*oPy*/;
	(self.Float + 29)->CB_current_value = 0.0; /*oPz*/;
	(self.Float + 30)->CB_current_value = 0.0; /*oAx*/;
	(self.Float + 31)->CB_current_value = 0.0; /*oAy*/;
	(self.Float + 32)->CB_current_value = 0.0; /*oAz*/;
	(self.Bool + 2)->CB_current_value = 0; /*armActive*/;
	(self.Bool + 3)->CB_current_value = 0; /*memArmActive*/;
	(self.Float + 33)->CB_current_value = 0.0; /*refAngZ*/;
	(self.Float + 34)->CB_current_value = 0.0; /*refAngY*/;
	(self.Float + 35)->CB_current_value = 0.0; /*refAngX*/;
	(self.Float + 36)->CB_current_value = 0.0; /*refZ*/;
	(self.Float + 37)->CB_current_value = 0.0; /*refY*/;
	(self.Float + 38)->CB_current_value = 0.0; /*refX*/;
	(self.Float + 39)->CB_current_value = 0.0; /*refSizeOthX*/;
	(self.Float + 40)->CB_current_value = 0.0; /*refAngOthZ*/;
	(self.Float + 41)->CB_current_value = 0.0; /*refAngOthY*/;
	(self.Float + 42)->CB_current_value = 0.0; /*refAngOthX*/;
	(self.Float + 43)->CB_current_value = 0.0; /*refOthZ*/;
	(self.Float + 44)->CB_current_value = 0.0; /*refOthY*/;
	(self.Float + 45)->CB_current_value = 0.0; /*refOthX*/;
	(self.Bool + 4)->CB_current_value = 0; /*empty1*/;
	(self.Bool + 5)->CB_current_value = 0; /*empty2*/;
	(self.Bool + 6)->CB_current_value = 0; /*empty3*/;
	(self.Bool + 7)->CB_current_value = 0; /*empty4*/;
	(self.Bool + 8)->CB_current_value = 0; /*empty5*/;
	(self.Bool + 9)->CB_current_value = 0; /*empty6*/;
	(self.Bool + 10)->CB_current_value = 0; /*empty7*/;
	(self.Bool + 11)->CB_current_value = 0; /*empty8*/;
	(self.Int + 17)->CB_current_value = 0; /*weight*/;
}

int motion_control__product()
{

/*****************************************************
                             Internals
******************************************************/
	product_status_act_prod = self.Int + 0;
	product_m11 = self.Float + 0;
	product_m12 = self.Float + 1;
	product_m13 = self.Float + 2;
	product_m14 = self.Float + 3;
	product_m21 = self.Float + 4;
	product_m22 = self.Float + 5;
	product_m23 = self.Float + 6;
	product_m24 = self.Float + 7;
	product_m31 = self.Float + 8;
	product_m32 = self.Float + 9;
	product_m33 = self.Float + 10;
	product_m34 = self.Float + 11;
	product_m41 = self.Float + 12;
	product_m42 = self.Float + 13;
	product_m43 = self.Float + 14;
	product_m44 = self.Float + 15;
	product_movementConv = self.Float + 16;
	product_contact = self.Bool + 0;
	product_centreRotX = self.Float + 17;
	product_centreRotZ = self.Float + 18;
	product_centreRotY = self.Float + 19;
	product_moveDirection = self.Int + 1;
	product_idConv = self.Int + 2;
	product_moveAuthorization = self.Bool + 1;
	product_number = self.Int + 3;
	product_idProdBelow = self.Int + 4;
	product_prodId = self.Int + 5;
	product_oldOtherPx = self.Float + 20;
	product_oldOtherPy = self.Float + 21;
	product_oldOtherPz = self.Float + 22;
	product_oldOtherRx = self.Float + 23;
	product_oldOtherRy = self.Float + 24;
	product_oldOtherRz = self.Float + 25;
	product_gravitySpeed = self.Float + 26;
	product_oldIdConv = self.Int + 6;
	product_display1 = self.Int + 7;
	product_display2 = self.Int + 8;
	product_display3 = self.Int + 9;
	product_display4 = self.Int + 10;
	product_display5 = self.Int + 11;
	product_display6 = self.Int + 12;
	product_display7 = self.Int + 13;
	product_display8 = self.Int + 14;
	product_display9 = self.Int + 15;
	product_display10 = self.Int + 16;
	product_oPx = self.Float + 27;
	product_oPy = self.Float + 28;
	product_oPz = self.Float + 29;
	product_oAx = self.Float + 30;
	product_oAy = self.Float + 31;
	product_oAz = self.Float + 32;
	product_armActive = self.Bool + 2;
	product_memArmActive = self.Bool + 3;
	product_refAngZ = self.Float + 33;
	product_refAngY = self.Float + 34;
	product_refAngX = self.Float + 35;
	product_refZ = self.Float + 36;
	product_refY = self.Float + 37;
	product_refX = self.Float + 38;
	product_refSizeOthX = self.Float + 39;
	product_refAngOthZ = self.Float + 40;
	product_refAngOthY = self.Float + 41;
	product_refAngOthX = self.Float + 42;
	product_refOthZ = self.Float + 43;
	product_refOthY = self.Float + 44;
	product_refOthX = self.Float + 45;
	product_empty1 = self.Bool + 4;
	product_empty2 = self.Bool + 5;
	product_empty3 = self.Bool + 6;
	product_empty4 = self.Bool + 7;
	product_empty5 = self.Bool + 8;
	product_empty6 = self.Bool + 9;
	product_empty7 = self.Bool + 10;
	product_empty8 = self.Bool + 11;
	product_weight = self.Int + 17;
	
	if (prodGetMustInit(myself)) {
		printf("Laaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa\n");
		motion_control__product_init();
		prodSetMustInit(myself, 0);
	}
/*****************************************************
                             Component user code
******************************************************/
	if(otherModel == __prelude) {
			/* ----------------------------------------------------------------	*/
			/* Initialisation of local variables of the instance of the product	*/
			/* ----------------------------------------------------------------	*/
			weight =0;
			movementConv = 0.0;
			contact = 0;
			moveAuthorization = 1;
			idConv=-1;
			armActive = 0;
			
			if (!empty1) weight +=15;
			if (!empty2) weight +=15;
			if (!empty3) weight +=15;
			if (!empty4) weight +=15;
			if (!empty5) weight +=15;
			if (!empty6) weight +=15;
			if (!empty7) weight +=15;
			if (!empty8) weight +=15;
			
			
		return;
	}

	if(otherModel == __postlude) {
			/* ----------------------------------------------------------------	*/
			/* If there is no contact, the box falls down thanks to gravity	*/	
			/* ----------------------------------------------------------------	*/
			if (contact == 0) 
				moveZBy(myself, gravitySpeed*-1.0);
			
			
			/* ----------------------------------------------------------------	*/
			/* Shift on conveyors if authorization					*/
			/* ----------------------------------------------------------------	*/
			if (moveAuthorization == 1 && (idConv>=0)) 
				moveRelativeAnotherBy(myself, idConv, movementConv, 0.0, 0.0);
			
			
			/* ----------------------------------------------------------------	*/
			/* Reset memories								*/
			/* ----------------------------------------------------------------	*/
			if (contact == 0) {
				moveDirection = 0;
				idProdBelow = -1;
			}
			
			memArmActive = armActive;
		return;
	}

#if defined(mx_use_motion_lib__qrreader) | defined(mx_verif)
#ifdef SY_MSW
#define otherVar(var) (otherPointers.mx_acces_motion_lib__qrreader__##var)
#else
#define otherVar(var) (otherPointers.mx_acces_motion_lib__qrreader__/**/var)
#endif
	if(otherModel == motion_lib__qrreader) {
			otherVar(number) = number;
		return;
	}
#undef otherVar
#endif

#if defined(mx_use_motion_lib__weighingscale) | defined(mx_verif)
#ifdef SY_MSW
#define otherVar(var) (otherPointers.mx_acces_motion_lib__weighingscale__##var)
#else
#define otherVar(var) (otherPointers.mx_acces_motion_lib__weighingscale__/**/var)
#endif
	if(otherModel == motion_lib__weighingscale) {
			otherVar(weight) = weight;
		return;
	}
#undef otherVar
#endif

#if defined(mx_use_motion_lib__pusher) | defined(mx_verif)
#ifdef SY_MSW
#define otherVar(var) (otherPointers.mx_acces_motion_lib__pusher__##var)
#else
#define otherVar(var) (otherPointers.mx_acces_motion_lib__pusher__/**/var)
#endif
	if(otherModel == motion_lib__pusher) {
			moveBy(myself , otherVar(moveX1),otherVar(moveY1) , otherVar(moveZ1));
		return;
	}
#undef otherVar
#endif

#if defined(mx_use_motion_control__robotic_arm) | defined(mx_verif)
#ifdef SY_MSW
#define otherVar(var) (otherPointers.mx_acces_motion_control__robotic_arm__##var)
#else
#define otherVar(var) (otherPointers.mx_acces_motion_control__robotic_arm__/**/var)
#endif
	if(otherModel == motion_control__robotic_arm) {
			armActive = otherVar(functionActiveLocale);
			if (armActive && !memArmActive ) {
			
			// --- take arm/product relative position reference ---
				refOthX = getPositionX(other);
				refOthY = getPositionY(other);
				refOthZ = getPositionZ(other);
				refAngOthX = getAngleX(other);
				refAngOthY = getAngleY(other);
				refAngOthZ = getAngleZ(other);
				refSizeOthX = getSizeX(other);
				
				// move myself to the arm space reference 
				moveBy(myself, -refOthX, -refOthY, -refOthZ);
			
				rotateZBy(myself,-refAngOthZ,0.0, 0.0, 0.0);
				rotateYBy(myself,-refAngOthY,0.0, 0.0, 0.0);
				rotateXBy(myself,-refAngOthX,0.0, 0.0, 0.0);
			
				refX = getPositionX(myself);
				refY = getPositionY(myself);
				refZ = getPositionZ(myself);
				refAngX = getAngleX(myself);
				refAngY = getAngleY(myself);
				refAngZ = getAngleZ(myself);
			
				rotateBy(myself,refAngOthX,refAngOthY, refAngOthZ, 0.0, 0.0, 0.0); // relocate the product
				moveBy(myself, refOthX, refOthY, refOthZ);
			}
			
			if (armActive && memArmActive) {
				rotateTo(myself,refAngX,refAngY,refAngZ, getPositionX(myself), getPositionY(myself), getPositionZ(myself));
				moveTo(myself, refX, refY, refZ);
				rotateBy(myself,getAngleX(other),getAngleY(other), getAngleZ(other), 0.0, 0.0, 0.0);
				moveBy(myself, getPositionX(other), getPositionY(other), getPositionZ(other));
			
			}
		return;
	}
#undef otherVar
#endif

#if defined(mx_use_motion_control__product_killer) | defined(mx_verif)
#ifdef SY_MSW
#define otherVar(var) (otherPointers.mx_acces_motion_control__product_killer__##var)
#else
#define otherVar(var) (otherPointers.mx_acces_motion_control__product_killer__/**/var)
#endif
	if(otherModel == motion_control__product_killer) {
			/* ----------------------------------------------------------------	*/
			/* Destruction of the current instance of the product if the killer	*/
			/* is active.									*/
			/* ----------------------------------------------------------------	*/
			
			if (otherVar(isActive)) prodKill(myself);
		return;
	}
#undef otherVar
#endif

#if defined(mx_use_motion_lib__conveyor) | defined(mx_verif)
#ifdef SY_MSW
#define otherVar(var) (otherPointers.mx_acces_motion_lib__conveyor__##var)
#else
#define otherVar(var) (otherPointers.mx_acces_motion_lib__conveyor__/**/var)
#endif
	if(otherModel == motion_lib__conveyor) {
			/* ----------------------------------------------------------------	*/
			/* Interaction between the product and conveyors			*/
			/* Convention : a conveyor is always oriented so that its X-axis is	*/
			/*	  	  in the same direction as the flow			*/
			/* ----------------------------------------------------------------	*/
			
			/* ----------------------------------------------------------------	*/
			/* We retrieve the conveyor's data if the center of the product is	*/
			/* included in the volume of the conveyor.				*/
			/* ----------------------------------------------------------------	*/
			if ((idConv == -1) || isMyCenterIncludedInXYOfOther(myself, other)) {
				/* Traverse speed on the conveyor	*/
				movementConv = otherVar(incShiftProduct) ;
			
				/* Keep trace of traverse direction of the conveyor		*/
				if (movementConv  > 0.0) moveDirection = DIR_FLOW;
				if (movementConv  < 0.0) moveDirection = DIR_INVERSE_FLOW;
			
				/* Get the identifier of the conveyor in order to be		*/
				/* able to shift the product in its coordinate system		*/
				idConv = other;
				oldIdConv = idConv;
			
				/* If the conveyor is a traverser, a lift, ...  		*/
				/* the product msut follow it	 				*/
				if (otherVar(incrementPositionX) != 0.0) moveXBy(myself, otherVar(incrementPositionX));
				if (otherVar(incrementPositionY) != 0.0) moveYBy(myself, otherVar(incrementPositionY));
				if (otherVar(incrementPositionZ) != 0.0) alignTwoObjectsRelativeThirdZ(myself, other, other);
			
				/* Management of the rotation of the product on a 		*/
				/* swivelling table, rotary plate ring, ...			*/
				/* Keep trace of the rotation center in order to be 		*/
				/* able to give them to products that could be stacked		*/
				/* above this instance						*/
				centreRotX = otherVar(centerRotX);
				centreRotY = otherVar(centerRotY);
				centreRotZ = otherVar(centerRotZ);
			
				if ((otherVar(incRotX) != 0.0) || (otherVar(incRotY)!= 0) || (otherVar(incRotZ)!= 0.0))
					rotateBy(myself, otherVar(incRotX), otherVar(incRotY), otherVar(incRotZ), centreRotX, centreRotY, centreRotZ);  
			
			}
			
			/* ----------------------------------------------------------------	*/
			/* The product lays on a solid						*/
			/* ----------------------------------------------------------------	*/
			contact = 1;
			
		return;
	}
#undef otherVar
#endif

#if defined(mx_use_motion_lib__presence_sensor) | defined(mx_verif)
#ifdef SY_MSW
#define otherVar(var) (otherPointers.mx_acces_motion_lib__presence_sensor__##var)
#else
#define otherVar(var) (otherPointers.mx_acces_motion_lib__presence_sensor__/**/var)
#endif
	if(otherModel == motion_lib__presence_sensor) {
			/* ----------------------------------------------------------------	*/
			/* Information to the sensor that a product is in its detection area	*/
			/* ----------------------------------------------------------------	*/
			
			otherVar(detection) = 1;
			
		return;
	}
#undef otherVar
#endif

#if defined(mx_use_motion_control__product) | defined(mx_verif)
#ifdef SY_MSW
#define otherVar(var) (otherPointers.mx_acces_motion_control__product__##var)
#else
#define otherVar(var) (otherPointers.mx_acces_motion_control__product__/**/var)
#endif
	if(otherModel == motion_control__product) {
			/* ----------------------------------------------------------------	*/
			/* Interaction between two products	 				*/
			/* ----------------------------------------------------------------	*/
			
			
			/* Verify that the product whose position has to be updated is 	*/
			/* located on one side of the current instance and not above or 	*/
			/* below it.									*/
			if ((getCenterZ(myself) > getCenterZ(other) - (getSizeZ(other)/2.0)) && (getCenterZ(myself) < getCenterZ(other)+(getSizeZ(other)/2.0))) {
				/* Update the position of the product that is behind the 	*/
				/* other according to the shift direction				*/
				if ( (moveDirection == DIR_FLOW && (getCenterXInAnother(myself, idConv) < getCenterXInAnother(other, idConv))) 
					|| (moveDirection == DIR_INVERSE_FLOW && (getCenterXInAnother(myself, idConv) > getCenterXInAnother(other, idConv))) ) {
			
					/* Align the 2 products one behind the other if its speed is lower 		*/
					if (fabs(movementConv ) > fabs(otherVar(movementConv )))
						alignTwoObjectsRelativeThirdX(myself, other, idConv);
			
					/* Forbid the shift						*/
					moveAuthorization=0;
				 }
			}
			
			/* Interaction with the product below					*/
			if ((getCenterZ(myself)  > getCenterZ(other)+ (getSizeZ(other) / 2)) 
				&&  ((getCenterX(myself) > getCenterX(other) - (getSizeX(other)/4.0)) && (getCenterX(myself) < getCenterX(other)+(getSizeX(other)/4.0)))
				&&  ((getCenterY(myself) > getCenterY(other) - (getSizeY(other)/4.0)) && (getCenterY(myself) < getCenterY(other)+(getSizeY(other)/4.0))))  {
				/* Same movement than the product below				*/
				if (idProdBelow == other) {
					moveBy(myself, getPositionX(other)-oldOtherPx, getPositionY(other)-oldOtherPy, getPositionZ(other)-oldOtherPz);
					rotateBy(myself, getAngleX(other)-oldOtherRx, getAngleY(other)-oldOtherRy, getAngleZ(other)-oldOtherRz,
							otherVar(centreRotX), otherVar(centreRotY), otherVar(centreRotZ));
				}
				oldOtherPx=getPositionX(other); oldOtherPy=getPositionY(other); oldOtherPz=getPositionZ(other);
				oldOtherRx=getAngleX(other); oldOtherRy=getAngleY(other); oldOtherRz=getAngleZ(other);
				centreRotX=otherVar(centreRotX); centreRotY=otherVar(centreRotY); centreRotZ=otherVar(centreRotZ);
			
				idProdBelow = other;
			
				/* The product lays on a solid					*/
				contact = 1;
			
				/* The next line of code can be added in order to align 	*/
				/* correctly  the products one on the others ...		*/
				/* We should align both products in Z-axis according to the 	*/
				/* absolute coordinate system and not according to the 	*/
				/* relative system coordinate in order to have a generic 	*/
				/* behaviour.								*/
				/* => adapt this according to the application when needed	*/
			
				/* alignTwoObjectsRelativeThirdZ(myself, other, other); 	*/
				
			}
			
		return;
	}
#undef otherVar
#endif

#if defined(mx_use_motion_control__product_creator) | defined(mx_verif)
#ifdef SY_MSW
#define otherVar(var) (otherPointers.mx_acces_motion_control__product_creator__##var)
#else
#define otherVar(var) (otherPointers.mx_acces_motion_control__product_creator__/**/var)
#endif
	if(otherModel == motion_control__product_creator) {
			/* ----------------------------------------------------------------	*/
			/* Information to the creation module that a product is in its area	*/
			/* ----------------------------------------------------------------	*/
			 
			otherVar(productPresence) = 1;
			
		return;
	}
#undef otherVar
#endif

	free_all_alloc();
	return 0;
}
#undef status_act_prod
#undef m11
#undef m12
#undef m13
#undef m14
#undef m21
#undef m22
#undef m23
#undef m24
#undef m31
#undef m32
#undef m33
#undef m34
#undef m41
#undef m42
#undef m43
#undef m44
#undef movementConv
#undef contact
#undef centreRotX
#undef centreRotZ
#undef centreRotY
#undef moveDirection
#undef idConv
#undef moveAuthorization
#undef number
#undef idProdBelow
#undef prodId
#undef oldOtherPx
#undef oldOtherPy
#undef oldOtherPz
#undef oldOtherRx
#undef oldOtherRy
#undef oldOtherRz
#undef gravitySpeed
#undef oldIdConv
#undef display1
#undef display2
#undef display3
#undef display4
#undef display5
#undef display6
#undef display7
#undef display8
#undef display9
#undef display10
#undef oPx
#undef oPy
#undef oPz
#undef oAx
#undef oAy
#undef oAz
#undef armActive
#undef memArmActive
#undef refAngZ
#undef refAngY
#undef refAngX
#undef refZ
#undef refY
#undef refX
#undef refSizeOthX
#undef refAngOthZ
#undef refAngOthY
#undef refAngOthX
#undef refOthZ
#undef refOthY
#undef refOthX
#undef empty1
#undef empty2
#undef empty3
#undef empty4
#undef empty5
#undef empty6
#undef empty7
#undef empty8
#undef weight


int motion_control__product__test_interarctions()
{
#if defined(mx_use_motion_lib__qrreader) | defined(mx_verif)
	if(otherModel == motion_lib__qrreader) {
		return 1;
	}
#endif

#if defined(mx_use_motion_lib__weighingscale) | defined(mx_verif)
	if(otherModel == motion_lib__weighingscale) {
		return 1;
	}
#endif

#if defined(mx_use_motion_lib__pusher) | defined(mx_verif)
	if(otherModel == motion_lib__pusher) {
		return 1;
	}
#endif

#if defined(mx_use_motion_control__robotic_arm) | defined(mx_verif)
	if(otherModel == motion_control__robotic_arm) {
		return 1;
	}
#endif

#if defined(mx_use_motion_control__product_killer) | defined(mx_verif)
	if(otherModel == motion_control__product_killer) {
		return 1;
	}
#endif

#if defined(mx_use_motion_lib__conveyor) | defined(mx_verif)
	if(otherModel == motion_lib__conveyor) {
		return 1;
	}
#endif

#if defined(mx_use_motion_lib__presence_sensor) | defined(mx_verif)
	if(otherModel == motion_lib__presence_sensor) {
		return 1;
	}
#endif

#if defined(mx_use_motion_control__product) | defined(mx_verif)
	if(otherModel == motion_control__product) {
		return 1;
	}
#endif

#if defined(mx_use_motion_control__product_creator) | defined(mx_verif)
	if(otherModel == motion_control__product_creator) {
		return 1;
	}
#endif

	return 0;
}
