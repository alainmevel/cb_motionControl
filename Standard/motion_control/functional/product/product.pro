UTF8b#      TNI Sos produit   �   �

      2��~�Ox�hP]d�9�                                                   product           (         	                                    !   #   %   '   )   +   -   /   1   3   5   7   9   ;   =   ?   A   C   E   G   I   K   M   b   �   �   .This module simulates a generic product "box".                   3#define DIR_FLOW		1#define DIR_INVERSE_FLOW	-1  /* ----------------------------------------------------------------	*//* Initialisation of local variables of the instance of the product	*//* ----------------------------------------------------------------	*/weight =0;movementConv = 0.0;contact = 0;moveAuthorization = 1;idConv=-1;armActive = 0;if (!empty1) weight +=15;if (!empty2) weight +=15;if (!empty3) weight +=15;if (!empty4) weight +=15;if (!empty5) weight +=15;if (!empty6) weight +=15;if (!empty7) weight +=15;if (!empty8) weight +=15;  7/* ----------------------------------------------------------------	*//* If there is no contact, the box falls down thanks to gravity	*/	/* ----------------------------------------------------------------	*/if (contact == 0) 	moveZBy(myself, gravitySpeed*-1.0);/* ----------------------------------------------------------------	*//* Shift on conveyors if authorization					*//* ----------------------------------------------------------------	*/if (moveAuthorization == 1 && (idConv>=0)) 	moveRelativeAnotherBy(myself, idConv, movementConv, 0.0, 0.0);/* ----------------------------------------------------------------	*//* Reset memories								*//* ----------------------------------------------------------------	*/if (contact == 0) {	moveDirection = 0;	idProdBelow = -1;}memArmActive = armActive;   	   �   �   �   �   �   �   �   �   �
      w   header      3#define DIR_FLOW		1#define DIR_INVERSE_FLOW	-1                    
      w   instanceNamingPatternProperty      <CLASS_PATTERN>                    
      w   commentPatternProperty      <CLASS_PATTERN>                    
      }��_�Ao�y�*��       false               true   true               status_act_prod               
      >����        
      }�W�xťGĒ�@T�t       false               true   true               m11               
      ?����          �?
   	   }�w�HkO(�[��d��       false               true   true               m12   
            
   
   ?����            
      }=6H���M}��^{A��g       false               true   true               m13               
      ?����            
      }r�!���C���D��       false               true   true               m14               
      ?����            
      }{��:�B�A�j�=��       false               true   true               m21               
      ?����            
      }�7���AA���j?O�       false               true   true               m22               
      ?����          �?
      }u�3u�,D�����3��       false               true   true               m23               
      ?����            
      }bs�֗O��ꍸ�Q�`       false               true   true               m24               
      ?����            
      }�/��zL0�_�uC!J       false               true   true               m31               
      ?����            
      }ֿ��@Ý��o��%�       false               true   true               m32               
      ?����            
      }��AA���5���s       false               true   true               m33               
      ?����          �?
      }(ƛ��HK��>n����Q       false               true   true               m34               
      ?����            
      }�`��P�A�%6��l�U       false               true   true               m41                
       ?����            
   !   }����hG;���Љܻ�       false               true   true               m42   "            
   "   ?����            
   #   }�(���B��پL       false               true   true               m43   $            
   $   ?����            
   %   }�Y�;@���l��.�       false               true   true               m44   &            
   &   ?����          �?
   '   }0��<�B]���.�s��       false               true   true               movementConv   (            
   (   ?����            
   )   }����M�,���]��       false               true   true               contact   *            
   *   ;����        
   +   }"d�dC~��4���       false               true   true               
centreRotX   ,            
   ,   ?����            
   -   }ٯ p�eL��6�&=}e       false               true   true               
centreRotZ   .            
   .   ?����            
   /   }���~�fL���L��̗       false               true   true               
centreRotY   0            
   0   ?����            
   1   }Ʀ��uAi�	�����       false               true   true               moveDirection   2            
   2   =����        
   3   }#[+A���;�        false               true   true               idConv   4            
   4   >����        
   5   }�ak���Gv���1���       false               true   true               moveAuthorization   6            
   6   ;����        
   7   }�sL���L��
+�ʯ       false               true   true               number   8            
   8   =����        
   9   }��&�(O����o�G�       false               true   true               idProdBelow   :            
   :   >����        
   ;   }�"6�_�M"�UC�tjV�       false               true   true               prodId   <            
   <   =����        
   =   }��+��Nؘr\.         false               true   true               
oldOtherPx   >            
   >   ?����            
   ?   }�w`�E��U\1b�m�       false               true   true               
oldOtherPy   @            
   @   ?����            
   A   }���7�HI���uu!       false               true   true               
oldOtherPz   B            
   B   ?����            
   C   }�����9DM����8�       false               true   true               
oldOtherRx   D            
   D   ?����            
   E   }�H@��J��9��@l��       false               true   true               
oldOtherRy   F            
   F   ?����            
   G   }� ��Mv�r�8��       false               true   true               
oldOtherRz   H            
   H   ?����            
   I   }0����L��)�%�dO<       false               true   true               gravitySpeed   J            
   J   ?����            
   K   }9��5�Gg��r�rP       false               true   true               	oldIdConv   L            
   L   >����        
   M   ~9|u�̦M8��sl�s�       false                       
   N   P   R   T   V   X   Z   \   ^   `   display
   N   }�q���@����	N:��       false               true   true       M       display1   O   M        
   O   =����        
   P   }	K8|�E�����Cʻ@       false               true   true       M       display2   Q   M        
   Q   =����        
   R   }��!|J��o��7L       false               true   true       M       display3   S   M        
   S   =����        
   T   }kx�G��A��`���       false               true   true       M       display4   U   M        
   U   =����        
   V   }XkE���IW���)n�I       false               true   true       M       display5   W   M        
   W   =����        
   X   }�W2��I}�6"�-�Z       false               true   true       M       display6   Y   M        
   Y   =����        
   Z   }~�귺�K��l�iL�v       false               true   true       M       display7   [   M        
   [   =����        
   \   }���"K9���b�HE       false               true   true       M       display8   ]   M        
   ]   =����        
   ^   }���j�H�d�M�/       false               true   true       M       display9   _   M        
   _   =����        
   `   }'�(@NL*�Bz��=�N       false               true   true       M       	display10   a   M        
   a   =����        
   b   ~x�p���C�d�i�l(q       false                          c   e   g   i   k   m   o   q   s   u   w   y   {   }      �   �   �   �   �   �   arm
   c   }"c@G�Yk ��       false               true   true       b       oPx   d   b        
   d   ?����            
   e   }��nh+M����q8       false               true   true       b       oPy   f   b        
   f   ?����            
   g   }F1D�F��Wc�ϘX|       false               true   true       b       oPz   h   b        
   h   ?����            
   i   }(�g1�&L�kn���}       false               true   true       b       oAx   j   b        
   j   ?����            
   k   };���J�H����u��       false               true   true       b       oAy   l   b        
   l   ?����            
   m   }c�Z��Fp��WC�w�       false               true   true       b       oAz   n   b        
   n   ?����            
   o   }֫K�G�Ln�C�X���       false               true   true       b       	armActive   p   b        
   p   ;����        
   q   }ܨ���L����3�X+       false               true   true       b       memArmActive   r   b        
   r   ;����        
   s   }c�<j[K=�#Q�����       false               true   true       b       refAngZ   t   b        
   t   ?����            
   u   }0(��B鮷E���H�       false               true   true       b       refAngY   v   b        
   v   ?����            
   w   }�`n�)N��Y�014��       false               true   true       b       refAngX   x   b        
   x   ?����            
   y   }.�uLL�����s���       false               true   true       b       refZ   z   b        
   z   ?����            
   {   }����M��W�_�u       false               true   true       b       refY   |   b        
   |   ?����            
   }   }���)�DU��Q�\�7J       false               true   true       b       refX   ~   b        
   ~   ?����            
      }�w����IU���ξ       false               true   true       b       refSizeOthX   �   b        
   �   ?����            
   �   }'0�-B%GE����c,�\       false               true   true       b       
refAngOthZ   �   b        
   �   ?����            
   �   }C��&.�@P�G��U�b�       false               true   true       b       
refAngOthY   �   b        
   �   ?����            
   �   }�6P��F���Jt(�       false               true   true       b       
refAngOthX   �   b        
   �   ?����            
   �   }GꄬOه���&�       false               true   true       b       refOthZ   �   b        
   �   ?����            
   �   })x?1�)O���g�        false               true   true       b       refOthY   �   b        
   �   ?����            
   �   }`��;`A�`�����/       false               true   true       b       refOthX   �   b        
   �   ?����            
   �   ~��E֐<�F���       false                          �   �   �   �   �   �   �   �   empty
   �   }��	s��LÑ�O<u�|       false               true   true       �       empty1   �   �        
   �   ;����        
   �   }���W�Nݛ�[s�z       false               true   true       �       empty2   �   �        
   �   ;����        
   �   }v��N}J;��P���       false               true   true       �       empty3   �   �        
   �   ;����        
   �   }��}aO���VF���       false               true   true       �       empty4   �   �        
   �   ;����        
   �   }HK1�ȑJm�H%[|��f       false               true   true       �       empty5   �   �        
   �   ;����        
   �   }��c��7D����9o�       false               true   true       �       empty6   �   �        
   �   ;����        
   �   }
���Q.I{�[����
       false               true   true       �       empty7   �   �        
   �   ;����        
   �   }6�`���I��~�
��Q       false               true   true       �       empty8   �   �        
   �   ;����        
   �   }٪�5�@�=Bb�J       false               true   true               weight   �            
   �   =����        
   �   4                       qrreader   
motion_lib   otherVar(number) = number;
   �   4                       weighingscale   
motion_lib   otherVar(weight) = weight;
   �   4                       pusher   
motion_lib   GmoveBy(myself , otherVar(moveX1),otherVar(moveY1) , otherVar(moveZ1));
   �   4                       robotic_arm   motion_control  �armActive = otherVar(functionActiveLocale);if (armActive && !memArmActive ) {// --- take arm/product relative position reference ---	refOthX = getPositionX(other);	refOthY = getPositionY(other);	refOthZ = getPositionZ(other);	refAngOthX = getAngleX(other);	refAngOthY = getAngleY(other);	refAngOthZ = getAngleZ(other);	refSizeOthX = getSizeX(other);		// move myself to the arm space reference 	moveBy(myself, -refOthX, -refOthY, -refOthZ);	rotateZBy(myself,-refAngOthZ,0.0, 0.0, 0.0);	rotateYBy(myself,-refAngOthY,0.0, 0.0, 0.0);	rotateXBy(myself,-refAngOthX,0.0, 0.0, 0.0);	refX = getPositionX(myself);	refY = getPositionY(myself);	refZ = getPositionZ(myself);	refAngX = getAngleX(myself);	refAngY = getAngleY(myself);	refAngZ = getAngleZ(myself);	rotateBy(myself,refAngOthX,refAngOthY, refAngOthZ, 0.0, 0.0, 0.0); // relocate the product	moveBy(myself, refOthX, refOthY, refOthZ);}if (armActive && memArmActive) {	rotateTo(myself,refAngX,refAngY,refAngZ, getPositionX(myself), getPositionY(myself), getPositionZ(myself));	moveTo(myself, refX, refY, refZ);	rotateBy(myself,getAngleX(other),getAngleY(other), getAngleZ(other), 0.0, 0.0, 0.0);	moveBy(myself, getPositionX(other), getPositionY(other), getPositionZ(other));}
   �   4                       product_killer   motion_control  /* ----------------------------------------------------------------	*//* Destruction of the current instance of the product if the killer	*//* is active.									*//* ----------------------------------------------------------------	*/if (otherVar(isActive)) prodKill(myself);
   �   4                       conveyor   
motion_lib  �/* ----------------------------------------------------------------	*//* Interaction between the product and conveyors			*//* Convention : a conveyor is always oriented so that its X-axis is	*//*	  	  in the same direction as the flow			*//* ----------------------------------------------------------------	*//* ----------------------------------------------------------------	*//* We retrieve the conveyor's data if the center of the product is	*//* included in the volume of the conveyor.				*//* ----------------------------------------------------------------	*/if ((idConv == -1) || isMyCenterIncludedInXYOfOther(myself, other)) {	/* Traverse speed on the conveyor	*/	movementConv = otherVar(incShiftProduct) ;	/* Keep trace of traverse direction of the conveyor		*/	if (movementConv  > 0.0) moveDirection = DIR_FLOW;	if (movementConv  < 0.0) moveDirection = DIR_INVERSE_FLOW;	/* Get the identifier of the conveyor in order to be		*/	/* able to shift the product in its coordinate system		*/	idConv = other;	oldIdConv = idConv;	/* If the conveyor is a traverser, a lift, ...  		*/	/* the product msut follow it	 				*/	if (otherVar(incrementPositionX) != 0.0) moveXBy(myself, otherVar(incrementPositionX));	if (otherVar(incrementPositionY) != 0.0) moveYBy(myself, otherVar(incrementPositionY));	if (otherVar(incrementPositionZ) != 0.0) alignTwoObjectsRelativeThirdZ(myself, other, other);	/* Management of the rotation of the product on a 		*/	/* swivelling table, rotary plate ring, ...			*/	/* Keep trace of the rotation center in order to be 		*/	/* able to give them to products that could be stacked		*/	/* above this instance						*/	centreRotX = otherVar(centerRotX);	centreRotY = otherVar(centerRotY);	centreRotZ = otherVar(centerRotZ);	if ((otherVar(incRotX) != 0.0) || (otherVar(incRotY)!= 0) || (otherVar(incRotZ)!= 0.0))		rotateBy(myself, otherVar(incRotX), otherVar(incRotY), otherVar(incRotZ), centreRotX, centreRotY, centreRotZ);  }/* ----------------------------------------------------------------	*//* The product lays on a solid						*//* ----------------------------------------------------------------	*/contact = 1;
   �   4                       presence_sensor   
motion_lib   �/* ----------------------------------------------------------------	*//* Information to the sensor that a product is in its detection area	*//* ----------------------------------------------------------------	*/otherVar(detection) = 1;
   �   4                       product   motion_control  
l/* ----------------------------------------------------------------	*//* Interaction between two products	 				*//* ----------------------------------------------------------------	*//* Verify that the product whose position has to be updated is 	*//* located on one side of the current instance and not above or 	*//* below it.									*/if ((getCenterZ(myself) > getCenterZ(other) - (getSizeZ(other)/2.0)) && (getCenterZ(myself) < getCenterZ(other)+(getSizeZ(other)/2.0))) {	/* Update the position of the product that is behind the 	*/	/* other according to the shift direction				*/	if ( (moveDirection == DIR_FLOW && (getCenterXInAnother(myself, idConv) < getCenterXInAnother(other, idConv))) 		|| (moveDirection == DIR_INVERSE_FLOW && (getCenterXInAnother(myself, idConv) > getCenterXInAnother(other, idConv))) ) {		/* Align the 2 products one behind the other if its speed is lower 		*/		if (fabs(movementConv ) > fabs(otherVar(movementConv )))			alignTwoObjectsRelativeThirdX(myself, other, idConv);		/* Forbid the shift						*/		moveAuthorization=0;	 }}/* Interaction with the product below					*/if ((getCenterZ(myself)  > getCenterZ(other)+ (getSizeZ(other) / 2)) 	&&  ((getCenterX(myself) > getCenterX(other) - (getSizeX(other)/4.0)) && (getCenterX(myself) < getCenterX(other)+(getSizeX(other)/4.0)))	&&  ((getCenterY(myself) > getCenterY(other) - (getSizeY(other)/4.0)) && (getCenterY(myself) < getCenterY(other)+(getSizeY(other)/4.0))))  {	/* Same movement than the product below				*/	if (idProdBelow == other) {		moveBy(myself, getPositionX(other)-oldOtherPx, getPositionY(other)-oldOtherPy, getPositionZ(other)-oldOtherPz);		rotateBy(myself, getAngleX(other)-oldOtherRx, getAngleY(other)-oldOtherRy, getAngleZ(other)-oldOtherRz,				otherVar(centreRotX), otherVar(centreRotY), otherVar(centreRotZ));	}	oldOtherPx=getPositionX(other); oldOtherPy=getPositionY(other); oldOtherPz=getPositionZ(other);	oldOtherRx=getAngleX(other); oldOtherRy=getAngleY(other); oldOtherRz=getAngleZ(other);	centreRotX=otherVar(centreRotX); centreRotY=otherVar(centreRotY); centreRotZ=otherVar(centreRotZ);	idProdBelow = other;	/* The product lays on a solid					*/	contact = 1;	/* The next line of code can be added in order to align 	*/	/* correctly  the products one on the others ...		*/	/* We should align both products in Z-axis according to the 	*/	/* absolute coordinate system and not according to the 	*/	/* relative system coordinate in order to have a generic 	*/	/* behaviour.								*/	/* => adapt this according to the application when needed	*/	/* alignTwoObjectsRelativeThirdZ(myself, other, other); 	*/	}
   �   4                       product_creator   motion_control   �/* ----------------------------------------------------------------	*//* Information to the creation module that a product is in its area	*//* ----------------------------------------------------------------	*/ otherVar(productPresence) = 1;
I�xe