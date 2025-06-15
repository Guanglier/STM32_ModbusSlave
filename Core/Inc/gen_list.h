/*
 * gen_list.h
 *
 *  Created on: May 1, 2025
 *      Author: c.monange
 */

//===========================================================================
//		TO DO : OPTIMIZE mettre un compteur de l'index max pour accélérer les recherches cad ne pas parcourir tout le tableau
//				si 3 elements dans la liste et qu'on recherche un elmnt qui n'existe pas, ne pas parcourir toute la liste
//
//			Generic list that holds a pointer to void*
//
//			UTILISATION
//
//		u16_key = identifier of the data, must be unique. can be every value except GEN_LIST_INVALID_KEY
//
//	uint16_t  data;
//	uint16_t  *PtrData;
//
//	gen_liste_init ();
//	gen_liste_add ( u16_key, (void*) &gen_liste_add );
//
//	PtrData = (uint16*) gen_liste_get ( u16_key );
//	if ( 0 != PtrData ){
//		// do some stuff
//	}else{
//		//there error !!
//	}
//
//
//	if ( 0 != gen_liste_delete ( u16_key ) ){
//		// do some stuff
//	}else{
//		//there error !!
//	}
//
//
//===========================================================================


#ifndef SRC_GEN_LIST_H_
#define SRC_GEN_LIST_H_


	#include "stdint.h"

	#define GEN_LIST_INVALID_KEY			0xffff

	//max of all elements on the list
	#define GEN_LIST_INTERNAL_MAX_ELMT	50



	void 		gen_liste_init 		();
	uint8_t 	gen_liste_add 		( const uint16_t u16_key, void* const li_ptr_data);
	void* 		gen_liste_get 		( const uint16_t u16_key);
	uint8_t 	gen_liste_delete 	( const uint16_t u16_key);
	uint8_t 	gen_liste_optimize 	();

#endif /* SRC_GEN_LIST_H_ */

















