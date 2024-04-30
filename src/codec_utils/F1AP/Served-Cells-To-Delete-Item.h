/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "F1AP-IEs"
 * 	found in "../F1.asn1"
 * 	`asn1c -D ../F1_output/ -fcompound-names -fno-include-deps -findirect-choice -gen-PER`
 */

#ifndef	_Served_Cells_To_Delete_Item_H_
#define	_Served_Cells_To_Delete_Item_H_


#include <asn_application.h>

/* Including external dependencies */
#include "NRCGI.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct ProtocolExtensionContainer;

/* Served-Cells-To-Delete-Item */
typedef struct Served_Cells_To_Delete_Item {
	NRCGI_t	 oldNRCGI;
	struct ProtocolExtensionContainer	*iE_Extensions;	/* OPTIONAL */
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} Served_Cells_To_Delete_Item_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_Served_Cells_To_Delete_Item;

#ifdef __cplusplus
}
#endif

#endif	/* _Served_Cells_To_Delete_Item_H_ */
#include <asn_internal.h>
