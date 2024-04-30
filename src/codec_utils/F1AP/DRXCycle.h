/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "F1AP-IEs"
 * 	found in "../F1.asn1"
 * 	`asn1c -D ../F1_output/ -fcompound-names -fno-include-deps -findirect-choice -gen-PER`
 */

#ifndef	_DRXCycle_H_
#define	_DRXCycle_H_


#include <asn_application.h>

/* Including external dependencies */
#include "LongDRXCycleLength.h"
#include "ShortDRXCycleLength.h"
#include "ShortDRXCycleTimer.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct ProtocolExtensionContainer;

/* DRXCycle */
typedef struct DRXCycle {
	LongDRXCycleLength_t	 longDRXCycleLength;
	ShortDRXCycleLength_t	*shortDRXCycleLength;	/* OPTIONAL */
	ShortDRXCycleTimer_t	*shortDRXCycleTimer;	/* OPTIONAL */
	struct ProtocolExtensionContainer	*iE_Extensions;	/* OPTIONAL */
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} DRXCycle_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_DRXCycle;

#ifdef __cplusplus
}
#endif

#endif	/* _DRXCycle_H_ */
#include <asn_internal.h>
