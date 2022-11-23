/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "../ASN1_Input/rrc_15_3_asn.asn1"
 * 	`asn1c -D ../RRC_output_14Nov/ -fcompound-names -fno-include-deps -findirect-choice -gen-PER`
 */

#ifndef	_UEAssistanceInformation_v1540_IEs_H_
#define	_UEAssistanceInformation_v1540_IEs_H_


#include <asn_application.h>

/* Including external dependencies */
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct OverheatingAssistance;

/* UEAssistanceInformation-v1540-IEs */
typedef struct UEAssistanceInformation_v1540_IEs {
	struct OverheatingAssistance	*overheatingAssistance;	/* OPTIONAL */
	struct UEAssistanceInformation_v1540_IEs__nonCriticalExtension {
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *nonCriticalExtension;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} UEAssistanceInformation_v1540_IEs_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_UEAssistanceInformation_v1540_IEs;
extern asn_SEQUENCE_specifics_t asn_SPC_UEAssistanceInformation_v1540_IEs_specs_1;
extern asn_TYPE_member_t asn_MBR_UEAssistanceInformation_v1540_IEs_1[2];

#ifdef __cplusplus
}
#endif

#endif	/* _UEAssistanceInformation_v1540_IEs_H_ */
#include <asn_internal.h>
