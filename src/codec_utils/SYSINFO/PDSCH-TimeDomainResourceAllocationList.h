/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "../../mib_sib1.asn1"
 * 	`asn1c -D ./mib_sib_out/ -fcompound-names -fno-include-deps -findirect-choice -gen-PER -no-gen-example`
 */

#ifndef	_PDSCH_TimeDomainResourceAllocationList_H_
#define	_PDSCH_TimeDomainResourceAllocationList_H_


#include <asn_application.h>

/* Including external dependencies */
#include <asn_SEQUENCE_OF.h>
#include <constr_SEQUENCE_OF.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct PDSCH_TimeDomainResourceAllocation;

/* PDSCH-TimeDomainResourceAllocationList */
typedef struct PDSCH_TimeDomainResourceAllocationList {
	A_SEQUENCE_OF(struct PDSCH_TimeDomainResourceAllocation) list;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} PDSCH_TimeDomainResourceAllocationList_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_PDSCH_TimeDomainResourceAllocationList;
extern asn_SET_OF_specifics_t asn_SPC_PDSCH_TimeDomainResourceAllocationList_specs_1;
extern asn_TYPE_member_t asn_MBR_PDSCH_TimeDomainResourceAllocationList_1[1];
extern asn_per_constraints_t asn_PER_type_PDSCH_TimeDomainResourceAllocationList_constr_1;

#ifdef __cplusplus
}
#endif

#endif	/* _PDSCH_TimeDomainResourceAllocationList_H_ */
#include <asn_internal.h>
