/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "../../mib_sib1.asn1"
 * 	`asn1c -D ./mib_sib_out/ -fcompound-names -fno-include-deps -findirect-choice -gen-PER -no-gen-example`
 */

#include "PUSCH-ConfigCommon.h"

#include "PUSCH-TimeDomainResourceAllocationList.h"
/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
static int
memb_msg3_DeltaPreamble_constraint_1(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	long value;
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	value = *(const long *)sptr;
	
	if((value >= -1 && value <= 6)) {
		/* Constraint check succeeded */
		return 0;
	} else {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

static int
memb_p0_NominalWithGrant_constraint_1(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	long value;
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	value = *(const long *)sptr;
	
	if((value >= -202 && value <= 24)) {
		/* Constraint check succeeded */
		return 0;
	} else {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

static asn_oer_constraints_t asn_OER_type_groupHoppingEnabledTransformPrecoding_constr_2 CC_NOTUSED = {
	{ 0, 0 },
	-1};
static asn_per_constraints_t asn_PER_type_groupHoppingEnabledTransformPrecoding_constr_2 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 0,  0,  0,  0 }	/* (0..0) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static asn_oer_constraints_t asn_OER_memb_msg3_DeltaPreamble_constr_5 CC_NOTUSED = {
	{ 1, 0 }	/* (-1..6) */,
	-1};
static asn_per_constraints_t asn_PER_memb_msg3_DeltaPreamble_constr_5 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 3,  3, -1,  6 }	/* (-1..6) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static asn_oer_constraints_t asn_OER_memb_p0_NominalWithGrant_constr_6 CC_NOTUSED = {
	{ 2, 0 }	/* (-202..24) */,
	-1};
static asn_per_constraints_t asn_PER_memb_p0_NominalWithGrant_constr_6 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 8,  8, -202,  24 }	/* (-202..24) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static const asn_INTEGER_enum_map_t asn_MAP_groupHoppingEnabledTransformPrecoding_value2enum_2[] = {
	{ 0,	7,	"enabled" }
};
static const unsigned int asn_MAP_groupHoppingEnabledTransformPrecoding_enum2value_2[] = {
	0	/* enabled(0) */
};
static const asn_INTEGER_specifics_t asn_SPC_groupHoppingEnabledTransformPrecoding_specs_2 = {
	asn_MAP_groupHoppingEnabledTransformPrecoding_value2enum_2,	/* "tag" => N; sorted by tag */
	asn_MAP_groupHoppingEnabledTransformPrecoding_enum2value_2,	/* N => "tag"; sorted by N */
	1,	/* Number of elements in the maps */
	0,	/* Enumeration is not extensible */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_groupHoppingEnabledTransformPrecoding_tags_2[] = {
	(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_groupHoppingEnabledTransformPrecoding_2 = {
	"groupHoppingEnabledTransformPrecoding",
	"groupHoppingEnabledTransformPrecoding",
	&asn_OP_NativeEnumerated,
	asn_DEF_groupHoppingEnabledTransformPrecoding_tags_2,
	sizeof(asn_DEF_groupHoppingEnabledTransformPrecoding_tags_2)
		/sizeof(asn_DEF_groupHoppingEnabledTransformPrecoding_tags_2[0]) - 1, /* 1 */
	asn_DEF_groupHoppingEnabledTransformPrecoding_tags_2,	/* Same as above */
	sizeof(asn_DEF_groupHoppingEnabledTransformPrecoding_tags_2)
		/sizeof(asn_DEF_groupHoppingEnabledTransformPrecoding_tags_2[0]), /* 2 */
	{ &asn_OER_type_groupHoppingEnabledTransformPrecoding_constr_2, &asn_PER_type_groupHoppingEnabledTransformPrecoding_constr_2, NativeEnumerated_constraint },
	0, 0,	/* Defined elsewhere */
	&asn_SPC_groupHoppingEnabledTransformPrecoding_specs_2	/* Additional specs */
};

asn_TYPE_member_t asn_MBR_PUSCH_ConfigCommon_1[] = {
	{ ATF_POINTER, 4, offsetof(struct PUSCH_ConfigCommon, groupHoppingEnabledTransformPrecoding),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_groupHoppingEnabledTransformPrecoding_2,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"groupHoppingEnabledTransformPrecoding"
		},
	{ ATF_POINTER, 3, offsetof(struct PUSCH_ConfigCommon, pusch_TimeDomainAllocationList),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_PUSCH_TimeDomainResourceAllocationList,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"pusch-TimeDomainAllocationList"
		},
	{ ATF_POINTER, 2, offsetof(struct PUSCH_ConfigCommon, msg3_DeltaPreamble),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_NativeInteger,
		0,
		{ &asn_OER_memb_msg3_DeltaPreamble_constr_5, &asn_PER_memb_msg3_DeltaPreamble_constr_5,  memb_msg3_DeltaPreamble_constraint_1 },
		0, 0, /* No default value */
		"msg3-DeltaPreamble"
		},
	{ ATF_POINTER, 1, offsetof(struct PUSCH_ConfigCommon, p0_NominalWithGrant),
		(ASN_TAG_CLASS_CONTEXT | (3 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_NativeInteger,
		0,
		{ &asn_OER_memb_p0_NominalWithGrant_constr_6, &asn_PER_memb_p0_NominalWithGrant_constr_6,  memb_p0_NominalWithGrant_constraint_1 },
		0, 0, /* No default value */
		"p0-NominalWithGrant"
		},
};
static const int asn_MAP_PUSCH_ConfigCommon_oms_1[] = { 0, 1, 2, 3 };
static const ber_tlv_tag_t asn_DEF_PUSCH_ConfigCommon_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_PUSCH_ConfigCommon_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* groupHoppingEnabledTransformPrecoding */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* pusch-TimeDomainAllocationList */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 }, /* msg3-DeltaPreamble */
    { (ASN_TAG_CLASS_CONTEXT | (3 << 2)), 3, 0, 0 } /* p0-NominalWithGrant */
};
asn_SEQUENCE_specifics_t asn_SPC_PUSCH_ConfigCommon_specs_1 = {
	sizeof(struct PUSCH_ConfigCommon),
	offsetof(struct PUSCH_ConfigCommon, _asn_ctx),
	asn_MAP_PUSCH_ConfigCommon_tag2el_1,
	4,	/* Count of tags in the map */
	asn_MAP_PUSCH_ConfigCommon_oms_1,	/* Optional members */
	4, 0,	/* Root/Additions */
	4,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_PUSCH_ConfigCommon = {
	"PUSCH-ConfigCommon",
	"PUSCH-ConfigCommon",
	&asn_OP_SEQUENCE,
	asn_DEF_PUSCH_ConfigCommon_tags_1,
	sizeof(asn_DEF_PUSCH_ConfigCommon_tags_1)
		/sizeof(asn_DEF_PUSCH_ConfigCommon_tags_1[0]), /* 1 */
	asn_DEF_PUSCH_ConfigCommon_tags_1,	/* Same as above */
	sizeof(asn_DEF_PUSCH_ConfigCommon_tags_1)
		/sizeof(asn_DEF_PUSCH_ConfigCommon_tags_1[0]), /* 1 */
	{ 0, 0, SEQUENCE_constraint },
	asn_MBR_PUSCH_ConfigCommon_1,
	4,	/* Elements count */
	&asn_SPC_PUSCH_ConfigCommon_specs_1	/* Additional specs */
};

