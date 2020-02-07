/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "../../mib_sib1.asn1"
 * 	`asn1c -D ./mib_sib_out/ -fcompound-names -fno-include-deps -findirect-choice -gen-PER -no-gen-example`
 */

#include "PDCCH-ConfigSIB1.h"

asn_TYPE_member_t asn_MBR_PDCCH_ConfigSIB1_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct PDCCH_ConfigSIB1, controlResourceSetZero),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_ControlResourceSetZero,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"controlResourceSetZero"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct PDCCH_ConfigSIB1, searchSpaceZero),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_SearchSpaceZero,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"searchSpaceZero"
		},
};
static const ber_tlv_tag_t asn_DEF_PDCCH_ConfigSIB1_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_PDCCH_ConfigSIB1_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* controlResourceSetZero */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 } /* searchSpaceZero */
};
asn_SEQUENCE_specifics_t asn_SPC_PDCCH_ConfigSIB1_specs_1 = {
	sizeof(struct PDCCH_ConfigSIB1),
	offsetof(struct PDCCH_ConfigSIB1, _asn_ctx),
	asn_MAP_PDCCH_ConfigSIB1_tag2el_1,
	2,	/* Count of tags in the map */
	0, 0, 0,	/* Optional elements (not needed) */
	-1,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_PDCCH_ConfigSIB1 = {
	"PDCCH-ConfigSIB1",
	"PDCCH-ConfigSIB1",
	&asn_OP_SEQUENCE,
	asn_DEF_PDCCH_ConfigSIB1_tags_1,
	sizeof(asn_DEF_PDCCH_ConfigSIB1_tags_1)
		/sizeof(asn_DEF_PDCCH_ConfigSIB1_tags_1[0]), /* 1 */
	asn_DEF_PDCCH_ConfigSIB1_tags_1,	/* Same as above */
	sizeof(asn_DEF_PDCCH_ConfigSIB1_tags_1)
		/sizeof(asn_DEF_PDCCH_ConfigSIB1_tags_1[0]), /* 1 */
	{ 0, 0, SEQUENCE_constraint },
	asn_MBR_PDCCH_ConfigSIB1_1,
	2,	/* Elements count */
	&asn_SPC_PDCCH_ConfigSIB1_specs_1	/* Additional specs */
};

