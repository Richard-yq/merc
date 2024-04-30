/*******************************************************************************
################################################################################
#   Copyright (c) [2017-2019] [Radisys]                                        #
#                                                                              #
#   Licensed under the Apache License, Version 2.0 (the "License");            #
#   you may not use this file except in compliance with the License.           #
#   You may obtain a copy of the License at                                    #
#                                                                              #
#       http://www.apache.org/licenses/LICENSE-2.0                             #
#                                                                              #
#   Unless required by applicable law or agreed to in writing, software        #
#   distributed under the License is distributed on an "AS IS" BASIS,          #
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   #
#   See the License for the specific language governing permissions and        #
#   limitations under the License.                                             #
################################################################################
*******************************************************************************/

/* This file contains F1AP message handler functions */
#include "common_def.h"
#include "du_tmr.h"
#include "ckw.h"
#include "ckw.x"
#include "kwu.h"
#include "kwu.x"
#include "lkw.h"
#include "lrg.h"
#include "legtp.h"
#include "lkw.x"
#include "lrg.x"
#include "F1AP-PDU.h"
#include "InitiatingMessage.h"
#include "SuccessfulOutcome.h"
#include "du_app_mac_inf.h"
#include "du_cfg.h"
#include "du_app_rlc_inf.h"
#include "du_e2ap_mgr.h"
#include "du_e2ap_msg_hdl.h"
#include "du_mgr_main.h"
#include "du_mgr.h"
#include "du_utils.h"
#include "du_ue_mgr.h"
#include "RAT-Type.h"
#include "NRFreqInfo.h"
#include "NRCGI.h"
#include "TDD-Info.h"
#include "NR-Mode-Info.h"
#include "ServedPLMNs-List.h"
#include "GNB-DU-Served-Cells-List.h"
#include "Served-Cell-Information.h"
#include "ProtocolExtensionContainer.h"
#include "RRC-Version.h"
#include "FDD-Info.h"
#include "FreqBandNrItem.h"
#include "ServedPLMNs-Item.h"
#include "GNB-DU-Served-Cells-Item.h"
#include "SliceSupportItem.h"
#include "FeatureSetUplinkPerCC.h"
#include "SliceSupportItem.h"
#include "Served-Cells-To-Modify-Item.h"
#include "Served-Cells-To-Delete-List.h"
#include "Served-Cells-To-Delete-Item.h"
#include "QoSInformation.h"
#include "ULUPTNLInformation-ToBeSetup-List.h"
#include "DRB-Information.h"
#include "DRBs-ToBeSetup-Item.h"
#include "DRBs-ToBeSetupMod-Item.h"
#include "DRBs-ToBeModified-Item.h"
#include "CUtoDURRCInformation.h"
#include "SCell-ToBeSetup-List.h"
#include "SRBs-ToBeSetup-List.h"
#include "DRBs-ToBeSetup-List.h"
#include "SRBs-ToBeSetup-Item.h"
#include "DRBs-ToBeModified-List.h"
#include "RRCContainer.h"
#include "UE-CapabilityRAT-ContainerList.h"
#include "DRBs-Setup-List.h"
#include "CellGroupConfig.h"
#include "ResetAll.h"
#include "ResetType.h"
#include "Cells-to-be-Activated-List.h"
#include "GNB-CU-Name.h"
#include "SRBs-SetupMod-List.h"
#include "DRBs-SetupMod-List.h"
#include "DRBs-ToBeSetupMod-List.h"
#include "PagingCell-Item.h"
#include "PagingCell-list.h"
#include "QoS-Characteristics.h"
#include "ULUPTNLInformation-ToBeSetup-Item.h"
#include "Flows-Mapped-To-DRB-Item.h"
#include "NonDynamic5QIDescriptor.h"
#include "Dynamic5QIDescriptor.h"
#include "FeatureSetDownlinkPerCC.h"
#include "FeatureSets.h"
#include "UE-NR-Capability.h"
#include "UE-CapabilityRAT-Container.h"
#include "UE-CapabilityRAT-ContainerListRRC.h"
#include "GNB-DU-System-Information.h"
#include "CellGroupConfigRrc.h"
#include "MAC-CellGroupConfig.h"
#include "SchedulingRequestConfig.h"
#include "SchedulingRequestToAddMod.h"
#include "BSR-Config.h"
#include "TAG-Config.h"
#include "TAG.h"
#include "PHR-Config.h"
#include "RLC-Config.h"
#include "UL-AM-RLC.h"
#include "DL-AM-RLC.h"
#include "LogicalChannelConfig.h"
#include "RLC-BearerConfig.h"
#include "PhysicalCellGroupConfig.h"
#include "SpCellConfig.h"
#include "TDD-UL-DL-ConfigDedicated.h"
#include "ServingCellConfig.h"
#include "ControlResourceSet.h"
#include "SearchSpace.h"
#include "PDCCH-Config.h"
#include "PDSCH-TimeDomainResourceAllocation.h"
#include "PDSCH-TimeDomainResourceAllocationList.h"
#include "PDSCH-CodeBlockGroupTransmission.h"
#include "PDSCH-ServingCellConfig.h"
#include "DMRS-DownlinkConfig.h"
#include "PDSCH-Config.h"
#include "BWP-DownlinkDedicated.h"
#include "BWP-Downlink.h"
#include "PUSCH-TimeDomainResourceAllocation.h"
#include "PUSCH-TimeDomainResourceAllocationList.h"
#include "DMRS-UplinkConfig.h"
#include "PUSCH-Config.h"
#include "SRS-ResourceId.h"
#include "SRS-Resource.h"
#include "SRS-ResourceSet.h"
#include "SRS-Config.h"
#include "BWP-UplinkDedicated.h"
#include "PUSCH-ServingCellConfig.h"
#include "UplinkConfig.h"
#include "DUtoCURRCContainer.h"
#include "GBR-QoSFlowInformation.h"
#include "QoSFlowLevelQoSParameters.h"
#include "PUCCH-Config.h"
#include "PUCCH-ResourceSet.h"
#include "PUCCH-Resource.h"
#include "PUCCH-PowerControl.h"
#include "P0-PUCCH.h"
#include "PUCCH-PathlossReferenceRS.h"
#include "PUCCH-format0.h"
#include "PUCCH-format1.h"
#include "PUCCH-format2.h"
#include "PUCCH-format3.h"
#include "PUCCH-format4.h"
#include "PUCCH-FormatConfig.h"
#include "SchedulingRequestResourceConfig.h"
#include<ProtocolIE-Field.h>
#include "ProtocolExtensionField.h"
#include "odu_common_codec.h"
#include "du_mgr.h"
#include "du_cell_mgr.h"
#include "du_f1ap_msg_hdl.h"
#include "DRBs-Setup-Item.h"
#include "DLUPTNLInformation-ToBeSetup-List.h"
#include "DLUPTNLInformation-ToBeSetup-Item.h"
#include "UPTransportLayerInformation.h"
#include "GTPTunnel.h"
#include "SupportedSULFreqBandItem.h"
#include "du_f1ap_conversions.h"
#include "CNUEPagingIdentity.h"
#include "PCCH-Config.h"
#include "SCS-SpecificCarrier.h"
#include "FrequencyInfoDL.h"
#include "DownlinkConfigCommon.h"
#include "FrequencyInfoUL.h"
#include "UplinkConfigCommon.h"
#include "TDD-UL-DL-ConfigCommon.h"
#include "RACH-ConfigDedicated.h"
#include "CFRA-SSB-Resource.h"
#include "BWP-UplinkCommon.h"
#include "ReconfigurationWithSync.h"
#include "BCCH-DL-SCH-Message.h"
#include "du_sys_info_hdl.h"
#include "DRX-ConfigRrc.h"
#include "MeasurementTimingConfigurationRrc.h"
#include "MeasurementTimingConfigurationRrc-IEs.h"
#include "MeasTimingList.h"
#include "MeasTiming.h"
#include "Cells-Status-List.h"
#include "Cells-Status-Item.h"

#ifdef O1_ENABLE
#include "CmInterface.h"
extern StartupConfig g_cfg;
#endif

DuCfgParams duCfgParam;

/******************************************************************
 *
 * @brief Function to fetch lcId based on DRB Id
 *
 * @details
 *
 *    Function : fetchLcId
 *
 *    @params[in] drbId
 *
 *    Functionality: Function to fetch lcId based on DRB Id
 *
 * Returns: lcId - SUCCESS
 *          RFAILED - FAILURE
 *****************************************************************/

uint8_t fetchLcId(uint8_t drbId)
{
   uint8_t cellIdx = 0, ueIdx = 0, lcIdx = 0, numLcs = 0, lcId = 0;

   for(cellIdx = 0; cellIdx < MAX_NUM_CELL; cellIdx++)
   {
      for(ueIdx = 0; ueIdx < MAX_NUM_UE; ueIdx++)
      {
         if(duCb.actvCellLst[cellIdx] != NULLP)
         {
            numLcs = duCb.actvCellLst[cellIdx]->ueCb[ueIdx].duRlcUeCfg.numLcs;
            for(lcIdx = 0; lcIdx < numLcs; lcIdx++)
            {
               if(duCb.actvCellLst[cellIdx]->ueCb[ueIdx].duRlcUeCfg.rlcLcCfg[lcIdx].rlcBearerCfg.rbId == drbId && \
                  duCb.actvCellLst[cellIdx]->ueCb[ueIdx].duRlcUeCfg.rlcLcCfg[lcIdx].rlcBearerCfg.rbType == RB_TYPE_DRB)
               {
                  lcId = duCb.actvCellLst[cellIdx]->ueCb[ueIdx].duRlcUeCfg.rlcLcCfg[lcIdx].rlcBearerCfg.lcId;
                  return lcId;
               }
            }
         }
      }
   }
   DU_LOG("\nERROR   -->  DU_APP: fetchLcId() failed for drbId %d", drbId);
   return RFAILED;
}

/*******************************************************************
*
* @brief Adding F1AP pdu to reserved pdu list
*
* @details
*
*    Function : addToReservedF1apPduList 
*
*    Functionality: Adding F1AP pdu to reserved pdu list.
*     These pdu are awaiting aknowledgment from CU
*
* @params[in] uint8_t transId, F1AP_PDU_t *f1apMsg
*
* @return ROK - success
*         RFAILED - failure
*
* ****************************************************************/
void addToReservedF1apPduList(uint8_t transId, F1AP_PDU_t *f1apPdu)
{
   CmLList         *node = NULLP;
   ReservedF1apPduInfo *pduInfo = NULLP;
   DU_ALLOC(pduInfo, sizeof(ReservedF1apPduInfo));
   if(pduInfo)
   {
      DU_ALLOC(node, sizeof(CmLList));
      if(node)
      {
         pduInfo->transId = transId;
         pduInfo->f1apMsg = (void*) f1apPdu;

         node->node = (PTR)pduInfo;
         cmLListAdd2Tail(&duCb.reservedF1apPduList, node);
      }
   }
}

/*******************************************************************
*
* @brief searching for F1AP pdu from ReservedF1apPduList 
*
* @details
*
*    Function : searchFromReservedF1apPduList 
*
*    Functionality: searching for F1AP pdu information
*
* @params[in] uint8_t transId
*
* @return pointer to F1AP_PDU_t
*
* ****************************************************************/

CmLList *searchFromReservedF1apPduList(uint8_t transId)
{
   CmLList         *node;
   ReservedF1apPduInfo *f1apPdu;
   if(duCb.reservedF1apPduList.count)
   {
      CM_LLIST_FIRST_NODE(&duCb.reservedF1apPduList, node);
      while(node)
      {
         f1apPdu = (ReservedF1apPduInfo*)node->node;
         if(f1apPdu->transId == transId)
         {
            return node;
         }
         node = node->next;
      }
   }
   return NULL;
}

/*******************************************************************
*
* @brief deleting F1AP pdu information from ReservedF1apPduList
*
* @details
*
*    Function : deleteFromReservedF1apPduList 
*
*    Functionality: deleting pdu information from ReservedF1apPduList
*
* @params[in] CmLList *node 
*
* @return void 
*
* ****************************************************************/

void deleteFromReservedF1apPduList(CmLList *node)
{
   ReservedF1apPduInfo *f1apPdu;

   if(node != NULL)
   {
      f1apPdu = (ReservedF1apPduInfo *)node->node;
      cmLListDelFrm(&duCb.reservedF1apPduList, node);
      DU_FREE(f1apPdu, sizeof(ReservedF1apPduInfo));
      DU_FREE(node, sizeof(CmLList));
      node = NULL;
   }
}

/*******************************************************************
 *
 * @brief Builds Uplink Info for NR 
 *
 * @details
 *
 *    Function : BuildULNRInfo
 *
 *    Functionality: Building NR Uplink Info
 *
 * @params[in] NRFreqInfo_t *ulnrfreq
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
uint8_t BuildULNRInfo(NRFreqInfo_t *ulnrfreq)
{
   uint8_t idx=0;
   ulnrfreq->nRARFCN = duCfgParam.srvdCellLst[0].duCellInfo.f1Mode.mode.\
		       fdd.ulNrFreqInfo.nrArfcn;
   ulnrfreq->freqBandListNr.list.count = 1;
   ulnrfreq->freqBandListNr.list.size = sizeof(FreqBandNrItem_t *);
   DU_ALLOC(ulnrfreq->freqBandListNr.list.array,ulnrfreq->freqBandListNr.list.size);
   if(ulnrfreq->freqBandListNr.list.array == NULLP)
   {
      return RFAILED;
   }
   for(idx=0;idx<ulnrfreq->freqBandListNr.list.count;idx++)
   {
      DU_ALLOC(ulnrfreq->freqBandListNr.list.array[idx],sizeof(FreqBandNrItem_t));
      if(ulnrfreq->freqBandListNr.list.array[idx] == NULLP)
      {
	 return RFAILED;
      }
   }
   ulnrfreq->freqBandListNr.list.array[0]->freqBandIndicatorNr = \
								 duCfgParam.srvdCellLst[0].duCellInfo.f1Mode.mode.fdd.ulNrFreqInfo.\
								 freqBand[0].nrFreqBand;
   ulnrfreq->freqBandListNr.list.array[0]->supportedSULBandList.list.count = SUL_BAND_COUNT;
   return ROK;
}
/*******************************************************************
 *
 * @brief Builds Downlink NR Info 
 *
 * @details
 *
 *    Function : BuildDLNRInfo
 *
 *    Functionality: Building Downlink NR Info
 *    
 * @params[in] NRFreqInfo_t *dlnrfreq
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
uint8_t BuildDLNRInfo(NRFreqInfo_t *dlnrfreq)
{
   uint8_t idx=0;
   dlnrfreq->nRARFCN = duCfgParam.srvdCellLst[0].duCellInfo.f1Mode.mode.\
		       fdd.dlNrFreqInfo.nrArfcn;
   dlnrfreq->freqBandListNr.list.count = 1;
   dlnrfreq->freqBandListNr.list.size = sizeof(FreqBandNrItem_t *);
   DU_ALLOC(dlnrfreq->freqBandListNr.list.array,dlnrfreq->freqBandListNr.list.size);
   if(dlnrfreq->freqBandListNr.list.array == NULLP)
   {
      return RFAILED;   
   }
   for(idx=0;idx< dlnrfreq->freqBandListNr.list.count;idx++)
   {
      DU_ALLOC(dlnrfreq->freqBandListNr.list.array[idx],sizeof(FreqBandNrItem_t));
      if(dlnrfreq->freqBandListNr.list.array[idx] == NULLP)
      {
	 return RFAILED;
      }
   }   
   dlnrfreq->freqBandListNr.list.array[0]->freqBandIndicatorNr = \
								 duCfgParam.srvdCellLst[0].duCellInfo.f1Mode.mode.fdd.dlNrFreqInfo.\
								 freqBand[0].nrFreqBand;
   dlnrfreq->freqBandListNr.list.array[0]->supportedSULBandList.list.count = SUL_BAND_COUNT;

   return ROK;
}

/*******************************************************************
 *
 * @brief Builds Nrcgi 
 *
 * @details
 *
 *    Function : BuildNrcgi
 *
 *    Functionality: Building the PLMN ID and NR Cell id
 *
 * @params[in] NRCGI_t *nrcgi
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
uint8_t BuildNrcgi(NRCGI_t *nrcgi)
{
   uint8_t ret;
   uint8_t byteSize = 5;
   /* Allocate Buffer Memory */
   nrcgi->pLMN_Identity.size = PLMN_SIZE * sizeof(uint8_t);
   DU_ALLOC(nrcgi->pLMN_Identity.buf, nrcgi->pLMN_Identity.size);
   if(nrcgi->pLMN_Identity.buf == NULLP)
   {
      return RFAILED;
   }
   ret = buildPlmnId(duCfgParam.srvdCellLst[0].duCellInfo.cellInfo.nrCgi.plmn,\
	 nrcgi->pLMN_Identity.buf); // Building PLMN function
   if(ret != ROK)
   {
      return RFAILED;
   }
   /*nrCellIdentity*/
   nrcgi->nRCellIdentity.size = byteSize * sizeof(uint8_t);
   DU_ALLOC(nrcgi->nRCellIdentity.buf, nrcgi->nRCellIdentity.size); 
   if(nrcgi->nRCellIdentity.buf == NULLP)
   {
      return RFAILED;
   }
   fillBitString(&nrcgi->nRCellIdentity, ODU_VALUE_FOUR, ODU_VALUE_FIVE, duCfgParam.sib1Params.cellIdentity);

   return ROK;
}
/*******************************************************************
 *
 * @brief Builds FiveGStac 
 *
 * @details
 *
 *    Function : BuildFiveGSTac
 *
 *    Functionality: Building the FiveGSTac
 *
 * @params[in] OCTET_STRING_t *fivegsTac
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
uint8_t BuildFiveGSTac(Served_Cell_Information_t *servcell)
{
   DU_ALLOC(servcell->fiveGS_TAC,sizeof(FiveGS_TAC_t));
   if(servcell->fiveGS_TAC == NULLP)
   {
      return RFAILED;
   }
   servcell->fiveGS_TAC->size = 3 * sizeof(uint8_t);
   DU_ALLOC(servcell->fiveGS_TAC->buf,\
	 sizeof(servcell->fiveGS_TAC->size));
   if(servcell->fiveGS_TAC->buf == NULLP)
   {
      return RFAILED;
   }
   servcell->fiveGS_TAC->buf[0] = 0;
   servcell->fiveGS_TAC->buf[1] = 0;
   servcell->fiveGS_TAC->buf[2] = duCfgParam.srvdCellLst[0].duCellInfo.tac;
   return ROK;  
}

/*******************************************************************
 *
 * @brief fill nr frequency information
 *
 * @details
 *
 *    Function : fillNrTddInfo 
 *
 *    Functionality: fill nr frequency information
 *
 * @params[in] NRFreqInfo_t freqInfo
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
uint8_t fillNrTddInfo(TDD_Info_t *tddInfo)
{
   uint8_t elementCnt = 1, freqBandListIdx = 0, supportedBandIdx = 0;
   NRFreqInfo_t *freqInfo = NULLP;

   if(tddInfo == NULLP)
   {
      DU_LOG("\nERROR  --> DU APP : Null pointer received at fillNrTddInfo");
      return RFAILED;
   }
   
   freqInfo = &tddInfo->nRFreqInfo;
   freqInfo->nRARFCN = duCfgParam.srvdCellLst[0].duCellInfo.f1Mode.mode.tdd.nrFreqInfo.nrArfcn; 

   freqInfo->freqBandListNr.list.count = elementCnt; 
   freqInfo->freqBandListNr.list.size = freqInfo->freqBandListNr.list.count  * sizeof(FreqBandNrItem_t *);
   DU_ALLOC(freqInfo->freqBandListNr.list.array, freqInfo->freqBandListNr.list.size );
   if(!freqInfo->freqBandListNr.list.array)
   {
      DU_LOG("\nERROR  --> DU APP : Memory allocation failed at fillNrTddInfo");
      return RFAILED;
   }

   for(freqBandListIdx = 0; freqBandListIdx<freqInfo->freqBandListNr.list.count; freqBandListIdx++)
   {
      DU_ALLOC(freqInfo->freqBandListNr.list.array[freqBandListIdx],  sizeof(FreqBandNrItem_t ));
      if(!freqInfo->freqBandListNr.list.array[freqBandListIdx])
      {
         DU_LOG("\nERROR  --> DU APP : Memory allocation failed at fillNrTddInfo");
         return RFAILED;
      }

      freqInfo->freqBandListNr.list.array[freqBandListIdx]->freqBandIndicatorNr = duCfgParam.srvdCellLst[0].duCellInfo.\
      f1Mode.mode.tdd.nrFreqInfo.freqBand[0].nrFreqBand;

      freqInfo->freqBandListNr.list.array[freqBandListIdx]->supportedSULBandList.list.count = elementCnt;
      freqInfo->freqBandListNr.list.array[freqBandListIdx]->supportedSULBandList.list.size = freqInfo->freqBandListNr.list.array[freqBandListIdx]->\
      supportedSULBandList.list.count * sizeof(SupportedSULFreqBandItem_t*);

      DU_ALLOC(freqInfo->freqBandListNr.list.array[freqBandListIdx]->supportedSULBandList.list.array,\
            freqInfo->freqBandListNr.list.array[freqBandListIdx]->supportedSULBandList.list.size);
      if(!freqInfo->freqBandListNr.list.array[freqBandListIdx]->supportedSULBandList.list.array)
      {
         DU_LOG("\nERROR  --> DU APP : Memory allocation failed at fillNrTddInfo");
         return RFAILED;
      }

      for(supportedBandIdx = 0; supportedBandIdx<freqInfo->freqBandListNr.list.array[freqBandListIdx]->supportedSULBandList.list.count; supportedBandIdx++)
      {
         DU_ALLOC(freqInfo->freqBandListNr.list.array[freqBandListIdx]->supportedSULBandList.list.array[supportedBandIdx],\
               sizeof(SupportedSULFreqBandItem_t));
         if(!freqInfo->freqBandListNr.list.array[freqBandListIdx]->supportedSULBandList.list.array[supportedBandIdx])
         {
            DU_LOG("\nERROR  --> DU APP : Memory allocation failed at fillNrTddInfo");
            return RFAILED;
         }

         freqInfo->freqBandListNr.list.array[freqBandListIdx]->supportedSULBandList.list.array[supportedBandIdx]->freqBandIndicatorNr =\
         duCfgParam.srvdCellLst[0].duCellInfo.f1Mode.mode.tdd.nrFreqInfo.freqBand[0].sulBand[0];
      }
   }

   tddInfo->transmission_Bandwidth.nRSCS = duCfgParam.srvdCellLst[0].duCellInfo.f1Mode.mode.tdd.nrFreqInfo.sulInfo.sulTxBw.nrScs;
   tddInfo->transmission_Bandwidth.nRNRB = duCfgParam.srvdCellLst[0].duCellInfo.f1Mode.mode.tdd.nrFreqInfo.sulInfo.sulTxBw.nrb;

   return ROK;
}

/*******************************************************************
 *
 * @brief Builds NR Mode 
 *
 * @details
 *
 *    Function : BuildNrMode
 *
 *    Functionality: Building the NR Mode
 *
 * @params[in] NR_Mode_Info_t *fdd
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
uint8_t BuildNrMode(NR_Mode_Info_t *mode)
{
   uint8_t BuildDLNRInforet=0;
   uint8_t BuildULNRInforet=0; 
   
#ifdef NR_TDD
   mode->present = NR_Mode_Info_PR_tDD;
#else
   mode->present = NR_Mode_Info_PR_fDD;
#endif   
   
   if(mode->present == NR_Mode_Info_PR_fDD)
   {
      DU_ALLOC(mode->choice.fDD,sizeof(FDD_Info_t));
      if(mode->choice.fDD == NULLP)
      {
         DU_LOG("\nERROR  --> Memory allocation failed in BuildNrMode");
         return RFAILED;
      }
      BuildULNRInforet = BuildULNRInfo(&mode->choice.fDD->uL_NRFreqInfo);
      if(BuildULNRInforet != ROK)
      {
         DU_LOG("\nERROR  --> Failed to build UlNrFreqInfo");
         return RFAILED;    
      }
      BuildDLNRInforet = BuildDLNRInfo(&mode->choice.fDD->dL_NRFreqInfo);
      if(BuildDLNRInforet != ROK)
      {
         DU_LOG("\nERROR  --> Failed to build DlNrFreqInfo");
         return RFAILED;
      }
      mode->choice.fDD->uL_Transmission_Bandwidth.nRSCS = \
                                                          duCfgParam.srvdCellLst[0].duCellInfo.\
                                                          f1Mode.mode.fdd.ulTxBw.nrScs;
      mode->choice.fDD->uL_Transmission_Bandwidth.nRNRB = \
                                                          duCfgParam.srvdCellLst[0].duCellInfo.\
                                                          f1Mode.mode.fdd.ulTxBw.nrb;
      mode->choice.fDD->dL_Transmission_Bandwidth.nRSCS = \
                                                          duCfgParam.srvdCellLst[0].duCellInfo.\
                                                          f1Mode.mode.fdd.dlTxBw.nrScs;
      mode->choice.fDD->dL_Transmission_Bandwidth.nRNRB = \
                                                          duCfgParam.srvdCellLst[0].duCellInfo.\
                                                          f1Mode.mode.fdd.dlTxBw.nrb;
   }
   else if(mode->present == NR_Mode_Info_PR_tDD) 
   {
      DU_ALLOC(mode->choice.tDD,sizeof(TDD_Info_t));
      if(mode->choice.tDD == NULLP)
      {
         DU_LOG("\nERROR  --> Memory allocation failed in BuildNrMode");
         return RFAILED;
      }

      if(fillNrTddInfo(mode->choice.tDD) != ROK)
      {
         DU_LOG("\nERROR  --> Failed to fill Nr TDD information");
         return RFAILED;
      }

   }

   return ROK;
}
/*******************************************************************
 *
 * @brief Builds IE Extensions for Served PLMNs 
 *
 * @details
 *
 *    Function : BuildExtensions
 *
 *    Functionality: Building the IE Extensions
 *
 * @params[in] struct ProtocolExtensionContainer_4624P3 *buildextend
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
uint8_t BuildExtensions(ProtocolExtensionContainer_4624P3_t **ieExtend)
{
   uint8_t idx=0, plmnidx=0, sliceLstIdx=0;
   uint8_t elementCnt=0, extensionCnt=0;

   extensionCnt=IE_EXTENSION_LIST_COUNT;
   DU_ALLOC(*ieExtend,sizeof(ProtocolExtensionContainer_4624P3_t));
   if((*ieExtend) == NULLP)
   {
      DU_LOG("ERROR  --> DU_APP : BuildExtensions(): Memory allocation failed");
      return RFAILED;
   }
   (*ieExtend)->list.count = extensionCnt;
   (*ieExtend)->list.size = \
                            extensionCnt * sizeof(ServedPLMNs_ItemExtIEs_t *);
   DU_ALLOC((*ieExtend)->list.array,(*ieExtend)->list.size);
   if((*ieExtend)->list.array == NULLP)
   {
      DU_LOG("ERROR  --> DU_APP : BuildExtensions(): Memory allocation failed");
      return RFAILED;
   }
   for(plmnidx=0;plmnidx<extensionCnt;plmnidx++)
   {
      DU_ALLOC((*ieExtend)->list.array[plmnidx],\
            sizeof(ServedPLMNs_ItemExtIEs_t));
      if((*ieExtend)->list.array[plmnidx] == NULLP)
      {
         DU_LOG("ERROR  --> DU_APP : BuildExtensions(): Memory allocation failed");
         return RFAILED;
      }
   }
   
   elementCnt = duCfgParam.srvdCellLst[0].duCellInfo.cellInfo.srvdPlmn[0].taiSliceSuppLst.numSupportedSlices;
   idx = 0;
   (*ieExtend)->list.array[idx]->id = ProtocolIE_ID_id_TAISliceSupportList;
   (*ieExtend)->list.array[idx]->criticality = Criticality_ignore;
   (*ieExtend)->list.array[idx]->extensionValue.present = \
   ServedPLMNs_ItemExtIEs__extensionValue_PR_SliceSupportList;
   (*ieExtend)->list.array[idx]->extensionValue.choice.SliceSupportList.\
      list.count = elementCnt;
   (*ieExtend)->list.array[idx]->extensionValue.choice.SliceSupportList.\
      list.size = (*ieExtend)->list.array[idx]->extensionValue.choice.SliceSupportList.\
      list.count * sizeof(SliceSupportItem_t *);

   DU_ALLOC((*ieExtend)->list.array[idx]->extensionValue.choice.SliceSupportList.\
         list.array, elementCnt * sizeof(SliceSupportItem_t *));
   if((*ieExtend)->list.array[idx]->extensionValue.choice.SliceSupportList.\
         list.array == NULLP)
   {
      DU_LOG("ERROR  --> DU_APP : BuildExtensions(): Memory allocation failed");
      return RFAILED;
   }

   for(sliceLstIdx =0; sliceLstIdx<elementCnt; sliceLstIdx++)
   {
      DU_ALLOC((*ieExtend)->list.array[idx]->extensionValue.choice.SliceSupportList.\
            list.array[sliceLstIdx],sizeof(SliceSupportItem_t));
      if((*ieExtend)->list.array[idx]->extensionValue.choice.SliceSupportList.\
            list.array[sliceLstIdx] == NULLP) 
      {
         DU_LOG("ERROR  --> DU_APP : BuildExtensions(): Memory allocation failed");
         return RFAILED;
      }
      (*ieExtend)->list.array[idx]->extensionValue.choice.SliceSupportList.\
         list.array[sliceLstIdx]->sNSSAI.sST.size = sizeof(uint8_t);
      DU_ALLOC((*ieExtend)->list.array[idx]->extensionValue.choice.SliceSupportList\
            .list.array[sliceLstIdx]->sNSSAI.sST.buf,(*ieExtend)->list.array[idx]->\
            extensionValue.choice.SliceSupportList.\
            list.array[sliceLstIdx]->sNSSAI.sST.size);
      if((*ieExtend)->list.array[idx]->extensionValue.choice.SliceSupportList\
            .list.array[sliceLstIdx]->sNSSAI.sST.buf == NULLP)
      {
         DU_LOG("ERROR  --> DU_APP : BuildExtensions(): Memory allocation failed");
         return RFAILED;
      }
      (*ieExtend)->list.array[idx]->extensionValue.choice.SliceSupportList.\
         list.array[sliceLstIdx]->sNSSAI.sST.buf[0] = duCfgParam.srvdCellLst[0].duCellInfo.\
         cellInfo.srvdPlmn[0].taiSliceSuppLst.snssai[sliceLstIdx]->sst;
      
      DU_ALLOC((*ieExtend)->list.array[idx]->extensionValue.choice.SliceSupportList.\
            list.array[sliceLstIdx]->sNSSAI.sD,sizeof(OCTET_STRING_t));
      if((*ieExtend)->list.array[idx]->extensionValue.choice.SliceSupportList.\
            list.array[sliceLstIdx]->sNSSAI.sD == NULLP)
      {
         DU_LOG("ERROR  --> DU_APP : BuildExtensions(): Memory allocation failed");
         return RFAILED;
      }
      (*ieExtend)->list.array[idx]->extensionValue.choice.SliceSupportList.\
         list.array[sliceLstIdx]->sNSSAI.sD->size = 3 * sizeof(uint8_t);
      DU_ALLOC((*ieExtend)->list.array[idx]->extensionValue.choice.SliceSupportList.\
            list.array[sliceLstIdx]->sNSSAI.sD->buf, (*ieExtend)->list.array[idx]->extensionValue.choice.\
            SliceSupportList.list.array[sliceLstIdx]->sNSSAI.sD->size);
      if((*ieExtend)->list.array[idx]->extensionValue.choice.SliceSupportList.\
            list.array[sliceLstIdx]->sNSSAI.sD->buf == NULLP)
      {
         DU_LOG("ERROR  --> DU_APP : BuildExtensions(): Memory allocation failed");
         return RFAILED;
      }
      memcpy((*ieExtend)->list.array[idx]->extensionValue.choice.SliceSupportList.\
      list.array[sliceLstIdx]->sNSSAI.sD->buf, duCfgParam.srvdCellLst[0].duCellInfo.\
      cellInfo.srvdPlmn[0].taiSliceSuppLst.snssai[sliceLstIdx]->sd, (*ieExtend)->list.array[idx]->\
      extensionValue.choice.SliceSupportList.list.array[sliceLstIdx]->sNSSAI.sD->size);
   }
   return ROK;
}
/*******************************************************************
 *
 * @brief Builds Served PLMN 
 *
 * @details
 *
 *    Function : BuildServedPlmn
 *
 *    Functionality: Building the Served PLMN
 *
 * @params[in] GNB_DU_Served_Cells_Item_t *srvCellItem
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
uint8_t  BuildServedPlmn(ServedPLMNs_List_t *srvplmn)
{  
   uint8_t  plmnidx;
   uint8_t  servPlmnCnt=1;
   uint8_t buildPlmnIdret=0;
   uint8_t BuildExtensionsret=0;
   srvplmn->list.count = servPlmnCnt;
   srvplmn->list.size = \
			servPlmnCnt*sizeof(ServedPLMNs_Item_t *);
   DU_ALLOC(srvplmn->list.array,srvplmn->list.size);
   if(srvplmn->list.array == NULLP)
   {
      return RFAILED;
   }
   for(plmnidx=0; plmnidx<servPlmnCnt; plmnidx++)
   {   
      DU_ALLOC(srvplmn->list.array[plmnidx],\
	    sizeof(ServedPLMNs_Item_t));
      if(srvplmn->list.array[plmnidx] == NULLP)
      {
	 return RFAILED;
      }  
   }
   srvplmn->list.array[0]->pLMN_Identity.size = PLMN_SIZE * sizeof(uint8_t);
   DU_ALLOC(srvplmn->list.array[0]->pLMN_Identity.buf, srvplmn->list.array[0]->pLMN_Identity.size);
   buildPlmnIdret = buildPlmnId(duCfgParam.srvdCellLst[0].duCellInfo.cellInfo.nrCgi.plmn,\
	 srvplmn->list.array[0]->pLMN_Identity.buf);
   if(buildPlmnIdret!= ROK)
   {
      return RFAILED;
   }
   BuildExtensionsret = BuildExtensions(&srvplmn->list.array[0]->iE_Extensions);
   if(BuildExtensionsret!= ROK)
   {
      return RFAILED;
   }
   return ROK;
}

/*******************************************************************
 *
 * @brief Frees Measurement Timing configuration 
 *
 * @details
 *
 *    Function : FreeMeasTimingConf
 *
 *    Functionality: Frees Timing Configuration
 *
 * @params[in] MeasurementTimingConfigurationRrc_t measTimingConfRrc
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
void FreeMeasTimingConf(MeasurementTimingConfigurationRrc_t   measTimingConfRrc)
{
   uint8_t measIeIdx = 0;
   MeasurementTimingConfigurationRrc_IEs_t  *measTimingConfIEs = NULLP;

   if(measTimingConfRrc.criticalExtensions.choice.c1)
   {
      if(measTimingConfRrc.criticalExtensions.choice.c1->choice.measTimingConf)
      {
         measTimingConfIEs = measTimingConfRrc.criticalExtensions.choice.c1->choice.measTimingConf;
         if(measTimingConfIEs->measTiming)
         {
            if(measTimingConfIEs->measTiming->list.array)
            {
               for(measIeIdx = 0; measIeIdx < measTimingConfIEs->measTiming->list.count; measIeIdx++)
               {
                  if(measTimingConfIEs->measTiming->list.array[measIeIdx])
                  {
                     DU_FREE(measTimingConfIEs->measTiming->list.array[measIeIdx]->frequencyAndTiming, sizeof(struct MeasTiming__frequencyAndTiming));
                     DU_FREE(measTimingConfIEs->measTiming->list.array[measIeIdx], sizeof(MeasTiming_t));
                  }
               }
               DU_FREE(measTimingConfIEs->measTiming->list.array, measTimingConfIEs->measTiming->list.size);
            }
            DU_FREE(measTimingConfIEs->measTiming, sizeof(MeasTimingList_t));
         }
         DU_FREE(measTimingConfRrc.criticalExtensions.choice.c1->choice.measTimingConf, sizeof(MeasurementTimingConfigurationRrc_IEs_t));
      }
      DU_FREE(measTimingConfRrc.criticalExtensions.choice.c1, sizeof(struct MeasurementTimingConfigurationRrc__criticalExtensions__c1));
   }
}

/*******************************************************************
 *
 * @brief Builds MEasuerment Timing Configuration
 *
 * @details
 *
 *    Function : BuildMeasTimingConf
 *
 *    Functionality: Building Measurement Timing Configuration
 *
 * @params[in] Pointer to octet string to store Measurement timing
 *             Configuration
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
uint8_t BuildMeasTimingConf(OCTET_STRING_t *measTimingConf)
{
   uint8_t ret = RFAILED;
   uint8_t elementCnt = 0;
   uint8_t measIeIdx = 0;
   asn_enc_rval_t encRetVal;
   MeasurementTimingConfigurationRrc_t   measTimingConfRrc;
   MeasurementTimingConfigurationRrc_IEs_t  *measTimingConfIEs = NULLP;
   struct MeasTiming__frequencyAndTiming  *freqAndTiming = NULLP;

   while(true)
   {
      measTimingConfRrc.criticalExtensions.present = MeasurementTimingConfigurationRrc__criticalExtensions_PR_c1;
      DU_ALLOC(measTimingConfRrc.criticalExtensions.choice.c1, sizeof(struct MeasurementTimingConfigurationRrc__criticalExtensions__c1));
      if(!measTimingConfRrc.criticalExtensions.choice.c1)
      {
         DU_LOG("ERROR  --> DU_APP : BuildMeasTimingConf(): Memory allocation failure for critical extension choice C1");
         break;
      }
      measTimingConfRrc.criticalExtensions.choice.c1->present = MeasurementTimingConfigurationRrc__criticalExtensions__c1_PR_measTimingConf;

      DU_ALLOC(measTimingConfRrc.criticalExtensions.choice.c1->choice.measTimingConf, sizeof(MeasurementTimingConfigurationRrc_IEs_t));
      if(!measTimingConfRrc.criticalExtensions.choice.c1->choice.measTimingConf)
      {
         DU_LOG("ERROR  --> DU_APP : BuildMeasTimingConf(): Memory allocation failure for measTimingConf");
         break;
      }
      measTimingConfIEs = measTimingConfRrc.criticalExtensions.choice.c1->choice.measTimingConf;

      DU_ALLOC(measTimingConfIEs->measTiming, sizeof(MeasTimingList_t));
      if(!measTimingConfIEs->measTiming)
      {
         DU_LOG("ERROR  --> DU_APP : BuildMeasTimingConf(): Memory allocation failure for MeasTimingList");
         break;
      }

      elementCnt = 1;  
      measTimingConfIEs->measTiming->list.count = elementCnt;
      measTimingConfIEs->measTiming->list.size = elementCnt * sizeof(MeasTiming_t *);
      DU_ALLOC(measTimingConfIEs->measTiming->list.array, measTimingConfIEs->measTiming->list.size);
      if(!measTimingConfIEs->measTiming->list.array)
      {
         DU_LOG("ERROR  --> DU_APP : BuildMeasTimingConf(): Memory allocation failure for MeasTimingList array");
         break;
      }

      for(measIeIdx = 0; measIeIdx < elementCnt; measIeIdx++)
      {
         DU_ALLOC(measTimingConfIEs->measTiming->list.array[measIeIdx], sizeof(MeasTiming_t));
         if(!measTimingConfIEs->measTiming->list.array[measIeIdx])
         {
            DU_LOG("ERROR  --> DU_APP : BuildMeasTimingConf(): Memory allocation failure for MeasTimingList array index %d", measIeIdx);
            break;
         }
      }
      if(measIeIdx < elementCnt)
      {
         break;
      }

      measIeIdx = 0;
      DU_ALLOC(measTimingConfIEs->measTiming->list.array[measIeIdx]->frequencyAndTiming, sizeof(struct MeasTiming__frequencyAndTiming));
      if(!measTimingConfIEs->measTiming->list.array[measIeIdx]->frequencyAndTiming)
      {
         DU_LOG("ERROR  --> DU_APP : BuildMeasTimingConf(): Memory allocation failure for frequencyAndTiming");
         break;
      }
      freqAndTiming = measTimingConfIEs->measTiming->list.array[measIeIdx]->frequencyAndTiming;
      freqAndTiming->carrierFreq = MEAS_TIMING_ARFCN;
      freqAndTiming->ssbSubcarrierSpacing = duCfgParam.macCellCfg.ssbCfg.scsCmn;
      freqAndTiming->ssb_MeasurementTimingConfiguration.periodicityAndOffset.present = duCfgParam.macCellCfg.ssbCfg.ssbPeriod + 1;
      freqAndTiming->ssb_MeasurementTimingConfiguration.periodicityAndOffset.choice.sf20 = duCfgParam.macCellCfg.ssbCfg.ssbScOffset;
      freqAndTiming->ssb_MeasurementTimingConfiguration.duration = duCfgParam.srvdCellLst[0].duCellInfo.measTimeCfgDuration;

      /* Encode the F1SetupRequest type as APER */
      xer_fprint(stdout, &asn_DEF_MeasurementTimingConfigurationRrc, &measTimingConfRrc);

      memset(encBuf, 0, ENC_BUF_MAX_LEN);
      encBufSize = 0;
      encRetVal = uper_encode(&asn_DEF_MeasurementTimingConfigurationRrc, 0, &measTimingConfRrc, PrepFinalEncBuf, encBuf);

      /* Encode results */
      if(encRetVal.encoded == ENCODE_FAIL)
      {     
         DU_LOG("\nERROR  -->  F1AP : Could not encode Measurement Timing Configuration structure (at %s)\n",\
               encRetVal.failed_type ? encRetVal.failed_type->name : "unknown");
         break;
      }     
      else  
      {     
         DU_LOG("\nDEBUG   -->  F1AP : Created APER encoded buffer for Measurement Timing Configuration \n");
#ifdef DEBUG_ASN_PRINT
         for(measIeIdx=0; measIeIdx< encBufSize; measIeIdx++)
         {
            printf("%x",encBuf[measIeIdx]);
         }
#endif

         measTimingConf->size = encBufSize;
         DU_ALLOC(measTimingConf->buf, encBufSize);
         if(measTimingConf->buf == NULLP)
         {
            DU_LOG("\nERROR  -->  F1AP : Memory allocation failed for MeasurementTimingConfiguration buffer");
            return RFAILED;
         }
         memcpy(measTimingConf->buf, &encBuf, encBufSize);

         FreeMeasTimingConf(measTimingConfRrc);

         ret = ROK;
         break;
      }
   }
   return ret;
}

/*******************************************************************
 *
 * @brief Builds Served Cell List
 *
 * @details
 *
 *    Function : BuildServedCellList
 *
 *    Functionality: Building Served Cell List
 *
 * @params[in] PLMNID plmn
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/

uint8_t BuildServedCellList(GNB_DU_Served_Cells_List_t *duServedCell)
{
   uint8_t  BuildNrcgiret=0;
   uint8_t  BuildFiveGSTacret=0;
   uint8_t  BuildServedPlmnret=0;
   uint8_t  BuildNrModeret=0;
   uint8_t  idx;
   uint8_t  plmnidx;
   uint8_t  plmnCnt=1;
   GNB_DU_Served_Cells_Item_t *srvCellItem;
   duServedCell->list.size = plmnCnt * sizeof(GNB_DU_Served_Cells_ItemIEs_t *);
   duServedCell->list.count = plmnCnt;

   DU_ALLOC(duServedCell->list.array, duServedCell->list.size);
   if(duServedCell->list.array == NULLP)
   {
      return RFAILED;
   }
   for(plmnidx=0; plmnidx<plmnCnt; plmnidx++)
   {
      DU_ALLOC(duServedCell->list.array[plmnidx],\
            sizeof(GNB_DU_Served_Cells_ItemIEs_t));
      if(duServedCell->list.array[plmnidx] == NULLP)
      {
         return RFAILED;
      }
   }
   idx = 0;
   duServedCell->list.array[idx]->id = ProtocolIE_ID_id_GNB_DU_Served_Cells_Item;
   duServedCell->list.array[idx]->criticality = Criticality_reject;
   duServedCell->list.array[idx]->value.present = \
                                                  GNB_DU_Served_Cells_ItemIEs__value_PR_GNB_DU_Served_Cells_Item;
   srvCellItem = \
                 &duServedCell->list.array[idx]->value.choice.GNB_DU_Served_Cells_Item;
   /*nRCGI*/
   BuildNrcgiret = BuildNrcgi(&srvCellItem->served_Cell_Information.nRCGI);
   if(BuildNrcgiret != ROK)
   {
      return RFAILED;
   }
   /*nRPCI*/
   srvCellItem->served_Cell_Information.nRPCI = \
                                                duCfgParam.srvdCellLst[0].duCellInfo.cellInfo.nrPci;

   /* fiveGS_TAC */
   BuildFiveGSTacret = BuildFiveGSTac(&srvCellItem->served_Cell_Information);
   if(BuildFiveGSTacret != ROK)
   {
      return RFAILED;
   }

   /* Served PLMNs */
   BuildServedPlmnret = BuildServedPlmn(&srvCellItem->served_Cell_Information.servedPLMNs);
   if(BuildServedPlmnret !=ROK)
   {
      return RFAILED;
   }

   /* nR Mode Info with FDD/TDD */
   BuildNrModeret = BuildNrMode(&srvCellItem->served_Cell_Information.nR_Mode_Info);
   if(BuildNrModeret != ROK)
   {
      return RFAILED;
   }

   /*Measurement timing Config*/
   if(BuildMeasTimingConf(&srvCellItem->served_Cell_Information.measurementTimingConfiguration) != ROK)
      return RFAILED;

   /* GNB DU System Information */
   DU_ALLOC(srvCellItem->gNB_DU_System_Information,
         sizeof(GNB_DU_System_Information_t));
   if(!srvCellItem->gNB_DU_System_Information)
   {
      return RFAILED;
   }
   /* MIB */
   srvCellItem->gNB_DU_System_Information->mIB_message.size = duCfgParam.srvdCellLst[0].duSysInfo.mibLen;
   DU_ALLOC(srvCellItem->gNB_DU_System_Information->mIB_message.buf,
         srvCellItem->gNB_DU_System_Information->mIB_message.size);
   if(!srvCellItem->gNB_DU_System_Information->mIB_message.buf)
   {
      return RFAILED;
   }
   memcpy(srvCellItem->gNB_DU_System_Information->mIB_message.buf, duCfgParam.srvdCellLst[0].duSysInfo.mibMsg, \
         srvCellItem->gNB_DU_System_Information->mIB_message.size);

   /* SIB1 */
   srvCellItem->gNB_DU_System_Information->sIB1_message.size =\
                                                              duCfgParam.srvdCellLst[0].duSysInfo.sib1Len;

   DU_ALLOC(srvCellItem->gNB_DU_System_Information->sIB1_message.buf,
         srvCellItem->gNB_DU_System_Information->sIB1_message.size);
   if(!srvCellItem->gNB_DU_System_Information->sIB1_message.buf)
   {
      return RFAILED;
   }
   for(int x=0; x<srvCellItem->gNB_DU_System_Information->sIB1_message.size; x++)
   {
      srvCellItem->gNB_DU_System_Information->sIB1_message.buf[x]=\
                                                                  duCfgParam.srvdCellLst[0].duSysInfo.sib1Msg[x];
   }
   return ROK; 
}                                                                                                                  
/*******************************************************************
 *
 * @brief Builds RRC Version 
 *
 * @details
 *
 *    Function : BuildRrcVer
 *
 *    Functionality: Building RRC Version
 *
 * @params[in] int idx,int elementCnt,RRC_Version_t *rrcver
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
uint8_t BuildRrcVer(RRC_Version_t *rrcVer)
{
   uint8_t rrcExt;
   uint8_t rrcLatest;
   rrcVer->latest_RRC_Version.size = sizeof(uint8_t);
   DU_ALLOC(rrcVer->latest_RRC_Version.buf,sizeof(uint8_t));
   if(rrcVer->latest_RRC_Version.buf == NULLP)
   {
      return RFAILED;
   }
   rrcVer->latest_RRC_Version.buf[0] = 0;
   rrcVer->latest_RRC_Version.bits_unused = 5;
   DU_ALLOC(rrcVer->iE_Extensions,sizeof(ProtocolExtensionContainer_4624P81_t));
   if(rrcVer->iE_Extensions == NULLP)
   {  
      return RFAILED;
   }
   rrcVer->iE_Extensions->list.count = 1;
   rrcVer->iE_Extensions->list.size = sizeof(RRC_Version_ExtIEs_t *);
   DU_ALLOC(rrcVer->iE_Extensions->list.array,rrcVer->iE_Extensions->list.size);
   if(rrcVer->iE_Extensions->list.array == NULLP)
   {
      return RFAILED;
   }
   rrcExt = 0;
   DU_ALLOC(rrcVer->iE_Extensions->list.array[0],\
	 sizeof(RRC_Version_ExtIEs_t));
   if(rrcVer->iE_Extensions->list.array[0] == NULLP)
   {
      return RFAILED;
   }
   rrcVer->iE_Extensions->list.array[rrcExt]->id = \
						   ProtocolIE_ID_id_latest_RRC_Version_Enhanced;
   rrcVer->iE_Extensions->list.array[rrcExt]->criticality = Criticality_reject;
   rrcVer->iE_Extensions->list.array[rrcExt]->extensionValue.present =\
								      RRC_Version_ExtIEs__extensionValue_PR_Latest_RRC_Version_Enhanced;
   rrcVer->iE_Extensions->list.array[rrcExt]->extensionValue.choice\
      .Latest_RRC_Version_Enhanced.size = 3*sizeof(uint8_t);
   DU_ALLOC(rrcVer->iE_Extensions->list.array[rrcExt]->extensionValue.choice\
	 .Latest_RRC_Version_Enhanced.buf,rrcVer->iE_Extensions->list.\
	 array[rrcExt]->extensionValue.choice.Latest_RRC_Version_Enhanced.size);
   if(rrcVer->iE_Extensions->list.array[rrcExt]->extensionValue.choice\
	 .Latest_RRC_Version_Enhanced.buf == NULLP)
   {
      return RFAILED;
   }
   rrcLatest = 0;
   rrcVer->iE_Extensions->list.array[rrcExt]->extensionValue.choice.\
      Latest_RRC_Version_Enhanced.buf[rrcLatest] = 15;
   rrcLatest++;
   rrcVer->iE_Extensions->list.array[rrcExt]->extensionValue.choice.\
      Latest_RRC_Version_Enhanced.buf[rrcLatest] = 5;
   rrcLatest++;
   rrcVer->iE_Extensions->list.array[rrcExt]->extensionValue.choice.\
      Latest_RRC_Version_Enhanced.buf[rrcLatest] = 0;
   return ROK;
}
/*******************************************************************
 *
 * @brief Sends F1 msg over SCTP
 *
 * @details
 *
 *    Function : sendF1APMsg
 *
 *    Functionality: Sends F1 msg over SCTP
 *
 * @params[in] Region region
 *             Pool pool
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
uint8_t sendF1APMsg()
{
   Buffer *mBuf = NULLP;
  
   if(ODU_GET_MSG_BUF(DU_APP_MEM_REGION, DU_POOL, &mBuf) == ROK)
   {
      if(ODU_ADD_POST_MSG_MULT((Data *)encBuf, encBufSize, mBuf) == ROK)
      {
	    ODU_PRINT_MSG(mBuf, 0,0);

	    if(sctpSend(mBuf, F1_INTERFACE) != ROK)
	    {
	       DU_LOG("\nERROR  -->  F1AP : SCTP Send failed");
	       ODU_PUT_MSG_BUF(mBuf);
	       return RFAILED;
	    }
      }
      else
      {
	 DU_LOG("\nERROR  -->  F1AP : ODU_ADD_POST_MSG_MULT failed");
	 ODU_PUT_MSG_BUF(mBuf);
	 return RFAILED;
      }
      ODU_PUT_MSG_BUF(mBuf);
   }
   else
   {
      DU_LOG("\nERROR  -->  F1AP : Failed to allocate memory");
      return RFAILED;
   }
   return ROK; 
} /* sendF1APMsg */

/*******************************************************************
 *
 * @brief  deallocating the memory of function BuildAndSendF1SetupReq()
 *
 * @details
 *
 *    Function :  FreeRrcVer
 *
 *    Functionality: deallocating the memory of function BuildRrcVer
 *
 * @params[in] RRC_Version_t *rrcVer
 * 
 * @return void
 *
 *****************************************************************/
void FreeRrcVer(RRC_Version_t *rrcVer)
{
   if(rrcVer->latest_RRC_Version.buf != NULLP)
   {
      if(rrcVer->iE_Extensions != NULLP)
      {
	 if(rrcVer->iE_Extensions->list.array != NULLP)
	 {
	    if(rrcVer->iE_Extensions->list.array[0] != NULLP)
	    {
	       if(rrcVer->iE_Extensions->list.array[0]->extensionValue.choice.Latest_RRC_Version_Enhanced.buf
		     != NULLP)
	       {
		  DU_FREE(rrcVer->iE_Extensions->list.array[0]->extensionValue.choice\
			.Latest_RRC_Version_Enhanced.buf,rrcVer->iE_Extensions->list.\
			array[0]->extensionValue.choice.Latest_RRC_Version_Enhanced.size);
	       }
	       DU_FREE(rrcVer->iE_Extensions->list.array[0],sizeof(RRC_Version_ExtIEs_t));
	    }
	    DU_FREE(rrcVer->iE_Extensions->list.array,sizeof(RRC_Version_ExtIEs_t*));
	 }
	 DU_FREE(rrcVer->iE_Extensions,sizeof(ProtocolExtensionContainer_4624P81_t));
      }
      DU_FREE(rrcVer->latest_RRC_Version.buf,rrcVer->latest_RRC_Version.size);
   }
}

/*******************************************************************
 *
 * @brief Deallocating memory of TDD NrFreqInfo 
 *
 * @details
 *
 *    Function : freeTddNrFreqInfo 
 *
 *    Functionality: freeTddNrFreqInfo 
 *
 * @params[in]  F1AP_PDU_t *f1apDuCfg
 *
 * @return ROK     - void
 *
 * ****************************************************************/
void freeTddNrFreqInfo(NRFreqInfo_t *freqInfo)
{
   uint8_t freqBandListIdx = 0, supportedBandIdx = 0;

   if(freqInfo->freqBandListNr.list.array)
   {
      for(freqBandListIdx = 0; freqBandListIdx<freqInfo->freqBandListNr.list.count; freqBandListIdx++)
      {
         if(freqInfo->freqBandListNr.list.array[freqBandListIdx])
         {
            if(freqInfo->freqBandListNr.list.array[freqBandListIdx]->supportedSULBandList.list.array)
            {
               for(supportedBandIdx = 0; supportedBandIdx<freqInfo->freqBandListNr.list.array[freqBandListIdx]->supportedSULBandList.list.count; supportedBandIdx++)
               {
                  DU_FREE(freqInfo->freqBandListNr.list.array[freqBandListIdx]->supportedSULBandList.list.array[supportedBandIdx],\
                        sizeof(SupportedSULFreqBandItem_t));
               }
               DU_FREE(freqInfo->freqBandListNr.list.array[freqBandListIdx]->supportedSULBandList.list.array,\
                     freqInfo->freqBandListNr.list.array[freqBandListIdx]->supportedSULBandList.list.size);

            }
            DU_FREE(freqInfo->freqBandListNr.list.array[freqBandListIdx],  sizeof(FreqBandNrItem_t ));
         }
      }
      DU_FREE(freqInfo->freqBandListNr.list.array, freqInfo->freqBandListNr.list.size );
   }
}

/*******************************************************************
 *
 * @brief Deallocating memory allocated for Nr fdd frequencey mode 
 *
 * @details
 *
 *    Function : freeFddNrFreqInfo 
 *
 *    Functionality:Free memory allocated for Nr fdd frequencey mode 
 *
 * @params[in]  
 *
 * @return ROK     - void
 *
 * ****************************************************************/
void freeFddNrFreqInfo(FDD_Info_t *fDD)
{
   uint8_t arrIdx =0;

   if(fDD != NULLP)
   {
      if(fDD->uL_NRFreqInfo.freqBandListNr.list.array != NULLP)
      {
         DU_FREE(fDD->uL_NRFreqInfo.freqBandListNr.list.\
               array[arrIdx], sizeof(FreqBandNrItem_t));
         DU_FREE(fDD->uL_NRFreqInfo.freqBandListNr.list.array, \
               fDD->uL_NRFreqInfo.freqBandListNr.list.size);
      }

      if(fDD->dL_NRFreqInfo.freqBandListNr.list.array != NULLP)
      {
         DU_FREE(fDD->dL_NRFreqInfo.freqBandListNr.list.\
               array[arrIdx], sizeof(FreqBandNrItem_t));
         DU_FREE(fDD->dL_NRFreqInfo.freqBandListNr.list.array,\
               fDD->dL_NRFreqInfo.freqBandListNr.list.size);
      }
      DU_FREE(fDD,sizeof(FDD_Info_t));
   }
}

/*******************************************************************
 *
 * @brief  deallocating the memory of function BuildAndSendF1SetupReq()
 *
 * @details
 *
 *    Function :  FreeServedCellList
 *
 *    Functionality:  deallocating the memory of function BuildServedCellList

 *
 * @params[in] GNB_DU_Served_Cells_List_t *duServedCell
 *
 * @return void
 *
 * ****************************************************************/
void FreeServedCellList( GNB_DU_Served_Cells_List_t *duServedCell)
{
   uint8_t   plmnCnt= 1;
   uint8_t  extensionCnt=IE_EXTENSION_LIST_COUNT;
   uint8_t  plmnIdx=0, sliceIdx=0;
   GNB_DU_Served_Cells_Item_t *srvCellItem;
   ServedPLMNs_Item_t  *servedPlmnItem;
   SliceSupportItem_t  *sliceSupportItem;

   if(duServedCell->list.array!=NULLP)
   {
      if(duServedCell->list.array[0]!=NULLP)
      {
         srvCellItem= &duServedCell->list.array[0]->value.choice.GNB_DU_Served_Cells_Item;

         DU_FREE(srvCellItem->served_Cell_Information.nRCGI.pLMN_Identity.buf,\
               srvCellItem->served_Cell_Information.nRCGI.pLMN_Identity.size * sizeof(uint8_t));
         DU_FREE(srvCellItem->served_Cell_Information.nRCGI.nRCellIdentity.buf,\
               srvCellItem->served_Cell_Information.nRCGI.nRCellIdentity.size * sizeof(uint8_t));

         if(srvCellItem->served_Cell_Information.fiveGS_TAC!=NULLP)
         {
            DU_FREE(srvCellItem->served_Cell_Information.fiveGS_TAC->buf,\
                  sizeof(srvCellItem->served_Cell_Information.fiveGS_TAC->size));
            DU_FREE(srvCellItem->served_Cell_Information.fiveGS_TAC,sizeof(FiveGS_TAC_t));
         }

         if(srvCellItem->served_Cell_Information.servedPLMNs.list.array!=NULLP)
         {
            if(srvCellItem->served_Cell_Information.servedPLMNs.list.array[plmnIdx] != NULLP)
            {
               servedPlmnItem = srvCellItem->served_Cell_Information.servedPLMNs.list.array[plmnIdx];
               DU_FREE(servedPlmnItem->pLMN_Identity.buf, servedPlmnItem->pLMN_Identity.size);

               if(servedPlmnItem->iE_Extensions != NULLP)
               {
                  if(servedPlmnItem->iE_Extensions->list.array != NULLP)
                  {
                     if(servedPlmnItem->iE_Extensions->list.array[0] != NULLP)
                     {
                        if(servedPlmnItem->iE_Extensions->list.array[0]->extensionValue.choice.\
                              SliceSupportList.list.array != NULLP)
                        {
                           for(sliceIdx =0; sliceIdx<servedPlmnItem->iE_Extensions->list.array[0]->\
                                 extensionValue.choice.SliceSupportList.list.count; sliceIdx++)
                           {
                              if(servedPlmnItem->iE_Extensions->list.array[0]->extensionValue.choice.\
                                    SliceSupportList.list.array[sliceIdx] != NULLP)
                              {
                                 sliceSupportItem = servedPlmnItem->iE_Extensions->list.array[0]->\
                                                    extensionValue.choice.SliceSupportList.list.array[sliceIdx];

                                 DU_FREE(sliceSupportItem->sNSSAI.sST.buf, sizeof(uint8_t));

                                 if(sliceSupportItem->sNSSAI.sD != NULLP)
                                 {
                                    DU_FREE(sliceSupportItem->sNSSAI.sD->buf,\
                                          sliceSupportItem->sNSSAI.sD->size);
                                    DU_FREE(sliceSupportItem->sNSSAI.sD, sizeof(OCTET_STRING_t));
                                 }

                                 DU_FREE(servedPlmnItem->iE_Extensions->list.array[0]->extensionValue.\
                                       choice.SliceSupportList.list.array[sliceIdx], sizeof(SliceSupportItem_t));
                              }
                           }
                           DU_FREE(servedPlmnItem->iE_Extensions->list.array[0]->extensionValue.choice.\
                                 SliceSupportList.list.array, servedPlmnItem->iE_Extensions->list.array[0]->\
                                 extensionValue.choice.SliceSupportList.list.size);
                        }
                        DU_FREE(servedPlmnItem->iE_Extensions->list.array[0],\
                              sizeof(ServedPLMNs_ItemExtIEs_t));
                     }
                     DU_FREE(servedPlmnItem->iE_Extensions->list.array,\
                           extensionCnt*sizeof(ServedPLMNs_ItemExtIEs_t*));
                  }
                  DU_FREE(servedPlmnItem->iE_Extensions, sizeof(ProtocolExtensionContainer_4624P3_t));
               }
               DU_FREE(srvCellItem->served_Cell_Information.servedPLMNs.list.array[plmnIdx],\
                     sizeof(ServedPLMNs_Item_t));
            }
            DU_FREE(srvCellItem->served_Cell_Information.servedPLMNs.list.array,\
                  sizeof(ServedPLMNs_Item_t *));
         }

         if(srvCellItem->served_Cell_Information.nR_Mode_Info.present == NR_Mode_Info_PR_fDD)
         {
            freeFddNrFreqInfo(srvCellItem->served_Cell_Information.nR_Mode_Info.choice.fDD);
         }
         else   
         {
            if(srvCellItem->served_Cell_Information.nR_Mode_Info.choice.tDD != NULLP)
            {
               freeTddNrFreqInfo(&srvCellItem->served_Cell_Information.nR_Mode_Info.choice.tDD->nRFreqInfo);
               DU_FREE(srvCellItem->served_Cell_Information.nR_Mode_Info.choice.tDD, sizeof(TDD_Info_t));
            }
         }
         
         DU_FREE(srvCellItem->served_Cell_Information.measurementTimingConfiguration.buf,\
               srvCellItem->served_Cell_Information.measurementTimingConfiguration.size);

         if(srvCellItem->gNB_DU_System_Information != NULLP)
         {
            if(srvCellItem->gNB_DU_System_Information->mIB_message.buf != NULLP)
            {
               DU_FREE(srvCellItem->gNB_DU_System_Information->mIB_message.buf,\
                     srvCellItem->gNB_DU_System_Information->mIB_message.size);
            }

            if(srvCellItem->gNB_DU_System_Information->sIB1_message.buf != NULLP)
            { 
               DU_FREE(srvCellItem->gNB_DU_System_Information->sIB1_message.buf,\
                     srvCellItem->gNB_DU_System_Information->sIB1_message.size);
            }

            DU_FREE(srvCellItem->gNB_DU_System_Information, sizeof(GNB_DU_System_Information_t));
         }

         DU_FREE(duServedCell->list.array[0], sizeof(GNB_DU_Served_Cells_ItemIEs_t));
      }
      DU_FREE(duServedCell->list.array, plmnCnt * sizeof(GNB_DU_Served_Cells_ItemIEs_t*));
   }
}

/*******************************************************************
 *
 * @brief  deallocating the memory of function BuildAndSendF1SetupReq()
 *
 * @details
 *
 *    Function :  FreeF1SetupReq
 *
 *    Functionality:  deallocating the memory of function BuildAndSendF1SetupReq
 *
 * @params[in] F1AP_PDU_t *f1apMsg
 *
 * @return void
 *
 * ****************************************************************/
void FreeF1SetupReq(F1AP_PDU_t *f1apMsg)
{
   uint8_t ieIdx, ieIdx2;
   F1SetupRequest_t *f1SetupReq=NULLP;

   if(f1apMsg != NULLP)
   {
      if(f1apMsg->choice.initiatingMessage != NULLP)
      {
         f1SetupReq = &f1apMsg->choice.initiatingMessage->value.choice.F1SetupRequest;
         if(f1SetupReq->protocolIEs.list.array != NULLP)
         {
            for(ieIdx = 0; ieIdx < f1SetupReq->protocolIEs.list.count; ieIdx++)
            {
               if(f1SetupReq->protocolIEs.list.array[ieIdx] != NULLP)
               {
                  switch(f1SetupReq->protocolIEs.list.array[ieIdx]->id)
                  {
                     case ProtocolIE_ID_id_TransactionID:
                        break;
                     case ProtocolIE_ID_id_gNB_DU_ID:
                        DU_FREE(f1SetupReq->protocolIEs.list.array[ieIdx]->value.choice.GNB_DU_ID.buf,\
                              f1SetupReq->protocolIEs.list.array[ieIdx]->value.choice.GNB_DU_ID.size);
                        break;
                     case ProtocolIE_ID_id_gNB_DU_Name:
                        DU_FREE(f1SetupReq->protocolIEs.list.array[ieIdx]->value.choice.GNB_DU_Name.buf,\
                              strlen((char *)duCfgParam.duName));
                        break;
                     case ProtocolIE_ID_id_gNB_DU_Served_Cells_List:
                        FreeServedCellList(&f1SetupReq->protocolIEs.list.\
                              array[ieIdx]->value.choice.GNB_DU_Served_Cells_List);
                        break;
                     case ProtocolIE_ID_id_GNB_DU_RRC_Version:
                        FreeRrcVer(&f1SetupReq->protocolIEs.list.array[ieIdx]->value.choice.RRC_Version);
                        break;
                     default:
                        DU_LOG("\nERROR  -->  Invalid Event Type %ld", f1SetupReq->protocolIEs.list.array[ieIdx]->id);
                        break;
                  }
               }
            }
            for(ieIdx2=0; ieIdx2< ieIdx; ieIdx2++)
            {
               DU_FREE(f1SetupReq->protocolIEs.list.array[ieIdx2],sizeof(F1SetupRequestIEs_t));
            }
            DU_FREE(f1SetupReq->protocolIEs.list.array,\
                  f1SetupReq->protocolIEs.list.size);
         }
         DU_FREE(f1apMsg->choice.initiatingMessage, sizeof(InitiatingMessage_t));
      }
      DU_FREE(f1apMsg, sizeof(F1AP_PDU_t));
   }
}

/*******************************************************************
 *
 * @brief Builds and Send the F1SetupRequest
 *
 * @details
 *
 *    Function : BuildAndSendF1SetupReq
 *
 * Functionality:Fills the F1SetupRequest
 *
 * @return ROK     - success
 *         RFAILED - failure
 *
 ******************************************************************/
uint8_t BuildAndSendF1SetupReq()
{
   uint8_t   ret, ieIdx, elementCnt;
   F1AP_PDU_t                 *f1apMsg = NULLP;
   F1SetupRequest_t           *f1SetupReq=NULLP;
   GNB_DU_Served_Cells_List_t *duServedCell=NULLP;
   RRC_Version_t              *rrcVer=NULLP;
   asn_enc_rval_t             encRetVal;        /* Encoder return value */
   ret= RFAILED;

   DU_LOG("\nINFO   -->  F1AP : Building F1 Setup Request\n");
   do
   {
      DU_ALLOC(f1apMsg, sizeof(F1AP_PDU_t));
      if(f1apMsg == NULLP)
      {
         break;
      }
      f1apMsg->present = F1AP_PDU_PR_initiatingMessage;
      DU_ALLOC(f1apMsg->choice.initiatingMessage, sizeof(InitiatingMessage_t));
      if(f1apMsg->choice.initiatingMessage == NULLP)
      {
         break;
      }
      f1apMsg->choice.initiatingMessage->procedureCode = ProcedureCode_id_F1Setup;
      f1apMsg->choice.initiatingMessage->criticality = Criticality_reject;
      f1apMsg->choice.initiatingMessage->value.present = \
                                                         InitiatingMessage__value_PR_F1SetupRequest;

      f1SetupReq = &f1apMsg->choice.initiatingMessage->value.choice.F1SetupRequest;

      elementCnt = 5;

      f1SetupReq->protocolIEs.list.count = elementCnt;
      f1SetupReq->protocolIEs.list.size = elementCnt * sizeof(F1SetupRequestIEs_t *);

      /* Initialize the F1Setup members */
      DU_ALLOC(f1SetupReq->protocolIEs.list.array,f1SetupReq->protocolIEs.list.size);
      if(f1SetupReq->protocolIEs.list.array == NULLP)
      {
         break;
      }
      for(ieIdx=0; ieIdx<elementCnt; ieIdx++)
      {
         DU_ALLOC(f1SetupReq->protocolIEs.list.array[ieIdx],\
               sizeof(F1SetupRequestIEs_t));
         if(f1SetupReq->protocolIEs.list.array[ieIdx] == NULLP)
         {
            break;
         }
      }

      ieIdx = 0;
      /*TransactionID*/
      f1SetupReq->protocolIEs.list.array[ieIdx]->id = ProtocolIE_ID_id_TransactionID;
      f1SetupReq->protocolIEs.list.array[ieIdx]->criticality = Criticality_reject;
      f1SetupReq->protocolIEs.list.array[ieIdx]->value.present =\
                                                                F1SetupRequestIEs__value_PR_TransactionID;
      f1SetupReq->protocolIEs.list.array[ieIdx]->value.choice.TransactionID = \
                                                                              TRANS_ID;

      /*DU ID*/
      ieIdx++;
      f1SetupReq->protocolIEs.list.array[ieIdx]->id = ProtocolIE_ID_id_gNB_DU_ID;
      f1SetupReq->protocolIEs.list.array[ieIdx]->criticality = Criticality_reject;
      f1SetupReq->protocolIEs.list.array[ieIdx]->value.present = \
                                                                 F1SetupRequestIEs__value_PR_GNB_DU_ID;
      f1SetupReq->protocolIEs.list.array[ieIdx]->value.choice.GNB_DU_ID.size =\
                                                                              sizeof(uint8_t);

      DU_ALLOC(f1SetupReq->protocolIEs.list.array[ieIdx]->value.choice.GNB_DU_ID.buf,\
            f1SetupReq->protocolIEs.list.array[ieIdx]->value.choice.GNB_DU_ID.size);
      if(f1SetupReq->protocolIEs.list.array[ieIdx]->value.choice.GNB_DU_ID.buf == \
            NULLP)
      {
         break;
      }

      f1SetupReq->protocolIEs.list.array[ieIdx]->value.choice.GNB_DU_ID.buf[0] =\
                                                                                duCfgParam.duId;
      /* DU name IE is of type printableString_t which wireshark is unable to decode.
       * However this string is decoded successfully on online decoders.
       * Since this is an optional IE and the value received in it are not
       * used as of now, eliminating this IE for now to avoid wireshark error.
       */
      /*DU Name*/
      if(duCfgParam.duName != NULL)
      {
         ieIdx++;
         f1SetupReq->protocolIEs.list.array[ieIdx]->id = ProtocolIE_ID_id_gNB_DU_Name;
         f1SetupReq->protocolIEs.list.array[ieIdx]->criticality = Criticality_ignore;
         f1SetupReq->protocolIEs.list.array[ieIdx]->value.present = F1SetupRequestIEs__value_PR_GNB_DU_Name;
         f1SetupReq->protocolIEs.list.array[ieIdx]->value.choice.GNB_DU_Name.size = strlen((char *)duCfgParam.duName);
         DU_ALLOC(f1SetupReq->protocolIEs.list.array[ieIdx]->value.choice.GNB_DU_Name.buf, \
               f1SetupReq->protocolIEs.list.array[ieIdx]->value.choice.GNB_DU_Name.size);
         if(f1SetupReq->protocolIEs.list.array[ieIdx]->value.choice.GNB_DU_Name.buf == NULLP)
         {
            break;
         }
         strcpy((char*)f1SetupReq->protocolIEs.list.array[ieIdx]->value.choice.GNB_DU_Name.buf,
               (char*)duCfgParam.duName);
      }

      /*Served Cell list */
      ieIdx++;
      f1SetupReq->protocolIEs.list.array[ieIdx]->id = \
                                                      ProtocolIE_ID_id_gNB_DU_Served_Cells_List;
      f1SetupReq->protocolIEs.list.array[ieIdx]->criticality = Criticality_reject;
      f1SetupReq->protocolIEs.list.array[ieIdx]->value.present = \
                                                                 F1SetupRequestIEs__value_PR_GNB_DU_Served_Cells_List;
      duServedCell = &f1SetupReq->protocolIEs.list.\
                     array[ieIdx]->value.choice.GNB_DU_Served_Cells_List;
      if(BuildServedCellList(duServedCell))
      {
         break;
      }

      /*RRC Version*/
      ieIdx++;
      f1SetupReq->protocolIEs.list.array[ieIdx]->id = \
                                                      ProtocolIE_ID_id_GNB_DU_RRC_Version ;
      f1SetupReq->protocolIEs.list.array[ieIdx]->criticality = Criticality_reject;
      f1SetupReq->protocolIEs.list.array[ieIdx]->value.present = \
                                                                 F1SetupRequestIEs__value_PR_RRC_Version;
      rrcVer = &f1SetupReq->protocolIEs.list.array[ieIdx]->value.choice.RRC_Version;
      if(BuildRrcVer(rrcVer))
      {
         break;
      }

      xer_fprint(stdout, &asn_DEF_F1AP_PDU, f1apMsg);

      /* Encode the F1SetupRequest type as APER */
      memset(encBuf, 0, ENC_BUF_MAX_LEN);
      encBufSize = 0;
      encRetVal = aper_encode(&asn_DEF_F1AP_PDU, 0, f1apMsg, PrepFinalEncBuf,\
            encBuf);

      /* Encode results */
      if(encRetVal.encoded == ENCODE_FAIL)
      {
         DU_LOG("\nERROR  -->  F1AP : Could not encode F1SetupRequest structure (at %s)\n",\
               encRetVal.failed_type ? encRetVal.failed_type->name : "unknown");
         break;
      }
      else
      {
         DU_LOG("\nDEBUG   -->  F1AP : Created APER encoded buffer for F1SetupRequest\n");
#ifdef DEBUG_ASN_PRINT
         for(ieIdx=0; ieIdx< encBufSize; ieIdx++)
         {
            printf("%x",encBuf[ieIdx]);
         }
#endif
         

      }
      
      /* Sending msg */
      if(sendF1APMsg() != ROK)
      {
         DU_LOG("\nERROR  -->  F1AP : Sending F1 Setup request failed");
         break;
      }
      
      if(fillE2NodeComponentReqInfo(F1, duCfgParam.duId,  E2_NODE_COMPONENT_ADD, encBufSize, encBuf) !=ROK)
      {
         DU_LOG("\nERROR  -->  F1AP : Failed to add the e2 node in the list");
         break;
      }

      ret=ROK;
      break;
   }while(true);

   FreeF1SetupReq(f1apMsg);

   return ret;
}/* End of BuildAndSendF1SetupReq */

/*******************************************************************
 *
 * @brief Deallocating memory allocated for Served_Cells_To_Modify_Item_t
 *
 * @details
 *
 *    Function : freeCellsToModifyItem 
 *
 *    Functionality: Deallocating memory of variables allocated in
 *                    BuildAndSendDUConfigUpdate function
 *
 * @params[in]  Served_Cells_To_Modify_Item_t *modifyItem
 *
 * @return ROK     - void
 *
 * ****************************************************************/

void freeCellsToModifyItem(Served_Cells_To_Modify_Item_t *modifyItem)
{
   uint8_t arrIdx=0, servedPlmnIdx=0, sliceLstIdx=0;
   ServedPLMNs_Item_t *servedPlmnItem = NULLP;
   SliceSupportItem_t *sliceSupportItem = NULLP;

   DU_FREE(modifyItem->oldNRCGI.pLMN_Identity.buf, modifyItem->oldNRCGI.pLMN_Identity.size);
   DU_FREE(modifyItem->oldNRCGI.nRCellIdentity.buf, modifyItem->oldNRCGI.nRCellIdentity.size);

   DU_FREE(modifyItem->served_Cell_Information.nRCGI.pLMN_Identity.buf,\
           modifyItem->served_Cell_Information.nRCGI.pLMN_Identity.size);
   DU_FREE(modifyItem->served_Cell_Information.nRCGI.nRCellIdentity.buf,\
         modifyItem->served_Cell_Information.nRCGI.nRCellIdentity.size);

   if(modifyItem->served_Cell_Information.servedPLMNs.list.array != NULLP)
   {
      if(modifyItem->served_Cell_Information.servedPLMNs.list.array[arrIdx] != NULLP)
      {
         servedPlmnItem = modifyItem->served_Cell_Information.servedPLMNs.list.array[arrIdx];

         DU_FREE(servedPlmnItem->pLMN_Identity.buf,servedPlmnItem->pLMN_Identity.size);

         if(servedPlmnItem->iE_Extensions != NULLP)
         {
            if(servedPlmnItem->iE_Extensions->list.array != NULLP)
            {
               if(servedPlmnItem->iE_Extensions->list.array[arrIdx] != NULLP)
               {
                  if(servedPlmnItem->iE_Extensions->list.array[arrIdx]->extensionValue.choice.SliceSupportList.\
                        list.array != NULLP)
                  {
                     for(sliceLstIdx =0; sliceLstIdx<servedPlmnItem->iE_Extensions->list.array[arrIdx]->\
                           extensionValue.choice.SliceSupportList.list.count; sliceLstIdx++)
                     {
                        if(servedPlmnItem->iE_Extensions->list.array[arrIdx]->extensionValue.choice.SliceSupportList.\
                              list.array[sliceLstIdx] != NULLP)
                        {

                           sliceSupportItem = servedPlmnItem->iE_Extensions->list.array[arrIdx]->extensionValue.choice.\
                                              SliceSupportList.list.array[sliceLstIdx];

                           DU_FREE(sliceSupportItem->sNSSAI.sST.buf, sliceSupportItem->sNSSAI.sST.size);
                           if(sliceSupportItem->sNSSAI.sD != NULLP)
                           {
                              DU_FREE(sliceSupportItem->sNSSAI.sD->buf, sliceSupportItem->sNSSAI.sD->size);
                              DU_FREE(sliceSupportItem->sNSSAI.sD,sizeof(OCTET_STRING_t));
                           }
                           DU_FREE(servedPlmnItem->iE_Extensions->list.array[arrIdx]->extensionValue.choice.\
                                 SliceSupportList.list.array[sliceLstIdx], sizeof(SliceSupportItem_t));
                        }
                     }
                     DU_FREE(servedPlmnItem->iE_Extensions->list.array[arrIdx]->extensionValue.\
                           choice.SliceSupportList.list.array,\
                           servedPlmnItem->iE_Extensions->list.array[arrIdx]->\
                           extensionValue.choice.SliceSupportList.list.size);
                  }
               }
               for(servedPlmnIdx=0; servedPlmnIdx< servedPlmnItem->iE_Extensions->list.count ; servedPlmnIdx++)
               {
                  DU_FREE(servedPlmnItem->iE_Extensions->list.array[servedPlmnIdx], sizeof(ServedPLMNs_ItemExtIEs_t ));
               }
               DU_FREE(servedPlmnItem->iE_Extensions->list.array, servedPlmnItem->iE_Extensions->list.size);
            }
            DU_FREE(servedPlmnItem->iE_Extensions,sizeof(ProtocolExtensionContainer_4624P3_t));
         }
      }
      for(servedPlmnIdx=0; servedPlmnIdx<modifyItem->served_Cell_Information.servedPLMNs.list.count; servedPlmnIdx++)
      {
         DU_FREE(modifyItem->served_Cell_Information.servedPLMNs.list.array[servedPlmnIdx], sizeof(ServedPLMNs_Item_t));
      }
      DU_FREE(modifyItem->served_Cell_Information.servedPLMNs.list.array,\
         modifyItem->served_Cell_Information.servedPLMNs.list.size);
   }
   
   if(modifyItem->served_Cell_Information.nR_Mode_Info.present == NR_Mode_Info_PR_fDD)
   {
      freeFddNrFreqInfo(modifyItem->served_Cell_Information.nR_Mode_Info.choice.fDD);
   }  
   else
   {
      if(modifyItem->served_Cell_Information.nR_Mode_Info.choice.tDD)
      {
         freeTddNrFreqInfo(&modifyItem->served_Cell_Information.nR_Mode_Info.choice.tDD->nRFreqInfo);
         DU_FREE(modifyItem->served_Cell_Information.nR_Mode_Info.choice.tDD, sizeof(TDD_Info_t));
      }
   }
   DU_FREE(modifyItem->served_Cell_Information.measurementTimingConfiguration.buf,\
         modifyItem->served_Cell_Information.measurementTimingConfiguration.size);
}

/*******************************************************************
 *
 * @brief Deallocating memory of BuildAndSendDUConfigUpdate
 *
 * @details
 *
 *    Function : FreeDUConfigUpdate
 *
 *    Functionality: Deallocating memory of variables allocated in
 *                    BuildAndSendDUConfigUpdate function
 *
 * @params[in]  F1AP_PDU_t *f1apDuCfg
 *
 * @return ROK     - void
 *
 * ****************************************************************/
void FreeDUConfigUpdate(F1AP_PDU_t *f1apDuCfg)
{
   uint8_t  idx=0,ieIdx=0, cellModifyIdx=0, cellDeleteIdx=0;
   GNBDUConfigurationUpdate_t *duCfgUpdate = NULLP;
   Served_Cells_To_Modify_List_t  *cellsToModify=NULLP;
   Served_Cells_To_Delete_List_t  *cellsToDelete=NULLP;
   Served_Cells_To_Delete_Item_t  *deleteItem = NULLP;
   Served_Cells_To_Delete_ItemIEs_t *deleteItemIe = NULLP;
   Cells_Status_ItemIEs_t *cellStatusItemIE;

   if(f1apDuCfg != NULLP)
   {
      if(f1apDuCfg->choice.initiatingMessage != NULLP)
      {
         duCfgUpdate = &f1apDuCfg->choice.initiatingMessage->\
                       value.choice.GNBDUConfigurationUpdate;
         if(duCfgUpdate->protocolIEs.list.array != NULLP)
         {
            for(ieIdx=0; ieIdx<duCfgUpdate->protocolIEs.list.count; ieIdx++)
            {
               if(duCfgUpdate->protocolIEs.list.array[ieIdx] != NULLP)
               {
                  switch(duCfgUpdate->protocolIEs.list.array[ieIdx]->id)
                  {
                     case ProtocolIE_ID_id_Served_Cells_To_Modify_List:
                        {
                           cellsToModify = &duCfgUpdate->protocolIEs.list.array[ieIdx]->\
                                           value.choice.Served_Cells_To_Modify_List;
                           if(cellsToModify->list.array != NULLP)
                           {
                              for(cellModifyIdx=0; cellModifyIdx<cellsToModify->list.count ;cellModifyIdx++)
                              {
                                 if(cellsToModify->list.array[cellModifyIdx] != NULLP)
                                 {
                                    freeCellsToModifyItem(&cellsToModify->list.array[cellModifyIdx]->value.choice.\
                                          Served_Cells_To_Modify_Item);
                                    DU_FREE(cellsToModify->list.array[cellModifyIdx],\
                                          sizeof(Served_Cells_To_Modify_ItemIEs_t));
                                 }
                              }
                              DU_FREE(cellsToModify->list.array,cellsToModify->list.size);
                           }
                           break;
                        }
                     case ProtocolIE_ID_id_Served_Cells_To_Delete_List:
                        {
                           cellsToDelete = &duCfgUpdate->protocolIEs.list.array[ieIdx]->\
                                           value.choice.Served_Cells_To_Delete_List;
                           if(cellsToDelete->list.array != NULLP)
                           {
                              for(cellDeleteIdx=0; cellDeleteIdx<cellsToDelete->list.count ;cellDeleteIdx++)
                              {
                                 if(cellsToDelete->list.array[cellDeleteIdx] != NULLP)
                                 {
                                    deleteItemIe = ((Served_Cells_To_Delete_ItemIEs_t*)\
                                          cellsToDelete->list.array[cellDeleteIdx]);
                                    deleteItem=&deleteItemIe->value.choice.Served_Cells_To_Delete_Item;
                                    DU_FREE(deleteItem->oldNRCGI.pLMN_Identity.buf,\
                                          deleteItem->oldNRCGI.pLMN_Identity.size); 
                                    DU_FREE(deleteItem->oldNRCGI.nRCellIdentity.buf,\
                                          deleteItem->oldNRCGI.nRCellIdentity.size);
                                    DU_FREE(cellsToDelete->list.array[cellDeleteIdx],\
                                          sizeof(Served_Cells_To_Delete_ItemIEs_t));
                                 }
                              }
                              DU_FREE(cellsToDelete->list.array,cellsToDelete->list.size);
                           }

                           break;
                        }
                     case ProtocolIE_ID_id_gNB_DU_ID:
                        {
                           DU_FREE(duCfgUpdate->protocolIEs.list.array[ieIdx]->value.choice.GNB_DU_ID.buf,\
                                 duCfgUpdate->protocolIEs.list.array[ieIdx]->value.choice.GNB_DU_ID.size);
                           break;
                        }
                     case ProtocolIE_ID_id_Cells_Status_List:
                        {
                           for(idx = 0; idx < duCfgUpdate->protocolIEs.list.array[ieIdx]->value.choice.Cells_Status_List.list.count; idx++)
                           {
                              cellStatusItemIE = (Cells_Status_ItemIEs_t *)duCfgUpdate->protocolIEs.list.array[ieIdx]->value.choice.Cells_Status_List.list.array[idx];
                              DU_FREE(cellStatusItemIE->value.choice.Cells_Status_Item.nRCGI.nRCellIdentity.buf, cellStatusItemIE->value.choice.Cells_Status_Item.nRCGI.nRCellIdentity.size);
                              DU_FREE(cellStatusItemIE->value.choice.Cells_Status_Item.nRCGI.pLMN_Identity.buf, cellStatusItemIE->value.choice.Cells_Status_Item.nRCGI.pLMN_Identity.size);
                              DU_FREE(duCfgUpdate->protocolIEs.list.array[ieIdx]->value.choice.Cells_Status_List.list.array[idx],sizeof(Cells_Status_ItemIEs_t));
                           }
                           DU_FREE(duCfgUpdate->protocolIEs.list.array[ieIdx]->value.choice.Cells_Status_List.list.array,\
                                 duCfgUpdate->protocolIEs.list.array[ieIdx]->value.choice.Cells_Status_List.list.size);
                        }
                  }
                  DU_FREE(duCfgUpdate->protocolIEs.list.array[ieIdx],\
                        sizeof(GNBDUConfigurationUpdateIEs_t));
               }
            }
            DU_FREE(duCfgUpdate->protocolIEs.list.array,duCfgUpdate->protocolIEs.list.size);
         }
         DU_FREE(f1apDuCfg->choice.initiatingMessage,sizeof(InitiatingMessage_t));
      }
      DU_FREE(f1apDuCfg, (Size)sizeof(F1AP_PDU_t));
   }
}

/*******************************************************************
 *
 * @brief Fills Served Plmns required in ServCellInfo IE
 *
 * @details
 *
 *    Function : fillServedPlmns
 *
 *    Functionality: Fills Served Plmns required in ServCellInfo IE
 *
 * @params[in] Pointer to ServedPLMNs_List_t *
 *
 * @return ROK     - success
 *         RFAILED - failure
 *
 *****************************************************************/

uint8_t fillServedPlmns(ServedPLMNs_List_t *servedPlmn)
{
   uint8_t ieIdx=0, arrayIdx=0, ieListCnt=0, elementCnt=0, sliceLstIdx=0;

   servedPlmn->list.array[arrayIdx]->pLMN_Identity.size = 3*sizeof(uint8_t);
   DU_ALLOC(servedPlmn->list.array[arrayIdx]->pLMN_Identity.buf, servedPlmn->list.\
         array[arrayIdx]->pLMN_Identity.size);
   if(servedPlmn->list.array[arrayIdx]->pLMN_Identity.buf == NULLP)
   {
      DU_LOG("ERROR  --> DU_APP : fillServedPlmns(): Memory allocation failed");
      return RFAILED;
   }
   buildPlmnId(duCfgParam.srvdCellLst[arrayIdx].duCellInfo.cellInfo.srvdPlmn[arrayIdx].plmn,\
         servedPlmn->list.array[arrayIdx]->pLMN_Identity.buf);
   DU_ALLOC(servedPlmn->list.array[arrayIdx]->iE_Extensions,sizeof(ProtocolExtensionContainer_4624P3_t));
   if(servedPlmn->list.array[arrayIdx]->iE_Extensions == NULLP)
   {
      DU_LOG("ERROR  --> DU_APP : fillServedPlmns(): Memory allocation failed");
      return RFAILED;
   }

   ieListCnt=1;
   servedPlmn->list.array[arrayIdx]->iE_Extensions->list.count = ieListCnt;
   servedPlmn->list.array[arrayIdx]->iE_Extensions->list.size = ieListCnt *sizeof(ServedPLMNs_ItemExtIEs_t *);
   DU_ALLOC(servedPlmn->list.array[arrayIdx]->iE_Extensions->list.array,servedPlmn->list.array[arrayIdx]->\
         iE_Extensions->list.size);
   if(servedPlmn->list.array[arrayIdx]->iE_Extensions->list.array == NULLP)
   {
      DU_LOG("ERROR  --> DU_APP : fillServedPlmns(): Memory allocation failed");
      return RFAILED;
   }
   for(ieIdx=arrayIdx;ieIdx<ieListCnt;ieIdx++)
   {
      DU_ALLOC(servedPlmn->list.array[arrayIdx]->iE_Extensions->list.array[ieIdx],\
            sizeof(ServedPLMNs_ItemExtIEs_t));
      if(servedPlmn->list.array[arrayIdx]->iE_Extensions->list.array[ieIdx] == NULLP)
      {
         DU_LOG("ERROR  --> DU_APP : fillServedPlmns(): Memory allocation failed");
         return RFAILED;
      }
   }
   
   ieIdx = 0;
   elementCnt = duCfgParam.srvdCellLst[0].duCellInfo.cellInfo.srvdPlmn[0].taiSliceSuppLst.numSupportedSlices; 
   servedPlmn->list.array[arrayIdx]->iE_Extensions->list.array[ieIdx]->id =ProtocolIE_ID_id_TAISliceSupportList;
   servedPlmn->list.array[arrayIdx]->iE_Extensions->list.array[ieIdx]->criticality = Criticality_ignore;
   servedPlmn->list.array[arrayIdx]->iE_Extensions->list.array[ieIdx]->extensionValue.present = \
   ServedPLMNs_ItemExtIEs__extensionValue_PR_SliceSupportList;
   servedPlmn->list.array[arrayIdx]->iE_Extensions->list.array[ieIdx]->extensionValue.choice.SliceSupportList.\
      list.count = elementCnt;
   servedPlmn->list.array[arrayIdx]->\
      iE_Extensions->list.array[ieIdx]->extensionValue.choice.SliceSupportList.\
      list.size = elementCnt * sizeof(SliceSupportItem_t *);
   DU_ALLOC(servedPlmn->list.array[arrayIdx]->\
         iE_Extensions->list.array[ieIdx]->extensionValue.choice.SliceSupportList.\
         list.array,servedPlmn->list.array[arrayIdx]->\
         iE_Extensions->list.array[ieIdx]->extensionValue.choice.SliceSupportList.list.size);
   if(servedPlmn->list.array[arrayIdx]->\
         iE_Extensions->list.array[ieIdx]->extensionValue.choice.SliceSupportList.\
         list.array == NULLP)
   {
      DU_LOG("ERROR  --> DU_APP : fillServedPlmns(): Memory allocation failed");
      return RFAILED;
   }

   for(sliceLstIdx =0; sliceLstIdx< elementCnt; sliceLstIdx++)
   {
      DU_ALLOC(servedPlmn->list.array[arrayIdx]->\
      iE_Extensions->list.array[ieIdx]->extensionValue.choice.SliceSupportList.\
      list.array[sliceLstIdx],sizeof( SliceSupportItem_t));
      if(servedPlmn->list.array[arrayIdx]->\
      iE_Extensions->list.array[ieIdx]->extensionValue.choice.SliceSupportList.\
      list.array[sliceLstIdx] == NULLP)
      {   
         DU_LOG("ERROR  --> DU_APP : fillServedPlmns(): Memory allocation failed");
         return RFAILED;
      }
      
      servedPlmn->list.array[arrayIdx]->\
      iE_Extensions->list.array[ieIdx]->extensionValue.choice.SliceSupportList.\
      list.array[sliceLstIdx]->sNSSAI.sST.size = sizeof(uint8_t);
      DU_ALLOC(servedPlmn->list.array[arrayIdx]->\
      iE_Extensions->list.array[ieIdx]->extensionValue.choice.SliceSupportList.\
      list.array[sliceLstIdx]->sNSSAI.sST.buf,servedPlmn->list.array[arrayIdx]->\
      iE_Extensions->list.array[ieIdx]->extensionValue.choice.SliceSupportList.list.array[sliceLstIdx]->\
      sNSSAI.sST.size);
      
      if(servedPlmn->list.array[arrayIdx]->\
      iE_Extensions->list.array[ieIdx]->extensionValue.choice.SliceSupportList.\
      list.array[sliceLstIdx]->sNSSAI.sST.buf == NULLP)
      {
         DU_LOG("ERROR  --> DU_APP : fillServedPlmns(): Memory allocation failed");
         return RFAILED;
      }
      servedPlmn->list.array[arrayIdx]->\
      iE_Extensions->list.array[ieIdx]->extensionValue.choice.SliceSupportList.\
      list.array[sliceLstIdx]->sNSSAI.sST.buf[arrayIdx] =  duCfgParam.srvdCellLst[arrayIdx].duCellInfo.\
      cellInfo.srvdPlmn[arrayIdx].taiSliceSuppLst.snssai[sliceLstIdx]->sst;

      DU_ALLOC(servedPlmn->list.array[arrayIdx]->\
      iE_Extensions->list.array[ieIdx]->extensionValue.choice.SliceSupportList.\
      list.array[sliceLstIdx]->sNSSAI.sD,sizeof(OCTET_STRING_t));
      if(servedPlmn->list.array[arrayIdx]->\
      iE_Extensions->list.array[ieIdx]->extensionValue.choice.SliceSupportList.\
      list.array[sliceLstIdx]->sNSSAI.sD == NULLP)
      {
         DU_LOG("ERROR  --> DU_APP : fillServedPlmns(): Memory allocation failed");
         return RFAILED;
      }
      servedPlmn->list.array[arrayIdx]->\
      iE_Extensions->list.array[ieIdx]->extensionValue.choice.SliceSupportList.\
      list.array[sliceLstIdx]->sNSSAI.sD->size = 3 * sizeof(uint8_t);
      DU_ALLOC(servedPlmn->list.array[arrayIdx]->\
      iE_Extensions->list.array[ieIdx]->extensionValue.choice.SliceSupportList.\
      list.array[sliceLstIdx]->sNSSAI.sD->buf,servedPlmn->list.array[arrayIdx]->\
      iE_Extensions->list.array[ieIdx]->extensionValue.choice.SliceSupportList.\
      list.array[sliceLstIdx]->sNSSAI.sD->size);
      if(servedPlmn->list.array[arrayIdx]->\
      iE_Extensions->list.array[ieIdx]->extensionValue.choice.SliceSupportList.\
      list.array[sliceLstIdx]->sNSSAI.sD->buf == NULLP)
      {
         DU_LOG("ERROR  --> DU_APP : fillServedPlmns(): Memory allocation failed");
         return RFAILED;
      }
      memcpy(servedPlmn->list.array[arrayIdx]->\
      iE_Extensions->list.array[ieIdx]->extensionValue.choice.SliceSupportList.\
      list.array[sliceLstIdx]->sNSSAI.sD->buf, duCfgParam.srvdCellLst[arrayIdx].duCellInfo.\
      cellInfo.srvdPlmn[arrayIdx].taiSliceSuppLst.snssai[sliceLstIdx]->sd,\
      servedPlmn->list.array[arrayIdx]->iE_Extensions->list.array[ieIdx]->extensionValue.choice.SliceSupportList.\
      list.array[sliceLstIdx]->sNSSAI.sD->size);
   }
   return ROK;
}

/*******************************************************************
 *
 * @brief Fills Nr Fdd Info required in ServCellInfo IE
 *
 * @details
 *
 *    Function : fillNrFddInfo
 *
 *    Functionality: Fills Nr Fdd Info required in ServCellInfo IE
 *
 * @params[in] FDD_Info_t *fDD
 *
 * @return ROK     - success
 *         RFAILED - failure
 *
 *****************************************************************/

uint8_t fillNrFddInfo(FDD_Info_t *fDD)
{
   fDD->uL_NRFreqInfo.nRARFCN = duCfgParam.srvdCellLst[0].duCellInfo.\
      f1Mode.mode.fdd.ulNrFreqInfo.nrArfcn;
   fDD->uL_NRFreqInfo.freqBandListNr.list.count = 1;
   fDD->uL_NRFreqInfo.freqBandListNr.list.size = sizeof(FreqBandNrItem_t*);
   DU_ALLOC(fDD->uL_NRFreqInfo.freqBandListNr.list.\
	 array, fDD->uL_NRFreqInfo.freqBandListNr.list.size);
   if(fDD->uL_NRFreqInfo.freqBandListNr.list.array == NULLP)
   {
      DU_LOG("\nERROR  --> Memory allocation failed in fillNrFddInfo");
      return RFAILED;
   }

   DU_ALLOC(fDD->uL_NRFreqInfo.freqBandListNr.list.array[0], \
      sizeof(FreqBandNrItem_t));
   if(fDD->uL_NRFreqInfo.freqBandListNr.list.array[0] == NULLP)
   {
      DU_LOG("\nERROR  --> Memory allocation failed in fillNrFddInfo");
      return RFAILED;
   }
   
   fDD->uL_NRFreqInfo.freqBandListNr.list.array[0]->freqBandIndicatorNr = \
      duCfgParam.srvdCellLst[0].duCellInfo.f1Mode.mode.fdd.ulNrFreqInfo.\
      freqBand[0].nrFreqBand;
   fDD->uL_NRFreqInfo.freqBandListNr.list.array[0]->supportedSULBandList.list.count=0;
   fDD->dL_NRFreqInfo.nRARFCN = duCfgParam.srvdCellLst[0].duCellInfo.f1Mode.mode.fdd.\
      dlNrFreqInfo.nrArfcn;
   fDD->dL_NRFreqInfo.freqBandListNr.list.count = 1;
   fDD->dL_NRFreqInfo.freqBandListNr.list.size = sizeof(FreqBandNrItem_t *);
   DU_ALLOC(fDD->dL_NRFreqInfo.freqBandListNr.list.array, fDD->dL_NRFreqInfo.freqBandListNr.list.size);
   if(fDD->dL_NRFreqInfo.freqBandListNr.list.array == NULLP)
   {
      DU_LOG("\nERROR  --> Memory allocation failed in fillNrFddInfo");
      return RFAILED;
   }
   
   DU_ALLOC(fDD->dL_NRFreqInfo.freqBandListNr.list.array[0],  sizeof(FreqBandNrItem_t));
   if(fDD->dL_NRFreqInfo.freqBandListNr.list.array[0] == NULLP)
   {
      DU_LOG("\nERROR  --> Memory allocation failed in fillNrFddInfo");
      return RFAILED;
   }

   fDD->dL_NRFreqInfo.freqBandListNr.list.array[0]->freqBandIndicatorNr = \
      duCfgParam.srvdCellLst[0].duCellInfo.f1Mode.mode.fdd.dlNrFreqInfo.\
      freqBand[0].nrFreqBand;
   fDD->dL_NRFreqInfo.freqBandListNr.list.array[0]->supportedSULBandList.list.count=0;
   
   /*Transmission Bandwidth*/
   fDD->uL_Transmission_Bandwidth.nRSCS = duCfgParam.srvdCellLst[0].duCellInfo.\
      f1Mode.mode.fdd.ulTxBw.nrScs;
   fDD->uL_Transmission_Bandwidth.nRNRB = duCfgParam.srvdCellLst[0].duCellInfo.\
      f1Mode.mode.fdd.ulTxBw.nrb;
   fDD->dL_Transmission_Bandwidth.nRSCS = duCfgParam.srvdCellLst[0].duCellInfo.\
      f1Mode.mode.fdd.dlTxBw.nrScs;
   fDD->dL_Transmission_Bandwidth.nRNRB = duCfgParam.srvdCellLst[0].duCellInfo.\
      f1Mode.mode.fdd.dlTxBw.nrb;

   return ROK;
}

/*******************************************************************
 *
 * @brief Fills ServCellInfo IE
 *
 * @details
 *
 *    Function : fillServedCellInfo
 *
 *    Functionality: Fills ServCellInfo
 *
 * @params[in] Pointer to Served_Cell_Information_t *
 *
 * @return ROK     - success
 *         RFAILED - failure
 *
 *****************************************************************/

uint8_t fillServedCellInfo(Served_Cell_Information_t *srvCellInfo)
{
   uint8_t ieIdx, ieListCnt;

   /*nRCGI*/
   srvCellInfo->nRCGI.pLMN_Identity.size =3*sizeof(uint8_t);
   DU_ALLOC(srvCellInfo->nRCGI.pLMN_Identity.buf,\
	 srvCellInfo->nRCGI.pLMN_Identity.size);
   if(srvCellInfo->nRCGI.pLMN_Identity.buf == NULLP)
   {
      DU_LOG("\nERROR  --> Memory allocation failed in fillServedCellInfo");
      return RFAILED;
   }
   buildPlmnId(duCfgParam.srvdCellLst[0].duCellInfo.cellInfo.nrCgi.plmn,\
	 srvCellInfo->nRCGI.pLMN_Identity.buf);
   srvCellInfo->nRCGI.nRCellIdentity.size =5*sizeof(uint8_t);
   DU_ALLOC(srvCellInfo->nRCGI.nRCellIdentity.buf,\
	 srvCellInfo->nRCGI.nRCellIdentity.size);
   if(srvCellInfo->nRCGI.nRCellIdentity.buf == NULLP)
   {   
      DU_LOG("\nERROR  --> Memory allocation failed in fillServedCellInfo");
      return RFAILED;
   }
   
   fillBitString(&srvCellInfo->nRCGI.nRCellIdentity, ODU_VALUE_FOUR, ODU_VALUE_FIVE, duCfgParam.sib1Params.cellIdentity);
   /*nRPCI*/
   srvCellInfo->nRPCI = duCfgParam.srvdCellLst[0].duCellInfo.cellInfo.nrPci;

   /*servedPLMNs*/
   ieListCnt = 1;
   srvCellInfo->servedPLMNs.list.count = ieListCnt;
   srvCellInfo->servedPLMNs.list.size = ieListCnt*sizeof(ServedPLMNs_Item_t *);
   DU_ALLOC(srvCellInfo->servedPLMNs.list.array, srvCellInfo->servedPLMNs.list.size);
   if(srvCellInfo->servedPLMNs.list.array == NULLP)
   {
      DU_LOG("\nERROR  --> Memory allocation failed in fillServedCellInfo");
      return RFAILED;
   }
   for(ieIdx=0; ieIdx < ieListCnt; ieIdx++)
   {
      DU_ALLOC(srvCellInfo->servedPLMNs.list.array[ieIdx], sizeof(ServedPLMNs_Item_t));
      if(srvCellInfo->servedPLMNs.list.array[ieIdx]== NULLP)
      {
         DU_LOG("\nERROR  --> Memory allocation failed in fillServedCellInfo");
         return RFAILED;
      }
   }
   if(fillServedPlmns(&srvCellInfo->servedPLMNs))
   {
      DU_LOG("\nERROR  --> Failed to fill Served Plmn info");
      return RFAILED;
   }

#ifndef NR_TDD
   /*nR Mode Info with FDD*/
   srvCellInfo->nR_Mode_Info.present = NR_Mode_Info_PR_fDD;
   DU_ALLOC(srvCellInfo->nR_Mode_Info.choice.fDD, sizeof(FDD_Info_t));
   if(srvCellInfo->nR_Mode_Info.choice.fDD == NULLP)
   {
      DU_LOG("\nERROR  --> Memory allocation failed in fillServedCellInfo");
      return RFAILED;
   }
   if(fillNrFddInfo(srvCellInfo->nR_Mode_Info.choice.fDD))
   {
       DU_LOG("\nERROR  --> Failed to fill the Nr FDD information");
      return RFAILED;
   }
#else
   srvCellInfo->nR_Mode_Info.present = NR_Mode_Info_PR_tDD;   
   DU_ALLOC(srvCellInfo->nR_Mode_Info.choice.tDD, sizeof(TDD_Info_t));
   if(srvCellInfo->nR_Mode_Info.choice.tDD == NULLP)
   {
      DU_LOG("\nERROR  --> Memory allocation failed in fillServedCellInfo");
      return RFAILED;
   }
   if(fillNrTddInfo(srvCellInfo->nR_Mode_Info.choice.tDD) != ROK)
   {
      DU_LOG("\nERROR  --> Failed to fill the Nr TDD information");
      return RFAILED;
   }
#endif

   /*Measurement timing Config*/
   if(BuildMeasTimingConf(&srvCellInfo->measurementTimingConfiguration) != ROK)
      return RFAILED;

   return ROK;
}

/*******************************************************************
 *
 * @brief Fills ServCellToModItem IE
 *
 * @details
 *
 *    Function : fillServCellToModItem
 *
 *    Functionality: Fills ServCellToModItem IE
 *
 * @params[in] Pointer to Served_Cells_To_Modify_Item_t *
 *
 * @return ROK     - success
 *         RFAILED - failure
 *
 *****************************************************************/

uint8_t fillServCellToModItem(Served_Cells_To_Modify_Item_t *modifyItem)
{
   /*pLMN_Identity*/
   modifyItem->oldNRCGI.pLMN_Identity.size = 3*sizeof(uint8_t);
   DU_ALLOC(modifyItem->oldNRCGI.pLMN_Identity.buf,modifyItem->oldNRCGI.pLMN_Identity.size);
   if(modifyItem->oldNRCGI.pLMN_Identity.buf == NULLP)
   {
      return RFAILED;
   }
   buildPlmnId(duCfgParam.srvdCellLst[0].duCellInfo.cellInfo.nrCgi.plmn,\
	 modifyItem->oldNRCGI.pLMN_Identity.buf);

   /*nRCellIdentity*/
   modifyItem->oldNRCGI.nRCellIdentity.size = 5*sizeof(uint8_t);
   DU_ALLOC(modifyItem->oldNRCGI.nRCellIdentity.buf,\
	 modifyItem->oldNRCGI.nRCellIdentity.size);
   if(modifyItem->oldNRCGI.nRCellIdentity.buf == NULLP)
   {
      return RFAILED;
   }
   fillBitString(&modifyItem->oldNRCGI.nRCellIdentity, ODU_VALUE_FOUR, ODU_VALUE_FIVE, duCfgParam.sib1Params.cellIdentity);

   if(fillServedCellInfo(&modifyItem->served_Cell_Information))
      return RFAILED;
   else
      return ROK;
}

/*******************************************************************
 *
 * @brief Builds ServCellToModList
 *
 * @details
 *
 *    Function : buildServCellToModList
 *
 *    Functionality: Builds the serv cell to Mod List
 *
 * @params[in] Pointer to Served_Cells_To_Modify_List_t *
 *
 * @return ROK     - success
 *         RFAILED - failure
 *
 *****************************************************************/

uint8_t buildServCellToModList(Served_Cells_To_Modify_List_t *cellsToModify)
{
   uint8_t ieListCnt, ieIdx;
   Served_Cells_To_Modify_Item_t *modifyItem = NULLP;

   ieListCnt = 1;
   cellsToModify->list.count = ieListCnt;
   cellsToModify->list.size = ieListCnt*sizeof(Served_Cells_To_Modify_ItemIEs_t *);
   DU_ALLOC(cellsToModify->list.array,cellsToModify->list.size);
   if(cellsToModify->list.array == NULLP)
   {
      return RFAILED;
   }
   for(ieIdx=0; ieIdx< ieListCnt; ieIdx++)
   {
      DU_ALLOC(cellsToModify->list.array[ieIdx],sizeof(Served_Cells_To_Modify_ItemIEs_t));
      if(cellsToModify->list.array[ieIdx] == NULLP)
      {
	 return RFAILED;
      }
   }
   cellsToModify->list.array[0]->id = ProtocolIE_ID_id_Served_Cells_To_Modify_Item;
   cellsToModify->list.array[0]->criticality = Criticality_reject;
   cellsToModify->list.array[0]->value.present =\
      Served_Cells_To_Modify_ItemIEs__value_PR_Served_Cells_To_Modify_Item;
   modifyItem=&cellsToModify->list.array[0]->value.choice.Served_Cells_To_Modify_Item;

   if(fillServCellToModItem(modifyItem))
      return RFAILED;
   else
      return ROK;
}
/*******************************************************************
 *
 * @brief filling the DeleteItemList
 *
 * @details
 *
 *    Function : fillCellToDeleteItem 
 *
 *    Functionality: Filling the DeleteItemIe 
 *
 * @params[in] Pointer to Served_Cells_To_Delete_ItemIEs_t 
 *
 * @return ROK     - success
 *         RFAILED - failure
 *
 *****************************************************************/
uint8_t fillCellToDeleteItem(struct Served_Cells_To_Delete_ItemIEs *deleteItemIe)
{
   Served_Cells_To_Delete_Item_t *deleteItem=NULLP;
   
   deleteItemIe->id = ProtocolIE_ID_id_Served_Cells_To_Delete_Item;
   deleteItemIe->criticality = Criticality_reject;
   deleteItemIe->value.present =\
   Served_Cells_To_Delete_ItemIEs__value_PR_Served_Cells_To_Delete_Item;
   deleteItem=&deleteItemIe->value.choice.Served_Cells_To_Delete_Item;

   /*pLMN_Identity*/
   deleteItem->oldNRCGI.pLMN_Identity.size = 3*sizeof(uint8_t);
   DU_ALLOC(deleteItem->oldNRCGI.pLMN_Identity.buf,deleteItem->oldNRCGI.pLMN_Identity.size);
   if(deleteItem->oldNRCGI.pLMN_Identity.buf == NULLP)
   {
      DU_LOG("ERROR  --> F1AP: fillCellToDeleteItem(): Failed to allocate the memory");
      return RFAILED;
   }
   buildPlmnId(duCfgParam.srvdCellLst[0].duCellInfo.cellInfo.nrCgi.plmn,\
         deleteItem->oldNRCGI.pLMN_Identity.buf);

   /*nRCellIdentity*/
   deleteItem->oldNRCGI.nRCellIdentity.size = 5*sizeof(uint8_t);
   DU_ALLOC(deleteItem->oldNRCGI.nRCellIdentity.buf,\
         deleteItem->oldNRCGI.nRCellIdentity.size);
   if(deleteItem->oldNRCGI.nRCellIdentity.buf == NULLP)
   {
      DU_LOG("ERROR  --> F1AP: fillCellToDeleteItem(): Failed to allocate the memory");
      return RFAILED;
   }
   fillBitString(&deleteItem->oldNRCGI.nRCellIdentity, ODU_VALUE_FOUR, ODU_VALUE_FIVE, duCfgParam.sib1Params.cellIdentity);
   return ROK;
} 

/*******************************************************************
 *
 * @brief Builds ServCellToDeleteList
 *
 * @details
 *
 *    Function : buildServCellToDeleteList
 *
 *    Functionality: Builds the serv cell to delete List
 *
 * @params[in] Pointer to Served_Cells_To_Delete_List_t *
 *
 * @return ROK     - success
 *         RFAILED - failure
 *
 *****************************************************************/
 
uint8_t buildServCellToDeleteList(Served_Cells_To_Delete_List_t *cellsToDelete)
{
   uint8_t ieListCnt, arrIdx;
   
   ieListCnt = 1;
   cellsToDelete->list.count = ieListCnt;
   cellsToDelete->list.size = ieListCnt*sizeof(Served_Cells_To_Delete_ItemIEs_t *);
   
   DU_ALLOC(cellsToDelete->list.array,cellsToDelete->list.size);
   if(cellsToDelete->list.array == NULLP)
   {
      DU_LOG("\nERROR  --> F1AP : buildServCellToDeleteList(): Memory allocation failed");
      return RFAILED;
   }
   
   for(arrIdx=0; arrIdx< ieListCnt; arrIdx++)
   {
      DU_ALLOC(cellsToDelete->list.array[arrIdx],sizeof(Served_Cells_To_Delete_ItemIEs_t));
      if(cellsToDelete->list.array[arrIdx] == NULLP)
      {
         DU_LOG("\nERROR  --> F1AP : buildServCellToDeleteList(): Memory allocation failed");
         return RFAILED;
      }
   }
   
   arrIdx=0;
   if(fillCellToDeleteItem((Served_Cells_To_Delete_ItemIEs_t*)cellsToDelete->list.array[arrIdx]) !=ROK)
   {
      DU_LOG("\nERROR  -->  F1AP: buildServCellToDeleteList(): failed to fill Served_Cells_To_Delete_ItemIEs");
      return RFAILED;
   }
   return ROK;
}

/*******************************************************************
 *
 * @brief Builds CellsStatusList
 *
 * @details
 *
 *    Function : buildCellsStatusList
 *
 *    Functionality: Builds the Cell Status List
 *
 * @params[in] Pointer to Cells_Status_List_t *
 *
 * @return ROK     - success
 *         RFAILED - failure
 *
 *****************************************************************/
uint8_t buildCellsStatusList(Cells_Status_List_t *cellStatusList)
{
   uint8_t elementCnt = 0, idx = 0, ret = ROK;
   Cells_Status_ItemIEs_t *cellStatusItemIE;

   elementCnt = 1;
   cellStatusList->list.count = elementCnt;
   cellStatusList->list.size = elementCnt * sizeof(Cells_Status_ItemIEs_t *);
   DU_ALLOC(cellStatusList->list.array, cellStatusList->list.size);

   for(idx = 0; idx < elementCnt; idx++)
   {
      DU_ALLOC(cellStatusList->list.array[idx], sizeof(Cells_Status_ItemIEs_t));
      if(!cellStatusList->list.array[idx])
      {
         DU_LOG("ERROR  --> F1AP: buildCellsStatusList() memory allocation failure");
         return RFAILED;
      }
   }
   idx = 0;
   cellStatusItemIE = (Cells_Status_ItemIEs_t *)cellStatusList->list.array[idx];
   cellStatusItemIE->id = ProtocolIE_ID_id_Cells_Status_Item;
   cellStatusItemIE->criticality = Criticality_reject;
   cellStatusItemIE->value.present = Cells_Status_ItemIEs__value_PR_Cells_Status_Item;
   ret = BuildNrcgi(&cellStatusItemIE->value.choice.Cells_Status_Item.nRCGI);
   if(ret == RFAILED)
   {
         DU_LOG("ERROR  --> F1AP: buildCellsStatusList() NRCGI failed");
         return RFAILED;
   }
   cellStatusItemIE->value.choice.Cells_Status_Item.service_status.service_state = Service_State_in_service;
   return ROK;
}

/*******************************************************************
 *
 * @brief Builds and sends the DUConfigUpdate
 *
 * @details
 *
 *    Function : BuildAndSendDUConfigUpdate
 *
 *    Functionality: Constructs the DU Update message and sends
 *                   it to the CU through SCTP.
 *
 * @params[in] void **buf,Buffer to which encoded pattern is written into
 * @params[in] int *size,size of buffer
 *
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
uint8_t BuildAndSendDUConfigUpdate(ServCellAction servCellAction)
{
   uint8_t ret =0, ieIdx=0, elementCnt=0;
   bool memAlloctionFailure = false;
   F1AP_PDU_t                 *f1apDuCfg = NULLP;
   GNBDUConfigurationUpdate_t *duCfgUpdate = NULLP;
   asn_enc_rval_t encRetVal;     /* Encoder return value */
   
   memset(&encRetVal, 0, sizeof(asn_enc_rval_t));
   ret= RFAILED;

   while(true)
   {
      DU_LOG("\nINFO   -->  F1AP : Building DU config update\n");
      /* Allocate the memory for F1DuCfg */
      DU_ALLOC(f1apDuCfg, sizeof(F1AP_PDU_t));
      if(f1apDuCfg == NULLP)
      {
         DU_LOG("\nERROR  -->  F1AP : BuildAndSendDUConfigUpdate(): Memory allocation for F1AP-PDU failed");
         break;
      }

      f1apDuCfg->present = F1AP_PDU_PR_initiatingMessage;
      DU_ALLOC(f1apDuCfg->choice.initiatingMessage,sizeof(InitiatingMessage_t));
      if(f1apDuCfg->choice.initiatingMessage == NULLP)
      {
         DU_LOG("\nERROR  -->  F1AP : BuildAndSendDUConfigUpdate(): Memory allocation for F1AP-PDU failed");
         break;
      }

      f1apDuCfg->choice.initiatingMessage->procedureCode = \
                                                           ProcedureCode_id_gNBDUConfigurationUpdate;
      f1apDuCfg->choice.initiatingMessage->criticality = Criticality_reject;
      f1apDuCfg->choice.initiatingMessage->value.present = \
                                                           InitiatingMessage__value_PR_GNBDUConfigurationUpdate;
      duCfgUpdate = &f1apDuCfg->choice.initiatingMessage->value.\
                    choice.GNBDUConfigurationUpdate;
      elementCnt = 4;
      duCfgUpdate->protocolIEs.list.count = elementCnt;
      duCfgUpdate->protocolIEs.list.size = \
                                           elementCnt * sizeof(GNBDUConfigurationUpdateIEs_t*);

      /* Initialize the F1Setup members */
      DU_ALLOC(duCfgUpdate->protocolIEs.list.array,duCfgUpdate->protocolIEs.list.size);
      if(duCfgUpdate->protocolIEs.list.array == NULLP)
      {
         DU_LOG("ERROR  -->  F1AP : BuildAndSendDUConfigUpdate(): Memory allocation failed");
         break;
      }
      for(ieIdx=0; ieIdx<elementCnt; ieIdx++)
      {
         DU_ALLOC(duCfgUpdate->protocolIEs.list.array[ieIdx],sizeof(GNBDUConfigurationUpdateIEs_t));
         if(duCfgUpdate->protocolIEs.list.array[ieIdx] == NULLP)
         {
            DU_LOG("ERROR  -->  F1AP : BuildAndSendDUConfigUpdate(): Memory allocation failed");
            memAlloctionFailure = true;
            break;
         }
      }
      
      if(memAlloctionFailure == true)
      {
         break;
      }
      /*TransactionID*/
      ieIdx = 0;
      duCfgUpdate->protocolIEs.list.array[ieIdx]->id=ProtocolIE_ID_id_TransactionID;
      duCfgUpdate->protocolIEs.list.array[ieIdx]->criticality= Criticality_reject;
      duCfgUpdate->protocolIEs.list.array[ieIdx]->value.present = \
      GNBDUConfigurationUpdateIEs__value_PR_TransactionID;
      duCfgUpdate->protocolIEs.list.array[ieIdx]->value.choice.TransactionID = TRANS_ID;
      
      ieIdx++;
      if(servCellAction == SERV_CELL_TO_MODIFY)
      {
         /*Served Cell to Modify */
         duCfgUpdate->protocolIEs.list.array[ieIdx]->id = \
         ProtocolIE_ID_id_Served_Cells_To_Modify_List;
         duCfgUpdate->protocolIEs.list.array[ieIdx]->criticality =Criticality_reject;
         duCfgUpdate->protocolIEs.list.array[ieIdx]->value.present = \
         GNBDUConfigurationUpdateIEs__value_PR_Served_Cells_To_Modify_List;
         if(buildServCellToModList(&duCfgUpdate->protocolIEs.list.array[ieIdx]->value.choice.\
                  Served_Cells_To_Modify_List))
         {
            DU_LOG("ERROR  --> DU APP : BuildAndSendDUConfigUpdate(): failed to build ServCellToModList");
            break;
         }
      }
      else
      {
         /*Served Cell to Delete */ 
         duCfgUpdate->protocolIEs.list.array[ieIdx]->id = \
         ProtocolIE_ID_id_Served_Cells_To_Delete_List;
         duCfgUpdate->protocolIEs.list.array[ieIdx]->criticality =Criticality_reject;
         duCfgUpdate->protocolIEs.list.array[ieIdx]->value.present = \
         GNBDUConfigurationUpdateIEs__value_PR_Served_Cells_To_Delete_List;
         if(buildServCellToDeleteList(&duCfgUpdate->protocolIEs.list.array[ieIdx]->value.choice.\
         Served_Cells_To_Delete_List)!=ROK)
         {
            DU_LOG("ERROR  --> DU APP : BuildAndSendDUConfigUpdate(): failed to build ServCellToDeleteList");
            break;
         }
         
      }
      // TODO :GNB DU SYS INFO:MIB AND SIB1 INFORMATION TO BE BUILT AND FILLED HERE
      
      /*Cell Status List*/
      ieIdx++;
      duCfgUpdate->protocolIEs.list.array[ieIdx]->id = ProtocolIE_ID_id_Cells_Status_List;
      duCfgUpdate->protocolIEs.list.array[ieIdx]->criticality = Criticality_reject;
      duCfgUpdate->protocolIEs.list.array[ieIdx]->value.present = \
           GNBDUConfigurationUpdateIEs__value_PR_Cells_Status_List;
      ret = buildCellsStatusList(&duCfgUpdate->protocolIEs.list.array[ieIdx]->value.choice.Cells_Status_List);
      if(ret == RFAILED)
      {
         DU_LOG("ERROR  --> DU APP : BuildAndSendDUConfigUpdate(): Cell Status List building failed");
         break;
      }

      /*GNB DU ID */
      ieIdx++;
      duCfgUpdate->protocolIEs.list.array[ieIdx]->id = ProtocolIE_ID_id_gNB_DU_ID;
      duCfgUpdate->protocolIEs.list.array[ieIdx]->criticality = Criticality_reject;
      duCfgUpdate->protocolIEs.list.array[ieIdx]->value.present = \
      GNBDUConfigurationUpdateIEs__value_PR_GNB_DU_ID;
      duCfgUpdate->protocolIEs.list.array[ieIdx]->value.choice.GNB_DU_ID.size = sizeof(uint8_t);
      DU_ALLOC(duCfgUpdate->protocolIEs.list.array[ieIdx]->value.choice.GNB_DU_ID.buf,\
            duCfgUpdate->protocolIEs.list.array[ieIdx]->value.choice.GNB_DU_ID.size);
      if(duCfgUpdate->protocolIEs.list.array[ieIdx]->value.choice.GNB_DU_ID.buf == NULLP)
      {
         DU_LOG("ERROR  --> DU APP : BuildAndSendDUConfigUpdate(): Memory allocation failed for GNB_DU_ID");
         break;
      }
      duCfgUpdate->protocolIEs.list.array[ieIdx]->value.choice.GNB_DU_ID.buf[0] = duCfgParam.duId;

      xer_fprint(stdout, &asn_DEF_F1AP_PDU, f1apDuCfg);

      /* Encode the DU Config Update type as APER */
      memset((uint8_t *)encBuf, 0, ENC_BUF_MAX_LEN);
      encBufSize = 0;
      encRetVal = aper_encode(&asn_DEF_F1AP_PDU, 0, f1apDuCfg, PrepFinalEncBuf, encBuf);

      /* Checking encode results */
      if(encRetVal.encoded == ENCODE_FAIL)
      {
         DU_LOG("ERROR  -->  F1AP : Could not encode DUConfigUpdate structure (at %s)\n",\
               encRetVal.failed_type ? encRetVal.failed_type->name : "unknown");
         break;
      }
      else
      {
         DU_LOG("\nDEBUG   -->  F1AP : Created APER encoded buffer for DUConfigUpdate\n");
#ifdef DEBUG_ASN_PRINT
         for(ieIdx =0; ieIdx < encBufSize; ieIdx++)
         {
            printf("%x",encBuf[ieIdx]);
         }
#endif
      }
      
      /* Sending msg */
      if(sendF1APMsg() != ROK)
      {
         DU_LOG("\nERROR  -->  F1AP : Sending GNB-DU Config Update failed");
         break;
      }
      
      if(fillE2NodeComponentReqInfo(F1, duCfgParam.duId, E2_NODE_COMPONENT_UPDATE, encBufSize, encBuf)!=ROK)
      {
         DU_LOG("\nERROR  -->  F1AP : Failed to update the e2 node in the list");
         break;
      }

      ret = ROK;
      break;
   }
  
   addToReservedF1apPduList(TRANS_ID,f1apDuCfg);
   return ret;
}


/*******************************************************************
 *
 * @brief free the ULRRCMessageTransfer
 *
 * @details
 *
 *    Function : FreeULRRCMessageTransfer
 *
 *    Functionality: Deallocating the memory of variable allocated in
 *                      FreeULRRCMessageTransfer
 *
 * @params[in]
 *
 * @return ROK     - void
 *
 ******************************************************************/
void FreeULRRCMessageTransfer( F1AP_PDU_t *f1apMsg)
{
   uint8_t idx1;
   ULRRCMessageTransfer_t  *ulRRCMsg;

   if(f1apMsg != NULLP)
   { 
      if(f1apMsg->choice.initiatingMessage != NULLP)
      {
	 ulRRCMsg = &f1apMsg->choice.initiatingMessage->value.choice.ULRRCMessageTransfer;
	 if(ulRRCMsg->protocolIEs.list.array != NULLP)
	 {
	    for(idx1=0;idx1<ulRRCMsg->protocolIEs.list.count;idx1++)
	    {
	       if(ulRRCMsg->protocolIEs.list.array[idx1] != NULLP)
	       {
		  if(ulRRCMsg->protocolIEs.list.array[idx1]->value.present ==
			ULRRCMessageTransferIEs__value_PR_RRCContainer)
		  {
		     DU_FREE(ulRRCMsg->protocolIEs.list.array[idx1]->value.choice.RRCContainer.buf,
			   ulRRCMsg->protocolIEs.list.array[idx1]->value.choice.RRCContainer.size);
		  }
		  DU_FREE(ulRRCMsg->protocolIEs.list.array[idx1],sizeof(ULRRCMessageTransferIEs_t));
	       }
	    }
	    DU_FREE(ulRRCMsg->protocolIEs.list.array,ulRRCMsg->protocolIEs.list.size );
	 }
	 DU_FREE(f1apMsg->choice.initiatingMessage,sizeof(InitiatingMessage_t));
      }
      DU_FREE(f1apMsg,sizeof(F1AP_PDU_t));
   }
}
/*******************************************************************
 *
 * @brief Builds and sends the ULRRCMessageTransfer 
 *
 * @details
 *
 *    Function : BuildAndSendULRRCMessageTransfer
 *
 *    Functionality: Constructs the UL RRC Message Transfer and sends
 *                   it to the CU through SCTP.
 *
 * @params[in] 
 *
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
uint8_t BuildAndSendULRRCMessageTransfer(DuUeCb  *ueCb, uint8_t lcId, \
      uint16_t msgLen, uint8_t *rrcMsg)
{
   uint8_t                 elementCnt=0, idx1=0, idx=0;
   uint8_t                 ret = RFAILED;
   F1AP_PDU_t              *f1apMsg = NULLP;
   ULRRCMessageTransfer_t  *ulRRCMsg = NULLP;
   asn_enc_rval_t          encRetVal;        /* Encoder return value */
   
   memset(&encRetVal, 0, sizeof(asn_enc_rval_t));

   while(true)
   {
      DU_LOG("\nINFO   -->  F1AP : Building UL RRC Message Transfer Message\n");

      DU_ALLOC(f1apMsg, sizeof(F1AP_PDU_t));
      if(f1apMsg == NULLP)
      {
	 DU_LOG(" ERROR  -->  F1AP : Memory allocation for F1AP-PDU failed");
	 break;
      }
      f1apMsg->present = F1AP_PDU_PR_initiatingMessage;
      DU_ALLOC(f1apMsg->choice.initiatingMessage,sizeof(InitiatingMessage_t));
      if(f1apMsg->choice.initiatingMessage == NULLP)
      {
	 DU_LOG(" ERROR  -->  F1AP : Memory allocation for	F1AP-PDU failed");
	 break;
      }
      f1apMsg->choice.initiatingMessage->procedureCode = ProcedureCode_id_ULRRCMessageTransfer;
      f1apMsg->choice.initiatingMessage->criticality = Criticality_ignore;
      f1apMsg->choice.initiatingMessage->value.present = \
							 InitiatingMessage__value_PR_ULRRCMessageTransfer;
      ulRRCMsg =
	 &f1apMsg->choice.initiatingMessage->value.choice.ULRRCMessageTransfer;
      elementCnt = 4;
      ulRRCMsg->protocolIEs.list.count = elementCnt;
      ulRRCMsg->protocolIEs.list.size = \
					elementCnt * sizeof(ULRRCMessageTransferIEs_t *);

      /* Initialize the F1Setup members */
      DU_ALLOC(ulRRCMsg->protocolIEs.list.array, ulRRCMsg->protocolIEs.list.size);
      if(ulRRCMsg->protocolIEs.list.array == NULLP)
      {
	 DU_LOG(" ERROR  -->  F1AP : Memory allocation for UL RRC MessageTransferIEs failed");
	 break;
      }
      for(idx=0; idx<elementCnt; idx++)
      {
	 DU_ALLOC(ulRRCMsg->protocolIEs.list.array[idx],sizeof(ULRRCMessageTransferIEs_t));
	 if(ulRRCMsg->protocolIEs.list.array[idx] == NULLP)
	 {
	    break;
	 }
      }

      idx1 = 0;

      /*GNB CU UE F1AP ID*/
      ulRRCMsg->protocolIEs.list.array[idx1]->id = ProtocolIE_ID_id_gNB_CU_UE_F1AP_ID;
      ulRRCMsg->protocolIEs.list.array[idx1]->criticality = Criticality_reject;
      ulRRCMsg->protocolIEs.list.array[idx1]->value.present = \
							      ULRRCMessageTransferIEs__value_PR_GNB_CU_UE_F1AP_ID;
      ulRRCMsg->protocolIEs.list.array[idx1]->value.choice.GNB_CU_UE_F1AP_ID = ueCb->gnbCuUeF1apId;

      /*GNB DU UE F1AP ID*/
      idx1++;
      ulRRCMsg->protocolIEs.list.array[idx1]->id = ProtocolIE_ID_id_gNB_DU_UE_F1AP_ID;
      ulRRCMsg->protocolIEs.list.array[idx1]->criticality = Criticality_reject;
      ulRRCMsg->protocolIEs.list.array[idx1]->value.present = \
							      ULRRCMessageTransferIEs__value_PR_GNB_DU_UE_F1AP_ID;
      ulRRCMsg->protocolIEs.list.array[idx1]->value.choice.GNB_DU_UE_F1AP_ID = ueCb->gnbDuUeF1apId;

      /*SRBID*/
      idx1++;
      ulRRCMsg->protocolIEs.list.array[idx1]->id = ProtocolIE_ID_id_SRBID;
      ulRRCMsg->protocolIEs.list.array[idx1]->criticality = Criticality_reject;
      ulRRCMsg->protocolIEs.list.array[idx1]->value.present = \
							      ULRRCMessageTransferIEs__value_PR_SRBID;
      ulRRCMsg->protocolIEs.list.array[idx1]->value.choice.SRBID = lcId;

      /*RRCContainer*/
      idx1++;
      ulRRCMsg->protocolIEs.list.array[idx1]->id  = ProtocolIE_ID_id_RRCContainer;
      ulRRCMsg->protocolIEs.list.array[idx1]->criticality = Criticality_reject;
      ulRRCMsg->protocolIEs.list.array[idx1]->value.present = \
							      ULRRCMessageTransferIEs__value_PR_RRCContainer;
      ulRRCMsg->protocolIEs.list.array[idx1]->value.choice.RRCContainer.size = msgLen;
      DU_ALLOC(ulRRCMsg->protocolIEs.list.array[idx1]->value.choice.RRCContainer.buf,
	    ulRRCMsg->protocolIEs.list.array[idx1]->value.choice.RRCContainer.size)
      if(!ulRRCMsg->protocolIEs.list.array[idx1]->value.choice.RRCContainer.buf)
      {
	 DU_LOG(" ERROR  -->  F1AP : Memory allocation for BuildAndSendULRRCMessageTransfer failed");
	 break;
      }
      memset(ulRRCMsg->protocolIEs.list.array[idx1]->value.choice.RRCContainer.buf, 0, msgLen);
      memcpy(ulRRCMsg->protocolIEs.list.array[idx1]->value.choice.RRCContainer.buf, \
	    rrcMsg, ulRRCMsg->protocolIEs.list.array[idx1]->value.choice.RRCContainer.size);

      xer_fprint(stdout, &asn_DEF_F1AP_PDU, f1apMsg);

      /* Encode the F1SetupRequest type as APER */
      memset(encBuf, 0, ENC_BUF_MAX_LEN);
      encBufSize = 0;
      encRetVal = aper_encode(&asn_DEF_F1AP_PDU, 0, f1apMsg, PrepFinalEncBuf,\
	    encBuf);
      /* Encode results */
      if(encRetVal.encoded == ENCODE_FAIL)
      {
	 DU_LOG( "\nERROR  -->  F1AP : Could not encode ULRRCMessageTransfer structure (at %s)\n",\
	       encRetVal.failed_type ? encRetVal.failed_type->name : "unknown");
	 break;
      }
      else
      {
	 DU_LOG("\nDEBUG  -->  F1AP : Created APER encoded buffer for ULRRCMessageTransfer\n");
#ifdef DEBUG_ASN_PRINT
	 for(int i=0; i< encBufSize; i++)
	 {
	    printf("%x",encBuf[i]);
	 }
#endif
      }

      /* Sending  msg  */
      if(sendF1APMsg()	!=	ROK)
      {
	 DU_LOG("\nERROR  -->   F1AP : Sending	UL RRC Message Transfer Failed");
	 break;
      }
      ret = ROK;
      break;
   }
   FreeULRRCMessageTransfer(f1apMsg);

   return ret;
}/* End of BuildAndSendULRRCMessageTransfer*/

/*******************************************************************
 *
 * @brief Builds tag config 
 *
 * @details
 *
 *    Function : BuildTagConfig 
 *
 *    Functionality: Builds tag config in MacCellGroupConfig
 *
 * @params[in] TAG_Config *tag_Config
 *
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
uint8_t BuildTagConfig(DuUeCb *ueCb, struct TAG_Config *tagConfig)
{
   struct TAG_Config__tag_ToAddModList *tagList;
   uint8_t                     idx, elementCnt;

   tagConfig->tag_ToReleaseList = NULLP;
   tagConfig->tag_ToAddModList = NULLP;
   DU_ALLOC(tagConfig->tag_ToAddModList, sizeof(struct TAG_Config__tag_ToAddModList));
   if(!tagConfig->tag_ToAddModList)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failure in BuildTagConfig");
      return RFAILED;
   }

   if(ueCb == NULLP)
      elementCnt = ODU_VALUE_ONE;
   else
      elementCnt = ueCb->duMacUeCfg.macCellGrpCfg.tagCfg.addModListCount;

   tagList = tagConfig->tag_ToAddModList;
   tagList->list.count = elementCnt;
   tagList->list.size  =  elementCnt * sizeof(struct TAG *);

   tagList->list.array = NULLP;
   DU_ALLOC(tagList->list.array, tagList->list.size);
   if(!tagList->list.array)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failure in BuildTagConfig");
      return RFAILED;
   }

   for(idx=0; idx<tagList->list.count; idx++)
   {
      tagList->list.array[idx] = NULLP;
      DU_ALLOC(tagList->list.array[idx], sizeof(struct TAG));
      if(!tagList->list.array[idx])
      {
         DU_LOG("\nERROR  -->  F1AP : Memory allocation failure in BuildTagConfig");
         return RFAILED;
      }
   }

   if(ueCb == NULLP)
   {
      idx = 0;
      tagList->list.array[idx]->tag_Id = TAG_ID;
      tagList->list.array[idx]->timeAlignmentTimer = TIME_ALIGNMENT_TMR;
   }
   else
   {
      for(idx=0; idx<tagList->list.count; idx++)
      {
         tagList->list.array[idx]->tag_Id = ueCb->duMacUeCfg.macCellGrpCfg.tagCfg.addModList[idx].tagId;
         tagList->list.array[idx]->timeAlignmentTimer = ueCb->duMacUeCfg.macCellGrpCfg.tagCfg.addModList[idx].timeAlignTimer;
      }
   }

   return ROK;
}

/*******************************************************************
 *
 * @brief Builds PHR Config 
 *
 * @details
 *
 *    Function : BuildPhrConfig
 *
 *    Functionality: Builds phrConfig in MacCellGroupConfig
 *
 * @params[in] PHR Config *
 *
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
uint8_t BuildPhrConfig(DuUeCb *ueCb, struct MAC_CellGroupConfig__phr_Config *phrConfig)
{

   phrConfig->present = MAC_CellGroupConfig__phr_Config_PR_setup;
   phrConfig->choice.setup = NULLP;
   DU_ALLOC(phrConfig->choice.setup, sizeof(struct PHR_Config));
   if(!phrConfig->choice.setup)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failure in BuildPhrConfig");
      return RFAILED;
   }

   if(ueCb == NULLP)
   {
      phrConfig->choice.setup->phr_PeriodicTimer        = PHR_PERIODIC_TMR;
      phrConfig->choice.setup->phr_ProhibitTimer        = PHR_PROHIBHIT_TMR;
      phrConfig->choice.setup->phr_Tx_PowerFactorChange = PHR_PWR_FACTOR_CHANGE;
      phrConfig->choice.setup->multiplePHR              = false;
      phrConfig->choice.setup->dummy                    = false;
      phrConfig->choice.setup->phr_Type2OtherCell       = false;
      phrConfig->choice.setup->phr_ModeOtherCG          = PHR_MODE_OTHER_CG;
   }
   else
   {
      phrConfig->choice.setup->phr_PeriodicTimer        = ueCb->duMacUeCfg.macCellGrpCfg.phrCfg.periodicTimer;
      phrConfig->choice.setup->phr_ProhibitTimer        = ueCb->duMacUeCfg.macCellGrpCfg.phrCfg.prohibitTimer;
      phrConfig->choice.setup->phr_Tx_PowerFactorChange = ueCb->duMacUeCfg.macCellGrpCfg.phrCfg.txPowerFactor;
      phrConfig->choice.setup->multiplePHR              = ueCb->duMacUeCfg.macCellGrpCfg.phrCfg.multiplePHR;
      phrConfig->choice.setup->dummy                    = ueCb->duMacUeCfg.macCellGrpCfg.phrCfg.dummy;
      phrConfig->choice.setup->phr_Type2OtherCell       = ueCb->duMacUeCfg.macCellGrpCfg.phrCfg.phrType2OtherCell;
      phrConfig->choice.setup->phr_ModeOtherCG          = ueCb->duMacUeCfg.macCellGrpCfg.phrCfg.phrOtherCG;
   }

   return ROK;
}

/*******************************************************************
 *
 * @brief Builds BSR Config 
 *
 * @details
 *
 *    Function : BuildBsrConfig
 *
 *    Functionality: Builds BuildBsrConfig in MacCellGroupConfig
 *
 * @params[in] BSR_Config *bsrConfig
 *
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
uint8_t BuildBsrConfig(DuUeCb *ueCb, struct BSR_Config *bsrConfig)
{
   if(ueCb == NULLP)
   {
      bsrConfig->periodicBSR_Timer = PERIODIC_BSR_TMR;
      bsrConfig->retxBSR_Timer     = RETX_BSR_TMR;
      bsrConfig->logicalChannelSR_DelayTimer = NULLP;
   }
   else
   {
      bsrConfig->periodicBSR_Timer = convertBsrPeriodicTmrValueToEnum(ueCb->duMacUeCfg.macCellGrpCfg.bsrTmrCfg.periodicTimer);
      bsrConfig->retxBSR_Timer     = convertBsrRetxTmrValueToEnum(ueCb->duMacUeCfg.macCellGrpCfg.bsrTmrCfg.retxTimer);

      bsrConfig->logicalChannelSR_DelayTimer = NULLP;
      DU_ALLOC(bsrConfig->logicalChannelSR_DelayTimer, sizeof(long));
      if(bsrConfig->logicalChannelSR_DelayTimer == NULLP)
      {
         DU_LOG("\nERROR  --> DU APP: Memory allocation failed in BuildBsrConfig");
         return RFAILED;
      }
      *(bsrConfig->logicalChannelSR_DelayTimer) = convertLcSrDelayTmrValueToEnum(ueCb->duMacUeCfg.macCellGrpCfg.bsrTmrCfg.srDelayTimer);
   }

   return ROK;
}

/*******************************************************************
 *
 * @brief Builds scheduling request config 
 *
 * @details
 *
 *    Function : BuildSchedulingReqConfig 
 *
 *    Functionality: Builds BuildSchedulingReqConfig in MacCellGroupConfig
 *
 * @params[in] SchedulingRequestConfig *schedulingRequestConfig
 *
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
uint8_t BuildSchedulingReqConfig(DuUeCb *ueCb, struct SchedulingRequestConfig *schedulingRequestConfig)
{
   struct SchedulingRequestConfig__schedulingRequestToAddModList *schReqList;
   uint8_t                     idx, elementCnt;

   schedulingRequestConfig->schedulingRequestToAddModList = NULLP;
   DU_ALLOC(schedulingRequestConfig->schedulingRequestToAddModList,
	 sizeof(struct SchedulingRequestConfig__schedulingRequestToAddModList));
   if(!schedulingRequestConfig->schedulingRequestToAddModList)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failure in BuildSchedulingReqConfig");
      return RFAILED;
   }

   if(ueCb == NULLP)
      elementCnt = ODU_VALUE_ONE;
   else
      elementCnt = ueCb->duMacUeCfg.macCellGrpCfg.schReqCfg.addModListCount;

   schReqList = schedulingRequestConfig->schedulingRequestToAddModList;
   schReqList->list.count = elementCnt;
   schReqList->list.size  = elementCnt * sizeof(struct SchedulingRequestToAddMod *);

   schReqList->list.array = NULLP;
   DU_ALLOC(schReqList->list.array, schReqList->list.size);
   if(!schReqList->list.array)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failure in BuildSchedulingReqConfig");
      return RFAILED;
   }

   for(idx=0; idx<schReqList->list.count; idx++)
   {
      schReqList->list.array[idx] = NULLP;
      DU_ALLOC(schReqList->list.array[idx], sizeof(struct SchedulingRequestToAddMod));
      if(!schReqList->list.array[idx])
      {
         DU_LOG("\nERROR  -->  F1AP : Memory allocation failure in BuildSchedulingReqConfig");
         return RFAILED;
      }
   }

   if(ueCb == NULLP)
   {
      idx = 0;
      schReqList->list.array[idx]->schedulingRequestId = SCH_REQ_ID;

      schReqList->list.array[idx]->sr_ProhibitTimer = NULLP;
      DU_ALLOC(schReqList->list.array[idx]->sr_ProhibitTimer, sizeof(long));
      if(!schReqList->list.array[idx]->sr_ProhibitTimer)
      {
         DU_LOG("\nERROR  -->  F1AP : Memory allocation failure in BuildSchedulingReqConfig");
         return RFAILED;
      }
      *(schReqList->list.array[idx]->sr_ProhibitTimer) = SR_PROHIBIT_TMR;
      schReqList->list.array[idx]->sr_TransMax = SR_TRANS_MAX;
   }
   else
   {
      for(idx=0; idx<schReqList->list.count; idx++)
      {
         schReqList->list.array[idx]->schedulingRequestId = ueCb->duMacUeCfg.macCellGrpCfg.schReqCfg.addModList[idx].schedReqId;

         schReqList->list.array[idx]->sr_ProhibitTimer = NULLP;
         DU_ALLOC(schReqList->list.array[idx]->sr_ProhibitTimer, sizeof(long));
         if(!schReqList->list.array[idx]->sr_ProhibitTimer)
         {
            DU_LOG("\nERROR  -->  F1AP : Memory allocation failure in BuildSchedulingReqConfig");
            return RFAILED;
         }
         *(schReqList->list.array[idx]->sr_ProhibitTimer) = ueCb->duMacUeCfg.macCellGrpCfg.schReqCfg.addModList[idx].srProhibitTmr;
         schReqList->list.array[idx]->sr_TransMax = ueCb->duMacUeCfg.macCellGrpCfg.schReqCfg.addModList[idx].srTransMax;
      }
   }

   schedulingRequestConfig->schedulingRequestToReleaseList = NULLP;

   return ROK;
}

/*******************************************************************
 *
 * @brief Builds RLC Configuration for AM mode
 *
 * @details
 *
 *    Function : BuildRlcConfigAm
 *
 *    Functionality: 
 *       Builds AM mode RLC Config in BuildRlcBearerToAddModList
 *
 * @params[in] AmBearerCfg *amCfg
 *             RLC_Config_t  *rlcConfig
 *
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
uint8_t BuildRlcConfigAm(AmBearerCfg *amCfg, struct RLC_Config *rlcConfig)
{
   rlcConfig->choice.am = NULLP;
   DU_ALLOC(rlcConfig->choice.am, sizeof(struct RLC_Config__am));
   if(!rlcConfig->choice.am)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failure in BuildRlcConfigAm");
      return RFAILED;
   }

   /* Fill AM UL configuration */
   rlcConfig->choice.am->ul_AM_RLC.sn_FieldLength = NULLP;
   DU_ALLOC(rlcConfig->choice.am->ul_AM_RLC.sn_FieldLength, sizeof(SN_FieldLengthAM_t));
   if(!rlcConfig->choice.am->ul_AM_RLC.sn_FieldLength)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failure in BuildRlcConfigAm");
      return RFAILED;
   }

   /* Fill default AM UL configuration if input pointer to DU database is NULL */
   if(amCfg == NULLP)
   {
      
      *(rlcConfig->choice.am->ul_AM_RLC.sn_FieldLength) = SN_FIELD_LEN_12BIT; /*As per Spec 38.331, "Network configures only value size12 in SN-FieldLengthAM for SRB"*/
      rlcConfig->choice.am->ul_AM_RLC.t_PollRetransmit  = T_POLL_RETRANSMIT;
      rlcConfig->choice.am->ul_AM_RLC.pollPDU           = POLL_PDU;
      rlcConfig->choice.am->ul_AM_RLC.pollByte          = POLL_BYTE;
      rlcConfig->choice.am->ul_AM_RLC.maxRetxThreshold  = MAX_RETX_THRESHOLD;
   }
   else
   {
      *(rlcConfig->choice.am->ul_AM_RLC.sn_FieldLength) = covertAmSnLenFromIntEnumToRrcEnum(amCfg->dlAmCfg.snLenDl);
      rlcConfig->choice.am->ul_AM_RLC.t_PollRetransmit  = covertPollRetxTmrValueToEnum(amCfg->dlAmCfg.pollRetxTmr);
      rlcConfig->choice.am->ul_AM_RLC.pollPDU           = covertPollPduValueToEnum(amCfg->dlAmCfg.pollPdu);
      rlcConfig->choice.am->ul_AM_RLC.pollByte          = covertPollByteValueToEnum(amCfg->dlAmCfg.pollByte);
      rlcConfig->choice.am->ul_AM_RLC.maxRetxThreshold  = covertMaxRetxValueToEnum(amCfg->dlAmCfg.maxRetxTh);
   }

   /* Fill AM DL configuraion */
   rlcConfig->choice.am->dl_AM_RLC.sn_FieldLength = NULLP;
   DU_ALLOC(rlcConfig->choice.am->dl_AM_RLC.sn_FieldLength, sizeof(SN_FieldLengthAM_t)); 
   if(!rlcConfig->choice.am->dl_AM_RLC.sn_FieldLength)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failure in BuildRlcConfigAm");
      return RFAILED;
   }

   /* Fill default AM DL configuration if input pointer to DU database is NULL */
   if(amCfg == NULLP)
   {
      *(rlcConfig->choice.am->dl_AM_RLC.sn_FieldLength) = SN_FIELD_LEN_12BIT; /*As per Spec 38.331, "Network configures only value size12 in SN-FieldLengthAM for SRB"*/
      rlcConfig->choice.am->dl_AM_RLC.t_Reassembly      = T_REASSEMBLY;
      rlcConfig->choice.am->dl_AM_RLC.t_StatusProhibit  = T_STATUS_PROHIBHIT;
   }
   else /* Fill AM configuration from DU database */
   {
      *(rlcConfig->choice.am->dl_AM_RLC.sn_FieldLength) = covertAmSnLenFromIntEnumToRrcEnum(amCfg->ulAmCfg.snLenUl);
      rlcConfig->choice.am->dl_AM_RLC.t_Reassembly      = convertReasmblTmrValueToEnum(amCfg->ulAmCfg.reAssemTmr);
      rlcConfig->choice.am->dl_AM_RLC.t_StatusProhibit  = convertProhibitTmrValueToEnum(amCfg->ulAmCfg.statProhTmr);
   }
   return ROK;
}

/*******************************************************************
 *
 * @brief Builds RLC Config for UM Bidirection
 *
 * @details
 *
 *    Function : BuildRlcConfig UmBiDir
 *
 *    Functionality: 
 *       Builds RLC Config for UM Bidirection in BuildRlcBearerToAddModList 
 *
 * @params[in] UmBiDirBearerCfg *umBiDirCfg
 *             RLC_Config_t *rlcConfig
 *
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
uint8_t BuildRlcConfigUmBiDir(UmBiDirBearerCfg *umBiDirCfg, struct RLC_Config *rlcConfig)
{
   rlcConfig->choice.um_Bi_Directional = NULLP;
   DU_ALLOC(rlcConfig->choice.um_Bi_Directional, sizeof(struct RLC_Config__um_Bi_Directional));
   if(rlcConfig->choice.um_Bi_Directional == NULLP)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failure in BuildRlcConfigUmBiDir");
      return RFAILED;
   }

   /* Fill UM Bidirectional UL configuration */
   rlcConfig->choice.um_Bi_Directional->ul_UM_RLC.sn_FieldLength = NULLP;
   DU_ALLOC(rlcConfig->choice.um_Bi_Directional->ul_UM_RLC.sn_FieldLength, sizeof(SN_FieldLengthUM_t));
   if(rlcConfig->choice.um_Bi_Directional->ul_UM_RLC.sn_FieldLength == NULLP)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failure in BuildRlcConfigUmBiDir");
      return RFAILED;
   }

   if(umBiDirCfg != NULLP)
   {
      *(rlcConfig->choice.um_Bi_Directional->ul_UM_RLC.sn_FieldLength) = covertUmSnLenFromIntEnumToRrcEnum(umBiDirCfg->dlUmCfg.snLenDlUm);     
   }

   /* Fill UM Bidirectional DL configuration */
   rlcConfig->choice.um_Bi_Directional->dl_UM_RLC.sn_FieldLength = NULLP;
   DU_ALLOC(rlcConfig->choice.um_Bi_Directional->dl_UM_RLC.sn_FieldLength, sizeof(SN_FieldLengthUM_t));
   if(rlcConfig->choice.um_Bi_Directional->dl_UM_RLC.sn_FieldLength == NULLP)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failure in BuildRlcConfigUmBiDir");
      return RFAILED;
   }

   if(umBiDirCfg != NULLP)
   {
      *(rlcConfig->choice.um_Bi_Directional->dl_UM_RLC.sn_FieldLength) = covertUmSnLenFromIntEnumToRrcEnum(umBiDirCfg->ulUmCfg.snLenUlUm);     
      rlcConfig->choice.um_Bi_Directional->dl_UM_RLC.t_Reassembly = convertReasmblTmrValueToEnum(umBiDirCfg->ulUmCfg.reAssemTmr);
   }

   return ROK;
}

/*******************************************************************
 *
 * @brief Builds RLC Config for UM Uni directional UL
 *
 * @details
 *
 *    Function : BuildRlcConfigUmUniDirUl
 *
 *    Functionality: 
 *       Builds RLC Config for UM Unidirection UL in BuildRlcBearerToAddModList 
 *
 * @params[in] UmUniDirDlBearerCfg *umUniDirDlCfg
 *             RLC_Config_t *rlcConfig
 *
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
uint8_t BuildRlcConfigUmUniDirUl(UmUniDirDlBearerCfg *umUniDirDlCfg, RLC_Config_t *rlcConfig)
{
   rlcConfig->choice.um_Uni_Directional_UL = NULLP;
   DU_ALLOC(rlcConfig->choice.um_Uni_Directional_UL , sizeof(struct RLC_Config__um_Uni_Directional_UL));
   if(rlcConfig->choice.um_Uni_Directional_UL == NULLP)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failure in BuildRlcConfigUmUniDirUl");
      return RFAILED;
   }

   rlcConfig->choice.um_Uni_Directional_UL->ul_UM_RLC.sn_FieldLength = NULLP;
   DU_ALLOC(rlcConfig->choice.um_Uni_Directional_UL->ul_UM_RLC.sn_FieldLength, sizeof(SN_FieldLengthUM_t));
   if(rlcConfig->choice.um_Uni_Directional_UL->ul_UM_RLC.sn_FieldLength == NULLP)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failure in BuildRlcConfigUmUniDirUl");
      return RFAILED;
   }

   if(umUniDirDlCfg != NULLP)
   {
      *(rlcConfig->choice.um_Uni_Directional_UL->ul_UM_RLC.sn_FieldLength) = covertUmSnLenFromIntEnumToRrcEnum(umUniDirDlCfg->dlUmCfg.snLenDlUm);
   }

   return ROK;
}

/*******************************************************************
 *
 * @brief Builds RLC Config for UM Uni directional DL
 *
 * @details
 *
 *    Function : BuildRlcConfigUmUniDirDl
 *
 *    Functionality: 
 *       Builds RLC Config for UM Unidirection DL in BuildRlcBearerToAddModList 
 *
 * @params[in] UmUniDirUlBearerCfg *umUniDirUlCfg
 *             RLC_Config_t *rlcConfig
 *
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
uint8_t BuildRlcConfigUmUniDirDl(UmUniDirUlBearerCfg *umUniDirUlCfg, RLC_Config_t *rlcConfig)
{
   rlcConfig->choice.um_Uni_Directional_DL = NULLP;
   DU_ALLOC(rlcConfig->choice.um_Uni_Directional_DL , sizeof(struct RLC_Config__um_Uni_Directional_DL));
   if(rlcConfig->choice.um_Uni_Directional_DL == NULLP)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failure in BuildRlcConfigUmUniDirDl");
      return RFAILED;
   }

   rlcConfig->choice.um_Uni_Directional_DL->dl_UM_RLC.sn_FieldLength = NULLP;
   DU_ALLOC(rlcConfig->choice.um_Uni_Directional_DL->dl_UM_RLC.sn_FieldLength, sizeof(SN_FieldLengthUM_t));
   if(rlcConfig->choice.um_Uni_Directional_DL->dl_UM_RLC.sn_FieldLength == NULLP)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failure in BuildRlcConfigUmUniDirDl");
      return RFAILED;
   }

   if(umUniDirUlCfg != NULLP)
   {
      *(rlcConfig->choice.um_Uni_Directional_DL->dl_UM_RLC.sn_FieldLength) = covertUmSnLenFromIntEnumToRrcEnum(umUniDirUlCfg->ulUmCfg.snLenUlUm);
      rlcConfig->choice.um_Uni_Directional_DL->dl_UM_RLC.t_Reassembly = convertReasmblTmrValueToEnum(umUniDirUlCfg->ulUmCfg.reAssemTmr);
   }

   return ROK;
}

/*******************************************************************
 *
 * @brief Builds RLC Config
 *
 * @details
 *
 *    Function : BuildRlcConfig
 *
 *    Functionality: Builds RLC Config in BuildRlcBearerToAddModList 
 *
 * @params[in] RLC_Config_t *rlcConfig
 *
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
uint8_t BuildRlcConfig(RlcBearerCfg *rbCfg, struct RLC_Config *rlcConfig)
{
   
   /* Fill default values if rbCfg is NULL */
   if(rbCfg == NULLP)
   {
      rlcConfig->present = RLC_Config_PR_am;
      BuildRlcConfigAm(NULLP, rlcConfig);
   }
   /* If RbCfg is present, fill RLC configurations from DU Database */
   else
   {
      rlcConfig->present = covertRlcModeFromIntEnumToRrcEnum(rbCfg->rlcMode);
      switch(rlcConfig->present)
      {
         case RLC_Config_PR_am:
            BuildRlcConfigAm(rbCfg->u.amCfg, rlcConfig);
            break;
         case RLC_Config_PR_um_Bi_Directional:
            BuildRlcConfigUmBiDir(rbCfg->u.umBiDirCfg, rlcConfig);
            break;
         case RLC_Config_PR_um_Uni_Directional_UL:
            BuildRlcConfigUmUniDirUl(rbCfg->u.umUniDirDlCfg, rlcConfig);
            break;
         case RLC_Config_PR_um_Uni_Directional_DL:
            BuildRlcConfigUmUniDirDl(rbCfg->u.umUniDirUlCfg, rlcConfig);
            break;
         case RLC_Config_PR_NOTHING:
         default:
            break;
      }
   }

   return ROK;
}

/*******************************************************************
 *
 * @brief Builds MAC LC Config
 *
 * @details
 *
 *    Function : BuildMacLCConfig 
 *
 *    Functionality: Builds MAC LC Config in BuildRlcBearerToAddModList 
 *
 * @params[in] struct LogicalChannelConfig macLcConfig
 *
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
uint8_t BuildMacLCConfig(LcCfg *lcCfgDb, struct LogicalChannelConfig *macLcConfig)
{
   macLcConfig->ul_SpecificParameters = NULLP;
   DU_ALLOC(macLcConfig->ul_SpecificParameters, sizeof(struct LogicalChannelConfig__ul_SpecificParameters));
   if(!macLcConfig->ul_SpecificParameters)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failure in BuildMacLCConfig");
      return RFAILED;
   }

   if(lcCfgDb == NULLP)
   {
      macLcConfig->ul_SpecificParameters->priority = MAC_LC_PRIORITY;
      macLcConfig->ul_SpecificParameters->prioritisedBitRate =	PRIORTISIED_BIT_RATE;
      macLcConfig->ul_SpecificParameters->bucketSizeDuration =	BUCKET_SIZE_DURATION;
   }
   else
   {
      macLcConfig->ul_SpecificParameters->priority = lcCfgDb->ulLcCfg.priority;
      macLcConfig->ul_SpecificParameters->prioritisedBitRate = lcCfgDb->ulLcCfg.pbr;
      macLcConfig->ul_SpecificParameters->bucketSizeDuration = lcCfgDb->ulLcCfg.bsd;
   }

   macLcConfig->ul_SpecificParameters->allowedServingCells = NULLP;
   macLcConfig->ul_SpecificParameters->allowedSCS_List = NULLP;
   macLcConfig->ul_SpecificParameters->maxPUSCH_Duration = NULLP;
   macLcConfig->ul_SpecificParameters->configuredGrantType1Allowed = NULLP;

   macLcConfig->ul_SpecificParameters->logicalChannelGroup = NULLP;
   DU_ALLOC(macLcConfig->ul_SpecificParameters->logicalChannelGroup,	sizeof(long));
   if(!macLcConfig->ul_SpecificParameters->logicalChannelGroup)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failure in BuildMacLCConfig");
      return RFAILED;
   }

   if(lcCfgDb == NULLP)
      *(macLcConfig->ul_SpecificParameters->logicalChannelGroup) = LC_GRP;
   else
      *(macLcConfig->ul_SpecificParameters->logicalChannelGroup) = lcCfgDb->ulLcCfg.lcGroup;

   macLcConfig->ul_SpecificParameters->schedulingRequestID = NULLP;
   DU_ALLOC(macLcConfig->ul_SpecificParameters->schedulingRequestID,	sizeof(SchedulingRequestId_t));
   if(!macLcConfig->ul_SpecificParameters->schedulingRequestID)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failure in BuildMacLCConfig");
      return RFAILED;
   }

   if(lcCfgDb == NULLP)
      *(macLcConfig->ul_SpecificParameters->schedulingRequestID) = SCH_REQ_ID;
   else
      *(macLcConfig->ul_SpecificParameters->schedulingRequestID) = lcCfgDb->ulLcCfg.schReqId;

   macLcConfig->ul_SpecificParameters->logicalChannelSR_Mask = false;
   macLcConfig->ul_SpecificParameters->logicalChannelSR_DelayTimerApplied = false;
   macLcConfig->ul_SpecificParameters->bitRateQueryProhibitTimer = NULLP;

   return ROK;
}

/*******************************************************************
 *
 * @brief Builds RLC Bearer to Add/Mod list
 *
 * @details
 *
 *    Function :BuildRlcBearerToAddModList 
 *
 *    Functionality: Builds RLC Bearer to Add/Mod list in DuToCuRrcContainer
 *
 * @params[in] rlc_BearerToAddModList
 *
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
uint8_t BuildRlcBearerToAddModList(DuUeCb *ueCb, struct CellGroupConfigRrc__rlc_BearerToAddModList *rlcBearerList)
{
   uint8_t  idx = 0, lcIdx=0, macLcIdx = 0, elementCnt = 0;

   if(ueCb == NULLP)
      elementCnt = 1;
   else if(ueCb->f1UeDb->actionType == UE_CTXT_CFG_QUERY)
      elementCnt = ueCb->duRlcUeCfg.numLcs;
   else
   {
      for(lcIdx = 0; lcIdx<ueCb->duRlcUeCfg.numLcs; lcIdx++)
      {
         if(ueCb->duRlcUeCfg.rlcLcCfg[lcIdx].rlcBearerCfg.isLcAddModRspSent == false)
            elementCnt++;
      }
   }
   rlcBearerList->list.count = elementCnt;
   rlcBearerList->list.size  = elementCnt * sizeof(struct RLC_BearerConfig *);

   rlcBearerList->list.array = NULLP;
   DU_ALLOC(rlcBearerList->list.array, rlcBearerList->list.size);
   if(!rlcBearerList->list.array)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failure in BuildRlcBearerToAddModList");
      return RFAILED;
   }

   for(idx=0; idx<rlcBearerList->list.count; idx++)
   {
      rlcBearerList->list.array[idx] = NULLP;
      DU_ALLOC(rlcBearerList->list.array[idx], sizeof(struct RLC_BearerConfig));
      if(!rlcBearerList->list.array[idx])
      {
         DU_LOG("\nERROR  -->  F1AP : Memory allocation failure in BuildRlcBearerToAddModList");
         return RFAILED;
      }
   }

   if(ueCb == NULLP)
   {
      idx=0;
      rlcBearerList->list.array[idx]->logicalChannelIdentity = SRB1_LCID;
      DU_ALLOC(rlcBearerList->list.array[idx]->servedRadioBearer, sizeof(struct RLC_BearerConfig__servedRadioBearer));
      if(!rlcBearerList->list.array[idx]->servedRadioBearer)
      {     
         DU_LOG("\nERROR  -->  F1AP : Memory allocation failure in BuildRlcBearerToAddModList");
         return RFAILED;
      }     
      rlcBearerList->list.array[idx]->servedRadioBearer->present = RLC_BearerConfig__servedRadioBearer_PR_srb_Identity;
      rlcBearerList->list.array[idx]->servedRadioBearer->choice.srb_Identity = SRB1_LCID;
      rlcBearerList->list.array[idx]->reestablishRLC = NULLP;

      /* Fill RLC related Configurations for this Radio Bearer */
      rlcBearerList->list.array[idx]->rlc_Config = NULLP;
      DU_ALLOC(rlcBearerList->list.array[idx]->rlc_Config, sizeof(struct RLC_Config));
      if(!rlcBearerList->list.array[idx]->rlc_Config)
      {
         DU_LOG("\nERROR  -->  F1AP : Memory allocation failure in BuildRlcBearerToAddModList");
         return RFAILED;
      }
      if(BuildRlcConfig(NULLP, rlcBearerList->list.array[idx]->rlc_Config) != ROK)
      {
         DU_LOG("\nERROR  -->  F1AP : BuildRlcConfig failed");
         return RFAILED;
      }

      /* Fill MAC related configurations for this Radio Bearer */
      rlcBearerList->list.array[idx]->mac_LogicalChannelConfig = NULLP;
      DU_ALLOC(rlcBearerList->list.array[idx]->mac_LogicalChannelConfig, sizeof(struct LogicalChannelConfig));
      if(!rlcBearerList->list.array[idx]->mac_LogicalChannelConfig)
      {
         DU_LOG("\nERROR  -->  F1AP : Memory allocation failure in BuildRlcBearerToAddModList");
         return RFAILED;
      }
      if(BuildMacLCConfig(NULLP, rlcBearerList->list.array[idx]->mac_LogicalChannelConfig) != ROK)
      {
         DU_LOG("\nERROR  -->  F1AP : BuildMacLCConfig failed");
         return RFAILED;
      }
   }
   else
   {
      idx=0;
      for(lcIdx=0; lcIdx<ueCb->duRlcUeCfg.numLcs; lcIdx++)
      {
         if((ueCb->f1UeDb->actionType != UE_CTXT_CFG_QUERY) && (ueCb->duRlcUeCfg.rlcLcCfg[lcIdx].rlcBearerCfg.isLcAddModRspSent == true))
            continue;

         /* Fill Logical channel identity */
         rlcBearerList->list.array[idx]->logicalChannelIdentity = ueCb->duRlcUeCfg.rlcLcCfg[lcIdx].rlcBearerCfg.lcId;

         /* Fill Radio Bearer Id and type (DRB/SRB) for this logical channel */
         DU_ALLOC(rlcBearerList->list.array[idx]->servedRadioBearer, sizeof(struct RLC_BearerConfig__servedRadioBearer));
         if(!rlcBearerList->list.array[idx]->servedRadioBearer)
         {
            DU_LOG("\nERROR  -->  F1AP : Memory allocation failure in BuildRlcBearerToAddModList");
            return RFAILED;
         }
         rlcBearerList->list.array[idx]->servedRadioBearer->present = \
                                                                      covertRbTypeFromIntEnumToRrcEnum(ueCb->duRlcUeCfg.rlcLcCfg[lcIdx].rlcBearerCfg.rbType);
         switch(rlcBearerList->list.array[idx]->servedRadioBearer->present)
         {
            case RLC_BearerConfig__servedRadioBearer_PR_srb_Identity: 
               rlcBearerList->list.array[idx]->servedRadioBearer->choice.srb_Identity = ueCb->duRlcUeCfg.rlcLcCfg[lcIdx].rlcBearerCfg.rbId;
               break;
            case RLC_BearerConfig__servedRadioBearer_PR_drb_Identity:
               rlcBearerList->list.array[idx]->servedRadioBearer->choice.drb_Identity = ueCb->duRlcUeCfg.rlcLcCfg[lcIdx].rlcBearerCfg.rbId;
               break;
            case RLC_BearerConfig__servedRadioBearer_PR_NOTHING:
            default:
               break;
         }
         ueCb->duRlcUeCfg.rlcLcCfg[lcIdx].rlcBearerCfg.isLcAddModRspSent = true;

         rlcBearerList->list.array[idx]->reestablishRLC = NULLP;

         /* Fill RLC related Configurations for this Radio Bearer */
         rlcBearerList->list.array[idx]->rlc_Config = NULLP;
         DU_ALLOC(rlcBearerList->list.array[idx]->rlc_Config, sizeof(struct RLC_Config));
         if(!rlcBearerList->list.array[idx]->rlc_Config)
         {
            DU_LOG("\nERROR  -->  F1AP : Memory allocation failure in BuildRlcBearerToAddModList");
            return RFAILED;
         }
         if(BuildRlcConfig(&ueCb->duRlcUeCfg.rlcLcCfg[lcIdx].rlcBearerCfg, rlcBearerList->list.array[idx]->rlc_Config) != ROK)
         {
            DU_LOG("\nERROR  -->  F1AP : BuildRlcConfig failed");
            return RFAILED;
         }

         /* Fill MAC related configurations for this Radio Bearer */
         rlcBearerList->list.array[idx]->mac_LogicalChannelConfig = NULLP;
         DU_ALLOC(rlcBearerList->list.array[idx]->mac_LogicalChannelConfig, sizeof(struct LogicalChannelConfig));
         if(!rlcBearerList->list.array[idx]->mac_LogicalChannelConfig)
         {
            DU_LOG("\nERROR  -->  F1AP : Memory allocation failure in BuildRlcBearerToAddModList");
            return RFAILED;
         }
         for(macLcIdx = 0; macLcIdx < ueCb->duMacUeCfg.numLcs; macLcIdx++)
         {
            if(ueCb->duMacUeCfg.lcCfgList[macLcIdx].lcConfig.lcId == ueCb->duRlcUeCfg.rlcLcCfg[lcIdx].rlcBearerCfg.lcId)
            {
               if(BuildMacLCConfig(&ueCb->duMacUeCfg.lcCfgList[macLcIdx].lcConfig, rlcBearerList->list.array[idx]->mac_LogicalChannelConfig) != ROK)
               {
                  DU_LOG("\nERROR  -->  F1AP : BuildMacLCConfig failed");
                  return RFAILED;
               }
               break;
            }
         }

         idx++;
      }
   }
   return ROK;
}

/*******************************************************************
 *
 * @brief Build Control resource set to add/modify list 
 *
 * @details
 *
 *    Function : BuildControlRSetToAddModList
 *
 *    Functionality: Build Control resource set to add/modify list
 *
 * @params[in] 
 * struct PDCCH_Config__controlResourceSetToAddModList *controlRSetList
 *
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
uint8_t BuildControlRSetToAddModList(PdcchConfig *pdcchCfg, struct PDCCH_Config__controlResourceSetToAddModList *controlRSetList)
{
   uint8_t idx;
   uint8_t elementCnt;
   uint8_t numBytes, bitsUnused;
   struct ControlResourceSet *controlRSet;
   uint8_t freqDomainResource[FREQ_DOM_RSRC_SIZE] = {0};
   uint8_t coreset0EndPrb, coreset1StartPrb, coreset1NumPrb;

   if(pdcchCfg == NULLP)
      elementCnt = 1;
   else
      elementCnt = pdcchCfg->numCRsetToAddMod;

   controlRSetList->list.count = elementCnt;
   controlRSetList->list.size = elementCnt * sizeof(struct ControlResourceSet *);

   controlRSetList->list.array = NULLP;
   DU_ALLOC(controlRSetList->list.array, controlRSetList->list.size);
   if(!controlRSetList->list.array)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildControlRSetToAddModList");
      return RFAILED;
   }

   for(idx = 0; idx < elementCnt; idx++)
   {
      controlRSetList->list.array[idx] = NULLP;
      DU_ALLOC(controlRSetList->list.array[idx], sizeof(struct ControlResourceSet));
      if(!controlRSetList->list.array[idx])
      {
         DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildControlRSetToAddModList");
         return RFAILED;
      }
   }

   for(idx = 0; idx < elementCnt; idx++)
   {
      controlRSet = controlRSetList->list.array[idx];

      if(pdcchCfg == NULLP)
         controlRSet->controlResourceSetId = PDCCH_CTRL_RSRC_SET_ONE_ID;
      else
         controlRSet->controlResourceSetId = pdcchCfg->cRSetToAddModList[idx].cRSetId;

      /* size 6 bytes
       * 3 LSBs unsued
       * Bit string stored ff0000000000
       */
      numBytes = 6;
      bitsUnused = 3;
      controlRSet->frequencyDomainResources.size = numBytes * sizeof(uint8_t);

      controlRSet->frequencyDomainResources.buf = NULLP;
      DU_ALLOC(controlRSet->frequencyDomainResources.buf, controlRSet->frequencyDomainResources.size);
      if(!controlRSet->frequencyDomainResources.buf)
      {
         DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildControlRSetToAddModList");
         return RFAILED;
      }

      memset(controlRSet->frequencyDomainResources.buf, 0, FREQ_DOM_RSRC_SIZE);

      if(pdcchCfg == NULLP)
      {
         coreset0EndPrb = CORESET0_END_PRB;
         coreset1StartPrb = coreset0EndPrb + 6;
         coreset1NumPrb = CORESET1_NUM_PRB;
         /* calculate the PRBs */
         fillCoresetFeqDomAllocMap(((coreset1StartPrb)/6), (coreset1NumPrb/6), freqDomainResource);
         memcpy(controlRSet->frequencyDomainResources.buf, freqDomainResource, FREQ_DOM_RSRC_SIZE);
         controlRSet->frequencyDomainResources.bits_unused = bitsUnused;

         controlRSet->duration = PDCCH_CTRL_RSRC_SET_ONE_DURATION;
         controlRSet->cce_REG_MappingType.present = ControlResourceSet__cce_REG_MappingType_PR_nonInterleaved;
         controlRSet->precoderGranularity = PDCCH_CTRL_RSRC_SET_ONE_PRECOD_GRANULARITY;
      }
      else
      {
         memcpy(controlRSet->frequencyDomainResources.buf, pdcchCfg->cRSetToAddModList[idx].freqDomainRsrc, FREQ_DOM_RSRC_SIZE);
         controlRSet->frequencyDomainResources.bits_unused = bitsUnused;
         controlRSet->duration = pdcchCfg->cRSetToAddModList[idx].duration;
         controlRSet->cce_REG_MappingType.present = pdcchCfg->cRSetToAddModList[idx].cceRegMappingType;
         controlRSet->precoderGranularity = pdcchCfg->cRSetToAddModList[idx].precoderGranularity;
      }
      controlRSet->tci_StatesPDCCH_ToAddList = NULLP;
      controlRSet->tci_StatesPDCCH_ToReleaseList = NULLP;
      controlRSet->tci_PresentInDCI = NULLP;

#if 0
      uint8_t tciStateIdx;

      DU_ALLOC(controlRset->tci_StatesPDCCH_ToAddList, \
            sizeof(struct ControlResourceSet__tci_StatesPDCCH_ToAddList));
      if(!controlRset->tci_StatesPDCCH_ToAddList)
      {
         DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildControlRSetToAddModList");
         return RFAILED;
      }

      elementCnt = 1;
      controlRset->tci_StatesPDCCH_ToAddList->list.count = elementCnt;
      controlRset->tci_StatesPDCCH_ToAddList->list.size = elementCnt * sizeof(TCI_StateId_t *);
      DU_ALLOC(controlRset->tci_StatesPDCCH_ToAddList->list.array, \
            controlRset->tci_StatesPDCCH_ToAddList->list.size)
         if(!controlRset->tci_StatesPDCCH_ToAddList->list.array)
         {
            DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildControlRSetToAddModList");
            return RFAILED;
         }

      for(tciStateIdx = 0; tciStateIdx <elementCntl; tciStateIdx++)
      {
         DU_ALLOC(controlRset->tci_StatesPDCCH_ToAddList->list.array[tciStateIdx], sizeof(TCI_StateId_t));
         if(!controlRset->tci_StatesPDCCH_ToAddList->list.array[tciStateIdx])
         {
            DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildControlRSetToAddModList");
            return RFAILED;
         }
      }

      tciStateIdx = 0;
      /* TODO */
      *(controlRset->tci_StatesPDCCH_ToAddList->list.array[tciStateIdx]);

      DU_ALLOC(controlRset->tci_PresentInDCI, sizeof(long));
      if(!controlRset->tci_PresentInDCI)
      {
         DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildControlRSetToAddModList");
         return RFAILED;
      }
      /* TODO */
      *(controlRset->tci_PresentInDCI);
#endif

      controlRSet->pdcch_DMRS_ScramblingID = NULLP;
      DU_ALLOC(controlRSet->pdcch_DMRS_ScramblingID, sizeof(long));
      if(!controlRSet->pdcch_DMRS_ScramblingID)
      {
         DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildControlRSetToAddModList");
         return RFAILED;
      }
      if(pdcchCfg == NULLP)
         *(controlRSet->pdcch_DMRS_ScramblingID) = SCRAMBLING_ID;
      else
         *(controlRSet->pdcch_DMRS_ScramblingID) = pdcchCfg->cRSetToAddModList[idx].dmrsScramblingId;
   }
   return ROK;
} /* End BuildControlRSetToAddModList */

/*******************************************************************
 *
 * @brief build SlotOffset for SearchSpace
 *
 * @details
 *
 *    Function : BuildSearchSpcSlotOffset
 *
 *    Functionality: Build Slot Offset for search space to add/modify list
 *
 * @params[in] SearchSpace__monitoringSlotPeriodicityAndOffset *mSlotPeriodicityAndOffset
 *             uint16_t slotOffset
 * @return void
 *
 * ****************************************************************/
void BuildSearchSpcSlotOffset(struct SearchSpace__monitoringSlotPeriodicityAndOffset *mSlotPeriodicityAndOffset,  uint16_t slotOffset)
{
   switch(mSlotPeriodicityAndOffset->present)
   {
      case SearchSpace__monitoringSlotPeriodicityAndOffset_PR_sl1:
         mSlotPeriodicityAndOffset->choice.sl1 = slotOffset;
         break;
      case SearchSpace__monitoringSlotPeriodicityAndOffset_PR_sl2:
         mSlotPeriodicityAndOffset->choice.sl2 = slotOffset;
         break;
      case SearchSpace__monitoringSlotPeriodicityAndOffset_PR_sl4:
         mSlotPeriodicityAndOffset->choice.sl4 = slotOffset;
         break;
      case SearchSpace__monitoringSlotPeriodicityAndOffset_PR_sl5:
         mSlotPeriodicityAndOffset->choice.sl5 = slotOffset;
         break;
      case SearchSpace__monitoringSlotPeriodicityAndOffset_PR_sl8:
         mSlotPeriodicityAndOffset->choice.sl8 = slotOffset;
         break;
      case SearchSpace__monitoringSlotPeriodicityAndOffset_PR_sl10:
         mSlotPeriodicityAndOffset->choice.sl10 = slotOffset;
         break;
      case SearchSpace__monitoringSlotPeriodicityAndOffset_PR_sl16:
         mSlotPeriodicityAndOffset->choice.sl16 = slotOffset;
         break;
      case SearchSpace__monitoringSlotPeriodicityAndOffset_PR_sl20:
         mSlotPeriodicityAndOffset->choice.sl20 = slotOffset;
         break;
      case SearchSpace__monitoringSlotPeriodicityAndOffset_PR_sl40:
         mSlotPeriodicityAndOffset->choice.sl40 = slotOffset;
         break;
      case SearchSpace__monitoringSlotPeriodicityAndOffset_PR_sl80:
         mSlotPeriodicityAndOffset->choice.sl80 = slotOffset;
         break;
      case SearchSpace__monitoringSlotPeriodicityAndOffset_PR_sl160:
         mSlotPeriodicityAndOffset->choice.sl160 = slotOffset;
         break;
      case SearchSpace__monitoringSlotPeriodicityAndOffset_PR_sl320:
         mSlotPeriodicityAndOffset->choice.sl320 = slotOffset;
         break;
      case SearchSpace__monitoringSlotPeriodicityAndOffset_PR_sl640:
         mSlotPeriodicityAndOffset->choice.sl640 = slotOffset;
         break;
      case SearchSpace__monitoringSlotPeriodicityAndOffset_PR_sl1280:
         mSlotPeriodicityAndOffset->choice.sl1280 = slotOffset;
         break;
      case SearchSpace__monitoringSlotPeriodicityAndOffset_PR_sl2560:
         mSlotPeriodicityAndOffset->choice.sl2560 = slotOffset;
         break;
      default:
         break;
   }
}


/*******************************************************************
 *
 * @brief Build search space to add/modify list
 *
 * @details
 *
 *    Function : BuildSearchSpcToAddModList
 *
 *    Functionality: Build search space to add/modify list
 *
 * @params[in] 
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
uint8_t BuildSearchSpcToAddModList(PdcchConfig *pdcchCfg, struct PDCCH_Config__searchSpacesToAddModList *searchSpcList)
{
   uint8_t idx;
   uint8_t numBytes;
   uint8_t byteIdx;
   uint8_t bitsUnused;
   uint8_t elementCnt;
   struct SearchSpace *searchSpc;

   if(pdcchCfg == NULLP)
      elementCnt = 1;
   else
      elementCnt = pdcchCfg->numSearchSpcToAddMod;

   searchSpcList->list.count = elementCnt;
   searchSpcList->list.size = elementCnt * sizeof(struct SearchSpace *);

   searchSpcList->list.array = NULLP;
   DU_ALLOC(searchSpcList->list.array, searchSpcList->list.size);
   if(!searchSpcList->list.array)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildSearchSpcToAddModList");
      return RFAILED;
   }

   for(idx = 0; idx < elementCnt; idx++)
   {
      searchSpcList->list.array[idx] = NULLP;
      DU_ALLOC(searchSpcList->list.array[idx], sizeof(struct SearchSpace));
      if(!searchSpcList->list.array[idx])
      {
         DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildSearchSpcToAddModList");
         return RFAILED;
      }
   }

   for(idx = 0; idx < elementCnt; idx++)
   {
      searchSpc = searchSpcList->list.array[idx];

      if(pdcchCfg == NULLP)
         searchSpc->searchSpaceId = PDCCH_SRCH_SPC_TWO_ID;
      else
         searchSpc->searchSpaceId = pdcchCfg->searchSpcToAddModList[idx].searchSpaceId;

      searchSpc->controlResourceSetId = NULLP;
      DU_ALLOC(searchSpc->controlResourceSetId, sizeof(ControlResourceSetId_t));
      if(!searchSpc->controlResourceSetId)
      {
         DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildSearchSpcToAddModList");
         return RFAILED;
      }
      if(pdcchCfg == NULLP)
         *(searchSpc->controlResourceSetId) = PDCCH_CTRL_RSRC_SET_ONE_ID;
      else
         *(searchSpc->controlResourceSetId) = pdcchCfg->searchSpcToAddModList[idx].cRSetId;

      searchSpc->monitoringSlotPeriodicityAndOffset = NULLP;
      DU_ALLOC(searchSpc->monitoringSlotPeriodicityAndOffset, sizeof(struct SearchSpace__monitoringSlotPeriodicityAndOffset));
      if(!searchSpc->monitoringSlotPeriodicityAndOffset)
      {
         DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildSearchSpcToAddModList");
         return RFAILED;
      }
      if(pdcchCfg == NULLP)
         searchSpc->monitoringSlotPeriodicityAndOffset->present = SearchSpace__monitoringSlotPeriodicityAndOffset_PR_sl1;
      else
      {
         searchSpc->monitoringSlotPeriodicityAndOffset->present = \
                     pdcchCfg->searchSpcToAddModList[idx].mSlotPeriodicityAndOffset.mSlotPeriodicity;
         BuildSearchSpcSlotOffset(searchSpc->monitoringSlotPeriodicityAndOffset, \
                     pdcchCfg->searchSpcToAddModList[idx].mSlotPeriodicityAndOffset.mSlotOffset);
      }

      searchSpc->duration = NULLP;
      searchSpc->monitoringSymbolsWithinSlot = NULLP;
      DU_ALLOC(searchSpc->monitoringSymbolsWithinSlot, sizeof(BIT_STRING_t));
      if(!searchSpc->monitoringSymbolsWithinSlot)
      {
         DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildSearchSpcToAddModList");
         return RFAILED;
      }

      /* Values taken from reference logs :
       * size 2 bytes
       * 2 LSBs unsued
       * Bit string stores 8000
       */
      numBytes = 2;
      bitsUnused = 2;
      searchSpc->monitoringSymbolsWithinSlot->size = numBytes * sizeof(uint8_t);
      searchSpc->monitoringSymbolsWithinSlot->buf = NULLP;
      DU_ALLOC(searchSpc->monitoringSymbolsWithinSlot->buf, searchSpc->monitoringSymbolsWithinSlot->size);
      if(!searchSpc->monitoringSymbolsWithinSlot->buf)
      {
         DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildSearchSpcToAddModList");
         return RFAILED;
      }
      if(pdcchCfg == NULLP)
      {
         byteIdx = 0;
         searchSpc->monitoringSymbolsWithinSlot->buf[byteIdx++] = PDCCH_SYMBOL_WITHIN_SLOT /* setting MSB to 128 i.e. 0x80 */;
         searchSpc->monitoringSymbolsWithinSlot->buf[byteIdx++] = 0;
      }
      else
         memcpy(searchSpc->monitoringSymbolsWithinSlot->buf, pdcchCfg->searchSpcToAddModList[idx].mSymbolsWithinSlot, numBytes);
      searchSpc->monitoringSymbolsWithinSlot->bits_unused = bitsUnused;

      searchSpc->nrofCandidates = NULLP;
      DU_ALLOC(searchSpc->nrofCandidates, sizeof(struct SearchSpace__nrofCandidates));
      if(!searchSpc->nrofCandidates)
      {
         DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildSearchSpcToAddModList");
         return RFAILED;
      }

      if(pdcchCfg == NULLP)
      {
         searchSpc->nrofCandidates->aggregationLevel1 = PDCCH_SRCH_SPC_TWO_AGG_LVL1_CANDIDATE;
         searchSpc->nrofCandidates->aggregationLevel2 = PDCCH_SRCH_SPC_TWO_AGG_LVL2_CANDIDATE;
         searchSpc->nrofCandidates->aggregationLevel4 = PDCCH_SRCH_SPC_TWO_AGG_LVL4_CANDIDATE;
         searchSpc->nrofCandidates->aggregationLevel8 = PDCCH_SRCH_SPC_TWO_AGG_LVL8_CANDIDATE;
         searchSpc->nrofCandidates->aggregationLevel16 = PDCCH_SRCH_SPC_TWO_AGG_LVL16_CANDIDATE;
      }
      else
      {
         searchSpc->nrofCandidates->aggregationLevel1 = pdcchCfg->searchSpcToAddModList[idx].numCandidatesAggLevel1;
         searchSpc->nrofCandidates->aggregationLevel2 = pdcchCfg->searchSpcToAddModList[idx].numCandidatesAggLevel2;
         searchSpc->nrofCandidates->aggregationLevel4 = pdcchCfg->searchSpcToAddModList[idx].numCandidatesAggLevel4;
         searchSpc->nrofCandidates->aggregationLevel8 = pdcchCfg->searchSpcToAddModList[idx].numCandidatesAggLevel8;
         searchSpc->nrofCandidates->aggregationLevel16 = pdcchCfg->searchSpcToAddModList[idx].numCandidatesAggLevel16;
      }

      searchSpc->searchSpaceType = NULLP;
      DU_ALLOC(searchSpc->searchSpaceType, sizeof(struct SearchSpace__searchSpaceType));
      if(!searchSpc->searchSpaceType)
      {
         DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildSearchSpcToAddModList");
         return RFAILED;
      }
      if(pdcchCfg == NULLP)
         searchSpc->searchSpaceType->present = SearchSpace__searchSpaceType_PR_ue_Specific;
      else
         searchSpc->searchSpaceType->present = pdcchCfg->searchSpcToAddModList[idx].searchSpaceType;

      searchSpc->searchSpaceType->choice.ue_Specific = NULLP;
      DU_ALLOC(searchSpc->searchSpaceType->choice.ue_Specific, sizeof(struct SearchSpace__searchSpaceType__ue_Specific));
      if(!searchSpc->searchSpaceType->choice.ue_Specific)
      {
         DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildSearchSpcToAddModList");
         return RFAILED;
      }  
      if(pdcchCfg == NULLP)
         searchSpc->searchSpaceType->choice.ue_Specific->dci_Formats = PDCCH_SRCH_SPC_TWO_UE_SPEC_DCI_FORMAT;
      else
         searchSpc->searchSpaceType->choice.ue_Specific->dci_Formats = pdcchCfg->searchSpcToAddModList[idx].ueSpecificDciFormat;
   }
   return ROK;
}/* End BuildSearchSpcToAddModList */

/*******************************************************************
 *
 * @brief Builds BWP DL dedicated PDCCH config
 *
 * @details
 *
 *    Function : BuildBWPDlDedPdcchCfg
 *
 *    Functionality: Builds BWP DL dedicated PDCCH config
 *
 * @params[in] struct PDCCH_Config *pdcchCfg
 *
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
uint8_t BuildBWPDlDedPdcchCfg(PdcchConfig *pdcchCfgDb, struct PDCCH_Config *pdcchCfg)
{
   pdcchCfg->controlResourceSetToAddModList = NULLP;
   DU_ALLOC(pdcchCfg->controlResourceSetToAddModList, sizeof(struct PDCCH_Config__controlResourceSetToAddModList));
   if(!pdcchCfg->controlResourceSetToAddModList)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildBWPDlDedPdcchCfg");
      return RFAILED;
   }

   if(BuildControlRSetToAddModList(pdcchCfgDb, pdcchCfg->controlResourceSetToAddModList) != ROK)
   {
      DU_LOG("\nERROR  --> F1AP : Failed in BuildControlRSetToAddModList()");
      return RFAILED;
   }

   pdcchCfg->controlResourceSetToReleaseList = NULLP;

   pdcchCfg->searchSpacesToAddModList = NULLP;
   DU_ALLOC(pdcchCfg->searchSpacesToAddModList, sizeof(struct PDCCH_Config__searchSpacesToAddModList));
   if(!pdcchCfg->searchSpacesToAddModList)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildBWPDlDedPdcchCfg");
      return RFAILED;
   }

   if(BuildSearchSpcToAddModList(pdcchCfgDb, pdcchCfg->searchSpacesToAddModList) != ROK)
   {
      DU_LOG("\nERROR  --> F1AP : Failed in BuildSearchSpcToAddModList()");
      return RFAILED;
   }

   pdcchCfg->searchSpacesToReleaseList = NULLP;
   pdcchCfg->downlinkPreemption = NULLP;
   pdcchCfg->tpc_PUSCH = NULLP;
   pdcchCfg->tpc_PUCCH = NULLP;
   pdcchCfg->tpc_SRS = NULLP;

   return ROK;
}

/*******************************************************************
 * @brief Builds DMRS DL PDSCH Mapping type A
 *
 * @details
 *
 *    Function : BuildDMRSDLPdschMapTypeA
 *
 *    Functionality: Builds DMRS DL PDSCH Mapping type A
 *
 * @params[in]
 * struct PDSCH_Config__dmrs_DownlinkForPDSCH_MappingTypeA *dmrsDlCfg
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
uint8_t BuildDMRSDLPdschMapTypeA(PdschConfig *pdschCfg, struct PDSCH_Config__dmrs_DownlinkForPDSCH_MappingTypeA *dmrsDlCfg)
{
   dmrsDlCfg->present = PDSCH_Config__dmrs_DownlinkForPDSCH_MappingTypeA_PR_setup;
   dmrsDlCfg->choice.setup = NULLP;
   DU_ALLOC(dmrsDlCfg->choice.setup, sizeof(struct DMRS_DownlinkConfig));
   if(!dmrsDlCfg->choice.setup)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildDMRSDLPdschMapTypeA");
      return RFAILED;
   }

   dmrsDlCfg->choice.setup->dmrs_Type = NULLP;
   dmrsDlCfg->choice.setup->dmrs_AdditionalPosition = NULLP;
   DU_ALLOC(dmrsDlCfg->choice.setup->dmrs_AdditionalPosition, sizeof(long));
   if(!dmrsDlCfg->choice.setup->dmrs_AdditionalPosition)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildDMRSDLPdschMapTypeA");
      return RFAILED;
   }
   if(pdschCfg == NULLP)
      *(dmrsDlCfg->choice.setup->dmrs_AdditionalPosition) = DMRS_ADDITIONAL_POS;
   else
      *(dmrsDlCfg->choice.setup->dmrs_AdditionalPosition) = pdschCfg->dmrsDlCfgForPdschMapTypeA.addPos;

   dmrsDlCfg->choice.setup->maxLength = NULLP;
   dmrsDlCfg->choice.setup->scramblingID0 = NULLP;
   dmrsDlCfg->choice.setup->scramblingID1 = NULLP;
   dmrsDlCfg->choice.setup->phaseTrackingRS = NULLP;

   return ROK;
}

/*******************************************************************
 *
 * @brief Builds TCI states to add/modify list
 *
 * @details
 *
 *    Function : BuildTCIStatesToAddModList
 *
 *    Functionality:Builds TCI states to add/modify list
 *
 * @params[in] 
 * struct PDSCH_Config__tci_StatesToAddModList *tciStatesList
 *
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
uint8_t BuildTCIStatesToAddModList(struct PDSCH_Config__tci_StatesToAddModList *tciStatesList)
{
   return ROK;
}

/*******************************************************************
 *
 * @brief Builds PDSCH time domain allocation list
 *
 * @details
 *
 *    Function : BuildPdschTimeDomAllocList
 *
 *    Functionality: Builds PDSCH time domain allocation list
 *
 * @params[in] 
 * struct PDSCH_Config__pdsch_TimeDomainAllocationList *timeDomAllocList
 *
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
uint8_t BuildPdschTimeDomAllocList(PdschConfig *pdschCfg, struct PDSCH_Config__pdsch_TimeDomainAllocationList *timeDomAllocList)
{
   uint8_t idx;
   uint8_t elementCnt;
   struct PDSCH_TimeDomainResourceAllocation *timeDomAlloc;

   timeDomAllocList->present = PDSCH_Config__pdsch_TimeDomainAllocationList_PR_setup;

   timeDomAllocList->choice.setup = NULLP;
   DU_ALLOC(timeDomAllocList->choice.setup, sizeof(struct PDSCH_TimeDomainResourceAllocationList));
   if(!timeDomAllocList->choice.setup)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildPdschTimeDomAllocList");
      return RFAILED;
   }

if(pdschCfg == NULLP)
   elementCnt = 2;
else
elementCnt = pdschCfg->numTimeDomRsrcAlloc;
   timeDomAllocList->choice.setup->list.count = elementCnt;
   timeDomAllocList->choice.setup->list.size = elementCnt * sizeof(struct PDSCH_TimeDomainResourceAllocation *);

   timeDomAllocList->choice.setup->list.array = NULLP;
   DU_ALLOC(timeDomAllocList->choice.setup->list.array, timeDomAllocList->choice.setup->list.size);
   if(!timeDomAllocList->choice.setup->list.array)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildPdschTimeDomAllocList");
      return RFAILED;
   }

   for(idx = 0; idx < elementCnt; idx++)
   {
      timeDomAllocList->choice.setup->list.array[idx] = NULLP;
      DU_ALLOC(timeDomAllocList->choice.setup->list.array[idx], \
            sizeof(struct PDSCH_TimeDomainResourceAllocation));
      if(!timeDomAllocList->choice.setup->list.array[idx])
      {
         DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildPdschTimeDomAllocList");
         return RFAILED;
      }
   }

   if(pdschCfg == NULLP)
   {
      idx = 0;
      timeDomAlloc = timeDomAllocList->choice.setup->list.array[idx];
      DU_ALLOC(timeDomAlloc->k0, sizeof(long));
      if(!timeDomAlloc->k0)
      {
         DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildPdschTimeDomAllocList");
         return RFAILED;
      }
      *(timeDomAlloc->k0) = 0;
      timeDomAlloc->mappingType = PDSCH_MAPPING_TYPE_A;
      timeDomAlloc->startSymbolAndLength = \
                                           calcSliv(PDSCH_START_SYMBOL, PDSCH_LENGTH_SYMBOL);

      idx++;
      timeDomAlloc = timeDomAllocList->choice.setup->list.array[idx];
      DU_ALLOC(timeDomAlloc->k0, sizeof(long));
      if(!timeDomAlloc->k0)
      {
         DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildPdschTimeDomAllocList");
         return RFAILED;
      }
      *(timeDomAlloc->k0) = 1;
      timeDomAlloc->mappingType = PDSCH_MAPPING_TYPE_A;
      timeDomAlloc->startSymbolAndLength = calcSliv(PDSCH_START_SYMBOL, PDSCH_LENGTH_SYMBOL);
   }
   else
   {
      for(idx = 0; idx < elementCnt; idx++)
      {
         timeDomAlloc = timeDomAllocList->choice.setup->list.array[idx];
         if(pdschCfg->timeDomRsrcAllociList[idx].k0)
         {
            DU_ALLOC(timeDomAlloc->k0, sizeof(long));
            if(!timeDomAlloc->k0)
            {
               DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildPdschTimeDomAllocList");
               return RFAILED;
            }
            *(timeDomAlloc->k0) = *(pdschCfg->timeDomRsrcAllociList[idx].k0);
         }
         timeDomAlloc->mappingType = pdschCfg->timeDomRsrcAllociList[idx].mappingType;
         timeDomAlloc->startSymbolAndLength = pdschCfg->timeDomRsrcAllociList[idx].startSymbolAndLength;
      }
   }

   return ROK;
}

/*******************************************************************
 *
 * @brief Builds PDSCH PRB Bundling type
 *
 * @details
 *
 *    Function : BuildPdschPrbBundlingType
 *
 *    Functionality: Builds PDSCH PRB Bundling type
 *
 * @params[in] 
 * struct PDSCH_Config__prb_BundlingType *prbBndlType
 *
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
uint8_t BuildPdschPrbBundlingType(PdschConfig *pdschCfg, struct PDSCH_Config__prb_BundlingType *prbBndlType)
{
   if(pdschCfg == NULLP)
      prbBndlType->present = PDSCH_Config__prb_BundlingType_PR_staticBundling;
   else
      prbBndlType->present = pdschCfg->bundlingType;

   prbBndlType->choice.staticBundling = NULLP;
   DU_ALLOC(prbBndlType->choice.staticBundling, \
	 sizeof(struct PDSCH_Config__prb_BundlingType__staticBundling));
   if(!prbBndlType->choice.staticBundling)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildPdschPrbBundlingType");
      return RFAILED;
   }
   prbBndlType->choice.staticBundling->bundleSize = NULLP;

   return ROK;
}

/*******************************************************************
 *
 * @brief Builds BWP DL dedicated PDSCH config 
 *
 * @details
 *
 *    Function : BuildBWPDlDedPdschCfg
 *
 *    Functionality: Builds BWP DL dedicated PDSCH config
 *
 * @params[in] struct PDSCH_Config *pdschCfg
 *
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
uint8_t BuildBWPDlDedPdschCfg(PdschConfig *pdschCfgDb, struct PDSCH_Config *pdschCfg)
{
   pdschCfg->dataScramblingIdentityPDSCH = NULLP;

   pdschCfg->dmrs_DownlinkForPDSCH_MappingTypeA = NULLP;
   DU_ALLOC(pdschCfg->dmrs_DownlinkForPDSCH_MappingTypeA, sizeof(struct PDSCH_Config__dmrs_DownlinkForPDSCH_MappingTypeA));
   if(!pdschCfg->dmrs_DownlinkForPDSCH_MappingTypeA)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildBWPDlDedPdschCfg");
      return RFAILED;
   }

   if(BuildDMRSDLPdschMapTypeA(pdschCfgDb, pdschCfg->dmrs_DownlinkForPDSCH_MappingTypeA) != ROK)
   {
      return RFAILED;
   }

   pdschCfg->dmrs_DownlinkForPDSCH_MappingTypeB = NULLP;
   pdschCfg->tci_StatesToAddModList = NULLP;
   pdschCfg->tci_StatesToReleaseList = NULLP;
   pdschCfg->vrb_ToPRB_Interleaver = NULLP;
#if 0
   DU_ALLOC(pdschCfg->tci_StatesToAddModList, sizeof(struct PDSCH_Config__tci_StatesToAddModList));
   if(!pdschCfg->tci_StatesToAddModList)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildBWPDlDedPdschCfg");
      return RFAILED;
   }
   if(BuildTCIStatesToAddModList(pdschCfg->tci_StatesToAddModList) != ROK)
   {
      return RFAILED;
   }
#endif

if(pdschCfgDb == NULLP)
   pdschCfg->resourceAllocation = RES_ALLOC_TYPE;
else
pdschCfg->resourceAllocation = pdschCfgDb->resourceAllocType;

   pdschCfg->pdsch_TimeDomainAllocationList = NULLP;
   DU_ALLOC(pdschCfg->pdsch_TimeDomainAllocationList, sizeof(struct PDSCH_Config__pdsch_TimeDomainAllocationList));
   if(!pdschCfg->pdsch_TimeDomainAllocationList)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildBWPDlDedPdschCfg");
      return RFAILED;
   }
   if(BuildPdschTimeDomAllocList(pdschCfgDb, pdschCfg->pdsch_TimeDomainAllocationList) != ROK)
   {
      return RFAILED;
   }

   pdschCfg->pdsch_AggregationFactor = NULLP;
   pdschCfg->rateMatchPatternToAddModList = NULLP;
   pdschCfg->rateMatchPatternToReleaseList = NULLP;
   pdschCfg->rateMatchPatternGroup1 = NULLP;
   pdschCfg->rateMatchPatternGroup2 = NULLP;
   if(pdschCfgDb == NULLP)
      pdschCfg->rbg_Size = PDSCH_RBG_SIZE;
   else
      pdschCfg->rbg_Size = pdschCfgDb->rbgSize;
   pdschCfg->mcs_Table = NULLP;

   pdschCfg->maxNrofCodeWordsScheduledByDCI = NULLP;
   DU_ALLOC(pdschCfg->maxNrofCodeWordsScheduledByDCI, sizeof(long));
   if(!pdschCfg->maxNrofCodeWordsScheduledByDCI)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildBWPDlDedPdschCfg");
      return RFAILED;
   }
   if(pdschCfgDb == NULLP)
      *(pdschCfg->maxNrofCodeWordsScheduledByDCI) = PDSCH_MAX_CODEWORD_SCH_BY_DCI;
   else
      *(pdschCfg->maxNrofCodeWordsScheduledByDCI) = pdschCfgDb->numCodeWordsSchByDci;

   if(BuildPdschPrbBundlingType(pdschCfgDb, &pdschCfg->prb_BundlingType) != ROK)
   {
      return RFAILED;
   }

   pdschCfg->zp_CSI_RS_ResourceToAddModList = NULLP;
   pdschCfg->zp_CSI_RS_ResourceToReleaseList = NULLP;
   pdschCfg->aperiodic_ZP_CSI_RS_ResourceSetsToAddModList = NULLP;
   pdschCfg->aperiodic_ZP_CSI_RS_ResourceSetsToReleaseList = NULLP;
   pdschCfg->sp_ZP_CSI_RS_ResourceSetsToAddModList = NULLP;
   pdschCfg->sp_ZP_CSI_RS_ResourceSetsToReleaseList = NULLP;
   pdschCfg->p_ZP_CSI_RS_ResourceSet = NULLP;

   return ROK;
}

/*******************************************************************
 *
 * @brief Builds intitial DL BWP
 * @details
 *
 *    Function : BuildInitialDlBWP 
 *
 *    Functionality: Builds intitial DL BWP in spCellCfgDed
 *
 * @params[in] BWP_DownlinkDedicated_t *dlBwp
 *
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
uint8_t BuildInitialDlBWP(InitialDlBwp *initiDlBwp, BWP_DownlinkDedicated_t *dlBwp)
{
   PdcchConfig *pdcchCfg = NULLP;
   PdschConfig *pdschCfg = NULLP;

   if(initiDlBwp)
   {
      if(initiDlBwp->pdcchPresent)
         pdcchCfg = &initiDlBwp->pdcchCfg;
      if(initiDlBwp->pdschPresent)
         pdschCfg = &initiDlBwp->pdschCfg;
   }

   dlBwp->pdcch_Config = NULLP;
   DU_ALLOC(dlBwp->pdcch_Config, sizeof(struct BWP_DownlinkDedicated__pdcch_Config));
   if(!dlBwp->pdcch_Config)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory Allocation failure in BuildInitialDlBWP");
      return RFAILED;
   }
   dlBwp->pdcch_Config->present = BWP_DownlinkDedicated__pdcch_Config_PR_setup; 

   dlBwp->pdcch_Config->choice.setup = NULLP;
   DU_ALLOC(dlBwp->pdcch_Config->choice.setup, sizeof(struct PDCCH_Config));
   if(!dlBwp->pdcch_Config->choice.setup)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory Allocation failure in BuildInitialDlBWP");
      return RFAILED;
   }
   if(BuildBWPDlDedPdcchCfg(pdcchCfg, dlBwp->pdcch_Config->choice.setup) != ROK)
   {
      return RFAILED;
   }

   dlBwp->pdsch_Config = NULLP;
   DU_ALLOC(dlBwp->pdsch_Config, sizeof(struct BWP_DownlinkDedicated__pdsch_Config));
   if(!dlBwp->pdsch_Config)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory Allocation failure in BuildInitialDlBWP");
      return RFAILED;
   }
   dlBwp->pdsch_Config->present = BWP_DownlinkDedicated__pdsch_Config_PR_setup;

   dlBwp->pdsch_Config->choice.setup = NULLP;
   DU_ALLOC(dlBwp->pdsch_Config->choice.setup, sizeof(struct PDSCH_Config));
   if(!dlBwp->pdsch_Config->choice.setup)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory Allocation failure in BuildInitialDlBWP");
      return RFAILED;
   }

   if(BuildBWPDlDedPdschCfg(pdschCfg, dlBwp->pdsch_Config->choice.setup) != ROK)
   {
      return RFAILED;
   }

   dlBwp->sps_Config = NULLP;
   dlBwp->radioLinkMonitoringConfig = NULLP; 
   return ROK;
}

/*******************************************************************
 *
 * @brief Builds DMRS UL Pusch Mapping type A
 *
 * @details
 *
 *    Function : BuildDMRSULPuschMapTypeA
 *
 *    Functionality: Builds DMRS UL Pusch Mapping type A
 *
 * @params[in] 
 *    struct PUSCH_Config__dmrs_UplinkForPUSCH_MappingTypeA *dmrsUlCfg
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
uint8_t BuildDMRSULPuschMapTypeA(DmrsUlCfg *ulDmrsCfgDb, struct PUSCH_Config__dmrs_UplinkForPUSCH_MappingTypeA *dmrsUlCfg)
{
   dmrsUlCfg->present = PUSCH_Config__dmrs_UplinkForPUSCH_MappingTypeA_PR_setup;
   dmrsUlCfg->choice.setup= NULLP;
   DU_ALLOC(dmrsUlCfg->choice.setup, sizeof(DMRS_UplinkConfig_t));
   if(!dmrsUlCfg->choice.setup)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildDMRSULPuschMapTypeA");
      return RFAILED;
   }

   dmrsUlCfg->choice.setup->dmrs_Type = NULLP;
   dmrsUlCfg->choice.setup->dmrs_AdditionalPosition = NULLP;
   DU_ALLOC(dmrsUlCfg->choice.setup->dmrs_AdditionalPosition, sizeof(long));
   if(!dmrsUlCfg->choice.setup->dmrs_AdditionalPosition)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildDMRSULPuschMapTypeA");
      return RFAILED;
   }
   if(ulDmrsCfgDb == NULLP)
      *(dmrsUlCfg->choice.setup->dmrs_AdditionalPosition) = DMRS_ADDITIONAL_POS; 
   else
      *(dmrsUlCfg->choice.setup->dmrs_AdditionalPosition) = ulDmrsCfgDb->addPos;

   dmrsUlCfg->choice.setup->phaseTrackingRS = NULLP;
   dmrsUlCfg->choice.setup->maxLength = NULLP;
   dmrsUlCfg->choice.setup->transformPrecodingDisabled = NULLP;
   DU_ALLOC(dmrsUlCfg->choice.setup->transformPrecodingDisabled, sizeof(struct DMRS_UplinkConfig__transformPrecodingDisabled));
   if(!dmrsUlCfg->choice.setup->transformPrecodingDisabled)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildDMRSULPuschMapTypeA");
      return RFAILED;
   }

   dmrsUlCfg->choice.setup->transformPrecodingDisabled->scramblingID0 = NULLP;
   DU_ALLOC(dmrsUlCfg->choice.setup->transformPrecodingDisabled->scramblingID0,\
	 sizeof(long));
   if(!dmrsUlCfg->choice.setup->transformPrecodingDisabled->scramblingID0)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildDMRSULPuschMapTypeA");
      return RFAILED;
   }
   if(ulDmrsCfgDb == NULLP)
      *(dmrsUlCfg->choice.setup->transformPrecodingDisabled->scramblingID0) = SCRAMBLING_ID;
   else
      *(dmrsUlCfg->choice.setup->transformPrecodingDisabled->scramblingID0) = ulDmrsCfgDb->transPrecodDisabled.scramblingId0;

   dmrsUlCfg->choice.setup->transformPrecodingDisabled->scramblingID1 = NULLP;
   dmrsUlCfg->choice.setup->transformPrecodingEnabled = NULLP;
   return ROK;
}

/*******************************************************************
 *
 * @brief Build PUSCH time domain allocation list
 *
 * @details
 *
 *    Function : BuildPuschTimeDomAllocList
 *
 *    Functionality: Build PUSCH time domain allocation list
 *
 * @params[in] 
 * struct PUSCH_Config__pusch_TimeDomainAllocationList *timeDomAllocList
 *
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
uint8_t BuildPuschTimeDomAllocList(PuschCfg *puschCfgDb, struct PUSCH_Config__pusch_TimeDomainAllocationList *timeDomAllocList)
{
   uint8_t idx;
   uint8_t elementCnt;
   PUSCH_TimeDomainResourceAllocation_t  *timeDomAlloc;

   timeDomAllocList->present = PUSCH_Config__pusch_TimeDomainAllocationList_PR_setup;
   timeDomAllocList->choice.setup = NULLP;
   DU_ALLOC(timeDomAllocList->choice.setup, sizeof(struct PUSCH_TimeDomainResourceAllocationList));
   if(!timeDomAllocList->choice.setup)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildPuschTimeDomAllocList");
      return RFAILED;
   }

   if(puschCfgDb == NULLP)
      elementCnt = 2;
   else
      elementCnt = puschCfgDb->numTimeDomRsrcAlloc;

   timeDomAllocList->choice.setup->list.count = elementCnt;
   timeDomAllocList->choice.setup->list.size = elementCnt * sizeof(PUSCH_TimeDomainResourceAllocation_t *);
   timeDomAllocList->choice.setup->list.array = NULLP;
   DU_ALLOC(timeDomAllocList->choice.setup->list.array, timeDomAllocList->choice.setup->list.size);
   if(!timeDomAllocList->choice.setup->list.array)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildPuschTimeDomAllocList");
      return RFAILED;
   }

   for(idx = 0; idx < elementCnt; idx++)
   {
      timeDomAllocList->choice.setup->list.array[idx] = NULLP;
      DU_ALLOC(timeDomAllocList->choice.setup->list.array[idx], sizeof(PUSCH_TimeDomainResourceAllocation_t));
      if(!timeDomAllocList->choice.setup->list.array[idx])
      {
         DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildPuschTimeDomAllocList");
         return RFAILED;
      }
   }

   for(idx = 0; idx < elementCnt; idx++)
   {
      timeDomAlloc = timeDomAllocList->choice.setup->list.array[idx];
      DU_ALLOC(timeDomAlloc->k2, sizeof(long));
      if(!timeDomAlloc->k2)
      {
         DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildPuschTimeDomAllocList");
         return RFAILED;
      }
      if(puschCfgDb == NULLP)
      {
         if(idx == 0)
            *(timeDomAlloc->k2) = PUSCH_K2_CFG1;
         else if(idx == 1)
            *(timeDomAlloc->k2) = PUSCH_K2_CFG2;

         timeDomAlloc->mappingType = PUSCH_MAPPING_TYPE_A;
         timeDomAlloc->startSymbolAndLength = calcSliv(PUSCH_START_SYMBOL, PUSCH_LENGTH_SYMBOL);
      }
      else
      {
         *(timeDomAlloc->k2) = puschCfgDb->timeDomRsrcAllocList[idx].k2;
         timeDomAlloc->mappingType = puschCfgDb->timeDomRsrcAllocList[idx].mappingType;
         timeDomAlloc->startSymbolAndLength = puschCfgDb->timeDomRsrcAllocList[idx].startSymbolAndLength;
      }
   }

   return ROK;
}

/*******************************************************************
 *
 * @brief Builds BWP UL dedicated PUSCH Config
 *
 * @details
 *
 *    Function : BuildBWPUlDedPuschCfg
 *
 *    Functionality:
 *      Builds BWP UL dedicated PUSCH Config
 *
 * @params[in] : PUSCH_Config_t *puschCfg
 *    
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
uint8_t BuildBWPUlDedPuschCfg(PuschCfg *puschCfgDb, PUSCH_Config_t *puschCfg)
{
   DmrsUlCfg *ulDmrsCfg = NULLP;
   
   if(puschCfgDb)
      ulDmrsCfg = &puschCfgDb->dmrsUlCfgForPuschMapTypeA;

   puschCfg->dataScramblingIdentityPUSCH = NULLP;
   DU_ALLOC(puschCfg->dataScramblingIdentityPUSCH, sizeof(long));
   if(!puschCfg->dataScramblingIdentityPUSCH)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildBWPUlDedPuschCfg");
      return RFAILED;
   }
   if(puschCfgDb == NULLP)
      *(puschCfg->dataScramblingIdentityPUSCH) = SCRAMBLING_ID;
   else
      *(puschCfg->dataScramblingIdentityPUSCH) = puschCfgDb->dataScramblingId;

   puschCfg->txConfig = NULLP;
   puschCfg->dmrs_UplinkForPUSCH_MappingTypeA = NULLP;
   DU_ALLOC(puschCfg->dmrs_UplinkForPUSCH_MappingTypeA, sizeof(struct PUSCH_Config__dmrs_UplinkForPUSCH_MappingTypeA));
   if(!puschCfg->dmrs_UplinkForPUSCH_MappingTypeA)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildBWPUlDedPuschCfg");
      return RFAILED;
   }

   if(BuildDMRSULPuschMapTypeA(ulDmrsCfg, puschCfg->dmrs_UplinkForPUSCH_MappingTypeA) != ROK)
   {
      return RFAILED;
   }

   puschCfg->dmrs_UplinkForPUSCH_MappingTypeB = NULLP;
   puschCfg->pusch_PowerControl = NULLP;
   puschCfg->frequencyHopping = NULLP;
   puschCfg->frequencyHoppingOffsetLists = NULLP;

   if(puschCfgDb == NULLP)
      puschCfg->resourceAllocation = RES_ALLOC_TYPE;
   else
      puschCfg->resourceAllocation = puschCfgDb->resourceAllocType;

   puschCfg->pusch_TimeDomainAllocationList = NULLP;
   DU_ALLOC(puschCfg->pusch_TimeDomainAllocationList, sizeof(struct PUSCH_Config__pusch_TimeDomainAllocationList));
   if(!puschCfg->pusch_TimeDomainAllocationList)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildBWPUlDedPuschCfg");
      return RFAILED;
   }

   if(BuildPuschTimeDomAllocList(puschCfgDb, puschCfg->pusch_TimeDomainAllocationList) != ROK)
   {
      return RFAILED;
   }

   puschCfg->pusch_AggregationFactor = NULLP;
   puschCfg->mcs_Table = NULLP;
   puschCfg->mcs_TableTransformPrecoder = NULLP;
   puschCfg->transformPrecoder = NULLP;
   DU_ALLOC(puschCfg->transformPrecoder, sizeof(long));
   if(!puschCfg->transformPrecoder)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildBWPUlDedPuschCfg");
      return RFAILED;
   }
   if(puschCfgDb == NULLP)
      *(puschCfg->transformPrecoder) = PUSCH_TRANSFORM_PRECODER;
   else
      *(puschCfg->transformPrecoder) = puschCfgDb->transformPrecoder;

   puschCfg->codebookSubset = NULLP;
   puschCfg->maxRank = NULLP;
   puschCfg->rbg_Size = NULLP;
   puschCfg->uci_OnPUSCH = NULLP;
   puschCfg->tp_pi2BPSK = NULLP;

   return ROK;
}

/*******************************************************************
 *
 * @brief Builds PUCCH resource set add/modify list
 *
 * @details
 *
 *    Function : BuildPucchRsrcSetAddModList
 *
 *    Functionality:
 *      Builds PUCCH resource set add/modify list
 *
 * @params[in] : PucchResrcSetCfg *rsrcSetCfgDb
 *               struct PUCCH_Config__resourceSetToAddModList *rsrcSetAddModList
 *
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
uint8_t BuildPucchRsrcSetAddModList(PucchResrcSetCfg *rsrcSetCfgDb, \
   struct PUCCH_Config__resourceSetToAddModList *resourceSetToAddModList)
{
   uint8_t elementCnt = 0, rsrcSetIdx = 0, rsrcIdx = 0;
   PUCCH_ResourceSet_t *rsrcSet = NULLP;

   if(rsrcSetCfgDb == NULLP)
      elementCnt = 1;
   else
      elementCnt = rsrcSetCfgDb->resrcSetToAddModListCount;

   resourceSetToAddModList->list.count = elementCnt;
   resourceSetToAddModList->list.size = elementCnt * sizeof(PUCCH_ResourceSet_t *);
   DU_ALLOC(resourceSetToAddModList->list.array, resourceSetToAddModList->list.size);
   if(resourceSetToAddModList->list.array == NULLP)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildPucchRsrcSetAddModList");
      return RFAILED;
   }
   for(rsrcSetIdx=0; rsrcSetIdx < resourceSetToAddModList->list.count; rsrcSetIdx++)
   {
      DU_ALLOC(resourceSetToAddModList->list.array[rsrcSetIdx], sizeof(PUCCH_ResourceSet_t));
      if(resourceSetToAddModList->list.array[rsrcSetIdx] == NULLP)
      {
         DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildPucchRsrcSetAddModList");
         return RFAILED;
      }
   }

   for(rsrcSetIdx=0; rsrcSetIdx < resourceSetToAddModList->list.count; rsrcSetIdx++)
   {
      rsrcSet = resourceSetToAddModList->list.array[rsrcSetIdx];

      /* Resource set Id */
      if(rsrcSetCfgDb == NULLP)
         rsrcSet->pucch_ResourceSetId = 1;
      else
         rsrcSet->pucch_ResourceSetId = rsrcSetCfgDb->resrcSetToAddModList[rsrcSetIdx].resrcSetId;
 
      /* Resource list of a resource set */
      if(rsrcSetCfgDb == NULLP)
         elementCnt = 1;
      else
         elementCnt = rsrcSetCfgDb->resrcSetToAddModList[rsrcSetIdx].resrcListCount;
      rsrcSet->resourceList.list.count = elementCnt;
      rsrcSet->resourceList.list.size = elementCnt * sizeof(PUCCH_ResourceId_t *);
      DU_ALLOC(rsrcSet->resourceList.list.array, rsrcSet->resourceList.list.size);
      if(rsrcSet->resourceList.list.array == NULLP)
      {
         DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildPucchRsrcSetAddModList");
         return RFAILED;
      }

      for(rsrcIdx=0; rsrcIdx < rsrcSet->resourceList.list.count; rsrcIdx++)
      {
         DU_ALLOC(rsrcSet->resourceList.list.array[rsrcIdx], sizeof(PUCCH_ResourceId_t));
         if(rsrcSet->resourceList.list.array[rsrcIdx] == NULLP)
         {
            DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildPucchRsrcSetAddModList");
            return RFAILED;
         }
      }
      for(rsrcIdx=0; rsrcIdx < rsrcSet->resourceList.list.count; rsrcIdx++)
      {
         if(rsrcSetCfgDb == NULLP)
            *(rsrcSet->resourceList.list.array[rsrcIdx]) = 1;
         else
            *(rsrcSet->resourceList.list.array[rsrcIdx]) = rsrcSetCfgDb->resrcSetToAddModList[rsrcSetIdx].resrcList[rsrcIdx];
      }

      /* Max payload size (minus 1) in a Resource set */
      rsrcSet->maxPayloadMinus1 = NULLP;
      if(rsrcSetCfgDb && rsrcSetCfgDb->resrcSetToAddModList[rsrcSetIdx].maxPayLoadSize)
      {
         DU_ALLOC(rsrcSet->maxPayloadMinus1, sizeof(long));
         if(rsrcSet->maxPayloadMinus1 == NULLP)
         {
            DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildPucchRsrcSetAddModList");
            return RFAILED;
         }
         *(rsrcSet->maxPayloadMinus1) = rsrcSetCfgDb->resrcSetToAddModList[rsrcSetIdx].maxPayLoadSize;
      }
   }
   return ROK;
}

/*******************************************************************
 *
 * @brief Builds PUCCH resource add/modify list
 *
 * @details
 *
 *    Function : BuildPucchRsrcAdddModList
 *
 *    Functionality:
 *      Builds PUCCH resource add/modify list
 *
 * @params[in] : PucchResrcCfg *rsrcSetCfgDb
 *               struct PUCCH_Config__resourceSetToAddModList *rsrcSetAddModList
 *
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
uint8_t BuildPucchRsrcAddModList(PucchResrcCfg *rsrcCfgDb, struct PUCCH_Config__resourceToAddModList  *resourceToAddModList)
{
   uint8_t elementCnt = 0, rsrcIdx = 0;
   PUCCH_Resource_t *rsrc = NULLP;

   if(rsrcCfgDb == NULLP)
      elementCnt = 1;
   else
      elementCnt = rsrcCfgDb->resrcToAddModListCount;
   resourceToAddModList->list.count = elementCnt;
   resourceToAddModList->list.size = elementCnt * sizeof(PUCCH_Resource_t *);
   DU_ALLOC(resourceToAddModList->list.array, resourceToAddModList->list.size);
   if(resourceToAddModList->list.array == NULLP)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildPucchRsrcAddModList");
      return RFAILED;
   }
   for(rsrcIdx=0; rsrcIdx < resourceToAddModList->list.count; rsrcIdx++)
   {
      DU_ALLOC(resourceToAddModList->list.array[rsrcIdx], sizeof(PUCCH_Resource_t));
      if(resourceToAddModList->list.array[rsrcIdx] == NULLP)
      {
         DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildPucchRsrcAddModList");
         return RFAILED;
      }
   }

   for(rsrcIdx=0; rsrcIdx < resourceToAddModList->list.count; rsrcIdx++)
   {
      rsrc = resourceToAddModList->list.array[rsrcIdx];

      if(rsrcCfgDb == NULLP)
      {
         rsrc->pucch_ResourceId = 1;
         rsrc->startingPRB = 0;
         rsrc->format.present = PUCCH_Resource__format_PR_format1; 
         DU_ALLOC(rsrc->format.choice.format1, sizeof(PUCCH_format1_t));
         if(rsrc->format.choice.format1 == NULLP)
         {
            DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildPucchRsrcAddModList");
            return RFAILED;
         }  
         rsrc->format.choice.format1->initialCyclicShift = 0;
         rsrc->format.choice.format1->nrofSymbols = 4;
         rsrc->format.choice.format1->startingSymbolIndex = 0;
         rsrc->format.choice.format1->timeDomainOCC = 0;
      }
      else
      {
         rsrc->pucch_ResourceId = rsrcCfgDb->resrcToAddModList[rsrcIdx].resrcId;
         rsrc->startingPRB = rsrcCfgDb->resrcToAddModList[rsrcIdx].startPrb;
         if(rsrcCfgDb->resrcToAddModList[rsrcIdx].intraFreqHop)
         {
            DU_ALLOC(rsrc->intraSlotFrequencyHopping, sizeof(long));
            if(rsrc->intraSlotFrequencyHopping == NULLP)
            {
               DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildPucchRsrcAddModList");
               return RFAILED;
            }
            *(rsrc->intraSlotFrequencyHopping) = rsrcCfgDb->resrcToAddModList[rsrcIdx].intraFreqHop;
         }
         else
            rsrc->intraSlotFrequencyHopping = NULLP;

         if(rsrcCfgDb->resrcToAddModList[rsrcIdx].secondPrbHop)
         {
            DU_ALLOC(rsrc->secondHopPRB, sizeof(PRB_Id_t));
            if(rsrc->secondHopPRB == NULLP)
            {
               DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildPucchRsrcAddModList");
               return RFAILED;
            }
            *(rsrc->secondHopPRB) = rsrcCfgDb->resrcToAddModList[rsrcIdx].secondPrbHop;
         }
         else
            rsrc->secondHopPRB = NULLP;
         rsrc->format.present = covertPucchFormatIntEnumToRrcEnum(rsrcCfgDb->resrcToAddModList[rsrcIdx].pucchFormat); 

         switch(rsrc->format.present)
         {
            case PUCCH_Resource__format_PR_NOTHING:
               break;
            case PUCCH_Resource__format_PR_format0:
               {
                  DU_ALLOC(rsrc->format.choice.format0, sizeof(PUCCH_format0_t));
                  if(rsrc->format.choice.format0 == NULLP)
                  {
                     DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildPucchRsrcAddModList");
                     return RFAILED;
                  }
                  rsrc->format.choice.format0->initialCyclicShift = rsrcCfgDb->resrcToAddModList[rsrcIdx].PucchFormat.format0->initialCyclicShift;
                  rsrc->format.choice.format0->nrofSymbols = rsrcCfgDb->resrcToAddModList[rsrcIdx].PucchFormat.format0->numSymbols;
                  rsrc->format.choice.format0->startingSymbolIndex = rsrcCfgDb->resrcToAddModList[rsrcIdx].PucchFormat.format0->startSymbolIdx;
                  break;
               }

            case PUCCH_Resource__format_PR_format1:
               {
                  DU_ALLOC(rsrc->format.choice.format1, sizeof(PUCCH_format1_t));
                  if(rsrc->format.choice.format1 == NULLP)
                  {
                     DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildPucchRsrcAddModList");
                     return RFAILED;
                  }  
                  rsrc->format.choice.format1->initialCyclicShift = rsrcCfgDb->resrcToAddModList[rsrcIdx].PucchFormat.format1->initialCyclicShift;
                  rsrc->format.choice.format1->nrofSymbols = rsrcCfgDb->resrcToAddModList[rsrcIdx].PucchFormat.format1->numSymbols;
                  rsrc->format.choice.format1->startingSymbolIndex = rsrcCfgDb->resrcToAddModList[rsrcIdx].PucchFormat.format1->startSymbolIdx;
                  rsrc->format.choice.format1->timeDomainOCC = rsrcCfgDb->resrcToAddModList[rsrcIdx].PucchFormat.format1->timeDomOCC;
                  break;
               }

            case PUCCH_Resource__format_PR_format2:
               {
                  DU_ALLOC(rsrc->format.choice.format2, sizeof(PUCCH_format2_t));
                  if(rsrc->format.choice.format2 == NULLP)
                  {
                     DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildPucchRsrcAddModList");
                     return RFAILED;
                  } 
                  rsrc->format.choice.format2->nrofPRBs = rsrcCfgDb->resrcToAddModList[rsrcIdx].PucchFormat.format2->numPrbs;
                  rsrc->format.choice.format2->nrofSymbols = rsrcCfgDb->resrcToAddModList[rsrcIdx].PucchFormat.format2->numSymbols;
                  rsrc->format.choice.format2->startingSymbolIndex = rsrcCfgDb->resrcToAddModList[rsrcIdx].PucchFormat.format2->startSymbolIdx;
                  break;
               }

            case PUCCH_Resource__format_PR_format3:
               {
                  DU_ALLOC(rsrc->format.choice.format3, sizeof(PUCCH_format3_t));
                  if(rsrc->format.choice.format3 == NULLP)
                  {
                     DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildPucchRsrcAddModList");
                     return RFAILED;
                  }
                  rsrc->format.choice.format3->nrofPRBs = rsrcCfgDb->resrcToAddModList[rsrcIdx].PucchFormat.format3->numPrbs;
                  rsrc->format.choice.format3->nrofSymbols = rsrcCfgDb->resrcToAddModList[rsrcIdx].PucchFormat.format3->numSymbols;
                  rsrc->format.choice.format3->startingSymbolIndex = rsrcCfgDb->resrcToAddModList[rsrcIdx].PucchFormat.format3->startSymbolIdx;
                  break;
               }

            case PUCCH_Resource__format_PR_format4:
               {
                  DU_ALLOC(rsrc->format.choice.format4, sizeof(PUCCH_format4_t));
                  if(rsrc->format.choice.format4 == NULLP)
                  {
                     DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildPucchRsrcAddModList");
                     return RFAILED;
                  }
                  rsrc->format.choice.format4->nrofSymbols = rsrcCfgDb->resrcToAddModList[rsrcIdx].PucchFormat.format4->numSymbols;
                  rsrc->format.choice.format4->occ_Length = rsrcCfgDb->resrcToAddModList[rsrcIdx].PucchFormat.format4->occLen;
                  rsrc->format.choice.format4->occ_Index = rsrcCfgDb->resrcToAddModList[rsrcIdx].PucchFormat.format4->occIdx;
                  rsrc->format.choice.format4->startingSymbolIndex = rsrcCfgDb->resrcToAddModList[rsrcIdx].PucchFormat.format4->startSymbolIdx;
                  break;
               }
         }
      }
   }
   return ROK;
}

/*******************************************************************
 *
 * @brief Builds PUCCH format  config
 *
 * @details
 *
 *    Function : BuildPucchFormat
 *
 *    Functionality: Builds PUCCH format  config
 *
 * @params[in] : PucchFormatCfg *formatDb
 *               PUCCH_FormatConfig_t *format
 *
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
uint8_t BuildPucchFormat(PucchFormatCfg *formatDb, PUCCH_FormatConfig_t *format)
{
   /* Inter Slot Fequency hopping */
   format->interslotFrequencyHopping = NULLP;
   if((formatDb != NULLP) && (formatDb->interSlotFreqHop == true))
   {
      DU_ALLOC(format->interslotFrequencyHopping, sizeof(long));
      if(format->interslotFrequencyHopping)
      {
         DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildPucchFormat");
         return RFAILED;
      }
      *(format->interslotFrequencyHopping) = PUCCH_FormatConfig__interslotFrequencyHopping_enabled;
   }

   /* Additional DMRS */
   format->additionalDMRS = NULLP;
   if((formatDb != NULLP) && (formatDb->addDmrs == true))
   {
      DU_ALLOC(format->additionalDMRS, sizeof(long));
      if(format->additionalDMRS)
      {
         DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildPucchFormat");
         return RFAILED;
      }
      *(format->additionalDMRS) = PUCCH_FormatConfig__additionalDMRS_true;
   }

    /* Maximum code rate */
   format->maxCodeRate = NULLP;
   if((formatDb != NULLP) && (formatDb->maxCodeRate != 0))
   {
      DU_ALLOC(format->maxCodeRate, sizeof(long));
      if(format->maxCodeRate)
      {
         DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildPucchFormat");
         return RFAILED;
      }  
      *(format->maxCodeRate) = formatDb->maxCodeRate;
   }
 
   /* Number of slots */
   format->nrofSlots = NULLP;
   if((formatDb == NULLP) || ((formatDb != NULLP) && (formatDb->numSlots != 0)))
   {
      DU_ALLOC(format->nrofSlots, sizeof(long));
      if(format->nrofSlots == NULLP)
      {
         DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildPucchFormat");
         return RFAILED;
      }
      if(formatDb == NULLP)
         *(format->nrofSlots) = PUCCH_FormatConfig__nrofSlots_n4;
      else
         *(format->nrofSlots) = formatDb->numSlots;
   }

   /* Pi2BPSK*/
   format->pi2BPSK = NULLP;
   if((formatDb != NULLP) && (formatDb->pi2BPSK == true))
   {
      DU_ALLOC(format->pi2BPSK, sizeof(long));
      if(format->pi2BPSK)
      {     
         DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildPucchFormat");
         return RFAILED;
      }     
      *(format->pi2BPSK) = PUCCH_FormatConfig__pi2BPSK_enabled;
   }

   /* Simultaneous HARQ ACK and CSI */
   format->simultaneousHARQ_ACK_CSI = NULLP;
   if((formatDb != NULLP) && (formatDb->harqAckCSI == true))
   {
      DU_ALLOC(format->simultaneousHARQ_ACK_CSI, sizeof(long));
      if(format->simultaneousHARQ_ACK_CSI)
      {     
         DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildPucchFormat");
         return RFAILED;
      }     
      *(format->simultaneousHARQ_ACK_CSI) = PUCCH_FormatConfig__simultaneousHARQ_ACK_CSI_true;
   }

   return ROK;
}


/*******************************************************************
 *
 * @brief Builds PUCCH scheduling request list
 *
 * @details
 *
 *    Function : BuildPucchSchReqAddModList
 *
 *    Functionality:
 *      Builds PUCCH scheduling request list
 *
 * @params[in] : PucchSchedReqCfg *schReqDb
 *               struct PUCCH_Config__schedulingRequestResourceToAddModList *schReqRsrcToAddModList
 *
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
uint8_t BuildPucchSchReqAddModList(PucchSchedReqCfg *schReqDb, \
   struct PUCCH_Config__schedulingRequestResourceToAddModList *schReqRsrcToAddModList)
{
   uint8_t elementCnt = 0, schReqIdx = 0;
   SchedulingRequestResourceConfig_t *schReqRsrc;

   elementCnt = schReqDb->schedAddModListCount;
   schReqRsrcToAddModList->list.count = elementCnt;
   schReqRsrcToAddModList->list.size = elementCnt *sizeof(struct SchedulingRequestResourceConfig *);

   schReqRsrcToAddModList->list.array = NULLP;
   DU_ALLOC(schReqRsrcToAddModList->list.array, schReqRsrcToAddModList->list.size);
   if(schReqRsrcToAddModList->list.array == NULLP)
   {
      DU_LOG("\nERROR  --> DU APP: Memory allocation failed in BuildPucchSchReqAddModList");
      return RFAILED;
   }

   for(schReqIdx = 0; schReqIdx < schReqRsrcToAddModList->list.count; schReqIdx++)
   {
      DU_ALLOC(schReqRsrcToAddModList->list.array[schReqIdx], schReqRsrcToAddModList->list.size);
      if(schReqRsrcToAddModList->list.array[schReqIdx] == NULLP)
      {
         DU_LOG("\nERROR  --> DU APP: Memory allocation failed in BuildPucchSchReqAddModList");
         return RFAILED;
      }
   }

   for(schReqIdx = 0; schReqIdx < schReqRsrcToAddModList->list.count; schReqIdx++)
   {
      schReqRsrc = schReqRsrcToAddModList->list.array[schReqIdx];
      schReqRsrc->schedulingRequestResourceId = schReqDb->schedAddModList[schReqIdx].resrcId;
      schReqRsrc->schedulingRequestID = schReqDb->schedAddModList[schReqIdx].requestId;

      if(schReqDb->schedAddModList[schReqIdx].periodicity)
      {
         schReqRsrc->periodicityAndOffset = NULLP;
         DU_ALLOC(schReqRsrc->periodicityAndOffset, sizeof(struct SchedulingRequestResourceConfig__periodicityAndOffset));
         if(schReqRsrc->periodicityAndOffset == NULLP)
         {
            DU_LOG("\nERROR  --> DU APP: Memory allocation failed in BuildPucchSchReqAddModList");
            return RFAILED;
         }

         schReqRsrc->periodicityAndOffset->present = schReqDb->schedAddModList[schReqIdx].periodicity;
         switch(schReqRsrc->periodicityAndOffset->present)
         {
            case SchedulingRequestResourceConfig__periodicityAndOffset_PR_NOTHING:
               break;
            case SchedulingRequestResourceConfig__periodicityAndOffset_PR_sym2:
               schReqRsrc->periodicityAndOffset->choice.sym2 = schReqDb->schedAddModList[schReqIdx].offset;
               break;
            case SchedulingRequestResourceConfig__periodicityAndOffset_PR_sym6or7:
               schReqRsrc->periodicityAndOffset->choice.sym6or7 = schReqDb->schedAddModList[schReqIdx].offset;
               break;
            case SchedulingRequestResourceConfig__periodicityAndOffset_PR_sl1:
               schReqRsrc->periodicityAndOffset->choice.sl1 = schReqDb->schedAddModList[schReqIdx].offset;
               break;
            case SchedulingRequestResourceConfig__periodicityAndOffset_PR_sl2:
               schReqRsrc->periodicityAndOffset->choice.sl2 = schReqDb->schedAddModList[schReqIdx].offset;
               break;
            case SchedulingRequestResourceConfig__periodicityAndOffset_PR_sl4:
               schReqRsrc->periodicityAndOffset->choice.sl4 = schReqDb->schedAddModList[schReqIdx].offset;
               break;
            case SchedulingRequestResourceConfig__periodicityAndOffset_PR_sl5:
               schReqRsrc->periodicityAndOffset->choice.sl5 = schReqDb->schedAddModList[schReqIdx].offset;
               break;
            case SchedulingRequestResourceConfig__periodicityAndOffset_PR_sl8:
               schReqRsrc->periodicityAndOffset->choice.sl8 = schReqDb->schedAddModList[schReqIdx].offset;
               break;
            case SchedulingRequestResourceConfig__periodicityAndOffset_PR_sl10:
               schReqRsrc->periodicityAndOffset->choice.sl10 = schReqDb->schedAddModList[schReqIdx].offset;
               break;
            case SchedulingRequestResourceConfig__periodicityAndOffset_PR_sl16:
               schReqRsrc->periodicityAndOffset->choice.sl16 = schReqDb->schedAddModList[schReqIdx].offset;
               break;
            case SchedulingRequestResourceConfig__periodicityAndOffset_PR_sl20:
               schReqRsrc->periodicityAndOffset->choice.sl20 = schReqDb->schedAddModList[schReqIdx].offset;
               break;
            case SchedulingRequestResourceConfig__periodicityAndOffset_PR_sl40:
               schReqRsrc->periodicityAndOffset->choice.sl40 = schReqDb->schedAddModList[schReqIdx].offset;
               break;
            case SchedulingRequestResourceConfig__periodicityAndOffset_PR_sl80:
               schReqRsrc->periodicityAndOffset->choice.sl80 = schReqDb->schedAddModList[schReqIdx].offset;
               break;
            case SchedulingRequestResourceConfig__periodicityAndOffset_PR_sl160:
               schReqRsrc->periodicityAndOffset->choice.sl160 = schReqDb->schedAddModList[schReqIdx].offset;
               break;
            case SchedulingRequestResourceConfig__periodicityAndOffset_PR_sl320:
               schReqRsrc->periodicityAndOffset->choice.sl320 = schReqDb->schedAddModList[schReqIdx].offset;
               break;
            case SchedulingRequestResourceConfig__periodicityAndOffset_PR_sl640:
               schReqRsrc->periodicityAndOffset->choice.sl640 = schReqDb->schedAddModList[schReqIdx].offset;
               break;
         }
      }

      if(schReqDb->schedAddModList[schReqIdx].resrc)
      {
         schReqRsrc->resource = NULLP;
         DU_ALLOC(schReqRsrc->resource, sizeof(PUCCH_ResourceId_t));
         if(schReqRsrc->resource == NULLP)
         {
            DU_LOG("\nERROR  --> DU APP: Memory allocation failed in BuildPucchSchReqAddModList");
            return RFAILED;
         }
         *(schReqRsrc->resource) = schReqDb->schedAddModList[schReqIdx].resrc;

      }
   }
   return ROK;
}

/*******************************************************************
 *
 * @brief Builds PUCCH multi csi resource list
 *
 * @details
 *
 *    Function : BuildPucchMultiCsiRsrcList
 *
 *    Functionality:
 *      Builds PUCCH multi csi resource list
 *
 * @params[in] : PucchMultiCsiCfg *multiCsiDb
 *               struct PUCCH_Config__multi_CSI_PUCCH_ResourceList  *multiCsiRsrcList
 *
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
uint8_t BuildPucchMultiCsiRsrcList(PucchMultiCsiCfg *multiCsiDb, struct PUCCH_Config__multi_CSI_PUCCH_ResourceList  *multiCsiRsrcList)
{
   uint8_t elementCnt = 0, rsrcIdx = 0;

   elementCnt = multiCsiDb->multiCsiResrcListCount;
   multiCsiRsrcList->list.count = elementCnt;
   multiCsiRsrcList->list.size = elementCnt * sizeof(PUCCH_ResourceId_t *);
   multiCsiRsrcList->list.array = NULLP;
   DU_ALLOC(multiCsiRsrcList->list.array, multiCsiRsrcList->list.size);
   if(multiCsiRsrcList->list.array == NULLP)
   {
      DU_LOG("\nERROR  --> DU APP: Memory allocation failed in BuildPucchMultiCsiRsrcList");
      return RFAILED;
   }

   for(rsrcIdx = 0; rsrcIdx<multiCsiRsrcList->list.count; rsrcIdx++)
   {
      DU_ALLOC(multiCsiRsrcList->list.array[rsrcIdx], sizeof(PUCCH_ResourceId_t));
      if(multiCsiRsrcList->list.array[rsrcIdx] == NULLP)
      {
         DU_LOG("\nERROR  --> DU APP: Memory allocation failed in BuildPucchMultiCsiRsrcList");
         return RFAILED;
      }
   }

   for(rsrcIdx = 0; rsrcIdx<multiCsiRsrcList->list.count; rsrcIdx++)
   {
      *(multiCsiRsrcList->list.array[rsrcIdx]) = multiCsiDb->multiCsiResrcList[rsrcIdx];
   }
   return ROK;
}

/*******************************************************************
 *
 * @brief Builds DL data -to- Ul Ack list
 *
 * @details
 *
 *    Function : BuildDlDataToUlAckList
 *
 *    Functionality: Builds DL data -to- Ul Ack list
 *
 * @params[in] : PucchDlDataToUlAck *dlDataToUlAckDb
 *               struct PUCCH_Config__dl_DataToUL_ACK * dlDataToUlACKList
 *
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
uint8_t BuildDlDataToUlAckList(PucchDlDataToUlAck *dlDataToUlAckDb, struct PUCCH_Config__dl_DataToUL_ACK * dlDataToUlACKList)
{
   uint8_t elementCnt = 0, arrIdx = 0;

   if(dlDataToUlAckDb == NULLP)
      elementCnt = 2;
   else
      elementCnt = dlDataToUlAckDb->dlDataToUlAckListCount;

   dlDataToUlACKList->list.count = elementCnt;
   dlDataToUlACKList->list.size = elementCnt * sizeof(long *);
   DU_ALLOC(dlDataToUlACKList->list.array, dlDataToUlACKList->list.size);
   if(dlDataToUlACKList->list.array == NULLP)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildDlDataToUlAckList");
      return RFAILED;
   }   

   for(arrIdx = 0; arrIdx <  dlDataToUlACKList->list.count; arrIdx++)
   {
      DU_ALLOC(dlDataToUlACKList->list.array[arrIdx], sizeof(long));
      if(dlDataToUlACKList->list.array[arrIdx] == NULLP)
      {
         DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildDlDataToUlAckList");
         return RFAILED;
      }   
   }

   if(dlDataToUlAckDb == NULLP)
   {
      arrIdx = 0;
      *(dlDataToUlACKList->list.array[arrIdx++]) = 1;
      *(dlDataToUlACKList->list.array[arrIdx]) = 2;
   }
   else
   {
      for(arrIdx = 0; arrIdx <  dlDataToUlACKList->list.count; arrIdx++)
      {
         *(dlDataToUlACKList->list.array[arrIdx]) = dlDataToUlAckDb->dlDataToUlAckList[arrIdx];
      }
   }
   return ROK;
}

/*******************************************************************
 *
 * @brief Builds BWP UL dedicated PUCCH Config
 *
 * @details
 *
 *    Function : BuildBWPUlDedPucchCfg
 *
 *    Functionality:
 *      Builds BWP UL dedicated PUCCH Config
 *
 * @params[in] : PUCCH_Config_t *pucchCfg
 *
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
uint8_t BuildBWPUlDedPucchCfg(PucchCfg *pucchCfgDb, PUCCH_Config_t *pucchCfg)
{
   PucchResrcSetCfg *rsrcSetCfgDb = NULLP;
   PucchResrcCfg *rsrcCfgDb = NULLP;
   PucchFormatCfg *format1Db = NULLP;
   PucchFormatCfg *format2Db = NULLP;
   PucchFormatCfg *format3Db = NULLP;
   PucchFormatCfg *format4Db = NULLP;
   PucchSchedReqCfg *schReqDb = NULLP;   
   PucchMultiCsiCfg  *multiCsiDb = NULLP;
   PucchDlDataToUlAck *dlDataToUlAckDb = NULLP;

   if(pucchCfgDb)
   {
      rsrcSetCfgDb = pucchCfgDb->resrcSet;
      rsrcCfgDb = pucchCfgDb->resrc;
      format1Db = pucchCfgDb->format1;
      format2Db = pucchCfgDb->format2;
      format3Db = pucchCfgDb->format3;
      format4Db = pucchCfgDb->format4;
      schReqDb = pucchCfgDb->schedReq;
      multiCsiDb = pucchCfgDb->multiCsiCfg;
      dlDataToUlAckDb = pucchCfgDb->dlDataToUlAck;
   }

   /* RESOURCE SET */
   DU_ALLOC(pucchCfg->resourceSetToAddModList, sizeof(struct PUCCH_Config__resourceSetToAddModList));
   if(pucchCfg->resourceSetToAddModList == NULL)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildBWPUlDedPucchCfg");
      return RFAILED;
   }

   if(BuildPucchRsrcSetAddModList(rsrcSetCfgDb, pucchCfg->resourceSetToAddModList) != ROK)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildBWPUlDedPucchCfg");
      return RFAILED;
   }

   /* PUCCH RESOURCE */
   DU_ALLOC(pucchCfg->resourceToAddModList, sizeof(struct PUCCH_Config__resourceToAddModList));
   if(pucchCfg->resourceToAddModList == NULLP)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildBWPUlDedPucchCfg");
      return RFAILED;
   }

   if(BuildPucchRsrcAddModList(rsrcCfgDb, pucchCfg->resourceToAddModList) != ROK)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildBWPUlDedPucchCfg");
      return RFAILED;
   }

   /* PUCCH Format 1 */
   DU_ALLOC(pucchCfg->format1, sizeof(struct PUCCH_Config__format1));
   if(pucchCfg->format1 == NULLP)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildBWPUlDedPucchCfg");
      return RFAILED;
   }
   
   pucchCfg->format1->present = PUCCH_Config__format1_PR_setup;
   DU_ALLOC(pucchCfg->format1->choice.setup, sizeof(PUCCH_FormatConfig_t));
   if(pucchCfg->format1->choice.setup == NULLP)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildBWPUlDedPucchCfg");
      return RFAILED;
   }

   if(BuildPucchFormat(format1Db, pucchCfg->format1->choice.setup) != ROK)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildBWPUlDedPucchCfg");
      return RFAILED;
   }

   /* PUCCH Format 2 */
   if(format2Db)
   {
      DU_ALLOC(pucchCfg->format2, sizeof(struct PUCCH_Config__format2));
      if(pucchCfg->format2 == NULLP)
      {
         DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildBWPUlDedPucchCfg");
         return RFAILED;
      }

      pucchCfg->format2->present = PUCCH_Config__format2_PR_setup;
      DU_ALLOC(pucchCfg->format2->choice.setup, sizeof(PUCCH_FormatConfig_t));
      if(pucchCfg->format2->choice.setup == NULLP)
      {
         DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildBWPUlDedPucchCfg");
         return RFAILED;
      }

      if(BuildPucchFormat(format2Db, pucchCfg->format2->choice.setup) != ROK)
      {
         DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildBWPUlDedPucchCfg");
         return RFAILED;
      }
   }

   /* PUCCH Format 3 */
   if(format3Db)
   {
      DU_ALLOC(pucchCfg->format3, sizeof(struct PUCCH_Config__format3));
      if(pucchCfg->format3 == NULLP)
      {
         DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildBWPUlDedPucchCfg");
         return RFAILED;
      }

      pucchCfg->format3->present = PUCCH_Config__format3_PR_setup;
      DU_ALLOC(pucchCfg->format3->choice.setup, sizeof(PUCCH_FormatConfig_t));
      if(pucchCfg->format3->choice.setup == NULLP)
      {
         DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildBWPUlDedPucchCfg");
         return RFAILED;
      }

      if(BuildPucchFormat(format3Db, pucchCfg->format3->choice.setup) != ROK)
      {
         DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildBWPUlDedPucchCfg");
         return RFAILED;
      }
   }

   /* PUCCH Format 4 */
   if(format4Db)
   {
      DU_ALLOC(pucchCfg->format4, sizeof(struct PUCCH_Config__format4));
      if(pucchCfg->format4 == NULLP)
      {
         DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildBWPUlDedPucchCfg");
         return RFAILED;
      }

      pucchCfg->format4->present = PUCCH_Config__format4_PR_setup;
      DU_ALLOC(pucchCfg->format4->choice.setup, sizeof(PUCCH_FormatConfig_t));
      if(pucchCfg->format4->choice.setup == NULLP)
      {
         DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildBWPUlDedPucchCfg");
         return RFAILED;
      }

      if(BuildPucchFormat(format4Db, pucchCfg->format4->choice.setup) != ROK)
      {
         DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildBWPUlDedPucchCfg");
         return RFAILED;
      }
   }

   /* Scheduling Request */
   if(schReqDb && (schReqDb->schedAddModListCount != 0))
   {
      pucchCfg->schedulingRequestResourceToAddModList = NULLP;
      DU_ALLOC(pucchCfg->schedulingRequestResourceToAddModList, sizeof(struct PUCCH_Config__schedulingRequestResourceToAddModList));
      if(pucchCfg->schedulingRequestResourceToAddModList == NULLP)
      {
         DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildBWPUlDedPucchCfg");
         return RFAILED;
      }

      if(BuildPucchSchReqAddModList(schReqDb, pucchCfg->schedulingRequestResourceToAddModList) != ROK)
      {
         DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildBWPUlDedPucchCfg");
         return RFAILED;
      }
   }

   /* Multi CSI */
   if(multiCsiDb && (multiCsiDb->multiCsiResrcListCount != 0))
   {
      pucchCfg->multi_CSI_PUCCH_ResourceList = NULLP;
      DU_ALLOC(pucchCfg->multi_CSI_PUCCH_ResourceList, sizeof(struct PUCCH_Config__multi_CSI_PUCCH_ResourceList));
      if(pucchCfg->multi_CSI_PUCCH_ResourceList == NULLP)
      {
         DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildBWPUlDedPucchCfg");
         return RFAILED;
      }

      if(BuildPucchMultiCsiRsrcList(multiCsiDb, pucchCfg->multi_CSI_PUCCH_ResourceList) != ROK)
      {
         DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildBWPUlDedPucchCfg");
         return RFAILED;
      }
   }

   /* DL DATA TO UL ACK */
   DU_ALLOC(pucchCfg->dl_DataToUL_ACK, sizeof(struct PUCCH_Config__dl_DataToUL_ACK));
   if(pucchCfg->dl_DataToUL_ACK == NULLP)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildBWPUlDedPucchCfg");
      return RFAILED;
   }

   if(BuildDlDataToUlAckList(dlDataToUlAckDb, pucchCfg->dl_DataToUL_ACK) != ROK)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildBWPUlDedPucchCfg");
      return RFAILED;
   }
   
   /* TODO : spatial relation info add/mod list and power control*/

   return ROK;
}

/*******************************************************************
 *
 * @brief Fills SRS resource to add/modify list 
 *
 * @details
 *
 *    Function : BuildSrsRsrcAddModList
 *
 *    Functionality: Fills SRS resource to add/modify list
 *
 * @params[in] 
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
uint8_t BuildSrsRsrcAddModList(struct SRS_Config__srs_ResourceToAddModList *resourceList)
{
   uint8_t   elementCnt;
   uint8_t   rsrcIdx;

   elementCnt = 1;
   resourceList->list.count = elementCnt;
   resourceList->list.size = elementCnt * sizeof(SRS_Resource_t *);
   resourceList->list.array = NULLP;
   DU_ALLOC(resourceList->list.array, resourceList->list.size);
   if(!resourceList->list.array)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildSrsRsrcAddModList");
      return RFAILED;
   }

   for(rsrcIdx = 0; rsrcIdx < resourceList->list.count; rsrcIdx++)
   {
      DU_ALLOC(resourceList->list.array[rsrcIdx], sizeof(SRS_Resource_t));
      if(!resourceList->list.array[rsrcIdx])
      {
	 DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildSrsRsrcAddModList");
	 return RFAILED;
      }
   }

   rsrcIdx = 0;
   resourceList->list.array[rsrcIdx]->srs_ResourceId = SRS_RSRC_ID;
   resourceList->list.array[rsrcIdx]->nrofSRS_Ports = SRS_Resource__nrofSRS_Ports_port1;
   resourceList->list.array[rsrcIdx]->transmissionComb.present = SRS_Resource__transmissionComb_PR_n2;

   resourceList->list.array[rsrcIdx]->transmissionComb.choice.n2 = NULLP;
   DU_ALLOC(resourceList->list.array[rsrcIdx]->transmissionComb.choice.n2, \
	 sizeof(struct SRS_Resource__transmissionComb__n2));
   if(!resourceList->list.array[rsrcIdx]->transmissionComb.choice.n2)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildSrsRsrcAddModList");
      return RFAILED;
   }
   resourceList->list.array[rsrcIdx]->transmissionComb.choice.n2->combOffset_n2\
      = SRS_COMB_OFFSET_N2;
   resourceList->list.array[rsrcIdx]->transmissionComb.choice.n2->cyclicShift_n2\
      = SRS_CYCLIC_SHIFT_N2;

   resourceList->list.array[rsrcIdx]->resourceMapping.startPosition = \
								      PUSCH_START_SYMBOL;
   resourceList->list.array[rsrcIdx]->resourceMapping.nrofSymbols =  \
								     SRS_Resource__resourceMapping__nrofSymbols_n1;
   resourceList->list.array[rsrcIdx]->resourceMapping.repetitionFactor = \
									 SRS_Resource__resourceMapping__repetitionFactor_n1;

   resourceList->list.array[rsrcIdx]->freqDomainPosition = SRS_FREQ_DOM_POS;
   resourceList->list.array[rsrcIdx]->freqDomainShift = SRS_FREQ_DOM_SHIFT;
   resourceList->list.array[rsrcIdx]->freqHopping.c_SRS = C_SRS;
   resourceList->list.array[rsrcIdx]->freqHopping.b_SRS = B_SRS;
   resourceList->list.array[rsrcIdx]->freqHopping.b_hop = B_HOP;
   resourceList->list.array[rsrcIdx]->groupOrSequenceHopping = \
							       SRS_Resource__groupOrSequenceHopping_neither;

   /* Setting resource type to aperiodic for intergration purposes */
   resourceList->list.array[rsrcIdx]->resourceType.present = \
							     SRS_Resource__resourceType_PR_aperiodic;
   resourceList->list.array[rsrcIdx]->resourceType.choice.aperiodic = NULLP;
   DU_ALLOC(resourceList->list.array[rsrcIdx]->resourceType.choice.aperiodic,
	 sizeof(struct SRS_Resource__resourceType__aperiodic));
   if(!resourceList->list.array[rsrcIdx]->resourceType.choice.aperiodic)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildSrsRsrcAddModList");
      return RFAILED;
   }

   resourceList->list.array[rsrcIdx]->sequenceId = SRS_SEQ_ID;

   return ROK;
}

/*******************************************************************
 *
 * @brief Build SRS resource set Add/mod list
 *
 * @details
 *
 *    Function : BuildSrsRsrcSetAddModList
 *
 *    Functionality: Build SRS resource set Add/mod list
 *
 * @params[in] 
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
   uint8_t BuildSrsRsrcSetAddModList
(
 struct SRS_Config__srs_ResourceSetToAddModList *rsrcSetList
 )
{
   uint8_t  elementCnt;
   uint8_t  rSetIdx;
   uint8_t  rsrcIdx;
   struct SRS_ResourceSet__srs_ResourceIdList *rsrcIdList;

   elementCnt = 1;
   rsrcSetList->list.count = elementCnt;
   rsrcSetList->list.size = elementCnt * sizeof(SRS_ResourceSet_t *);
   rsrcSetList->list.array = NULLP;
   DU_ALLOC(rsrcSetList->list.array, rsrcSetList->list.size);
   if(!rsrcSetList->list.array)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildSrsRsrcSetAddModList");
      return RFAILED;
   }

   for(rSetIdx = 0; rSetIdx < rsrcSetList->list.count; rSetIdx++)
   {
      DU_ALLOC(rsrcSetList->list.array[rSetIdx], sizeof(SRS_ResourceSet_t));
      if(!rsrcSetList->list.array[rSetIdx])
      {
	 DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildSrsRsrcSetAddModList");
	 return RFAILED;
      }
   }

   rSetIdx = 0;
   rsrcSetList->list.array[rSetIdx]->srs_ResourceSetId = SRS_RSET_ID;

   /* Fill Resource Id list in resource set */
   rsrcSetList->list.array[rSetIdx]->srs_ResourceIdList = NULLP;
   DU_ALLOC(rsrcSetList->list.array[rSetIdx]->srs_ResourceIdList,\
	 sizeof(struct SRS_ResourceSet__srs_ResourceIdList));
   if(!rsrcSetList->list.array[rSetIdx]->srs_ResourceIdList)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildSrsRsrcSetAddModList");
      return RFAILED;
   }

   elementCnt = 1;
   rsrcIdList = rsrcSetList->list.array[rSetIdx]->srs_ResourceIdList;
   rsrcIdList->list.count = elementCnt;
   rsrcIdList->list.size = elementCnt * sizeof(SRS_ResourceId_t *);
   rsrcIdList->list.array = NULLP;
   DU_ALLOC(rsrcIdList->list.array, rsrcIdList->list.size);
   if(!rsrcIdList->list.array)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildSrsRsrcSetAddModList");
      return RFAILED;
   }

   for(rsrcIdx = 0; rsrcIdx < rsrcIdList->list.count; rsrcIdx++)
   {
      DU_ALLOC(rsrcIdList->list.array[rsrcIdx], sizeof(SRS_ResourceId_t));
      if(!rsrcIdList->list.array[rsrcIdx])
      {
	 DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildSrsRsrcSetAddModList");
	 return RFAILED;
      }
   }

   rsrcIdx = 0;
   *rsrcIdList->list.array[rsrcIdx] = SRS_RSRC_ID;

   /* Fill resource type */
   rsrcSetList->list.array[rSetIdx]->resourceType.present = \
							    SRS_ResourceSet__resourceType_PR_aperiodic;

   rsrcSetList->list.array[rSetIdx]->resourceType.choice.aperiodic = NULLP;
   DU_ALLOC(rsrcSetList->list.array[rSetIdx]->resourceType.choice.aperiodic, \
	 sizeof(struct SRS_ResourceSet__resourceType__aperiodic));
   if(!rsrcSetList->list.array[rSetIdx]->resourceType.choice.aperiodic)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildSrsRsrcSetAddModList");
      return RFAILED;
   }
   rsrcSetList->list.array[rSetIdx]->resourceType.choice.aperiodic->aperiodicSRS_ResourceTrigger \
      = APERIODIC_SRS_RESRC_TRIGGER;

   /* TODO : Fill values for below IEs as expected by Viavi */
   rsrcSetList->list.array[rSetIdx]->resourceType.choice.aperiodic->csi_RS = NULLP;
   rsrcSetList->list.array[rSetIdx]->resourceType.choice.aperiodic->slotOffset = NULLP;


   rsrcSetList->list.array[rSetIdx]->usage = SRS_ResourceSet__usage_codebook;
   rsrcSetList->list.array[rSetIdx]->alpha = NULLP;
   rsrcSetList->list.array[rSetIdx]->p0 = NULLP;
   rsrcSetList->list.array[rSetIdx]->pathlossReferenceRS = NULLP;
   rsrcSetList->list.array[rSetIdx]->srs_PowerControlAdjustmentStates = NULLP;

   return ROK;
}

/*******************************************************************
 *
 * @brief Builds BWP UL dedicated SRS Config
 *
 * @details
 *
 *    Function : BuildBWPUlDedSrsCfg
 *
 *    Functionality: Builds BWP UL dedicated SRS Config
 *
 * @params[in] SRS Config 
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
uint8_t BuildBWPUlDedSrsCfg(SRS_Config_t *srsCfg)
{
   srsCfg->srs_ResourceSetToReleaseList = NULLP;
   srsCfg->srs_ResourceSetToAddModList = NULLP;
   DU_ALLOC(srsCfg->srs_ResourceSetToAddModList, \
	 sizeof(struct SRS_Config__srs_ResourceSetToAddModList));
   if(!srsCfg->srs_ResourceSetToAddModList)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildBWPUlDedSrsCfg");
      return RFAILED;
   }
   if(BuildSrsRsrcSetAddModList(srsCfg->srs_ResourceSetToAddModList) != ROK)
   {
      return RFAILED;
   }

   srsCfg->srs_ResourceToReleaseList = NULLP;

   /* Resource to Add/Modify list */
   srsCfg->srs_ResourceToAddModList = NULLP;
   DU_ALLOC(srsCfg->srs_ResourceToAddModList, \
	 sizeof(struct SRS_Config__srs_ResourceToAddModList));
   if(!srsCfg->srs_ResourceToAddModList)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildBWPUlDedSrsCfg");
      return RFAILED;
   }

   if(BuildSrsRsrcAddModList(srsCfg->srs_ResourceToAddModList) != ROK)
   {
      return RFAILED;
   }
   srsCfg->tpc_Accumulation = NULLP;

   return ROK;
}



/*******************************************************************
 *
 * @brief Builds Pusch Serving cell Config
 *
 * @details
 *
 *    Function : BuildPuschSrvCellCfg
 *
 *    Functionality: Builds Pusch Serving cell Config
 *
 * @params[in] struct UplinkConfig__pusch_ServingCellConfig *puschCfg
 *
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
uint8_t BuildPuschSrvCellCfg(struct UplinkConfig__pusch_ServingCellConfig *puschCfg)
{
   puschCfg->present = UplinkConfig__pusch_ServingCellConfig_PR_setup;
   puschCfg->choice.setup = NULLP;
   DU_ALLOC(puschCfg->choice.setup, sizeof(struct PUSCH_ServingCellConfig));
   if(!puschCfg->choice.setup)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildPuschSrvCellCfg");
      return RFAILED;
   }

   puschCfg->choice.setup->codeBlockGroupTransmission = NULLP;
   puschCfg->choice.setup->rateMatching = NULLP;
   puschCfg->choice.setup->xOverhead = NULLP;
   puschCfg->choice.setup->ext1 = NULLP;

   DU_ALLOC(puschCfg->choice.setup->ext1, sizeof(struct PUSCH_ServingCellConfig__ext1));
   if(!puschCfg->choice.setup->ext1)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildPuschSrvCellCfg");
      return RFAILED;
   }

   puschCfg->choice.setup->ext1->maxMIMO_Layers = NULLP;
   DU_ALLOC(puschCfg->choice.setup->ext1->maxMIMO_Layers, sizeof(long));
   if(!puschCfg->choice.setup->ext1->maxMIMO_Layers)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildPuschSrvCellCfg");
      return RFAILED;
   }
   *(puschCfg->choice.setup->ext1->maxMIMO_Layers) = PUSCH_MAX_MIMO_LAYERS;

   puschCfg->choice.setup->ext1->processingType2Enabled= NULLP;
   DU_ALLOC(puschCfg->choice.setup->ext1->processingType2Enabled,sizeof(BOOLEAN_t));
   if(!puschCfg->choice.setup->ext1->processingType2Enabled)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildPuschSrvCellCfg");
      return RFAILED;
   }
   *(puschCfg->choice.setup->ext1->processingType2Enabled) = PUSCH_PROCESS_TYPE2_ENABLED;

   return ROK;
}

/*******************************************************************
 *
 * @brief Builds inital UL BWP
 *
 * @details
 *
 *    Function : BuildInitialUlBWP
 *
 *    Functionality: Builds initial UL BWP
 *
 * @params[in] BWP_UplinkDedicated_t *ulBwp
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
uint8_t BuildInitialUlBWP(InitialUlBwp *initUlBwp, BWP_UplinkDedicated_t *ulBwp)
{
   PucchCfg *pucchCfg = NULLP;
   PuschCfg *puschCfg = NULLP;

   if(initUlBwp)
   {
      if(initUlBwp->pucchPresent)
         pucchCfg = &initUlBwp->pucchCfg;
      if(initUlBwp->puschPresent)
         puschCfg = &initUlBwp->puschCfg;
   }

   ulBwp->pucch_Config = NULLP;
   DU_ALLOC(ulBwp->pucch_Config, sizeof(struct BWP_UplinkDedicated__pucch_Config));
   if(!ulBwp->pucch_Config)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildInitialUlBWP");
      return RFAILED;
   }

   ulBwp->pucch_Config->present = BWP_UplinkDedicated__pucch_Config_PR_setup;
   ulBwp->pucch_Config->choice.setup = NULLP;
   DU_ALLOC(ulBwp->pucch_Config->choice.setup, sizeof(PUCCH_Config_t));
   if(!ulBwp->pucch_Config->choice.setup)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildInitialUlBWP");
      return RFAILED;
   }

   if(BuildBWPUlDedPucchCfg(pucchCfg, ulBwp->pucch_Config->choice.setup) != ROK)
   {
      return RFAILED;
   }

   /* Fill BWP UL dedicated PUSCH config */
   ulBwp->pusch_Config = NULLP;
   DU_ALLOC(ulBwp->pusch_Config, sizeof(struct BWP_UplinkDedicated__pusch_Config));
   if(!ulBwp->pusch_Config)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildInitialUlBWP");
      return RFAILED;
   }

   ulBwp->pusch_Config->present = BWP_UplinkDedicated__pusch_Config_PR_setup;
   ulBwp->pusch_Config->choice.setup = NULLP;
   DU_ALLOC(ulBwp->pusch_Config->choice.setup, sizeof(PUSCH_Config_t));
   if(!ulBwp->pusch_Config->choice.setup)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildInitialUlBWP");
      return RFAILED;
   }

   if(BuildBWPUlDedPuschCfg(puschCfg, ulBwp->pusch_Config->choice.setup) != ROK)
   {
      return RFAILED;
   }

   ulBwp->configuredGrantConfig = NULLP;

   /* Fill BPW UL dedicated SRS config */
   ulBwp->srs_Config = NULLP;
   DU_ALLOC(ulBwp->srs_Config, sizeof(struct BWP_UplinkDedicated__srs_Config));
   if(!ulBwp->srs_Config)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildInitialUlBWP");
      return RFAILED;
   }

   ulBwp->srs_Config->present = BWP_UplinkDedicated__srs_Config_PR_setup;
   ulBwp->srs_Config->choice.setup = NULLP;
   DU_ALLOC(ulBwp->srs_Config->choice.setup, sizeof(SRS_Config_t));
   if(!ulBwp->srs_Config->choice.setup)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildInitialUlBWP");
      return RFAILED;
   }

   if(BuildBWPUlDedSrsCfg(ulBwp->srs_Config->choice.setup) != ROK)
   {
      return RFAILED;   
   }

   ulBwp->beamFailureRecoveryConfig = NULLP;

   return ROK;
}

/*******************************************************************
 *
 * @brief Builds UL config
 * @details
 *
 *    Function : BuildUlCfg 
 *
 *    Functionality: Builds UL config in spCellCfgDed
 *
 * @params[in] UplinkConfig_t *ulCfg
 *
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
uint8_t BuildUlCfg(ServCellRecfgInfo *servCellRecfg, UplinkConfig_t *ulCfg)
{
   InitialUlBwp *initUlBwp = NULLP;

   if(servCellRecfg)
   {
      initUlBwp = &servCellRecfg->initUlBwp;
   }

   ulCfg->initialUplinkBWP = NULLP;
   DU_ALLOC(ulCfg->initialUplinkBWP, sizeof(BWP_UplinkDedicated_t));
   if(!ulCfg->initialUplinkBWP)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory Allocation failed in BuildUlCfg");
      return RFAILED;
   }

   if(BuildInitialUlBWP(initUlBwp, ulCfg->initialUplinkBWP) != ROK)
   {
      return RFAILED;
   }

   ulCfg->uplinkBWP_ToReleaseList = NULLP;
   ulCfg->uplinkBWP_ToAddModList = NULLP;
   ulCfg->firstActiveUplinkBWP_Id = NULLP;
   DU_ALLOC(ulCfg->firstActiveUplinkBWP_Id, sizeof(BWP_Id_t));
   if(!ulCfg->firstActiveUplinkBWP_Id)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory Allocation failed in BuildUlCfg");
      return RFAILED;
   }
   if(servCellRecfg == NULLP)
      *(ulCfg->firstActiveUplinkBWP_Id) = ACTIVE_UL_BWP_ID;
   else
      *(ulCfg->firstActiveUplinkBWP_Id) = servCellRecfg->firstActvUlBwpId;

   ulCfg->pusch_ServingCellConfig = NULLP;
   DU_ALLOC(ulCfg->pusch_ServingCellConfig, sizeof(struct UplinkConfig__pusch_ServingCellConfig));
   if(!ulCfg->pusch_ServingCellConfig)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory Allocation failed in BuildUlCfg");
      return RFAILED;
   }

   if(BuildPuschSrvCellCfg(ulCfg->pusch_ServingCellConfig) != ROK)
   {
      return RFAILED;
   }

   ulCfg->carrierSwitching = NULLP;
   ulCfg->ext1 = NULLP;
   return ROK;
}

/*******************************************************************
 *
 * @brief Builds PDSCH serving cell config
 * @details
 *
 *    Function : BuildPdschSrvCellCfg
 *
 *    Functionality: Builds PDSCH serving cell config in spCellCfgDed
 *
 * @params[in] struct ServingCellConfig__pdsch_ServingCellConfig *pdschCfg 
 *
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
uint8_t BuildPdschSrvCellCfg(PdschServCellCfg *pdschServCellDb, struct ServingCellConfig__pdsch_ServingCellConfig *pdschCfg)
{
   pdschCfg->present =  ServingCellConfig__pdsch_ServingCellConfig_PR_setup;
   pdschCfg->choice.setup = NULLP;
   DU_ALLOC(pdschCfg->choice.setup, sizeof( struct PDSCH_ServingCellConfig));
   if(!pdschCfg->choice.setup)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildPdschSrvCellCfg");
      return RFAILED;
   }

   /* Code Block Group Transmission */
   pdschCfg->choice.setup->codeBlockGroupTransmission = NULLP;
   if(pdschServCellDb && (pdschServCellDb->maxCodeBlkGrpPerTb || pdschServCellDb->codeBlkGrpFlushInd))
   {
      DU_ALLOC(pdschCfg->choice.setup->codeBlockGroupTransmission, sizeof(struct PDSCH_ServingCellConfig__codeBlockGroupTransmission));
      if(pdschCfg->choice.setup->codeBlockGroupTransmission == NULLP)
      {
         DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildPdschSrvCellCfg");
         return RFAILED;
      }

      pdschCfg->choice.setup->codeBlockGroupTransmission->present = PDSCH_ServingCellConfig__codeBlockGroupTransmission_PR_setup;
      pdschCfg->choice.setup->codeBlockGroupTransmission->choice.setup = NULLP;
      DU_ALLOC(pdschCfg->choice.setup->codeBlockGroupTransmission->choice.setup, sizeof(struct PDSCH_CodeBlockGroupTransmission ));
      if(pdschCfg->choice.setup->codeBlockGroupTransmission->choice.setup == NULLP)
      {
         DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildPdschSrvCellCfg");
         return RFAILED;
      }

      pdschCfg->choice.setup->codeBlockGroupTransmission->choice.setup->maxCodeBlockGroupsPerTransportBlock = \
         *(pdschServCellDb->maxCodeBlkGrpPerTb);
      pdschCfg->choice.setup->codeBlockGroupTransmission->choice.setup->codeBlockGroupFlushIndicator = \
         *(pdschServCellDb->codeBlkGrpFlushInd);
   }

   /* xOverhead */
   pdschCfg->choice.setup->xOverhead = NULLP;
   if(pdschServCellDb && pdschServCellDb->xOverhead)
   {
      DU_ALLOC(pdschCfg->choice.setup->xOverhead, sizeof(long));
      if(pdschCfg->choice.setup->xOverhead == NULLP)
      {
         DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildPdschSrvCellCfg");
         return RFAILED;
      }
      *(pdschCfg->choice.setup->xOverhead) = *(pdschServCellDb->xOverhead);
   }

   /* Number of HARQ processes */
   pdschCfg->choice.setup->nrofHARQ_ProcessesForPDSCH = NULLP;
   DU_ALLOC(pdschCfg->choice.setup->nrofHARQ_ProcessesForPDSCH, sizeof(long));
   if(!pdschCfg->choice.setup->nrofHARQ_ProcessesForPDSCH)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildPdschSrvCellCfg");
      return RFAILED;
   }

   if(pdschServCellDb == NULLP)
   *(pdschCfg->choice.setup->nrofHARQ_ProcessesForPDSCH) = PDSCH_NUM_HARQ_PROC;
   else
   *(pdschCfg->choice.setup->nrofHARQ_ProcessesForPDSCH) = pdschServCellDb->numHarqProcForPdsch;

   pdschCfg->choice.setup->pucch_Cell = NULLP;

   /* Extension */
   pdschCfg->choice.setup->ext1 = NULLP;
   if(pdschServCellDb && pdschServCellDb->maxMimoLayers)
   {
      DU_ALLOC(pdschCfg->choice.setup->ext1, sizeof(struct PDSCH_ServingCellConfig__ext1));
      if(pdschCfg->choice.setup->ext1 == NULLP)
      {
         DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildPdschSrvCellCfg");
         return RFAILED;
      }

      DU_ALLOC(pdschCfg->choice.setup->ext1->maxMIMO_Layers, sizeof(long));
      if(pdschCfg->choice.setup->ext1->maxMIMO_Layers == NULLP)
      {
         DU_LOG("\nERROR  -->  F1AP : Memory allocation failed in BuildPdschSrvCellCfg");
         return RFAILED;
      }
      *(pdschCfg->choice.setup->ext1->maxMIMO_Layers) = *(pdschServCellDb->maxMimoLayers);
   }

   return ROK;
}

/*******************************************************************
 *
 * @brief Builds CSI Meas config
 * @details
 *
 *    Function : BuildCsiMeasCfg 
 *
 *    Functionality: Builds CSI Meas config in spCellCfgDed
 *
 * @params[in] struct ServingCellConfig__csi_MeasConfig *csiMeasCfg
 *
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
uint8_t BuildCsiMeasCfg(struct ServingCellConfig__csi_MeasConfig *csiMeasCfg)
{

   return ROK;
}

/*******************************************************************
 *
 * @brief Builds DL BWP to add/modify list
 * @details
 *
 *    Function : BuildDlBwpToAddModList
 *
 *    Functionality: Builds DL BWP to add/modify list
 *
 * @params[in] ServCellRecfgInfo *servCellRecfg, 
 *             struct ServingCellConfig__downlinkBWP_ToAddModList *dlBwpAddModList
 *
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/ 
uint8_t BuildDlBwpToAddModList(ServCellRecfgInfo *servCellRecfg, struct ServingCellConfig__downlinkBWP_ToAddModList *dlBwpAddModList)
{
   uint8_t elementCnt, idx;

   elementCnt = servCellRecfg->numDlBwpToAddOrMod;
   dlBwpAddModList->list.count = elementCnt;
   dlBwpAddModList->list.size = elementCnt * sizeof(struct BWP_Downlink *);
   dlBwpAddModList->list.array = NULLP;
   DU_ALLOC(dlBwpAddModList->list.array, dlBwpAddModList->list.size);
   if(dlBwpAddModList->list.array == NULLP)
   {
      DU_LOG("\nERROR  --> DU APP: Memory allocation failure in BuildDlBwpToAddModList");
      return RFAILED;
   }

   for(idx=0; idx<dlBwpAddModList->list.count; idx++)
   {
      DU_ALLOC(dlBwpAddModList->list.array[idx], sizeof(BWP_Downlink_t));
      if(dlBwpAddModList->list.array[idx] == NULLP)
      {
         DU_LOG("\nERROR  --> DU APP: Memory allocation failure in BuildDlBwpToAddModList");
         return RFAILED;
      }
   }

   for(idx=0; idx<dlBwpAddModList->list.count; idx++)
   {
      dlBwpAddModList->list.array[idx]->bwp_Id = servCellRecfg->dlBwpToAddOrModList[idx].bwpId;
      dlBwpAddModList->list.array[idx]->bwp_Common = NULLP;
      dlBwpAddModList->list.array[idx]->bwp_Dedicated = NULLP;
   }
   return ROK;
}

/*******************************************************************
 *
 * @brief Builds Spcell config dedicated
 * @details
 *
 *    Function : BuildSpCellCfgDed
 *
 *    Functionality: Builds sp cell config dedicated in spCellCfg
 *
 * @params[in] ServingCellConfig_t srvCellCfg
 *
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
uint8_t BuildSpCellCfgDed(DuUeCb *ueCb, ServingCellConfig_t *srvCellCfg)
{
   ServCellRecfgInfo *servCellRecfg = NULLP;
   InitialDlBwp *initDlBwp = NULLP;
   PdschServCellCfg *pdschServCellDb = NULLP;

   if(ueCb)
   {
      servCellRecfg = &ueCb->duMacUeCfg.spCellCfg.servCellCfg;
      initDlBwp = &servCellRecfg->initDlBwp;
      pdschServCellDb = &servCellRecfg->pdschServCellCfg;
   }

   srvCellCfg->tdd_UL_DL_ConfigurationDedicated = NULLP;

   srvCellCfg->initialDownlinkBWP = NULLP;
   DU_ALLOC(srvCellCfg->initialDownlinkBWP, sizeof(BWP_DownlinkDedicated_t));
   if(!srvCellCfg->initialDownlinkBWP)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failure in BuildSpCellCfgDed");
      return RFAILED;
   }

   if(BuildInitialDlBWP(initDlBwp, srvCellCfg->initialDownlinkBWP) != ROK)
   {
      DU_LOG("\nERROR  -->  F1AP : BuildInitialDlBWP failed");
      return RFAILED;
   }

   srvCellCfg->downlinkBWP_ToReleaseList = NULLP;

   srvCellCfg->downlinkBWP_ToAddModList = NULLP;
   if(ueCb && ueCb->duMacUeCfg.spCellCfg.servCellCfg.numDlBwpToAddOrMod)
   {
      DU_ALLOC(srvCellCfg->downlinkBWP_ToAddModList, sizeof(struct ServingCellConfig__downlinkBWP_ToAddModList));
      if(srvCellCfg->downlinkBWP_ToAddModList == NULLP)
      {
         DU_LOG("\nERROR  -->  F1AP : Memory allocation failure in BuildSpCellCfgDed");
         return RFAILED;
      }

      if(BuildDlBwpToAddModList(&ueCb->duMacUeCfg.spCellCfg.servCellCfg, srvCellCfg->downlinkBWP_ToAddModList) != ROK)
      {
         DU_LOG("\nERROR  -->  F1AP : BuildInitialDlBWP failed");
         return RFAILED;
      }
   }

   srvCellCfg->firstActiveDownlinkBWP_Id = NULLP;
   DU_ALLOC(srvCellCfg->firstActiveDownlinkBWP_Id, sizeof(long));
   if(!srvCellCfg->firstActiveDownlinkBWP_Id)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failure in BuildSpCellCfgDed");
      return RFAILED;
   }
   if(ueCb == NULLP)
      *(srvCellCfg->firstActiveDownlinkBWP_Id) = ACTIVE_DL_BWP_ID;
   else
      *(srvCellCfg->firstActiveDownlinkBWP_Id) = ueCb->duMacUeCfg.spCellCfg.servCellCfg.firstActvDlBwpId;

   srvCellCfg->bwp_InactivityTimer = NULLP;

   srvCellCfg->defaultDownlinkBWP_Id = NULLP;
   DU_ALLOC(srvCellCfg->defaultDownlinkBWP_Id, sizeof(long));
   if(!srvCellCfg->defaultDownlinkBWP_Id)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failure in BuildSpCellCfgDed");
      return RFAILED;
   }
   if(ueCb == NULLP)
      *(srvCellCfg->defaultDownlinkBWP_Id) = ACTIVE_DL_BWP_ID;
   else
      *(srvCellCfg->defaultDownlinkBWP_Id) = ueCb->duMacUeCfg.spCellCfg.servCellCfg.defaultDlBwpId;

   srvCellCfg->uplinkConfig = NULLP;
   DU_ALLOC(srvCellCfg->uplinkConfig, sizeof(UplinkConfig_t));
   if(!srvCellCfg->uplinkConfig)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failure in BuildSpCellCfgDed");
      return RFAILED;
   }

   if(BuildUlCfg(servCellRecfg, srvCellCfg->uplinkConfig) != ROK)
   {
      DU_LOG("\nERROR  -->  F1AP : BuildUlCfg failed");
      return RFAILED;
   }
   srvCellCfg->supplementaryUplink = NULLP;
   srvCellCfg->pdcch_ServingCellConfig = NULLP;

   srvCellCfg->pdsch_ServingCellConfig = NULLP;
   DU_ALLOC(srvCellCfg->pdsch_ServingCellConfig, sizeof(struct	ServingCellConfig__pdsch_ServingCellConfig));
   if(!srvCellCfg->pdsch_ServingCellConfig)
   {
      DU_LOG("\nERROR  -->  F1AP : Memory allocation failure in BuildSpCellCfgDed");
      return RFAILED;
   }

   if(BuildPdschSrvCellCfg(pdschServCellDb, srvCellCfg->pdsch_ServingCellConfig) != ROK)
   {
      DU_LOG("\nERROR  -->  F1AP : BuildPdschSrvCellCfg failed");
      return RFAILED;
   }

   srvCellCfg->csi_MeasConfig = NULLP;
#if 0
   DU_ALLOC(srvCellCfg->csi_MeasConfig, sizeof(struct	ServingCellConfig__csi_MeasConfig))
      if(!srvCellCfg->csi_MeasConfig)
      {
	 DU_LOG("\nERROR  -->  F1AP : Memory allocation failure in BuildSpCellCfgDed");
	 return RFAILED;
      }

   if(BuildCsiMeasCfg(srvCellCfg->csi_MeasConfig) != ROK)
   {
      DU_LOG("\nF1AP : BuildCsiMeasCfg failed");
      return RFAILED;
   }
#endif
   srvCellCfg->sCellDeactivationTimer = NULLP;
   srvCellCfg->crossCarrierSchedulingConfig = NULLP;
   srvCellCfg->tag_Id = TAG_ID;
   srvCellCfg->dummy = NULLP;
   srvCellCfg->pathlossReferenceLinking = NULLP;
   srvCellCfg->servingCellMO = NULLP;
   srvCellCfg->ext1 = NULLP;

   return ROK;
}

/*******************************************************************
 *
 * @brief Fills SCS specific carrier list in DL frequency info
 *
 * @details
 *
 *    Function : BuildScsSpecificCarrierListDl
 *
 *    Functionality: Fills SCS specific carrier list in DL frequency info
 *
 * @params[in] Pointer to struct FrequencyInfoDL__scs_SpecificCarrierList
 *
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
uint8_t BuildScsSpecificCarrierListDl(struct FrequencyInfoDL__scs_SpecificCarrierList *scsCarrierList)
{
   uint8_t elementCnt = 0, listIdx = 0;
   ScsSpecCarrier duScsSpecCarrier = duCfgParam.sib1Params.srvCellCfgCommSib.dlCfg.dlScsCarrier;

   elementCnt = ODU_VALUE_ONE;
   scsCarrierList->list.count = elementCnt;
   scsCarrierList->list.size = elementCnt * sizeof(SCS_SpecificCarrier_t *);

   DU_ALLOC(scsCarrierList->list.array, scsCarrierList->list.size);
   if(!scsCarrierList->list.array)
   {
      DU_LOG("\nERROR  -->  DU APP : Memory allocation failed for scs carrier list array \
         in BuildScsSpecificCarrierListDl()");
      return RFAILED;
   }

   for(listIdx = 0; listIdx < elementCnt; listIdx++)
   {
      DU_ALLOC(scsCarrierList->list.array[listIdx], sizeof(SCS_SpecificCarrier_t));
      if(!scsCarrierList->list.array[listIdx])
      {    
         DU_LOG("\nERROR  -->  DU APP : Memory allocation failed for SCS Specific Carrier list array \
            element in BuildScsSpecificCarrierListDl()");
         return RFAILED;
      }    
   }

   listIdx = 0;
   scsCarrierList->list.array[listIdx]->offsetToCarrier = duScsSpecCarrier.scsOffset;
   scsCarrierList->list.array[listIdx]->subcarrierSpacing = duScsSpecCarrier.scs;
   scsCarrierList->list.array[listIdx]->carrierBandwidth = duScsSpecCarrier.scsBw;

   return ROK;
}

/*******************************************************************
 *
 * @brief Fills DL frequency info in DL config common
 *
 * @details
 *
 *    Function : BuildFreqInfoDl
 *
 *    Functionality: Fills DL frequency info in DL config common
 *
 * @params[in] Pointer to DownlinkConfigCommon_t
 *
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
uint8_t BuildFreqInfoDl(FrequencyInfoDL_t *frequencyInfoDL)
{
   uint8_t freqBandIdx = 0, elementCnt = 0;
   DlCfgCommon  dlCfg = duCfgParam.sib1Params.srvCellCfgCommSib.dlCfg;

   /* TODO : Fill SSB Absolute Frequency */
   /*
      DU_ALLOC(frequencyInfoDL->absoluteFrequencySSB, sizeof(ARFCN_ValueNR_t));
      if(!frequencyInfoDL->absoluteFrequencySSB)
      {
      DU_LOG("\nERROR  -->  DU APP : Memory allocation failed for SSB Absolute Frequency in BuildFreqInfoDl()");
      return RFAILED;
      }
      frequencyInfoDL->absoluteFrequencySSB = ?;
      */

   /* NR Multi Frequency Band List */
   elementCnt = ODU_VALUE_ONE;
   frequencyInfoDL->frequencyBandList.list.count = elementCnt;
   frequencyInfoDL->frequencyBandList.list.size = elementCnt * sizeof(FreqBandIndicatorNR_t *);

   DU_ALLOC(frequencyInfoDL->frequencyBandList.list.array, frequencyInfoDL->frequencyBandList.list.size);
   if(!frequencyInfoDL->frequencyBandList.list.array)
   {
      DU_LOG("\nERROR  -->  DU APP : Memory allocation failed for Frequency Band List array in BuildFreqInfoDl()");
      return RFAILED;
   }

   for(freqBandIdx = 0; freqBandIdx < elementCnt; freqBandIdx++)
   {
      DU_ALLOC(frequencyInfoDL->frequencyBandList.list.array[freqBandIdx], sizeof(FreqBandIndicatorNR_t));
      if(!frequencyInfoDL->frequencyBandList.list.array[freqBandIdx])
      {
         DU_LOG("\nERROR  -->  DU APP : Memory allocation failed for frequency band array element in BuildFreqInfoDl()");
         return RFAILED;
      }
   }

   freqBandIdx = 0;
   *(frequencyInfoDL->frequencyBandList.list.array[freqBandIdx]) = dlCfg.freqBandInd;

   /* TODO : Absolute Frequency to Point A */
   //frequencyInfoDL->absoluteFrequencyPointA

   /* Subcarrier Spacing specifc carrier List */
   if((BuildScsSpecificCarrierListDl(&frequencyInfoDL->scs_SpecificCarrierList)) != ROK)
   {
      DU_LOG("\nERROR  -->  DU APP : Failed to fill SCS specific carrier list DL in BuildFreqInfoDl()");
      return RFAILED;
   }

   return ROK;

}

/*******************************************************************
 *
 * @brief Fills DL config common in Serving cell config common
 *
 * @details
 *
 *    Function : BuildDlConfigCommon
 *
 *    Functionality: Fills DL config common in Serving cell config common
 *
 * @params[in] Pointer to DownlinkConfigCommon_t
 *
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
uint8_t BuildDlConfigCommon(DownlinkConfigCommon_t *dlCfgCommon)
{
   /* DL Frequency Info */
   DU_ALLOC(dlCfgCommon->frequencyInfoDL, sizeof(FrequencyInfoDL_t));
   if(!dlCfgCommon->frequencyInfoDL)
   {
      DU_LOG("\nERROR  --> DU APP : Memory allocation failed for DL frequency info in BuildDlConfigCommon()");
      return RFAILED;
   }
   if((BuildFreqInfoDl(dlCfgCommon->frequencyInfoDL))!= ROK)
   {
      DU_LOG("\nERROR  --> DU APP : Failed to fill DL frequency info in BuildDlConfigCommon()");
      return RFAILED;
   }

   /* DL BWP config common */
   DU_ALLOC(dlCfgCommon->initialDownlinkBWP, sizeof(BWP_DownlinkCommon_t));
   if(!dlCfgCommon->initialDownlinkBWP)
   {
      DU_LOG("\nERROR  --> DU APP : Memory allocation failed for DL BWP config common in BuildDlConfigCommon()");
      return RFAILED;
   }
   if((BuildBwpDlCommon(dlCfgCommon->initialDownlinkBWP)) != ROK)
   {
      DU_LOG("\nERROR  --> DU APP : Failed to fill DL DWP config common in BuildDlConfigCommon()");
      return RFAILED;
   }

   return ROK;
}

/*******************************************************************
 *
 * @brief Fills SCS specific carrier list in UL frequency Info
 *
 * @details
 *
 *    Function : BuildScsSpecificCarrierListUl
 *
 *    Functionality: Fills SCS specific carrier list in UL frequency Info
 *
 * @params[in] Pointer to struct FrequencyInfoUL__scs_SpecificCarrierList
 *
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
uint8_t BuildScsSpecificCarrierListUl(struct FrequencyInfoUL__scs_SpecificCarrierList *scsCarrierList)
{
   uint8_t elementCnt = 0, listIdx = 0; 
   ScsSpecCarrier   duScsSpecCarrier = duCfgParam.sib1Params.srvCellCfgCommSib.ulCfg.ulScsCarrier;

   elementCnt = ODU_VALUE_ONE;
   scsCarrierList->list.count = elementCnt;
   scsCarrierList->list.size = elementCnt * sizeof(SCS_SpecificCarrier_t *);

   DU_ALLOC(scsCarrierList->list.array, scsCarrierList->list.size);
   if(!scsCarrierList->list.array)
   {
      DU_LOG("\nERROR  -->  DU APP : SCS Specific Carrier list memory allocation failed");
      return RFAILED;
   }

   for(listIdx = 0; listIdx < scsCarrierList->list.count; listIdx++)
   {
      DU_ALLOC(scsCarrierList->list.array[listIdx], sizeof(SCS_SpecificCarrier_t));
      if(!scsCarrierList->list.array[listIdx])
      {    
         DU_LOG("\nERROR  -->  DU APP : SCS Specific Carrier list memory allocation failed");
         return RFAILED;
      }    
   }
   listIdx = 0; 
   scsCarrierList->list.array[listIdx]->offsetToCarrier = duScsSpecCarrier.scsOffset;
   scsCarrierList->list.array[listIdx]->subcarrierSpacing = duScsSpecCarrier.scs;
   scsCarrierList->list.array[listIdx]->carrierBandwidth = duScsSpecCarrier.scsBw;

   return ROK;
}

/*******************************************************************
 *
 * @brief Fills frequency info in UL config common
 *
 * @details
 *
 *    Function : BuildFreqInfoUl
 *
 *    Functionality: Fills frequency info in UL config common
 *
 * @params[in] Pointer to FrequencyInfoUL_t
 *
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
uint8_t BuildFreqInfoUl(FrequencyInfoUL_t *frequencyInfoUL)
{
   uint8_t elementCnt = 0, listIdx= 0;
   UlCfgCommon  ulCfg = duCfgParam.sib1Params.srvCellCfgCommSib.ulCfg;

   /* NR Multi Frequency Band List */
   DU_ALLOC(frequencyInfoUL->frequencyBandList, sizeof(MultiFrequencyBandListNR_t));
   if(!frequencyInfoUL->frequencyBandList)
   {
      DU_LOG("\nERROR  -->  DU APP : Memory allocation failed for frequency band list in BuildFreqInfoUl()");
      return RFAILED;
   }

   elementCnt = ODU_VALUE_ONE;
   frequencyInfoUL->frequencyBandList->list.count = elementCnt;
   frequencyInfoUL->frequencyBandList->list.size = elementCnt * sizeof(FreqBandIndicatorNR_t *);

   DU_ALLOC(frequencyInfoUL->frequencyBandList->list.array, frequencyInfoUL->frequencyBandList->list.size);
   if(!frequencyInfoUL->frequencyBandList->list.array)
   {
      DU_LOG("\nERROR  -->  DU APP : Memory allocation failed for frequency band array in BuildFreqInfoUl()");
      return RFAILED;
   }

   for(listIdx = 0; listIdx < elementCnt; listIdx++)
   {
      DU_ALLOC(frequencyInfoUL->frequencyBandList->list.array[listIdx], sizeof(FreqBandIndicatorNR_t));
      if(!frequencyInfoUL->frequencyBandList->list.array[listIdx])
      {
         DU_LOG("\nERROR  -->  DU APP : Memory allocation failed for frequency band array element in BuildFreqInfoUl()");
         return RFAILED;
      }
   }

   listIdx = 0;
   *(frequencyInfoUL->frequencyBandList->list.array[listIdx]) = ulCfg.freqBandInd;

   /* TODO : Fill Absolute frequency point A */
   /*
      DU_ALLOC(frequencyInfoUL->absoluteFrequencyPointA, sizeof(ARFCN_ValueNR_t));
      if(!frequencyInfoUL->absoluteFrequencyPointA)
      {
      DU_LOG("\nERROR  -->  DU APP : Memory allocation failed for absolute frequency point A in BuildFreqInfoUl()");
      return RFAILED;
      }
    *(frequencyInfoUL->absoluteFrequencyPointA) = ?;
    */

   /* Subcarrier Spacing specifc carrier */
   if((BuildScsSpecificCarrierListUl(&frequencyInfoUL->scs_SpecificCarrierList)) != ROK) 
   {
      DU_LOG("\nERROR  --> DU APP : Failed to fill SCS Specific Carrier list UL in BuildFreqInfoUl()");
      return RFAILED;
   }

   /* P-MAX */
   DU_ALLOC(frequencyInfoUL->p_Max, sizeof(P_Max_t));
   if(!frequencyInfoUL->p_Max)
   {
      DU_LOG("\nERROR  -->  DU APP : UL Frequency Infoo  memory allocation failure");
      return RFAILED;
   }
   *frequencyInfoUL->p_Max = ulCfg.pMax;

   return ROK;
}

/*******************************************************************
 *
 * @brief Fills UL config common in Serving cell config common
 *
 * @details
 *
 *    Function : BuildUlConfigCommon
 *
 *    Functionality: Fills UL config common in Serving cell config common
 *
 * @params[in] Pointer to UplinkConfigCommon_t
 *
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
uint8_t BuildUlConfigCommon(UplinkConfigCommon_t *ulCfgCommon)
{
   /* UL Frequency Info */
   DU_ALLOC(ulCfgCommon->frequencyInfoUL, sizeof(FrequencyInfoUL_t));
   if(!ulCfgCommon->frequencyInfoUL)
   {
      DU_LOG("\nERROR  -->  DU APP : Memory allocation failed for UL frequency info in BuildUlConfigCommon()");
      return RFAILED;
   }

   if((BuildFreqInfoUl(ulCfgCommon->frequencyInfoUL)) != ROK)
   {
      DU_LOG("\nERROR  -->  DU APP : Failed to fill frequency info UL in BuildUlConfigCommon()");
      return RFAILED;
   }

   /* UL BWP common */
   DU_ALLOC(ulCfgCommon->initialUplinkBWP, sizeof(BWP_UplinkCommon_t));
   if(!ulCfgCommon->initialUplinkBWP)
   {
      DU_LOG("\nERROR  -->  DU APP : Memory allocation failed for initial UL BWP in BuildUlConfigCommon()");
      return RFAILED;
   }

   if((BuildBwpUlCommon(ulCfgCommon->initialUplinkBWP)) != ROK)
   {
      DU_LOG("\nERROR  -->  DU APP : Failed to fill BWP UL common in BuildUlConfigCommon()");
      return RFAILED;
   }

   /* Time Alignment timer */
   ulCfgCommon->dummy = duCfgParam.sib1Params.srvCellCfgCommSib.ulCfg.timeAlignTimerComm;

   return ROK;
}

/*******************************************************************
 *
 * @brief Fills SSB position in burst in SP cell config common
 *
 * @details
 *
 *    Function : BuildSsbPosInBurst
 *
 *    Functionality: 
 *       Fills SSB position in burst in SP cell config common
 *
 * @params[in] Pointer to struct ServingCellConfigCommon__ssb_PositionsInBurst
 *
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
uint8_t BuildSsbPosInBurst(struct ServingCellConfigCommon__ssb_PositionsInBurst *ssbPosInBurst)
{
   uint8_t bitStringSizeInBytes = 0;

   ssbPosInBurst->present = ServingCellConfigCommon__ssb_PositionsInBurst_PR_mediumBitmap;

   /* As per spec 38.331,in the definition of ServingCellConfigCommon */
   bitStringSizeInBytes = 1;
   ssbPosInBurst->choice.mediumBitmap.size = bitStringSizeInBytes * sizeof(uint8_t);

   DU_ALLOC(ssbPosInBurst->choice.mediumBitmap.buf, ssbPosInBurst->choice.mediumBitmap.size);
   if(!ssbPosInBurst->choice.mediumBitmap.buf)
   {
      DU_LOG("\nERROR  -->  DU APP : Memory Allocation failed for medium bit map buffer in BuildSsbPosInBurst()");
      return RFAILED;
   }

   if((fillBitString(&ssbPosInBurst->choice.mediumBitmap, 0, bitStringSizeInBytes, \
               duCfgParam.sib1Params.srvCellCfgCommSib.ssbPosInBurst)) != ROK)
   {
      DU_LOG("\nERROR  -->  DU APP : Failed to fill medium bit map in BuildSsbPosInBurst()");
      return RFAILED;
   }

   return ROK;
}

/*******************************************************************
 *
 * @brief Fills SP cell config common in Reconfig with Sync
 *
 * @details
 *
 *    Function : BuildSpCellConfigCommon
 *
 *    Functionality: Fills SP cell config common in Reconfig with Sync
 *
 * @params[in] Pointer to ServingCellConfigCommon_t
 *
 * @return ROK     - success
 *         RFAILED - failure
 *
 * ****************************************************************/
uint8_t BuildSpCellConfigCommon(ServingCellConfigCommon_t *spCellConfigCommon)
{
   /* Physical Cell Identity */
   DU_ALLOC(spCellConfigCommon->physCellId, sizeof(PhysCellId_t));
   if(!spCellConfigCommon->physCellId)
   {
      DU_LOG("\nERROR  -->  DU APP : Failed to allocate memory for physical cell id in BuildSpCellConfigCommon()");
      return RFAILED;
   } 
   *(spCellConfigCommon->physCellId) = NR_PCI;

   /* Downlink Config Common */
   DU_ALLOC(spCellConfigCommon->downlinkConfigCommon, sizeof(DownlinkConfigCommon_t));
   if(!spCellConfigCommon->downlinkConfigCommon)
   {
      DU_LOG("\nERROR  -->  DU APP : Failed to allocate memory for DL Config Common in BuildSpCellConfigCommon()");
      return RFAILED;
   }
   if((BuildDlConfigCommon(spCellConfigCommon->downlinkConfigCommon)) != ROK)
   {
      DU_LOG("\nERROR  -->  DU APP : Failed to fill DL config commin in BuildSpCellConfigCommon()");
      return RFAILED;
   }

   /* Uplinlink Config Common */
   DU_ALLOC(spCellConfigCommon->uplinkConfigCommon, sizeof(UplinkConfigCommon_t));
   if(!spCellConfigCommon->uplinkConfigCommon)
   {
      DU_LOG("\nERROR  -->  DU APP : Failed to allocate memory for UL Config Common in BuildSpCellConfigCommon()");
      return RFAILED;
   }
   if((BuildUlConfigCommon(spCellConfigCommon->uplinkConfigCommon)) != ROK)
   {
      DU_LOG("\nERROR  -->  DU APP : Failed to fill UL config commin in BuildSpCellConfigCommon()");
      return RFAILED;
   }

   /* Timing Advance offset */
   DU_ALLOC(spCellConfigCommon->n_TimingAdvanceOffset, sizeof(long));
   if(!spCellConfigCommon->n_TimingAdvanceOffset)
   {
      DU_LOG("\nERROR  -->  DU APP : Failed to allocate memory for Timing Advance Offset in BuildSpCellConfigCommon()");
      return RFAILED;
   }
   *(spCellConfigCommon->n_TimingAdvanceOffset) = ServingCellConfigCommon__n_TimingAdvanceOffset_n0;

   /* SSB Position In Burst */
   DU_ALLOC(spCellConfigCommon->ssb_PositionsInBurst, sizeof(struct ServingCellConfigCommon__ssb_PositionsInBurst));
   if(!spCellConfigCommon->ssb_PositionsInBurst)
   {
      DU_LOG("\nERROR  -->  DU APP : Failed to allocate memory for SSB Position In Burst in BuildSpCellConfigCommon()");
      return RFAILED;
   }
   if((BuildSsbPosInBurst(spCellConfigCommon->ssb_PositionsInBurst)) != ROK)
   {
      DU_LOG("\nERROR  -->  DU APP : Failed to fill SSB Position In Burst in BuildSpCellConfigCommon()");
      ret