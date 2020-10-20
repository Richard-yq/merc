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

/************************************************************************
 
     Name:     LTE-MAC layer
  
     Type:     C source file
  
     Desc:     C source code for Entry point fucntions
  
     File:     rg_uim.c 
  
**********************************************************************/

/** @file rg_uim.c.
@brief This module acts as an interface handler for upper interface and 
manages Pst and Sap related information for upper interface APIs.
*/

static const char* RLOG_MODULE_NAME="MAC";
static int RLOG_MODULE_ID=4096;
static int RLOG_FILE_ID=178;

/* header include files -- defines (.h) */

/* header/extern include files (.x) */
/* header include files (.h) */
#include "common_def.h"
#include "crg.h"           /* CRG defines */
#include "lrg.h"           /* layer management defines for LTE-MAC */
#include "tfu.h"
#include "rgu.h"
#include "rg_sch_inf.h"
#include "rg_env.h"        /* customizable defines and macros for MAC */
#include "rg.h"            /* defines and macros for MAC */
#include "rg_err.h"        /* RG error defines */

/* header/extern include files (.x) */

#include "crg.x"           /* CRG types */
#include "lrg.x"           /* layer management typedefs for MAC */
#include "tfu.x"
#include "rgu.x"
#include "rg_sch_inf.x"
#include "rg_prg.x"        /* PRG interface typedefs*/
#include "du_app_mac_inf.h"
#include "rg.x"            /* typedefs for MAC */

#include "ss_rbuf.h"
#include "ss_rbuf.x"

/* local defines */

/* local typedefs */
 
/* local externs */
 
/* forward references */

#if defined(SPLIT_RLC_DL_TASK) && defined(RLC_MAC_STA_RSP_RBUF)
S16 rgBatchProc(Void);
#endif
uint8_t rgRguDlSap;
uint8_t rgRguUlSap;
/**
 * @brief Handler for Bind request.
 *
 * @details
 *
 *     Function : RgUiRguBndReq
 *     
 *     This function handles the bind request from MAC Service User.
 *     
 *           
 *  @param[in]  Pst  *pst
 *  @param[in]  SuId suId
 *  @param[in]  SpId spId
 *  @return  S16
 *      -# ROK 
 *      -# RFAILED 
 **/
#ifdef ANSI
S16 RgUiRguBndReq
(
Pst  *pst,
SuId suId,
SpId spId
)
#else
S16 RgUiRguBndReq(pst, suId, spId)
Pst  *pst;
SuId suId;
SpId spId;
#endif
{
   Inst      inst; 
   S16       ret = ROK;
   Pst       tmpPst;   /* Temporary Post Structure */
   RgUstaDgn dgn;      /* Alarm diagnostics structure */

   RG_IS_INST_VALID(pst->dstInst);
   inst = pst->dstInst - RG_INST_START;

   tmpPst.prior       = pst->prior;
   tmpPst.route       = pst->route;
   tmpPst.selector    = pst->selector;
   tmpPst.region      = rgCb[inst].rgInit.region;
   tmpPst.pool        = rgCb[inst].rgInit.pool;
   tmpPst.srcProcId   = rgCb[inst].rgInit.procId;
   tmpPst.srcEnt      = rgCb[inst].rgInit.ent;
   tmpPst.srcInst     = rgCb[inst].rgInit.inst;
   tmpPst.event       = EVTNONE;
   tmpPst.dstProcId   = pst->srcProcId;
   tmpPst.dstEnt      = pst->srcEnt;
   tmpPst.dstInst     = pst->srcInst;

   if(spId == rgCb[inst].rguSap[spId].sapCfg.spId)
   {
      /* Check the state of the SAP */
      switch (rgCb[inst].rguSap[spId].sapSta.sapState)
      {
         case LRG_NOT_CFG: /* SAP Not configured */
            RGDBGINFO(inst,(rgPBuf(inst), "SAP Not Configured\n"));
            rgFillDgnParams(inst,&dgn, LRG_USTA_DGNVAL_MEM); 
            ret = rgLMMStaInd(inst,LCM_CATEGORY_INTERFACE,LRG_NOT_CFG,
                  LCM_CAUSE_INV_SAP, &dgn);
            RLOG0(L_DEBUG,"SAP Not Configured");
            ret = RgUiRguBndCfm(&tmpPst, suId, CM_BND_NOK);
            break;
         case LRG_UNBND: /* SAP is not bound */
            RLOG0(L_DEBUG,"SAP Not yet bound");
            rgCb[inst].rguSap[spId].sapSta.sapState = LRG_BND;
            rgCb[inst].rguSap[spId].sapCfg.suId = suId;
            /* Send Bind Confirm with status as SUCCESS */
            /*T2K - Passing spId as it is required to access the SAP CB*/
            ret = rgUIMRguBndCfm(inst,spId, CM_BND_OK);
            /* Indicate to Layer manager */ 
            rgFillDgnParams(inst,&dgn, LRG_USTA_DGNVAL_MEM); 
            ret = rgLMMStaInd(inst,LCM_CATEGORY_INTERFACE,LRG_EVENT_RGUSAP_ENB,
                  LCM_CAUSE_UNKNOWN, &dgn);
            break;
         case LRG_BND: /* SAP is already bound*/
            RLOG0(L_DEBUG,"SAP already bound");
            /*T2K - Passing spId as it is required to access the SAP CB*/
            ret = rgUIMRguBndCfm(inst,spId, CM_BND_OK);
            break;
         default: /* Should Never Enter here */
#if (ERRCLASS & ERRCLS_ADD_RES)      
            RGLOGERROR(inst,ERRCLS_INT_PAR, ERG008, (ErrVal)rgCb[inst].rguSap[spId].sapSta.sapState,
                  "Invalid SAP State:RgUiRguBndReq failed\n");
#endif
            /*T2K - Passing spId as it is required to access the SAP CB*/
            ret = rgUIMRguBndCfm(inst,spId, CM_BND_NOK);
            break;
      }
   }
   else
   {
#if (ERRCLASS & ERRCLS_ADD_RES)      
      RGLOGERROR(inst,ERRCLS_INT_PAR, ERG009, (ErrVal)rgCb[inst].rguSap[spId].sapCfg.spId,
            "Invalid SAP Id:RgUiRguBndReq failed\n");
#endif
      /*T2K - Passing spId as it is required to access the SAP CB*/
      ret = rgUIMRguBndCfm(inst,spId, CM_BND_NOK);
   }
   return (ret);
}  /* RgUiRguBndReq */


/**
 * @brief Handler for Unbind request.
 *
 * @details
 *
 *     Function : RgUiRguUbndReq
 *     
 *     This function handles the unbind request from MAC Service User.
 *     
 *           
 *  @param[in]  Pst    *pst
 *  @param[in]  SpId   spId
 *  @param[in]  Reason reason
 *  @return  S16
 *      -# ROK 
 *      -# RFAILED 
 **/
#ifdef ANSI
S16 RgUiRguUbndReq
(
Pst    *pst,
SpId   spId,
Reason reason
)
#else
S16 RgUiRguUbndReq(pst, spId, reason)
Pst    *pst;
SpId   spId;
Reason reason;
#endif
{
   Inst      inst;

   RG_IS_INST_VALID(pst->dstInst);
   inst = pst->dstInst - RG_INST_START;
   /* SAP Id validation */
   if (spId == rgCb[inst].rguSap[spId].sapCfg.spId)
   {
      switch(rgCb[inst].rguSap[spId].sapSta.sapState)
      {
         case LRG_BND: /* SAP is already bound*/
            RLOG0(L_DEBUG,"SAP already bound");
            /* setting SAP state to UN BOUND */
            rgCb[inst].rguSap[spId].sapSta.sapState = LRG_UNBND;
            break;
         default:
#if (ERRCLASS & ERRCLS_ADD_RES)      
     RLOG1(L_ERROR,"Invalid SAP State:%d RgUiRguUbndReq failed",
                  rgCb[inst].rguSap[spId].sapSta.sapState);
       
#endif
            break;
      }
   }
   else
   {
#if (ERRCLASS & ERRCLS_ADD_RES)      
      RGLOGERROR(inst,ERRCLS_INT_PAR, ERG011, (ErrVal)rgCb[inst].rguSap[spId].sapCfg.spId,
            "Invalid SAP Id:RgUiRguUbndReq failed\n");
#endif
      return RFAILED;
   }
   return ROK;
}  /* RgUiRguUbndReq */
/**
 * @brief API for sending bind confirm from MAC to RLC
 *
 * @details
 *
 *     Function: rgUIMRguBndCfm
 *     
 *     This API is invoked to send bind confirm from MAC to RLC.
 *     This API fills in Pst structure and SAP Ids and invokes 
 *     bind confirm API towards RLC.
 *           
 *  @param[in] Inst        inst
 *  @param[in]  SuId          suId
 *  @param[in]  uint8_t            status
 *  @return  S16
 *      -# ROK 
 *      -# RFAILED 
 **/
#ifdef ANSI
S16 rgUIMRguBndCfm
(
Inst inst,
SpId spId,
uint8_t status
)
#else
S16 rgUIMRguBndCfm(inst,spId, status)
Inst          inst;
SpId          spId;
uint8_t       status;
#endif
{
   S16  ret = ROK;
   
   ret = RgUiRguBndCfm(&rgCb[inst].rguSap[spId].sapCfg.sapPst, 
                      rgCb[inst].rguSap[spId].sapCfg.suId, status);
   if (ret != ROK)
   {
      
      RLOG0(L_ERROR,"RgUiRguBndCfm Failed ");
      return (ret);
   }
   return (ret);
}  /* rgUIMRguBndCfm*/


/**
 * @brief Handler for dedicated DatReq from RGU
 *
 * @details
 *
 *     Function : RgUiRguDDatReq
 *     
 *     This function validates SAP and invokes ROM for further processing
 *     
 *  @param[in]  Pst             *pst 
 *  @param[in]  SpId            spId 
 *  @param[in]  RguDDatReqInfo  *datReq
 *  @return  S16
 *      -# ROK 
 *      -# RFAILED 
 **/
#ifdef ANSI
S16 RgUiRguDDatReq
(
Pst             *pst,
SpId            spId,
RguDDatReqInfo  *datReq
)
#else
S16 RgUiRguDDatReq(pst, spId, datReq)
Pst             *pst;
SpId            spId;
RguDDatReqInfo  *datReq;
#endif
{
   S16   ret = ROK;
   Inst  inst;
#ifndef NO_ERRCLS 
   uint32_t   id;
   uint32_t   id1;
   uint32_t   id2;
   uint32_t   id3;
#endif
   
   RG_IS_INST_VALID(pst->dstInst);
   inst = pst->dstInst - RG_INST_START;
#ifndef NO_ERRCLS
   if (datReq == NULLP)
   {
      RLOG0(L_ERROR,"Input Message Buffer is NULL");
      return RFAILED;
   }
   
   if(rgCb[inst].rguSap[spId].sapCfg.spId == spId)
   {
      switch (rgCb[inst].rguSap[spId].sapSta.sapState)
      {
         case LRG_BND: /* SAP is bound */
            RLOG0(L_DEBUG,"SAP is already bound");
            break;
         default: /* Should never reach here */
#if (ERRCLASS & ERRCLS_ADD_RES)      
            RLOG1(L_ERROR,"Invalid SAP State:%d RgUiRguDDatReq failed",
                  rgCb[inst].rguSap[spId].sapSta.sapState);
#endif
#ifndef L2_OPTMZ
            for(id3 = 0; id3 < datReq->nmbOfUeGrantPerTti; id3++)
            {
               RG_DROP_RGUDDATREQ_MBUF(datReq->datReq[id3]);
            }
#endif
            return RFAILED;
      }
   }
   else
   {
#if (ERRCLASS & ERRCLS_ADD_RES)      
      RGLOGERROR(inst,ERRCLS_INT_PAR, ERG013, (ErrVal)spId,
            "Invalid SAP Id:RgUiRguDDatReq failed\n");
#endif
#ifndef L2_OPTMZ
      for(id3 = 0; id3 < datReq->nmbOfUeGrantPerTti; id3++)
      {
         RG_DROP_RGUDDATREQ_MBUF(datReq->datReq[id3]);
      }
#endif
      return RFAILED;
   }

   /* Update RGU SAP statistics for received sdu count */
   /*ccpu00118201 - ADD - Send trace only when its enabled*/
   if(rgCb[inst].rgInit.trc)
   {
      for(id3 = 0; id3 < datReq->nmbOfUeGrantPerTti; id3++)
      {
         RguDDatReqPerUe *datReqPerUe = &datReq->datReq[id3];
         for (id = 0; id < datReqPerUe->nmbOfTbs; id++)
         {
            for (id1 = 0; id1 < datReqPerUe->datReqTb[id].nmbLch; id1++)
            {
               /* rgCb.rguSap.sapSts.numPduRcvd is updated by 
                * rgROMDedDatReq -> rgUpdtRguDedSts function
                * So numPduRcvd updation is commented here */
               /* rgCb.rguSap.sapSts.numPduRcvd +=
                  datReq->datReqTb[id].lchData[id1].pdu.numPdu; */
               for (id2 = 0; id2 < datReqPerUe->datReqTb[id].lchData[id1].pdu.numPdu; id2++)
               {
                  RG_SEND_TRC_IND(inst,datReqPerUe->datReqTb[id].
                        lchData[id1].pdu.mBuf[id2], EVTRGUDDATREQ);
               }
            }
         }
      }
   }
#endif

   /* Call Ownership module for further processing */
   ret = rgROMDedDatReq(inst,datReq);
    SPutStaticBuffer(pst->region, pst->pool, (Data *)datReq,sizeof(RguDDatReqInfo), SS_SHARABLE_MEMORY);
   datReq = NULLP;
   return (ret);
}  /* RgUiRguDDatReq */


/**
 * @brief Handler for common DatReq from RGU
 *
 * @details
 *
 *     Function : RgUiRguCDatReq
 *     
 *     This function validates SAP invokes ROM for further processing
 *     
 *  @param[in]  Pst             *pst 
 *  @param[in]  SpId            spId 
 *  @param[in]  RguCDatReqInfo  *datReq
 *  @return  S16
 *      -# ROK 
 *      -# RFAILED 
 **/
#ifdef ANSI
S16 RgUiRguCDatReq
(
Pst             *pst,
SpId            spId,
RguCDatReqInfo  *datReq
)
#else
S16 RgUiRguCDatReq(pst, spId, datReq)
Pst             *pst;
SpId            spId;
RguCDatReqInfo  *datReq;
#endif
{
   Inst  inst;
   S16   ret = ROK;
   
   RG_IS_INST_VALID(pst->dstInst);
   inst = pst->dstInst - RG_INST_START;
#ifndef NO_ERRCLS
   if (datReq == NULLP)
   {
      RLOG0(L_ERROR,"Input Message Buffer is NULL");
      return RFAILED;
   }
   
   if(rgCb[inst].rguSap[spId].sapCfg.spId == spId)
   {
      switch (rgCb[inst].rguSap[spId].sapSta.sapState)
      {
         case LRG_BND: /* SAP is bound */
            RLOG0(L_DEBUG,"SAP is already bound");
            break;
         default: /* Should never reach here */
#if (ERRCLASS & ERRCLS_ADD_RES)      
            RLOG1(L_ERROR,"Invalid SAP State:%d RgUiRguCDatReq failed",
                  rgCb[inst].rguSap[spId].sapSta.sapState);
#endif
            return RFAILED;
      }
   }
   else
   {
#if (ERRCLASS & ERRCLS_ADD_RES)      
      RLOG1(L_ERROR,"Invalid SAP Id:%d RgUiRguCDatReq failed ",spId);
#endif
      return RFAILED;
   }
#endif

   /* Update RGU SAP statistics for received sdu count */
   /* rgCb.rguSap.sapSts.numPduRcvd is updated by 
    * rgROMCmnDatReq ->rgUpdtRguCmnSts function
    * So numPduRcvd updation is commented here */
   /* rgCb.rguSap.sapSts.numPduRcvd++; */

   ret = rgROMCmnDatReq(inst,datReq);
   /*ccpu00118201 - ADD - Send trace only when its enabled*/
   if(rgCb[inst].rgInit.trc)
   {
      RG_SEND_TRC_IND(inst,datReq->pdu, EVTRGUCDATREQ);
   }
   if (ret == RFAILED)
   {
      RG_DROP_RGUCDATREQ_MBUF(datReq);
   }
   ret = SPutStaticBuffer(pst->region, pst->pool,(Data *)datReq,sizeof(RguCDatReqInfo) , SS_SHARABLE_MEMORY);
   datReq = NULLP;
   return (ret);
}  /* RgUiRguCDatReq */


/**
 * @brief Handler for dedicated StaRsp from RGU
 *
 * @details
 *
 *     Function : RgUiRguDStaRsp
 *     
 *     This function validates SAP and invokes ROM for further processing
 *           
 *  @param[in]  Pst             *pst 
 *  @param[in]  SpId            spId 
 *  @param[in]  RguDStaRspInfo  *staRsp
 *  @return  S16
 *      -# ROK 
 *      -# RFAILED 
 **/
#ifdef ANSI
S16 RgUiRguDStaRsp
(
Pst             *pst,
SpId            spId,
RguDStaRspInfo  *staRsp
)
#else
S16 RgUiRguDStaRsp(pst, spId, staRsp)
Pst             *pst;
SpId            spId;
RguDStaRspInfo  *staRsp;
#endif
{
   Inst  inst;
   S16   ret       = ROK;
   volatile uint32_t     startTime = 0;


   RG_IS_INST_VALID(pst->dstInst);
   inst = pst->dstInst - RG_INST_START;
   /*starting Task*/
   SStartTask(&startTime, PID_MAC_STA_RSP);

   ret = rgROMDedStaRsp(inst,staRsp);
   if (ret != ROK)
   {
      RLOG_ARG0(L_ERROR,DBG_CELLID,staRsp->cellId,
                "Processing Of Status Response Failed");
   }


   /*stoping Task*/
   SStopTask(startTime, PID_MAC_STA_RSP);
   return (ret);
}  /* RgUiRguDStaRsp */


/**
 * @brief Handler for common StaRsp from RGU
 *
 * @details
 *
 *     Function : RgUiRguCStaRsp
 *     
 *     This function validates SAP and invokes ROM 
 *     for further processing
 *     
 *           
 *  @param[in]  Pst             *pst 
 *  @param[in]  SpId            spId 
 *  @param[in]  RguCStaRspInfo  *staRsp
 *  @return  S16
 *      -# ROK 
 *      -# RFAILED 
 **/
#ifdef ANSI
S16 RgUiRguCStaRsp
(
Pst             *pst,
SpId            spId,
RguCStaRspInfo  *staRsp
)
#else
S16 RgUiRguCStaRsp(pst, spId, staRsp)
Pst             *pst;
SpId            spId;
RguCStaRspInfo  *staRsp;
#endif
{
   Inst  inst;
   S16   ret = ROK;

   RG_IS_INST_VALID(pst->dstInst);
   inst = pst->dstInst - RG_INST_START;
#ifndef NO_ERRCLS
   if (staRsp == NULLP)
   {
      RLOG0(L_ERROR,"Input Response Buffer is NULL");
      return RFAILED;
   }

   if (spId == rgCb[inst].rguSap[spId].sapCfg.spId)
   {
      switch (rgCb[inst].rguSap[spId].sapSta.sapState)
      {
         case LRG_BND: /* SAP is bound */
            RLOG0(L_DEBUG,"SAP is already bound");
            break;
         default: /* Should never reach here */
#if (ERRCLASS & ERRCLS_ADD_RES)      
            RLOG1(L_ERROR,"Invalid SAP State:%d RgUiRguCStaRsp failed",
                  rgCb[inst].rguSap[spId].sapSta.sapState);
#endif
            return RFAILED;
      }
   }
   else
   {
#if (ERRCLASS & ERRCLS_ADD_RES)      
      RLOG1(L_ERROR,"Invalid SAP Id:%d RgUiRguCStaRsp failed",spId);
#endif
      return RFAILED;
   }
#endif

   ret = rgROMCmnStaRsp(inst,staRsp);
   if (ret != ROK)
   {
      RLOG_ARG0(L_ERROR,DBG_CELLID,staRsp->cellId,"Processing Of Status Response Failed");
      return (ret);
   }

   ret = SPutStaticBuffer(pst->region, pst->pool, (Data *)staRsp,sizeof(RguCStaRspInfo) , SS_SHARABLE_MEMORY);
   staRsp = NULLP;
   return (ret);
}  /* RgUiRguCStaRsp */

#ifdef LTE_L2_MEAS

/**
 * @brief Handler for L2M MeasReq from RGU
 *
 * @details
 *
 *     Function :RgUiRguL2MUlThrpMeasReq 
 *     
 *     This function validates SAP and invokes ROM for further processing
 *           
 *  @param[in]  Pst             *pst 
 *  @param[in]  SpId            spId 
 *  @param[in]  RguL2MUlThrpMeasReqInfo  *measReq
 *  @return  S16
 *      -# ROK 
 *      -# RFAILED 
 **/
#ifdef ANSI
S16 RgUiRguL2MUlThrpMeasReq 
(
Pst             *pst,
SpId            spId,
RguL2MUlThrpMeasReqInfo  *measReq
)
#else
S16 RgUiRguL2MUlThrpMeasReq(pst, spId, measReq)
Pst             *pst;
SpId            spId;
RguL2MUlThrpMeasReqInfo  *measReq;
#endif
{
   Inst  inst;

   S16   ret = ROK;

   RG_IS_INST_VALID(pst->dstInst);
   inst = pst->dstInst - RG_INST_START;
#ifndef NO_ERRCLS
   if (measReq == NULLP)
   {
      RLOG0(L_ERROR,"Input Response Buffer is NULL");
      return RFAILED;
   }

   if (spId == rgCb[inst].rguSap[spId].sapCfg.spId)
   {
      switch (rgCb[inst].rguSap[spId].sapSta.sapState)
      {
         case LRG_BND: /* SAP is bound */
            RLOG0(L_DEBUG,"SAP is already bound");
            break;
         default: /* Should never reach here */
#if (ERRCLASS & ERRCLS_ADD_RES)      
            RLOG1(L_ERROR,"Invalid SAP State:%d RgUiRguL2MUlThrpMeasReq failed",
                  rgCb[inst].rguSap[spId].sapSta.sapState);
#endif
            return RFAILED;
      }
   }
   else
   {
#if (ERRCLASS & ERRCLS_ADD_RES)      
      RLOG1(L_ERROR,"Invalid SAP Id:%d RgUiRguL2MUlThrpMeasReq failed",spId);
#endif
      return RFAILED;
   }
#endif

   ret = rgROML2MUlThrpMeasReq(inst,measReq);
   if (ret != ROK)
   {
      RLOG_ARG0(L_ERROR,DBG_CELLID,measReq->cellId,"Processing Of Meas Request Failed");
   }

  SPutStaticBuffer(pst->region, pst->pool, (Data *)measReq,sizeof(RguL2MUlThrpMeasReqInfo) , SS_SHARABLE_MEMORY);
   measReq= NULLP;
   return (ret);
}  /* RgUiRguL2MUlThrpMeasReq */
#endif

/**
 * @brief Handler for sending staInd to dedicated logical channels of a UE 
 *
 * @details
 *
 *     Function : rgUIMSndDedStaInd
 *     
 *     This function fills SAP and Pst information to send the staInd to
 *     a UE.
 *     
 *           
 *  @param[in] Inst        inst
 *  @param[in] RgUpSapCb  *rguSap 
 *  @param[in]  RgRguDedStaInd  *staInd
 *  @return  S16
 *      -# ROK 
 *      -# RFAILED 
 **/
#ifdef ANSI
S16 rgUIMSndDedStaInd
(
Inst         inst,
RgUpSapCb    *rguSap,
RgRguDedStaInd  *staInd
)
#else
S16 rgUIMSndDedStaInd(inst,rguSap,staInd)
Inst         inst;
RgUpSapCb    *rguSap;
RgRguDedStaInd  *staInd;
#endif
{
   S16  ret = ROK;
   
   RGDBGPRM(inst,(rgPBuf(inst),"rgUIMSndDedStaInd(): staInd = %p;\n", (void *)staInd));
   
   ret = RgUiRguDStaInd(&(rguSap->sapCfg.sapPst), rguSap->sapCfg.suId, 
         staInd);
   if (ret != ROK)
   {
      RLOG_ARG0(L_ERROR,DBG_CELLID,staInd->cellId,"RgUiRguDStaInd Failed");
      return (ret);
   }
   return (ret);
}  /* rgUIMSndDedStaInd */


/**
 * @brief Handler for sending staInd to a common logical channel.
 *
 * @details
 *
 *     Function : rgUIMSndCmnStaInd
 *     
 *     This function fills SAP and Pst information to send the staInd to
 *     a common logical channel.
 *     
 *           
 *  @param[in] Inst        inst
 *  @param[in] RgUpSapCb  *rguSap 
 *  @param[in]  RgRguCmnStaInd  *staInd
 *  @return  S16
 *      -# ROK 
 *      -# RFAILED 
 **/
#ifdef ANSI
S16 rgUIMSndCmnStaInd
(
Inst            inst,
RgUpSapCb    *rguDlSap,
RgRguCmnStaInd  *staInd
)
#else
S16 rgUIMSndCmnStaInd(inst,rguDlSap,staInd)
Inst          inst,
RgUpSapCb    *rguDlSap,
RgRguCmnStaInd  *staInd;
#endif
{
   S16  ret = ROK;

   ret = RgUiRguCStaInd(&(rguDlSap->sapCfg.sapPst), rguDlSap->sapCfg.suId, 
         staInd);
   if (ret != ROK)
   {
      RLOG_ARG0(L_ERROR,DBG_CELLID,staInd->cellId,"RgUiRguCStaInd Failed");
      return (ret);
   }
   return (ret);
}  /* rgUIMSndCmnStaInd */


/**
 * @brief Handler for sending datInd to dedicated logical channels of a UE 
 *
 * @details
 *
 *     Function : rgUIMSndDedDatInd
 *     
 *     This function fills SAP and Pst information to send the datInd to
 *     a UE.
 *     
 *           
 *  @param[in] Inst        inst
 *  @param[in] RgUpSapCb  *rguUlSap 
 *  @param[in]  RgRguDedDatInd  *datInd
 *  @return  S16
 *      -# ROK 
 *      -# RFAILED 
 **/
#ifdef ANSI
S16 rgUIMSndDedDatInd
(
Inst         inst,
RgUpSapCb    *rguUlSap,
RgRguDedDatInd  *datInd
)
#else
S16 rgUIMSndDedDatInd(datInd)
Inst         inst;
RgUpSapCb    *rguUlSap;
RgRguDedDatInd  *datInd;
#endif
{
   S16  ret = ROK;

   rguUlSap->sapSts.numPduTxmit += datInd->numLch;
#ifndef SS_RBUF
   ret = RgUiRguDDatInd(&(rguUlSap->sapCfg.sapPst), rguUlSap->sapCfg.suId, 
         datInd);
   if (ret != ROK)
   {
      RLOG_ARG0(L_ERROR,DBG_CELLID,datInd->cellId,"RgUiRguDdatInd Failed");
      return (ret);
   }
#else
   SRngIncrWIndx(SS_RNG_BUF_ULMAC_TO_ULRLC);
   SsRngInfoTbl[SS_RNG_BUF_ULMAC_TO_ULRLC].pktRate++;
#endif
   return (ret);
}  /* rgUIMSndDedDatInd */


/**
 * @brief Handler for sending datInd to a common logical channel.
 *
 * @details
 *
 *     Function : rgUIMSndCmnDatInd
 *     
 *     This function fills SAP and Pst information to send the datInd to
 *     a common logical channel.
 *     
 *           
 *  @param[in] Inst        inst
 *  @param[in] RgUpSapCb  *rguSap 
 *  @param[in]  RgRguCmnDatInd  *datInd
 *  @return  S16
 *      -# ROK 
 *      -# RFAILED 
 **/
#ifdef ANSI
S16 rgUIMSndCmnDatInd
(
Inst         inst,
RgUpSapCb    *rguUlSap,
RgRguCmnDatInd  *datInd
)
#else
S16 rgUIMSndCmnDatInd(datInd)
Inst         inst;
RgUpSapCb    *rguUlSap;
RgRguCmnDatInd  *datInd;
#endif
{
   S16  ret = ROK;

   RGDBGPRM(inst,(rgPBuf(inst),"rgUIMSndCmnDatInd(): staInd = %p;\n", (void *)datInd));

   rguUlSap->sapSts.numPduTxmit++;

   RGDBGPRM(inst,(rgPBuf(inst),"rgUIMSndCmnDatInd suId = %d\n", rguUlSap->sapCfg.suId));   
   ret = RgUiRguCDatInd(&(rguUlSap->sapCfg.sapPst), rguUlSap->sapCfg.suId, 
         datInd);
   if (ret != ROK)
   {
      RGDBGERRNEW(inst,(rgPBuf(inst),"RgUiRguCDatInd Failed\n"));
      RLOG_ARG0(L_ERROR,DBG_CELLID,datInd->cellId,"RgUiRguCDatInd Failed");
      return (ret);
   }
   return (ret);
}  /* rgUIMSndCmnDatInd */

/**

 * @brief API for bind request from RRC towards MAC. 
 *
 * @details
 *
 *     Function: RgUiCrgBndReq
 *     
 *     This API is invoked by RRC towards MAC to bind CRG SAP. 
 *     These API validates the Pst, spId, suId and sends the bind confirm to RRC.
 *
 *           
 *  @param[in]  Pst   *pst
 *  @param[in]  SuId  suId
 *  @param[in]  SpId  spId
 *  @return  S16
 *      -# ROK 
 *      -# RFAILED 
 **/
#ifdef ANSI
S16 RgUiCrgBndReq
(
Pst   *pst, 
SuId  suId,
SpId  spId
)
#else
S16 RgUiCrgBndReq(pst, suId, spId)
Pst   *pst; 
SuId  suId;
SpId  spId;
#endif
{
   S16       ret = ROK;
   Pst       tmpPst;   /* Temporary Post Structure */
   RgUstaDgn dgn;      /* Alarm diagnostics structure */
   Inst      inst;

   RG_IS_INST_VALID(pst->dstInst);
   inst = pst->dstInst - RG_INST_START;

   tmpPst.prior       = pst->prior;
   tmpPst.route       = pst->route;
   tmpPst.selector    = pst->selector;
   tmpPst.region      = rgCb[inst].rgInit.region;
   tmpPst.pool        = rgCb[inst].rgInit.pool;
   tmpPst.srcProcId   = rgCb[inst].rgInit.procId;
   tmpPst.srcEnt      = rgCb[inst].rgInit.ent;
   tmpPst.srcInst     = rgCb[inst].rgInit.inst;
   tmpPst.event       = EVTNONE;
   tmpPst.dstProcId   = pst->srcProcId;
   tmpPst.dstEnt      = pst->srcEnt;
   tmpPst.dstInst     = pst->srcInst;


   if(spId == rgCb[inst].crgSap.sapCfg.spId)
   {
      /* Check the state of the SAP */
      switch (rgCb[inst].crgSap.sapSta.sapState)
      {
         case LRG_NOT_CFG: /* SAP Not configured */
            
            rgFillDgnParams(inst,&dgn, LRG_USTA_DGNVAL_MEM); 
            ret = rgLMMStaInd(inst,LCM_CATEGORY_INTERFACE,LRG_NOT_CFG,
                  LCM_CAUSE_INV_SAP, &dgn);
            RLOG0(L_DEBUG,"SAP Not Configured");
            ret = RgUiCrgBndCfm(&tmpPst, suId, CM_BND_NOK);
            break;
         case LRG_UNBND: /* SAP is not bound */
            RLOG0(L_DEBUG,"SAP Not yet bound");
            
            rgCb[inst].crgSap.sapSta.sapState = LRG_BND;
            rgCb[inst].crgSap.sapCfg.suId = suId;
            /* Send Bind Confirm with status as SUCCESS */
            ret = rgUIMCrgBndCfm(inst,suId, CM_BND_OK);
            /* Indicate to Layer manager */
            rgFillDgnParams(inst,&dgn, LRG_USTA_DGNVAL_MEM); 
            ret = rgLMMStaInd(inst,LCM_CATEGORY_INTERFACE,LRG_EVENT_CRGSAP_ENB,
                  LCM_CAUSE_UNKNOWN, &dgn);
            break;
         case LRG_BND: /* SAP is already bound*/
            RLOG0(L_DEBUG,"SAP is already bound");
            
            ret = rgUIMCrgBndCfm(inst,suId, CM_BND_OK);
            break;
         default: /* Should Never Enter here */
#if (ERRCLASS & ERRCLS_ADD_RES)      
            RLOG1(L_ERROR,"Invalid SAP State:%d RgUiCrgBndReq failed",
                  rgCb[inst].crgSap.sapSta.sapState);
#endif
            ret = rgUIMCrgBndCfm(inst,suId, CM_BND_NOK);
            break;
      }
   }
   else
   {
#if (ERRCLASS & ERRCLS_ADD_RES)      
      RLOG1(L_ERROR,"Invalid SAP Id:%d RgUiCrgBndReq failed",
           rgCb[inst].crgSap.sapCfg.spId);
#endif
      ret = rgUIMCrgBndCfm(inst,suId, CM_BND_NOK);
   }
   return (ret);
}  /* RgUiCrgBndReq */


/**
 * @brief API for unbind request from RRC towards MAC. 
 *
 * @details
 *
 *     Function: RgUiCrgUbndReq
 *     
 *     This API is invoked by RRC towards MAC to unbind CRG SAP. 
 *     These API validates the Pst, spId, suId and sends the bind confirm to RRC.
 *
 *           
 *  @param[in]  Pst    *pst
 *  @param[in]  SuId   suId
 *  @param[in]  Reason reason
 *  @return  S16
 *      -# ROK 
 *      -# RFAILED 
 **/
#ifdef ANSI
S16 RgUiCrgUbndReq
(
Pst    *pst,
SpId   spId,
Reason reason
)
#else
S16 RgUiCrgUbndReq(pst, spId, reason)
Pst    *pst; 
SpId   spId;
Reason reason;
#endif
{
   Inst      inst;

   RG_IS_INST_VALID(pst->dstInst);
   inst = pst->dstInst - RG_INST_START;
   /* SAP Id validation */
   if (spId == rgCb[inst].crgSap.sapCfg.spId)
   {
      switch(rgCb[inst].crgSap.sapSta.sapState)
      {
         case LRG_BND: /* SAP is already bound*/
            /* setting SAP state to UN BOUND */
            RLOG0(L_DEBUG, "SAP is already bound");
            
            rgCb[inst].crgSap.sapSta.sapState = LRG_UNBND;
            break;
         default:
#if (ERRCLASS & ERRCLS_ADD_RES)
            RLOG1(L_ERROR,"Invalid SAP State:%d RgUiCrgUbndReq failed",
                  rgCb[inst].crgSap.sapSta.sapState);
#endif
            return RFAILED;
      }
   }
   else
   {
#if (ERRCLASS & ERRCLS_ADD_RES)      
      RLOG1(L_ERROR,"Invalid SAP Id:%d RgUiCrgUbndReq failed",
            rgCb[inst].crgSap.sapCfg.spId);
#endif
      return RFAILED;
   }
   return ROK;
}  /* RgUiCrgUbndReq */

/**
 * @brief API for sending bind confirm from MAC to RRC
 *
 * @details
 *
 *     Function: rgUIMRgrBndCfm
 *     
 *     This API is invoked to send bind confirm from MAC to RRC.
 *     This API fills in Pst structure and SAP Ids and invokes 
 *     bind confirm API towards RRC.
 *           
 *  @param[in] Inst        inst
 *  @param[in]  SuId          suId
 *  @param[in]  uint8_t            status
 *  @return  S16
 *      -# ROK 
 *      -# RFAILED 
 **/
#ifdef ANSI
S16 rgUIMCrgBndCfm
(
Inst  inst,
SuId suId,
uint8_t status
)
#else
S16 rgUIMCrgBndCfm(inst,suId, status)
Inst          inst;
SuId          suId;
uint8_t       status;
#endif
{

   if(RgUiCrgBndCfm(&(rgCb[inst].crgSap.sapCfg.sapPst), rgCb[inst].crgSap.sapCfg.suId, status) != ROK)
   {
      RLOG0(L_ERROR,"RgUiCrgBndCfm Failed ");
      return RFAILED;
   }

   return ROK;
}  /* rgUIMCrgBndCfm*/

/**
 * @brief API for configuration request from RRC towards MAC. 
 *
 * @details
 *
 *     Function: RgUiCrgCfgReq
 *     
 *     This API is invoked by RRC towards MAC to configure MAC. 
 *     These API validates the Pst, spId, suId and transfers the config request 
 *     specific information to corresponding ownership module (COM) API.
 *
 *           
 *  @param[in]  Pst           *pst
 *  @param[in]  SpId          spId
 *  @param[in]  CrgCfgTransId transId
 *  @param[in]  CrgCfgReqInfo *cfgReqInfo
 *  @return  S16
 *      -# ROK 
 *      -# RFAILED 
 **/
#ifdef ANSI
S16 RgUiCrgCfgReq
(
Pst           *pst, 
SpId          spId,
CrgCfgTransId transId,
CrgCfgReqInfo *cfgReqInfo
)
#else
S16 RgUiCrgCfgReq(pst, spId, transId, cfgReqInfo)
Pst           *pst; 
SpId          spId;
CrgCfgTransId transId;
CrgCfgReqInfo *cfgReqInfo;
#endif
{
   Inst      inst;
   S16       ret       = ROK;
   uint8_t   cfmStatus = 0x00ff;
   uint8_t   prntTrans[CRG_CFG_TRANSID_SIZE+1];

   RG_IS_INST_VALID(pst->dstInst);
   inst = pst->dstInst - RG_INST_START;
   /* Ensuring transId is always Null terminated. */
   memcpy(prntTrans, transId.trans, CRG_CFG_TRANSID_SIZE);
   prntTrans[CRG_CFG_TRANSID_SIZE] = '\0';


   /* CrgCfgReqInfo Validation for NULLP */
   if (cfgReqInfo == NULLP)
   {
      RLOG0(L_ERROR,"Input Param crgReqInfo is NULL ");
      rgUIMCrgCfgCfm(inst,transId, cfmStatus); 
      return RFAILED;
   }

   /* Upper SAP Id and State validation */
   if (spId == rgCb[inst].crgSap.sapCfg.spId)
   {
      switch(rgCb[inst].crgSap.sapSta.sapState)
      {
         case LRG_BND: /* SAP is already bound */
            RLOG0(L_DEBUG,"SAP is already bound");
            break;
         default: /* Should never reach here */
#if (ERRCLASS & ERRCLS_ADD_RES)      
            RLOG1(L_ERROR,"Invalid SAP State:%d RgUiCrgCfgReq failed",
                  rgCb[inst].crgSap.sapSta.sapState);
#endif
         SPutSBuf (pst->region, pst->pool, (Data *)cfgReqInfo,
               sizeof(CrgCfgReqInfo));
         cfgReqInfo = NULLP;

            rgUIMCrgCfgCfm(inst,transId, cfmStatus);
            return RFAILED;
      }
   }
   else
   {
#if (ERRCLASS & ERRCLS_ADD_RES)      
      RLOG1(L_ERROR,"Invalid SAP Id:%d RgUiCrgCfgReq failed",
            rgCb[inst].crgSap.sapCfg.spId);
#endif
      SPutSBuf (pst->region, pst->pool, (Data *)cfgReqInfo,
            sizeof(CrgCfgReqInfo));
      cfgReqInfo = NULLP;
      rgUIMCrgCfgCfm(inst,transId, cfmStatus); 
      return RFAILED;
   }
   ret = rgCOMCfgReq(inst,transId, cfgReqInfo);
   SPutSBuf (pst->region, pst->pool, (Data *)cfgReqInfo,
         sizeof(CrgCfgReqInfo));
   cfgReqInfo = NULLP;
   if (ret != ROK)
   {
      RLOG0(L_ERROR,"Configuration Request Handling Failed ");
      return RFAILED;
   }

   return ROK;
}  /* RgUiCrgCfgReq */

/**
 * @brief API for sending configuration confirm from MAC to RRC
 *
 * @details
 *
 *     Function: rgUIMCrgCfgCfm
 *     
 *     This API is invoked to send configuration confirm from MAC to RRC.
 *     This API fills in Pst structure and SAP Ids and invokes 
 *     config confirm API towards RRC.
 *           
 *  @param[in] Inst        inst
 *  @param[in]  CrgCfgTransId transId
 *  @param[in]  uint8_t            status
 *  @return  S16
 *      -# ROK 
 *      -# RFAILED 
 **/
#ifdef ANSI
S16 rgUIMCrgCfgCfm
(
Inst      inst,
CrgCfgTransId transId,
uint8_t       status
)
#else
S16 rgUIMCrgCfgCfm(inst,transId, status)
Inst      inst;
CrgCfgTransId transId;
uint8_t       status;
#endif
{
   S16  ret = ROK;
   uint8_t   prntTrans[CRG_CFG_TRANSID_SIZE+1];

   memcpy(prntTrans, transId.trans, CRG_CFG_TRANSID_SIZE);
   prntTrans[CRG_CFG_TRANSID_SIZE] = '\0';


   ret = RgUiCrgCfgCfm(&(rgCb[inst].crgSap.sapCfg.sapPst), rgCb[inst].crgSap.sapCfg.suId, transId, status);
   if (ret != ROK)
   {
      RLOG0(L_ERROR,"RgUiCrgCfgCfm Failed ");
      return (ret);
   }

   return (ret);
}  /* rgUIMCrgCfgCfm */
#if defined(SPLIT_RLC_DL_TASK) && defined(RLC_MAC_STA_RSP_RBUF)

#ifdef ANSI
S16 rgBatchProc
(
Void
)
#else
S16 rgBatchProc()
Void;
#endif
{
/* Read from Ring Buffer and process RLC BO Update */
   Pst pst = {0};
   SpId spId = 0;
   RguDStaRspInfo  *staRsp;
   uint32_t elmIndx = 0;
#ifndef LTE_ADV
/* Fill pst */
   pst.srcProcId = 1;
   pst.dstProcId = 1;
   pst.dstEnt = ENTMAC;
   pst.dstInst = 0;
   pst.srcEnt = ENTRLC;
   pst.srcInst = 1;
   pst.prior = PRIOR0;
   pst.route = RTESPEC;
   pst.event = EVTRGUDSTARSP;
   pst.region = 0;
   pst.pool = 0;
   pst.selector = 2; /*SM_SELECTOR_LC */
#else
#endif
  
   elmIndx = (uint32_t)SRngGetRIndx(SS_RNG_BUF_DLRLC_TO_DLMAC);
   while(NULLP != elmIndx)
   {
      staRsp = (RguDStaRspInfo *)elmIndx;
#ifdef LTE_ADV
      pst = staRsp->post;
#endif
      RgUiRguDStaRsp(&pst, spId, staRsp);

      elmIndx = NULLP;
      staRsp = NULLP;
      SRngIncrRIndx(SS_RNG_BUF_DLRLC_TO_DLMAC);

      if((elmIndx = (uint32_t)SRngGetRIndx(SS_RNG_BUF_DLRLC_TO_DLMAC)) == NULLP)
      break;
   }
   return ROK;
}
#endif

/**********************************************************************
 
         End of file
**********************************************************************/
