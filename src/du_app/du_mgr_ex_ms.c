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

/* This file contains message handling functionality for DU APP */

#include "du_sctp.h"
#include "du_f1ap_msg_hdl.h"
#include "lsctp.h"
#include "legtp.h"

extern S16 cmUnpkLkwCfgCfm(LkwCfgCfm func,Pst *pst, Buffer *mBuf);
extern S16 cmUnpkLkwCntrlCfm(LkwCntrlCfm func,Pst *pst, Buffer *mBuf);
extern S16 cmUnpkLrgCfgCfm(LrgCfgCfm func,Pst *pst, Buffer *mBuf);

/**************************************************************************
 * @brief Task Initiation callback function. 
 *
 * @details
 *
 *     Function : duActvInit 
 *    
 *     Functionality:
 *             This function is supplied as one of parameters during DU APP's 
 *             task registration. SSI will invoke this function once, after
 *             it creates and attaches this TAPA Task to a system task.
 *     
 * @param[in]  Ent entity, the entity ID of this task.     
 * @param[in]  Inst inst, the instance ID of this task.
 * @param[in]  Region region, the region ID registered for memory 
 *              usage of this task.
 * @param[in]  Reason reason.
 * @return ROK     - success
 *         RFAILED - failure
 ***************************************************************************/
S16 duActvInit(Ent entity, Inst inst, Region region, Reason reason)
{
   duCb.init.procId  = SFndProcId();
   duCb.init.ent     = entity;
   duCb.init.inst    = inst;
   duCb.init.region  = region;
   duCb.init.reason  = reason;
   duCb.init.cfgDone = FALSE;
   duCb.init.pool    = DU_POOL;
   duCb.init.acnt    = FALSE;
   duCb.init.trc     = FALSE;
   duCb.init.usta    = TRUE;
   duCb.mem.region   = DFLT_REGION;
   duCb.mem.pool     = DU_POOL;

   duCb.f1Status     = FALSE;

   if(ROK != cmHashListInit(&(duCb.cellLst), 
            (U16) DU_MAX_CELLS,
            (U16) 0,
            (Bool) FALSE, 
            (U16) CM_HASH_KEYTYPE_CONID,
            0,   
            0))
   {
      DU_LOG("\nDU_APP : cellLst Initialization Failed");
   }

   if(ROK != cmHashListInit(&(duCb.actvCellLst), 
            (U16) DU_MAX_CELLS,
            (U16) 0,
            (Bool) FALSE, 
            (U16) CM_HASH_KEYTYPE_CONID,
            0,   
            0))
   {
      DU_LOG("\nDU_APP : ActvCellLst Initialization Failed");
   }

   SSetProcId(DU_PROC);

   return ROK;

}

/**************************************************************************
 * @brief Task Activation callback function. 
 *
 * @details
 *
 *      Function : duActvTsk 
 * 
 *      Functionality:
 *           Primitives invoked by DU APP's users/providers through
 *           a loosely coupled interface arrive here by means of 
 *           SSI's message handling. This API is registered with
 *           SSI during the Task Registration of DU APP.
 *     
 * @param[in]  Pst     *pst, Post structure of the primitive.     
 * @param[in]  Buffer *mBuf, Packed primitive parameters in the
 *  buffer.
 * @return ROK     - success
 *         RFAILED - failure
 *
 ***************************************************************************/
S16 duActvTsk(Pst *pst, Buffer *mBuf)
{
   S16 ret = ROK;

   switch(pst->srcEnt)
   {
      case ENTDUAPP:
         {
            switch(pst->event)
            {
               case EVTCFG:
                  {
                     DU_LOG("\n****** Received initial configs at DU APP ******\n");
                     duProcCfgComplete();
                     SPutMsg(mBuf);
                     break;
                  }
               default:
                  {
                     DU_LOG("\nDU_APP : Invalid event received at duActvTsk from ENTDUAPP");
                     SPutMsg(mBuf);
                     ret = RFAILED;
                  }
            }

            break;
         }
      case ENTKW:
         {
            switch(pst->event)
            {
               case LKW_EVT_CFG_CFM:
                  {
                     ret = cmUnpkLkwCfgCfm(duHdlRlcCfgComplete, pst, mBuf);
                     break;
                  }
               case LKW_EVT_CNTRL_CFM:
                  {
                     ret = cmUnpkLkwCntrlCfm(duHdlRlcCntrlCfgComplete, pst, mBuf);
                     break;
                  }
               case LKW_EVT_STA_IND:
                  {
                     break;
                  }
               default:
                  {
                     DU_LOG("\nDU_APP : Invalid event %d received at duActvTsk from ENTKW", \
                           pst->event);
                     SPutMsg(mBuf);
                     ret = RFAILED;
                  }
            }
            break;
         }
      case ENTRG:
         {
            switch(pst->event)
            {
               //Config complete
               case EVTCFG:
                  {
                     SPutMsg(mBuf);
                     break;
                  }
               case EVTLRGCFGCFM:
                  {
                     ret = cmUnpkLrgCfgCfm(duHdlMacCfgComplete, pst, mBuf);
                     break;
                  }
               case EVTLRGCNTRLCFM:
                  {
                     break;
                  }
               case EVTMACSCHGENCFGCFM:
                  {
                     ret = cmUnpkLrgSchCfgCfm(duHdlSchCfgComplete, pst, mBuf);
                     break;
                  }

               default:
                  {
                     DU_LOG("\nDU_APP : Invalid event received at duActvTsk from ENTRG");
                     SPutMsg(mBuf);
                     ret = RFAILED;
                  }
            }

            break;
         }
      case ENTSCTP:
         {
            switch(pst->event)
            {
               case EVTSCTPDATA:
               {
                  F1APMsgHdlr(mBuf);
                  break;
               }
               case EVTSCTPNTFY:
               {
                  ret = cmUnpkSctpNtfy(duSctpNtfyHdl, pst, mBuf);
                  break;
               }
               default:
               {
                  DU_LOG("\nDU_APP : Invalid event received at duActvTsk from ENTSCTP");
                  ret = RFAILED;
               }

            }
            SPutMsg(mBuf);
            break;
         }
      case ENTEGTP:
         {
            switch(pst->event)
            {
               case EVTCFGCFM:
               {
                  cmUnpkEgtpCfgCfm(duHdlEgtpCfgComplete, mBuf);
                  break;
               }
               case EVTSRVOPENCFM:
               {
                  cmUnpkEgtpSrvOpenCfm(duHdlEgtpSrvOpenComplete, mBuf);
                  break;
               }
               case EVTTNLMGMTCFM:
               {
                  cmUnpkEgtpTnlMgmtCfm(duHdlEgtpTnlMgmtCfm, mBuf);
                  break;
               }
               default:
               {
                  DU_LOG("\nDU_APP : Invalid event[%d] received at duActvTsk from ENTEGTP", pst->event);
                  ret = RFAILED;
               }
            }
            SPutMsg(mBuf);
            break;
         }
      default:
         {
            DU_LOG("\nDU_APP : DU APP can not process message from Entity %d", pst->srcEnt);
            SPutMsg(mBuf);
            ret = RFAILED;
         }

   }
   SExitTsk();
   return ret;
}

/**********************************************************************
         End of file
**********************************************************************/
