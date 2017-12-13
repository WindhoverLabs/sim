/************************************************************************
** Includes
*************************************************************************/
#include <string.h>

#include "cfe.h"

#include "sim_app.h"
#include "sim_msg.h"
#include "sim_version.h"
#include <fcntl.h>
#include <errno.h>
#include <arpa/inet.h>
#include <strings.h>
#include <unistd.h>
#include "mavlink.h"
#include "simlib.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Instantiate the application object.                             */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
SIM oSIM;



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Default constructor.                                            */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
SIM::SIM()
{

}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Destructor constructor.                                         */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
SIM::~SIM()
{

}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Initialize event tables.                                        */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
int32 SIM::InitEvent()
{
    int32  iStatus=CFE_SUCCESS;

    /* Register the table with CFE */
    iStatus = CFE_EVS_Register(0, 0, CFE_EVS_BINARY_FILTER);
    if (iStatus != CFE_SUCCESS)
    {
        (void) CFE_ES_WriteToSysLog("SIM - Failed to register with EVS (0x%08lX)\n", iStatus);
    }

    return iStatus;
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Initialize Message Pipes                                        */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
int32 SIM::InitPipe()
{
    int32  iStatus=CFE_SUCCESS;

    /* Init schedule pipe and subscribe to wakeup messages */
    iStatus = CFE_SB_CreatePipe(&SchPipeId,
    		SIM_SCH_PIPE_DEPTH,
			SIM_SCH_PIPE_NAME);
    if (iStatus == CFE_SUCCESS)
    {
        iStatus = CFE_SB_SubscribeEx(SIM_WAKEUP_MID, SchPipeId, CFE_SB_Default_Qos, SIM_WAKEUP_MID_MAX_MSG_COUNT);
        if (iStatus != CFE_SUCCESS)
        {
            (void) CFE_EVS_SendEvent(SIM_SUBSCRIBE_ERR_EID, CFE_EVS_ERROR,
            		"Sch Pipe failed to subscribe to SIM_WAKEUP_MID. (0x%08lX)",
                    iStatus);
            goto SIM_InitPipe_Exit_Tag;
        }

        iStatus = CFE_SB_SubscribeEx(SIM_SEND_HK_MID, SchPipeId, CFE_SB_Default_Qos, SIM_SEND_HK_MID_MAX_MSG_COUNT);
        if (iStatus != CFE_SUCCESS)
        {
            (void) CFE_EVS_SendEvent(SIM_SUBSCRIBE_ERR_EID, CFE_EVS_ERROR,
					 "CMD Pipe failed to subscribe to SIM_SEND_HK_MID. (0x%08X)",
					 (unsigned int)iStatus);
            goto SIM_InitPipe_Exit_Tag;
        }
    }
    else
    {
        (void) CFE_EVS_SendEvent(SIM_PIPE_INIT_ERR_EID, CFE_EVS_ERROR,
			 "Failed to create SCH pipe (0x%08lX)",
			 iStatus);
        goto SIM_InitPipe_Exit_Tag;
    }

    /* Init command pipe and subscribe to command messages */
    iStatus = CFE_SB_CreatePipe(&CmdPipeId,
    		SIM_CMD_PIPE_DEPTH,
			SIM_CMD_PIPE_NAME);
    if (iStatus == CFE_SUCCESS)
    {
        /* Subscribe to command messages */
        iStatus = CFE_SB_Subscribe(SIM_CMD_MID, CmdPipeId);

        if (iStatus != CFE_SUCCESS)
        {
            (void) CFE_EVS_SendEvent(SIM_SUBSCRIBE_ERR_EID, CFE_EVS_ERROR,
				 "CMD Pipe failed to subscribe to SIM_CMD_MID. (0x%08lX)",
				 iStatus);
            goto SIM_InitPipe_Exit_Tag;
        }
    }
    else
    {
        (void) CFE_EVS_SendEvent(SIM_PIPE_INIT_ERR_EID, CFE_EVS_ERROR,
			 "Failed to create CMD pipe (0x%08lX)",
			 iStatus);
        goto SIM_InitPipe_Exit_Tag;
    }

SIM_InitPipe_Exit_Tag:
    return iStatus;
}
    

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Initialize Global Variables                                     */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void SIM::InitData()
{
    /* Init housekeeping message. */
    CFE_SB_InitMsg(&HkTlm,
    		SIM_HK_TLM_MID, sizeof(HkTlm), TRUE);
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* SIM initialization                                              */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
int32 SIM::InitApp()
{
    int32  iStatus   = CFE_SUCCESS;
    int8   hasEvents = 0;

    iStatus = InitEvent();
    if (iStatus != CFE_SUCCESS)
    {
        (void) CFE_ES_WriteToSysLog("SIM - Failed to init events (0x%08lX)\n", iStatus);
        goto SIM_InitApp_Exit_Tag;
    }
    else
    {
        hasEvents = 1;
    }

    iStatus = InitPipe();
    if (iStatus != CFE_SUCCESS)
    {
        goto SIM_InitApp_Exit_Tag;
    }

    InitData();

    iStatus = InitConfigTbl();
    if (iStatus != CFE_SUCCESS)
    {
        goto SIM_InitApp_Exit_Tag;
    }

    iStatus = ListenerInit();
    if (iStatus != CFE_SUCCESS)
    {
        goto SIM_InitApp_Exit_Tag;
    }

SIM_InitApp_Exit_Tag:
    if (iStatus == CFE_SUCCESS)
    {
        (void) CFE_EVS_SendEvent(SIM_INIT_INF_EID, CFE_EVS_INFORMATION,
                                 "Initialized.  Version %d.%d.%d",
								 SIM_MAJOR_VERSION,
								 SIM_MINOR_VERSION,
								 SIM_REVISION);
    }
    else
    {
        if (hasEvents == 1)
        {
            (void) CFE_ES_WriteToSysLog("SIM - Application failed to initialize\n");
        }
    }

    return iStatus;
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Receive and Process Messages                                    */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

int32 SIM::RcvSchPipeMsg(int32 iBlocking)
{
    int32           iStatus=CFE_SUCCESS;
    CFE_SB_Msg_t*   MsgPtr=NULL;
    CFE_SB_MsgId_t  MsgId;

    /* Stop Performance Log entry */
    CFE_ES_PerfLogExit(SIM_MAIN_TASK_PERF_ID);

    /* Wait for WakeUp messages from scheduler */
    iStatus = CFE_SB_RcvMsg(&MsgPtr, SchPipeId, iBlocking);

    /* Start Performance Log entry */
    CFE_ES_PerfLogEntry(SIM_MAIN_TASK_PERF_ID);

    if (iStatus == CFE_SUCCESS)
    {
        MsgId = CFE_SB_GetMsgId(MsgPtr);
        switch (MsgId)
        {
            case SIM_WAKEUP_MID:
                /* TODO:  Do something here. */
                break;

            case SIM_SEND_HK_MID:
                ProcessCmdPipe();
                ReportHousekeeping();
                break;

            default:
                (void) CFE_EVS_SendEvent(SIM_MSGID_ERR_EID, CFE_EVS_ERROR,
                     "Recvd invalid SCH msgId (0x%04X)", MsgId);
        }
    }
    else if (iStatus == CFE_SB_NO_MESSAGE)
    {
        /* TODO: If there's no incoming message, you can do something here, or 
         * nothing.  Note, this section is dead code only if the iBlocking arg
         * is CFE_SB_PEND_FOREVER. */
        iStatus = CFE_SUCCESS;
    }
    else if (iStatus == CFE_SB_TIME_OUT)
    {
        /* TODO: If there's no incoming message within a specified time (via the
         * iBlocking arg, you can do something here, or nothing.  
         * Note, this section is dead code only if the iBlocking arg
         * is CFE_SB_PEND_FOREVER. */
        iStatus = CFE_SUCCESS;
    }
    else
    {
        (void) CFE_EVS_SendEvent(SIM_RCVMSG_ERR_EID, CFE_EVS_ERROR,
			  "SCH pipe read error (0x%08lX).", iStatus);
    }

    return iStatus;
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Process Incoming Commands                                       */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

void SIM::ProcessCmdPipe()
{
    int32 iStatus = CFE_SUCCESS;
    CFE_SB_Msg_t*   CmdMsgPtr=NULL;
    CFE_SB_MsgId_t  CmdMsgId;

    /* Process command messages until the pipe is empty */
    while (1)
    {
        iStatus = CFE_SB_RcvMsg(&CmdMsgPtr, CmdPipeId, CFE_SB_POLL);
        if(iStatus == CFE_SUCCESS)
        {
            CmdMsgId = CFE_SB_GetMsgId(CmdMsgPtr);
            switch (CmdMsgId)
            {
                case SIM_CMD_MID:
                    ProcessAppCmds(CmdMsgPtr);
                    break;

                default:
                    /* Bump the command error counter for an unknown command.
                     * (This should only occur if it was subscribed to with this
                     *  pipe, but not handled in this switch-case.) */
                    HkTlm.usCmdErrCnt++;
                    (void) CFE_EVS_SendEvent(SIM_MSGID_ERR_EID, CFE_EVS_ERROR,
                                      "Recvd invalid CMD msgId (0x%04X)", (unsigned short)CmdMsgId);
                    break;
            }
        }
        else if (iStatus == CFE_SB_NO_MESSAGE)
        {
            break;
        }
        else
        {
            (void) CFE_EVS_SendEvent(SIM_RCVMSG_ERR_EID, CFE_EVS_ERROR,
                  "CMD pipe read error (0x%08lX)", iStatus);
            break;
        }
    }
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Process SIM Commands                                            */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

void SIM::ProcessAppCmds(CFE_SB_Msg_t* MsgPtr)
{
    uint32  uiCmdCode=0;

    if (MsgPtr != NULL)
    {
        uiCmdCode = CFE_SB_GetCmdCode(MsgPtr);
        switch (uiCmdCode)
        {
            case SIM_NOOP_CC:
                HkTlm.usCmdCnt++;
                (void) CFE_EVS_SendEvent(SIM_CMD_NOOP_EID, CFE_EVS_INFORMATION,
					"Recvd NOOP. Version %d.%d.%d.%d",
					SIM_MAJOR_VERSION,
					SIM_MINOR_VERSION,
					SIM_REVISION,
					SIM_MISSION_REV);
                break;

            case SIM_RESET_CC:
                HkTlm.usCmdCnt = 0;
                HkTlm.usCmdErrCnt = 0;
                break;

            default:
                HkTlm.usCmdErrCnt++;
                (void) CFE_EVS_SendEvent(SIM_CC_ERR_EID, CFE_EVS_ERROR,
                                  "Recvd invalid command code (%u)", (unsigned int)uiCmdCode);
                break;
        }
    }
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Send SIM Housekeeping                                           */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

void SIM::ReportHousekeeping()
{
    CFE_SB_TimeStampMsg((CFE_SB_Msg_t*)&HkTlm);
    CFE_SB_SendMsg((CFE_SB_Msg_t*)&HkTlm);
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Publish Output Data                                             */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Verify Command Length                                           */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
boolean SIM::VerifyCmdLength(CFE_SB_Msg_t* MsgPtr,
                           uint16 usExpectedLen)
{
    boolean bResult  = TRUE;
    uint16  usMsgLen = 0;

    if (MsgPtr != NULL)
    {
        usMsgLen = CFE_SB_GetTotalMsgLength(MsgPtr);

        if (usExpectedLen != usMsgLen)
        {
            bResult = FALSE;
            CFE_SB_MsgId_t MsgId = CFE_SB_GetMsgId(MsgPtr);
            uint16 usCmdCode = CFE_SB_GetCmdCode(MsgPtr);

            (void) CFE_EVS_SendEvent(SIM_MSGLEN_ERR_EID, CFE_EVS_ERROR,
                              "Rcvd invalid msgLen: msgId=0x%08X, cmdCode=%d, "
                              "msgLen=%d, expectedLen=%d",
                              MsgId, usCmdCode, usMsgLen, usExpectedLen);
            HkTlm.usCmdErrCnt++;
        }
    }

    return bResult;
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* SIM Application C style main entry point.                       */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
extern "C" void SIM_AppMain()
{
    oSIM.AppMain();
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* SIM Application C++ style main entry point.                     */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void SIM::AppMain()
{
    /* Register the application with Executive Services */
    uiRunStatus = CFE_ES_APP_RUN;

    int32 iStatus = CFE_ES_RegisterApp();
    if (iStatus != CFE_SUCCESS)
    {
        (void) CFE_ES_WriteToSysLog("SIM - Failed to register the app (0x%08lX)\n", iStatus);
    }

    /* Start Performance Log entry */
    CFE_ES_PerfLogEntry(SIM_MAIN_TASK_PERF_ID);

    /* Perform application initializations */
    if (iStatus == CFE_SUCCESS)
    {
        iStatus = InitApp();
    }

    if (iStatus == CFE_SUCCESS)
    {
        /* Do not perform performance monitoring on startup sync */
        CFE_ES_PerfLogExit(SIM_MAIN_TASK_PERF_ID);
        CFE_ES_WaitForStartupSync(SIM_STARTUP_TIMEOUT_MSEC);
        CFE_ES_PerfLogEntry(SIM_MAIN_TASK_PERF_ID);
    }
    else
    {
        uiRunStatus = CFE_ES_APP_ERROR;
    }

    /* Application main loop */
    while (CFE_ES_RunLoop(&uiRunStatus) == TRUE)
    {
        RcvSchPipeMsg(SIM_SCH_PIPE_PEND_TIME);

        iStatus = AcquireConfigPointers();
        if(iStatus != CFE_SUCCESS)
        {
            /* We apparently tried to load a new table but failed.  Terminate the application. */
            uiRunStatus = CFE_ES_APP_ERROR;
        }
    }

    /* Stop Performance Log entry */
    CFE_ES_PerfLogExit(SIM_MAIN_TASK_PERF_ID);

    /* Exit the application */
    CFE_ES_ExitApp(uiRunStatus);
}


int32 SIM::ListenerInit()
{
	int32 Status = CFE_SUCCESS;
	int32 TaskID = 0;
    int   reuseaddr = 1;
	struct sockaddr_in address;

    if((Socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0)
    {
    	//TODO:  Add event
    	//OS_printf("OSAL:  Failed to create sim socket.  errno: %i\n", errno);
    	Status = -1;
    	goto end_of_function;
    }

    setsockopt(Socket, SOL_SOCKET, SO_REUSEADDR, &reuseaddr, sizeof(reuseaddr));

    bzero((char *) &address, sizeof(address));
    address.sin_family      = AF_INET;
    address.sin_addr.s_addr = htonl (INADDR_ANY);
    address.sin_port        = htons(SIM_PORT);

	if ( (bind(Socket, (struct sockaddr *) &address, sizeof(address)) < 0) )
	{
    	//TODO:  Add event
    	//OS_printf("OSAL:  Failed to bind sim socket.  errno: %i\n", errno);
		Status = -1;
		goto end_of_function;
	}

    Status = OS_MutSemCreate(&MutexID, SIM_MUTEX_NAME, 0);
	if (Status != CFE_SUCCESS)
	{
		goto end_of_function;
	}

	SIMLIB_SetSocket(Socket);

	ChildContinueFlag = true;

	Status = CFE_ES_CreateChildTask(&ListenerTaskID,
				SIM_LISTENER_TASK_NAME,
				SIM::ListenerTask_c,
				NULL,
				SIM_LISTENER_TASK_STACK_SIZE,
				SIM_LISTENER_TASK_PRIORITY,
				OS_FP_ENABLED);

end_of_function:
    return Status;

}


bool  SIM::ChildContinueExec(void)
{
	bool result;

    OS_MutSemTake(MutexID);
    result = ChildContinueFlag;
    OS_MutSemGive(MutexID);

	return result;
}


void SIM::StopChild(void)
{
    OS_MutSemTake(MutexID);
    ChildContinueFlag = false;
    OS_MutSemGive(MutexID);
}


void SIM::ListenerTask_c(void)
{
	oSIM.ListenerTask();
}

void SIM::ListenerTask(void)
{
	char buffer[SIM_MAX_MESSAGE_SIZE] = {};
	uint32 size = 0;

	CFE_ES_RegisterChildTask();

	while(ChildContinueExec())
	{
		size = recv(Socket,
						   (char *)buffer,
						   (size_t)&size, 0);
		if(size <= 0)
		{
	    	//TODO:  Add event
	    	//OS_printf("OSAL:  Failed to receive message from sim\n");
			OS_TaskDelay(1000);
		}
		else
		{
			mavlink_message_t msg;
			mavlink_status_t status;
			int32 i = 0;
			char temp;

			for (i = 0; i < size; ++i)
			{
				temp = buffer[i];
				if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status))
				{
					switch(msg.msgid)
					{
						case MAVLINK_MSG_ID_HIL_RC_INPUTS_RAW:
						{
							mavlink_hil_rc_inputs_raw_t 		decodedMsg;
							mavlink_msg_hil_rc_inputs_raw_decode(&msg, &decodedMsg);
							break;
						}

						case MAVLINK_MSG_ID_HIL_SENSOR:
						{
							mavlink_hil_sensor_t 				decodedMsg;
							mavlink_msg_hil_sensor_decode(&msg, &decodedMsg);

							if(decodedMsg.fields_updated & 0x00000007)
							{
								SIMLIB_SetAccel(decodedMsg.xacc, decodedMsg.yacc, decodedMsg.zacc);
							}

							if(decodedMsg.fields_updated & 0x00000038)
							{
								SIMLIB_SetGyro(decodedMsg.xgyro, decodedMsg.ygyro, decodedMsg.zgyro);
							}

							if(decodedMsg.fields_updated & 0x000001a0)
							{
								SIMLIB_SetMag(decodedMsg.xmag, decodedMsg.ymag, decodedMsg.zmag);
							}

							if(decodedMsg.fields_updated & 0x00000600)
							{
								SIMLIB_SetPressure(decodedMsg.abs_pressure, decodedMsg.diff_pressure);
							}

							if(decodedMsg.fields_updated & 0x00000800)
							{
								SIMLIB_SetPressureAltitude(decodedMsg.pressure_alt);
							}

							if(decodedMsg.fields_updated & 0x00001000)
							{
								SIMLIB_SetTemp(decodedMsg.temperature);
							}

							break;
						}

						case MAVLINK_MSG_ID_HIL_GPS:
						{
							mavlink_hil_gps_t 					decodedMsg;
							mavlink_msg_hil_gps_decode(&msg, &decodedMsg);
							break;
						}

						case MAVLINK_MSG_ID_HIL_OPTICAL_FLOW:
						{
							mavlink_hil_optical_flow_t 			decodedMsg;
							mavlink_msg_hil_optical_flow_decode(&msg, &decodedMsg);
							break;
						}

						case MAVLINK_MSG_ID_HIL_STATE_QUATERNION:
						{
							mavlink_hil_state_quaternion_t 		decodedMsg;
							mavlink_msg_hil_state_quaternion_decode(&msg, &decodedMsg);
							break;
						}

						case MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE:
						{
							mavlink_vision_position_estimate_t 	decodedMsg;
							mavlink_msg_vision_position_estimate_decode(&msg, &decodedMsg);
							break;
						}

						default:
							printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", msg.sysid, msg.compid, msg.len, msg.msgid);
							break;
					}
				}
			}
		}
	}

	CFE_ES_ExitChildTask();
}


/************************/
/*  End of File Comment */
/************************/
