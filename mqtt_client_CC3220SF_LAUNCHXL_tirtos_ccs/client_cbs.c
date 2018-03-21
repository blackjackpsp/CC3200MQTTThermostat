/*
 * Copyright (c) 2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

//*****************************************************************************
//
//! \addtogroup mqtt_server
//! @{
//
//*****************************************************************************
/* Standard includes                                                          */
#include <stdlib.h>

/* Kernel (Non OS/Free-RTOS/TI-RTOS) includes                                 */
#include "pthread.h"
#include "mqueue.h"

/* Common interface includes                                                  */
#include "uart_term.h"

/* Application includes                                                       */
#include "client_cbs.h"

extern bool gResetApplication;

//*****************************************************************************
//                          LOCAL DEFINES
//*****************************************************************************
#define APP_PRINT               Report

#define OS_WAIT_FOREVER         (0xFFFFFFFF)
#define OS_NO_WAIT              (0)
#define OS_OK                   (0)

#define MQTTClientCbs_ConnackRC(data) (data & 0xff) /**< CONNACK: Return Code (LSB) */

//*****************************************************************************
//                 GLOBAL VARIABLES
//*****************************************************************************

/* Message Queue                                                              */
extern mqd_t g_PBQueue;
extern char *topic[];
struct client_info client_info_table[MAX_CONNECTION];

//*****************************************************************************
//                 Queue external function
//*****************************************************************************
extern int32_t MQTT_SendMsgToQueue(struct msgQueue *queueElement);

//****************************************************************************
//                      CLIENT CALLBACKS
//****************************************************************************


//*****************************************************************************
//
//! Callback in case of various event (for clients connection with remote
//! broker)
//!
//! \param[in]  event       - is a event occurred
//! \param[in]  metaData    - is the pointer for the message buffer
//!                           (for this event)
//! \param[in]  metaDateLen - is the length of the message buffer
//! \param[in]  data        - is the pointer to the buffer for data
//!                           (for this event)
//! \param[in]  dataLen     - is the length of the buffer data
//!
//! return none
//
//*****************************************************************************
void MqttClientCallback(int32_t event , void * metaData , uint32_t metaDateLen , void *data , uint32_t dataLen)
{
    int32_t i = 0;

    switch((MQTTClient_EventCB)event)
    {
        case MQTT_CLIENT_OPERATION_CB_EVENT:
        {
            switch (((MQTTClient_OperationMetaDataCB_t*)metaData)->messageType)
            {
                case MQTTCLIENT_OPERATION_CONNACK:
                {
                    uint16_t *ConnACK = (uint16_t*) data;
                    APP_PRINT("CONNACK:\n\r");
                    /* Check if Conn Ack return value is Success (0) or       */
                    /* Error - Negative value                                 */
                    if ( 0 == (MQTTClientCbs_ConnackRC(*ConnACK)) )
                    {
                        APP_PRINT("Connection Success\n\r");
                    }
                    else
                    {
                        APP_PRINT("Connection Error: %d\n\r", *ConnACK);
                    }
                    break;
                }

                case MQTTCLIENT_OPERATION_EVT_PUBACK:
                {
                    char *PubAck = (char *) data;
                    APP_PRINT("PubAck:\n\r");
                    APP_PRINT("%s\n\r", PubAck);
                    break;
                }

                case MQTTCLIENT_OPERATION_SUBACK:
                {
                    APP_PRINT("Sub Ack:\n\r");
                    APP_PRINT("Granted QoS Levels are:\n\r");
                    for (i = 0; i < dataLen; i++)
                    {
                        APP_PRINT("%s :QoS %d\n\r", topic[i], ((unsigned char*) data)[i]);
                    }
                    break;
                }

                case MQTTCLIENT_OPERATION_UNSUBACK:
                {
                    char *UnSub = (char *) data;
                    APP_PRINT("UnSub Ack \n\r");
                    APP_PRINT("%s\n\r", UnSub);
                    break;
                }

                default:
                    break;
            }
            break;
        }
        case MQTT_CLIENT_RECV_CB_EVENT:
        {
            MQTTClient_RecvMetaDataCB_t *recvMetaData =  (MQTTClient_RecvMetaDataCB_t *)metaData;
            uint32_t bufSizeReqd = 0;
            uint32_t topicOffset;
            uint32_t payloadOffset;

            struct publishMsgHeader msgHead;

            char *pubBuff = NULL;
            struct msgQueue queueElem;

            topicOffset = sizeof(struct publishMsgHeader);
            payloadOffset = sizeof(struct publishMsgHeader) + recvMetaData->topLen + 1;

            bufSizeReqd += sizeof(struct publishMsgHeader);
            bufSizeReqd += recvMetaData->topLen + 1;
            bufSizeReqd += dataLen + 1;
            pubBuff = (char *) malloc(bufSizeReqd);

            if (pubBuff == NULL)
            {
                APP_PRINT("malloc failed: recv_cb\n\r");
                return;
            }

            msgHead.topicLen = recvMetaData->topLen;
            msgHead.payLen = dataLen;
            msgHead.retain = recvMetaData->retain;
            msgHead.dup = recvMetaData->dup;
            msgHead.qos = recvMetaData->qos;
            memcpy((void*) pubBuff, &msgHead, sizeof(struct publishMsgHeader));

            /* copying the topic name into the buffer                         */
            memcpy((void*) (pubBuff + topicOffset), (const void*)recvMetaData->topic, recvMetaData->topLen);
            memset((void*) (pubBuff + topicOffset + recvMetaData->topLen), '\0', 1);

            /* copying the payload into the buffer                            */
            memcpy((void*) (pubBuff + payloadOffset), (const void*) data, dataLen);
            memset((void*) (pubBuff + payloadOffset + dataLen), '\0', 1);

            APP_PRINT("\n\rMsg Recvd. by client\n\r");
            APP_PRINT("TOPIC: %s\n\r", pubBuff + topicOffset);
            APP_PRINT("PAYLOAD: %s\n\r", pubBuff + payloadOffset);
            APP_PRINT("QOS: %d\n\r", recvMetaData->qos);

            if (recvMetaData->retain)
            {
                APP_PRINT("Retained\n\r");
            }

            if (recvMetaData->dup)
            {
                APP_PRINT("Duplicate\n\r");
            }

            /* filling the queue element details                              */
            queueElem.event  = MSG_RECV_BY_CLIENT;
            queueElem.msgPtr = pubBuff;
            queueElem.topLen = recvMetaData->topLen;

            /* signal to the main task                                        */
            if (MQTT_SendMsgToQueue(&queueElem))
            {
                UART_PRINT("\n\n\rQueue is full\n\n\r");
            }
            break;
        }
        case MQTT_CLIENT_DISCONNECT_CB_EVENT:
        {
            gResetApplication = true;
            APP_PRINT("BRIDGE DISCONNECTION\n\r");
            break;
        }
    }
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
