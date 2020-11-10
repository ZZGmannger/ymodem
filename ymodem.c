/**
  ******************************************************************************
  * File Name          : ymodem.c
  * Description        : This file provides code for the ymodem receiver.
  *             
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright by zzg 2020
  ******************************************************************************
  */
/*---------------------------------------------------------------------------------
#include
----------------------------------------------------------------------------------*/
#include "ymodem.h"
/*---------------------------------------------------------------------------------
#define
----------------------------------------------------------------------------------*/
#define SOH ((uint8_t)0x01) /* start of 128-byte data packet */
#define STX ((uint8_t)0x02) /* start of 1024-byte data packet */
#define EOT ((uint8_t)0x04) /* end of transmission */
#define ACK ((uint8_t)0x06) /* acknowledge */
#define NAK ((uint8_t)0x15) /* negative acknowledge */
#define CA ((uint8_t)0x18)  /* two of these in succession aborts */
#define C_C ((uint8_t)0x43)
#define YMD ((uint8_t)0x0D)
/*---------------------------------------------------------------------------------
----------------------------------------------------------------------------------*/
static ym_session_t active_session;

/*---------------------------------------------------------------------------------
#enum
----------------------------------------------------------------------------------*/
typedef enum
{
    SE_STATUS_IDLE,
    SE_STATUS_READY,
    SE_STATUS_RECIEVING_START,
    SE_STATUS_RECIEVING_DATA,
    SE_STATUS_RECIEVING_END
} SESSION_STATUS_t;

typedef enum
{
    EVT_TIMEOUT,
    EVT_RX_ERR,
    EVT_RX_SEQ_STEP_ERR,
    EVT_RX_SEQ_SAME_ERR,
    EVT_RX_DONE
} SESSION_EVT_t;

/*----------------------------------------------------------------------------------------
ymodem session opt
----------------------------------------------------------------------------------------*/
static void ym_session_timeout_evt_start(ym_session_t *session, uint32_t time);
static void ym_session_timeout_evt_stop(ym_session_t *session);
static int ym_session_post_evt(ym_session_t *session, uint8_t evt);
/**=====================================================================================
  * @brief  ymodem session setup and register
  * @param  session: point to a static session
  * @param  scb: session receive callback (only file)
  * @param  tx: lowlayer tx function (maybe uart_send_byte)
  * @retval NULL
  =====================================================================================*/
void ym_session_init(ym_session_t *session, session_rx_callback scb, session_tx tx)
{
    if (session == NULL || tx == NULL)
    {
        return;
    }

    memset((uint8_t *)session, 0, sizeof(ym_session_t));

    session->rx_cb = scb;
    session->tx = tx;
    session->state = SE_STATUS_IDLE;
}

/**=====================================================================================
  * @brief  ymodem session  start
  * @param  session: point to a static session
  * @retval NULL
  =====================================================================================*/
void ym_session_start(ym_session_t *session)
{
    ym_session_t *tail_session = NULL;
    if (session == NULL)
    {
        return;
    }

    tail_session = &active_session;
    while (tail_session->next != NULL)
    {
        tail_session = tail_session->next;
    }
    tail_session->next = session;
    session->next = NULL;

    session->state = SE_STATUS_READY;
    ym_session_timeout_evt_start(session, 500);
}

/**=====================================================================================
  * @brief  ymodem session stop
  * @param  session: point to a static session
  * @retval NULL
  =====================================================================================*/
void ym_session_stop(ym_session_t *session)
{
    ym_session_t *found_session = NULL;
    ym_session_t *prev_session = NULL;

    memset((uint8_t *)&session->file, 0, sizeof(session->file));
    memset(session->buf, 0, sizeof(session->buf));

    session->buf_len = 0;
    session->state = SE_STATUS_IDLE;
    session->seq = 0;
    session->last_seq = 0;

    found_session = &active_session;
    while (found_session->next != NULL)
    {
        prev_session = found_session;
        found_session = found_session->next;
        if (found_session == session)
        {
            prev_session->next = found_session->next;
            found_session->next = NULL;
            break;
        }
    }
}
/**=====================================================================================
  * @brief  ym_session_set_state
  * @param  session: point to a static session
  * @param  state:  SESSION_STATUS_t
  * @retval NULL
  =====================================================================================*/
static void ym_session_set_state(ym_session_t *session, SESSION_STATUS_t state)
{
    uint16_t timeout = 6000;
    switch (state)
    {
        case SE_STATUS_IDLE:
        {
            session->tx(CA);
            session->tx(CA);
            ym_session_stop(session);
//            ym_session_start(session);
            break;
        }
        case SE_STATUS_READY:
        {
            timeout = 500;
            break;
        }
        case SE_STATUS_RECIEVING_START:
        {
            break;
        }

        case SE_STATUS_RECIEVING_DATA:
        {
            session->tx(ACK);
            session->tx(C_C);
            break;
        }
        case SE_STATUS_RECIEVING_END:
        {
            session->tx(NAK);
            break;
        }
    }
    ym_session_timeout_evt_start(session, timeout);
    session->state = state;
}

/**=====================================================================================
  * @brief  ym_session_fsm
  * @param  session: point to a static session
  * @param  evt:  SESSION_EVT_t
  * @retval NULL
  =====================================================================================*/
static void ym_session_fsm(ym_session_t *session, SESSION_EVT_t evt)
{
    switch (session->state)
    {
        case SE_STATUS_IDLE:
        {
            ym_session_set_state(session, SE_STATUS_READY);
            break;
        }
        case SE_STATUS_READY:
        {
            session->tx(C_C);
            ym_session_timeout_evt_start(session, 500);
            break;
        }
        case SE_STATUS_RECIEVING_START:
        {
            if (evt == EVT_RX_DONE)
            {
                uint8_t file_name_len = strlen((char *)(session->buf + 3));

                strncpy((char *)session->file.name, (char *)(session->buf + 3),
                        ((file_name_len > FILE_NAME_MAX) ? FILE_NAME_MAX : file_name_len));

                session->file.size = atoi((char *)(session->buf + 3 + file_name_len + 1));
                session->file.recvd_size = 0;
                YM_LOG("file: %s ,byte : %d", session->file.name, session->file.size);

                ym_session_set_state(session, SE_STATUS_RECIEVING_DATA);
            }
            else if (evt == EVT_RX_SEQ_STEP_ERR || evt == EVT_TIMEOUT)
            {
                /*seq err or timeout quit the session*/
                ym_session_set_state(session, SE_STATUS_IDLE);
            }
            else
            {
                session->tx(NAK);
            }
            break;
        }
        case SE_STATUS_RECIEVING_DATA:
        {
            if (evt == EVT_RX_DONE)
            {
                /*SOH data packets*/
                if (session->buf_len == SOH_PACKET_LEN)
                {
                    session->file.recvd_size += (SOH_PACKET_LEN - 5);
                }
                /*STX data packets*/
                else if (session->buf_len == STX_PACKET_LEN)
                {
                    session->file.recvd_size += (STX_PACKET_LEN - 5);
                }
                /*EOT*/
                else
                {
                    ym_session_set_state(session, SE_STATUS_RECIEVING_END);
                    break;
                }
                /*normal SOH and STX packets receiveing*/
                uint16_t pad_size = 0;
                if (session->file.recvd_size >= session->file.size)
                {
                    pad_size = session->file.recvd_size - session->file.size;
                    session->file.percent = 100;
                }
                else
                {
                    session->file.percent = (uint8_t)((1.0 * session->file.recvd_size) / session->file.size * 100.0);
                    YM_LOG("percent : %d% \r\n", session->file.percent);
                }
                if ((session->rx_cb != NULL) && (session->buf_len == SOH_PACKET_LEN || session->buf_len == STX_PACKET_LEN))
                {
                    session->rx_cb(session->buf + 3, session->buf_len - 5 - pad_size);
                }
                session->tx(ACK);
            }
            else if (evt == EVT_RX_SEQ_SAME_ERR)
            {
                session->tx(ACK);
            }
            else if (evt == EVT_TIMEOUT)
            {
                ym_session_set_state(session, SE_STATUS_IDLE);
            }
            else
            {
                session->tx(NAK);
            }
            ym_session_timeout_evt_start(session, 3000);
            break;
        }
        case SE_STATUS_RECIEVING_END:
        {
            if (evt == EVT_RX_DONE)
            {
                /*EOT or the last packet*/
                if (session->buf_len == SOH_PACKET_LEN || session->buf[0] == YMD)
                {
                    session->tx(ACK);
                    ym_session_set_state(session, SE_STATUS_IDLE);
                }
                else
                {
                    /*EOT*/
                    session->tx(ACK);
                    session->tx(C_C);
                }
            }
            else
            {
                session->tx(NAK);
            }
            break;
        }
    }
    session->buf_len = 0;
}

/*----------------------------------------------------------------------------------------
ymodem event opt
----------------------------------------------------------------------------------------*/
/**=====================================================================================
  * @brief  ym_session_timeout_evt_start
  * @param  session: point to a static session
  * @param  time:  timeout (ms)
  * @retval NULL
  =====================================================================================*/
static void ym_session_timeout_evt_start(ym_session_t *session, uint32_t time)
{
    if (session == NULL)
    {
        YM_LOG("no such session!");
        return;
    }
    session->timeout = time;
    session->timeout_en = 1;
}
/**=====================================================================================
  * @brief  ym_session_timeout_evt_stop
  * @param  session: point to a static session
  * @param   
  * @retval NULL
  =====================================================================================*/
static void ym_session_timeout_evt_stop(ym_session_t *session)
{
    if (session == NULL)
    {
        YM_LOG("no such session!");
        return;
    }
    session->timeout = 0;
    session->timeout_en = 0;
}
/**=====================================================================================
  * @brief  ym session post evt directly
  * @param  session: point to a static session
  * @param   
  * @retval NULL
  =====================================================================================*/
static int ym_session_post_evt(ym_session_t *session, uint8_t evt)
{
    if (session == NULL)
    {
        YM_LOG("no such session!");
        return -1;
    }
    if (session->evt.num >= SESSION_MAX_EVT)
    {
        YM_LOG("session evt oversize!");
        return -2;
    }
    session->evt.fifo[session->evt.windex] = evt;
    session->evt.windex = (session->evt.windex + 1) % SESSION_MAX_EVT;
    ++session->evt.num;
    return 0;
}

/**=====================================================================================
  * @brief  ym_session_get_evt
  * @param  session: point to a static session
  * @param   evt :SESSION_EVT_t
  * @retval NULL
  =====================================================================================*/
static int ym_session_get_evt(ym_session_t *session, uint8_t *evt)
{
    if (session == NULL)
    {
        YM_LOG("no such session!");
        return -1;
    }
    if (session->evt.num)
    {
        *evt = session->evt.fifo[session->evt.rindex];
        session->evt.rindex = (session->evt.rindex + 1) % SESSION_MAX_EVT;
        --session->evt.num;

        return 0;
    }
    // YM_LOG("no evt exsit in session!");
    return -2;
}

/**=====================================================================================
  * @brief  ym_session_evt_dispatch
  * @retval NULL
  =====================================================================================*/
void ym_session_evt_dispatch(void)
{
    ym_session_t *session = &active_session;
    uint8_t evt;

    while (session->next != NULL)
    {
        session = session->next;
        /*check timeout evt*/
        if (session->timeout_en)
        {
            if (session->timeout)
            {
                session->timeout--;
            }
            else
            {
                session->timeout_en = 0;
                ym_session_post_evt(session, EVT_TIMEOUT);
            }
        }
        /*get evt and dispatch evt*/
        if (ym_session_get_evt(session, &evt) == 0)
        {
            YM_LOG("state: %d , evt %d\r\n", session->state, evt);
            ym_session_fsm(session, evt);
        }
    }
}

/*----------------------------------------------------------------------------------------
ymodem lower layer data input opt
----------------------------------------------------------------------------------------*/

/**=====================================================================================
  * @brief  ym_crc16
  * @param  buf
  * @param  len
  * @retval crc16 value
  =====================================================================================*/
static uint16_t ym_crc16(uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0;
    uint16_t i = 0;

    while (len--)
    {
        crc = crc ^ (int)(*buf++) << 8;
        for (i = 8; i != 0; i--)
        {
            if (crc & 0x8000)
            {
                crc = crc << 1 ^ 0x1021;
            }
            else
            {
                crc = crc << 1;
            }
        }
    }
    return crc;
}

/**=====================================================================================
  * @brief  ym_session_parser_byte,used to uart data one by one (RXNB INT)
  * @param  session: point to a static session
  * @param   data:receive data
  * @retval NULL
  =====================================================================================*/
/*data one by one*/
void ym_session_parser_byte(ym_session_t *session, uint8_t data)
{
    uint16_t crc;
    if (session->buf_len < STX_PACKET_LEN)
    {
        session->buf[session->buf_len++] = data;

        switch (session->buf[0])
        {
            case SOH:
            {
                if (session->buf_len == SOH_PACKET_LEN)
                {
                    session->buf_rx_done = 1;
                }
                break;
            }
            case STX:
            {
                if (session->buf_len == STX_PACKET_LEN)
                {
                    session->buf_rx_done = 1;
                }
                break;
            }
            case EOT:
            {
                if (session->buf_len == 1)
                {
                    session->buf_rx_done = 1;
                }
                break;
            }
            case YMD:
            {
                if (session->buf_len == 1)
                {
                    session->buf_rx_done = 1;
                }
                break;
            }
            default:
            {
                ym_session_post_evt(session, EVT_RX_ERR);
                YM_LOG("unknow heard error!\r\n");
                return;
            }
        }
    }
    else
    {
        ym_session_post_evt(session, EVT_RX_ERR);
        YM_LOG("packet oversize error!");
        return;
    }

    if (session->buf_rx_done)
    {
        session->buf_rx_done = 0;
        if (session->buf[0] != EOT && session->buf[0] != YMD)
        {
            /*check seq*/
            if ((session->buf[1] + session->buf[2]) != 0xff)
            {
                YM_LOG("packet header sequence error!");
                ym_session_post_evt(session, EVT_RX_ERR);
                return;
            }

            /*check crc*/
            crc = (session->buf[session->buf_len - 2] << 8) | session->buf[session->buf_len - 1];
            if (ym_crc16(session->buf + 3, session->buf_len - 5) != crc)
            {
                YM_LOG("packet crc error!");
                ym_session_post_evt(session, EVT_RX_ERR);
                return;
            }
            /*updata session sequece*/
            session->last_seq = session->seq;
            session->seq = session->buf[1];
            if (session->seq == 0)
            {
                if (session->last_seq == 0)
                {
                    /*the start frame*/
                    ym_session_set_state(session, SE_STATUS_RECIEVING_START);
                }
                else if ((session->last_seq != 0xff) && (session->state != SE_STATUS_RECIEVING_END))
                {
                    /*if seq flip and not the end status, the last seq must be 0xff */
                    YM_LOG("packet seq error!");
                    ym_session_post_evt(session, EVT_RX_SEQ_STEP_ERR);
                    return;
                }
            }
            else if ((session->seq - session->last_seq) > 1)
            {
                /*session->seq - session->last_seq = 1*/
                YM_LOG("packet seq step error!");
                ym_session_post_evt(session, EVT_RX_SEQ_STEP_ERR);
                return;
            }
            else if (session->seq == session->last_seq)
            {
                /*ack lost*/
                YM_LOG("packet seq same error!");
                ym_session_post_evt(session, EVT_RX_SEQ_SAME_ERR);
                return;
            }
            ym_session_post_evt(session, EVT_RX_DONE);
        }
        else
        {
            ym_session_post_evt(session, EVT_RX_DONE);
        }
    }
}

/**=====================================================================================
  * @brief  ym_session_parser_byte,used to uart data one by one (IDLE INT)
  * @param  session: point to a static session
  * @param   buf : point to receive buff
  * @param   len : receive length
  * @retval NULL
  =====================================================================================*/
/*data receive by frame*/
void ym_parser_frame(ym_session_t *session, uint8_t *buf, uint16_t len)
{
    uint16_t crc;

    memcpy(session->buf, buf, len);
    session->buf_len = len;
    /*check length*/
    switch (buf[0])
    {
        case SOH:
        {
            if (len != SOH_PACKET_LEN)
            {
                ym_session_post_evt(session, EVT_RX_ERR);
                YM_LOG("SOH packet length error!");
                return;
            }
            break;
        }
        case STX:
        {
            if (len != STX_PACKET_LEN)
            {
                ym_session_post_evt(session, EVT_RX_ERR);
                YM_LOG("STX packet length error!");
                return;
            }
            break;
        }
        case EOT:
        case YMD:
        {
            if (len == 1)
            {
                ym_session_post_evt(session, EVT_RX_DONE);
            }
            else
            {
                ym_session_post_evt(session, EVT_RX_ERR);
                YM_LOG("EOT or YMD packet length error!");
            }
            return;
        }
        default:
        {
            ym_session_post_evt(session, EVT_RX_ERR);
            YM_LOG("packet unknow frame error!");
            return;
        }
    }
    /*check header sequece*/
    if ((buf[1] + buf[2]) != 0xff)
    {
        ym_session_post_evt(session, EVT_RX_ERR);
        YM_LOG("packet header sequence error!");
        return;
    }
    /*check crc*/
    crc = (buf[len - 2] << 8) | buf[len - 1];
    if (ym_crc16(buf + 3, len - 5) != crc)
    {
        ym_session_post_evt(session, EVT_RX_ERR);
        YM_LOG("packet crc error!");
        return;
    }

    /*updata session sequece*/
    session->last_seq = session->seq;
    session->seq = session->buf[1];
    if (session->seq == 0)
    {
        if (session->last_seq == 0)
        {
            /*the start frame*/
            ym_session_set_state(session, SE_STATUS_RECIEVING_START);
        }
        else if ((session->last_seq != 0xff) && (session->state != SE_STATUS_RECIEVING_END))
        {
            /*if seq flip and not the end status, the last seq must be 0xff */
            YM_LOG("packet seq error!");
            ym_session_post_evt(session, EVT_RX_SEQ_STEP_ERR);
            return;
        }
    }
    else if ((session->seq - session->last_seq) > 1)
    {
        /*session->seq - session->last_seq = 1*/
        YM_LOG("packet seq step error!");
        ym_session_post_evt(session, EVT_RX_SEQ_STEP_ERR);
        return;
    }
    else if (session->seq == session->last_seq)
    {
        /*ack lost*/
        YM_LOG("packet seq same error!");
        ym_session_post_evt(session, EVT_RX_SEQ_SAME_ERR);
        return;
    }
    ym_session_post_evt(session, EVT_RX_DONE);
}

/*------------------------------------------end---------------------------------------------------------*/
