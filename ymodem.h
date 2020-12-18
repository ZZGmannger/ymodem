#ifndef __YMODEM__H
#define __YMODEM__H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "main.h"

#define SOH_DATA_1024_LEN  (1024)
#define STX_DATA_128_LEN   (128)
#define SOH_PACKET_LEN   (1+2+128+2)
#define STX_PACKET_LEN   (1+2+1024+2)

#define YM_LOG(...)   //printf

#define SESSION_MAX_EVT 8
#define FILE_NAME_MAX  (24)

typedef enum
{
	YM_FILE_START,
	YM_FILE_RECV,
	YM_FILE_DONE
}YM_CB_EVT_t;


typedef void (*session_rx_callback)(void* buf , uint16_t len ,YM_CB_EVT_t evt);
typedef void (*session_tx)(uint8_t data);

typedef struct 
{
    uint8_t fifo[SESSION_MAX_EVT];
    uint8_t rindex;
    uint8_t windex;
    uint8_t num;
}sessiom_evt_t;

typedef struct
{
    char     name[FILE_NAME_MAX];
    uint32_t size;
    uint32_t recvd_size;
    uint8_t  percent;
}file_info_t;

struct ym_session
{ 
    struct ym_session* next;
    sessiom_evt_t evt;

    uint8_t timeout_en;
    uint32_t timeout;

    file_info_t  file;

    uint8_t  buf[STX_PACKET_LEN];
    uint16_t buf_len;
    uint8_t  buf_rx_done;
    uint8_t  seq;
    uint8_t  last_seq;

    uint8_t  state;
    session_rx_callback rx_cb; 
    session_tx tx;
};
typedef struct ym_session ym_session_t;

void ym_session_init(ym_session_t *session , session_rx_callback scb , session_tx tx);
void ym_session_start(ym_session_t *session);
void ym_session_stop(ym_session_t *session);
void ym_session_evt_dispatch(void);

void ym_session_parser_byte(ym_session_t *session , uint8_t data);
void ym_parser_frame(ym_session_t *session , uint8_t* buf ,uint16_t len);



#endif

